#include "camera_web.h"
#include "wifi_credentials.local.h"
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_psram.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

// 本模块日志标签；协议数据使用 HTTP 输出，不与串口日志混合。
static constexpr char TAG[] = "camera_web";

CameraWeb& CameraWeb::instance()
{
    // 实例一直存活到固件重启，网络回调不会引用已销毁的栈对象。
    static CameraWeb app;
    return app;
}

esp_err_t CameraWeb::start()
{
    ESP_LOGI(TAG, "start() >>");
    esp_err_t result = initCamera();
    if (result == ESP_OK)
    {
        result = initWifi();
    }
    if (result == ESP_OK)
    {
        result = startServers();
    }
    ESP_LOGI(TAG, "start() << result=%s", esp_err_to_name(result));
    return result;
}

esp_err_t CameraWeb::initCamera()
{
    ESP_LOGI(TAG, "initCamera() >> psram_bytes=%u", unsigned(esp_psram_get_size()));
    if (!esp_psram_is_initialized() || esp_psram_get_size() < 2 * 1024 * 1024)
    {
        // 高分辨率 JPEG 需要外部帧缓冲；硬件不符时明确失败，不隐式降级。
        ESP_LOGE(TAG, "initCamera() << result=PSRAM_NOT_READY");
        return ESP_ERR_NO_MEM;
    }

    camera_config_t config = {};
    // AI-Thinker 板型固定引脚，GPIO0 用于相机时钟，不再用于普通输出。
    config.pin_pwdn = 32;
    config.pin_reset = -1;
    config.pin_xclk = 0;
    config.pin_sccb_sda = 26;
    config.pin_sccb_scl = 27;
    config.pin_d0 = 5;
    config.pin_d1 = 18;
    config.pin_d2 = 19;
    config.pin_d3 = 21;
    config.pin_d4 = 36;
    config.pin_d5 = 39;
    config.pin_d6 = 34;
    config.pin_d7 = 35;
    config.pin_vsync = 25;
    config.pin_href = 23;
    config.pin_pclk = 22;
    config.xclk_freq_hz = 20000000;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.pixel_format = PIXFORMAT_JPEG;
    // 按最大支持分辨率分配缓冲，运行时切换分辨率不会超出初始容量。
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
    esp_err_t result = esp_camera_init(&config);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "initCamera() << result=%s", esp_err_to_name(result));
        return result;
    }
    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor == nullptr || sensor->id.PID != OV2640_PID)
    {
        // 当前参数范围按 OV2640 验证，其他传感器不能套用该组网页选项。
        ESP_LOGE(TAG, "initCamera() << result=UNSUPPORTED_SENSOR");
        esp_camera_deinit();
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (sensor->set_framesize(sensor, FRAMESIZE_VGA) != 0 ||
        sensor->set_quality(sensor, 12) != 0)
    {
        ESP_LOGE(TAG, "initCamera() << result=INITIAL_SETTINGS_FAILED");
        esp_camera_deinit();
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "initCamera() << result=OK sensor=OV2640 resolution=640x480");
    return ESP_OK;
}

esp_err_t CameraWeb::initWifi()
{
    ESP_LOGI(TAG, "initWifi() >> ssid=%s", WIFI_SSID);
    static_assert(sizeof(WIFI_SSID) - 1 <= 32);
    static_assert(sizeof(WIFI_PASSWORD) - 1 <= 64);
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // 只在 NVS 无法正常初始化时重建分区，不在每次启动时清除配置。
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase failed");
        result = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(result, TAG, "nvs init failed");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop failed");
    if (esp_netif_create_default_wifi_sta() == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }
    const wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "wifi init failed");
    // 凭据仅由本地头文件提供，避免写入 Wi-Fi 配置的 NVS 副本或日志。
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage failed");
    const esp_timer_create_args_t timer = {
        .callback = reconnectWifi,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "wifi_retry",
        .skip_unhandled_events = false,
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer, &m_retryTimer), TAG, "retry timer failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, onWifiEvent, this, &m_wifiEvent), TAG, "wifi event failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, onWifiEvent, this, &m_ipEvent), TAG, "ip event failed");
    wifi_config_t config = {};
    std::memcpy(config.sta.ssid, WIFI_SSID, sizeof(WIFI_SSID) - 1);
    std::memcpy(config.sta.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD) - 1);
    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    config.sta.pmf_cfg.capable = true;
    config.sta.pmf_cfg.required = false;
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "wifi mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &config), TAG, "wifi config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");
    // 禁用 Wi-Fi 省电以减少视频传输中的间歇延迟。
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "wifi power mode failed");
    ESP_LOGI(TAG, "initWifi() << result=OK waiting_for_ip");
    return ESP_OK;
}

void CameraWeb::onWifiEvent(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    auto* app = static_cast<CameraWeb*>(arg);
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        app->m_connected.store(true);
        esp_timer_stop(app->m_retryTimer);
        const auto* event = static_cast<ip_event_got_ip_t*>(data);
        ESP_LOGI(TAG, "Wi-Fi connected; open http://" IPSTR "/", IP2STR(&event->ip_info.ip));
        return;
    }
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
    {
        reconnectWifi(app);
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        const auto* event = static_cast<wifi_event_sta_disconnected_t*>(data);
        app->m_connected.store(false);
        // 断线使旧视频循环失效；三秒后重试，持续等待路由器恢复。
        app->m_epoch.fetch_add(1);
        ESP_LOGW(TAG, "Wi-Fi disconnected reason=%u; retry in 3 seconds", event->reason);
        esp_timer_stop(app->m_retryTimer);
        esp_timer_start_once(app->m_retryTimer, 3000000);
    }
}

void CameraWeb::reconnectWifi(void* arg)
{
    auto* app = static_cast<CameraWeb*>(arg);
    if (!app->m_connected.load())
    {
        const esp_err_t result = esp_wifi_connect();
        if (result != ESP_OK)
        {
            ESP_LOGW(TAG, "connect() << result=%s", esp_err_to_name(result));
            esp_timer_start_once(app->m_retryTimer, 3000000);
        }
    }
}

esp_err_t CameraWeb::startServers()
{
    ESP_LOGI(TAG, "startServers() >>");
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 8192;
    config.max_uri_handlers = 5;
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;
    config.send_wait_timeout = 3;
    config.recv_wait_timeout = 3;
    ESP_RETURN_ON_ERROR(httpd_start(&m_control, &config), TAG, "control server failed");
    const char* paths[] = {"/", "/status", "/control", "/stop", "/capture"};
    for (const char* path : paths)
    {
        httpd_uri_t uri = {};
        uri.uri = path;
        uri.method = (std::strcmp(path, "/control") == 0 ||
                      std::strcmp(path, "/stop") == 0) ? HTTP_POST : HTTP_GET;
        uri.handler = onRequest;
        uri.user_ctx = this;
        const esp_err_t result = httpd_register_uri_handler(m_control, &uri);
        if (result != ESP_OK)
        {
            httpd_stop(m_control);
            m_control = nullptr;
            return result;
        }
    }
    // 长时间传输的视频独占另一 HTTP 任务，控制页面仍可响应。
    config.server_port = 81;
    config.ctrl_port += 1;
    config.max_uri_handlers = 1;
    config.max_open_sockets = 1;
    const esp_err_t result = httpd_start(&m_stream, &config);
    if (result != ESP_OK)
    {
        httpd_stop(m_control);
        m_control = nullptr;
        return result;
    }
    httpd_uri_t uri = {};
    uri.uri = "/stream";
    uri.method = HTTP_GET;
    uri.handler = onStream;
    uri.user_ctx = this;
    const esp_err_t registered = httpd_register_uri_handler(m_stream, &uri);
    if (registered != ESP_OK)
    {
        httpd_stop(m_stream);
        httpd_stop(m_control);
        m_stream = nullptr;
        m_control = nullptr;
        return registered;
    }
    ESP_LOGI(TAG, "startServers() << result=OK control=80 stream=81");
    return ESP_OK;
}

esp_err_t CameraWeb::onRequest(httpd_req_t* req)
{
    auto* app = static_cast<CameraWeb*>(req->user_ctx);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    if (std::strcmp(req->uri, "/") == 0)
    {
        // HTML 随固件编译嵌入，不依赖电脑上的开发服务器或外部 CDN。
        extern const char pageStart[] asm("_binary_index_html_start");
        httpd_resp_set_type(req, "text/html; charset=utf-8");
        return httpd_resp_send(req, pageStart, HTTPD_RESP_USE_STRLEN);
    }
    if (std::strcmp(req->uri, "/status") == 0)
    {
        return app->sendStatus(req);
    }
    if (std::strcmp(req->uri, "/control") == 0)
    {
        return app->setControl(req);
    }
    if (std::strcmp(req->uri, "/stop") == 0)
    {
        // 代次变化使旧循环退出；下次启动的视频使用新的代次，不受旧请求影响。
        app->m_epoch.fetch_add(1);
        return httpd_resp_sendstr(req, "OK");
    }
    if (std::strcmp(req->uri, "/capture") == 0)
    {
        camera_fb_t* frame = nullptr;
        {
            std::lock_guard<std::mutex> lock(app->m_mutex);
            frame = esp_camera_fb_get();
        }
        // 帧缓冲由驱动拥有，使用 RAII 保证发送失败时也归还。
        std::unique_ptr<camera_fb_t, decltype(&esp_camera_fb_return)>
            holder(frame, esp_camera_fb_return);
        if (frame == nullptr)
        {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Capture failed");
        }
        httpd_resp_set_type(req, "image/jpeg");
        return httpd_resp_send(req, reinterpret_cast<const char*>(frame->buf), frame->len);
    }
    return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
}

esp_err_t CameraWeb::sendStatus(httpd_req_t* req)
{
    char json[512];
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        const sensor_t* sensor = esp_camera_sensor_get();
        if (sensor == nullptr)
        {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera not ready");
        }
        const auto& state = sensor->status;
        std::snprintf(json, sizeof(json),
            "{\"size\":%d,\"quality\":%d,\"max_fps\":%d,\"actual_fps\":%.2f,"
            "\"brightness\":%d,\"contrast\":%d,\"saturation\":%d,"
            "\"hmirror\":%d,\"vflip\":%d,\"streaming\":%s,\"connected\":%s}",
            m_size, state.quality, m_maxFps, m_actualFps,
            state.brightness, state.contrast, state.saturation, state.hmirror, state.vflip,
            m_streaming ? "true" : "false", m_connected.load() ? "true" : "false");
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

esp_err_t CameraWeb::setControl(httpd_req_t* req)
{
    char body[96] = {};
    char name[24] = {};
    char text[16] = {};
    if (req->content_len == 0 || req->content_len >= sizeof(body))
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid body length");
    }
    size_t received = 0;
    // TCP 数据可能分多次到达；三秒接收超时由 HTTP 服务器统一限制。
    while (received < req->content_len)
    {
        const int count = httpd_req_recv(req, body + received, req->content_len - received);
        if (count <= 0)
        {
            return ESP_FAIL;
        }
        received += count;
    }
    if (httpd_query_key_value(body, "var", name, sizeof(name)) != ESP_OK ||
        httpd_query_key_value(body, "val", text, sizeof(text)) != ESP_OK)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected var and val");
    }
    char* end = nullptr;
    errno = 0;
    const long value = std::strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || value < -2 || value > 63)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid integer");
    }
    ESP_LOGI(TAG, "setControl() >> name=%s value=%ld", name, value);
    bool applied;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        // 参数设置与取帧互斥，避免传感器寄存器在另一个任务采集时被并发修改。
        applied = applySetting(name, static_cast<int>(value));
    }
    ESP_LOGI(TAG, "setControl() << result=%s", applied ? "OK" : "REJECTED");
    if (!applied)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unsupported setting or range");
    }
    return sendStatus(req);
}

bool CameraWeb::applySetting(const char* name, int value)
{
    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor == nullptr)
    {
        return false;
    }
    if (std::strcmp(name, "max_fps") == 0)
    {
        if (value < 1 || value > 30)
        {
            return false;
        }
        m_maxFps = value;
        return true;
    }
    if (std::strcmp(name, "size") == 0)
    {
        framesize_t size;
        // 网页编号不依赖驱动枚举的底层数值；只允许 OV2640 支持的六档分辨率。
        switch (value)
        {
            case 0: size = FRAMESIZE_QQVGA; break;
            case 1: size = FRAMESIZE_QVGA; break;
            case 2: size = FRAMESIZE_VGA; break;
            case 3: size = FRAMESIZE_SVGA; break;
            case 4: size = FRAMESIZE_XGA; break;
            case 5: size = FRAMESIZE_UXGA; break;
            default: return false;
        }
        if (sensor->set_framesize(sensor, size) != 0)
        {
            return false;
        }
        m_size = value;
        return true;
    }
    if (std::strcmp(name, "quality") == 0)
    {
        // 数值越小画质越高；最低限制 10，避免极端高码率挤满帧缓冲。
        return value >= 10 && value <= 63 && sensor->set_quality(sensor, value) == 0;
    }
    if (std::strcmp(name, "brightness") == 0)
    {
        return value >= -2 && value <= 2 && sensor->set_brightness(sensor, value) == 0;
    }
    if (std::strcmp(name, "contrast") == 0)
    {
        return value >= -2 && value <= 2 && sensor->set_contrast(sensor, value) == 0;
    }
    if (std::strcmp(name, "saturation") == 0)
    {
        return value >= -2 && value <= 2 && sensor->set_saturation(sensor, value) == 0;
    }
    if (std::strcmp(name, "hmirror") == 0)
    {
        return (value == 0 || value == 1) && sensor->set_hmirror(sensor, value) == 0;
    }
    if (std::strcmp(name, "vflip") == 0)
    {
        return (value == 0 || value == 1) && sensor->set_vflip(sensor, value) == 0;
    }
    return false;
}

void CameraWeb::finishStream()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_streaming = false;
    m_actualFps = 0;
}

esp_err_t CameraWeb::onStream(httpd_req_t* req)
{
    auto* app = static_cast<CameraWeb*>(req->user_ctx);
    const unsigned epoch = app->m_epoch.load();
    if (!app->m_connected.load())
    {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "Wi-Fi disconnected");
    }
    ESP_LOGI(TAG, "onStream() >>");
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    unsigned frames = 0;
    int64_t window = 0;
    int64_t nextFrame = esp_timer_get_time();
    esp_err_t result = ESP_OK;
    while (app->m_connected.load() && epoch == app->m_epoch.load())
    {
        // 使用单调时钟限速；向上取整到 tick，保证发送频率不超过设定上限。
        const int64_t remaining = nextFrame - esp_timer_get_time();
        if (remaining > 0)
        {
            const int64_t tickUs = 1000000 / configTICK_RATE_HZ;
            vTaskDelay(static_cast<TickType_t>((remaining + tickUs - 1) / tickUs));
        }
        if (epoch != app->m_epoch.load())
        {
            break;
        }
        const int64_t started = esp_timer_get_time();
        camera_fb_t* frame = nullptr;
        int maxFps;
        {
            std::lock_guard<std::mutex> lock(app->m_mutex);
            maxFps = app->m_maxFps;
            app->m_streaming = true;
            frame = esp_camera_fb_get();
        }
        std::unique_ptr<camera_fb_t, decltype(&esp_camera_fb_return)>
            holder(frame, esp_camera_fb_return);
        if (frame == nullptr || frame->format != PIXFORMAT_JPEG)
        {
            ESP_LOGE(TAG, "onStream() frame acquisition failed");
            result = ESP_FAIL;
            break;
        }
        char header[128];
        const int length = std::snprintf(header, sizeof(header),
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            unsigned(frame->len));
        // 网络发送不持有传感器锁，慢速客户端不会阻止网页调节参数。
        result = httpd_resp_send_chunk(req, header, length);
        if (result == ESP_OK)
        {
            result = httpd_resp_send_chunk(req, reinterpret_cast<const char*>(frame->buf), frame->len);
        }
        if (result == ESP_OK)
        {
            result = httpd_resp_send_chunk(req, "\r\n", 2);
        }
        if (result != ESP_OK)
        {
            break;
        }
        const int64_t now = esp_timer_get_time();
        if (window == 0)
        {
            // 以第一帧完成时间为起点，只统计之后的帧间隔，避免首个窗口多算一帧。
            window = now;
        }
        else
        {
            ++frames;
        }
        if (now - window >= 1000000)
        {
            std::lock_guard<std::mutex> lock(app->m_mutex);
            app->m_actualFps = frames * 1000000.0 / (now - window);
            frames = 0;
            window = now;
        }
        // 传输本身已经较慢时不追赶历史帧，避免恢复后瞬间突发发送。
        nextFrame = started + (1000000 + maxFps - 1) / maxFps;
    }
    app->finishStream();
    if (result == ESP_OK)
    {
        result = httpd_resp_send_chunk(req, "--frame--\r\n", HTTPD_RESP_USE_STRLEN);
        if (result == ESP_OK)
        {
            result = httpd_resp_send_chunk(req, nullptr, 0);
        }
    }
    ESP_LOGI(TAG, "onStream() << result=%s", esp_err_to_name(result));
    return result;
}
