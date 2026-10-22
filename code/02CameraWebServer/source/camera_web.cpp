#include "camera_web.h"
#if CONFIG_CAMERA_WIFI_STA
// 只有路由器模式依赖本地凭据；热点固件不需要也不包含路由器密码。
#include "wifi_credentials.local.h"
#endif
#include <cerrno>
#include <algorithm>
#include <cmath>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_psram.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#if CONFIG_CAMERA_PERF_TEST
#include "driver/ledc.h"
#include "driver/gpio.h"
#endif

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
    // 尽早撤销 PCA9685 可能保留的输出；未连接电机板时仍允许使用相机。
    const esp_err_t motorResult = m_motor.start();
    if (motorResult != ESP_OK)
    {
        ESP_LOGW(TAG, "Motor unavailable result=%s; camera startup continues", esp_err_to_name(motorResult));
    }
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
    // 外部时钟由板级实测选择；20 MHz 在当前板上会导致无线发送长时间停顿。
    config.xclk_freq_hz = CONFIG_CAMERA_XCLK_FREQ_HZ;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.pixel_format = PIXFORMAT_JPEG;
    // 按最大支持分辨率分配缓冲，运行时切换分辨率不会超出初始容量。
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    // 有空闲缓冲时采集；双缓冲允许下一帧采集与上一帧发送重叠，满后等待归还。
    config.fb_count = m_frameCount;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
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
        sensor->set_quality(sensor, 12) != 0 || applySensorClock(sensor) != ESP_OK)
    {
        ESP_LOGE(TAG, "initCamera() << result=INITIAL_SETTINGS_FAILED");
        esp_camera_deinit();
        return ESP_FAIL;
    }
    // 每个缓冲都可能保留初始化 UXGA 阶段的旧帧，逐个丢弃后再开放 HTTP 服务。
    for (int index = 0; index < m_frameCount; ++index)
    {
        camera_fb_t* initial = esp_camera_fb_get();
        if (initial == nullptr)
        {
            ESP_LOGE(TAG, "initCamera() << result=INITIAL_FRAME_TIMEOUT");
            esp_camera_deinit();
            return ESP_ERR_TIMEOUT;
        }
        esp_camera_fb_return(initial);
    }
    // 初始化阶段尚无并发请求；先发布快照，再开放网页和运动服务。
    m_size = 2;
    cacheStatus();
    ESP_LOGI(TAG, "initCamera() << result=OK sensor=OV2640 resolution=640x480 fb_count=%d grab_mode=when_empty cpu_mhz=%d xclk_hz=%d",
             m_frameCount, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, CONFIG_CAMERA_XCLK_FREQ_HZ);
    return ESP_OK;
}

// 输入：传感器；输出：设置结果；作用：使用芯片内部倍频，避免提高板上 GPIO0 时钟。
esp_err_t CameraWeb::applySensorClock(sensor_t* sensor)
{
#if CONFIG_CAMERA_SENSOR_CLK2X
    // 驱动的 set_framesize 会重写 CLKRC，初始化和后续分辨率切换都必须重新应用。
    ESP_LOGI(TAG, "sensor set_reg() >> reg=0x111 mask=0x80 value=0x80");
    const int result = sensor->set_reg(sensor, 0x111, 0x80, 0x80);
    ESP_LOGI(TAG, "sensor set_reg() << result=%d", result);
    if (result != 0)
    {
        return ESP_FAIL;
    }
#endif
    // 大分辨率沿用驱动的较慢像素时钟，避免高分辨率瞬时数据流超过 DMA 搬运余量。
    const int divider = sensor->status.framesize > FRAMESIZE_SVGA && CONFIG_CAMERA_DVP_DIV < 12 ?
        12 : CONFIG_CAMERA_DVP_DIV;
    ESP_LOGI(TAG, "sensor set_reg() >> reg=0x0d3 mask=0x7f divider=%d", divider);
    const int dvpResult = sensor->set_reg(sensor, 0xd3, 0x7f, divider);
    ESP_LOGI(TAG, "sensor set_reg() << result=%d", dvpResult);
    return dvpResult == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t CameraWeb::initWifi()
{
#if CONFIG_CAMERA_WIFI_AP
    ESP_LOGI(TAG, "initWifi() >> mode=AP ssid=%s", CONFIG_CAMERA_AP_SSID);
    static_assert(sizeof(CONFIG_CAMERA_AP_SSID) > 1 && sizeof(CONFIG_CAMERA_AP_SSID) <= 33);
    static_assert(sizeof(CONFIG_CAMERA_AP_PASSWORD) >= 9 && sizeof(CONFIG_CAMERA_AP_PASSWORD) <= 64);
#else
    ESP_LOGI(TAG, "initWifi() >> ssid=%s", WIFI_SSID);
    static_assert(sizeof(WIFI_SSID) - 1 <= 32);
    static_assert(sizeof(WIFI_PASSWORD) - 1 <= 64);
#endif
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
#if CONFIG_CAMERA_WIFI_AP
    esp_netif_t* netif = esp_netif_create_default_wifi_ap();
#else
    esp_netif_t* netif = esp_netif_create_default_wifi_sta();
#endif
    if (netif == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }
    const wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_LOGI(TAG, "Wi-Fi TX config type=%d static_buffers=%d tcp_snd_buf=%d",
             init.tx_buf_type, init.static_tx_buf_num, CONFIG_LWIP_TCP_SND_BUF_DEFAULT);
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "wifi init failed");
    // 两种模式均使用内存配置，不把密码写入 Wi-Fi 配置的 NVS 副本或日志。
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, onWifiEvent, this, &m_wifiEvent), TAG, "wifi event failed");
    wifi_config_t config = {};
#if CONFIG_CAMERA_WIFI_AP
    // 设置固定地址前显式停止 DHCP；地址更新后重新启用，热点启动时分配客户端地址。
    // 即使 DHCP 尚未运行，ESP-NETIF 也要求它处于 STOPPED 状态才允许设置 IP。
    ESP_RETURN_ON_ERROR(esp_netif_dhcps_stop(netif), TAG, "AP DHCP stop failed");
    esp_netif_ip_info_t ip = {};
    ip.ip.addr = ESP_IP4TOADDR(192, 168, 4, 1);
    ip.gw.addr = ip.ip.addr;
    ip.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);
    ESP_RETURN_ON_ERROR(esp_netif_set_ip_info(netif, &ip), TAG, "AP address failed");
    ESP_RETURN_ON_ERROR(esp_netif_dhcps_start(netif), TAG, "AP DHCP start failed");
    std::memcpy(config.ap.ssid, CONFIG_CAMERA_AP_SSID, sizeof(CONFIG_CAMERA_AP_SSID) - 1);
    config.ap.ssid_len = sizeof(CONFIG_CAMERA_AP_SSID) - 1;
    std::memcpy(config.ap.password, CONFIG_CAMERA_AP_PASSWORD, sizeof(CONFIG_CAMERA_AP_PASSWORD) - 1);
    config.ap.channel = 6;
    config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    // 允许手机和电脑接入；视频服务仍只支持一个预览连接。
    config.ap.max_connection = 2;
    config.ap.pmf_cfg.required = false;
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "AP mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &config), TAG, "AP config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "AP start failed");
#else
    // 保留原有 STA 连接及三秒重试逻辑，AP 分支不创建此定时器。
    const esp_timer_create_args_t timer = {
        .callback = reconnectWifi,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "wifi_retry",
        .skip_unhandled_events = false,
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer, &m_retryTimer), TAG, "retry timer failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, onWifiEvent, this, &m_ipEvent), TAG, "ip event failed");
    std::memcpy(config.sta.ssid, WIFI_SSID, sizeof(WIFI_SSID) - 1);
    std::memcpy(config.sta.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD) - 1);
    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    config.sta.pmf_cfg.capable = true;
    config.sta.pmf_cfg.required = false;
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "wifi mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &config), TAG, "wifi config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");
#endif
    // 两种接入方式均显式关闭调制解调器省电，并回读状态，避免仅依赖默认策略。
    ESP_LOGI(TAG, "esp_wifi_set_ps() >> type=WIFI_PS_NONE");
    result = esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "esp_wifi_set_ps() << result=%s", esp_err_to_name(result));
    ESP_RETURN_ON_ERROR(result, TAG, "wifi power mode failed");
    wifi_ps_type_t powerSave = WIFI_PS_MIN_MODEM;
    ESP_LOGI(TAG, "esp_wifi_get_ps() >>");
    result = esp_wifi_get_ps(&powerSave);
    ESP_LOGI(TAG, "esp_wifi_get_ps() << result=%s type=%d", esp_err_to_name(result), int(powerSave));
    ESP_RETURN_ON_ERROR(result, TAG, "wifi power mode readback failed");
    if (powerSave != WIFI_PS_NONE)
    {
        ESP_LOGE(TAG, "initWifi() << result=POWER_SAVE_NOT_DISABLED");
        return ESP_FAIL;
    }
    // SDK 参数单位为 0.25 dBm，80 表示 20 dBm 的发送功率上限；逐包功率仍受 PHY 约束。
    ESP_LOGI(TAG, "esp_wifi_set_max_tx_power() >> quarter_dbm=80");
    result = esp_wifi_set_max_tx_power(80);
    ESP_LOGI(TAG, "esp_wifi_set_max_tx_power() << result=%s", esp_err_to_name(result));
    ESP_RETURN_ON_ERROR(result, TAG, "wifi tx power failed");
    int8_t txPower = 0;
    ESP_LOGI(TAG, "esp_wifi_get_max_tx_power() >>");
    result = esp_wifi_get_max_tx_power(&txPower);
    ESP_LOGI(TAG, "esp_wifi_get_max_tx_power() << result=%s quarter_dbm=%d",
             esp_err_to_name(result), int(txPower));
    ESP_RETURN_ON_ERROR(result, TAG, "wifi tx power readback failed");
    ESP_LOGI(TAG, "power settings cpu_mhz=%u wifi_ps=%d max_tx_quarter_dbm=%d",
             unsigned(esp_rom_get_cpu_ticks_per_us()), int(powerSave), int(txPower));
#if CONFIG_CAMERA_WIFI_AP
    ESP_LOGI(TAG, "initWifi() << result=OK mode=AP");
#else
    ESP_LOGI(TAG, "initWifi() << result=OK waiting_for_ip");
#endif
    return ESP_OK;
}

void CameraWeb::onWifiEvent(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    auto* app = static_cast<CameraWeb*>(arg);
#if CONFIG_CAMERA_WIFI_AP
    if (base != WIFI_EVENT)
    {
        return;
    }
    if (id == WIFI_EVENT_AP_START)
    {
        // AP 启动即表示网络接口就绪，不等待 STA 模式才会收到的 GOT_IP 事件。
        app->m_connected.store(true);
        ESP_LOGI(TAG, "SoftAP started ssid=%s; open http://192.168.4.1/", CONFIG_CAMERA_AP_SSID);
    }
    else if (id == WIFI_EVENT_AP_STOP)
    {
        app->m_connected.store(false);
        app->m_motor.requestStop();
        app->m_epoch.fetch_add(1);
        ESP_LOGW(TAG, "SoftAP stopped");
    }
    else if (id == WIFI_EVENT_AP_STACONNECTED)
    {
        const auto* event = static_cast<wifi_event_ap_staconnected_t*>(data);
        ESP_LOGI(TAG, "SoftAP client connected aid=%u", event->aid);
    }
    else if (id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        // 任一热点客户端离开都要求停车；只投递要求，不阻塞 Wi-Fi 事件线程。
        app->m_motor.requestStop();
        const auto* event = static_cast<wifi_event_ap_stadisconnected_t*>(data);
        ESP_LOGI(TAG, "SoftAP client disconnected aid=%u reason=%u", event->aid, event->reason);
        wifi_sta_list_t clients = {};
        const esp_err_t result = esp_wifi_ap_get_sta_list(&clients);
        if (result == ESP_OK && clients.num == 0)
        {
            // 最后一个客户端离开后结束旧视频代次，热点本身仍保持可连接。
            app->m_epoch.fetch_add(1);
        }
        else if (result != ESP_OK)
        {
            ESP_LOGW(TAG, "AP client list failed result=%s", esp_err_to_name(result));
        }
    }
#else
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
        app->m_motor.requestStop();
        // 断线使旧视频循环失效；三秒后重试，持续等待路由器恢复。
        app->m_epoch.fetch_add(1);
        ESP_LOGW(TAG, "Wi-Fi disconnected reason=%u; retry in 3 seconds", event->reason);
        esp_timer_stop(app->m_retryTimer);
        esp_timer_start_once(app->m_retryTimer, 3000000);
    }
#endif
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
    // 三个服务按内部九项加 4+1+3 个客户端保守预算，余量用于 DHCP 等网络操作。
    static_assert(CONFIG_LWIP_MAX_SOCKETS >= 20, "Three HTTP servers need at least 20 sockets");
    ESP_LOGI(TAG, "startServers() >>");
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 8192;
#if CONFIG_CAMERA_PERF_TEST
    config.max_uri_handlers = 6;
#else
    config.max_uri_handlers = 5;
#endif
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;
    config.send_wait_timeout = 3;
    config.recv_wait_timeout = 3;
    // 三个服务复用连接配置，所有新连接均禁用 TCP 小包合并等待。
    config.open_fn = onSocketOpen;
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
#if CONFIG_CAMERA_PERF_TEST
    // 测量接口只在专用测试构建中注册，常规固件不暴露停止相机的操作。
    httpd_uri_t perfUri = {};
    perfUri.uri = "/perf";
    perfUri.method = HTTP_GET;
    perfUri.handler = onRequest;
    perfUri.user_ctx = this;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(m_control, &perfUri), TAG, "perf route failed");
#endif
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
    // 运动单独使用服务任务；相机取帧、抓拍发送及参数设置不占用此请求队列。
    config.server_port = 82;
    config.ctrl_port += 1;
    config.stack_size = 4096;
    config.task_priority = tskIDLE_PRIORITY + 6;
    config.max_uri_handlers = 3;
    config.max_open_sockets = 3;
    // 不因临时只读请求淘汰仍有效的控制长连接。
    config.lru_purge_enable = false;
    config.close_fn = closeMotorSocket;
    config.recv_wait_timeout = 1;
    config.send_wait_timeout = 1;
    ESP_LOGI(TAG, "httpd_start() >> port=82 priority=6 stack_bytes=4096 max_clients=3 timeout_s=1");
    esp_err_t motionResult = httpd_start(&m_motion, &config);
    ESP_LOGI(TAG, "httpd_start() << result=%s port=82", esp_err_to_name(motionResult));
    if (motionResult == ESP_OK)
    {
        // 同一路径的 GET 只读状态，POST 才能续期或停车，均不接触相机锁。
        for (httpd_method_t method : {HTTP_GET, HTTP_POST})
        {
            httpd_uri_t motorUri = {};
            motorUri.uri = "/motor";
            motorUri.method = method;
            motorUri.handler = onMotor;
            motorUri.user_ctx = this;
            ESP_LOGI(TAG, "httpd_register_uri_handler() >> port=82 uri=/motor method=%s",
                     method == HTTP_GET ? "GET" : "POST");
            motionResult = httpd_register_uri_handler(m_motion, &motorUri);
            ESP_LOGI(TAG, "httpd_register_uri_handler() << result=%s", esp_err_to_name(motionResult));
            if (motionResult != ESP_OK) break;
        }
    }
    if (motionResult == ESP_OK)
    {
        httpd_uri_t socketUri = {};
        socketUri.uri = "/motor/ws";
        socketUri.method = HTTP_GET;
        socketUri.handler = onMotorSocket;
        socketUri.user_ctx = this;
        socketUri.is_websocket = true;
        socketUri.ws_pre_handshake_cb = openMotorSocket;
        ESP_LOGI(TAG, "httpd_register_uri_handler() >> port=82 uri=/motor/ws websocket=1");
        motionResult = httpd_register_uri_handler(m_motion, &socketUri);
        ESP_LOGI(TAG, "httpd_register_uri_handler() << result=%s", esp_err_to_name(motionResult));
    }
    if (motionResult != ESP_OK)
    {
        // 运动服务不可用时不留下半启动状态，关闭已创建的服务并保持电机停止。
        m_motor.stop("motion server failed");
        if (m_motion != nullptr) httpd_stop(m_motion);
        httpd_stop(m_stream);
        httpd_stop(m_control);
        m_motion = nullptr;
        m_stream = nullptr;
        m_control = nullptr;
        ESP_LOGE(TAG, "startServers() << result=%s", esp_err_to_name(motionResult));
        return motionResult;
    }
    ESP_LOGI(TAG, "startServers() << result=OK control=80 stream=81 motor=82 socket_budget=%d",
             CONFIG_LWIP_MAX_SOCKETS);
    return ESP_OK;
}

esp_err_t CameraWeb::onSocketOpen(httpd_handle_t server, int socketFd)
{
    const int enabled = 1;
    // HTTP 头、分块长度和 MJPEG 边界都是小包；禁用 Nagle，避免等待合并后才发送。
    if (setsockopt(socketFd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled)) != 0)
    {
        ESP_LOGE(TAG, "onSocketOpen() << result=SET_NODELAY_FAILED socket=%d errno=%d", socketFd, errno);
        return ESP_FAIL;
    }
    int actual = 0;
    socklen_t length = sizeof(actual);
    if (getsockopt(socketFd, IPPROTO_TCP, TCP_NODELAY, &actual, &length) != 0 || actual != 1)
    {
        // 读取实际状态，不能仅凭设置调用成功就宣称新连接已使用低延迟模式。
        ESP_LOGE(TAG, "onSocketOpen() << result=VERIFY_NODELAY_FAILED socket=%d errno=%d value=%d",
                 socketFd, errno, actual);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "onSocketOpen() << result=OK socket=%d tcp_nodelay=%d", socketFd, actual);
    return ESP_OK;
}

esp_err_t CameraWeb::onRequest(httpd_req_t* req)
{
    auto* app = static_cast<CameraWeb*>(req->user_ctx);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
#if CONFIG_CAMERA_PERF_TEST
    if (std::strncmp(req->uri, "/perf", 5) == 0 &&
        (req->uri[5] == '\0' || req->uri[5] == '?'))
    {
        return app->sendPerf(req);
    }
#endif
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
        const int64_t started = esp_timer_get_time();
        int64_t lockUs;
        camera_fb_t* frame = nullptr;
        {
            std::lock_guard<std::mutex> lock(app->m_mutex);
            lockUs = esp_timer_get_time() - started;
            ESP_LOGI(TAG, "capture esp_camera_fb_get() >>");
            frame = esp_camera_fb_get();
        }
        const int64_t gotFrame = esp_timer_get_time();
        // 帧缓冲由驱动拥有，使用 RAII 保证发送失败时也归还。
        std::unique_ptr<camera_fb_t, decltype(&esp_camera_fb_return)>
            holder(frame, esp_camera_fb_return);
        if (frame == nullptr)
        {
            ESP_LOGW(TAG, "capture esp_camera_fb_get() << result=NO_FRAME elapsed_ms=%lld",
                     (long long)((gotFrame - started) / 1000));
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Capture failed");
        }
        ESP_LOGI(TAG, "capture esp_camera_fb_get() << bytes=%u capture_ms=%lld lock_ms=%lld",
                 unsigned(frame->len), (long long)((gotFrame - started - lockUs) / 1000),
                 (long long)(lockUs / 1000));
        httpd_resp_set_type(req, "image/jpeg");
        ESP_LOGI(TAG, "capture httpd_resp_send() >> bytes=%u", unsigned(frame->len));
        const esp_err_t result = httpd_resp_send(req, reinterpret_cast<const char*>(frame->buf), frame->len);
        const int sendErrno = errno;
        ESP_LOGI(TAG, "capture httpd_resp_send() << result=%s errno=%d send_ms=%lld",
                 esp_err_to_name(result), result == ESP_OK ? 0 : sendErrno,
                 (long long)((esp_timer_get_time() - gotFrame) / 1000));
        return result;
    }
    return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
}

#if CONFIG_CAMERA_PERF_TEST
// 输入：可选的相机开关查询；输出：固定 64 KiB 数据；作用：隔离采图与 TCP 传输的影响。
esp_err_t CameraWeb::sendPerf(httpd_req_t* req)
{
    ESP_LOGI(TAG, "sendPerf() >> uri=%s", req->uri);
    char query[64] = {};
    char mode[12] = {};
    char clock[12] = {};
    char channel[8] = {};
    char frequency[12] = {};
    char drive[8] = {};
    char pll[8] = {};
    char buffers[8] = {};
    char dvp[8] = {};
    char sample[8] = {};
    httpd_req_get_url_query_str(req, query, sizeof(query));
    httpd_query_key_value(query, "camera", mode, sizeof(mode));
    httpd_query_key_value(query, "clock", clock, sizeof(clock));
    httpd_query_key_value(query, "channel", channel, sizeof(channel));
    httpd_query_key_value(query, "xclk", frequency, sizeof(frequency));
    httpd_query_key_value(query, "drive", drive, sizeof(drive));
    httpd_query_key_value(query, "pll", pll, sizeof(pll));
    httpd_query_key_value(query, "buffers", buffers, sizeof(buffers));
    httpd_query_key_value(query, "dvp", dvp, sizeof(dvp));
    httpd_query_key_value(query, "sample", sample, sizeof(sample));
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        // 只允许在没有预览时切换硬件，避免销毁仍由视频发送任务持有的帧。
        if (m_streaming)
        {
            ESP_LOGW(TAG, "sendPerf() << result=STREAM_BUSY");
            httpd_resp_set_status(req, "409 Conflict");
            return httpd_resp_sendstr(req, "Stop stream first");
        }
        esp_err_t result = ESP_OK;
        if (buffers[0] != '\0')
        {
            // 比较帧缓冲数时完整重建驱动，其他初始化参数使用同一组构建配置。
            const int count = std::atoi(buffers);
            if (count != 1 && count != 2)
            {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid buffer count");
            }
            if (count != m_frameCount)
            {
                ESP_LOGI(TAG, "perf rebuild camera >> buffers=%d", count);
                result = esp_camera_deinit();
                if (result == ESP_OK)
                {
                    m_frameCount = count;
                    result = initCamera();
                }
                ESP_LOGI(TAG, "perf rebuild camera << result=%s", esp_err_to_name(result));
            }
        }
        if (std::strcmp(mode, "off") == 0 && esp_camera_sensor_get() != nullptr)
        {
            ESP_LOGI(TAG, "esp_camera_deinit() >>");
            result = esp_camera_deinit();
            ESP_LOGI(TAG, "esp_camera_deinit() << result=%s", esp_err_to_name(result));
        }
        else if (std::strcmp(mode, "on") == 0 && esp_camera_sensor_get() == nullptr)
        {
            result = initCamera();
        }
        // 仅暂停时钟不释放驱动、DMA 内存或相机任务，用于排除内存释放带来的干扰。
        if (result == ESP_OK && esp_camera_sensor_get() != nullptr && clock[0] != '\0')
        {
            ESP_LOGI(TAG, "ledc camera clock >> mode=%s", clock);
            if (std::strcmp(clock, "off") == 0)
            {
                result = ledc_timer_pause(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
            }
            else if (std::strcmp(clock, "on") == 0)
            {
                result = ledc_timer_resume(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
            }
            else
            {
                result = ESP_ERR_INVALID_ARG;
            }
            ESP_LOGI(TAG, "ledc camera clock << result=%s", esp_err_to_name(result));
        }
        if (result == ESP_OK && esp_camera_sensor_get() != nullptr && frequency[0] != '\0')
        {
            // 测试频点均在 OV2640 外部时钟范围内，每次只改变一个参数。
            const int hz = std::atoi(frequency);
            ESP_LOGI(TAG, "ledc_set_freq() >> hz=%d", hz);
            result = hz == 8000000 || hz == 10000000 || hz == 16000000 || hz == 20000000 ?
                ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, hz) : ESP_ERR_INVALID_ARG;
            ESP_LOGI(TAG, "ledc_set_freq() << result=%s actual_hz=%u", esp_err_to_name(result),
                     unsigned(ledc_get_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0)));
        }
        if (result == ESP_OK && esp_camera_sensor_get() != nullptr && drive[0] != '\0')
        {
            // 保持频率与内存不变，只比较 GPIO0 输出驱动强度，测试后恢复默认档位。
            gpio_drive_cap_t strength = GPIO_DRIVE_CAP_2;
            if (std::strcmp(drive, "0") == 0)
            {
                strength = GPIO_DRIVE_CAP_0;
            }
            else if (std::strcmp(drive, "2") != 0)
            {
                result = ESP_ERR_INVALID_ARG;
            }
            if (result == ESP_OK)
            {
                ESP_LOGI(TAG, "gpio_set_drive_capability() >> gpio=0 level=%s", drive);
                result = gpio_set_drive_capability(GPIO_NUM_0, strength);
                ESP_LOGI(TAG, "gpio_set_drive_capability() << result=%s", esp_err_to_name(result));
            }
        }
        if (result == ESP_OK && esp_camera_sensor_get() != nullptr && pll[0] != '\0')
        {
            // OV2640 的 SENSOR bank CLKRC(0x11) bit7 控制内部倍频，不提高外部 GPIO 时钟。
            // 使用驱动的带 bank 地址接口，只修改这一位，其他分频位保持原样。
            sensor_t* sensor = esp_camera_sensor_get();
            if (std::strcmp(pll, "0") == 0 || std::strcmp(pll, "1") == 0)
            {
                ESP_LOGI(TAG, "sensor set_reg() >> reg=0x111 mask=0x80 pll=%s", pll);
                result = sensor->set_reg(sensor, 0x111, 0x80, pll[0] == '1' ? 0x80 : 0) == 0 ?
                    ESP_OK : ESP_FAIL;
                ESP_LOGI(TAG, "sensor set_reg() << result=%s", esp_err_to_name(result));
            }
            else
            {
                result = ESP_ERR_INVALID_ARG;
            }
        }
        if (result == ESP_OK && esp_camera_sensor_get() != nullptr && dvp[0] != '\0')
        {
            // 只比较合法的手动像素时钟分频；保留 R_DVP_SP 的自动分频控制位。
            const int divider = std::atoi(dvp);
            if (divider == 2 || divider == 4 || divider == 8 || divider == 12)
            {
                ESP_LOGI(TAG, "sensor set_reg() >> reg=0x0d3 mask=0x7f divider=%d", divider);
                result = esp_camera_sensor_get()->set_reg(esp_camera_sensor_get(), 0xd3, 0x7f, divider) == 0 ?
                    ESP_OK : ESP_FAIL;
                ESP_LOGI(TAG, "sensor set_reg() << result=%s", esp_err_to_name(result));
            }
            else
            {
                result = ESP_ERR_INVALID_ARG;
            }
        }
        // 测量接口可能关闭或重建相机，及时使缓存失效或发布重建后的状态。
        cacheStatus();
        if (result != ESP_OK)
        {
            ESP_LOGE(TAG, "sendPerf() << result=%s", esp_err_to_name(result));
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera switch failed");
        }
        if (sample[0] != '\0')
        {
            const int count = std::atoi(sample);
            if (count < 2 || count > 30 || esp_camera_sensor_get() == nullptr)
            {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid sample count");
            }
            int captured = 0;
            int64_t firstUs = 0;
            int64_t lastUs = 0;
            // 测量窗口内不发送图像，取帧后立即归还，单独得到采集通路的节拍上限。
            // 首帧可能已缓存，不把它算作新采样；遇到超时立即退出，最多等待一次驱动超时。
            for (int index = 0; index < count; ++index)
            {
                ESP_LOGI(TAG, "bench esp_camera_fb_get() >> index=%d", index);
                camera_fb_t* frame = esp_camera_fb_get();
                const int64_t now = esp_timer_get_time();
                ESP_LOGI(TAG, "bench esp_camera_fb_get() << index=%d result=%s", index,
                         frame != nullptr ? "OK" : "NO_FRAME");
                if (frame == nullptr)
                {
                    break;
                }
                if (captured == 0)
                {
                    firstUs = now;
                }
                lastUs = now;
                ++captured;
                esp_camera_fb_return(frame);
            }
            char json[160];
            const double fps = captured > 1 && lastUs > firstUs ?
                (captured - 1) * 1000000.0 / (lastUs - firstUs) : 0;
            std::snprintf(json, sizeof(json), "{\"captured\":%d,\"fps\":%.3f,\"elapsed_ms\":%lld,\"buffers\":%d}",
                          captured, fps, (long long)((lastUs - firstUs) / 1000), m_frameCount);
            ESP_LOGI(TAG, "sendPerf() << %s", json);
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_sendstr(req, json);
        }
    }
#if CONFIG_CAMERA_WIFI_AP
    if (channel[0] != '\0')
    {
        // 只调整开发板热点信道，不修改电脑配置；客户端可能需要重新关联热点。
        const int number = std::atoi(channel);
        ESP_LOGI(TAG, "esp_wifi_set_channel() >> channel=%d", number);
        const esp_err_t result = number == 1 || number == 6 || number == 11 ?
            esp_wifi_set_channel(number, WIFI_SECOND_CHAN_NONE) : ESP_ERR_INVALID_ARG;
        ESP_LOGI(TAG, "esp_wifi_set_channel() << result=%s", esp_err_to_name(result));
        if (result != ESP_OK)
        {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Channel switch failed");
        }
    }
    wifi_sta_list_t clients = {};
    const esp_err_t listResult = esp_wifi_ap_get_sta_list(&clients);
    if (listResult == ESP_OK && clients.num > 0)
    {
        ESP_LOGI(TAG, "perf AP client rssi_dbm=%d phy_n=%u internal_free=%u",
                 int(clients.sta[0].rssi), unsigned(clients.sta[0].phy_11n),
                 unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
    }
#endif
    // 不使用图像数据；固定负载在整个发送期间保持有效，结果不受 JPEG 内容影响。
    std::unique_ptr<char[]> data(new (std::nothrow) char[4096]);
    if (!data)
    {
        ESP_LOGE(TAG, "sendPerf() << result=NO_MEMORY");
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No memory");
    }
    std::memset(data.get(), 0x5a, 4096);
    httpd_resp_set_type(req, "application/octet-stream");
    const int64_t started = esp_timer_get_time();
    unsigned sentBytes = 0;
    esp_err_t result = ESP_OK;
    // 总量固定 64 KiB，任一分块失败立即退出，不用重试掩盖吞吐问题。
    for (int index = 0; index < 16 && result == ESP_OK; ++index)
    {
        result = httpd_resp_send_chunk(req, data.get(), 4096);
        if (result == ESP_OK)
        {
            sentBytes += 4096;
        }
    }
    const int sendErrno = errno;
    if (result == ESP_OK)
    {
        result = httpd_resp_send_chunk(req, nullptr, 0);
    }
    ESP_LOGI(TAG, "sendPerf() << result=%s errno=%d queued_bytes=%u send_ms=%lld camera=%s",
             esp_err_to_name(result), result == ESP_OK ? 0 : sendErrno, sentBytes,
             (long long)((esp_timer_get_time() - started) / 1000),
             esp_camera_sensor_get() == nullptr ? "off" : "on");
    return result;
}
#endif

// 输入：请求和来源要求；输出：是否允许；作用：HTTP 诊断可无来源，浏览器长连接必须同源设备。
bool CameraWeb::isMotorOrigin(httpd_req_t* req, bool required)
{
    char origin[96] = {};
    char host[64] = {};
    char allowed[80] = {};
    if (httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) != ESP_OK)
    {
        // 运动页面从同一设备的 80 端口加载，不接受缺少有效设备地址的请求。
        return false;
    }
    // 当前工程使用 IPv4/主机名和 HTTP；去掉运动端口，得到网页所在的 80 端口来源。
    const size_t length = std::strcspn(host, ":");
    std::snprintf(allowed, sizeof(allowed), "http://%.*s", static_cast<int>(length), host);
    if (httpd_req_get_hdr_value_len(req, "Origin") > 0)
    {
        if (httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof(origin)) != ESP_OK ||
            std::strcmp(origin, allowed) != 0)
        {
            // 仅允许本设备网页跨端口读取握手结果，不向任意网站开放运动通道。
            return false;
        }
        return true;
    }
    return !required;
}

// 输入：运动 HTTP 请求；输出：响应结果；作用：保留低频诊断和备用停车，不用于网页周期续期。
esp_err_t CameraWeb::onMotor(httpd_req_t* req)
{
    auto* app = static_cast<CameraWeb*>(req->user_ctx);
    if (!isMotorOrigin(req, false))
    {
        ESP_LOGW(TAG, "onMotor() << result=ORIGIN_REJECTED");
        return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Use the device webpage on port 80");
    }
    // SDK 保留字符串地址直到响应发送完成；缓冲必须留在当前请求调用栈中。
    char origin[96] = {};
    if (httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof(origin)) == ESP_OK)
    {
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", origin);
        httpd_resp_set_hdr(req, "Vary", "Origin");
    }
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return req->method == HTTP_GET ? app->sendMotor(req) : app->setMotor(req);
}

// 输入：升级请求；输出：准入结果；作用：握手前保留唯一控制连接，不自动启动电机。
esp_err_t CameraWeb::openMotorSocket(httpd_req_t* req)
{
    auto* app = static_cast<CameraWeb*>(req->user_ctx);
    ESP_LOGI(TAG, "openMotorSocket() >>");
    if (!isMotorOrigin(req, true))
    {
        ESP_LOGW(TAG, "openMotorSocket() << result=ORIGIN_REJECTED");
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Use the device webpage on port 80");
        return ESP_FAIL;
    }
    if (app->m_motorFd >= 0)
    {
        // 控制页面独占连接，额外页面不能抢占正在运动的操作者。
        ESP_LOGW(TAG, "openMotorSocket() << result=CONTROLLER_BUSY");
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "Another control page is connected");
        return ESP_FAIL;
    }
    app->m_motorFd = httpd_req_to_sockfd(req);
    app->m_motorSeq = 0;
    app->m_reportUs = 0;
    app->m_reportGen = 0;
    app->m_isReportedMoving = false;
    ESP_LOGI(TAG, "openMotorSocket() << result=ESP_OK socket=%d", app->m_motorFd);
    return ESP_OK;
}

// 输入：连接、序号和业务结果；输出：发送结果；作用：单次非阻塞写入完整 WS 文本帧。
esp_err_t CameraWeb::reportMotor(int socketFd, uint32_t sequence, esp_err_t result)
{
    const MotorState state = m_motor.getState();
    char text[126];
    const int length = std::snprintf(text, sizeof(text),
        "{\"e\":%d,\"r\":%d,\"m\":%d,\"f\":%d,\"g\":%lu,\"s\":%lu,\"c\":%d}",
        int(state.enabled), int(state.ready), int(state.moving), state.fault,
        static_cast<unsigned long>(state.generation), static_cast<unsigned long>(sequence), result);
    if (length <= 0 || length >= static_cast<int>(sizeof(text)))
    {
        // 短文本帧不超过 125 字节，拒绝截断数据，不能破坏后续帧边界。
        ESP_LOGE(TAG, "reportMotor() << result=STATE_TOO_LARGE");
        return ESP_FAIL;
    }
    std::array<uint8_t, 127> packet{};
    // 服务端帧无需掩码；将头和正文合成一次写入，避免两个微小 TCP 包。
    packet[0] = 0x81;
    packet[1] = static_cast<uint8_t>(length);
    std::memcpy(packet.data() + 2, text, length);
    ESP_LOGD(TAG, "send() >> socket=%d bytes=%d flags=MSG_DONTWAIT", socketFd, length + 2);
    const int count = send(socketFd, packet.data(), length + 2, MSG_DONTWAIT);
    const int sendErr = count < 0 ? errno : 0;
    ESP_LOGD(TAG, "send() << result=%d errno=%d", count, sendErr);
    if (count < 0 && (sendErr == EAGAIN || sendErr == EWOULDBLOCK))
    {
        // 缓冲拥塞时不排队、不等待 ACK；下一条输入合并为最新状态再次尝试。
        return ESP_OK;
    }
    if (count != length + 2)
    {
        // 已写入半帧时不能接着发送另一帧；关闭连接，触发失联停车。
        ESP_LOGW(TAG, "reportMotor() << result=SEND_FAILED bytes=%d errno=%d", count, sendErr);
        return ESP_FAIL;
    }
    m_reportUs = esp_timer_get_time();
    m_reportGen = state.generation;
    m_isReportedMoving = state.moving;
    return ESP_OK;
}

// 输入：12 字节二进制指令；输出：接收结果；作用：顺序验证后执行，正常续期每 500 ms 合并回报。
esp_err_t CameraWeb::onMotorSocket(httpd_req_t* req)
{
    auto* app = static_cast<CameraWeb*>(req->user_ctx);
    const int socketFd = httpd_req_to_sockfd(req);
    std::array<uint8_t, 12> data{};
    httpd_ws_frame_t frame = {};
    frame.payload = data.data();
    ESP_LOGD(TAG, "httpd_ws_recv_frame() >> socket=%d max_bytes=12", socketFd);
    const esp_err_t received = httpd_ws_recv_frame(req, &frame, data.size());
    ESP_LOGD(TAG, "httpd_ws_recv_frame() << result=%s bytes=%u", esp_err_to_name(received), unsigned(frame.len));
    if (received != ESP_OK || frame.type != HTTPD_WS_TYPE_BINARY || frame.len != data.size() ||
        socketFd != app->m_motorFd || data[0] > 2 || data[1] > 200 || data[2] > 200 || data[3] > 200)
    {
        // 畸形帧或非当前控制连接都不能执行运动；返回失败由关闭回调投递停车。
        ESP_LOGW(TAG, "onMotorSocket() << result=INVALID_FRAME socket=%d", socketFd);
        return ESP_FAIL;
    }
    uint32_t generation = 0;
    uint32_t sequence = 0;
    for (size_t index = 0; index < 4; ++index)
    {
        // 明确按网络大端解码，不依赖 ESP32 和浏览器端的本机字节序。
        generation = (generation << 8) | data[4 + index];
        sequence = (sequence << 8) | data[8 + index];
    }
    if (sequence == 0 || sequence <= app->m_motorSeq)
    {
        ESP_LOGW(TAG, "onMotorSocket() << result=STALE_SEQUENCE");
        return ESP_FAIL;
    }
    app->m_motorSeq = sequence;
    esp_err_t result = ESP_OK;
    if (data[0] == 2)
    {
        // 停车不要求代次，始终优先撤销当前输出；回报用于确认下一次按住的代次。
        result = app->m_motor.stop("websocket stop");
    }
    else if (data[0] == 1)
    {
        if (!app->m_connected.load())
        {
            app->m_motor.requestStop();
            result = ESP_ERR_INVALID_STATE;
        }
        else
        {
            result = app->m_motor.move(int(data[1]) - 100, int(data[2]) - 100,
                                       int(data[3]) - 100, generation);
        }
    }
    // 操作 0 只读取快照，不写外设，也不延长有效运动期限。
    const MotorState state = app->m_motor.getState();
    if (data[0] != 1 || result != ESP_OK || state.generation != app->m_reportGen ||
        state.moving != app->m_isReportedMoving ||
        esp_timer_get_time() - app->m_reportUs >= 500000)
    {
        // 查询、停车、故障立即回报；相同运动心跳不逐条发送响应。
        const esp_err_t sent = app->reportMotor(socketFd, sequence, result);
        return result == ESP_OK ? sent : ESP_FAIL;
    }
    return ESP_OK;
}

// 输入：关闭的连接；输出：无；作用：仅控制连接断开才停车，不让诊断 GET 的关闭干扰车辆。
void CameraWeb::closeMotorSocket(httpd_handle_t server, int socketFd)
{
    auto& app = CameraWeb::instance();
    if (socketFd == app.m_motorFd)
    {
        app.m_motorFd = -1;
        app.m_motorSeq = 0;
        app.m_motor.requestStop();
        ESP_LOGW(TAG, "closeMotorSocket() >> controller_disconnected socket=%d", socketFd);
    }
    ESP_LOGD(TAG, "close() >> socket=%d", socketFd);
    const int result = close(socketFd);
    ESP_LOGD(TAG, "close() << result=%d", result);
}

// 输入：传感器；输出：读取是否有效；作用：采集当前快门、额外行与行时序，发布前检查读数一致性。
bool CameraWeb::readExposure(sensor_t* sensor)
{
    ESP_LOGI(TAG, "readExposure() >>");
    m_expReadUs = esp_timer_get_time();
    m_isExposureValid = false;
    // 顺序依次为 PID、模式、时钟、水平补偿、帧长度、AEC、快门、额外行及复读校验。
    const int regs[] = {0x10a, 0x112, 0x111, 0x12a, 0x12b, 0x146, 0x147,
                        0x113, 0x145, 0x110, 0x104, 0x12e, 0x12d,
                        0x145, 0x110, 0x12e, 0x103, 0x0c2};
    int values[std::size(regs)]{};
    // 常规采样每两秒一轮，设置操作可立即回读；任一显式失败退出，等待下一轮或用户重试。
    for (size_t index = 0; index < std::size(regs); ++index)
    {
        ESP_LOGI(TAG, "sensor get_reg() >> reg=0x%03x mask=0xff", regs[index]);
        values[index] = sensor->get_reg(sensor, regs[index], 0xff);
        ESP_LOGI(TAG, "sensor get_reg() << reg=0x%03x value=%d", regs[index], values[index]);
        if (values[index] < 0 || values[index] > 255)
        {
            ESP_LOGW(TAG, "readExposure() << result=READ_FAILED");
            return false;
        }
    }
    // 底层 SCCB 失败可能返回零；PID 检查防止将全零读数发布成有效的零曝光。
    // AEC 自动更新时复读高位与中位，避免跨进位拼出不存在的快门或额外行数。
    if (values[0] != 0x26 || (values[8] & 0x3f) != (values[13] & 0x3f) ||
        values[9] != values[14] || values[11] != values[15])
    {
        ESP_LOGW(TAG, "readExposure() << result=INVALID_OR_CHANGING_REGISTERS");
        return false;
    }
    int rowClocks;
    int frameLines;
    // 根据传感器真实工作模式换算，不能用网页输出宽度或发送 FPS 代替行时序。
    switch (values[1] & 0x70)
    {
        case 0x00: rowClocks = 1922; frameLines = 1248; break;
        case 0x40: rowClocks = 1190; frameLines = 672; break;
        case 0x20: rowClocks = 595; frameLines = 336; break;
        default:
            ESP_LOGW(TAG, "readExposure() << result=UNSUPPORTED_MODE");
            return false;
    }
    const int dummyPixels = ((values[3] & 0xf0) << 4) | values[4];
    const int dummyLines = (values[6] << 8) | values[5];
    // JPEG 使用 YUV422 采集链路；参照 OpenMV OV2640 的时钟模型，结果明确作为估算值。
    // DVP 分频只控制 DSP 输出，不作为传感器积分时钟；不能使用 JPEG 网络发送耗时换算。
    const double clockHz = double(sensor->xclk_freq_hz) * 3 * ((values[2] & 0x80) ? 2 : 1) /
                           ((values[2] & 0x3f) + 2);
    if (clockHz <= 0 || sensor->pixformat != PIXFORMAT_JPEG)
    {
        ESP_LOGW(TAG, "readExposure() << result=UNSUPPORTED_CLOCK_OR_FORMAT");
        return false;
    }
    m_lineUs = (rowClocks + dummyPixels) * 2.0 * 1000000 / clockHz;
    const int shutter = ((values[8] & 0x3f) << 10) | (values[9] << 2) | (values[10] & 3);
    // 数据手册规定快门最长一帧；额外曝光行另计，不能遗漏低照度下增加的积分时间。
    m_expLines = std::min(shutter, frameLines + dummyLines);
    m_extraLines = (values[11] << 8) | values[12];
    m_maxShutter = std::min(1200, frameLines + dummyLines);
    m_isAutoExposure = (values[7] & 1) != 0;
    if (m_isAutoExposure)
    {
        // 手动模式只冻结自动控制，不改动 AGC；保存自动模式设置供用户重新开启时恢复。
        m_autoDsp = values[17] & 0x80;
        m_autoFrames = values[16] & 0xc0;
    }
    if (m_isAutoExposure || m_expMaxLines == 0)
    {
        // 额外曝光寄存器只有 16 位，输入范围不得超过驱动快门与额外行的可写总量。
        m_expMaxLines = std::min(m_maxShutter + 65535, std::max(m_maxShutter, m_expLines + m_extraLines));
    }
    sensor->status.aec = m_isAutoExposure ? 1 : 0;
    m_isExposureValid = true;
    ESP_LOGI(TAG, "readExposure() << result=OK auto=%d shutter=%d extra=%d line_us=%.3f exposure_ms=%.3f",
             int(m_isAutoExposure), m_expLines, m_extraLines, m_lineUs,
             (m_expLines + m_extraLines) * m_lineUs / 1000);
    return true;
}

// 输入：传感器与总行数；输出：是否成功；作用：写入手动积分量，短曝光时清除历史额外行。
bool CameraWeb::setExposure(sensor_t* sensor, int lines)
{
    ESP_LOGI(TAG, "setExposure() >> lines=%d", lines);
    if (!m_isExposureValid || m_isAutoExposure || lines < 1 || lines > m_expMaxLines)
    {
        // 自动模式拒绝手动值，避免下一帧被 AEC 覆盖后界面却提示成功。
        ESP_LOGW(TAG, "setExposure() << result=INVALID_STATE_OR_RANGE");
        return false;
    }
    const int shutter = std::min(lines, m_maxShutter);
    const int extra = lines - shutter;
    ESP_LOGI(TAG, "sensor set_aec_value() >> value=%d", shutter);
    int result = sensor->set_aec_value(sensor, shutter);
    ESP_LOGI(TAG, "sensor set_aec_value() << result=%d", result);
    // 额外行与快门共同控制积分；不足一帧时写零，保证可以从较长曝光调回短曝光。
    for (int index = 0; index < 2 && result == 0; ++index)
    {
        const int reg = 0x12d + index;
        const int value = (extra >> (index * 8)) & 0xff;
        ESP_LOGI(TAG, "sensor set_reg() >> reg=0x%03x mask=0xff value=%d", reg, value);
        result = sensor->set_reg(sensor, reg, 0xff, value);
        ESP_LOGI(TAG, "sensor set_reg() << reg=0x%03x result=%d", reg, result);
    }
    // 写入失败后也尝试回读实际值，不能继续把用户目标值显示成已生效值。
    const bool hasReadback = readExposure(sensor);
    const bool applied = result == 0 && hasReadback && m_expLines + m_extraLines == lines;
    if (!applied)
    {
        ESP_LOGW(TAG, "setExposure() << result=WRITE_OR_READBACK_FAILED");
        return false;
    }
    ESP_LOGI(TAG, "setExposure() << result=OK");
    return true;
}

// 输入：传感器和自动模式；输出：是否成功；作用：关闭时直接冻结当前寄存器，不写入历史默认曝光。
bool CameraWeb::setAutoExposure(sensor_t* sensor, bool enabled)
{
    ESP_LOGI(TAG, "setAutoExposure() >> enabled=%d", int(enabled));
    if (!readExposure(sensor))
    {
        ESP_LOGW(TAG, "setAutoExposure() << result=READ_FAILED");
        return false;
    }
    const bool wasAuto = m_isAutoExposure;
    if (enabled && wasAuto)
    {
        ESP_LOGI(TAG, "setAutoExposure() << result=UNCHANGED");
        return true;
    }
    // 先关闭 AEC 再冻结额外帧；开启时最后恢复 AEC，避免中间状态修改曝光。
    int result = 0;
    if (!enabled)
    {
        ESP_LOGI(TAG, "sensor set_exposure_ctrl() >> enabled=0");
        result = sensor->set_exposure_ctrl(sensor, 0);
        ESP_LOGI(TAG, "sensor set_exposure_ctrl() << result=%d", result);
    }
    const int regs[] = {0x0c2, 0x103};
    const int masks[] = {0x80, 0xc0};
    const int values[] = {enabled ? m_autoDsp : 0, enabled ? m_autoFrames : 0};
    // 仅修改曝光自动控制位，保留图像格式、窗口以及自动增益相关位。
    for (size_t index = 0; index < std::size(regs) && result == 0; ++index)
    {
        ESP_LOGI(TAG, "sensor set_reg() >> reg=0x%03x mask=0x%x value=%d",
                 regs[index], masks[index], values[index]);
        result = sensor->set_reg(sensor, regs[index], masks[index], values[index]);
        ESP_LOGI(TAG, "sensor set_reg() << reg=0x%03x result=%d", regs[index], result);
    }
    if (enabled && result == 0)
    {
        ESP_LOGI(TAG, "sensor set_exposure_ctrl() >> enabled=1");
        result = sensor->set_exposure_ctrl(sensor, 1);
        ESP_LOGI(TAG, "sensor set_exposure_ctrl() << result=%d", result);
    }
    if (wasAuto != enabled)
    {
        // 只在真正切换模式时重建范围；重复关闭自动曝光不缩小已建立的手动范围。
        m_expMaxLines = 0;
    }
    const bool hasReadback = readExposure(sensor);
    if (result != 0 || !hasReadback || m_isAutoExposure != enabled)
    {
        // 部分写入也回读并上报失败；下一次设置以硬件现状为准，不假设操作已回滚。
        ESP_LOGW(TAG, "setAutoExposure() << result=WRITE_OR_READBACK_FAILED");
        return false;
    }
    ESP_LOGI(TAG, "setAutoExposure() << result=OK");
    return true;
}

// 输入：无；输出：无；作用：相机初始化、设置变更和帧率更新时发布小型状态快照。
void CameraWeb::cacheStatus()
{
    // 调用方已持有相机锁，或仍在单线程初始化阶段；这里只短暂持有快照锁。
    ESP_LOGI(TAG, "esp_camera_sensor_get() >> status_snapshot");
    sensor_t* sensor = esp_camera_sensor_get();
    ESP_LOGI(TAG, "esp_camera_sensor_get() << ready=%d", int(sensor != nullptr));
    if (sensor == nullptr)
    {
        // 测量模式关闭相机时清空快照，不能把历史数据描述为当前有效参数。
        std::lock_guard<std::mutex> lock(m_statusMutex);
        m_statusJson[0] = '\0';
        return;
    }
    if (esp_timer_get_time() - m_expReadUs >= 2000000)
    {
        // 复用现有状态/帧率发布节拍；HTTP 网络发送和快照锁内不访问 SCCB。
        readExposure(sensor);
    }
    const auto& state = sensor->status;
    std::lock_guard<std::mutex> lock(m_statusMutex);
    std::snprintf(m_statusJson.data(), m_statusJson.size(),
        "{\"size\":%d,\"quality\":%d,\"max_fps\":%d,\"actual_fps\":%.2f,"
        "\"brightness\":%d,\"contrast\":%d,\"saturation\":%d,"
        "\"hmirror\":%d,\"vflip\":%d,\"streaming\":%s,"
        "\"auto_exposure\":%s,\"exposure_valid\":%s,\"exposure_us\":%lld,"
        "\"exposure_min_us\":%lld,\"exposure_max_us\":%lld,\"line_us\":%.3f",
        m_size, state.quality, m_maxFps, m_actualFps,
        state.brightness, state.contrast, state.saturation, state.hmirror, state.vflip,
        m_streaming ? "true" : "false", m_isAutoExposure ? "true" : "false",
        m_isExposureValid ? "true" : "false",
        std::llround((m_expLines + m_extraLines) * m_lineUs),
        static_cast<long long>(std::ceil(m_lineUs)),
        static_cast<long long>(std::floor(m_expMaxLines * m_lineUs)), m_lineUs);
}

esp_err_t CameraWeb::sendStatus(httpd_req_t* req)
{
    {
        // 空闲时也刷新自动曝光；视频取帧占锁时立即使用快照，不让状态请求等待取帧。
        std::unique_lock<std::mutex> lock(m_mutex, std::try_to_lock);
        if (lock.owns_lock() && esp_timer_get_time() - m_expReadUs >= 2000000)
        {
            cacheStatus();
        }
    }
    char json[1024];
    std::array<char, 512> cached;
    // 只读已发布快照；相机取帧即使等待四秒，也不会阻塞本次状态读取。
    const MotorState motor = m_motor.getState();
    {
        std::lock_guard<std::mutex> lock(m_statusMutex);
        cached = m_statusJson;
    }
    if (cached[0] == '\0')
    {
        // 即使相机不可用，错误响应也在释放快照锁后发送，避免慢客户端阻塞发布。
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera not ready");
    }
    std::snprintf(json, sizeof(json),
        "%s,\"connected\":%s,\"motor\":{\"enabled\":%s,\"ready\":%s,\"moving\":%s,\"fault\":%d,"
        "\"generation\":%lu,\"forward\":%d,\"right\":%d,\"turn\":%d,\"wheels\":[%d,%d,%d,%d]}}",
        cached.data(), m_connected.load() ? "true" : "false",
        motor.enabled ? "true" : "false", motor.ready ? "true" : "false", motor.moving ? "true" : "false",
        motor.fault, static_cast<unsigned long>(motor.generation), motor.forward, motor.right, motor.turn,
        motor.wheels[0], motor.wheels[1], motor.wheels[2], motor.wheels[3]);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

// 输入：请求；输出：独立电机 JSON；作用：避免运动续租依赖相机状态读取。
esp_err_t CameraWeb::sendMotor(httpd_req_t* req)
{
    const MotorState state = m_motor.getState();
    char json[384];
    std::snprintf(json, sizeof(json),
        "{\"enabled\":%s,\"ready\":%s,\"moving\":%s,\"fault\":%d,\"generation\":%lu,"
        "\"forward\":%d,\"right\":%d,\"turn\":%d,\"wheels\":[%d,%d,%d,%d]}",
        state.enabled ? "true" : "false", state.ready ? "true" : "false",
        state.moving ? "true" : "false", state.fault,
        static_cast<unsigned long>(state.generation), state.forward, state.right, state.turn,
        state.wheels[0], state.wheels[1], state.wheels[2], state.wheels[3]);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

// 输入：三轴百分数和指令代次表单；输出：HTTP 结果；作用：严格校验后续租或停车。
esp_err_t CameraWeb::setMotor(httpd_req_t* req)
{
    char body[160] = {};
    if (req->content_len == 0 || req->content_len >= sizeof(body))
    {
        // 有问题的控制请求撤销旧运动，不能依靠旧指令继续行驶。
        m_motor.stop("invalid body");
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid motor body length");
    }
    size_t received = 0;
    while (received < req->content_len)
    {
        // 网络分段接收直到正文完整；中断立即停车，不对部分参数执行运动。
        const int count = httpd_req_recv(req, body + received, req->content_len - received);
        if (count <= 0)
        {
            m_motor.stop("incomplete command");
            return ESP_FAIL;
        }
        received += count;
    }
    const char* names[] = {"forward", "right", "turn"};
    int values[3] = {};
    for (size_t axis = 0; axis < 3; ++axis)
    {
        char text[16] = {};
        char* end = nullptr;
        errno = 0;
        const esp_err_t parsed = httpd_query_key_value(body, names[axis], text, sizeof(text));
        const long value = std::strtol(text, &end, 10);
        if (parsed != ESP_OK || errno != 0 || end == text || *end != '\0' || value < -100 || value > 100)
        {
            m_motor.stop("invalid axis");
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected axes in -100..100");
        }
        values[axis] = static_cast<int>(value);
    }
    if (values[0] == 0 && values[1] == 0 && values[2] == 0)
    {
        // 全零停车不要求代次，即使网页状态已经过期也能撤销当前运动。
        const esp_err_t result = m_motor.stop("web stop");
        if (result != ESP_OK)
        {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Motor stop unconfirmed; check hardware");
        }
        return sendMotor(req);
    }
    if (m_motorFd >= 0)
    {
        // 长连接控制期间只保留 HTTP 备用停车和只读诊断，避免两个通道同时改变方向。
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Use the active WebSocket controller");
    }
    char text[16] = {};
    char* end = nullptr;
    errno = 0;
    const esp_err_t parsed = httpd_query_key_value(body, "generation", text, sizeof(text));
    const unsigned long generation = std::strtoul(text, &end, 10);
    if (parsed != ESP_OK || text[0] == '-' || errno != 0 || end == text || *end != '\0')
    {
        m_motor.stop("invalid generation");
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected unsigned generation");
    }
    if (!m_connected.load())
    {
        // 网络接口未就绪时只能停车，不接受新运动。
        m_motor.requestStop();
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "Network unavailable");
    }
    const esp_err_t result = m_motor.move(values[0], values[1], values[2], generation);
    if (result == ESP_ERR_INVALID_STATE)
    {
        // 迟到包只拒绝，不再触发停车，防止旧包干扰已经建立的新代次。
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Motor unavailable or stale generation; release and retry");
    }
    if (result != ESP_OK)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Motor driver failed; check hardware");
    }
    return sendMotor(req);
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
    if (errno != 0 || end == text || *end != '\0' || value < INT_MIN || value > INT_MAX)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid integer");
    }
    ESP_LOGI(TAG, "setControl() >> name=%s value=%ld", name, value);
    bool applied;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        // 参数设置与取帧互斥，避免传感器寄存器在另一个任务采集时被并发修改。
        applied = applySetting(name, static_cast<int>(value));
        cacheStatus();
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
        // 测量模式可能主动停止相机，拒绝控制请求，避免访问已销毁的传感器对象。
        return false;
    }
    if (std::strcmp(name, "auto_exposure") == 0)
    {
        // 只接受明确的布尔值，切换完成后以硬件回读为准。
        return (value == 0 || value == 1) && setAutoExposure(sensor, value == 1);
    }
    if (std::strcmp(name, "exposure_us") == 0)
    {
        if (!readExposure(sensor) || m_isAutoExposure ||
            value < std::ceil(m_lineUs) || value > std::floor(m_expMaxLines * m_lineUs))
        {
            // 接口以整数微秒接收；仅开放当前时序可表达的范围，不把错误值隐式夹紧。
            return false;
        }
        const int lines = std::clamp(static_cast<int>(std::llround(value / m_lineUs)), 1, m_expMaxLines);
        return setExposure(sensor, lines);
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
        if (!readExposure(sensor))
        {
            // 保留手动曝光前先取得真实时序，避免根据历史快照进行单位转换。
            return false;
        }
        const bool wasAuto = m_isAutoExposure;
        const double keepUs = (m_expLines + m_extraLines) * m_lineUs;
        if (sensor->set_framesize(sensor, size) != 0 || applySensorClock(sensor) != ESP_OK)
        {
            return false;
        }
        m_size = value;
        m_expMaxLines = 0;
        if (!readExposure(sensor))
        {
            // 分辨率已经切换，状态仍发布真实模式，但曝光不可用时不再尝试写入寄存器。
            return false;
        }
        if (!wasAuto)
        {
            // 手动模式尽量保持切换前的毫秒值；新模式范围不足时取最近的有效行数。
            const int lines = std::clamp(static_cast<int>(std::llround(keepUs / m_lineUs)), 1, m_expMaxLines);
            return setExposure(sensor, lines);
        }
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
    cacheStatus();
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
    ESP_LOGI(TAG, "onStream() >> socket=%d wifi_connected=%d internal_free=%u",
             httpd_req_to_sockfd(req), int(app->m_connected.load()),
             unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    unsigned frames = 0;
    // 记录本次连接完整发出的帧数，区分首帧失败和持续传输后的失败。
    unsigned totalFrames = 0;
    int64_t window = 0;
    int64_t nextFrame = esp_timer_get_time();
    esp_err_t result = ESP_OK;
    // 每个预览连接复用一个 PSRAM 发送缓冲，合并边界、JPEG 和结尾，减少微小 TCP 写入。
    std::unique_ptr<uint8_t, decltype(&heap_caps_free)> packet(nullptr, heap_caps_free);
    size_t capacity = 0;
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
        int64_t captureStart;
        int64_t captureUs;
        {
            std::lock_guard<std::mutex> lock(app->m_mutex);
            maxFps = app->m_maxFps;
            if (!app->m_streaming)
            {
                // 只在开始时发布状态，不为每一帧重复格式化 JSON。
                app->m_streaming = true;
                app->cacheStatus();
            }
            captureStart = esp_timer_get_time();
            frame = esp_camera_fb_get();
            captureUs = esp_timer_get_time() - captureStart;
        }
        std::unique_ptr<camera_fb_t, decltype(&esp_camera_fb_return)>
            holder(frame, esp_camera_fb_return);
        if (frame == nullptr || frame->format != PIXFORMAT_JPEG)
        {
            ESP_LOGE(TAG, "取帧失败 capture_ms=%lld lock_wait_ms=%lld frame_present=%d",
                     (long long)(captureUs / 1000),
                     (long long)((captureStart - started) / 1000), int(frame != nullptr));
            result = ESP_FAIL;
            break;
        }
        if (totalFrames == 0)
        {
            // 首帧记录尺寸和 JPEG 起始标志，后续正常帧不逐帧刷屏。
            const bool hasSoi = frame->len >= 2 && frame->buf[0] == 0xff && frame->buf[1] == 0xd8;
            ESP_LOGI(TAG, "首帧已取得 bytes=%u width=%u height=%u jpeg_soi=%d capture_ms=%lld lock_wait_ms=%lld",
                     unsigned(frame->len), unsigned(frame->width), unsigned(frame->height), int(hasSoi),
                     (long long)(captureUs / 1000),
                     (long long)((captureStart - started) / 1000));
        }
        char header[128];
        const size_t jpegBytes = frame->len;
        const int length = std::snprintf(header, sizeof(header),
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            unsigned(jpegBytes));
        const size_t needed = length + jpegBytes + 2;
        if (needed > capacity)
        {
            // 以 4 KiB 为单位扩容并复用，避免 JPEG 长度小幅变化造成逐帧堆分配。
            const size_t nextSize = (needed + 4095) & ~size_t(4095);
            ESP_LOGI(TAG, "heap_caps_realloc() >> bytes=%u caps=PSRAM|8BIT", unsigned(nextSize));
            auto* resized = static_cast<uint8_t*>(heap_caps_realloc(
                packet.get(), nextSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            ESP_LOGI(TAG, "heap_caps_realloc() << success=%d", int(resized != nullptr));
            if (resized == nullptr)
            {
                // 原缓冲在 realloc 失败时仍归 RAII 持有；退出并归还当前相机帧。
                result = ESP_ERR_NO_MEM;
                ESP_LOGE(TAG, "onStream() << result=TX_BUFFER_NO_MEMORY bytes=%u", unsigned(nextSize));
                break;
            }
            packet.release();
            packet.reset(resized);
            capacity = nextSize;
        }
        std::memcpy(packet.get(), header, length);
        std::memcpy(packet.get() + length, frame->buf, jpegBytes);
        packet.get()[length + jpegBytes] = '\r';
        packet.get()[length + jpegBytes + 1] = '\n';
        // 网络发送只使用独立副本，提前归还采集帧，使两个缓冲能够继续按原节拍采集。
        holder.reset();
        // 网络发送不持有传感器锁，慢速客户端不会阻止网页调节参数。
        const int64_t sendStart = esp_timer_get_time();
        const char* stage = "frame_packet";
        errno = 0;
        result = httpd_resp_send_chunk(req, reinterpret_cast<const char*>(packet.get()), needed);
        if (result != ESP_OK)
        {
            // 必须先保存 errno，再查询无线状态，避免后续接口覆盖原始发送错误。
            const int sendErrno = errno;
            const int64_t sendUs = esp_timer_get_time() - sendStart;
            ESP_LOGW(TAG, "视频发送失败 stage=%s result=%s errno=%d completed_frames=%u bytes=%u send_ms=%lld",
                     stage, esp_err_to_name(result), sendErrno, totalFrames,
                     unsigned(jpegBytes), (long long)(sendUs / 1000));
#if CONFIG_CAMERA_WIFI_AP
            // AP 没有上游路由器信号强度；改为记录热点状态及当前接入设备数量。
            wifi_sta_list_t clients = {};
            const esp_err_t wifiResult = esp_wifi_ap_get_sta_list(&clients);
            ESP_LOGW(TAG, "发送失败时 AP ready=%d clients=%d list_result=%s internal_free=%u",
                     int(app->m_connected.load()), wifiResult == ESP_OK ? clients.num : -1,
                     esp_err_to_name(wifiResult), unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
#else
            wifi_ap_record_t access = {};
            const esp_err_t wifiResult = esp_wifi_sta_get_ap_info(&access);
            if (wifiResult == ESP_OK)
            {
                ESP_LOGW(TAG, "发送失败时 Wi-Fi 仍关联 rssi_dbm=%d internal_free=%u",
                         access.rssi, unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
            }
            else
            {
                ESP_LOGW(TAG, "发送失败时无法取得 Wi-Fi 状态 result=%s", esp_err_to_name(wifiResult));
            }
#endif
            break;
        }
        if (totalFrames == 0)
        {
            ESP_LOGI(TAG, "首帧发送完成 send_ms=%lld",
                     (long long)((esp_timer_get_time() - sendStart) / 1000));
        }
        ++totalFrames;
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
            app->cacheStatus();
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
    ESP_LOGI(TAG, "onStream() << result=%s completed_frames=%u",
             esp_err_to_name(result), totalFrames);
    return result;
}
