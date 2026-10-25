#include "cloud_agent.h"
#include "camera_web.h"
#include "cloud_credentials.local.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "cJSON.h"
#include <algorithm>
#include <cstring>
#include <memory>
#include <sys/time.h>

// 云端传输日志标签；日志永不输出认证头和原始凭证。
static constexpr char TAG[] = "cloud_agent";
// 控制连接只传小消息，独立于 JPEG 上传连接。
static constexpr char CONTROL_URL[] = "wss://124.223.192.76/device/control";
// 视频保持当前 JPEG 质量与帧率上限，无转码。
static constexpr char VIDEO_URL[] = "wss://124.223.192.76/device/video";

// 输入：无；输出：UTC 毫秒；作用：校验服务器发出的短期指令期限。
static int64_t utcMilliseconds()
{
    timeval value{};
    gettimeofday(&value, nullptr);
    return int64_t(value.tv_sec) * 1000 + value.tv_usec / 1000;
}

CloudAgent& CloudAgent::instance()
{
    static CloudAgent agent;
    return agent;
}

esp_err_t CloudAgent::start()
{
    ESP_LOGI(TAG, "start() >>");
    if (m_queue) return ESP_ERR_INVALID_STATE;
    m_queue = xQueueCreate(4, sizeof(CloudPacket));
    if (!m_queue) return ESP_ERR_NO_MEM;
    TaskHandle_t task = nullptr;
    ESP_LOGI(TAG, "xTaskCreate() >> name=cloud_control stack=8192 priority=5");
    const BaseType_t result = xTaskCreate(controlTask, "cloud_control", 8192, this, 5, &task);
    ESP_LOGI(TAG, "xTaskCreate() << result=%ld", static_cast<long>(result));
    if (result != pdPASS) { vQueueDelete(m_queue); m_queue = nullptr; return ESP_ERR_NO_MEM; }
    ESP_LOGI(TAG, "start() << result=ESP_OK");
    return ESP_OK;
}

void CloudAgent::onControl(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    (void)base;
    auto* agent = static_cast<CloudAgent*>(arg);
    if (id == WEBSOCKET_EVENT_CONNECTED)
    {
        agent->m_hasControl.store(true);
        CameraWeb::instance().setCloudActive(true);
        ESP_LOGI(TAG, "onControl() << result=CONNECTED");
    }
    else if (id == WEBSOCKET_EVENT_DISCONNECTED || id == WEBSOCKET_EVENT_ERROR)
    {
        // 回调只投递停止；旧队列项因代次变化失效，不在此等待 I2C 或网络锁。
        agent->m_hasControl.store(false);
        agent->m_hasStop.store(true);
        agent->m_epoch.fetch_add(1);
        CameraWeb::instance().setCloudActive(false);
        ESP_LOGW(TAG, "onControl() << result=DISCONNECTED event=%ld", static_cast<long>(id));
    }
    else if (id == WEBSOCKET_EVENT_DATA)
    {
        const auto* event = static_cast<esp_websocket_event_data_t*>(data);
        if (event->op_code != 1 && event->op_code != 0) return;
        if (event->payload_offset == 0) agent->m_rxSize = 0;
        if (event->payload_len <= 0 || event->payload_len >= int(agent->m_rx.size()) ||
            event->data_len < 0 || event->payload_offset != int(agent->m_rxSize) ||
            agent->m_rxSize + event->data_len >= agent->m_rx.size())
        {
            agent->m_hasStop.store(true); agent->m_epoch.fetch_add(1);
            CameraWeb::instance().setCloudActive(false);
            return;
        }
        std::memcpy(agent->m_rx.data() + agent->m_rxSize, event->data_ptr, event->data_len);
        agent->m_rxSize += event->data_len;
        if (int(agent->m_rxSize) == event->payload_len)
        {
            CloudPacket packet;
            packet.epoch = agent->m_epoch.load();
            std::memcpy(packet.text.data(), agent->m_rx.data(), agent->m_rxSize);
            if (xQueueSend(agent->m_queue, &packet, 0) != pdTRUE)
            {
                // 有界队列满时拒绝旧动作继续运行，等待服务器因应答超时关闭控制页。
                agent->m_hasStop.store(true); agent->m_epoch.fetch_add(1);
                CameraWeb::instance().setCloudActive(false);
            }
        }
    }
}

void CloudAgent::onVideo(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    (void)base; (void)data;
    auto* agent = static_cast<CloudAgent*>(arg);
    if (id == WEBSOCKET_EVENT_CONNECTED) agent->m_hasVideo.store(true);
    else if (id == WEBSOCKET_EVENT_DISCONNECTED || id == WEBSOCKET_EVENT_ERROR) agent->m_hasVideo.store(false);
}

bool CloudAgent::sendText(const std::string& text)
{
    if (!m_hasControl.load()) return false;
    ESP_LOGI(TAG, "esp_websocket_client_send_text() >> bytes=%u timeout_ms=250", unsigned(text.size()));
    const int sent = esp_websocket_client_send_text(m_control, text.data(), text.size(), pdMS_TO_TICKS(250));
    ESP_LOGI(TAG, "esp_websocket_client_send_text() << bytes=%d", sent);
    if (sent == int(text.size())) return true;
    CameraWeb::instance().setCloudActive(false);
    m_hasStop.store(true);
    return false;
}

void CloudAgent::execute(const char* text)
{
    std::unique_ptr<cJSON, decltype(&cJSON_Delete)> command(cJSON_Parse(text), cJSON_Delete);
    if (!command || !cJSON_IsObject(command.get())) return;
    const auto* field = cJSON_GetObjectItemCaseSensitive(command.get(), "op");
    if (!cJSON_IsString(field)) return;
    const std::string op = field->valuestring;
    if (op == "stop")
    {
        m_lease = 0;
        CameraWeb::instance().stopCloudControl();
        return;
    }
    if (op == "open")
    {
        const auto* lease = cJSON_GetObjectItemCaseSensitive(command.get(), "lease");
        if (!cJSON_IsNumber(lease) || lease->valuedouble < 1 || lease->valuedouble > UINT32_MAX) return;
        m_lease = static_cast<uint32_t>(lease->valuedouble);
        CameraWeb::instance().openCloudControl(m_lease);
        return;
    }
    const auto* deadline = cJSON_GetObjectItemCaseSensitive(command.get(), "deadline_ms");
    // 只在校时后接收未过期的命令；迟到运动即使仍在 TCP 队列中也不会执行。
    if (!cJSON_IsNumber(deadline) || deadline->valuedouble < utcMilliseconds() ||
        deadline->valuedouble > utcMilliseconds() + 3000)
    {
        CameraWeb::instance().stopCloudControl();
        m_lease = 0;
        return;
    }
    std::unique_ptr<cJSON, decltype(&cJSON_Delete)> reply(cJSON_CreateObject(), cJSON_Delete);
    if (!reply) return;
    if (op == "motion")
    {
        const auto* lease = cJSON_GetObjectItemCaseSensitive(command.get(), "lease");
        const auto* values = cJSON_GetObjectItemCaseSensitive(command.get(), "bytes");
        if (!cJSON_IsNumber(lease) || lease->valuedouble != m_lease ||
            !cJSON_IsArray(values) || cJSON_GetArraySize(values) != 12) return;
        std::array<uint8_t, 12> bytes{};
        for (size_t index = 0; index < bytes.size(); ++index)
        {
            const auto* value = cJSON_GetArrayItem(values, index);
            if (!cJSON_IsNumber(value) || value->valuedouble < 0 || value->valuedouble > 255 ||
                value->valuedouble != value->valueint) return;
            bytes[index] = static_cast<uint8_t>(value->valueint);
        }
        const auto report = CameraWeb::instance().cloudMotion(bytes.data(), bytes.size(), m_lease);
        cJSON_AddStringToObject(reply.get(), "op", "motion_ack");
        cJSON_AddNumberToObject(reply.get(), "lease", m_lease);
        cJSON_AddStringToObject(reply.get(), "report", report.c_str());
    }
    else if (op == "http")
    {
        const auto* id = cJSON_GetObjectItemCaseSensitive(command.get(), "id");
        const auto* path = cJSON_GetObjectItemCaseSensitive(command.get(), "path");
        const auto* method = cJSON_GetObjectItemCaseSensitive(command.get(), "method");
        const auto* body = cJSON_GetObjectItemCaseSensitive(command.get(), "body");
        if (!cJSON_IsNumber(id) || !cJSON_IsString(path) || !cJSON_IsString(method) || !cJSON_IsString(body)) return;
        int status = 500;
        const auto result = CameraWeb::instance().cloudHttp(path->valuestring, method->valuestring, body->valuestring, status);
        cJSON_AddStringToObject(reply.get(), "op", "reply");
        cJSON_AddNumberToObject(reply.get(), "id", id->valuedouble);
        cJSON_AddNumberToObject(reply.get(), "status", status);
        cJSON_AddStringToObject(reply.get(), "body", result.c_str());
    }
    else return;
    std::unique_ptr<char, decltype(&cJSON_free)> encoded(cJSON_PrintUnformatted(reply.get()), cJSON_free);
    if (encoded) sendText(encoded.get());
}

void CloudAgent::controlTask(void* arg)
{
    auto* agent = static_cast<CloudAgent*>(arg);
    // STA 和 UTC 未就绪时只保留本地诊断；绝不跳过 TLS 证书时间校验。
    while (!CameraWeb::instance().isStaReady()) vTaskDelay(pdMS_TO_TICKS(1000));
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "ntp.aliyun.com");
    esp_sntp_setservername(1, "pool.ntp.org");
    esp_sntp_init();
    while (utcMilliseconds() < 1767225600000LL) vTaskDelay(pdMS_TO_TICKS(1000));
    agent->m_headers = std::string("X-Device-Key: ") + CLOUD_DEVICE_KEY + "\r\n";
    esp_websocket_client_config_t config = {};
    config.uri = CONTROL_URL;
    config.headers = agent->m_headers.c_str();
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.buffer_size = 2048;
    config.task_stack = 6144;
    config.network_timeout_ms = 1000;
    config.reconnect_timeout_ms = 3000;
    config.ping_interval_sec = 5;
    ESP_LOGI(TAG, "esp_websocket_client_init() >> control TLS verified");
    agent->m_control = esp_websocket_client_init(&config);
    ESP_LOGI(TAG, "esp_websocket_client_init() << success=%d", int(agent->m_control != nullptr));
    if (!agent->m_control) { vTaskDelete(nullptr); return; }
    esp_websocket_register_events(agent->m_control, WEBSOCKET_EVENT_ANY, onControl, agent);
    const esp_err_t started = esp_websocket_client_start(agent->m_control);
    ESP_LOGI(TAG, "esp_websocket_client_start() << result=%s", esp_err_to_name(started));
    if (started != ESP_OK) { vTaskDelete(nullptr); return; }
    TaskHandle_t video = nullptr;
    const BaseType_t made = xTaskCreate(videoTask, "cloud_video", 6144, agent, 3, &video);
    ESP_LOGI(TAG, "xTaskCreate() << name=cloud_video result=%ld", static_cast<long>(made));
    int64_t heartbeatUs = 0;
    while (true)
    {
        // 每五十毫秒检查断线通知，先停机再消费新命令；旧连接代次消息全部丢弃。
        if (agent->m_hasStop.exchange(false))
        {
            agent->m_lease = 0;
            CameraWeb::instance().stopCloudControl();
            if (agent->m_hasControl.load())
            {
                // 队列或发送异常尚未引发 SDK 断线时主动重连，避免停留在半失效连接。
                ESP_LOGI(TAG, "esp_websocket_client_stop() >> reason=protocol_reset");
                const esp_err_t stopped = esp_websocket_client_stop(agent->m_control);
                ESP_LOGI(TAG, "esp_websocket_client_stop() << result=%s", esp_err_to_name(stopped));
                const esp_err_t restarted = esp_websocket_client_start(agent->m_control);
                ESP_LOGI(TAG, "esp_websocket_client_start() << result=%s", esp_err_to_name(restarted));
            }
        }
        CloudPacket packet;
        if (xQueueReceive(agent->m_queue, &packet, pdMS_TO_TICKS(50)) == pdTRUE &&
            packet.epoch == agent->m_epoch.load() && agent->m_hasControl.load()) agent->execute(packet.text.data());
        const int64_t nowUs = esp_timer_get_time();
        if (agent->m_hasControl.load() && nowUs - heartbeatUs >= 1000000)
        {
            heartbeatUs = nowUs;
            agent->sendText("{\"op\":\"heartbeat\"}");
        }
    }
}

void CloudAgent::videoTask(void* arg)
{
    auto* agent = static_cast<CloudAgent*>(arg);
    while (!agent->m_hasControl.load()) vTaskDelay(pdMS_TO_TICKS(500));
    esp_websocket_client_config_t config = {};
    config.uri = VIDEO_URL;
    config.headers = agent->m_headers.c_str();
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.buffer_size = 4096;
    config.task_stack = 6144;
    config.network_timeout_ms = 1000;
    config.reconnect_timeout_ms = 3000;
    config.ping_interval_sec = 5;
    agent->m_video = esp_websocket_client_init(&config);
    if (!agent->m_video) { vTaskDelete(nullptr); return; }
    esp_websocket_register_events(agent->m_video, WEBSOCKET_EVENT_ANY, onVideo, agent);
    esp_websocket_client_start(agent->m_video);
    std::string jpeg;
    int64_t windowUs = esp_timer_get_time();
    unsigned frames = 0;
    double actualFps = 0;
    while (true)
    {
        if (!agent->m_hasVideo.load() || !agent->m_hasControl.load() || !CameraWeb::instance().isStaReady())
        { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        const int64_t startedUs = esp_timer_get_time();
        int fps = 10;
        if (!CameraWeb::instance().cloudCapture(jpeg, fps))
        { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
        const int64_t capturedUs = esp_timer_get_time();
        ESP_LOGI(TAG, "esp_websocket_client_send_bin() >> bytes=%u timeout_ms=500", unsigned(jpeg.size()));
        const int sent = esp_websocket_client_send_bin(agent->m_video, jpeg.data(), jpeg.size(), pdMS_TO_TICKS(500));
        ESP_LOGI(TAG, "esp_websocket_client_send_bin() << bytes=%d", sent);
        const int64_t nowUs = esp_timer_get_time();
        if (sent == int(jpeg.size()))
        {
            ++frames;
            if (nowUs - windowUs >= 1000000)
            {
                actualFps = frames * 1000000.0 / (nowUs - windowUs);
                frames = 0; windowUs = nowUs;
            }
            CameraWeb::instance().recordCloudFrame(jpeg.size(), capturedUs - startedUs, nowUs - capturedUs, actualFps);
        }
        else
        {
            // 半帧或发送超时不能继续拼接下一个 JPEG，重建视频连接并丢弃当前画面。
            esp_websocket_client_stop(agent->m_video);
            esp_websocket_client_start(agent->m_video);
        }
        const int64_t waitUs = 1000000 / std::clamp(fps, 1, 30) - (esp_timer_get_time() - startedUs);
        if (waitUs > 0) vTaskDelay(pdMS_TO_TICKS((waitUs + 999) / 1000));
    }
}
