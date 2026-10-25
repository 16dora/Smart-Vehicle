#include "camera_web.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>

// 云端业务和原本地 HTTP 模块分开记录，不输出设备凭证。
static constexpr char TAG[] = "cloud_controls";

// 输入：表单及字段；输出：严格十进制值；作用：不接受缺失、截断和溢出参数。
static bool readNumber(const std::string& body, const char* key, int& value)
{
    char text[24] = {};
    if (httpd_query_key_value(body.c_str(), key, text, sizeof(text)) != ESP_OK || text[0] == '\0') return false;
    char* end = nullptr;
    errno = 0;
    const long number = std::strtol(text, &end, 10);
    if (errno || *end != '\0' || number < INT_MIN || number > INT_MAX) return false;
    value = static_cast<int>(number);
    return true;
}

void CameraWeb::setCloudActive(bool active)
{
    m_cloudActive.store(active);
    // 回调不能阻塞 I2C；停车任务处理原子请求并保持机械臂输出。
    if (!active) requestMotionStop();
}

void CameraWeb::openCloudControl(uint32_t lease)
{
    ESP_LOGI(TAG, "openCloudControl() >> lease=%lu", static_cast<unsigned long>(lease));
    m_motor.stop("new cloud controller");
    m_arm.hold();
    m_cloudLease = lease;
    m_cloudSeq = 0;
    ESP_LOGI(TAG, "openCloudControl() << result=ESP_OK");
}

void CameraWeb::stopCloudControl()
{
    ESP_LOGI(TAG, "stopCloudControl() >>");
    m_cloudLease = 0;
    m_cloudSeq = 0;
    m_motor.stop("cloud controller stopped");
    m_arm.hold();
    ESP_LOGI(TAG, "stopCloudControl() << result=ESP_OK");
}

std::string CameraWeb::cloudArm(uint32_t sequence, esp_err_t result)
{
    char text[512];
    const int length = formatArm(text, sizeof(text), m_arm.getState(), sequence, result);
    return length > 0 && length < static_cast<int>(sizeof(text)) ? std::string(text, length) : "{}";
}

std::string CameraWeb::cloudMotion(const uint8_t* data, size_t size, uint32_t lease)
{
    esp_err_t result = ESP_ERR_INVALID_ARG;
    uint32_t sequence = 0;
    uint32_t generation = 0;
    if (size != 12 || data[0] > 8 || lease == 0 || lease != m_cloudLease || !m_staReady.load())
    {
        requestMotionStop();
        return "{}";
    }
    for (size_t index = 0; index < 4; ++index)
    {
        // 协议沿用当前网页的大端代次和序号，不能用宿主字节序直接转换。
        generation = (generation << 8) | data[4 + index];
        sequence = (sequence << 8) | data[8 + index];
    }
    if (sequence == 0 || sequence <= m_cloudSeq)
    {
        requestMotionStop();
        return "{}";
    }
    m_cloudSeq = sequence;
    if (data[0] <= 2 && data[1] <= 200 && data[2] <= 200 && data[3] <= 200)
    {
        if (data[0] == 0) result = ESP_OK;
        else if (data[0] == 2) result = m_motor.stop("cloud stop");
        else result = m_motor.move(int(data[1]) - 100, int(data[2]) - 100, int(data[3]) - 100, generation);
    }
    else if (data[0] >= 3 && data[1] <= 1 && data[3] == 0)
    {
        // 停止和查询无关节参数；角度及增量还需通过业务模块的软件限位校验。
        if ((data[0] == 5 || data[0] == 7 || data[0] == 8) && (data[1] != 0 || data[2] != 0)) result = ESP_ERR_INVALID_ARG;
        else if (data[0] == 5) result = m_arm.hold();
        else if (data[0] == 7) result = m_arm.release();
        else if (data[0] == 8) result = ESP_OK;
        else if ((data[0] == 3 || data[0] == 4) && data[2] >= 1 && data[2] <= 15)
            result = m_arm.setTarget(data[1], data[0] == 4 ? -int(data[2]) : int(data[2]), generation, true);
        else if (data[0] == 6 && data[2] <= 180)
            result = m_arm.setTarget(data[1], data[2], generation);
        return cloudArm(sequence, result);
    }
    const auto motor = m_motor.getState();
    char text[160];
    std::snprintf(text, sizeof(text), "{\"e\":%d,\"r\":%d,\"m\":%d,\"f\":%d,\"g\":%lu,\"s\":%lu,\"c\":%d}",
        int(motor.enabled), int(motor.ready), int(motor.moving), motor.fault,
        static_cast<unsigned long>(motor.generation), static_cast<unsigned long>(sequence), result);
    return text;
}

std::string CameraWeb::cloudHttp(const std::string& path, const std::string& method,
                               const std::string& body, int& status)
{
    ESP_LOGI(TAG, "cloudHttp() >> path=%s method=%s", path.c_str(), method.c_str());
    status = 200;
    std::string result;
    if (method == "GET" && path == "/status") result = cloudStatus();
    else if (method == "GET" && path == "/motor") result = cloudMotor();
    else if (method == "GET" && path == "/arm") result = cloudArm();
    else if (method == "POST" && path == "/motor")
    {
        int forward = 1, right = 1, turn = 1;
        if (readNumber(body, "forward", forward) && readNumber(body, "right", right) &&
            readNumber(body, "turn", turn) && forward == 0 && right == 0 && turn == 0)
        {
            m_motor.stop("cloud HTTP fallback stop");
            result = cloudMotor();
        }
        else status = 400;
    }
    else if (method == "POST" && path == "/control")
    {
        char name[32] = {};
        int value = 0;
        if (httpd_query_key_value(body.c_str(), "var", name, sizeof(name)) != ESP_OK || !readNumber(body, "val", value)) status = 400;
        else
        {
            // 与本地相机设置共用锁，发送视频时不持锁等待网络。
            std::lock_guard<std::mutex> lock(m_mutex);
            if (!applySetting(name, value)) status = 400;
            else cacheStatus();
        }
        if (status == 200) result = cloudStatus();
    }
    else if (method == "POST" && path == "/led")
    {
        if (body != "enabled=0" && body != "enabled=1") status = 400;
        else if (!m_ledReady.load()) status = 503;
        else
        {
            const bool enabled = body == "enabled=1";
            ESP_LOGI(TAG, "gpio_set_level() >> gpio=4 enabled=%d", int(enabled));
            const esp_err_t wrote = gpio_set_level(GPIO_NUM_4, enabled);
            ESP_LOGI(TAG, "gpio_set_level() << result=%s", esp_err_to_name(wrote));
            m_ledFault.store(wrote);
            if (wrote == ESP_OK) m_ledOn.store(enabled);
            else { m_ledReady.store(false); status = 500; }
            result = cloudStatus();
        }
    }
    else status = 404;
    if (result.empty()) result = "{\"message\":\"Device request rejected\"}";
    ESP_LOGI(TAG, "cloudHttp() << status=%d", status);
    return result;
}

bool CameraWeb::cloudCapture(std::string& jpeg, int& fps)
{
    std::unique_lock<std::mutex> lock(m_mutex, std::try_to_lock);
    if (!lock.owns_lock() || !m_cloudActive.load() || !m_staReady.load()) return false;
    fps = m_maxFps;
    ESP_LOGI(TAG, "esp_camera_fb_get() >> cloud_video");
    camera_fb_t* frame = esp_camera_fb_get();
    ESP_LOGI(TAG, "esp_camera_fb_get() << success=%d bytes=%u", int(frame != nullptr), frame ? unsigned(frame->len) : 0);
    if (!frame) return false;
    if (frame->format == PIXFORMAT_JPEG && frame->len <= 524288) jpeg.assign(reinterpret_cast<char*>(frame->buf), frame->len);
    else jpeg.clear();
    // 网络发送只使用独立副本，SDK 帧缓冲在进入网络操作前归还。
    ESP_LOGI(TAG, "esp_camera_fb_return() >>");
    esp_camera_fb_return(frame);
    ESP_LOGI(TAG, "esp_camera_fb_return() <<");
    return !jpeg.empty();
}

void CameraWeb::recordCloudFrame(size_t bytes, int64_t captureUs, int64_t sendUs, double fps)
{
    {
        std::lock_guard<std::mutex> lock(m_statusMutex);
        m_videoBytes += bytes; ++m_videoFrames;
        m_captureUs += captureUs; m_sendUs += sendUs; m_isVideoActive = true;
    }
    std::unique_lock<std::mutex> lock(m_mutex, std::try_to_lock);
    if (lock.owns_lock())
    {
        m_streaming = true; m_actualFps = fps;
        if (esp_timer_get_time() - m_expReadUs >= 2000000) cacheStatus();
    }
}
