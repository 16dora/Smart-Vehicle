#include "camera_web.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include <cstring>
#include <cstdlib>
#include <memory>

std::string CameraWeb::cloudStatus()
{
    {
        // 空闲时也刷新自动曝光；视频取帧占锁时立即使用快照，不让状态请求等待取帧。
        std::unique_lock<std::mutex> lock(m_mutex, std::try_to_lock);
        if (lock.owns_lock() && esp_timer_get_time() - m_expReadUs >= 2000000)
        {
            cacheStatus();
        }
    }
    char json[2048];
    std::array<char, 512> cached;
    std::array<char, 512> runtime;
    bool isVideoActive;
    int64_t runtimeAgeMs;
    // 只读已发布快照；相机取帧即使等待四秒，也不会阻塞本次状态读取。
    const MotorState motor = m_motor.getState();
    {
        std::lock_guard<std::mutex> lock(m_statusMutex);
        cached = m_statusJson;
        runtime = m_runtimeJson;
        isVideoActive = m_isVideoActive;
        runtimeAgeMs = m_runtimeAtUs == 0 ? -1 : (esp_timer_get_time() - m_runtimeAtUs) / 1000;
    }
    if (cached[0] == '\0')
    {
        // 即使相机不可用，错误响应也在释放快照锁后发送，避免慢客户端阻塞发布。
        return "{}";
    }
    // 使用原子快照提供两个接口的访问状态；不暴露任何 Wi-Fi 密码。
    const bool apReady = m_apReady.load();
    const bool staReady = m_staReady.load();
    const esp_ip4_addr_t staIp = {staReady ? m_staIp.load() : 0};
    std::snprintf(json, sizeof(json),
        "%s,\"connected\":%s,\"network\":{\"mode\":\"apsta\",\"ap_ready\":%s,"
        "\"sta_connected\":%s,\"ap_ip\":\"192.168.4.1\",\"sta_ip\":\"" IPSTR "\"},\"runtime\":%s,\"runtime_age_ms\":%lld,\"video_active\":%s,\"led_ready\":%s,\"led_on\":%s,\"led_fault\":%d,"
        "\"motor\":{\"enabled\":%s,\"ready\":%s,\"moving\":%s,\"fault\":%d,"
        "\"generation\":%lu,\"forward\":%d,\"right\":%d,\"turn\":%d,\"wheels\":[%d,%d,%d,%d]}}",
        cached.data(), (apReady || staReady) ? "true" : "false",
        apReady ? "true" : "false", staReady ? "true" : "false", IP2STR(&staIp),
        runtime[0] == '\0' ? "{}" : runtime.data(), static_cast<long long>(runtimeAgeMs),
        isVideoActive ? "true" : "false",
        m_ledReady.load() ? "true" : "false", m_ledOn.load() ? "true" : "false", int(m_ledFault.load()),
        motor.enabled ? "true" : "false", motor.ready ? "true" : "false", motor.moving ? "true" : "false",
        motor.fault, static_cast<unsigned long>(motor.generation), motor.forward, motor.right, motor.turn,
        motor.wheels[0], motor.wheels[1], motor.wheels[2], motor.wheels[3]);
    return json;
}

// 输入：请求；输出：独立电机 JSON；作用：避免运动续租依赖相机状态读取。
std::string CameraWeb::cloudMotor()
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
    return json;
}

// 输入：三轴百分数和指令代次表单；输出：HTTP 结果；作用：严格校验后续租或停车。
