#include "arm_control.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 舵机业务日志标签，与车轮和底层总线分别定位。
static constexpr char TAG[] = "arm_control";

// 关节 0、1 使用机械臂板末端 CH14、CH15，连续寄存器同时用于设角度与释放输出。
static constexpr uint8_t ARM_FIRST_CHANNEL = 14;

// 输入：无；输出：结果；作用：初始化机械臂板，所有通道关闭，不主动移动舵机。
esp_err_t ArmControl::start()
{
    ESP_LOGI(TAG, "start() >>");
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_isStarted)
    {
        // 重复启动不能重新撤销正在承重的舵机输出。
        ESP_LOGW(TAG, "start() << result=ALREADY_STARTED");
        return ESP_ERR_INVALID_STATE;
    }
    m_isStarted = true;
#if !CONFIG_ARM_ENABLED
    ESP_LOGI(TAG, "start() << result=DISABLED");
    return ESP_OK;
#else
    m_state.enabled = true;
    m_state.minimum = {CONFIG_ARM_JOINT0_MIN, CONFIG_ARM_JOINT1_MIN};
    m_state.maximum = {CONFIG_ARM_JOINT0_MAX, CONFIG_ARM_JOINT1_MAX};
    esp_err_t result = m_bus.start();
    if (result == ESP_OK) result = m_bus.writePca(true, 0xfd, 0x10);
    uint8_t mode = 0;
    const uint8_t reg = 0;
    if (result == ESP_OK) result = m_bus.transfer(true, &reg, 1, &mode);
    if (result == ESP_OK && (mode & 0x40) != 0) result = ESP_ERR_NOT_SUPPORTED;
    // 25 MHz / (4096 * 122) 约 50.03 Hz，舵机脉宽按同一时钟换算。
    if (result == ESP_OK) result = m_bus.writePca(true, 0x00, 0x30);
    if (result == ESP_OK) result = m_bus.writePca(true, 0xfe, 121);
    if (result == ESP_OK) result = m_bus.writePca(true, 0x01, 0x04);
    // 清理十六路历史寄存器，逐通道保持 FULL_OFF，直到明确收到目标指令。
    // 0xFD 是广播写入寄存器，不是独立开关，清零会破坏这些关闭位。
    std::array<uint8_t, 65> clear{};
    clear[0] = 0x06;
    for (size_t channel = 0; channel < 16; ++channel) clear[4 + channel * 4] = 0x10;
    if (result == ESP_OK) result = m_bus.transfer(true, clear.data(), clear.size());
    if (result == ESP_OK) result = m_bus.writePca(true, 0x00, 0x20);
    if (result == ESP_OK)
    {
        // 至少等待 500 us 后重启内部振荡器，保持每通道 FULL_OFF。
        vTaskDelay(pdMS_TO_TICKS(2) + 2);
        result = m_bus.writePca(true, 0x00, 0xa0);
    }
    if (result == ESP_OK) result = m_bus.transfer(true, &reg, 1, &mode);
    if (result == ESP_OK && (mode & 0x70) != 0x20) result = ESP_ERR_INVALID_RESPONSE;
    if (result == ESP_OK)
    {
        // 初始化时所有输出仍关闭，任务只会推进已经明确设置的目标。
        TaskHandle_t task = nullptr;
        ESP_LOGI(TAG, "xTaskCreate() >> name=arm_motion stack_bytes=3072 priority=4 period_ms=20");
        const BaseType_t made = xTaskCreate(motionTask, "arm_motion", 3072, this, 4, &task);
        ESP_LOGI(TAG, "xTaskCreate() << result=%ld", static_cast<long>(made));
        if (made == pdPASS) m_task.reset(task);
        else result = ESP_ERR_NO_MEM;
    }
    m_state.fault = result;
    m_state.ready = result == ESP_OK;
    if (result == ESP_OK) ESP_LOGI(TAG, "start() << result=ESP_OK outputs=off");
    else ESP_LOGW(TAG, "start() << result=%s", esp_err_to_name(result));
    return result;
#endif
}

// 输入：关节、角度与代次；输出：结果；作用：提交平滑目标，首次未知位置只明确设置一次初始输出。
esp_err_t ArmControl::setTarget(int joint, int angle, uint32_t generation, bool relative)
{
    ESP_LOGI(TAG, "setTarget() >> joint=%d angle=%d generation=%lu relative=%d", joint, angle, (unsigned long)generation, relative);
    std::lock_guard<std::mutex> lock(m_mutex);
    esp_err_t result = ESP_ERR_NOT_SUPPORTED;
#if CONFIG_ARM_ENABLED
    if (m_moveGen != m_generation.load()) holdLocked();
    if (!m_state.ready || generation != m_generation.load()) result = ESP_ERR_INVALID_STATE;
    else if (joint < 0 || joint > 1) result = ESP_ERR_INVALID_ARG;
    else if (relative && (m_state.angles[joint] < 0 || !m_state.holding[joint] || angle == 0 || angle < -15 || angle > 15)) result = ESP_ERR_INVALID_ARG;
    else if (!relative && (angle < m_state.minimum[joint] || angle > m_state.maximum[joint])) result = ESP_ERR_INVALID_ARG;
    else
    {
        // 连续指令在最新目标上增减 15 度，只保留一个目标；松手即丢弃尚未完成的位移。
        if (relative) angle = std::clamp(m_state.targets[joint] + angle, m_state.minimum[joint], m_state.maximum[joint]);
        result = ESP_OK;
        if (m_state.angles[joint] < 0 || !m_state.holding[joint]) result = writeAngle(joint, angle);
        if (result == ESP_OK)
        {
            m_state.targets[joint] = angle;
            m_state.slewing[joint] = m_state.angles[joint] != angle;
            m_untilUs[joint] = relative ? esp_timer_get_time() + 350000 : 0;
            // 同一次按住的续租不改变代次；停止或新固定目标才使在途旧指令失效。
            if (!relative) m_generation.fetch_add(1);
            m_moveGen = m_generation.load();
        }
    }
#endif
    if (result == ESP_OK) ESP_LOGI(TAG, "setTarget() << result=ESP_OK");
    else ESP_LOGW(TAG, "setTarget() << result=%s", esp_err_to_name(result));
    return result;
}

// 输入：关节、角度；输出：传输结果；作用：持业务锁下发单个角度，保留最后确认写入值。
esp_err_t ArmControl::writeAngle(int joint, int angle)
{
    const unsigned pulseUs = 1000 + unsigned(angle) * 1000 / 180;
    const unsigned ticks = (pulseUs * 25 + 61) / 122;
    const uint8_t data[] = {uint8_t(0x06 + (ARM_FIRST_CHANNEL + joint) * 4), 0, 0, uint8_t(ticks & 0xff), uint8_t(ticks >> 8)};
    const esp_err_t result = m_bus.transfer(true, data, sizeof(data));
    if (result == ESP_OK)
    {
        m_state.angles[joint] = angle;
        m_state.holding[joint] = true;
    }
    else
    {
        // 写入失败后的实际位置未知；停止所有推进并尽力关闭机械臂 PWM。
        m_state.ready = false;
        m_state.fault = result;
        holdLocked();
        const esp_err_t stopped = m_bus.writePca(true, 0xfd, 0x10);
        if (stopped == ESP_OK) m_state.holding = {};
    }
    return result;
}

// 输入：无；输出：无；作用：锁内停止推进并同步网络代次，不改变 PWM 保持状态。
void ArmControl::holdLocked()
{
    m_state.targets = m_state.angles;
    m_state.slewing = {};
    m_untilUs = {};
    m_moveGen = m_generation.load();
}

// 输入：无；输出：结果；作用：松手时立即丢弃未完成目标，保持最后下发角度。
esp_err_t ArmControl::hold()
{
    ESP_LOGI(TAG, "hold() >>");
    std::lock_guard<std::mutex> lock(m_mutex);
    m_generation.fetch_add(1);
    holdLocked();
    const esp_err_t result = m_state.ready ? ESP_OK : ESP_ERR_INVALID_STATE;
    if (result == ESP_OK) ESP_LOGI(TAG, "hold() << result=ESP_OK");
    else ESP_LOGW(TAG, "hold() << result=%s", esp_err_to_name(result));
    return result;
}

// 输入：微秒时刻；输出：无；作用：网络作废或租约到期时停止；否则平滑推进两度。
void ArmControl::advance(int64_t nowUs)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_moveGen != m_generation.load()) holdLocked();
    if (!m_state.ready) return;
    for (int joint = 0; joint < 2; ++joint)
    {
        // 连续控制超过 350 ms 无续租即停止，固定目标则允许自行完成。
        if (m_untilUs[joint] != 0 && nowUs >= m_untilUs[joint])
        {
            m_state.targets[joint] = m_state.angles[joint];
            m_state.slewing[joint] = false;
            m_untilUs[joint] = 0;
            m_generation.fetch_add(1);
            m_moveGen = m_generation.load();
        }
        if (!m_state.slewing[joint]) continue;
        const int current = m_state.angles[joint];
        const int delta = std::clamp(m_state.targets[joint] - current, -2, 2);
        const int angle = std::clamp(current + delta, m_state.minimum[joint], m_state.maximum[joint]);
        if (writeAngle(joint, angle) != ESP_OK) return;
        m_state.slewing[joint] = angle != m_state.targets[joint];
    }
}

// 输入：实例；输出：无；作用：周期推进目标，不等待 HTTP 回报，延迟后不补发历史动作。
void ArmControl::motionTask(void* arg)
{
    auto* arm = static_cast<ArmControl*>(arg);
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
        arm->advance(esp_timer_get_time());
    }
}

// 输入：任务句柄；输出：无；作用：在共享总线析构前释放平滑推进任务。
void ArmControl::releaseTask(std::remove_pointer_t<TaskHandle_t>* task)
{
    ESP_LOGI(TAG, "vTaskDelete() >> arm_motion");
    vTaskDelete(task);
    ESP_LOGI(TAG, "vTaskDelete() <<");
}

// 输入：无；输出：结果；作用：释放两路输出，保留最后写入值并作废旧指令。
esp_err_t ArmControl::release()
{
    ESP_LOGI(TAG, "release() >>");
    std::lock_guard<std::mutex> lock(m_mutex);
    m_generation.fetch_add(1);
    holdLocked();
    esp_err_t result = ESP_ERR_NOT_SUPPORTED;
#if CONFIG_ARM_ENABLED
    const uint8_t data[] = {uint8_t(0x06 + ARM_FIRST_CHANNEL * 4), 0, 0, 0, 0x10, 0, 0, 0, 0x10};
    result = m_bus.transfer(true, data, sizeof(data));
    if (result == ESP_OK) m_state.holding = {};
    else
    {
        m_state.ready = false;
        m_state.fault = result;
    }
#endif
    if (result == ESP_OK) ESP_LOGI(TAG, "release() << result=ESP_OK");
    else ESP_LOGW(TAG, "release() << result=%s", esp_err_to_name(result));
    return result;
}

// 输入：无；输出：无；作用：网络回调只作废目标，任务下一周期停止并保留扭矩。
void ArmControl::invalidate()
{
    ESP_LOGI(TAG, "invalidate() >>");
    const uint32_t generation = m_generation.fetch_add(1) + 1;
    ESP_LOGI(TAG, "invalidate() << generation=%lu", static_cast<unsigned long>(generation));
}

// 输入：无；输出：快照；作用：同时返回目标与最后下发角度，均不是实际位置反馈。
ArmState ArmControl::getState() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    ArmState state = m_state;
    state.generation = m_generation.load();
    return state;
}
