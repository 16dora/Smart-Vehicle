#include "arm_control.h"
#include "esp_log.h"
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
    m_state.fault = result;
    m_state.ready = result == ESP_OK;
    if (result == ESP_OK) ESP_LOGI(TAG, "start() << result=ESP_OK outputs=off");
    else ESP_LOGW(TAG, "start() << result=%s", esp_err_to_name(result));
    return result;
#endif
}

// 输入：关节、角度与代次；输出：结果；作用：严格拒绝越界及过期命令，只控制目标通道。
esp_err_t ArmControl::setTarget(int joint, int angle, uint32_t generation, bool relative)
{
    ESP_LOGI(TAG, "setTarget() >> joint=%d angle=%d generation=%lu relative=%d", joint, angle, (unsigned long)generation, relative);
    std::lock_guard<std::mutex> lock(m_mutex);
    esp_err_t result = ESP_ERR_NOT_SUPPORTED;
#if CONFIG_ARM_ENABLED
    if (!m_state.ready || generation != m_generation.load()) result = ESP_ERR_INVALID_STATE;
    else if (joint < 0 || joint > 1) result = ESP_ERR_INVALID_ARG;
    else if (relative && (m_state.angles[joint] < 0 || angle == 0 || angle < -15 || angle > 15)) result = ESP_ERR_INVALID_ARG;
    else if (!relative && (angle < m_state.minimum[joint] || angle > m_state.maximum[joint])) result = ESP_ERR_INVALID_ARG;
    else
    {
        // 增量在业务锁内基于最后确认目标计算，到软件边界后保持边界角度。
        if (relative)
        {
            angle += m_state.angles[joint];
            if (angle < m_state.minimum[joint]) angle = m_state.minimum[joint];
            if (angle > m_state.maximum[joint]) angle = m_state.maximum[joint];
        }
        // 角度为标称目标；1000～2000 us 对应 0～180 度，实际行程需卸载后校准。
        const unsigned pulseUs = 1000 + unsigned(angle) * 1000 / 180;
        const unsigned ticks = (pulseUs * 25 + 61) / 122;
        const uint8_t data[] = {uint8_t(0x06 + (ARM_FIRST_CHANNEL + joint) * 4), 0, 0, uint8_t(ticks & 0xff), uint8_t(ticks >> 8)};
        m_generation.fetch_add(1);
        result = m_bus.transfer(true, data, sizeof(data));
        if (result == ESP_OK)
        {
            m_state.angles[joint] = angle;
            m_state.holding[joint] = true;
        }
        else
        {
            // 部分写入后的实际输出未知；尽力关闭机械臂，不影响另一块车轮板的配置。
            m_bus.writePca(true, 0xfd, 0x10);
            m_state.ready = false;
            m_state.fault = result;
        }
    }
#endif
    if (result == ESP_OK) ESP_LOGI(TAG, "setTarget() << result=ESP_OK");
    else ESP_LOGW(TAG, "setTarget() << result=%s", esp_err_to_name(result));
    return result;
}

// 输入：无；输出：结果；作用：仅关闭机械臂两路输出，保留已知目标角度并作废旧指令。
esp_err_t ArmControl::release()
{
    ESP_LOGI(TAG, "release() >>");
    std::lock_guard<std::mutex> lock(m_mutex);
    m_generation.fetch_add(1);
    esp_err_t result = ESP_ERR_NOT_SUPPORTED;
#if CONFIG_ARM_ENABLED
    // 从 CH14 起连续关闭两个通道，与设角度使用同一映射。
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


// 输入：无；输出：无；作用：断网只作废迟到请求，不解除承重关节的保持力矩。
void ArmControl::invalidate()
{
    ESP_LOGI(TAG, "invalidate() >>");
    const uint32_t generation = m_generation.fetch_add(1) + 1;
    ESP_LOGI(TAG, "invalidate() << generation=%lu", static_cast<unsigned long>(generation));
}

// 输入：无；输出：状态副本；作用：锁内复制目标，再附上当前原子代次。
ArmState ArmControl::getState() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    ArmState state = m_state;
    state.generation = m_generation.load();
    return state;
}
