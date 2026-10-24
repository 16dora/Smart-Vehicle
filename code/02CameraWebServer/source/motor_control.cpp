#include "motor_control.h"
#include <algorithm>
#include <cstdlib>
#include "esp_log.h"
#include "esp_timer.h"

// 电机驱动日志标签，日志不进入 HTTP 协议正文。
static constexpr char TAG[] = "motor_control";
// 浏览器每 100 ms 续期；超过 500 ms 没有有效指令便关闭输出。
static constexpr int64_t LEASE_US = 500000;
#if CONFIG_MOTOR_ENABLED
// 单次软件 I2C 事务最多等待 20 ms，避免断线长期占用控制请求。
static constexpr int I2C_TIMEOUT_MS = 20;
// PCA9685 的四轮方向修正位，bit0 至 bit3 对应左前、右前、左后、右后。
static constexpr unsigned INVERT_MASK = CONFIG_MOTOR_INVERT_MASK;
#endif

// 输入：寄存器与值；输出：结果；作用：仅访问车轮数据分支。
esp_err_t MotorControl::writeRegister(uint8_t reg, uint8_t value)
{
    return m_bus.writePca(false, reg, value);
}

// 输入：寄存器；输出：值与结果；作用：只在车轮分支回读配置。
esp_err_t MotorControl::readRegister(uint8_t reg, uint8_t& value)
{
    ESP_LOGI(TAG, "readRegister() >> reg=0x%02x", reg);
    const esp_err_t result = m_bus.transfer(false, &reg, 1, &value);
    if (result == ESP_OK) ESP_LOGI(TAG, "readRegister() << result=ESP_OK value=0x%02x", value);
    else ESP_LOGE(TAG, "readRegister() << result=%s", esp_err_to_name(result));
    return result;
}

// 输入：构建配置；输出：初始化结果；作用：使输出先进入关闭状态，随后允许网页控制。
esp_err_t MotorControl::start()
{
    ESP_LOGI(TAG, "start() >>");
#if !CONFIG_MOTOR_ENABLED
    ESP_LOGI(TAG, "start() << result=DISABLED");
    return ESP_OK;
#else
    std::lock_guard<std::mutex> lock(m_mutex);
    m_state.enabled = true;
    if (m_isStarted)
    {
        // 固件生命周期只初始化一次，禁止重复创建停车任务。
        ESP_LOGW(TAG, "start() << result=ALREADY_STARTED");
        return ESP_ERR_INVALID_STATE;
    }
    m_isStarted = true;
    // 总线初始化由各模块幂等复用，车轮不再承担机械臂启动职责。
    esp_err_t result = m_bus.start();
    // 即使上一次固件仍在输出 PWM，也先通过全通道关闭位撤销它。
    if (result == ESP_OK) result = writeRegister(0xfd, 0x10);
    uint8_t mode = 0;
    if (result == ESP_OK) result = readRegister(0x00, mode);
    if (result == ESP_OK && (mode & 0x40) != 0)
    {
        // EXTCLK 是粘滞位；本接线没有外部振荡器，需断电解除已有外部时钟配置。
        ESP_LOGE(TAG, "Unexpected EXTCLK; power-cycle PCA9685 before use");
        result = ESP_ERR_NOT_SUPPORTED;
    }
    // MODE1: 睡眠时设置预分频；25 MHz / (4096 * (5+1)) 约为 1017 Hz。
    if (result == ESP_OK) result = writeRegister(0x00, 0x30);
    if (result == ESP_OK) result = writeRegister(0xfe, 5);
    // MODE2: 推挽、非反相、在 STOP 时统一更新输出，方向线使用 FULL_ON/FULL_OFF。
    if (result == ESP_OK) result = writeRegister(0x01, 0x04);
    if (result == ESP_OK) result = writeWheels({0, 0, 0, 0});
    if (result == ESP_OK) result = writeRegister(0x00, 0x20);
    if (result == ESP_OK)
    {
        // 振荡器要求至少 500 us；多留一个 tick 避免紧邻 tick 边界时实际等待过短。
        vTaskDelay(pdMS_TO_TICKS(2) + 2);
        result = writeRegister(0x00, 0xa0);
    }
    if (result == ESP_OK) result = readRegister(0x00, mode);
    if (result == ESP_OK && (mode & 0x70) != 0x20) result = ESP_ERR_INVALID_RESPONSE;
    if (result == ESP_OK)
    {
        TaskHandle_t task = nullptr;
        ESP_LOGI(TAG, "xTaskCreate() >> name=motor_watch stack_bytes=3072 priority=4 poll_ms=50");
        const BaseType_t made = xTaskCreate(watchTask, "motor_watch", 3072, this, 4, &task);
        ESP_LOGI(TAG, "xTaskCreate() << result=%ld", static_cast<long>(made));
        if (made == pdPASS) m_task.reset(task);
        else result = ESP_ERR_NO_MEM;
    }
    if (result != ESP_OK)
    {
        // 外设缺失时保留相机可用；记录电机故障并尽力保持输出关闭。
        setFault(result);
        ESP_LOGE(TAG, "start() << result=%s", esp_err_to_name(result));
        return result;
    }
    m_state.ready = true;
    ESP_LOGI(TAG, "start() << result=ESP_OK pwm_hz=1017 lease_ms=500 invert_mask=%u", INVERT_MASK);
    return ESP_OK;
#endif
}

// 输入：四轮逻辑占空比；输出：写结果；作用：依照 root index.html 映射 PWM/IN1/IN2。
esp_err_t MotorControl::writeWheels(const std::array<int, 4>& wheels)
{
#if CONFIG_MOTOR_ENABLED
    std::array<uint8_t, 49> data{};
    data[0] = 0x06;
    for (size_t wheel = 0; wheel < wheels.size(); ++wheel)
    {
        // 每轮占三通道；方向反向只修正电气输出，不改变网页的逻辑轮序。
        const int value = (INVERT_MASK & (1U << wheel)) ? -wheels[wheel] : wheels[wheel];
        const int levels[] = {std::abs(value), value > 0 ? 100 : 0, value < 0 ? 100 : 0};
        for (size_t line = 0; line < 3; ++line)
        {
            const size_t pos = 1 + (wheel * 3 + line) * 4;
            if (levels[line] == 0)
            {
                data[pos + 3] = 0x10; // FULL_OFF 确保零速和低电平方向线恒低。
            }
            else if (levels[line] == 100)
            {
                data[pos + 1] = 0x10; // FULL_ON 确保方向高电平不受 PWM 频率影响。
            }
            else
            {
                const unsigned ticks = levels[line] * 4096U / 100U;
                data[pos + 2] = ticks & 0xff;
                data[pos + 3] = (ticks >> 8) & 0x0f;
            }
        }
    }
    ESP_LOGI(TAG, "transfer() >> address=0x%02x reg=0x06 bytes=49 wheels=%d,%d,%d,%d invert=%u timeout_ms=%d",
             CONFIG_MOTOR_I2C_ADDRESS, wheels[0], wheels[1], wheels[2], wheels[3], INVERT_MASK, I2C_TIMEOUT_MS);
    // 每通道数据已明确写入 FULL_OFF/FULL_ON 和完整计数，直接覆盖停车时的广播值。
    // 不能再清零 ALL_LED_OFF_H（0xFD），否则会广播覆盖各通道的 PWM 计数高位。
    const esp_err_t result = m_bus.transfer(false, data.data(), data.size());
    if (result == ESP_OK) ESP_LOGI(TAG, "transfer() << result=ESP_OK");
    else ESP_LOGE(TAG, "transfer() << result=%s", esp_err_to_name(result));
    return result;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

// 输入：三轴百分数和代次；输出：执行结果；作用：X 型麦轮混控并刷新指令期限。
esp_err_t MotorControl::move(int forward, int right, int turn, uint32_t generation)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_state.ready || m_hasStop.load() || generation != m_state.generation)
    {
        // 停车后旧网络包不能重新启动电机，必须重新取得当前代次。
        return ESP_ERR_INVALID_STATE;
    }
    const int64_t gapUs = esp_timer_get_time() - m_lastCmdUs;
    if (m_state.moving && gapUs >= LEASE_US)
    {
        // 监视任务尚未被调度时也不允许迟到的请求续期，保证超时后旧代次立即失效。
        ESP_LOGW(TAG, "Command expired at receive gap_ms=%lld generation=%lu",
                 static_cast<long long>(gapUs / 1000), static_cast<unsigned long>(generation));
        stopLocked("command timeout");
        return ESP_ERR_INVALID_STATE;
    }
    if (m_state.moving && gapUs >= LEASE_US / 2)
    {
        // 只记录明显延迟的有效指令；正常 10 Hz 续期不增加串口负担。
        ESP_LOGW(TAG, "Command renew late gap_ms=%lld generation=%lu",
                 static_cast<long long>(gapUs / 1000), static_cast<unsigned long>(generation));
    }
    if (forward < -100 || forward > 100 || right < -100 || right > 100 || turn < -100 || turn > 100)
    {
        // 防止非 HTTP 调用方绕过输入边界；范围判断先于混控计算。
        return ESP_ERR_INVALID_ARG;
    }
    if (forward == 0 && right == 0 && turn == 0) return stopLocked("zero command");
    std::array<int, 4> wheels = {forward + right + turn, forward - right - turn,
                                forward - right + turn, forward + right - turn};
    int peak = 100;
    for (int value : wheels) peak = std::max(peak, std::abs(value));
    for (int& value : wheels) value = value * 100 / peak;
    esp_err_t result = ESP_OK;
    if (wheels != m_state.wheels)
    {
        ESP_LOGI(TAG, "move() >> forward=%d right=%d turn=%d generation=%lu",
                 forward, right, turn, static_cast<unsigned long>(generation));
        bool reversing = false;
        for (size_t wheel = 0; wheel < wheels.size(); ++wheel)
        {
            if (wheels[wheel] * m_state.wheels[wheel] < 0) reversing = true;
        }
        if (reversing)
        {
            // 任一轮反转时先撤销全部 PWM，等待至少一 PWM 周期后再改变方向。
            result = writeRegister(0xfd, 0x10);
            if (result == ESP_OK) vTaskDelay(pdMS_TO_TICKS(2) + 2);
        }
        if (result == ESP_OK) result = writeWheels(wheels);
        if (result != ESP_OK)
        {
            setFault(result);
            ESP_LOGE(TAG, "move() << result=%s", esp_err_to_name(result));
            return result;
        }
        ESP_LOGI(TAG, "move() << result=ESP_OK wheels=%d,%d,%d,%d",
                 wheels[0], wheels[1], wheels[2], wheels[3]);
    }
    // 相同指令只续租，不重复写外设；防止按住按钮时增加总线和串口负担。
    m_state.forward = forward;
    m_state.right = right;
    m_state.turn = turn;
    m_state.wheels = wheels;
    m_state.moving = true;
    m_lastCmdUs = esp_timer_get_time();
    return ESP_OK;
}

// 输入：原因；输出：关闭结果；作用：即使外设不可用也使旧指令代次失效。
esp_err_t MotorControl::stopLocked(const char* reason)
{
    ESP_LOGI(TAG, "stop() >> reason=%s", reason);
    ++m_state.generation;
    const esp_err_t result = m_bus.isReady() ? writeRegister(0xfd, 0x10) : ESP_OK;
    m_state.forward = 0;
    m_state.right = 0;
    m_state.turn = 0;
    m_state.wheels = {};
    m_state.moving = false;
    if (result != ESP_OK)
    {
        // I2C 写失败时无法保证物理停车，网页必须明确显示故障而不能继续运动。
        m_state.ready = false;
        m_state.fault = result;
        ESP_LOGE(TAG, "stop() << result=%s physical_stop=UNCONFIRMED", esp_err_to_name(result));
    }
    else ESP_LOGI(TAG, "stop() << result=ESP_OK generation=%lu",
                  static_cast<unsigned long>(m_state.generation));
    return result;
}

// 输入：原因；输出：关闭结果；作用：串行化 HTTP 停车与监视任务停车。
esp_err_t MotorControl::stop(const char* reason)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return stopLocked(reason);
}

// 输入：原始结果；输出：无；作用：尽力停车后保留第一次故障。
void MotorControl::setFault(esp_err_t result)
{
    stopLocked("driver fault");
    m_state.ready = false;
    m_state.fault = result;
}

// 输入：无；输出：无；作用：Wi-Fi 事件不能被慢速 I2C 操作阻塞。
void MotorControl::requestStop()
{
    m_hasStop.store(true);
}

// 输入：无；输出：状态副本；作用：HTTP 不直接访问电机内部共享数据。
MotorState MotorControl::getState() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_state;
}

// 输入：实例；输出：无；作用：独立任务每 50 ms 检查，停车不依赖浏览器或 HTTP 空闲。
void MotorControl::watchTask(void* arg)
{
    auto* motor = static_cast<MotorControl*>(arg);
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        std::lock_guard<std::mutex> lock(motor->m_mutex);
        // 先消费网络停车，再检查期限；停车仅触发一次，不在故障下无限重试。
        if (motor->m_hasStop.exchange(false))
        {
            // 网络事件只撤销车轮运动，其他模块由应用层分别通知。
            motor->stopLocked("network disconnected");
        }
        else if (motor->m_state.moving && esp_timer_get_time() - motor->m_lastCmdUs >= LEASE_US)
        {
            ESP_LOGW(TAG, "Command timeout age_ms=%lld lease_ms=500 generation=%lu",
                     static_cast<long long>((esp_timer_get_time() - motor->m_lastCmdUs) / 1000),
                     static_cast<unsigned long>(motor->m_state.generation));
            motor->stopLocked("command timeout");
        }
    }
}

// 输入：任务句柄；输出：无；作用：销毁模块前释放停车监视任务。
void MotorControl::releaseTask(std::remove_pointer_t<TaskHandle_t>* task)
{
    ESP_LOGI(TAG, "vTaskDelete() >> motor_watch");
    vTaskDelete(task);
    ESP_LOGI(TAG, "vTaskDelete() <<");
}
