#include "pca9685_bus.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

// 软件总线日志标签；逐引脚调试日志保留原有 DEBUG 级别。
static constexpr char TAG[] = "pca9685_bus";

// 输入：无；输出：初始化结果；作用：统一配置两个 SDA 和共享时钟，不产生业务输出。
esp_err_t Pca9685Bus::start()
{
    ESP_LOGI(TAG, "start() >>");
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_isStarted)
    {
        // 多个控制模块复用同一实例，不重新配置正在使用的总线。
        ESP_LOGI(TAG, "start() << result=%s reused=1", esp_err_to_name(m_startResult));
        return m_startResult;
    }
    m_isStarted = true;
#if CONFIG_MOTOR_ENABLED
    // 切换开漏前先释放锁存器，保持原有上电时序。
    drivePin(GPIO_NUM_13, 1);
    drivePin(GPIO_NUM_14, 1);
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << 13) | (1ULL << 14);
#if CONFIG_ARM_ENABLED
    drivePin(GPIO_NUM_2, 1);
    config.pin_bit_mask |= 1ULL << 2;
#endif
    config.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_LOGI(TAG, "gpio_config() >> mask=%llu mode=open_drain pullup=1", config.pin_bit_mask);
    m_startResult = gpio_config(&config);
    ESP_LOGI(TAG, "gpio_config() << result=%s", esp_err_to_name(m_startResult));
    m_isBusReady = m_startResult == ESP_OK;
#endif
    if (m_startResult == ESP_OK) ESP_LOGI(TAG, "start() << result=ESP_OK");
    else ESP_LOGE(TAG, "start() << result=%s", esp_err_to_name(m_startResult));
    return m_startResult;
}

// 输入：无；输出：就绪标志；作用：只读取初始化状态，不探测硬件。
bool Pca9685Bus::isReady() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_isBusReady;
}

// 输入：引脚与电平；输出：SDK 结果；作用：通过开漏输出产生 I2C 电平。
esp_err_t Pca9685Bus::drivePin(gpio_num_t pin, int level)
{
    ESP_LOGD(TAG, "gpio_set_level() >> pin=%d level=%d", int(pin), level);
    const esp_err_t result = gpio_set_level(pin, level);
    ESP_LOGD(TAG, "gpio_set_level() << result=%s", esp_err_to_name(result));
    return result;
}

// 输入：引脚；输出：电平；作用：采样时钟、数据和 ACK。
int Pca9685Bus::readPin(gpio_num_t pin)
{
    ESP_LOGD(TAG, "gpio_get_level() >> pin=%d", int(pin));
    const int level = gpio_get_level(pin);
    ESP_LOGD(TAG, "gpio_get_level() << level=%d", level);
    return level;
}

// 输入：无；输出：无；作用：提供标准模式软件时序，调度中断只会延长时钟低/高周期。
void Pca9685Bus::busDelay()
{
    ESP_LOGD(TAG, "esp_rom_delay_us() >> us=5");
    esp_rom_delay_us(5);
    ESP_LOGD(TAG, "esp_rom_delay_us() <<");
}

// 输入：无；输出：结果；作用：带截止时间等待时钟释放，支持拉伸但不无限阻塞停车。
esp_err_t Pca9685Bus::raiseClock()
{
    esp_err_t result = drivePin(GPIO_NUM_14, 1);
    while (result == ESP_OK && readPin(GPIO_NUM_14) == 0)
    {
        // 每 5 微秒重采样；包括调度等待在内，整个事务最多等待 20 ms。
        if (esp_timer_get_time() >= m_busUntilUs) return ESP_ERR_TIMEOUT;
        busDelay();
    }
    busDelay();
    return esp_timer_get_time() >= m_busUntilUs ? ESP_ERR_TIMEOUT : result;
}

// 输入：字节及方向；输出：读回值或 ACK 结果；作用：完成八位数据和第九位应答。
esp_err_t Pca9685Bus::shiftByte(uint8_t& value, bool reading)
{
    uint8_t received = 0;
    for (int bit = 7; bit >= 0; --bit)
    {
        // SDA 只在 SCL 低时变化；读模式始终释放 SDA，禁止推挽输出高电平。
        esp_err_t result = drivePin(GPIO_NUM_14, 0);
        if (result == ESP_OK) result = drivePin(m_sda, reading ? 1 : ((value >> bit) & 1));
        busDelay();
        if (result == ESP_OK) result = raiseClock();
        if (result != ESP_OK) return result;
        if (reading) received = uint8_t((received << 1) | readPin(m_sda));
    }
    esp_err_t result = drivePin(GPIO_NUM_14, 0);
    if (result == ESP_OK) result = drivePin(m_sda, 1);
    busDelay();
    // 读取仅一个字节，主机在第九位发送 NACK，然后结束事务。
    if (result == ESP_OK) result = raiseClock();
    if (result == ESP_OK && !reading && readPin(m_sda) != 0) result = ESP_ERR_NOT_FOUND;
    drivePin(GPIO_NUM_14, 0);
    busDelay();
    if (reading && result == ESP_OK) value = received;
    return result;
}

// 输入：分支、写入和可选读回；输出：结果；作用：在同一个互斥锁下执行完整 I2C 事务。
esp_err_t Pca9685Bus::transfer(bool arm, const uint8_t* data, size_t count, uint8_t* reply)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    ESP_LOGI(TAG, "transfer() >> branch=%s bytes=%u read=%d", arm ? "arm" : "wheel", unsigned(count), reply != nullptr);
    if (!m_isBusReady || count == 0)
    {
        ESP_LOGW(TAG, "transfer() << result=INVALID_STATE");
        return ESP_ERR_INVALID_STATE;
    }
    m_sda = arm ? GPIO_NUM_2 : GPIO_NUM_13;
    m_busUntilUs = esp_timer_get_time() + 20000;
    esp_err_t result = drivePin(GPIO_NUM_13, 1);
#if CONFIG_ARM_ENABLED
    if (result == ESP_OK) result = drivePin(GPIO_NUM_2, 1);
#endif
    if (result == ESP_OK) result = raiseClock();
    if (result == ESP_OK && readPin(m_sda) == 0)
    {
        // 已选分支可能在上次断电时停在应答阶段；最多九拍释放，再发 STOP。
        for (int pulse = 0; pulse < 9 && result == ESP_OK && readPin(m_sda) == 0; ++pulse)
        {
            result = drivePin(GPIO_NUM_14, 0);
            busDelay();
            if (result == ESP_OK) result = raiseClock();
        }
        if (result == ESP_OK && readPin(m_sda) == 0) result = ESP_ERR_INVALID_STATE;
    }
#if CONFIG_ARM_ENABLED
    // 非选分支必须实际为高，防止旧事务或短路中的从机被共享时钟继续推进。
    if (result == ESP_OK && readPin(arm ? GPIO_NUM_13 : GPIO_NUM_2) == 0) result = ESP_ERR_INVALID_STATE;
#endif
    const bool canStart = result == ESP_OK;
    if (canStart)
    {
        // 先发 START；另一条 SDA 保持高，因此不会收到 START 或地址。
        result = drivePin(m_sda, 0);
        busDelay();
        if (result == ESP_OK) result = drivePin(GPIO_NUM_14, 0);
    }
    uint8_t address = uint8_t((arm ? 0x40 : CONFIG_MOTOR_I2C_ADDRESS) << 1);
    if (result == ESP_OK) result = shiftByte(address, false);
    for (size_t index = 0; index < count && result == ESP_OK; ++index)
    {
        // 出错即终止，不在部分写入后继续发下一条业务数据。
        uint8_t value = data[index];
        result = shiftByte(value, false);
    }
    if (result == ESP_OK && reply != nullptr)
    {
        // 单寄存器读取使用重复 START，不释放总线锁。
        result = drivePin(m_sda, 1);
        if (result == ESP_OK) result = raiseClock();
        if (result == ESP_OK) result = drivePin(m_sda, 0);
        busDelay();
        drivePin(GPIO_NUM_14, 0);
        address |= 1;
        if (result == ESP_OK) result = shiftByte(address, false);
        if (result == ESP_OK) result = shiftByte(*reply, true);
    }
    // 无论 ACK 失败还是超时都尽力 STOP；保留原始错误，不自动重试运动数据。
    if (canStart)
    {
        drivePin(GPIO_NUM_14, 0);
        drivePin(m_sda, 0);
        busDelay();
        const esp_err_t stopped = raiseClock();
        const esp_err_t released = drivePin(m_sda, 1);
        busDelay();
        if (result == ESP_OK) result = stopped == ESP_OK ? released : stopped;
    }
    if (result == ESP_OK) ESP_LOGI(TAG, "transfer() << result=ESP_OK");
    else ESP_LOGE(TAG, "transfer() << result=%s branch=%s", esp_err_to_name(result), arm ? "arm" : "wheel");
    return result;
}

// 输入：分支、寄存器与值；输出：结果；作用：统一写入日志与软件总线访问。
esp_err_t Pca9685Bus::writePca(bool arm, uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    ESP_LOGI(TAG, "writePca() >> arm=%d reg=0x%02x value=0x%02x", arm, reg, value);
    const esp_err_t result = transfer(arm, data, sizeof(data));
    if (result == ESP_OK) ESP_LOGI(TAG, "writePca() << result=ESP_OK");
    else ESP_LOGE(TAG, "writePca() << result=%s", esp_err_to_name(result));
    return result;
}

