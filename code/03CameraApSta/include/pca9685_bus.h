#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include "driver/gpio.h"
#include "sdkconfig.h"

// 两个独立 SDA 分支共用 GPIO14；总线只负责硬件事务，不持有车轮或机械臂状态。
class Pca9685Bus
{
public:
    Pca9685Bus() = default;
    Pca9685Bus(const Pca9685Bus&) = delete;
    Pca9685Bus& operator=(const Pca9685Bus&) = delete;
    // 输入：构建配置；输出：初始化结果；作用：只初始化一次共享开漏引脚，重复调用返回原结果。
    esp_err_t start();
    // 输入：无；输出：是否就绪；作用：供模块在尚未初始化时跳过外设访问。
    bool isReady() const;
    // 输入：机械臂分支标志、写缓冲、可选单字节读回；输出：结果；作用：独占共享时钟完成一次事务。
    esp_err_t transfer(bool arm, const uint8_t* data, size_t count, uint8_t* reply = nullptr);
    // 输入：分支、寄存器与值；输出：结果；作用：发送单寄存器写入。
    esp_err_t writePca(bool arm, uint8_t reg, uint8_t value);

private:
    // 输入：引脚、电平；输出：SDK 结果；作用：产生开漏低电平或释放电平。
    static esp_err_t drivePin(gpio_num_t pin, int level);
    // 输入：引脚；输出：电平；作用：采样数据、ACK 与时钟拉伸。
    static int readPin(gpio_num_t pin);
    // 输入：无；输出：无；作用：每半周期至少等待 5 微秒。
    static void busDelay();
    // 输入：无；输出：结果；作用：在事务截止时间内等待 SCL 释放。
    esp_err_t raiseClock();
    // 输入：字节、读取标志；输出：读值或 ACK 结果；作用：完成八位与第九位应答。
    esp_err_t shiftByte(uint8_t& value, bool reading);

    mutable std::mutex m_mutex; // 初始化和所有分支事务唯一使用的总线锁，不跨业务模块回调。
    bool m_isStarted = false; // 是否已经尝试配置 GPIO，失败后也不隐式反复初始化。
    bool m_isBusReady = false; // 共享 GPIO 是否成功初始化。
    esp_err_t m_startResult = ESP_OK; // 首次初始化的结果，供后续模块复用。
    gpio_num_t m_sda = GPIO_NUM_13; // 当前持锁事务选择的数据分支。
    int64_t m_busUntilUs = 0; // 单次事务截止时间，单位单调微秒。
};
