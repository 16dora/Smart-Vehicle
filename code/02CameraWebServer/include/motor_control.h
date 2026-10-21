#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <type_traits>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// HTTP 使用的电机快照；轮序为左前、右前、左后、右后，数值为有符号占空比百分数。
struct MotorState
{
    bool enabled = false; // 当前构建是否启用电机。
    bool ready = false; // PCA9685 和停车监视任务是否初始化成功。
    bool moving = false; // 是否存在有效的非零运动指令，不是实际轮速反馈。
    esp_err_t fault = ESP_OK; // 最近的驱动故障；故障后需排除接线问题并重启。
    uint32_t generation = 1; // 停车后递增，拒绝网络中迟到的旧运动请求。
    int forward = 0; // 向前为正，范围 -100 到 100。
    int right = 0; // 向右平移为正，范围 -100 到 100。
    int turn = 0; // 顺时针旋转为正，范围 -100 到 100。
    std::array<int, 4> wheels{}; // 归一化后、接线反向修正前的轮占空比。
};

// 独占 PCA9685 总线及停车任务；电机锁独立于相机锁，不占用相机 LEDC。
class MotorControl
{
public:
    MotorControl() = default;
    ~MotorControl() = default;
    MotorControl(const MotorControl&) = delete;
    MotorControl& operator=(const MotorControl&) = delete;
    // 输入：构建配置；输出：初始化结果；作用：先关闭全部输出，再启动超时停车任务。
    esp_err_t start();
    // 输入：三个有符号百分数与指令代次；输出：执行结果；作用：混控四个麦轮并续租 500 ms。
    esp_err_t move(int forward, int right, int turn, uint32_t generation);
    // 输入：停车原因；输出：关闭输出的结果；作用：立即停车并使旧代次失效。
    esp_err_t stop(const char* reason);
    // 输入：无；输出：无；作用：Wi-Fi 回调只投递停车要求，避免阻塞网络事件任务。
    void requestStop();
    // 输入：无；输出：线程安全快照；作用：提供网页状态，不读取外设。
    MotorState getState() const;

private:
    // 输入：寄存器及值；输出：I2C 结果；作用：带超时写入单个 PCA9685 寄存器。
    esp_err_t writeRegister(uint8_t reg, uint8_t value);
    // 输入：寄存器；输出：读取值及结果；作用：确认设备和 MODE1 配置。
    esp_err_t readRegister(uint8_t reg, uint8_t& value);
    // 输入：四轮占空比；输出：I2C 结果；作用：一次 STOP 更新 CH0 至 CH11。
    esp_err_t writeWheels(const std::array<int, 4>& wheels);
    // 输入：停车原因；输出：I2C 结果；作用：调用方持锁时关闭全部通道。
    esp_err_t stopLocked(const char* reason);
    // 输入：原始故障；输出：无；作用：尽力停车，锁定故障并拒绝后续运动。
    void setFault(esp_err_t result);
    // 输入：实例；输出：无；作用：每 50 ms 检查指令租约和网络停车要求。
    static void watchTask(void* arg);
    // 输入：资源句柄；输出：无；作用：RAII 释放总线、设备和监视任务。
    static void releaseBus(std::remove_pointer_t<i2c_master_bus_handle_t>* bus);
    static void releaseDevice(std::remove_pointer_t<i2c_master_dev_handle_t>* device);
    static void releaseTask(std::remove_pointer_t<TaskHandle_t>* task);

    mutable std::mutex m_mutex; // HTTP 与停车任务共享的状态和 I2C 访问锁。
    MotorState m_state; // 仅在 m_mutex 保护下访问。
    int64_t m_lastCmdUs = 0; // 最后一次有效指令的接收时刻，单位微秒。
    std::atomic<bool> m_hasStop{false}; // 网络回调投递停车；运动请求不能清除此要求。
    std::unique_ptr<std::remove_pointer_t<i2c_master_bus_handle_t>, decltype(&releaseBus)>
        m_bus{nullptr, releaseBus}; // 先创建，最后释放，避免设备引用失效总线。
    std::unique_ptr<std::remove_pointer_t<i2c_master_dev_handle_t>, decltype(&releaseDevice)>
        m_device{nullptr, releaseDevice}; // PCA9685 的独占 SDK 设备句柄。
    std::unique_ptr<std::remove_pointer_t<TaskHandle_t>, decltype(&releaseTask)>
        m_task{nullptr, releaseTask}; // 先于设备与总线释放的停车监视任务。
};
