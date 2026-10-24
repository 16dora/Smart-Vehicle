#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include "pca9685_bus.h"

// 机械臂控制快照；角度和保持标志均为指令状态，不是位置反馈。
struct ArmState
{
    bool enabled = false; // 构建是否启用可选机械臂。
    bool ready = false; // 机械臂板是否完成配置与回读。
    esp_err_t fault = ESP_OK; // 首次初始化或输出错误，故障后需重启处理。
    uint32_t generation = 1; // 当前命令代次，拒绝旧请求。
    std::array<int, 2> angles{-1, -1}; // 最后确认写入的标称角度；-1 为未给定。
    std::array<int, 2> minimum{}; // 两个关节允许的最小标称角度，单位度。
    std::array<int, 2> maximum{}; // 两个关节允许的最大标称角度，单位度。
    std::array<bool, 2> holding{}; // 已确认启用输出的关节，不代表实际到位。
};

// 只管理舵机业务，全部总线操作经由外部共享实例串行执行。
class ArmControl
{
public:
    // 输入：固件生命周期内的总线；输出：实例；作用：借用总线，不拥有 GPIO 或车轮。
    explicit ArmControl(Pca9685Bus& bus) : m_bus(bus) {}
    ArmControl(const ArmControl&) = delete;
    ArmControl& operator=(const ArmControl&) = delete;
    // 输入：无；输出：结果；作用：关闭历史输出并配置 50 Hz，不自动回中。
    esp_err_t start();
    // 输入：关节、角度、代次；输出：结果；作用：限位校验后提交目标；relative 为真时按有符号角度增量推进，需先设置初始目标。
    esp_err_t setTarget(int joint, int angle, uint32_t generation, bool relative = false);
    // 输入：无；输出：结果；作用：关闭两路 PWM 并作废旧请求。
    esp_err_t release();
    // 输入：无；输出：无；作用：网络回调非阻塞地作废旧请求，保持关节扭矩。
    void invalidate();
    // 输入：无；输出：快照；作用：提供协议数据，不读取硬件。
    ArmState getState() const;

private:
    Pca9685Bus& m_bus; // 由 CameraWeb 拥有，生命周期长于本模块。
    mutable std::mutex m_mutex; // 保护机械臂状态与同一板的连续配置操作。
    ArmState m_state; // 仅在业务锁内读取或修改。
    std::atomic<uint32_t> m_generation{1}; // 网络事件不等待总线锁即可使旧请求失效。
    bool m_isStarted = false; // 防止重复初始化造成已设关节突然释放。
};
