#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include "esp_err.h"
#include "sdkconfig.h"
#if CONFIG_CAMERA_OLED_ENABLED
#include "driver/spi_master.h"
#endif

// 应用准备的显示数据；驱动不采集 CPU 或相机数据，也不判断曝光采样时间。
struct OledState
{
    bool isCpuValid = false; // 当前 CPU 窗口是否已有有效差值基线。
    std::array<double, 2> busyPct{}; // 两个核心忙碌率估算，单位百分数。
    double exposureMs = 0; // 相机积分时间估算，单位毫秒。
    bool isExpValid = false; // 曝光采样有效且未超过应用规定的十秒时效。
    bool isAuto = true; // 有效曝光采样时的自动曝光开关。
};

// SSD1306 SPI 显示驱动；初始化和刷新均由应用主任务串行调用。
class OledDisplay
{
public:
    OledDisplay() = default;
    OledDisplay(const OledDisplay&) = delete;
    OledDisplay& operator=(const OledDisplay&) = delete;
    // 输入：无；输出：结果；作用：配置 SPI2 和 RC 复位时序，初始化失败释放资源。
    esp_err_t start();
    // 输入：应用层准备的快照；输出：无；作用：绘制四行状态并发送完整显存。
    void update(const OledState& state);

private:
#if CONFIG_CAMERA_OLED_ENABLED
    // 输入：命令/数据标志与字节；输出：结果；作用：切换 D/C 并按 64 字节 FIFO 分块发送。
    esp_err_t write(bool isData, const uint8_t* bytes, size_t count);
    // 输入：显存、页与文本；输出：无；作用：绘制固定 6×16 字符，不引入外部字库。
    static void drawText(std::array<uint8_t, 1024>& frame, int page, const char* value);
    spi_device_handle_t m_device = nullptr; // 仅由主任务访问；空句柄表示尚未初始化或已失败。
#endif
};
