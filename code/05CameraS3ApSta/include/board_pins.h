#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"

#if !CONFIG_IDF_TARGET_ESP32S3
#error "Project 05 requires ESP32-S3; build with IDF_TARGET=esp32s3."
#endif

// 按天线朝左、摄像头朝右的元件面定义；外设沿用旧 AI-Thinker 母座的物理孔位。
namespace board
{
// 旧上排第 5 孔 GPIO13，连接车轮 PCA9685 的 SDA。
static constexpr gpio_num_t WHEEL_SDA = GPIO_NUM_47;
// 旧上排第 2 孔 GPIO2，连接机械臂 PCA9685 的独立 SDA。
static constexpr gpio_num_t ARM_SDA = GPIO_NUM_40;
// 旧上排第 3 孔 GPIO14，两块 PCA9685 共享的软件 I2C 时钟。
static constexpr gpio_num_t PCA_SCL = GPIO_NUM_39;
// 新 SH1106 的 I2C 数据线，接上排第 1 孔；MicroSD 保持禁用。
static constexpr gpio_num_t OLED_SDA = GPIO_NUM_41;
// 新 SH1106 的 I2C 时钟线，接上排第 6 孔；GPIO43/44 留给 UART0。
static constexpr gpio_num_t OLED_SCL = GPIO_NUM_14;
// 旧下排第 4 孔 VCC 在新板变为 GPIO19；固定为输入，该孔不能继续接电源。
static constexpr gpio_num_t SOCKET_OLD_VCC = GPIO_NUM_19;
// 旧下排第 5 孔 GND 在新板变为 GPIO20；允许保留地线，但禁用原生 USB 并保持输入。
static constexpr gpio_num_t SOCKET_OLD_GND = GPIO_NUM_20;
// 新板载 WS2812 的数据脚；通过 RMT 输出白色，不能按旧 GPIO4 普通灯驱动。
static constexpr gpio_num_t RGB_LED = GPIO_NUM_48;
// 相机 XCLK 使用 ESP32-S3-EYE 布线，已通过 OV3660 持续取图验证。
static constexpr int CAM_XCLK = 15;
// 相机 SCCB 数据脚，与 PCA9685 的软件 I2C 相互独立。
static constexpr int CAM_SDA = 4;
// 相机 SCCB 时钟脚，不复用底盘 SCL。
static constexpr int CAM_SCL = 5;
// 相机最低位数据线 D0，对应传感器 Y2。
static constexpr int CAM_D0 = 11;
// 相机数据线 D1，对应传感器 Y3。
static constexpr int CAM_D1 = 9;
// 相机数据线 D2，对应传感器 Y4。
static constexpr int CAM_D2 = 8;
// 相机数据线 D3，对应传感器 Y5。
static constexpr int CAM_D3 = 10;
// 相机数据线 D4，对应传感器 Y6。
static constexpr int CAM_D4 = 12;
// 相机数据线 D5，对应传感器 Y7。
static constexpr int CAM_D5 = 18;
// 相机数据线 D6，对应传感器 Y8。
static constexpr int CAM_D6 = 17;
// 相机最高位数据线 D7，对应传感器 Y9。
static constexpr int CAM_D7 = 16;
// 相机垂直同步输入，用于区分帧边界。
static constexpr int CAM_VSYNC = 6;
// 相机行有效输入，不作为普通外设 GPIO 使用。
static constexpr int CAM_HREF = 7;
// 相机像素时钟输入，使用板内连线，不占底盘排针。
static constexpr int CAM_PCLK = 13;
}
