# 05 ESP32-S3-N16R8 CAM＋OV3660 AP＋STA 小车

独立 ESP-IDF 5.5.5 / CMake / C++20 工程，复制自当前工作区的 `03CameraApSta`，不引用 03 的源码、配置或构建目录。默认目标固定为 `esp32s3`。

**当前临时诊断状态：OLED 已关闭，COM3/UART0 以 115200 输出启动和应用日志；直接 PSRAM DMA 已关闭，8 MB PSRAM 继续用于帧缓冲。已烧录并实测相机、热点、STA、网页、抓拍和 5 帧 MJPEG 正常。**

## 硬件依据与验证边界

- 用户提供的开发板元件面照片：`ESP32-S3-CAM` 丝印，左右各八针；相机排线为 OV3660 标识。
- 2026-10-28 经 COM3、esptool 4.12.0 实际读取：ESP32-S3 QFN56 revision v0.2，16 MB Flash、Quad、3.3 V；芯片信息识别 8 MB Embedded PSRAM AP_3v3、40 MHz 晶振。COM3 为 CH340 USB-UART。
- 本工程配置 16 MB QIO Flash、8 MB Octal PSRAM，Flash/PSRAM 均为 80 MHz。8 MB PSRAM 已在实际启动中识别，80 MHz 初始化和 SPI SRAM memory test 通过。
- 外接针位按照片及现有接线图比对。ESP32-S3-EYE 相机映射已通过 OV3660 PID=0x3660、VGA 抓拍和 5 帧 MJPEG 完整解码验证。GPIO48 WS2812 指令发送成功，灯珠实际发光仍需目视确认。
- 已上板验证相机和网络；OLED 当前禁用，底盘和机械臂未连接，不能据此宣称这些外设已验证。

## 功能与项目 03 的关系

保留当前 03 的 AP＋STA、STA 重连、80/81/82 三个 HTTP 服务、单查看者 MJPEG、单张 JPEG、图像设置、自动/手动毫秒曝光、运行参数、OLED、四轮麦克纳姆运动、两路机械臂及停止/断线保护。网页保留当前 03 的标签页、键盘映射、触摸操作、45% 速度初值和状态浮层；仅将“补光灯”改称“板载灯”。不接入项目 04 云端。

- 默认 VGA 640×480、JPEG 压缩值 12，发送上限 30 FPS，与当前 03 源码默认值一致；实际上限不代表采集或网络能够达到该帧率。
- 保留 03 的六档分辨率（160×120～1600×1200），没有为 OV3660 额外添加 2048×1536 选项。
- 亮度、对比度、饱和度 -2～2，JPEG 压缩值 10～63，镜像、翻转及最大 FPS 设置保留。
- 保留最新版 JPEG 头/扫描数据结构校验、损坏计数和有界恢复。
- 相机改用 OV3660，拒绝其他 PID；XCLK 默认 20 MHz，PLL 由组件按分辨率配置，不写 OV2640 的 0x111/0xd3 等寄存器。
- 曝光从 OV3660 PID、PLL、HTS/VTS、0x3500～0x3503 回读；按组件 `ov3660.c::calc_sysclk` 的时钟模型估算毫秒值，保留 1/16 行采样精度，手动目标按整行量化。该值不是光学实测，也不由发送 FPS 推算。
- 关闭 AEC 后重新读取冻结快门；必要时扩展 VTS 保留长曝光，关闭夜间自动扩帧。手动范围为当前 VTS 减四行，开启 AEC 时恢复原基础 VTS 和夜间位，保留 AGC。
- 板载灯使用 GPIO48/RMT 发送 WS2812 白色（32/255）/关闭指令，上电默认关闭。它不等于旧板白色补光 LED 的照明亮度；协议发送成功也无法证明灯珠实际发光。
- 两块 PCA9685 仍各用地址 0x40，独立 SDA、共用 SCL；车轮约 1017 Hz、舵机约 50 Hz，不改 PCA9685 地址焊盘。
- 车轮 500 ms 看门期限、机械臂连续推进 350 ms 期限、5°～175°软件限位、上电不自动回中/运动、失联不恢复旧动作均沿用 03。

## 固定 2×8 母座：逐孔对应

**观察方向：从元件面看，天线朝左、摄像头朝右；两行均从左到右编号 1～8。** 这与根目录接线图的旧板编号一致；将用户竖拍 S3 照片逆时针旋转 90° 即为该方向。不要以背面视角直接套用。

代码集中映射在 `include/board_pins.h`。新旧信号 GPIO 编号不同，但六根外设信号线的物理孔位相同，因此可以保留现有底盘接线。

| 上排孔位 | 旧 AI-Thinker | 新 S3 照片 | 05 用途 / 是否改线 |
|---|---|---|---|
| 1 | GPIO4 | GPIO41 | 旧板内置灯位置，默认不接外部线；05 使用板载 GPIO48 灯 |
| 2 | GPIO2 | GPIO40 | 机械臂 PCA9685 SDA，原孔位保留 |
| 3 | GPIO14 | GPIO39 | 两块 PCA9685 SCL，原孔位保留 |
| 4 | GPIO15 | GPIO38 | OLED D/C，原孔位保留 |
| 5 | GPIO13 | GPIO47 | 车轮 PCA9685 SDA，原孔位保留 |
| 6 | GPIO12 | GPIO14 | 原来未接，继续留空 |
| 7 | GND | GND | **新旧板共用地，优先把底盘所有地线接至此孔** |
| 8 | 5V | 5V | 5 V 电源原孔位保留 |

| 下排孔位 | 旧 AI-Thinker | 新 S3 照片 | 05 用途 / 是否改线 |
|---|---|---|---|
| 1 | GND | EN | **若旧底盘已接地，必须移到上排 7；否则 S3 一直复位** |
| 2 | U0T / GPIO1 | TXD / GPIO43 | OLED MOSI，原孔位保留，运行时关闭 UART0 |
| 3 | U0R / GPIO3 | RXD / GPIO44 | OLED SCLK，原孔位保留，烧录时断开 OLED |
| 4 | VCC | GPIO19 | **默认留空；若旧底盘接过电源，必须断开，不能给 GPIO19 供电** |
| 5 | GND | GPIO20 | **05 启动时配置为输入，允许保留旧地线；不得同时使用原生 USB** |
| 6 | GPIO0 | GPIO0 | BOOT，正常运行留空 |
| 7 | GPIO16 | GND | 留空；若还要换回旧板，不把该孔新增为公共地 |
| 8 | 3V3 | 3V3 | OLED/PCA9685 3.3 V 逻辑电源原孔位保留 |

### 必须处理的接口

1. 下排第 1 孔若接了地，必须把该地线移到上排第 7 孔（两代板均为 GND）。EN 是复位输入，不能由软件改成地。不要改用下排第 7 孔，因为换回旧板时那里是 PSRAM 相关 GPIO16。下排第 5 孔的旧地线可以保留：05 在其他外设初始化前将 GPIO19/20 配置为无上下拉输入，放弃原生 USB，使用已确认的 COM3 CH340 串口烧录。GPIO20 不得再配置为输出，不能在保留该地线时连接原生 USB；该约束针对本工程，其他固件需重新核对。
2. 下排第 4 孔由旧 VCC 变为 USB GPIO19，默认必须空置；如有接线，需要按其实际用途改接。5V、3V3 只使用表中明确的电源孔。
3. 不启用 MicroSD，并拔出卡。OLED/PCA9685 复用 GPIO38～40，不能同时驱动卡槽信号。
4. OLED 仍使用 TXD/RXD：烧录时断开 OLED 两根信号线；运行时断开 USB-UART 下载板或解除其 TX 对 GPIO44 的驱动。若 CH340 是新板不可断开的板载电路，需要进一步确认隔离方式或重新分配 OLED 时钟，不能只靠关闭串口日志解决外部驱动冲突。
5. 旧白光补光灯属于旧开发板的一部分，换板后不再存在；05 的开关控制新板 RGB 白色输出。若实际灯珠或 GPIO 不同，须修正灯光驱动，不能按旧 GPIO4 直接输出。

照片不能证明排距、板宽和安装方向与现有母座完全一致；插板前核对 5V、3V3、GND 位置。不要把新 EN 接到旧底盘地线上试启动。

## OLED 和底盘连接

- OLED SCLK → 新 RXD/GPIO44；MOSI → 新 TXD/GPIO43；D/C → GPIO38；CS、GND → 公共 GND，MISO 不接。
- OLED 保留原模块的 3.3V 逻辑和 5V VCC 双供电；不改变原屏幕供电方案。
- 车轮板 SDA → GPIO47；机械臂板 SDA → GPIO40；两板 SCL → GPIO39；两板 VCC 仍为 3.3V，上拉电压也为 3.3V。
- TB6612、四个电机、舵机、PCA9685 通道及 XL4015 供电保持 03 的连接；动力电流不经过 ESP32。
- 芯片内部 GPIO1/3/15 不再用于旧 OLED；不要按旧 GPIO 数字找线，应按母座物理孔位看上表。

## 编译与烧录

本机 Wi-Fi 凭证复制自 03 的忽略文件 `include/wifi_credentials.local.h`，不写入公共文档、日志或提交文件。其他电脑按 `wifi_credentials.example.h` 建立该本机文件。

```powershell
. E:\Espressif\Initialize-Idf.ps1 -IdfId esp-idf-5c53b602a76f039e2cc64575f7074be6
Set-Location 'F:\CppProject\Smart Vehicle\code\05CameraS3ApSta'
idf.py -DIDF_TARGET=esp32s3 build
```

新工程不复制旧 `sdkconfig` 或 `build/`，`sdkconfig.defaults` 负责首次生成 S3 配置，依赖锁定为本工程自己的 `dependencies.lock`。Flash 配置为 16 MB，但仍沿用 `SINGLE_APP_LARGE` 分区，未启用 OTA 或 SD 存储。

完成上面的母座电源/地线调整、断开 OLED 串口线并确认相机排线接好后，用户可执行：

```powershell
idf.py -p COM3 flash
```

当前临时关闭 OLED（`CAMERA_OLED_ENABLED=n`），UART0 默认控制台与 Bootloader INFO 日志已开启，波特率 115200；同时关闭 USB 第二控制台。当前无需连接 OLED。以后恢复旧 OLED TXD/RXD 接法时必须重新关闭 UART0 日志，或者先将 OLED 两根信号线迁到其他可用 GPIO。

本机 IDE 的 `idf.monitorNoReset=true` 会使打开 Monitor 时不自动复位；若错过启动网址，按开发板 RST/EN 键重新打印启动过程。本工程为兼容旧母座将 USB19/20 保持为输入；保留 GPIO20 接地时，只使用 CH340/UART0 下载通道，不能改接原生 USB。

临时 `CAMERA_PERF_TEST` 默认关闭。05 拒绝 03 的 LEDC 时钟暂停和 OV2640 `pll`/`dvp` 参数；S3 相机时钟来自 LCD_CAM，不能用旧接口测量。其余常规网页/控制接口沿用 03。

## 本轮验证记录（2026-10-28）

- ESP-IDF 5.5.5、Xtensa ESP32-S3 GCC 14.2.0 实际编译通过。常规固件为 `build/smart_vehicle_camera_s3_apsta.bin`，1,091,424 字节，应用分区剩余约 29%；镜像校验和及 SHA256 校验有效。
- 可选性能测量分支在独立 `build-perf/` 中编译通过。交付使用最终常规 `build/` 固件，包含 GPIO19/20 输入约束；常规配置和默认配置均关闭测量接口。
- 主机验证直接提取生产 `readExposure`、`setExposure`、`setAutoExposure` 函数体，以模拟 SCCB 覆盖 PLL/行周期、分数快门、手动范围、长曝光冻结/缩短/自动恢复、AGC 保留、幂等切换、错误 PID、寄存器失败、跨帧不一致和不支持的时钟源，全部通过。模拟不替代传感器实测；验证产物仅位于忽略的 `build/checks/`。
- 网页生产 JavaScript 通过 Node.js 语法检查；全文与当前 03 比对，除“补光灯→板载灯”文案外一致。
- 车轮/机械臂业务源码及其接口、主循环与当前 03 保持逐字节一致；03 的源码、头文件和网页 SHA256 比对无变化。
- Wi-Fi 本机凭证、生成配置、依赖缓存、构建和验证产物均被忽略；独立依赖锁已更新为 `esp32s3`。
- 创建工程阶段未自动烧录；后续按用户授权完成临时 UART 诊断和烧录，结果见下一节。未执行实体电机/舵机动作。

## OLED 临时关闭与热点故障定位（2026-10-28）

- 先对板上 Bootloader、分区表、应用执行只读校验，三项均匹配原 05 构建，排除烧录内容不一致。
- 用户确认仅连接相机和下载器，没有连接 OLED。关闭 OLED 并启用 UART0 日志后，真实报错为 `NO-SOI / NO-EOI`，随后 `INITIAL_FRAME_TIMEOUT` / `ESP_ERR_TIMEOUT`；相机初始化失败使 Wi-Fi 启动分支未执行，因此没有热点。
- 同次日志确认 OV3660 控制通信成功（SDA=4、SCL=5、地址 0x3c、PID 0x3660）；8 MB Octal PSRAM 的 80 MHz 初始化和内存自检通过。PCA9685 未连接的 `ESP_ERR_NOT_FOUND` 是预期告警，代码继续启动相机。
- 保持相机引脚、20 MHz XCLK、分辨率、JPEG 和双缓冲不变，仅关闭 `CONFIG_CAMERA_PSRAM_DMA`，重新编译并烧录。关闭后，初始化约 1.8 秒取帧成功，约 2.2 秒 AP 启动，约 3.8 秒 STA 获得地址。帧缓冲仍在 PSRAM，只是不使用直接 PSRAM DMA 路径。
- 热点：`SmartVehicle-CAM`，地址 `http://192.168.4.1/`；本次 DHCP 地址：`http://192.168.1.15/`（后续可能变化）。
- 实测 `/`、`/status`、`/capture` 均 HTTP 200；抓拍完整解码为 RGB 640×480；`http://192.168.1.15:81/stream` 连续 5 帧均完整解码为 640×480。仅作短时功能验证，不等于长期稳定性或完整帧率测试。
- 最终固件保持 OLED 关闭、UART0 115200、Bootloader INFO、USB 第二控制台关闭、直接 PSRAM DMA 关闭。Bootloader、应用、分区表均烧录并通过 Hash 校验。串口已释放，可直接打开 Monitor。
- 忽略目录中的证据：`build/checks/uart-diagnostic-boot.log`（原直接 DMA 失败），`no-psram-dma-boot.log`（成功启动），`no-psram-dma-flash.log`（烧录校验），`no-psram-dma-http-check.json`（HTTP 与视频解码验证）。

## 参考资料

- [Espressif esp32-camera 2.1.8 支持列表](https://components.espressif.com/components/espressif/esp32-camera/versions/2.1.8/readme)
- [Espressif OV3660 驱动及 PLL 模型](https://github.com/espressif/esp32-camera/blob/master/sensors/ov3660.c)
- [ESP32-S3 GPIO、USB 与保留引脚](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/api-reference/peripherals/gpio.html)

本项目的针位兼容结论来自用户照片与旧工程接线表的逐孔比对，不把其他同名 S3 板的原理图视为本板实测结果。
