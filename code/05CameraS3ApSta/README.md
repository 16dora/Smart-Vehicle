# 05 ESP32-S3-N16R8 CAM＋OV3660 AP＋STA 小车

独立 ESP-IDF 5.5.5 / CMake / C++20 工程，复制自当前工作区的 `03CameraApSta`，不引用 03 的源码、配置或构建目录。默认目标固定为 `esp32s3`。

**当前配置：启用直接 PSRAM DMA、64 字节 GDMA 突发与缓存对齐，16 MHz XCLK、三帧缓冲、优先最新帧；网络发送直接使用相机帧，不再复制整张 JPEG 到发送包。OLED 改为 SH1106 I²C（SDA41/SCL14），UART0 115200 和 Bootloader INFO 日志恢复，板载灯支持 0～100% 亮度调节。相机上电默认垂直翻转，网页默认勾选并以设备回读为准。**

## 硬件依据与验证边界

- 用户提供的开发板元件面照片：`ESP32-S3-CAM` 丝印，左右各八针；相机排线为 OV3660 标识。
- 2026-10-28 经 COM3、esptool 4.12.0 实际读取：ESP32-S3 QFN56 revision v0.2，16 MB Flash、Quad、3.3 V；芯片信息识别 8 MB Embedded PSRAM AP_3v3、40 MHz 晶振。COM3 为 CH340 USB-UART。
- 本工程配置 16 MB QIO Flash、8 MB Octal PSRAM，Flash/PSRAM 均为 80 MHz。8 MB PSRAM 已在实际启动中识别，80 MHz 初始化和 SPI SRAM memory test 通过。
- 外接针位按照片及现有接线图比对。ESP32-S3-EYE 相机映射已通过 OV3660 PID=0x3660、VGA 抓拍和持续 MJPEG 完整解码验证。GPIO48 WS2812 指令发送成功，灯珠实际发光仍需目视确认。
- 已上板验证相机和网络；本轮切换 SH1106 I²C；当前启动未检测到屏幕应答，需核对接线后复位并目视确认显示。底盘及机械臂运动未在本轮测试。

## 功能与项目 03 的关系

保留当前 03 的 AP＋STA、STA 重连、80/81/82 三个 HTTP 服务、单查看者 MJPEG、单张 JPEG、图像设置、自动/手动毫秒曝光、运行参数、OLED、四轮麦克纳姆运动、两路机械臂及停止/断线保护。网页保留当前 03 的标签页、键盘映射、触摸操作、45% 速度初值和状态浮层；板载灯新增亮度滑块并保留开关，支持视频断流重连。不接入项目 04 云端。

- 默认开启传感器垂直翻转，网页复选框初始勾选；允许运行时关闭，重启恢复默认。默认 VGA 640×480、JPEG 压缩值 12，发送上限 30 FPS，与当前 03 源码默认值一致；实际上限不代表采集或网络能够达到该帧率。
- 保留 03 的六档分辨率（160×120～1600×1200），没有为 OV3660 额外添加 2048×1536 选项。
- 亮度、对比度、饱和度 -2～2，JPEG 压缩值 10～63，镜像、翻转及最大 FPS 设置保留。
- 保留 JPEG 头/扫描数据结构校验，新增取帧超时及恢复次数统计；连续故障最多恢复两次，连续 30 张正常帧后更新恢复预算。网页断流或连续八秒无后端帧进展时，仅重连视频；停止预览和离页会取消重试，不恢复旧运动。
- 相机改用 OV3660，拒绝其他 PID；XCLK 默认 16 MHz，PLL 由组件按分辨率配置，不写 OV2640 的 0x111/0xd3 等寄存器。
- 曝光从 OV3660 PID、PLL、HTS/VTS、0x3500～0x3503 回读；按组件 `ov3660.c::calc_sysclk` 的时钟模型估算毫秒值，保留 1/16 行采样精度，手动目标按整行量化。该值不是光学实测，也不由发送 FPS 推算。
- 关闭 AEC 后重新读取冻结快门；必要时扩展 VTS 保留长曝光，关闭夜间自动扩帧。手动范围为当前 VTS 减四行，开启 AEC 时恢复原基础 VTS 和夜间位，保留 AGC。
- 板载灯使用 GPIO48/RMT 发送 WS2812 白色/关闭指令，亮度 0～100%，预设上电默认 13%（各通道约 33/255），上电默认关闭。关灯时调节只更新预设；0% 不发光，开启/关闭状态独立保留。它不等于旧板白色补光 LED 的照明亮度；协议发送成功也无法证明灯珠实际发光。
- 两块 PCA9685 仍各用地址 0x40，独立 SDA、共用 SCL；车轮约 1017 Hz、舵机约 50 Hz，不改 PCA9685 地址焊盘。
- 车轮 500 ms 看门期限、机械臂连续推进 350 ms 期限、5°～175°软件限位、上电不自动回中/运动、失联不恢复旧动作均沿用 03。

## 固定 2×8 母座：逐孔对应

**观察方向：从元件面看，天线朝左、摄像头朝右；两行均从左到右编号 1～8。** 这与根目录接线图的旧板编号一致；将用户竖拍 S3 照片逆时针旋转 90° 即为该方向。不要以背面视角直接套用。

代码集中映射在 `include/board_pins.h`。底盘两块 PCA9685 的三根信号线保持原物理孔位；更换 SH1106 后，两根 OLED 信号改接上排第 1/6 孔，旧屏的 GPIO38/43/44 接线移除。

| 上排孔位 | 旧 AI-Thinker | 新 S3 照片 | 05 用途 / 是否改线 |
|---|---|---|---|
| 1 | GPIO4 | GPIO41 | SH1106 SDA，新接线；05 板载灯另用 GPIO48 |
| 2 | GPIO2 | GPIO40 | 机械臂 PCA9685 SDA，原孔位保留 |
| 3 | GPIO14 | GPIO39 | 两块 PCA9685 SCL，原孔位保留 |
| 4 | GPIO15 | GPIO38 | 旧 OLED D/C 已释放，留空 |
| 5 | GPIO13 | GPIO47 | 车轮 PCA9685 SDA，原孔位保留 |
| 6 | GPIO12 | GPIO14 | SH1106 SCL，新接线 |
| 7 | GND | GND | **新旧板共用地，优先把底盘所有地线接至此孔** |
| 8 | 5V | 5V | 5 V 电源原孔位保留 |

| 下排孔位 | 旧 AI-Thinker | 新 S3 照片 | 05 用途 / 是否改线 |
|---|---|---|---|
| 1 | GND | EN | **若旧底盘已接地，必须移到上排 7；否则 S3 一直复位** |
| 2 | U0T / GPIO1 | TXD / GPIO43 | UART0 TX，接下载器 RX；移除旧 OLED MOSI 线 |
| 3 | U0R / GPIO3 | RXD / GPIO44 | UART0 RX，接下载器 TX；移除旧 OLED SCLK 线 |
| 4 | VCC | GPIO19 | **默认留空；若旧底盘接过电源，必须断开，不能给 GPIO19 供电** |
| 5 | GND | GPIO20 | **05 启动时配置为输入，允许保留旧地线；不得同时使用原生 USB** |
| 6 | GPIO0 | GPIO0 | BOOT，正常运行留空 |
| 7 | GPIO16 | GND | 留空；若还要换回旧板，不把该孔新增为公共地 |
| 8 | 3V3 | 3V3 | SH1106 VCC/PCA9685 3.3 V 逻辑电源 |

### 必须处理的接口

1. 下排第 1 孔若接了地，必须把该地线移到上排第 7 孔（两代板均为 GND）。EN 是复位输入，不能由软件改成地。不要改用下排第 7 孔，因为换回旧板时那里是 PSRAM 相关 GPIO16。下排第 5 孔的旧地线可以保留：05 在其他外设初始化前将 GPIO19/20 配置为无上下拉输入，放弃原生 USB，使用已确认的 COM3 CH340 串口烧录。GPIO20 不得再配置为输出，不能在保留该地线时连接原生 USB；该约束针对本工程，其他固件需重新核对。
2. 下排第 4 孔由旧 VCC 变为 USB GPIO19，默认必须空置；如有接线，需要按其实际用途改接。5V、3V3 只使用表中明确的电源孔。
3. 不启用 MicroSD，并拔出卡。OLED/PCA9685 使用卡槽相关 GPIO39～41，不能同时驱动卡槽信号。
4. 新 SH1106 只接 3V3、GND、SDA41、SCL14；旧屏的 GPIO38/43/44 及额外 5V 供电线移除。新接法与 UART0 分离，可以保留下载器连接并正常读取启动日志。
5. 旧白光补光灯属于旧开发板的一部分，换板后不再存在；05 的开关控制新板 RGB 白色输出。若实际灯珠或 GPIO 不同，须修正灯光驱动，不能按旧 GPIO4 直接输出。

照片不能证明排距、板宽和安装方向与现有母座完全一致；插板前核对 5V、3V3、GND 位置。不要把新 EN 接到旧底盘地线上试启动。

## OLED 和底盘连接

- SH1106 SDA → GPIO41（上排第 1 孔）；SCL → GPIO14（上排第 6 孔）；GND → 公共 GND（上排第 7 孔）。
- SH1106 VCC → 3V3（下排第 8 孔），只使用四根线；不沿用旧 SPI 屏的双电源、D/C、CS 接法。软件用 I2C0、100 kHz，依次探测 0x3C/0x3D，显示 128×64、8 页、列偏移 2。相机继续使用 I2C1，地址同为 0x3C 也不会冲突。
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

`sdkconfig.defaults` 负责首次生成 S3 配置；已有本机 `sdkconfig` 同步使用 16 MHz、直接 DMA、三缓冲、关闭测量接口。依赖锁定为本工程自己的 `dependencies.lock`，相机组件由 `source/idf_component.yml` 的 `override_path` 指向项目内修复版本。Flash 配置为 16 MB，但仍沿用 `SINGLE_APP_LARGE` 分区，未启用 OTA 或 SD 存储。

完成上面的母座电源/地线调整、移除旧 SPI OLED 接线并确认相机排线接好后，可正常使用 COM3：

```powershell
idf.py -p COM3 flash
idf.py -p COM3 monitor
```

当前 `CAMERA_OLED_ENABLED=y`，使用 SH1106 I²C 驱动；`ESP_CONSOLE_UART_DEFAULT=y`、Bootloader INFO、115200 波特率，USB 第二控制台保持关闭。下载完成可正常复位，不再需要为 OLED 断开下载器 TX/RX。I²C ACK 只能确认总线传输，实际显示仍需目视确认。

本机 IDE 的 `idf.monitorNoReset=true` 使打开 Monitor 时不自动复位；若错过启动网址，按 RST/EN。热点地址为 `http://192.168.4.1/`，本次 STA 为 `http://192.168.1.15/`（DHCP 地址可能变化）。本工程为兼容旧母座将 GPIO19/20 保持输入；保留 GPIO20 接地时，只使用 CH340/UART0，不使用原生 USB。

临时 `CAMERA_PERF_TEST` 默认关闭。05 拒绝 03 的 LEDC 时钟暂停和 OV2640 `pll`/`dvp` 参数；S3 相机时钟来自 LCD_CAM，不能用旧接口测量。其余常规网页/控制接口沿用 03。

## 直接 DMA 与卡顿修复（2026-10-28）

- 基于 `esp32-camera 2.1.8`（组件记录上游提交 `b0556a78e13d42974aae19116f8a1e14c29b0f4d`），修复代码存于 `components/esp32-camera/`；保留原 Apache-2.0 `LICENSE`，不修改依赖下载缓存。核心改动仅在 `driver/cam_hal.c` 和 `target/esp32s3/ll_cam.c`。
- OV3660 在 JPEG EOI 后可能继续输出填充。原直接 DMA 路径只检查帧尾的小窗口，误判 `NO-EOI`，最终 `INITIAL_FRAME_TIMEOUT`，网络启动分支未执行。现在按 JPEG 段长度跳过表/元数据，在熵编码区找到首个真正 EOI，仅裁剪有效长度，拒绝截断帧，不复制整帧。该问题亦见 [上游 issue #853](https://github.com/espressif/esp32-camera/issues/853)。
- 直接 DMA 缓冲按缓存行及 64 字节外部突发要求对齐；交给 DMA 前和 CPU 读取 DMA 结果前正确失效缓存。开启 GDMA 数据/描述符突发，预置外部块长；该初始化早于模式选择，不能根据尚未初始化的 `psram_mode` 条件设置。
- 突发配置参考该 issue 的[贡献者补丁](https://github.com/user-attachments/files/31724495/esp-camera.zip)，下载 SHA256：`5e5fc7fe8528763c179992f00c5c7c9a8d8b9eb5afb44ab6f5c795f8725cb1e5`；项目另外实现结构化 EOI 定位与缓存对齐，不宣称贡献者补丁是上游正式发布修复。
- 数据路径为 `LCD_CAM → GDMA → PSRAM 帧 → HTTP/TCP`。HTTP 发送结束才归还帧；三个缓冲让其他帧继续采集。移除了应用层整帧 `memcpy` 和逐帧发送包扩容，TCP/Wi-Fi 协议栈内部仍有复制，不是端到端零复制。
- 旧配置关闭状态轮询仍有秒级停顿，取帧约 0.6 ms，发送等待可达秒级，CPU 并未耗尽。相同固件和查询频率下仅切换 XCLK 及驱动匹配的 PLL：16 MHz 首轮 40 秒 707 帧、17.69 FPS、最大间隔 141 ms；20 MHz 复测 40 秒 84 帧、2.19 FPS、最大间隔 4469 ms；恢复 16 MHz 后 25 秒 446 帧、17.86 FPS、最大间隔 125 ms。三次均无 JPEG 解码失败，20 MHz 有一次状态请求超时。
- 同一对照中 RSSI 约 -48 dBm，16 MHz 的视频发送窗口均值约 4.2 ms，20 MHz 约 1904 ms；16 MHz ping 3～10 ms，20 MHz 19～203 ms。可以确认该板的时钟配置明显影响网络表现，但没有频谱仪/原理图，不能进一步断言是射频辐射、供电还是总线争用。
- 保留 `WIFI_PS_NONE`、TCP_NODELAY 和 16 个内部静态 Wi-Fi 发送缓冲；增大 TCP 发送窗口到 65535 字节，正式固件使用 `-O2` 性能编译并保留断言。未启用 Wi-Fi/lwIP 的 PSRAM 分配，因此不设置受该选项约束的 cache TX 缓冲。115200 串口默认过滤高频 SDK INFO，保留原日志调用、全部 WARN/ERROR 和 `camera_main` 启动网址，可用 `CAMERA_VERBOSE_LOGS` 开启详细日志。
- 车轮/机械臂状态查询只读，不是本次卡顿的充分原因。无控制连接时跳过无效连接巡检投递；保留 500 ms 连接巡检、车轮 500 ms 看门期限和机械臂 350 ms 连续推进期限。视频恢复不放宽运动保护。

## 直接 DMA 优化阶段验证（恢复 OLED 之前）

- 验证记录与原始 JSON 位于忽略的 `build/checks/`。直接 DMA/16 MHz/三缓冲链路连续 300 秒解码 5190 张 VGA JPEG，平均 17.30 FPS；帧间隔 P95 78 ms、P99 125 ms、最大 390 ms，无超过 500 ms 的间隔，无断流、解码或查询错误；同时每两秒请求状态、每秒各查一次车轮和机械臂状态，期间并发打开网页及抓拍三次均通过。该阶段为切换 `-O2` 前的同一采集/网络代码，最终 `-O2` 高频轮询结果另列。
- 上述 149 个活动状态窗口，内部空闲内存 124419～153127 字节、PSRAM 空闲约 7.23 MB；发送窗口均值 3.94～4.66 ms。JPEG 错误、取帧超时及相机恢复均为零。UART 结尾的 errno 104 出现在探针主动关闭视频之后，不是设备提前断流；最终运行时是否提前结束以逐帧探针结果为准。
- 最终 `-O2` 固件以网页实际 250 ms 周期分别查询车轮/机械臂，另每两秒读取状态：90 秒完整解码 1600 帧，17.78 FPS，P95 63 ms、P99 78 ms、最大间隔 109 ms；无断流、解码或查询错误，JPEG 错误/取帧超时/相机恢复均为零；最后窗口发送均值 3.33 ms。证据：`final-o2-ui-poll.json`。
- 最终固件的 160×120、320×240、640×480、800×600、1024×768、1600×1200 每档两张抓拍均完整解码，网页 HTTP 200；完成后恢复 VGA、自动曝光。证据：`final-resolution-check.json`。正式 `/perf` 返回 404。
- ESP-IDF 5.5.5 / GCC 14.2.0 编译通过，最终应用 1043824 字节，应用分区剩余约 32%；COM3 烧录后各段 Hash 校验通过。最终镜像 SHA256：`72448dd5f881f94ff291e71b532ee7e8871406bfb60bebc835f7d1a56d3b18e7`。清理无效默认项后增量构建镜像 Hash 未变，这是恢复 OLED 前的性能验证镜像；本轮新固件结果另列。
- 生产 JPEG 定界函数主机检查覆盖真实 JPEG、长尾填充、拼接双帧、元数据内 FF D9、截断和非法段长度；网页生产脚本通过语法检查及重试取消/合并检查。
- 本节测试时 OLED 禁用，底盘及机械臂未连接；未执行实体电机/舵机动作。其硬件运动性能需要连接后验证，不能由只读状态查询替代。
- 测试画面、曝光、路由器及无线环境会改变帧率；网页 30 FPS 是发送上限，不保证传感器或链路始终提供 30 FPS。当前选择以实测稳定为依据，不宣称所有环境的绝对最优。

## 历史：默认翻转与旧 SPI OLED 恢复（2026-10-28）

本节是切换 SH1106 之前的记录，旧 SPI 接法与关闭串口配置已由下一节替换；默认垂直翻转继续保留。

- `initCamera()` 在默认 VGA/JPEG 设置后调用 `set_vflip(1)`，失败则明确终止初始化；旧帧清理在翻转后执行。网页初始勾选“垂直翻转”，之后仍以 `/status` 的传感器回读更新，避免只翻转复选框却没有改变相机。
- OLED 默认启用，沿用 MOSI=GPIO43、SCLK=GPIO44、D/C=GPIO38、SPI2 1 MHz、不使用 SPI DMA；主任务每两秒刷新 CPU 与曝光信息。直接相机 DMA、16 MHz XCLK、三缓冲和 -O2 配置保持。
- OLED 与 UART0 复用，关闭应用控制台与 Bootloader 日志；保留日志代码并过滤 OLED 高频 INFO。烧录前断开 OLED 的两根 SPI 信号线；烧录后断开下载器 TX/RX 信号线，再接回 OLED，复位运行。可用 `esptool --after no_reset ... write_flash` 在写入后停留下载程序，避免下载器仍连接时自动进入 OLED 应用。
- 本轮 ESP-IDF 5.5.5 性能构建通过，生成配置确认 OLED 开启、应用控制台为空、Bootloader 日志关闭、直接 PSRAM DMA 开启；应用 1060192 字节，分区剩余约 31%。COM3 写入 Bootloader/分区表/应用并全部通过 Hash 校验，使用 `--after no_reset` 停留下载程序，等待用户断开下载器信号线、接回 OLED 并重新上电。日志：`build/checks/oled-vflip-build.log`、`oled-vflip-flash.log`。
- 本轮应用镜像 SHA256：`17b10f4dd7afdec1072b153056d9b6c2d3fe4e3dec3ac867a39e19d8b8086eb9`。屏幕是只写 SPI，无 ACK/MISO，不能用 SDK 返回成功代替实际亮屏；接回后的实际显示与网络验证尚待确认。

## SH1106 I²C 与板载灯亮度（2026-10-28）

- 新驱动在 `oled_display.cpp` 内使用 ESP-IDF 新版 I2C master API，独占 I2C0，与相机的 I2C1 分离。初始化开启 SH1106 内部 DC/DC，按 8 页显存从第 2 列写入 128 列，不发送旧 SSD1306 水平寻址命令。无屏幕应答或首帧发送失败时释放总线资源，仍允许相机和网络启动；接好屏幕后按 RST 重新探测。
- 主任务每两秒更新 CPU 与曝光，显示发送不持有相机/状态锁；保留 I2C ACK 检查、100 ms 单次发送超时和失败后下一周期重画。默认过滤高频 OLED INFO，启动成功时用 `camera_main` 打印屏幕地址与引脚。
- `POST /led` 支持 `enabled=0/1` 和/或 `brightness=0..100`；`/status`、`/led` 回报 `led_brightness`。百分数映射为 `round(percent*255/100)`，三个通道相同，0% 不发光。`led_on` 表示输出使能，因此允许 `led_on=true` 且亮度为 0；不表示存在物理发光反馈。重启恢复关灯与 13% 预设，不写入 NVS。
- 网页拖动时只显示百分数，松手/键盘 change 时提交一次；关灯时可预设亮度，已有请求未完成时禁用控件，轮询旧响应不覆盖新指令，超时不自动重发，后续状态查询重新确认。
- ESP-IDF 5.5.5 / -O2 编译、COM3 烧录与 Hash 校验通过。生成配置确认 OLED 开启、UART0/Bootloader INFO 恢复、直接 PSRAM DMA、16 MHz 与三缓冲保持。原始日志与探针均位于 `build/checks/sh1106-led-*`。
- 最终应用 1052384 字节，SHA256：`a043bbd8d266e54d119681ed59dd5074c6dd4d33ecee632b1cbfa5356d214257`。补齐 SDK 参数解析日志后重新构建、烧录、启动；回读确认 `vflip=1`、`led_ready=true`、`led_on=false`、`led_brightness=13`。COM3 已释放，可直接 Monitor；日志仍显示屏幕未应答。
- 生产 OLED 驱动主机验证覆盖 I2C0/SDA41/SCL14、两个地址、升压命令、8 页/1024 字节及列偏移、缺屏和首帧失败清理。生产 RMT 函数验证 0/1/13/50/100% 对应三通道波形、关灯、锁存时间与越界拒绝；网页生产脚本通过语法及预设、拖动/轮询隔离、忙碌串行、0% 和失败恢复检查。
- 上板 LED 接口验证关灯预设 42%、单独开灯、100%、0% 及复位预设；越界/负数/小数/空值/非法开关/超长值均返回 400 且不改变状态，最后恢复关灯和 13% 预设。`vflip=1`，网页和 VGA 抓拍通过；并发视频 90 秒完整解码 1599 帧，17.78 FPS、最长间隔 109 ms，无断流、损坏或状态查询错误，取帧超时/相机恢复均为零。
- 当前启动日志：OLED 两个地址未应答，`ESP_ERR_NOT_FOUND`，尚未验证实物亮屏或带实际显示刷新的视频性能。需要按本节四针接线并复位后验证；不能把主机协议检查或缺屏情况下的视频测试当作屏幕实测通过。未发送电机/舵机动作。

## 参考资料

- [ESP-IDF 5.5.5 I2C master API](https://docs.espressif.com/projects/esp-idf/en/v5.5.5/esp32s3/api-reference/peripherals/i2c.html)
- [Adafruit SH1106 初始化与寻址参考](https://github.com/adafruit/Adafruit_SH110x/blob/master/Adafruit_SH1106G.cpp)
- [Espressif esp32-camera 2.1.8 支持列表](https://components.espressif.com/components/espressif/esp32-camera/versions/2.1.8/readme)
- [Espressif OV3660 驱动及 PLL 模型](https://github.com/espressif/esp32-camera/blob/master/sensors/ov3660.c)
- [ESP32-S3 GPIO、USB 与保留引脚](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/api-reference/peripherals/gpio.html)

本项目的针位兼容结论来自用户照片与旧工程接线表的逐孔比对，不把其他同名 S3 板的原理图视为本板实测结果。
