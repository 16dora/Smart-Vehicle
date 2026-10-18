# ESP32-CAM 摄像头 Web 服务

独立 ESP-IDF v5.5.5 / CMake 工程，C++20。面向 AI-Thinker ESP32-CAM、OV2640、4MB PSRAM，按项目接线资料选择板型。

## 功能与默认值

- 支持两种构建模式：连接路由器 `301`（STA），或 ESP32 自建热点（AP）；常规构建默认保留 STA。
- 网页端口 80，视频端口 81；控制请求不与 MJPEG 长连接共用服务任务。
- 默认 VGA（640 × 480）、JPEG 压缩值 12、发送上限 10 FPS。
- 分辨率：160 × 120、320 × 240、640 × 480、800 × 600、1024 × 768、1600 × 1200。
- JPEG 压缩值 10–63，越小画质越高。最大帧率 1–30 FPS，表示视频发送上限，不保证达到此帧率。
- 亮度、对比度、饱和度各 -2–2；支持水平镜像、垂直翻转和单张 JPEG。
- 网页显示最近统计窗口的实际发送帧率，并支持启动、停止预览。
- 设置只在本次运行有效，重启后恢复默认。当前仅面向一个视频查看者。

## Wi-Fi 配置

路由器 STA 模式的本地实际配置在 `include/wifi_credentials.local.h`，已加入 Git 忽略；网页和日志不输出密码。
新检出的工程先将 `include/wifi_credentials.example.h` 复制为同目录下的 `wifi_credentials.local.h`，再填写密码。
固件必须包含连接所需凭据，构建出的二进制也应按包含凭据的文件管理。

只连接 WPA2 或兼容 WPA2 的 2.4 GHz 网络，不连接仅 5 GHz 的热点。
IP 由路由器分配；需要固定地址时可在路由器中配置 DHCP 地址保留。
手机或电脑连接同一局域网，路由器需要允许设备间访问，视频还需要能访问 81 端口。

### ESP32 自建热点（AP）

AP 模式独立工作，不连接路由器，也不向路由器模式自动回退。它不需要本地路由器凭据文件。

- 热点名称：`SmartVehicle-CAM`
- 热点密码：`vehiclecam123`
- 网页地址：`http://192.168.4.1/`
- 视频地址：`http://192.168.4.1:81/stream`
- 固定使用 2.4 GHz 信道 6，WPA2-PSK，最多两个接入设备，仍只支持一个视频查看者。

手机或电脑连接该热点后，选择继续使用此“无互联网”网络，再在浏览器输入上述 `http://` 地址。
此热点仅提供相机局域网服务，不提供互联网转发或自动弹出配网页面。

热点名称和密码可在 `idf.py menuconfig` → `Camera Web Server` 中修改；密码需要 8–63 字节，名称为 1–32 字节。
`/status` 的 `connected` 在 AP 模式表示热点网络已就绪，不表示已经连接路由器。

## 编译、烧录与查看 IP

在 PowerShell 中执行：

```powershell
. E:\Espressif\Initialize-Idf.ps1 -IdfId esp-idf-5c53b602a76f039e2cc64575f7074be6
Set-Location 'F:\CppProject\Smart Vehicle\code\02CameraWebServer'
idf.py build
```

上述命令继续构建路由器 STA 模式，产物位于 `build/`。

AP 模式使用独立配置和构建目录，在同一个子项目目录执行：

```powershell
idf.py -B build-ap -D SDKCONFIG=sdkconfig.ap -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.ap.defaults" build
```

确认允许替换板上应用后，AP 模式烧录及监视使用同一组路径参数（COM3 按实际端口调整）：

```powershell
idf.py -B build-ap -D SDKCONFIG=sdkconfig.ap -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.ap.defaults" -p COM3 flash
idf.py -B build-ap -D SDKCONFIG=sdkconfig.ap -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.ap.defaults" -p COM3 monitor --no-reset
```

AP 启动日志为 `SoftAP started ssid=SmartVehicle-CAM; open http://192.168.4.1/`。
常规 `sdkconfig` / `build/` 与 AP 的 `sdkconfig.ap` / `build-ap/` 相互独立；只修改 defaults 不覆盖已有完整配置。
需要调整已生成的 AP 配置时，执行 `idf.py -B build-ap -D SDKCONFIG=sdkconfig.ap menuconfig`。

如果希望直接使用 VSCode 现有的构建、烧录按钮，可在默认配置编辑器中选择 `Camera Web Server` → `Wi-Fi connection mode` → `Create camera hotspot (AP)`，保存后重新构建、烧录。
该操作会将常规 `build/` 也切到 AP；要恢复路由器方案，在同一菜单选择 `Connect to router (STA)`。

首次构建通过组件管理器下载锁定版本 `espressif/esp32-camera` 2.1.8；保留 `dependencies.lock` 以复现依赖。
确认允许替换开发板现有应用后，执行：

```powershell
idf.py -p COM3 flash
idf.py -p COM3 monitor --no-reset
```

VSCode 打开根目录 `Smart Vehicle.code-workspace`，在 ESP-IDF 项目选择中选中 `02CameraWebServer` 后再编译、烧录或监视；也可直接打开本子项目目录。
Monitor 为 115200 波特率，按 `Ctrl+]` 退出。同一串口不要被多个程序同时占用。

启动连接成功后串口打印：

```text
Wi-Fi connected; open http://<路由器分配的IP>/
```

浏览器打开实际 IP，点击“开始预览”。如 Monitor 接入时错过启动日志，可在路由器客户端列表查看 IP；需要重新观察启动过程时手动按 EN / RESET。
STA 模式 Wi-Fi 断线后每三秒重新尝试连接，重新取得 IP 后串口再次打印地址；浏览器中的视频可手动重新开始。

## 硬件与配置

相机引脚采用乐鑫 CameraWebServer 的 AI-Thinker 映射：

| 信号 | GPIO |
|---|---|
| PWDN / RESET | 32 / 不使用 |
| XCLK | 0 |
| SCCB SDA / SCL | 26 / 27 |
| D0 / D1 / D2 / D3 | 5 / 18 / 19 / 21 |
| D4 / D5 / D6 / D7 | 36 / 39 / 34 / 35 |
| VSYNC / HREF / PCLK | 25 / 23 / 22 |

默认配置开启 PSRAM（40 MHz）和 UART 日志，Flash 配置为 4MB，使用单个大应用分区。
系统 socket 额度设为 16；两个 HTTP 服务内部共需六个，客户端连接最多五个，另为网络操作保留余量。
Wi-Fi 使用 16 个静态 TX 缓冲，TCP 发送缓冲从 5760 增至 23040 字节；接收缓冲、Wi-Fi 协议和发送超时保持原配置。
网页与视频连接在接受时统一启用并回读 `TCP_NODELAY`，减少 HTTP 头部及 MJPEG 小块数据的合并等待。
CPU 默认运行在 240 MHz；相机任务固定在 CPU1，与 CPU0 的 Wi-Fi 任务分开。
系统动态电源管理和 STA 未连接省电均关闭；AP、STA 启动后都显式设置并回读 `WIFI_PS_NONE`。CPU 空闲不表示降频或进入 Wi-Fi 省电。
Wi-Fi 发送功率上限向 SDK 请求 20 dBm，同时打印回读值。当前板启动回读为 60 个 0.25 dBm，即 15 dBm；限制来源尚未确定，不能把 API 设置成功写成实际已有 20 dBm 发射功率。
按 UXGA 和高画质分配 PSRAM 帧缓冲，再将传感器设置为 VGA，支持运行时增大分辨率。当前默认两个缓冲，可通过 `Camera frame buffer count` 改为一个。
采用 `CAMERA_GRAB_WHEN_EMPTY`：后台在空闲缓冲中采集，全部缓冲占用后等待，归还缓冲后继续；没有查看视频时不会无限覆盖队列。这不是停止传感器时钟，也不保证硬件采集帧率等于网页的发送上限。
初始化时清理每个缓冲可能保留的最大分辨率旧帧；视频发送完成后及时归还帧缓冲，再更新统计状态。
GPIO4 的补光灯保持未启用；本工程不驱动电机、不使用 TF 卡，也不初始化其他车载外设。
检查相机排线插牢，使用稳定的板载 5V 供电与公共地。

PSRAM 未就绪或传感器不为 OV2640 时会记录错误并停止相机服务初始化。
高分辨率、高画质、弱光或较差的 Wi-Fi 信号会降低实际帧率；发送限速不等同于修改传感器硬件帧率。

## HTTP 接口

| 接口 | 方法 | 功能 |
|---|---|---|
| `http://IP/` | GET | 中文控制网页 |
| `http://IP/status` | GET | 当前参数与发送帧率 JSON |
| `http://IP/control` | POST | 表单 `var=quality&val=12` 等设置 |
| `http://IP/stop` | POST | 结束当前视频代次 |
| `http://IP/capture` | GET | 单张 JPEG |
| `http://IP:81/stream` | GET | MJPEG 视频 |

控制参数名：`size`（0–5，按上文分辨率顺序）、`quality`、`max_fps`、`brightness`、`contrast`、`saturation`、`hmirror`、`vflip`。非法参数或超出范围返回 400。
控制页面面向当前局域网内使用，不配置公网映射。

## 文件与上板验证

- `source/main.cpp`：应用入口。
- `source/camera_web.cpp`、`include/camera_web.h`：相机、网络、HTTP 与共享状态。
- `web/index.html`：随固件嵌入的静态页面，无外部 CDN 依赖。
- `source/idf_component.yml`、`dependencies.lock`：驱动版本约束和依赖锁定。
- `sdkconfig.defaults`：可复现的默认硬件配置。
- `source/Kconfig.projbuild`：STA/AP 模式及热点名称、密码选项。
- `sdkconfig.ap.defaults`：热点构建的配置覆盖；生成的 `sdkconfig.ap` 不提交版本库。

编译成功后，上板确认：串口打印 PSRAM 和 OV2640 信息、取得 IP、网页可见、预览正常、参数修改生效、非法值被拒绝、帧率上限有效、停止及重新预览正常、路由器恢复后可以重连。
未实际烧录前，不将上述项目记为硬件测试通过。

## 预览失败时的检查信息

视频开始时串口记录请求进入、首帧字节数、尺寸、JPEG 起始标志、取帧耗时和首帧发送耗时。
启动时应出现 `Wi-Fi TX config type=0 static_buffers=16 tcp_snd_buf=23040`；新 HTTP 连接应出现 `tcp_nodelay=1`。
发送失败会记录阶段（`frame_header`、`jpeg_data`、`frame_tail`）、原始 `errno`、已完成帧数、发送耗时，以及可取得的 Wi-Fi 信号强度和内部可用内存。
AP 模式不查询路由器信号强度，改为记录热点就绪状态和接入设备数量。
`errno=11` 表示发送等待超时，`errno=104` 表示对端重置连接；两者都不能单独证明 Wi-Fi 断线。
网页分别提示响应超时、HTTP 服务错误、返回数据错误或网络请求失败；状态请求不会重叠堆积。
网页请求使用 XMLHttpRequest 的五秒超时，不要求浏览器支持 `AbortSignal.timeout()`。
本次修改后需要重新烧录固件并刷新网页，开发板才会使用新的服务配置和嵌入页面。
是否达到 10 FPS 以完整 JPEG 的实际接收数量为准，编译成功或设置上限为 10 不代表实测通过。

## COM3 性能测量（2026-10-18）

本轮已通过 COM3 烧录 AP 测量固件，并保持 Windows 网络配置不变。测试时关闭其他相机页面，单个脚本顺序请求，串口日志与接收结果保存在 `build/perf/`。

测试顺序为：状态小包、单张 JPEG、固定 64 KiB 数据、完全停止相机后的相同数据、重新启用相机后的相同数据、连续 MJPEG。固定数据不依赖图片内容，避免把图像大小变化误当作性能改善。

第一组有效实测：

| 条件 | 接收结果 |
|---|---|
| 相机开启，状态接口 | 154 字节正文约 78 ms 完成 |
| 相机开启，单张 JPEG | 首次约 50 KB，仅收到 4320 字节后停顿，板端发送约 6981 ms 后超时 |
| 相机开启，固定 64 KiB | 约 14.3 秒仅收到 14160 字节，未完成 |
| 完全停止相机，固定 64 KiB | 两次均完整接收，分别约 563 ms、125 ms |
| 重新开启相机，固定 64 KiB | 约 22.2 秒仅收到 40960 字节，未完成 |
| 相机开启，MJPEG | 约 12 秒收到一张完整 JPEG |

发送停顿期间，CPU0 空闲率约 97%–99%，CPU1 约 99%，相机任务占用接近 0%。本轮数据不支持“后台采图耗尽 CPU”的解释。

第二组在同一固件、同一热点内进一步隔离时钟，保留相机驱动、任务与内存：

| 条件 | 固定 64 KiB 接收结果 |
|---|---|
| 20 MHz 外部时钟运行 | 约 5.6 秒仅收到 2880 字节，失败 |
| 仅暂停相机时钟 | 切换后首个请求约 1015 ms，后一个约 109 ms，均完整接收 |
| 恢复 20 MHz 时钟 | 再次失败，两个请求未收到正文 |
| 改为 16 MHz 时钟 | 两次完整接收，约 2719 ms、1218 ms，仍较慢 |
| 改为 10 MHz 时钟 | 再次发送超时 |

上述结果把范围缩小到相机时钟及其带动的硬件活动；AP 客户端信号约 −38 dBm，暂停时钟没有释放相机内存。尚不能仅凭软件测量区分具体的射频耦合与电源完整性问题。类似的频率相关 Wi-Fi 吞吐退化见 [Espressif 仓库问题 #5834](https://github.com/espressif/arduino-esp32/issues/5834)；当前板的有效参数以下述实测为准。

第三组验证 8 MHz 外部时钟，仍采用单帧缓冲：

| 条件 | 实测结果 |
|---|---|
| 固定 64 KiB | 296 ms、157 ms，均完整接收 |
| 两张约 40 KiB 的 VGA JPEG | 187 ms、125 ms，均完整，已查看图片内容 |
| 不启用内部倍频的 VGA 预览 | 约 12 秒收到 20 张完整 JPEG，约 1.66 FPS |
| 启用 OV2640 内部倍频的 VGA 预览 | 约 12 秒收到 40 张完整 JPEG，约 3.31 FPS |
| 倍频后降低画质至 40 | 约 3.42 FPS |
| 倍频后切换 QVGA | 约 3.35 FPS |
| 倍频后将发送上限改为 30 | 约 3.33 FPS |

这一组没有出现 `errno=11`；测试脚本主动关闭视频连接后出现的 `errno=104` 不计为传输超时。单帧缓冲下剩余限制更接近采集节拍，不能用降低画质或提高网页限速上限解决。

第四组固定 8 MHz 外部时钟和内部倍频，比较采集流水线：

| 条件 | 连续取帧测量 | 完整 MJPEG 接收 |
|---|---|---|
| 单缓冲，像素时钟分频 8 | 约 3.39 FPS | 约 3.31 FPS |
| 双缓冲，像素时钟分频 8 | 约 6.90 FPS | 约 6.58 FPS |
| 双缓冲，像素时钟分频 4 | 约 10.16 FPS | 119 帧 / 12 秒，约 9.92 FPS |
| 双缓冲，像素时钟分频 2 | 约 20.93 FPS | 受 10 FPS 上限限制，约 9.97 FPS |

连续取帧测量只用于比较采集节拍，不代表网络接收帧率，短窗口还可能包含已缓存的帧。最终选择分频 4，在达到目标帧率的同时保留较低的瞬时像素数据速率；大于 SVGA 的分辨率仍使用至少 12 的分频，未在本轮完成所有高分辨率档位的长期测试。

最终默认参数为：外部时钟 8 MHz、内部倍频启用、双帧缓冲、VGA 像素时钟分频 4。分辨率切换后重新应用内部倍频及对应分频，避免被驱动的分辨率配置覆盖。可在 `Camera Web Server` 中调整这些构建选项。

### 最终固件验证结果

已通过 COM3 烧录关闭临时测量功能的最终 AP 固件。启动回读确认 CPU 240 MHz、`wifi_ps=0`（`WIFI_PS_NONE`），系统动态电源管理与 STA 未连接省电在构建配置中关闭；射频功率请求值与实际回读差异见上文。

- VGA 640×480、画质 12、发送上限 10 FPS：持续约 60.079 秒接收 600 个完整 MJPEG 帧，平均 9.987 FPS，帧间稳态约 9.996 FPS。
- 所有 600 帧都检查了正文长度、JPEG 首尾及尺寸，并抽查实际图片内容；最大接收帧间隔约 141 ms。
- 同时每两秒请求 `/status`：30 次全部成功，最长 328 ms，没有五秒超时。
- 三次初始抓拍约 78–79 ms，停止视频后抓拍约 62 ms，图片完整。
- 通过 `/stop` 正常结束，板端返回 `ESP_OK`，本轮无发送超时或相机事件队列溢出。
- 分辨率切换可以生效；双缓冲可能保留两张旧分辨率图片，之后收到新分辨率，属于预缓存延迟。
- 另用 multipart 边界及 `Content-Length` 复核 5 FPS 上限：约 8.235 秒接收 41 帧，约 4.979 FPS。不能直接数 JPEG 正文中的结束标记作为视频帧数。
- 测试结束恢复 VGA、画质 12、10 FPS；临时 `/perf` 接口确认返回 404，COM3 已释放。
- 常规 AP、独立 `build-ap` 及独立 STA 构建均通过，无编译警告或错误。STA 本轮仅做编译检查，没有切换路由器进行上板测速。

原始日志和接收统计位于 `build/perf/release_verify_serial.log`、`build/perf/release_verify_results.json` 等文件；`build/` 为可清理目录，本节保留主要实测结论。测试期间未改动 Windows 网络设置。

临时测量开关位于 `Camera Web Server` → `Enable temporary performance measurements`，默认关闭。开启后每五秒打印任务运行时间，并提供 `/perf` 固定数据接口；`camera=off/on` 用于相机启停，`clock=off/on` 只暂停或恢复相机时钟，`xclk=8000000/10000000/16000000/20000000` 用于频率对比。`drive=0/2` 对比 GPIO0 输出驱动强度，`pll=0/1` 对比 OV2640 内部倍频，`buffers=1/2` 重建相机以比较缓冲数量，`dvp=2/4/8/12` 比较像素时钟分频，`sample=20` 连续取帧后只发送统计 JSON；这些仅用于本轮受控测量，不属于网页常规功能。切换前必须停止预览，测试后恢复计划保留的参数。常规固件关闭此开关后不注册该接口，也不启动运行时间采样循环。

参考：[官方驱动](https://github.com/espressif/esp32-camera)、[官方 AI-Thinker 引脚](https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h)。
