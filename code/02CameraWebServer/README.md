# ESP32-CAM 摄像头 Web 服务

独立 ESP-IDF v5.5.5 / CMake 工程，C++20。面向 AI-Thinker ESP32-CAM、OV2640、4MB PSRAM，按项目接线资料选择板型。

## 功能与默认值

- 连接现有路由器的 2.4 GHz 热点 `301`，通过 DHCP 获取局域网 IP。
- 网页端口 80，视频端口 81；控制请求不与 MJPEG 长连接共用服务任务。
- 默认 VGA（640 × 480）、JPEG 压缩值 12、发送上限 10 FPS。
- 分辨率：160 × 120、320 × 240、640 × 480、800 × 600、1024 × 768、1600 × 1200。
- JPEG 压缩值 10–63，越小画质越高。最大帧率 1–30 FPS，表示视频发送上限，不保证达到此帧率。
- 亮度、对比度、饱和度各 -2–2；支持水平镜像、垂直翻转和单张 JPEG。
- 网页显示最近统计窗口的实际发送帧率，并支持启动、停止预览。
- 设置只在本次运行有效，重启后恢复默认。当前仅面向一个视频查看者。

## Wi-Fi 配置

本地实际配置在 `include/wifi_credentials.local.h`，已加入 Git 忽略；网页和日志不输出密码。
新检出的工程先将 `include/wifi_credentials.example.h` 复制为同目录下的 `wifi_credentials.local.h`，再填写密码。
固件必须包含连接所需凭据，构建出的二进制也应按包含凭据的文件管理。

只连接 WPA2 或兼容 WPA2 的 2.4 GHz 网络，不连接仅 5 GHz 的热点。
IP 由路由器分配；需要固定地址时可在路由器中配置 DHCP 地址保留。
手机或电脑连接同一局域网，路由器需要允许设备间访问，视频还需要能访问 81 端口。

## 编译、烧录与查看 IP

在 PowerShell 中执行：

```powershell
. E:\Espressif\Initialize-Idf.ps1 -IdfId esp-idf-5c53b602a76f039e2cc64575f7074be6
Set-Location 'F:\CppProject\Smart Vehicle\code\02CameraWebServer'
idf.py build
```

首次构建通过组件管理器下载锁定版本 `espressif/esp32-camera` 2.1.8；保留 `dependencies.lock` 以复现依赖。
确认允许替换开发板现有应用后，执行：

```powershell
idf.py -p COM4 flash
idf.py -p COM4 monitor --no-reset
```

VSCode 打开根目录 `Smart Vehicle.code-workspace`，在 ESP-IDF 项目选择中选中 `02CameraWebServer` 后再编译、烧录或监视；也可直接打开本子项目目录。
Monitor 为 115200 波特率，按 `Ctrl+]` 退出。同一串口不要被多个程序同时占用。

启动连接成功后串口打印：

```text
Wi-Fi connected; open http://<路由器分配的IP>/
```

浏览器打开实际 IP，点击“开始预览”。如 Monitor 接入时错过启动日志，可在路由器客户端列表查看 IP；需要重新观察启动过程时手动按 EN / RESET。
Wi-Fi 断线后每三秒重新尝试连接，重新取得 IP 后串口再次打印地址；浏览器中的视频可手动重新开始。

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
按 UXGA 和高画质分配两个 PSRAM 帧缓冲，再将传感器设置为 VGA，支持运行时增大分辨率。
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

编译成功后，上板确认：串口打印 PSRAM 和 OV2640 信息、取得 IP、网页可见、预览正常、参数修改生效、非法值被拒绝、帧率上限有效、停止及重新预览正常、路由器恢复后可以重连。
未实际烧录前，不将上述项目记为硬件测试通过。

参考：[官方驱动](https://github.com/espressif/esp32-camera)、[官方 AI-Thinker 引脚](https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h)。
