# 04 远程小车控制

本项目将 03 的控制页面接入 Ubuntu 云端。控制页面的 HTML、CSS、控件和操作方式保持原样，仅修改通信脚本；独立登录页使用指定账号，所有图像和控制接口均要求有效登录。

## 目录

- `cloud_server/`：C++20 / Drogon 服务、CMake、原控制页和登录页。
- `cloud_server/deploy/`：Nginx、systemd、证书续期及 Visual Studio 调试示例。
- `vehicle_firmware/`：独立 ESP-IDF 5.5.5 工程，基于 03，新增云端客户端，不引用其他子项目的构建目录。
- `tools/remote_build.ps1`：本地打包、上传、远程构建和部署。
- `.local/`：本机凭证及临时验证产物，已忽略。

## 已部署入口

- 登录及控制：<https://124.223.192.76/>。
- 退出页面：<https://124.223.192.76/logout>。为保持控制页布局一致，退出操作放在独立地址。
- 初始账号：`smart`；初始密码使用本轮用户指定值，不写入源码、网页或文档。
- 公网 TCP 80 用于跳转和 ACME 证书验证，TCP 443 提供 HTTPS/WSS。
- 后端只监听 `127.0.0.1:8000`，不需要公开后端或小车本地的 81/82 端口。

SQLite 账号数据库在 `/var/lib/smart-vehicle/users.db`，保存 Argon2id 密码哈希；会话在内存中保存，重启或退出后需重新登录。Cookie 使用 Secure、HttpOnly、SameSite=Strict，所有修改请求和浏览器 WebSocket 验证 Origin。设备使用独立随机凭证，与网页登录密码无关；不实现用户与车辆绑定。

## 服务器开发环境

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build pkg-config git gdb rsync zip nginx \
  libdrogon-dev drogon libsodium-dev libsqlite3-dev libjsoncpp-dev uuid-dev \
  zlib1g-dev libssl-dev libpq-dev libmariadb-dev libbrotli-dev libhiredis-dev libyaml-cpp-dev
```

仓库 Drogon 的 CMake 导出包含数据库、Brotli 和 YAML 模块依赖，因此需要相应开发包；应用只使用 SQLite 账号库，不启动数据库服务器。该仓库 Trantor 未启用 TLS，公网 TLS 由 Nginx 处理。

## 本地源码、远程构建

在 Windows PowerShell 从项目根目录执行：

```powershell
& 'code/04RemoteVehicle/tools/remote_build.ps1'
& 'code/04RemoteVehicle/tools/remote_build.ps1' -Deploy
```

脚本只同步 `cloud_server/` 源码，不同步运行数据库、固件 Wi-Fi 凭证或设备密钥。远程源码在 `/home/ubuntu/smart-vehicle-cloud/src`，脚本构建在 `/home/ubuntu/smart-vehicle-cloud/build`。安装目标为 `/opt/smart-vehicle`，不会初始化或覆盖账号数据库及设备密钥。

首次安装必须先创建 `/etc/smart-vehicle/cloud.env` 和账号数据库。环境配置键为 `SV_DATABASE`、`SV_WEB_ROOT`、`SV_ORIGIN`、`SV_DEVICE_HASH`。最后一个值为 64 字符设备密钥文本的 SHA256 十六进制摘要；原始密钥仅存于本机忽略文件和固件，不放入网页目录或日志。

管理员创建或修改账号时，从 SSH 终端执行：

```bash
read -rsp '新密码: ' SV_INPUT_PASSWORD; printf '\n'
printf '%s\n' "$SV_INPUT_PASSWORD" | sudo -n env SV_DATABASE=/var/lib/smart-vehicle/users.db \
  /opt/smart-vehicle/bin/smart_vehicle_cloud --set-password smart
unset SV_INPUT_PASSWORD
sudo systemctl restart smart-vehicle
```

密码经标准输入读取，不出现在进程命令行中。重启撤销现有会话。

## Visual Studio

1. 安装“使用 C++ 的 Linux 和嵌入式开发”及 CMake 支持。
2. 打开本地 `cloud_server` 文件夹，不打开 ESP-IDF 固件目录作为服务端工程。
3. 在连接管理器添加 `ubuntu@124.223.192.76:22`，使用已有 SSH 认证。
4. 选择该远程目标和 `linux-debug` / `linux-release` 预设。IDE 自动同步源码并在 Ubuntu 构建。
5. `CMakePresets.json` 已提供远程目录映射；命令行也可以在远端源码目录运行 `cmake --preset linux-debug` 和 `cmake --build --preset linux-debug`。

需要 F5 断点调试时，先在远端执行：

```bash
sudo bash /home/ubuntu/smart-vehicle-cloud/src/deploy/prepare_debug.sh
sudo systemctl stop smart-vehicle
```

这会建立独立调试账号库和配置，不覆盖生产数据库。按 Visual Studio 的“调试和启动设置”创建配置，将 `deploy/launch.vs.json.example` 的参数用于 `smart_vehicle_cloud` 目标；保留 IDE 自动生成的 `projectTarget` 值。程序参数为 `--env-file /home/ubuntu/smart-vehicle-cloud/debug/debug.env`，GDB 和程序均在 Ubuntu 运行。调试期间 Nginx 继续代理 8000；同一端口只能运行一个实例。调试结束后执行 `sudo systemctl start smart-vehicle` 恢复常驻服务。

## HTTPS 证书

当前已使用 Let's Encrypt 为公网 IP 签发受信任证书。IP 证书有效期约六天，通过独立 Certbot 5.8 虚拟环境及 `smart-vehicle-cert.timer` 每天两次检查续期，续期成功后重载 Nginx。TCP 80 的 `/.well-known/acme-challenge/` 必须保持可达。

```bash
systemctl status smart-vehicle-cert.timer
sudo /opt/smart-vehicle-certbot/bin/certbot renew --dry-run
```

私钥位于服务器 `/etc/letsencrypt` 和 `/etc/smart-vehicle/server.key`，不复制到本地或固件。固件使用 ESP-IDF 内置可信根验证服务器身份，不固定短期叶证书。

## 小车固件

默认保持 VGA 640×480、JPEG 压缩值 12、发送上限 10 FPS。`include/wifi_credentials.local.h` 从本机 03 凭证复制；`include/cloud_credentials.local.h` 是本轮生成的设备凭证，两者均已忽略。其他电脑需按照 `.example.h` 创建对应本机文件，并让云端设备密钥摘要匹配。

```powershell
. E:\Espressif\Initialize-Idf.ps1 -IdfId esp-idf-5c53b602a76f039e2cc64575f7074be6
Set-Location 'F:\CppProject\Smart Vehicle\code\04RemoteVehicle\vehicle_firmware'
idf.py build
```

构建不烧录。准备好硬件后按实际串口执行 `idf.py -p COM端口 flash`。OLED 接线和烧录注意事项沿用 03；01、02、03 工程未修改。

STA 联网和 NTP 校时成功后建立两条 WSS，分别传输控制和 JPEG。控制接收采用四项有界队列，超过期限、旧序号和旧控制租约的运动指令不执行；断线不恢复旧动作。车轮沿用 500 ms 本地看门期限，机械臂失联停止推进并保持已输出角度。04 的本地 HTTP 修改接口及运动 WS 关闭，避免绕过云端登录；AP 保留只读诊断。云端视频工作时本地 MJPEG 拒绝争抢采集器。

## 协议和运行

- `/device/control`、`/device/video`：使用 `X-Device-Key` 认证的设备连接。
- `/motor/ws`：沿用原网页十二字节二进制帧，只有一个控制页面可持有连接。
- `/view/video`：浏览器接收二进制 JPEG，解码完成发送 `next`；服务只保存最新一帧，慢查看者不会积压历史画面。
- `/status`、`/control`、`/motor`、`/arm`、`/led`：经登录验证后转发，由设备实际应答后返回结果。
- `/capture`：返回两秒内最新 JPEG；离线或画面过期返回 503。
- `/api/login`、`/api/logout`：账号登录和会话撤销。

设备离线时页面明确显示错误，所有硬件状态未知且按钮禁用；不存在伪造运行状态或自动模拟车辆。协议连通性验证可以使用短暂模拟设备，但不能代表实际摄像头、Wi-Fi、机械臂或电机已上板验证。

```bash
systemctl status smart-vehicle
sudo journalctl -u smart-vehicle -n 50 --no-pager
```

## 本轮验证记录（2026-10-25）

- Ubuntu G++ 15.2 / CMake 4.2.3 / Drogon 1.9.11 实际构建、部署并运行。
- `linux-debug` CMake Preset 在远端实际构建成功；GDB 在 `main` 源码断点停止。
- 真实浏览器通过 HTTPS 登录和退出成功，登录后为原 03 控制界面。
- 对比确认控制页 `<script>` 之前的 HTML/CSS 与 03 完全一致。
- 协议探针验证匿名访问拦截、错误密码、Cookie、Origin 校验、二进制运动转发、真实设备应答匹配、JPEG 转发及退出撤销；探针使用临时模拟设备，完成后已断开。
- Let's Encrypt 正式 IP 证书签发成功，续期预演成功，定时器已启用。
- ESP-IDF 5.5.5 编译成功，固件约 1.28 MiB，应用分区剩余约 13%。
- 未烧录板卡，未执行真实电机或机械臂动作；硬件帧率、TLS 内存余量及公网实测延迟仍需上板确认。
