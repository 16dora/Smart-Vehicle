# ESP32 串口通信测试

目标：ESP32，板载 CH340，COM4，115200 波特率，8N1，无流控。
UART0 使用 GPIO1 (TX) / GPIO3 (RX)，通过现有 USB 连接即可测试。

## 编译与烧录

在 PowerShell 中执行：

```powershell
. E:\Espressif\Initialize-Idf.ps1 -IdfId esp-idf-5c53b602a76f039e2cc64575f7074be6
Set-Location 'F:\CppProject\Smart Vehicle\code\01SerialLoopBackTest'
idf.py build
idf.py -p COM4 flash
python serial_test.py --port COM4
```

烧录会替换板上原有应用。测试结束后脚本自动释放串口。
不要同时打开串口助手、监视器和自动测试脚本。

## 手动观察

执行 `idf.py -p COM4 monitor --no-reset`，按 Ctrl+] 退出。
也可以使用串口助手，设置 115200 / 8N1 / 无流控，发送末尾添加 LF 或 CRLF。

- 每秒输出 `HEARTBEAT seq=... uptime_ms=...`。
- 发送 `ping` 加换行，收到 `ECHO ping`。
- 单行最多 127 字节，超长行收到错误；随后可继续通信。
- 自动脚本验证普通回显、127 字节边界、超长处理与恢复，以及至少三次心跳。

应用关闭 ESP-IDF 标准控制台，避免日志混入测试协议；启动阶段仍可能看到 ROM/bootloader 日志。
该测试验证电脑、USB、CH340 和 ESP32 UART0 的双向链路，不覆盖 Wi-Fi、蓝牙或其他外设。

## 目录

- `CMakeLists.txt`：ESP-IDF 工程入口。
- `source/CMakeLists.txt`：源码组件注册。
- `source/main.c`：心跳与逐行回显。
- `include/`：预留头文件。
- `sdkconfig.defaults`：默认芯片与控制台配置。
- `serial_test.py`：电脑端自动通信检查。
