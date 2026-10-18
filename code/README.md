# ESP32 固件子项目

每个可独立编译、烧录的工程放在 `code/` 下独立的子文件夹中。
当前工程：

- `01SerialLoopBackTest/`：UART0 心跳与逐行回显。
- `02CameraWebServer/`：AI-Thinker ESP32-CAM 摄像头网页、MJPEG 预览及图像设置。

后续工程沿用编号加功能名称的目录形式，例如 `03GpioTest/`。
每个工程分别维护 `CMakeLists.txt`、`source/`、`include/`、
`sdkconfig.defaults`、README 以及所需测试脚本；`sdkconfig` 和 `build/` 也各自独立。

编译、烧录、Monitor 命令必须在选定子项目目录中执行，不能直接在 `code/` 执行。
VSCode 可以直接打开子项目文件夹，或使用根目录的 `Smart Vehicle.code-workspace`。
后续加入其他子项目时，在工作区中添加对应文件夹，并选择需要操作的工程。

当前工程操作步骤见 [01SerialLoopBackTest/README.md](01SerialLoopBackTest/README.md)。
摄像头工程操作步骤见 [02CameraWebServer/README.md](02CameraWebServer/README.md)。
