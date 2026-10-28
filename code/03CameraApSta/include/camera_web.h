#pragma once

#include <atomic>
#include <array>
#include <mutex>
#include "esp_camera.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "motor_control.h"
#include "arm_control.h"
#include "oled_display.h"

// 固件生命周期内拥有相机、网络和 HTTP 服务；共享状态由互斥锁或原子操作保护。
class CameraWeb
{
public:
    // 输入：无；输出：唯一应用实例；作用：避免可变文件级全局对象。
    static CameraWeb& instance();
    // 输入：无；输出：初始化结果；作用：启动相机、网页与选定模式的 Wi-Fi。
    esp_err_t start();
    // 输入：无；输出：无；作用：由主任务低频采样 CPU、内存、无线信号及视频累计量。
    void updateRuntime();
    CameraWeb(const CameraWeb&) = delete;
    CameraWeb& operator=(const CameraWeb&) = delete;

private:
    CameraWeb() = default;
#if CONFIG_CAMERA_OLED_ENABLED
    // 输入：CPU 有效性与忙碌率；输出：无；作用：准备显示快照，锁外调用独立 OLED 驱动。
    void updateOled(bool isCpuValid, const std::array<double, 2>& busyPct);
    OledDisplay m_display; // 主任务独占访问的可选显示驱动，不读取相机状态。
    // 以下曝光快照由 m_statusMutex 保护，OLED 不在持锁期间发送 SPI。
    // 最近成功读取的积分时间，单位毫秒。
    double m_oledExpMs = 0;
    // 最近曝光采样的单调微秒时间，用于十秒过期判断。
    int64_t m_oledExpAtUs = 0;
    // 最近曝光寄存器采样是否有效。
    bool m_isOledExpValid = false;
    // 最近采样时的自动曝光开关。
    bool m_isOledAuto = true;
#endif
    // 输入：无；输出：驱动结果；作用：配置 AI-Thinker 引脚与 PSRAM 帧缓冲。
    esp_err_t initCamera();
    // 输入：无；输出：结果；作用：配置 GPIO4 补光灯，启动默认关闭。
    esp_err_t initLed();
    // 输入：开关表单；输出：HTTP 结果；作用：设置补光灯并返回确认后的软件状态。
    esp_err_t setLed(httpd_req_t* req);
    // 输入：已初始化的 OV2640；输出：设置结果；作用：分辨率变化后恢复低外部时钟的内部倍频。
    esp_err_t applySensorClock(sensor_t* sensor);
    // 输入：相机帧；输出：结构是否有效；作用：校验原生 JPEG 表、分量和扫描边界。
    static bool isValidJpeg(const camera_fb_t* frame);
    // 输入：无；输出：恢复结果；作用：保留当前图像参数重新设置传感器。
    esp_err_t recoverCamera();
    // 输入：无；输出：初始化结果；作用：同时启动本地 AP 和路由器 STA 接口。
    esp_err_t initWifi();
    // 输入：无；输出：启动结果；作用：分离网页、视频和运动三个服务任务。
    esp_err_t startServers();
    // 输入：服务及新连接；输出：配置结果；作用：为网页和视频连接禁用小包合并等待。
    static esp_err_t onSocketOpen(httpd_handle_t server, int socketFd);
    // 输入：事件和实例；输出：无；作用：维护网络就绪状态，打印 IP 并处理断线。
    static void onWifiEvent(void* arg, esp_event_base_t base, int32_t id, void* data);
    // 输入：实例；输出：无；作用：间隔三秒尝试重新连接路由器。
    static void reconnectWifi(void* arg);
    // 输入：HTTP 请求；输出：处理结果；作用：分派短时控制请求。
    static esp_err_t onRequest(httpd_req_t* req);
    // 输入：独立运动请求；输出：HTTP 结果；作用：校验网页来源并直接读写电机状态。
    static esp_err_t onMotor(httpd_req_t* req);
    // 输入：无；输出：无；作用：网络事件非阻塞通知各控制模块，使车轮停车并作废旧舵机指令。
    void requestMotionStop();
    // 输入：机械臂 GET/POST；输出：状态或命令结果；作用：复用运动端口的来源校验，不占用相机锁。
    static esp_err_t onArm(httpd_req_t* req);
    // 输入：请求；输出：机械臂 JSON；作用：返回目标、输出状态及软件限位，不宣称实际到位。
    esp_err_t sendArm(httpd_req_t* req);
    // 输入：角度或释放表单；输出：结果；作用：校验完整请求后执行一次明确动作。
    esp_err_t setArm(httpd_req_t* req);
    // 输入：升级请求；输出：是否允许；作用：在握手前校验同设备来源及单控制连接。
    static esp_err_t openMotorSocket(httpd_req_t* req);
    // 输入：二进制指令；输出：处理结果；作用：接收定时推送，不逐条同步回报状态。
    static esp_err_t onMotorSocket(httpd_req_t* req);
    // 输入：服务及连接；输出：无；作用：控制连接关闭时立即投递停车，再关闭 socket。
    static void closeMotorSocket(httpd_handle_t server, int socketFd);
    // 输入：实例；输出：无；作用：定时向运动服务投递一次连接巡检，不跨任务访问连接字段。
    static void queueMotionCheck(void* arg);
    // 输入：实例；输出：无；作用：在运动 HTTP 任务中回收三秒无有效输入的控制连接。
    static void checkMotionSocket(void* arg);
    // 输入：请求及是否必须有来源；输出：是否匹配；作用：只允许同设备 80 端口网页。
    static bool isMotorOrigin(httpd_req_t* req, bool required);
    // 输入：连接、序号和执行结果；输出：发送结果；作用：一次非阻塞发送紧凑状态，拥塞时合并。
    esp_err_t reportMotor(int socketFd, uint32_t sequence, esp_err_t result);
    // 输入：输出缓冲、容量、机械臂快照、序号及结果；输出：JSON 长度；作用：统一 HTTP 和长连接状态格式。
    static int formatArm(char* text, size_t capacity, const ArmState& state, uint32_t sequence, esp_err_t result);
    // 输入：连接、序号及结果；输出：发送结果；作用：在现有控制连接上发送独立机械臂状态。
    esp_err_t reportArm(int socketFd, uint32_t sequence, esp_err_t result);
    // 输入：连接及文本；输出：发送结果；作用：单次发送完整 WS 帧，支持扩展长度。
    esp_err_t sendSocketText(int socketFd, const char* text, size_t length);
    // 输入：HTTP 请求；输出：传输结果；作用：发送有帧率上限的 MJPEG 视频。
    static esp_err_t onStream(httpd_req_t* req);
    // 输入：请求；输出：发送结果；作用：返回线程安全的参数及帧率快照。
    esp_err_t sendStatus(httpd_req_t* req);
    // 输入：无；输出：无；作用：在相机锁内发布快照，常规曝光采样间隔至少两秒。
    void cacheStatus();
    // 输入：已初始化的传感器；输出：读取是否有效；作用：读取硬件曝光及行时序，不依赖驱动缓存。
    bool readExposure(sensor_t* sensor);
    // 输入：传感器和自动模式；输出：切换是否成功；作用：冻结当前曝光或恢复自动曝光。
    bool setAutoExposure(sensor_t* sensor, bool enabled);
    // 输入：传感器和总曝光行数；输出：写入是否成功；作用：手动设置快门及额外曝光行。
    bool setExposure(sensor_t* sensor, int lines);
#if CONFIG_CAMERA_PERF_TEST
    // 输入：含 camera=off/on 的测试请求；输出：发送结果；作用：对比相机开关前后的固定数据传输。
    esp_err_t sendPerf(httpd_req_t* req);
#endif
    // 输入：表单请求；输出：HTTP 结果；作用：校验范围并设置相机参数。
    esp_err_t setControl(httpd_req_t* req);
    // 输入：三轴百分数和代次表单；输出：电机 JSON；作用：控制车辆或无条件停车。
    esp_err_t setMotor(httpd_req_t* req);
    // 输入：请求；输出：电机快照；作用：运动回复不等待相机互斥锁。
    esp_err_t sendMotor(httpd_req_t* req);
    // 输入：参数名和值；输出：是否设置成功；作用：映射允许的参数和分辨率。
    bool applySetting(const char* name, int value);
    // 输入：无；输出：无；作用：统一重连或停止后的帧率清理。
    void finishStream();

    // GPIO4 配置是否成功；补光灯失败不阻止相机和运动服务启动。
    std::atomic<bool> m_ledReady{false};
    // 最后成功写入的补光灯开关，不表示物理发光反馈。
    std::atomic<bool> m_ledOn{false};
    // 补光灯初始化或写入错误，用于页面明确显示不可用。
    std::atomic<esp_err_t> m_ledFault{ESP_OK};
    // 保护相机采集、传感器设置及运行统计，网络发送不持有此锁。
    mutable std::mutex m_mutex;
    // 保护相机/运行状态快照和视频累计量；持锁期间不取帧、不访问外设、不发送网络。
    mutable std::mutex m_statusMutex;
    // 缓存相机字段，发送时追加实时网络、电机字段及结束括号；空串表示相机不可用。
    std::array<char, 512> m_statusJson{};
    // 主任务发布的运行状态 JSON；状态请求直接复制，不等待相机或统计采样。
    std::array<char, 512> m_runtimeJson{};
    // 以下四项由快照锁保护；仅完整提交 HTTP 的 JPEG 帧计入累计量，跨连接不清零。
    // 成功提交的 JPEG 有效载荷总字节数，不包含 HTTP 头和链路重传。
    uint64_t m_videoBytes = 0;
    // 成功提交的完整 JPEG 帧数。
    uint64_t m_videoFrames = 0;
    // 已完成发送帧的取帧等待累计耗时，单位微秒。
    uint64_t m_captureUs = 0;
    // 已完成发送帧的 HTTP 提交累计耗时，单位微秒。
    uint64_t m_sendUs = 0;
    // 视频连接是否处于有效处理阶段，由视频任务在快照锁内更新。
    bool m_isVideoActive = false;
    // 最近运行快照的单调时间，单位微秒；由快照锁保护，零表示尚未采样。
    int64_t m_runtimeAtUs = 0;
    // 以下三项仅由主任务访问，不与视频或 HTTP 任务共享。
    // 上次 CPU 采样的单调时间，单位微秒；第一轮只建立计数基线。
    int64_t m_cpuAtUs = 0;
    // 上次两个核心的空闲任务累计微秒数，使用无符号差值处理 32 位回绕。
    std::array<uint32_t, 2> m_idleLast{};
    // 上次视频窗口累计值，顺序为字节数、完整帧数、取帧微秒数、发送微秒数。
    std::array<uint64_t, 4> m_videoLast{};
    // 以下曝光字段由相机锁保护；状态接口仅尝试取锁，视频忙时继续返回已发布快照。
    // 最近尝试读取曝光的单调时间，单位微秒；负初值保证启动时立即采样。
    int64_t m_expReadUs = -2000000;
    // 根据 OV2640 行宽、时钟分频和倍频换算的行周期估算值，单位微秒。
    double m_lineUs = 0;
    // 最近硬件快门的有效行数，不包括额外曝光行。
    int m_expLines = 0;
    // VSYNC 前增加的曝光行数，与快门行数共同决定积分时间。
    int m_extraLines = 0;
    // 当前模式及帧长度允许的手动快门上限，同时受驱动 1200 行限制。
    int m_maxShutter = 0;
    // 当前手动调节周期的总行数上限；缩短曝光不反复缩小输入范围。
    int m_expMaxLines = 0;
    // 最近成功读取的硬件自动曝光状态，启动保持驱动默认自动模式。
    bool m_isAutoExposure = true;
    // 曝光寄存器和时序是否读取成功；无效时界面不显示历史值为当前值。
    bool m_isExposureValid = false;
    // 自动模式原有的 DSP 曝光使能位，恢复自动曝光时原样恢复。
    int m_autoDsp = 0;
    // 自动模式原有的夜间额外帧设置，手动模式冻结，恢复时原样恢复。
    int m_autoFrames = 0;
    // 先于控制模块创建、晚于它们销毁，唯一拥有共享 GPIO14 总线事务锁。
    Pca9685Bus m_pcaBus;
    // 独立管理关节目标与限位，和车轮借用同一总线实例。
    ArmControl m_arm{m_pcaBus};
    // 独立维护轮状态与停车任务；销毁时先结束任务，再销毁共享总线。
    MotorControl m_motor{m_pcaBus};
    // 控制端口服务器，生命周期与固件一致。
    httpd_handle_t m_control = nullptr;
    // 视频端口服务器，生命周期与固件一致。
    httpd_handle_t m_stream = nullptr;
    // 82 端口运动服务，接收任务独立于相机，优先级高于网页和视频 HTTP 任务。
    httpd_handle_t m_motion = nullptr;
    // 以下连接字段仅由 82 端口 HTTP 任务读写，其他线程不访问，关闭时清空。
    int m_motorFd = -1;
    // 最后有效控制帧或握手的单调时间，单位微秒，仅由运动 HTTP 任务访问。
    int64_t m_motorRxUs = 0;
    // 失联连接已提交关闭，等待关闭回调前拒绝该连接继续执行命令。
    bool m_motorClosing = false;
    // 当前连接最后接收的递增序号，用于拒绝重放和错误排序。
    uint32_t m_motorSeq = 0;
    // 最近一次成功排入 TCP 的状态时间，单位微秒；正常运动每 500 ms 合并回报。
    int64_t m_reportUs = 0;
    // 最近回报的停车代次；变化时立即尝试发布状态。
    uint32_t m_reportGen = 0;
    // 最近已回报的运动状态，开始运动时立即尝试反馈，不等待下个周期。
    bool m_isReportedMoving = false;
    // 每 500 ms 投递一次巡检，固件生命周期内保持运行。
    esp_timer_handle_t m_motionTimer = nullptr;
    // 防止 HTTP 任务繁忙时不断积压巡检工作；定时器与 HTTP 任务共享。
    std::atomic<bool> m_checkQueued{false};
    // 路由器断线后三秒重试，热点运行不阻止 STA 重连。
    esp_timer_handle_t m_retryTimer = nullptr;
    // Wi-Fi 事件注册句柄，保证回调上下文生命周期稳定。
    esp_event_handler_instance_t m_wifiEvent = nullptr;
    // IP 事件注册句柄。
    esp_event_handler_instance_t m_ipEvent = nullptr;
    // 热点接口是否启动，独立于路由器连接；由 Wi-Fi 事件任务更新。
    std::atomic<bool> m_apReady{false};
    // STA 是否已取得路由器 IP，重连判断不能使用综合网络就绪标志。
    std::atomic<bool> m_staReady{false};
    // 路由器分配的 IPv4 网络字节序地址；未取得或失去地址时为零。
    std::atomic<uint32_t> m_staIp{0};
    // 任一接口可用即允许服务；STA 断线不能关闭仍运行的热点。
    std::atomic<bool> m_connected{false};
    // 每次停止请求或断网递增，旧视频循环发现变化后退出。
    std::atomic<unsigned> m_epoch{0};
    // 视频发送上限，单位帧每秒；受 m_mutex 保护。
    int m_maxFps = 30;
    // 最近统计窗口的发送帧率，单位帧每秒；受 m_mutex 保护。
    double m_actualFps = 0;
    // 当前分辨率的网页选项编号，显式映射到驱动枚举。
    int m_size = 2;
    // 相机帧缓冲数量；正常使用构建值，测量模式仅在停止预览后允许重建驱动来切换。
    int m_frameCount = CONFIG_CAMERA_FRAME_BUFFERS;
    // 当前是否正在传输视频；受 m_mutex 保护。
    bool m_streaming = false;
    // 累计被拒绝的损坏 JPEG 数；抓拍和视频任务并发更新，运行状态按快照读取。
    std::atomic<uint32_t> m_jpegErrors{0};
};
