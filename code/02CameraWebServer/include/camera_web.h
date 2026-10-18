#pragma once

#include <atomic>
#include <mutex>
#include "esp_camera.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_timer.h"

// 固件生命周期内拥有相机、网络和 HTTP 服务；共享状态由互斥锁或原子操作保护。
class CameraWeb
{
public:
    // 输入：无；输出：唯一应用实例；作用：避免可变文件级全局对象。
    static CameraWeb& instance();
    // 输入：无；输出：初始化结果；作用：启动相机、网页与路由器连接。
    esp_err_t start();
    CameraWeb(const CameraWeb&) = delete;
    CameraWeb& operator=(const CameraWeb&) = delete;

private:
    CameraWeb() = default;
    // 输入：无；输出：驱动结果；作用：配置 AI-Thinker 引脚与 PSRAM 帧缓冲。
    esp_err_t initCamera();
    // 输入：无；输出：初始化结果；作用：注册网络事件并启动 STA 模式。
    esp_err_t initWifi();
    // 输入：无；输出：启动结果；作用：分离控制端口和长连接视频端口。
    esp_err_t startServers();
    // 输入：事件和实例；输出：无；作用：打印 IP 并安排断线重连。
    static void onWifiEvent(void* arg, esp_event_base_t base, int32_t id, void* data);
    // 输入：实例；输出：无；作用：间隔三秒尝试重新连接路由器。
    static void reconnectWifi(void* arg);
    // 输入：HTTP 请求；输出：处理结果；作用：分派短时控制请求。
    static esp_err_t onRequest(httpd_req_t* req);
    // 输入：HTTP 请求；输出：传输结果；作用：发送有帧率上限的 MJPEG 视频。
    static esp_err_t onStream(httpd_req_t* req);
    // 输入：请求；输出：发送结果；作用：返回线程安全的参数及帧率快照。
    esp_err_t sendStatus(httpd_req_t* req);
    // 输入：表单请求；输出：HTTP 结果；作用：校验范围并设置相机参数。
    esp_err_t setControl(httpd_req_t* req);
    // 输入：参数名和值；输出：是否设置成功；作用：映射允许的参数和分辨率。
    bool applySetting(const char* name, int value);
    // 输入：无；输出：无；作用：统一重连或停止后的帧率清理。
    void finishStream();

    // 保护相机采集、传感器设置及运行统计，网络发送不持有此锁。
    mutable std::mutex m_mutex;
    // 控制端口服务器，生命周期与固件一致。
    httpd_handle_t m_control = nullptr;
    // 视频端口服务器，生命周期与固件一致。
    httpd_handle_t m_stream = nullptr;
    // 延迟重连定时器，由网络事件启停。
    esp_timer_handle_t m_retryTimer = nullptr;
    // Wi-Fi 事件注册句柄，保证回调上下文生命周期稳定。
    esp_event_handler_instance_t m_wifiEvent = nullptr;
    // IP 事件注册句柄。
    esp_event_handler_instance_t m_ipEvent = nullptr;
    // 当前网络是否取得有效 IP，由事件任务更新。
    std::atomic<bool> m_connected{false};
    // 每次停止请求或断网递增，旧视频循环发现变化后退出。
    std::atomic<unsigned> m_epoch{0};
    // 视频发送上限，单位帧每秒；受 m_mutex 保护。
    int m_maxFps = 10;
    // 最近统计窗口的发送帧率，单位帧每秒；受 m_mutex 保护。
    double m_actualFps = 0;
    // 当前分辨率的网页选项编号，显式映射到驱动枚举。
    int m_size = 2;
    // 当前是否正在传输视频；受 m_mutex 保护。
    bool m_streaming = false;
};
