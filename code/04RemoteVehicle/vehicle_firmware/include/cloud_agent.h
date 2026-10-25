#pragma once
#include "esp_websocket_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <atomic>
#include <array>
#include <string>

// 有界接收消息；连接代次阻止断线前的数据在新连接继续执行。
struct CloudPacket
{
    uint32_t epoch = 0; // 收到消息时的控制连接代次。
    std::array<char, 1024> text{}; // 最长 1023 字节 JSON 命令，含结束符。
};

// 两条独立 TLS WebSocket：控制任务和视频任务互不等待发送锁。
class CloudAgent
{
public:
    // 输入：无；输出：唯一实例；作用：保证 SDK 回调上下文在固件整个生命周期有效。
    static CloudAgent& instance();
    // 输入：无；输出：任务创建结果；作用：等待 STA 和时间同步后启动云端链路。
    esp_err_t start();
private:
    CloudAgent() = default;
    // 输入：实例；输出：无；作用：校时、连接、心跳与有界命令消费。
    static void controlTask(void* arg);
    // 输入：实例；输出：无；作用：保持默认画质和帧率上限，慢网络不补发旧帧。
    static void videoTask(void* arg);
    // 输入：SDK 事件；输出：无；作用：仅组装命令并投递队列，不在回调里操作硬件。
    static void onControl(void* arg, esp_event_base_t base, int32_t id, void* data);
    // 输入：SDK 事件；输出：无；作用：维护视频连接状态。
    static void onVideo(void* arg, esp_event_base_t base, int32_t id, void* data);
    // 输入：JSON 命令；输出：无；作用：校验有效期和控制代次后执行业务。
    void execute(const char* text);
    // 输入：JSON 文本；输出：是否完整发出；作用：超时后断开控制链路并撤销运动。
    bool sendText(const std::string& text);
    esp_websocket_client_handle_t m_control = nullptr; // 控制客户端，由控制任务管理。
    esp_websocket_client_handle_t m_video = nullptr; // 视频客户端，由视频任务发送。
    QueueHandle_t m_queue = nullptr; // 容量四项；满时撤销运动并丢弃命令。
    std::string m_headers; // 长期有效的设备认证头，不输出到日志。
    std::atomic<bool> m_hasControl{false}; // SDK 回调发布的控制连接状态。
    std::atomic<bool> m_hasVideo{false}; // SDK 回调发布的视频连接状态。
    std::atomic<bool> m_hasStop{false}; // 回调只投递停止，业务任务实际停机。
    std::atomic<uint32_t> m_epoch{1}; // 每次控制断线或接收异常递增。
    std::array<char, 1024> m_rx{}; // SDK 控制事件任务独占的分段接收缓冲。
    size_t m_rxSize = 0; // 已拼接字节数，必须和分段偏移一致。
    uint32_t m_lease = 0; // 仅控制任务访问的当前网页控制代次。
};
