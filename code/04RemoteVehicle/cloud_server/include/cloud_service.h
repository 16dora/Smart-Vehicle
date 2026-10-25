#pragma once

#include <drogon/drogon.h>
#include <drogon/WebSocketController.h>
#include <drogon/HttpFilter.h>
#include <sqlite3.h>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>

namespace vehicle
{
// HTTP 异步应答类型，网络回调可以由设备应答或超时巡检完成。
using Reply = std::function<void(const drogon::HttpResponsePtr&)>;
// 使用单调时钟限制会话、设备心跳和请求期限，不受系统校时影响。
using Clock = std::chrono::steady_clock;

// 登录会话只保存随机凭证及到期时刻，不保存用户原始密码。
struct Session
{
    Clock::time_point expires; // 会话绝对到期时刻，最长八小时。
};
// 短请求等待设备实际应答，绝不以已转发冒充已执行。
struct Pending
{
    Reply reply; // 最多调用一次的 HTTP 应答。
    Clock::time_point expires; // 三秒后不再接受迟到应答。
    std::string session; // 撤销会话后取消尚未完成的命令。
};
// WebSocket 上下文由框架持有；关闭回调仍需核对是否当前连接。
struct Link
{
    std::string path; // 区分设备控制、设备视频、浏览器运动和浏览器视频。
    std::string session; // 浏览器所属登录会话；设备连接留空。
    bool ready = true; // 视频仅在上一帧浏览器解码完成后才允许下一帧。
    uint64_t frame = 0; // 该浏览器最近收到的图像序号。
    Clock::time_point sent; // 超过五秒未确认则关闭慢速视频连接。
};

// 云端单车服务；所有共享连接和状态由同一个互斥锁保护。
class CloudService
{
public:
    // 输入：无；输出：服务实例；作用：供 Drogon 控制器访问固定生命周期服务。
    static CloudService& instance();
    // 输入：数据库、网页目录和公开来源；输出：无；作用：配置运行资源。
    void configure(const std::string& db, const std::string& web, const std::string& origin,
                   const std::string& tokenHash);
    // 输入：账号及密码；输出：是否成功；作用：仅管理命令创建或修改密码哈希。
    bool setPassword(const std::string& user, const std::string& password);
    // 输入：HTTP 请求及应答；输出：异步响应；作用：登录、鉴权、网页和设备短请求分派。
    void handle(const drogon::HttpRequestPtr& req, Reply&& reply);
    // 输入：请求；输出：是否允许升级；作用：握手前验证会话或设备凭证。
    bool allowSocket(const drogon::HttpRequestPtr& req);
    // 输入：请求与连接；输出：无；作用：绑定设备或浏览器身份。
    void opened(const drogon::HttpRequestPtr& req, const drogon::WebSocketConnectionPtr& socket);
    // 输入：连接、完整消息和类型；输出：无；作用：转发有期限的控制及有背压的图像。
    void received(const drogon::WebSocketConnectionPtr& socket, const std::string& data,
                  drogon::WebSocketMessageType type);
    // 输入：关闭连接；输出：无；作用：当前控制者失联立即撤销运动。
    void closed(const drogon::WebSocketConnectionPtr& socket);
    // 输入：无；输出：无；作用：每百毫秒清理过期会话、请求和断流。
    void tick();

private:
    CloudService() = default;
    // 输入：随机字节数；输出：十六进制凭证；作用：生成不可预测的会话和控制代次。
    static std::string randomToken(size_t bytes);
    // 输入：连接；输出：无；作用：延迟关闭，避免同线程关闭回调重入共享锁。
    static void closeSocket(const drogon::WebSocketConnectionPtr& socket);
    // 输入：请求；输出：会话编号；作用：只接受服务器仍保留的有效 Cookie。
    std::string sessionOf(const drogon::HttpRequestPtr& req);
    // 输入：会话；输出：是否有效；作用：锁内检查会话到期。
    bool validSession(const std::string& session) const;
    // 输入：无；输出：是否在线；作用：在线需有最近五秒内的设备控制心跳。
    bool online() const;
    // 输入：状态码与说明；输出：响应；作用：统一错误、缓存和安全响应头。
    static drogon::HttpResponsePtr response(int code, const std::string& body,
                                            const std::string& type = "application/json");
    // 输入：网页文件名；输出：响应；作用：只读取明确允许的文件，不公开整个文件目录。
    drogon::HttpResponsePtr page(const std::string& file);
    // 输入：说明；输出：无；作用：锁内作废当前控制代次并通知设备停车。
    void stopControl(const std::string& reason);
    // 输入：会话；输出：无；作用：锁内终止会话拥有的长连接。
    void revoke(const std::string& session);
    // 输入：协议对象；输出：无；作用：锁内向当前设备发送命令。
    void sendDevice(const Json::Value& value);
    // 输入：请求与应答；输出：无；作用：限额等待真实设备 HTTP 业务结果。
    void forward(const drogon::HttpRequestPtr& req, Reply&& reply, const std::string& session);
    // 输入：连接；输出：无；作用：锁内按最新帧及确认窗口发送单张 JPEG。
    void sendFrame(const drogon::WebSocketConnectionPtr& socket);

    std::unique_ptr<sqlite3, decltype(&sqlite3_close)> m_db{nullptr, sqlite3_close}; // 账号数据库唯一所有者。
    std::string m_web; // 网页资源目录，不包含数据库和密钥。
    std::string m_origin; // 唯一公开 HTTPS 来源，验证表单和 WebSocket 来源。
    std::string m_tokenHash; // 随机设备密钥的 SHA256 十六进制摘要。
    std::mutex m_mutex; // 两个网络线程与定时巡检共享状态锁。
    std::map<std::string, Session> m_sessions; // 有限数量的八小时登录会话。
    std::map<std::string, std::pair<Clock::time_point, unsigned>> m_attempts; // 每来源每分钟最多五次登录。
    std::map<uint32_t, Pending> m_pending; // 最多十六项在途业务请求。
    std::vector<drogon::WebSocketConnectionPtr> m_viewers; // 最多四个视频查看者。
    drogon::WebSocketConnectionPtr m_device; // 当前通过设备密钥认证的控制连接。
    drogon::WebSocketConnectionPtr m_video; // 当前通过设备密钥认证的视频上传连接。
    drogon::WebSocketConnectionPtr m_control; // 同一时刻唯一的网页控制者。
    Clock::time_point m_heartbeat{}; // 最近有效设备控制消息时间。
    Clock::time_point m_frameAt{}; // 最新 JPEG 的接收时间。
    Clock::time_point m_motionAt{}; // 一次设备动作确认的发送时间，防止控制管道持续积压。
    std::string m_frame; // 仅保留最新完整 JPEG，最大 512 KiB。
    uint64_t m_frameSeq = 0; // 当前帧发布序号。
    uint32_t m_request = 0; // HTTP 请求递增编号。
    uint32_t m_lease = 0; // 每次网页控制连接更换时生成新的设备控制代次。
    uint32_t m_sequence = 0; // 当前浏览器最近的有效二进制帧序号。
    bool m_isMotionBusy = false; // 等待设备确认期间丢弃新目标，停车仍允许优先传递。
};

// 握手鉴权过滤器，阻止未登录客户端先升级再尝试发送指令。
class SocketAuth : public drogon::HttpFilter<SocketAuth>
{
public:
    // 输入：请求与过滤回调；输出：允许或拒绝；作用：统一验证 WS 入口。
    void doFilter(const drogon::HttpRequestPtr& req, drogon::FilterCallback&& reject,
                  drogon::FilterChainCallback&& next) override;
};

// 四种连接共用协议控制器，身份和有效性由服务集中核对。
class CloudSocket : public drogon::WebSocketController<CloudSocket>
{
public:
    WS_PATH_LIST_BEGIN
    WS_PATH_ADD("/device/control", "vehicle::SocketAuth");
    WS_PATH_ADD("/device/video", "vehicle::SocketAuth");
    WS_PATH_ADD("/motor/ws", "vehicle::SocketAuth");
    WS_PATH_ADD("/view/video", "vehicle::SocketAuth");
    WS_PATH_LIST_END
    void handleNewConnection(const drogon::HttpRequestPtr& req,
                             const drogon::WebSocketConnectionPtr& socket) override;
    void handleNewMessage(const drogon::WebSocketConnectionPtr& socket, std::string&& data,
                          const drogon::WebSocketMessageType& type) override;
    void handleConnectionClosed(const drogon::WebSocketConnectionPtr& socket) override;
};
}
