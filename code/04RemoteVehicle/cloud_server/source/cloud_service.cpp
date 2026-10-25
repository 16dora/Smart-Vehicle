#include "cloud_service.h"
#include <sodium.h>
#include <fstream>
#include <sstream>

namespace vehicle
{
// 登录凭证绝对有效期为八小时，退出和服务重启立即失效。
static constexpr auto SESSION_LIFE = std::chrono::hours(8);

// 输入：JSON；输出：紧凑文本；作用：控制消息不携带不必要的空白。
std::string encode(const Json::Value& value)
{
    Json::StreamWriterBuilder writer;
    writer["indentation"] = "";
    return Json::writeString(writer, value);
}

// 输入：文本；输出：解析对象和成功标志；作用：拒绝畸形协议对象。
bool decode(const std::string& text, Json::Value& value)
{
    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    std::string error;
    return reader->parse(text.data(), text.data() + text.size(), &value, &error) && value.isObject();
}

CloudService& CloudService::instance()
{
    static CloudService service;
    return service;
}

void CloudService::configure(const std::string& db, const std::string& web,
                             const std::string& origin, const std::string& tokenHash)
{
    LOG_INFO << "configure() >> database=" << db;
    sqlite3* handle = nullptr;
    if (sqlite3_open_v2(db.c_str(), &handle, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE |
                       SQLITE_OPEN_FULLMUTEX, nullptr) != SQLITE_OK)
    {
        if (handle) sqlite3_close(handle);
        throw std::runtime_error("Cannot open account database");
    }
    m_db.reset(handle);
    sqlite3_busy_timeout(m_db.get(), 2000);
    const char* sql = "PRAGMA journal_mode=WAL; CREATE TABLE IF NOT EXISTS users("
        "username TEXT PRIMARY KEY,password_hash TEXT NOT NULL,enabled INTEGER NOT NULL DEFAULT 1);";
    if (sqlite3_exec(m_db.get(), sql, nullptr, nullptr, nullptr) != SQLITE_OK)
        throw std::runtime_error("Cannot initialize account database");
    m_web = web;
    m_origin = origin;
    m_tokenHash = tokenHash;
    LOG_INFO << "configure() << result=0";
}

bool CloudService::setPassword(const std::string& user, const std::string& password)
{
    LOG_INFO << "setPassword() >> user=" << user;
    if (user.empty() || user.size() > 64 || password.size() < 8 || password.size() > 128)
    {
        LOG_ERROR << "setPassword() << result=invalid_length";
        return false;
    }
    char hash[crypto_pwhash_STRBYTES];
    if (crypto_pwhash_str_alg(hash, password.data(), password.size(), crypto_pwhash_OPSLIMIT_INTERACTIVE,
        crypto_pwhash_MEMLIMIT_INTERACTIVE, crypto_pwhash_ALG_ARGON2ID13) != 0)
    {
        LOG_ERROR << "setPassword() << result=hash_failed";
        return false;
    }
    sqlite3_stmt* raw = nullptr;
    const char* sql = "INSERT INTO users(username,password_hash) VALUES(?,?) "
        "ON CONFLICT(username) DO UPDATE SET password_hash=excluded.password_hash,enabled=1";
    if (sqlite3_prepare_v2(m_db.get(), sql, -1, &raw, nullptr) != SQLITE_OK) return false;
    std::unique_ptr<sqlite3_stmt, decltype(&sqlite3_finalize)> stmt(raw, sqlite3_finalize);
    sqlite3_bind_text(raw, 1, user.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(raw, 2, hash, -1, SQLITE_TRANSIENT);
    const bool result = sqlite3_step(raw) == SQLITE_DONE;
    sodium_memzero(hash, sizeof(hash));
    LOG_INFO << "setPassword() << result=" << result;
    return result;
}

std::string CloudService::randomToken(size_t bytes)
{
    std::vector<unsigned char> data(bytes);
    randombytes_buf(data.data(), data.size());
    std::string token(bytes * 2 + 1, '\0');
    sodium_bin2hex(token.data(), token.size(), data.data(), data.size());
    token.resize(bytes * 2);
    return token;
}

void CloudService::closeSocket(const drogon::WebSocketConnectionPtr& socket)
{
    // forceClose 可能同步调用关闭回调；明确排入主循环，当前持锁处理结束后再关闭。
    drogon::app().getLoop()->queueInLoop(std::bind(&drogon::WebSocketConnection::forceClose, socket));
}

bool CloudService::validSession(const std::string& session) const
{
    const auto found = m_sessions.find(session);
    return found != m_sessions.end() && found->second.expires > Clock::now();
}

std::string CloudService::sessionOf(const drogon::HttpRequestPtr& req)
{
    const auto session = req->getCookie("SVSESSION");
    return validSession(session) ? session : "";
}

bool CloudService::online() const
{
    return m_device && m_device->connected() && Clock::now() - m_heartbeat < std::chrono::seconds(5);
}

drogon::HttpResponsePtr CloudService::response(int code, const std::string& body, const std::string& type)
{
    auto result = drogon::HttpResponse::newHttpResponse();
    result->setCustomStatusCode(code);
    result->setContentTypeString(type);
    result->setBody(body);
    result->addHeader("Cache-Control", "no-store");
    result->addHeader("X-Content-Type-Options", "nosniff");
    result->addHeader("X-Frame-Options", "DENY");
    result->addHeader("Referrer-Policy", "same-origin");
    return result;
}

drogon::HttpResponsePtr CloudService::page(const std::string& file)
{
    std::ifstream input(m_web + "/" + file, std::ios::binary);
    if (!input) return response(500, "{\"message\":\"Page unavailable\"}");
    std::ostringstream content;
    content << input.rdbuf();
    return response(200, content.str(), "text/html; charset=utf-8");
}

void CloudService::revoke(const std::string& session)
{
    m_sessions.erase(session);
    if (m_control && m_control->getContext<Link>()->session == session)
    {
        stopControl("session revoked");
        closeSocket(m_control);
        m_control.reset();
    }
    for (auto& socket : m_viewers)
        if (socket->getContext<Link>()->session == session) closeSocket(socket);
    for (auto it = m_pending.begin(); it != m_pending.end();)
    {
        if (it->second.session == session)
        {
            it->second.reply(response(401, "{\"message\":\"Session expired\"}"));
            it = m_pending.erase(it);
        }
        else ++it;
    }
}

void CloudService::handle(const drogon::HttpRequestPtr& req, Reply&& reply)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    const std::string path = req->path();
    const bool isPost = req->method() == drogon::Post;
    // 所有修改请求须来自公开页面；反向代理固定覆盖 Host 和来源协议头。
    if (isPost && req->getHeader("origin") != m_origin)
    {
        reply(response(403, "{\"message\":\"Invalid origin\"}"));
        return;
    }
    const std::string session = sessionOf(req);
    if (path == "/login" && req->method() == drogon::Get)
    {
        if (session.empty()) reply(page("login.html"));
        else
        {
            auto result = response(302, "");
            result->addHeader("Location", "/");
            reply(result);
        }
        return;
    }
    if (path == "/api/login" && isPost)
    {
        LOG_INFO << "login() >>";
        auto& attempt = m_attempts[req->getHeader("x-real-ip")];
        if (Clock::now() - attempt.first > std::chrono::minutes(1)) attempt = {Clock::now(), 0};
        if (++attempt.second > 5 || m_sessions.size() >= 32)
        {
            reply(response(429, "{\"message\":\"请求过于频繁，请稍后再试\"}"));
            LOG_WARN << "login() << result=rate_limited";
            return;
        }
        const auto body = req->getJsonObject();
        if (!body || !body->isObject() || !(*body)["username"].isString() || !(*body)["password"].isString())
        {
            reply(response(400, "{\"message\":\"请输入账号和密码\"}"));
            return;
        }
        const auto user = (*body)["username"].asString();
        auto password = (*body)["password"].asString();
        bool accepted = false;
        sqlite3_stmt* raw = nullptr;
        const char* sql = "SELECT password_hash FROM users WHERE username=? AND enabled=1";
        if (user.size() <= 64 && password.size() <= 128 &&
            sqlite3_prepare_v2(m_db.get(), sql, -1, &raw, nullptr) == SQLITE_OK)
        {
            std::unique_ptr<sqlite3_stmt, decltype(&sqlite3_finalize)> stmt(raw, sqlite3_finalize);
            sqlite3_bind_text(raw, 1, user.c_str(), -1, SQLITE_TRANSIENT);
            if (sqlite3_step(raw) == SQLITE_ROW)
            {
                const auto hash = reinterpret_cast<const char*>(sqlite3_column_text(raw, 0));
                accepted = crypto_pwhash_str_verify(hash, password.data(), password.size()) == 0;
            }
        }
        sodium_memzero(password.data(), password.size());
        if (!accepted)
        {
            reply(response(401, "{\"message\":\"账号或密码错误\"}"));
            LOG_WARN << "login() << result=rejected";
            return;
        }
        // 登录成功始终更换随机会话，防止沿用登录前的身份凭证。
        if (!session.empty()) revoke(session);
        const auto id = randomToken(32);
        m_sessions.emplace(id, Session{Clock::now() + SESSION_LIFE});
        auto result = response(200, "{\"result\":0}");
        drogon::Cookie cookie("SVSESSION", id);
        cookie.setPath("/");
        cookie.setSecure(true);
        cookie.setHttpOnly(true);
        cookie.setSameSite(drogon::Cookie::SameSite::kStrict);
        result->addCookie(cookie);
        reply(result);
        LOG_INFO << "login() << result=0";
        return;
    }
    if (session.empty())
    {
        const bool isPage = path == "/" || path == "/logout";
        auto result = response(isPage ? 302 : 401,
                               "{\"message\":\"请先登录\"}");
        // 自定义状态码在此版本不映射到 statusCode() 枚举，直接按页面类型设置跳转。
        if (isPage) result->addHeader("Location", "/login");
        reply(result);
        return;
    }
    if (path == "/" && req->method() == drogon::Get) reply(page("index.html"));
    else if (path == "/logout" && req->method() == drogon::Get) reply(page("logout.html"));
    else if (path == "/api/logout" && isPost)
    {
        revoke(session);
        auto result = response(200, "{\"result\":0}");
        result->addHeader("Set-Cookie", "SVSESSION=; Path=/; Max-Age=0; HttpOnly; Secure; SameSite=Strict");
        reply(result);
    }
    else if (path == "/api/session" && req->method() == drogon::Get)
        reply(response(200, "{\"authenticated\":true}"));
    else if (path == "/capture" && req->method() == drogon::Get)
    {
        if (!online() || m_frame.empty() || Clock::now() - m_frameAt > std::chrono::seconds(2))
            reply(response(503, "{\"message\":\"小车离线或图像已中断\"}"));
        else reply(response(200, m_frame, "image/jpeg"));
    }
    else if (path == "/stop" && isPost) reply(response(200, "OK", "text/plain"));
    else if ((path == "/status" || path == "/motor" || path == "/arm") && req->method() == drogon::Get)
        forward(req, std::move(reply), session);
    else if ((path == "/control" || path == "/led" || path == "/motor" || path == "/arm") && isPost)
        forward(req, std::move(reply), session);
    else reply(response(404, "{\"message\":\"Not found\"}"));
}

void CloudService::sendDevice(const Json::Value& value)
{
    if (m_device && m_device->connected()) m_device->send(encode(value));
}

void CloudService::forward(const drogon::HttpRequestPtr& req, Reply&& reply, const std::string& session)
{
    if (!online())
    {
        reply(response(503, "{\"message\":\"小车尚未连接云服务器\"}"));
        return;
    }
    if (m_pending.size() >= 16)
    {
        reply(response(429, "{\"message\":\"设备请求繁忙\"}"));
        return;
    }
    // 备用停车可由持有相同会话的页面提交，其他会话不能干扰现有控制者。
    if (req->method() == drogon::Post && m_control &&
        m_control->getContext<Link>()->session != session)
    {
        reply(response(409, "{\"message\":\"已有其他控制页面连接\"}"));
        return;
    }
    if (++m_request == 0) ++m_request;
    Json::Value packet;
    packet["op"] = "http";
    packet["id"] = m_request;
    packet["path"] = req->path();
    packet["method"] = req->method() == drogon::Post ? "POST" : "GET";
    packet["body"] = std::string(req->body());
    packet["deadline_ms"] = Json::Int64(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() + 1500);
    m_pending.emplace(m_request, Pending{std::move(reply), Clock::now() + std::chrono::seconds(3), session});
    sendDevice(packet);
}

bool CloudService::allowSocket(const drogon::HttpRequestPtr& req)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (req->path().starts_with("/device/"))
    {
        const auto key = req->getHeader("x-device-key");
        if (key.size() != 64 || m_tokenHash.size() != 64) return false;
        unsigned char digest[crypto_hash_sha256_BYTES];
        char text[crypto_hash_sha256_BYTES * 2 + 1];
        crypto_hash_sha256(digest, reinterpret_cast<const unsigned char*>(key.data()), key.size());
        sodium_bin2hex(text, sizeof(text), digest, sizeof(digest));
        return sodium_memcmp(text, m_tokenHash.data(), 64) == 0;
    }
    return req->getHeader("origin") == m_origin && !sessionOf(req).empty();
}

void CloudService::stopControl(const std::string& reason)
{
    LOG_INFO << "stopControl() >> reason=" << reason;
    Json::Value packet;
    packet["op"] = "stop";
    sendDevice(packet);
    m_lease = 0;
    m_sequence = 0;
    m_isMotionBusy = false;
    LOG_INFO << "stopControl() << result=0";
}

void CloudService::opened(const drogon::HttpRequestPtr& req, const drogon::WebSocketConnectionPtr& socket)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    auto context = std::make_shared<Link>();
    context->path = req->path();
    context->session = sessionOf(req);
    socket->setContext(context);
    if (context->path == "/device/control")
    {
        if (m_device && m_device->connected()) { closeSocket(socket); return; }
        m_device = socket;
        m_heartbeat = Clock::now();
        stopControl("device connected");
        if (m_control) closeSocket(m_control);
        m_control.reset();
        LOG_INFO << "opened() << result=device_online";
    }
    else if (context->path == "/device/video")
    {
        if (m_video && m_video->connected()) { closeSocket(socket); return; }
        m_video = socket;
        m_frame.clear();
    }
    else if (!validSession(context->session) || !online()) closeSocket(socket);
    else if (context->path == "/motor/ws")
    {
        if (m_control && m_control->connected()) { closeSocket(socket); return; }
        m_control = socket;
        m_lease = randombytes_random();
        if (m_lease == 0) m_lease = 1;
        m_sequence = 0;
        Json::Value packet;
        packet["op"] = "open";
        packet["lease"] = m_lease;
        sendDevice(packet);
    }
    else if (context->path == "/view/video")
    {
        if (m_viewers.size() >= 4) { closeSocket(socket); return; }
        m_viewers.push_back(socket);
        sendFrame(socket);
    }
}

void CloudService::sendFrame(const drogon::WebSocketConnectionPtr& socket)
{
    auto context = socket->getContext<Link>();
    if (!context->ready || context->frame == m_frameSeq || m_frame.empty()) return;
    socket->send(m_frame, drogon::WebSocketMessageType::Binary);
    context->ready = false;
    context->frame = m_frameSeq;
    context->sent = Clock::now();
}

void CloudService::received(const drogon::WebSocketConnectionPtr& socket, const std::string& data,
                            drogon::WebSocketMessageType type)
{
    if (type == drogon::WebSocketMessageType::Ping || type == drogon::WebSocketMessageType::Pong) return;
    std::lock_guard<std::mutex> lock(m_mutex);
    auto context = socket->getContext<Link>();
    if (!context) { closeSocket(socket); return; }
    if (socket == m_video && type == drogon::WebSocketMessageType::Binary)
    {
        if (data.size() < 4 || data.size() > 524288 || static_cast<unsigned char>(data[0]) != 0xff ||
            static_cast<unsigned char>(data[1]) != 0xd8 ||
            static_cast<unsigned char>(data[data.size()-2]) != 0xff ||
            static_cast<unsigned char>(data.back()) != 0xd9)
        { closeSocket(socket); return; }
        m_frame = data;
        m_frameAt = Clock::now();
        ++m_frameSeq;
        return;
    }
    if (socket == m_device && type == drogon::WebSocketMessageType::Text)
    {
        Json::Value packet;
        if (data.size() > 8192 || !decode(data, packet)) { closeSocket(socket); return; }
        if (!packet["op"].isString()) { closeSocket(socket); return; }
        m_heartbeat = Clock::now();
        const auto op = packet.get("op", "").asString();
        if (op == "reply" && packet["id"].isUInt() && packet["status"].isInt() && packet["body"].isString())
        {
            auto found = m_pending.find(packet["id"].asUInt());
            if (found != m_pending.end())
            {
                auto pending = std::move(found->second);
                m_pending.erase(found);
                if (validSession(pending.session) && pending.expires > Clock::now())
                {
                    const int code = packet.get("status", 502).asInt();
                    pending.reply(response(code >= 200 && code <= 599 ? code : 502,
                        packet.get("body", "{}").asString()));
                }
                else pending.reply(response(401, "{\"message\":\"Session expired\"}"));
            }
        }
        else if (op == "motion_ack" && packet["lease"].isUInt() && packet["lease"].asUInt() == m_lease)
        {
            m_isMotionBusy = false;
            if (m_control && validSession(m_control->getContext<Link>()->session) && packet["report"].isString())
                m_control->send(packet["report"].asString());
        }
        return;
    }
    if (!validSession(context->session) || !online()) { closeSocket(socket); return; }
    if (context->path == "/view/video")
    {
        if (type == drogon::WebSocketMessageType::Text && data == "next") context->ready = true;
        else closeSocket(socket);
        return;
    }
    if (socket != m_control || type != drogon::WebSocketMessageType::Binary || data.size() != 12)
    { closeSocket(socket); return; }
    const auto bytes = reinterpret_cast<const unsigned char*>(data.data());
    uint32_t sequence = 0;
    for (size_t index = 8; index < 12; ++index) sequence = (sequence << 8) | bytes[index];
    if (sequence == 0 || sequence <= m_sequence || bytes[0] > 8) { closeSocket(socket); return; }
    m_sequence = sequence;
    // 网络变慢时丢弃新目标，绝不堆积续租；停车和机械臂保持命令仍优先转发。
    if (m_isMotionBusy && bytes[0] != 2 && bytes[0] != 5) return;
    Json::Value packet;
    packet["op"] = "motion";
    packet["lease"] = m_lease;
    packet["deadline_ms"] = Json::Int64(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() + 400);
    for (size_t index = 0; index < 12; ++index) packet["bytes"].append(bytes[index]);
    m_isMotionBusy = true;
    m_motionAt = Clock::now();
    sendDevice(packet);
}

void CloudService::closed(const drogon::WebSocketConnectionPtr& socket)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (socket == m_control)
    {
        stopControl("controller disconnected");
        m_control.reset();
    }
    if (socket == m_device)
    {
        m_device.reset();
        m_frame.clear();
        if (m_control) closeSocket(m_control);
        for (auto& viewer : m_viewers) closeSocket(viewer);
        LOG_WARN << "closed() << result=device_offline";
    }
    if (socket == m_video) { m_video.reset(); m_frame.clear(); }
    std::erase(m_viewers, socket);
}

void CloudService::tick()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    const auto now = Clock::now();
    for (auto it = m_sessions.begin(); it != m_sessions.end();)
    {
        if (it->second.expires <= now)
        {
            const auto session = it->first;
            ++it;
            revoke(session);
        }
        else ++it;
    }
    for (auto it = m_attempts.begin(); it != m_attempts.end();)
    {
        if (now - it->second.first > std::chrono::minutes(2)) it = m_attempts.erase(it);
        else ++it;
    }
    for (auto it = m_pending.begin(); it != m_pending.end();)
    {
        if (it->second.expires <= now)
        {
            it->second.reply(response(504, "{\"message\":\"设备应答超时，结果未确认\"}"));
            it = m_pending.erase(it);
        }
        else ++it;
    }
    if (m_device && !online()) closeSocket(m_device);
    if (m_control && (!validSession(m_control->getContext<Link>()->session) ||
        (m_isMotionBusy && now - m_motionAt > std::chrono::milliseconds(500))))
    {
        stopControl("controller session or acknowledgement timeout");
        closeSocket(m_control);
    }
    for (auto& viewer : m_viewers)
    {
        auto context = viewer->getContext<Link>();
        if (!online() || !validSession(context->session) ||
            (!context->ready && now - context->sent > std::chrono::seconds(5)) ||
            (!m_frame.empty() && now - m_frameAt > std::chrono::seconds(2))) closeSocket(viewer);
        else sendFrame(viewer);
    }
}

void SocketAuth::doFilter(const drogon::HttpRequestPtr& req, drogon::FilterCallback&& reject,
                          drogon::FilterChainCallback&& next)
{
    if (CloudService::instance().allowSocket(req)) next();
    else
    {
        auto result = drogon::HttpResponse::newHttpJsonResponse(Json::Value("Authentication required"));
        result->setStatusCode(drogon::k403Forbidden);
        reject(result);
    }
}

void CloudSocket::handleNewConnection(const drogon::HttpRequestPtr& req,
                                     const drogon::WebSocketConnectionPtr& socket)
{ CloudService::instance().opened(req, socket); }
void CloudSocket::handleNewMessage(const drogon::WebSocketConnectionPtr& socket, std::string&& data,
                                  const drogon::WebSocketMessageType& type)
{ CloudService::instance().received(socket, data, type); }
void CloudSocket::handleConnectionClosed(const drogon::WebSocketConnectionPtr& socket)
{ CloudService::instance().closed(socket); }
}
