#include "cloud_service.h"
#include <sodium.h>
#include <cstdlib>
#include <iostream>
#include <fstream>

namespace vehicle
{
// 输入：变量名及默认值；输出：配置值；作用：运行秘密来自环境文件，不嵌入源码。
std::string environment(const char* name, const char* fallback)
{
    const char* value = std::getenv(name);
    return value ? value : fallback;
}
// 输入：请求及应答；输出：异步响应；作用：提供框架可推导签名的统一 HTTP 入口。
void serve(const drogon::HttpRequestPtr& req, Reply&& reply)
{
    CloudService::instance().handle(req, std::move(reply));
}
// 输入：环境配置文件；输出：无；作用：让远程调试读取个人配置，秘密不进入项目文件。
void loadEnvironment(const std::string& path)
{
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot read environment file");
    std::string line;
    while (std::getline(input, line))
    {
        // 只接受本服务配置，不允许文件覆盖 PATH 或其他进程启动环境。
        const auto split = line.find('=');
        if (split == std::string::npos || line.starts_with('#')) continue;
        const auto key = line.substr(0, split);
        if (key != "SV_DATABASE" && key != "SV_WEB_ROOT" && key != "SV_ORIGIN" && key != "SV_DEVICE_HASH")
            throw std::runtime_error("Unknown environment setting");
        if (setenv(key.c_str(), line.substr(split + 1).c_str(), 1) != 0)
            throw std::runtime_error("Cannot apply environment setting");
    }
}
}

// 输入：可选管理命令；输出：进程状态；作用：先初始化安全库和日志，再启动服务。
int main(int argc, char** argv)
{
    if (sodium_init() < 0) return 1;
    drogon::app().setLogLevel(trantor::Logger::kInfo);
    try
    {
        if (argc == 3 && std::string(argv[1]) == "--env-file") vehicle::loadEnvironment(argv[2]);
        auto& service = vehicle::CloudService::instance();
        service.configure(vehicle::environment("SV_DATABASE", "/var/lib/smart-vehicle/users.db"),
                          vehicle::environment("SV_WEB_ROOT", "/opt/smart-vehicle/web"),
                          vehicle::environment("SV_ORIGIN", "https://124.223.192.76"),
                          vehicle::environment("SV_DEVICE_HASH", ""));
        if (argc == 3 && std::string(argv[1]) == "--set-password")
        {
            // 管理员通过标准输入提供密码，避免密码出现在命令参数和日志中。
            std::string password;
            std::getline(std::cin, password);
            const bool result = service.setPassword(argv[2], password);
            sodium_memzero(password.data(), password.size());
            return result ? 0 : 1;
        }
        // 服务只监听回环接口，所有公网流量由 Nginx 终止 TLS 后代理。
        drogon::app().addListener("127.0.0.1", 8000)
            .setThreadNum(2).setClientMaxBodySize(4096).setClientMaxWebSocketMessageSize(524288)
            .setIdleConnectionTimeout(20).setDocumentRoot("/var/empty");
        drogon::app().registerHandlerViaRegex("/.*", &vehicle::serve);
        drogon::app().getLoop()->runEvery(0.1, std::bind(&vehicle::CloudService::tick, &service));
        LOG_INFO << "main() >> version=1.0 listen=127.0.0.1:8000";
        drogon::app().run();
        LOG_INFO << "main() << result=0";
        return 0;
    }
    catch (const std::exception& error)
    {
        LOG_ERROR << "main() << result=1 message=" << error.what();
        return 1;
    }
}
