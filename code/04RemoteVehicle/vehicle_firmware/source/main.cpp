#include "camera_web.h"
#include "cloud_agent.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if CONFIG_CAMERA_PERF_TEST
#include <cstring>
#include <vector>
#endif

// 输入：无；输出：无；作用：启动服务；OLED 固件关闭串口控制台以复用 UART0 引脚。
extern "C" void app_main(void)
{
    const esp_err_t result = CameraWeb::instance().start();
    if (result != ESP_OK)
    {
        // 初始化失败不反复重启，保留首个有效错误便于检查接线和 PSRAM。
        ESP_LOGE("camera_main", "start() << result=%s", esp_err_to_name(result));
        return;
    }
    // 复用原有主任务，首轮建立基线；后续采样不经过相机锁或 HTTP 请求队列。
    CloudAgent::instance().start();
    CameraWeb::instance().updateRuntime();
#if CONFIG_CAMERA_PERF_TEST
    // 微秒时钟下比较相邻五秒窗口；每个 CPU 的空闲时间分别按墙钟时间计算。
    std::vector<TaskStatus_t> previous(40);
    std::vector<TaskStatus_t> current(40);
    UBaseType_t prevCount = uxTaskGetSystemState(previous.data(), previous.size(), nullptr);
    int64_t previousUs = esp_timer_get_time();
    while (true)
    {
        // 主任务阻塞五秒，仅记录汇总值，避免逐帧日志本身拖慢服务。
        vTaskDelay(pdMS_TO_TICKS(5000));
        CameraWeb::instance().updateRuntime();
        const UBaseType_t count = uxTaskGetSystemState(current.data(), current.size(), nullptr);
        const int64_t currentUs = esp_timer_get_time();
        double idle0 = 0;
        double idle1 = 0;
        double camera = 0;
        double http = 0;
        double wifi = 0;
        double tcp = 0;
        for (UBaseType_t index = 0; index < count; ++index)
        {
            // 用任务编号匹配，跳过本窗口中新建的任务，避免把历史运行量算入本次采样。
            for (UBaseType_t old = 0; old < prevCount; ++old)
            {
                if (current[index].xTaskNumber != previous[old].xTaskNumber)
                {
                    continue;
                }
                const uint32_t delta = current[index].ulRunTimeCounter - previous[old].ulRunTimeCounter;
                const double percent = delta * 100.0 / (currentUs - previousUs);
                const char* name = current[index].pcTaskName;
                if (std::strcmp(name, "IDLE0") == 0)
                {
                    idle0 += percent;
                }
                else if (std::strcmp(name, "IDLE1") == 0)
                {
                    idle1 += percent;
                }
                else if (std::strcmp(name, "cam_task") == 0)
                {
                    camera += percent;
                }
                else if (std::strcmp(name, "httpd") == 0)
                {
                    http += percent;
                }
                else if (std::strcmp(name, "wifi") == 0)
                {
                    wifi += percent;
                }
                else if (std::strcmp(name, "tiT") == 0)
                {
                    tcp += percent;
                }
                break;
            }
        }
        ESP_LOGI("camera_perf", "window_ms=%lld tasks=%u idle0_pct=%.1f idle1_pct=%.1f camera_pct=%.1f http_pct=%.1f wifi_pct=%.1f tcp_pct=%.1f",
                 (long long)((currentUs - previousUs) / 1000), unsigned(count), idle0, idle1,
                 camera, http, wifi, tcp);
        previous.swap(current);
        prevCount = count;
        previousUs = currentUs;
    }
#else
    // 主任务每两秒采样一次并阻塞让出 CPU，不新增任务或网页轮询连接。
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(2000));
        CameraWeb::instance().updateRuntime();
    }
#endif
}
