#include "camera_web.h"
#include "esp_log.h"

// 输入：无；输出：无；作用：启动服务，失败时保持串口可见且不启动后续业务。
extern "C" void app_main(void)
{
    const esp_err_t result = CameraWeb::instance().start();
    if (result != ESP_OK)
    {
        // 初始化失败不反复重启，保留首个有效错误便于检查接线和 PSRAM。
        ESP_LOGE("camera_main", "start() << result=%s", esp_err_to_name(result));
    }
}
