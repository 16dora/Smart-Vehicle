#include "camera_web.h"
#include "board_pins.h"
// AP 与 STA 同时启动，沿用 02 项目的本地路由器凭据，不输出密码。
#include "wifi_credentials.local.h"
#include <cerrno>
#include <algorithm>
#include <cmath>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include "esp_check.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_psram.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#if CONFIG_CAMERA_PERF_TEST
#include "driver/ledc.h"
#endif

// 本模块日志标签；协议数据使用 HTTP 输出，不与串口日志混合。
static constexpr char TAG[] = "camera_web";

CameraWeb& CameraWeb::instance()
{
    // 实例一直存活到固件重启，网络回调不会引用已销毁的栈对象。
    static CameraWeb app;
    return app;
}

#if CONFIG_CAMERA_OLED_ENABLED
// 输入：CPU 统计窗口；输出：无；作用：复制显示快照，锁外交给 OLED 驱动刷新。
void CameraWeb::updateOled(bool isCpuValid, const std::array<double, 2>& busyPct)
{
    OledState state;
    state.isCpuValid = isCpuValid;
    state.busyPct = busyPct;
    {
        // 显示驱动不访问相机和统计状态；十秒曝光时效由数据来源层负责判断。
        std::lock_guard<std::mutex> lock(m_statusMutex);
        state.exposureMs = m_oledExpMs;
        state.isExpValid = m_isOledExpValid && m_oledExpAtUs > 0 &&
            esp_timer_get_time() - m_oledExpAtUs < 10000000;
        state.isAuto = m_isOledAuto;
    }
    m_display.update(state);
}
#endif

esp_err_t CameraWeb::start()
{
    ESP_LOGI(TAG, "start() >>");
    // COM3 使用 CH340/UART0；释放原生 USB 两脚，保留旧母座 GPIO20 孔的接地线。
    gpio_config_t socketPins{};
    socketPins.pin_bit_mask = (1ULL << board::SOCKET_OLD_VCC) | (1ULL << board::SOCKET_OLD_GND);
    socketPins.mode = GPIO_MODE_INPUT;
    socketPins.pull_up_en = GPIO_PULLUP_DISABLE;
    socketPins.pull_down_en = GPIO_PULLDOWN_DISABLE;
    socketPins.intr_type = GPIO_INTR_DISABLE;
    ESP_LOGI(TAG, "gpio_config() >> gpio=19,20 mode=INPUT pulls=disabled native_usb=disabled");
    const esp_err_t socketResult = gpio_config(&socketPins);
    ESP_LOGI(TAG, "gpio_config() << result=%s", esp_err_to_name(socketResult));
    if (socketResult != ESP_OK)
    {
        // 无法建立固定母座的输入约束时停止启动，不继续初始化底盘输出。
        ESP_LOGE(TAG, "start() << result=%s socket_input_setup_failed", esp_err_to_name(socketResult));
        return socketResult;
    }
    const esp_err_t ledResult = initLed();
    if (ledResult != ESP_OK) ESP_LOGW(TAG, "Flash LED unavailable result=%s", esp_err_to_name(ledResult));
#if CONFIG_CAMERA_OLED_ENABLED
    // OLED 独占 I2C0；UART0 保留启动日志，相机稍后独立初始化 I2C1。
    const esp_err_t oledResult = m_display.start();
    if (oledResult != ESP_OK)
    {
        // OLED 为可选外设；初始化失败不阻止相机和电机服务。
        ESP_LOGW(TAG, "OLED unavailable result=%s", esp_err_to_name(oledResult));
    }
#endif
    // 保持原有顺序：总线与机械臂先关闭历史输出，再启动车轮停车监视任务。
    const esp_err_t armResult = m_arm.start();
    if (armResult != ESP_OK)
    {
        ESP_LOGW(TAG, "Arm unavailable result=%s; other startup continues", esp_err_to_name(armResult));
    }
    // 尽早撤销 PCA9685 可能保留的输出；未连接电机板时仍允许使用相机。
    const esp_err_t motorResult = m_motor.start();
    if (motorResult != ESP_OK)
    {
        ESP_LOGW(TAG, "Motor unavailable result=%s; camera startup continues", esp_err_to_name(motorResult));
    }
    esp_err_t result = initCamera();
    if (result == ESP_OK)
    {
        result = initWifi();
    }
    if (result == ESP_OK)
    {
        result = startServers();
    }
    ESP_LOGI(TAG, "start() << result=%s", esp_err_to_name(result));
    return result;
}

// 输入：无；输出：初始化结果；作用：RMT 配置板载 WS2812，启动时发送熄灭指令。
esp_err_t CameraWeb::initLed()
{
    ESP_LOGI(TAG, "initLed() >> gpio=%d default=off", int(board::RGB_LED));
    rmt_tx_channel_config_t channel{};
    channel.gpio_num = board::RGB_LED;
    channel.clk_src = RMT_CLK_SRC_DEFAULT;
    channel.resolution_hz = 10000000;
    channel.mem_block_symbols = 64;
    channel.trans_queue_depth = 1;
    ESP_LOGI(TAG, "rmt_new_tx_channel() >> gpio=%d hz=10000000 symbols=64 queue=1", int(board::RGB_LED));
    esp_err_t result = rmt_new_tx_channel(&channel, &m_ledChannel);
    ESP_LOGI(TAG, "rmt_new_tx_channel() << result=%s", esp_err_to_name(result));
    if (result == ESP_OK)
    {
        // 复制编码器直接发送完整 24 位 GRB 和低电平锁存，不占用相机 LEDC。
        rmt_copy_encoder_config_t encoder{};
        ESP_LOGI(TAG, "rmt_new_copy_encoder() >>");
        result = rmt_new_copy_encoder(&encoder, &m_ledEncoder);
        ESP_LOGI(TAG, "rmt_new_copy_encoder() << result=%s", esp_err_to_name(result));
    }
    if (result == ESP_OK)
    {
        ESP_LOGI(TAG, "rmt_enable() >> gpio=%d", int(board::RGB_LED));
        result = rmt_enable(m_ledChannel);
        ESP_LOGI(TAG, "rmt_enable() << result=%s", esp_err_to_name(result));
        if (result == ESP_OK) result = writeLed(false, m_ledBrightness.load());
    }
    // 无 ACK 的灯光接口只能确认指令发送，不能检测灯珠是否在线或实际发光。
    m_ledFault.store(result);
    m_ledReady.store(result == ESP_OK);
    m_ledOn.store(false);
    if (result == ESP_OK) ESP_LOGI(TAG, "initLed() << result=ESP_OK");
    else ESP_LOGW(TAG, "initLed() << result=%s", esp_err_to_name(result));
    return result;
}

// 输入：开关与亮度百分数；输出：RMT 结果；作用：将白光强度映射到三个 8 位通道。
esp_err_t CameraWeb::writeLed(bool enabled, int brightness)
{
    ESP_LOGI(TAG, "writeLed() >> enabled=%d brightness=%d gpio=%d", int(enabled), brightness, int(board::RGB_LED));
    if (brightness < 0 || brightness > 100)
    {
        // 所有输出先校验百分数，不能将越界值截断为另一种发光强度。
        ESP_LOGW(TAG, "writeLed() << result=INVALID_BRIGHTNESS");
        return ESP_ERR_INVALID_ARG;
    }
    if (m_ledChannel == nullptr || m_ledEncoder == nullptr)
    {
        ESP_LOGW(TAG, "writeLed() << result=NOT_INITIALIZED");
        return ESP_ERR_INVALID_STATE;
    }
    // 关灯保留预设但输出全零；百分数按最近整数映射到 0～255，0% 即不发光。
    const uint8_t value = enabled ? static_cast<uint8_t>((brightness * 255 + 50) / 100) : 0;
    // 三个颜色值相同，GRB/RGB 顺序均为白色；每位 1.2 us，末尾锁存低电平 100 us。
    for (size_t index = 0; index < 24; ++index)
    {
        const bool one = (value & (0x80 >> (index % 8))) != 0;
        m_ledSymbols[index].level0 = 1;
        m_ledSymbols[index].duration0 = one ? 8 : 4;
        m_ledSymbols[index].level1 = 0;
        m_ledSymbols[index].duration1 = one ? 4 : 8;
    }
    m_ledSymbols[24].level0 = 0;
    m_ledSymbols[24].duration0 = 500;
    m_ledSymbols[24].level1 = 0;
    m_ledSymbols[24].duration1 = 500;
    rmt_transmit_config_t transfer{};
    ESP_LOGI(TAG, "rmt_transmit() >> symbols=%u enabled=%d", unsigned(m_ledSymbols.size()), int(enabled));
    esp_err_t result = rmt_transmit(m_ledChannel, m_ledEncoder, m_ledSymbols.data(), sizeof(m_ledSymbols), &transfer);
    ESP_LOGI(TAG, "rmt_transmit() << result=%s", esp_err_to_name(result));
    if (result == ESP_OK)
    {
        // 等待完成后才发布开关状态；超时后上层禁用后续写入，成员缓冲仍然有效。
        ESP_LOGI(TAG, "rmt_tx_wait_all_done() >> timeout_ms=100");
        result = rmt_tx_wait_all_done(m_ledChannel, 100);
        ESP_LOGI(TAG, "rmt_tx_wait_all_done() << result=%s", esp_err_to_name(result));
    }
    if (result == ESP_OK) ESP_LOGI(TAG, "writeLed() << result=ESP_OK");
    else ESP_LOGW(TAG, "writeLed() << result=%s", esp_err_to_name(result));
    return result;
}

// 输入：enabled=0/1、brightness=0～100，可单独或同时提供；输出：确认后的开关及亮度预设。
// 作用：绝对值设置，关灯时可调预设；不重试未知结果，不因调亮度隐式开灯。
esp_err_t CameraWeb::setLed(httpd_req_t* req)
{
    ESP_LOGI(TAG, "setLed() >> bytes=%u", unsigned(req->content_len));
    if (!isMotorOrigin(req, false))
    {
        ESP_LOGW(TAG, "setLed() << result=ORIGIN_REJECTED");
        return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Use the device webpage");
    }
    char body[48] = {};
    size_t offset = 0;
    esp_err_t result = ESP_OK;
    if (req->content_len == 0 || req->content_len >= sizeof(body)) result = ESP_ERR_INVALID_ARG;
    while (result == ESP_OK && offset < req->content_len)
    {
        // 收齐表单后才解析全部字段；不完整或非法请求不会先改变部分输出。
        ESP_LOGI(TAG, "httpd_req_recv() >> remaining=%u", unsigned(req->content_len - offset));
        const int count = httpd_req_recv(req, body + offset, req->content_len - offset);
        ESP_LOGI(TAG, "httpd_req_recv() << count=%d", count);
        if (count <= 0) result = ESP_ERR_INVALID_ARG;
        else offset += size_t(count);
    }
    bool enabled = m_ledOn.load();
    int brightness = m_ledBrightness.load();
    char onText[4] = {};
    char levelText[8] = {};
    ESP_LOGI(TAG, "httpd_query_key_value() >> key=enabled capacity=%u", unsigned(sizeof(onText)));
    const esp_err_t onResult = httpd_query_key_value(body, "enabled", onText, sizeof(onText));
    ESP_LOGI(TAG, "httpd_query_key_value() << result=%s value=%s", esp_err_to_name(onResult),
             onResult == ESP_OK ? onText : "");
    ESP_LOGI(TAG, "httpd_query_key_value() >> key=brightness capacity=%u", unsigned(sizeof(levelText)));
    const esp_err_t levelResult = httpd_query_key_value(body, "brightness", levelText, sizeof(levelText));
    ESP_LOGI(TAG, "httpd_query_key_value() << result=%s value=%s", esp_err_to_name(levelResult),
             levelResult == ESP_OK ? levelText : "");
    if (onResult == ESP_OK)
    {
        if (std::strcmp(onText, "0") == 0) enabled = false;
        else if (std::strcmp(onText, "1") == 0) enabled = true;
        else result = ESP_ERR_INVALID_ARG;
    }
    else if (onResult != ESP_ERR_NOT_FOUND) result = ESP_ERR_INVALID_ARG;
    if (levelResult == ESP_OK)
    {
        // 限定非空十进制整数，拒绝负数、小数及尾随字符；字段长度限制保证转换不会溢出。
        if (levelText[0] == '\0' || std::strspn(levelText, "0123456789") != std::strlen(levelText))
            result = ESP_ERR_INVALID_ARG;
        else
        {
            brightness = std::atoi(levelText);
            if (brightness > 100) result = ESP_ERR_INVALID_ARG;
        }
    }
    else if (levelResult != ESP_ERR_NOT_FOUND) result = ESP_ERR_INVALID_ARG;
    if (onResult == ESP_ERR_NOT_FOUND && levelResult == ESP_ERR_NOT_FOUND) result = ESP_ERR_INVALID_ARG;
    if (result != ESP_OK)
    {
        ESP_LOGW(TAG, "setLed() << result=INVALID_ARGUMENT");
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected enabled=0/1 and/or brightness=0..100");
    }
    if (!m_ledReady.load()) result = ESP_ERR_INVALID_STATE;
    if (result == ESP_OK)
    {
        result = writeLed(enabled, brightness);
        m_ledFault.store(result);
        if (result == ESP_OK)
        {
            // 只有 RMT 发送完成才发布整组设置；这些状态只确认软件输出，不代表光学反馈。
            m_ledBrightness.store(brightness);
            m_ledOn.store(enabled);
        }
        else m_ledReady.store(false);
    }
    if (result != ESP_OK)
    {
        ESP_LOGW(TAG, "setLed() << result=%s", esp_err_to_name(result));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Board LED unavailable");
    }
    char json[128];
    std::snprintf(json, sizeof(json), "{\"led_ready\":true,\"led_on\":%s,\"led_brightness\":%d,\"led_fault\":0}",
                  m_ledOn.load() ? "true" : "false", m_ledBrightness.load());
    httpd_resp_set_type(req, "application/json");
    ESP_LOGI(TAG, "setLed() << result=ESP_OK enabled=%d brightness=%d", int(enabled), brightness);
    return httpd_resp_sendstr(req, json);
}

esp_err_t CameraWeb::initCamera()
{
    ESP_LOGI(TAG, "initCamera() >> psram_bytes=%u", unsigned(esp_psram_get_size()));
    if (!esp_psram_is_initialized() || esp_psram_get_size() < 2 * 1024 * 1024)
    {
        // 高分辨率 JPEG 需要外部帧缓冲；硬件不符时明确失败，不隐式降级。
        ESP_LOGE(TAG, "initCamera() << result=PSRAM_NOT_READY");
        return ESP_ERR_NO_MEM;
    }

    camera_config_t config = {};
    // 相机板内连接已通过 OV3660 取图验证；底盘外设使用独立的固定孔位映射。
    config.pin_pwdn = -1;
    config.pin_reset = -1;
    config.pin_xclk = board::CAM_XCLK;
    config.pin_sccb_sda = board::CAM_SDA;
    config.pin_sccb_scl = board::CAM_SCL;
    config.pin_d0 = board::CAM_D0;
    config.pin_d1 = board::CAM_D1;
    config.pin_d2 = board::CAM_D2;
    config.pin_d3 = board::CAM_D3;
    config.pin_d4 = board::CAM_D4;
    config.pin_d5 = board::CAM_D5;
    config.pin_d6 = board::CAM_D6;
    config.pin_d7 = board::CAM_D7;
    config.pin_vsync = board::CAM_VSYNC;
    config.pin_href = board::CAM_HREF;
    config.pin_pclk = board::CAM_PCLK;
    // 本板实测 16 MHz 避免 20 MHz 的秒级网络停顿；PLL 由组件按分辨率设置。
    config.xclk_freq_hz = CONFIG_CAMERA_XCLK_FREQ_HZ;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.pixel_format = PIXFORMAT_JPEG;
    // 按最大支持分辨率分配缓冲，运行时切换分辨率不会超出初始容量。
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    // 直接 DMA 写 PSRAM；三缓冲在 TCP 持有一帧时仍可采集，并丢弃积压旧帧降低延迟。
    config.fb_count = m_frameCount;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
    esp_err_t result = esp_camera_init(&config);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "initCamera() << result=%s", esp_err_to_name(result));
        return result;
    }
    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor == nullptr || sensor->id.PID != OV3660_PID)
    {
        // 05 只开放 OV3660 的寄存器时序，型号不符时明确失败，不写入其他传感器。
        ESP_LOGE(TAG, "initCamera() << result=UNSUPPORTED_SENSOR");
        esp_camera_deinit();
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (sensor->set_framesize(sensor, FRAMESIZE_VGA) != 0 ||
        sensor->set_quality(sensor, 12) != 0)
    {
        ESP_LOGE(TAG, "initCamera() << result=INITIAL_SETTINGS_FAILED");
        esp_camera_deinit();
        return ESP_FAIL;
    }
    // 上电先设置传感器翻转，使网页勾选状态与实际图像一致；随后清理旧方向的排队帧。
    ESP_LOGI(TAG, "sensor set_vflip() >> enabled=1");
    const int flipped = sensor->set_vflip(sensor, 1);
    ESP_LOGI(TAG, "sensor set_vflip() << result=%d", flipped);
    if (flipped != 0)
    {
        // 默认方向设置失败时不发布错误的网页状态，也不继续使用未完成配置的相机。
        ESP_LOGE(TAG, "initCamera() << result=INITIAL_VFLIP_FAILED");
        esp_camera_deinit();
        return ESP_FAIL;
    }
    // 每个缓冲都可能保留初始化 UXGA 阶段的旧帧，逐个丢弃后再开放 HTTP 服务。
    for (int index = 0; index < m_frameCount; ++index)
    {
        camera_fb_t* initial = esp_camera_fb_get();
        if (initial == nullptr)
        {
            ESP_LOGE(TAG, "initCamera() << result=INITIAL_FRAME_TIMEOUT");
            esp_camera_deinit();
            return ESP_ERR_TIMEOUT;
        }
        esp_camera_fb_return(initial);
    }
    // 初始化阶段尚无并发请求；先发布快照，再开放网页和运动服务。
    m_size = 2;
    cacheStatus();
    ESP_LOGI("camera_main", "initCamera() << result=OK sensor=OV3660 resolution=640x480 fb_count=%d grab_mode=latest cpu_mhz=%d xclk_hz=%d direct_psram_dma=%d",
             m_frameCount, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, CONFIG_CAMERA_XCLK_FREQ_HZ,
             int(esp_camera_get_psram_mode()));
    return ESP_OK;
}



// 输入：驱动持有的 JPEG 帧；输出：头结构和扫描边界是否一致；作用：阻止损坏图像进入浏览器解码器。
bool CameraWeb::isValidJpeg(const camera_fb_t* frame)
{
    if (frame == nullptr || frame->buf == nullptr || frame->format != PIXFORMAT_JPEG || frame->len < 4 ||
        frame->buf[0] != 0xff || frame->buf[1] != 0xd8 ||
        frame->buf[frame->len - 2] != 0xff || frame->buf[frame->len - 1] != 0xd9)
    {
        // 先检查最小边界，后续偏移读取才能安全访问帧缓冲。
        return false;
    }
    const uint8_t* data = frame->buf;
    std::array<uint8_t, 4> components{};
    unsigned count = 0;
    unsigned quantTables = 0;
    unsigned huffTables = 0;
    size_t offset = 2;
    while (offset + 4 <= frame->len)
    {
        // 头段必须以标记开始；任何长度越界都视为采集内容损坏。
        if (data[offset++] != 0xff) return false;
        while (offset < frame->len && data[offset] == 0xff) ++offset;
        if (offset + 3 > frame->len) return false;
        const uint8_t marker = data[offset++];
        const size_t length = (size_t(data[offset]) << 8) | data[offset + 1];
        if (length < 2 || length > frame->len - offset) return false;
        if (marker == 0xdb)
        {
            // 原生基线 JPEG 使用八位量化表；检查每张表的完整长度和编号。
            size_t table = offset + 2;
            while (table < offset + length)
            {
                const uint8_t selector = data[table];
                if (selector > 3 || offset + length - table < 65) return false;
                quantTables |= 1U << selector;
                table += 65;
            }
        }
        else if (marker == 0xc4)
        {
            // Huffman 表的符号数量决定段内长度，不能仅信任外层 JPEG 首尾。
            size_t table = offset + 2;
            while (table < offset + length)
            {
                if (offset + length - table < 17) return false;
                const uint8_t selector = data[table++];
                if ((selector >> 4) > 1 || (selector & 0x0f) > 3) return false;
                unsigned symbols = 0;
                for (unsigned index = 0; index < 16; ++index) symbols += data[table++];
                if (symbols == 0 || symbols > 256 || symbols > offset + length - table) return false;
                huffTables |= 1U << (((selector >> 4) * 4) + (selector & 0x0f));
                table += symbols;
            }
        }
        else if (marker == 0xc0)
        {
            // 原生基线 JPEG 的分量编号必须唯一，宽高必须与驱动帧描述一致。
            if (length < 8 || data[offset + 2] != 8) return false;
            count = data[offset + 7];
            const unsigned height = (unsigned(data[offset + 3]) << 8) | data[offset + 4];
            const unsigned width = (unsigned(data[offset + 5]) << 8) | data[offset + 6];
            if (count == 0 || count > components.size() || length != 8 + count * 3 ||
                width != frame->width || height != frame->height) return false;
            for (unsigned index = 0; index < count; ++index)
            {
                const size_t item = offset + 8 + index * 3;
                components[index] = data[item];
                const uint8_t quant = data[item + 2];
                if (quant > 3 || (quantTables & (1U << quant)) == 0) return false;
                for (unsigned old = 0; old < index; ++old)
                {
                    if (components[old] == components[index]) return false;
                }
            }
        }
        else if (marker == 0xda)
        {
            // SOS 必须引用 SOF 中真实存在的分量；该检查可识别此前丢字节的扫描头。
            if (count == 0 || length != 6 + count * 2 || data[offset + 2] != count) return false;
            unsigned selected = 0;
            for (unsigned index = 0; index < count; ++index)
            {
                const uint8_t component = data[offset + 3 + index * 2];
                const uint8_t huff = data[offset + 4 + index * 2];
                if ((huff >> 4) > 3 || (huff & 0x0f) > 3 ||
                    (huffTables & (1U << (huff >> 4))) == 0 ||
                    (huffTables & (1U << (4 + (huff & 0x0f)))) == 0) return false;
                unsigned found = count;
                for (unsigned old = 0; old < count; ++old)
                {
                    if (components[old] == component) found = old;
                }
                if (found == count || (selected & (1U << found)) != 0) return false;
                selected |= 1U << found;
            }
            if (data[offset + length - 3] != 0 || data[offset + length - 2] != 63 ||
                data[offset + length - 1] != 0) return false;
            offset += length;
            while (offset + 1 < frame->len)
            {
                // 压缩数据中的 0xff 必须转义或构成重启标记，最后才允许 EOI。
                const void* found = std::memchr(data + offset, 0xff, frame->len - offset);
                if (found == nullptr) return false;
                offset = static_cast<const uint8_t*>(found) - data;
                if (offset + 1 >= frame->len) return false;
                const uint8_t next = data[offset + 1];
                if (next == 0xd9) return offset + 2 == frame->len;
                if (next != 0 && (next < 0xd0 || next > 0xd7)) return false;
                offset += 2;
            }
            return false;
        }
        offset += length;
    }
    return false;
}

// 输入：无；输出：恢复结果；作用：在相机锁内重新应用分辨率及曝光，恢复异常的 JPEG 编码状态。
esp_err_t CameraWeb::recoverCamera()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    ESP_LOGI(TAG, "recoverCamera() >> size=%d", m_size);
    const bool applied = applySetting("size", m_size);
    if (!applied)
    {
        ESP_LOGE(TAG, "recoverCamera() << result=REAPPLY_FAILED");
        return ESP_FAIL;
    }
    cacheStatus();
    m_cameraResets.fetch_add(1);
    ESP_LOGI(TAG, "recoverCamera() << result=OK");
    return ESP_OK;
}

esp_err_t CameraWeb::initWifi()
{
    ESP_LOGI(TAG, "initWifi() >> mode=APSTA ap_ssid=%s", CONFIG_CAMERA_AP_SSID);
    static_assert(sizeof(CONFIG_CAMERA_AP_SSID) > 1 && sizeof(CONFIG_CAMERA_AP_SSID) <= 33);
    static_assert(sizeof(CONFIG_CAMERA_AP_PASSWORD) >= 9 && sizeof(CONFIG_CAMERA_AP_PASSWORD) <= 64);
    static_assert(sizeof(WIFI_SSID) > 1 && sizeof(WIFI_SSID) <= 33);
    static_assert(sizeof(WIFI_PASSWORD) <= 65);
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // 只在 NVS 无法正常初始化时重建分区，不在每次启动时清除配置。
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase failed");
        result = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(result, TAG, "nvs init failed");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop failed");
    ESP_LOGI(TAG, "esp_netif_create_default_wifi_ap() >>");
    esp_netif_t* apNetif = esp_netif_create_default_wifi_ap();
    ESP_LOGI(TAG, "esp_netif_create_default_wifi_ap() << ready=%d", apNetif != nullptr);
    ESP_LOGI(TAG, "esp_netif_create_default_wifi_sta() >>");
    esp_netif_t* staNetif = esp_netif_create_default_wifi_sta();
    ESP_LOGI(TAG, "esp_netif_create_default_wifi_sta() << ready=%d", staNetif != nullptr);
    if (apNetif == nullptr || staNetif == nullptr)
    {
        // 两个默认接口都创建成功才允许启动双模式。
        ESP_LOGE(TAG, "initWifi() << result=NO_MEMORY");
        return ESP_ERR_NO_MEM;
    }
    const wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_LOGI(TAG, "Wi-Fi TX config type=%d static_buffers=%d tcp_snd_buf=%d",
             init.tx_buf_type, init.static_tx_buf_num, CONFIG_LWIP_TCP_SND_BUF_DEFAULT);
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "wifi init failed");
    // 两种模式均使用内存配置，不把密码写入 Wi-Fi 配置的 NVS 副本或日志。
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, onWifiEvent, this, &m_wifiEvent), TAG, "wifi event failed");
    // 热点固定本地地址，STA 地址由路由器 DHCP 分配；不启用互联网转发。
    esp_netif_ip_info_t ip = {};
    ip.ip.addr = ESP_IP4TOADDR(192, 168, 4, 1);
    ip.gw.addr = ip.ip.addr;
    ip.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);
    ESP_LOGI(TAG, "esp_netif_dhcps_stop() >> interface=AP");
    result = esp_netif_dhcps_stop(apNetif);
    ESP_LOGI(TAG, "esp_netif_dhcps_stop() << result=%s", esp_err_to_name(result));
    ESP_RETURN_ON_ERROR(result, TAG, "esp_netif_dhcps_stop failed");
    ESP_LOGI(TAG, "esp_netif_set_ip_info() >> interface=AP ip=192.168.4.1 mask=255.255.255.0");
    result = esp_netif_set_ip_info(apNetif, &ip);
    ESP_LOGI(TAG, "esp_netif_set_ip_info() << result=%s", esp_err_to_name(result));
    ESP_RETURN_ON_ERROR(result, TAG, "esp_netif_set_ip_info failed");
    ESP_LOGI(TAG, "esp_netif_dhcps_start() >> interface=AP");
    result = esp_netif_dhcps_start(apNetif);
    ESP_LOGI(TAG, "esp_netif_dhcps_start() << result=%s", esp_err_to_name(result));
    ESP_RETURN_ON_ERROR(result, TAG, "esp_netif_dhcps_start failed");
    const esp_timer_create_args_t timer = {
        .callback = reconnectWifi,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "wifi_retry",
        .skip_unhandled_events = false,
    };
    ESP_LOGI(TAG, "esp_timer_create() >> name=wifi_retry");
    result = esp_timer_create(&timer, &m_retryTimer);
    ESP_LOGI(TAG, "esp_timer_create() << result=%s", esp_err_to_name(result));
    ESP_RETURN_ON_ERROR(result, TAG, "esp_timer_create failed");
    ESP_LOGI(TAG, "esp_event_handler_instance_register() >> base=IP_EVENT id=ANY");
    result = esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, onWifiEvent, this, &m_ipEvent);
    ESP_LOGI(TAG, "esp_event_handler_instance_register() << result=%s", esp_err_to_name(result));
    ESP_RETURN_ON_ERROR(result, TAG, "esp_event_handler_instance_register failed");
    wifi_config_t ap = {};
    std::memcpy(ap.ap.ssid, CONFIG_CAMERA_AP_SSID, sizeof(CONFIG_CAMERA_AP_SSID) - 1);
    ap.ap.ssid_len = sizeof(CONFIG_CAMERA_AP_SSID) - 1;
    std::memcpy(ap.ap.password, CONFIG_CAMERA_AP_PASSWORD, sizeof(CONFIG_CAMERA_AP_PASSWORD) - 1);
    // 尚未连接路由器时使用信道 6；连接后热点由驱动跟随路由器信道。
    ap.ap.channel = 6;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap.ap.max_connection = 2;
    ap.ap.pmf_cfg.required = false;
    wifi_config_t sta = {};
    std::memcpy(sta.sta.ssid, WIFI_SSID, sizeof(WIFI_SSID) - 1);
    std::memcpy(sta.sta.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD) - 1);
    sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    sta.sta.pmf_cfg.capable = true;
    sta.sta.pmf_cfg.required = false;
    ESP_LOGI(TAG, "esp_wifi_set_mode() >> mode=APSTA");
    result = esp_wifi_set_mode(WIFI_MODE_APSTA);
    ESP_LOGI(TAG, "esp_wifi_set_mode() << result=%s", esp_err_to_name(result));
    ESP_RETURN_ON_ERROR(result, TAG, "esp_wifi_set_mode failed");
    ESP_LOGI(TAG, "esp_wifi_set_config() >> interface=AP auth=WPA2 max_clients=2");
    result = esp_wifi_set_config(WIFI_IF_AP, &ap);
    ESP_LOGI(TAG, "esp_wifi_set_config() << result=%s", esp_err_to_name(result));
    ESP_RETURN_ON_ERROR(result, TAG, "esp_wifi_set_config failed");
    ESP_LOGI(TAG, "esp_wifi_set_config() >> interface=STA auth_min=WPA2");
    result = esp_wifi_set_config(WIFI_IF_STA, &sta);
    ESP_LOGI(TAG, "esp_wifi_set_config() << result=%s", esp_err_to_name(result));
    ESP_RETURN_ON_ERROR(result, TAG, "esp_wifi_set_config failed");
    ESP_LOGI(TAG, "esp_wifi_start() >> mode=APSTA");
    result = esp_wifi_start();
    ESP_LOGI(TAG, "esp_wifi_start() << result=%s", esp_err_to_name(result));
    ESP_RETURN_ON_ERROR(result, TAG, "esp_wifi_start failed");
    // 两种接入方式均显式关闭调制解调器省电，并回读状态，避免仅依赖默认策略。
    ESP_LOGI(TAG, "esp_wifi_set_ps() >> type=WIFI_PS_NONE");
    result = esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "esp_wifi_set_ps() << result=%s", esp_err_to_name(result));
    ESP_RETURN_ON_ERROR(result, TAG, "wifi power mode failed");
    wifi_ps_type_t powerSave = WIFI_PS_MIN_MODEM;
    ESP_LOGI(TAG, "esp_wifi_get_ps() >>");
    result = esp_wifi_get_ps(&powerSave);
    ESP_LOGI(TAG, "esp_wifi_get_ps() << result=%s type=%d", esp_err_to_name(result), int(powerSave));
    ESP_RETURN_ON_ERROR(result, TAG, "wifi power mode readback failed");
    if (powerSave != WIFI_PS_NONE)
    {
        ESP_LOGE(TAG, "initWifi() << result=POWER_SAVE_NOT_DISABLED");
        return ESP_FAIL;
    }
    // SDK 参数单位为 0.25 dBm，80 表示 20 dBm 的发送功率上限；逐包功率仍受 PHY 约束。
    ESP_LOGI(TAG, "esp_wifi_set_max_tx_power() >> quarter_dbm=80");
    result = esp_wifi_set_max_tx_power(80);
    ESP_LOGI(TAG, "esp_wifi_set_max_tx_power() << result=%s", esp_err_to_name(result));
    ESP_RETURN_ON_ERROR(result, TAG, "wifi tx power failed");
    int8_t txPower = 0;
    ESP_LOGI(TAG, "esp_wifi_get_max_tx_power() >>");
    result = esp_wifi_get_max_tx_power(&txPower);
    ESP_LOGI(TAG, "esp_wifi_get_max_tx_power() << result=%s quarter_dbm=%d",
             esp_err_to_name(result), int(txPower));
    ESP_RETURN_ON_ERROR(result, TAG, "wifi tx power readback failed");
    ESP_LOGI(TAG, "power settings cpu_mhz=%u wifi_ps=%d max_tx_quarter_dbm=%d",
             unsigned(esp_rom_get_cpu_ticks_per_us()), int(powerSave), int(txPower));
    ESP_LOGI(TAG, "initWifi() << result=OK mode=APSTA");
    return ESP_OK;
}

// 输入：Wi-Fi/IP 事件；输出：无；作用：独立维护 AP 与 STA 状态，断线不阻塞事件任务。
void CameraWeb::onWifiEvent(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    auto* app = static_cast<CameraWeb*>(arg);
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        // 取得地址才允许从路由器侧访问；AP 侧就绪状态不变。
        const auto* event = static_cast<ip_event_got_ip_t*>(data);
        app->m_staIp.store(event->ip_info.ip.addr);
        app->m_staReady.store(true);
        app->m_connected.store(true);
        ESP_LOGI(TAG, "esp_timer_stop() >> name=wifi_retry");
        const esp_err_t result = esp_timer_stop(app->m_retryTimer);
        ESP_LOGI(TAG, "esp_timer_stop() << result=%s", esp_err_to_name(result));
        ESP_LOGI("camera_main", "STA connected; open http://" IPSTR "/; AP http://192.168.4.1/", IP2STR(&event->ip_info.ip));
        return;
    }
    if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP)
    {
        // DHCP 地址丢失不关闭热点；实际断开事件负责启动重连计时。
        const bool wasReady = app->m_staReady.exchange(false);
        app->m_staIp.store(0);
        app->m_connected.store(app->m_apReady.load());
        // 仅撤销曾经在线的路由器连接；后台重试失败不能反复打断热点控制。
        if (wasReady)
        {
            app->requestMotionStop();
            if (!app->m_apReady.load()) app->m_epoch.fetch_add(1);
        }
        ESP_LOGW(TAG, "STA lost IP; AP ready=%d", int(app->m_apReady.load()));
        return;
    }
    if (base != WIFI_EVENT) return;
    if (id == WIFI_EVENT_AP_START)
    {
        app->m_apReady.store(true);
        app->m_connected.store(true);
        ESP_LOGI("camera_main", "SoftAP started ssid=%s; open http://192.168.4.1/", CONFIG_CAMERA_AP_SSID);
    }
    else if (id == WIFI_EVENT_AP_STOP)
    {
        // AP 停止时仍可通过已有 STA 地址提供服务，运动则保守撤销。
        app->m_apReady.store(false);
        app->m_connected.store(app->m_staReady.load());
        app->requestMotionStop();
        if (!app->m_staReady.load()) app->m_epoch.fetch_add(1);
        ESP_LOGW(TAG, "SoftAP stopped; STA ready=%d", int(app->m_staReady.load()));
    }
    else if (id == WIFI_EVENT_AP_STACONNECTED)
    {
        const auto* event = static_cast<wifi_event_ap_staconnected_t*>(data);
        ESP_LOGI(TAG, "SoftAP client connected aid=%u", event->aid);
    }
    else if (id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        // 任一接入设备离开都先停车，不自动恢复运动；不中断另一接口的视频。
        app->requestMotionStop();
        const auto* event = static_cast<wifi_event_ap_stadisconnected_t*>(data);
        ESP_LOGI(TAG, "SoftAP client disconnected aid=%u reason=%u", event->aid, event->reason);
        wifi_sta_list_t clients = {};
        ESP_LOGI(TAG, "esp_wifi_ap_get_sta_list() >>");
        const esp_err_t result = esp_wifi_ap_get_sta_list(&clients);
        ESP_LOGI(TAG, "esp_wifi_ap_get_sta_list() << result=%s clients=%d",
                 esp_err_to_name(result), result == ESP_OK ? clients.num : -1);
        if (result == ESP_OK && clients.num == 0 && !app->m_staReady.load())
        {
            // 只有没有另一可用接口时，才撤销最后一个热点客户端留下的视频代次。
            app->m_epoch.fetch_add(1);
        }
    }
    else if (id == WIFI_EVENT_STA_START)
    {
        reconnectWifi(app);
    }
    else if (id == WIFI_EVENT_STA_DISCONNECTED)
    {
        const auto* event = static_cast<wifi_event_sta_disconnected_t*>(data);
        const bool wasReady = app->m_staReady.exchange(false);
        app->m_staIp.store(0);
        app->m_connected.store(app->m_apReady.load());
        // 仅撤销曾经在线的路由器连接；后台重试失败不能反复打断热点控制。
        if (wasReady)
        {
            app->requestMotionStop();
            if (!app->m_apReady.load()) app->m_epoch.fetch_add(1);
        }
        ESP_LOGW(TAG, "STA disconnected reason=%u; AP ready=%d; retry in 3 seconds",
                 event->reason, int(app->m_apReady.load()));
        ESP_LOGI(TAG, "esp_timer_stop() >> name=wifi_retry");
        const esp_err_t stopped = esp_timer_stop(app->m_retryTimer);
        ESP_LOGI(TAG, "esp_timer_stop() << result=%s", esp_err_to_name(stopped));
        ESP_LOGI(TAG, "esp_timer_start_once() >> name=wifi_retry timeout_us=3000000");
        const esp_err_t result = esp_timer_start_once(app->m_retryTimer, 3000000);
        if (result == ESP_OK) ESP_LOGI(TAG, "esp_timer_start_once() << result=ESP_OK");
        else ESP_LOGW(TAG, "esp_timer_start_once() << result=%s", esp_err_to_name(result));
    }
}

// 输入：固件实例；输出：无；作用：只根据 STA 状态重试，AP 运行不能抑制路由器连接。
void CameraWeb::reconnectWifi(void* arg)
{
    auto* app = static_cast<CameraWeb*>(arg);
    ESP_LOGI(TAG, "reconnectWifi() >> sta_ready=%d", int(app->m_staReady.load()));
    if (app->m_staReady.load())
    {
        // 已取得地址的连接不重复调用连接接口。
        ESP_LOGI(TAG, "reconnectWifi() << result=ALREADY_CONNECTED");
        return;
    }
    ESP_LOGI(TAG, "esp_wifi_connect() >> interface=STA");
    const esp_err_t result = esp_wifi_connect();
    ESP_LOGI(TAG, "esp_wifi_connect() << result=%s", esp_err_to_name(result));
    if (result != ESP_OK)
    {
        // 同步失败时三秒后重试；异步断开由 Wi-Fi 事件统一安排，不累计重试队列。
        ESP_LOGI(TAG, "esp_timer_start_once() >> name=wifi_retry timeout_us=3000000");
        const esp_err_t retry = esp_timer_start_once(app->m_retryTimer, 3000000);
        ESP_LOGW(TAG, "esp_timer_start_once() << result=%s", esp_err_to_name(retry));
    }
    if (result == ESP_OK) ESP_LOGI(TAG, "reconnectWifi() << result=ESP_OK");
    else ESP_LOGW(TAG, "reconnectWifi() << result=%s", esp_err_to_name(result));
}

esp_err_t CameraWeb::startServers()
{
    // 三个服务按内部九项加 4+1+3 个客户端保守预算，余量用于 DHCP 等网络操作。
    static_assert(CONFIG_LWIP_MAX_SOCKETS >= 20, "Three HTTP servers need at least 20 sockets");
    ESP_LOGI(TAG, "startServers() >>");
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 8192;
#if CONFIG_CAMERA_PERF_TEST
    config.max_uri_handlers = 7;
#else
    config.max_uri_handlers = 6;
#endif
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;
    config.send_wait_timeout = 3;
    config.recv_wait_timeout = 3;
    // 三个服务复用连接配置，所有新连接均禁用 TCP 小包合并等待。
    config.open_fn = onSocketOpen;
    ESP_RETURN_ON_ERROR(httpd_start(&m_control, &config), TAG, "control server failed");
    const char* paths[] = {"/", "/status", "/control", "/stop", "/capture", "/led"};
    for (const char* path : paths)
    {
        httpd_uri_t uri = {};
        uri.uri = path;
        uri.method = (std::strcmp(path, "/control") == 0 ||
                      std::strcmp(path, "/stop") == 0 || std::strcmp(path, "/led") == 0) ? HTTP_POST : HTTP_GET;
        uri.handler = onRequest;
        uri.user_ctx = this;
        const esp_err_t result = httpd_register_uri_handler(m_control, &uri);
        if (result != ESP_OK)
        {
            httpd_stop(m_control);
            m_control = nullptr;
            return result;
        }
    }
#if CONFIG_CAMERA_PERF_TEST
    // 测量接口只在专用测试构建中注册，常规固件不暴露停止相机的操作。
    httpd_uri_t perfUri = {};
    perfUri.uri = "/perf";
    perfUri.method = HTTP_GET;
    perfUri.handler = onRequest;
    perfUri.user_ctx = this;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(m_control, &perfUri), TAG, "perf route failed");
#endif
    // 长时间传输的视频独占另一 HTTP 任务，控制页面仍可响应。
    config.server_port = 81;
    config.ctrl_port += 1;
    config.max_uri_handlers = 1;
    config.max_open_sockets = 1;
    // 仅放宽视频 TCP 的短暂拥塞容忍；运动连接看门期限和超时不变。
    config.send_wait_timeout = 5;
    const esp_err_t result = httpd_start(&m_stream, &config);
    if (result != ESP_OK)
    {
        httpd_stop(m_control);
        m_control = nullptr;
        return result;
    }
    httpd_uri_t uri = {};
    uri.uri = "/stream";
    uri.method = HTTP_GET;
    uri.handler = onStream;
    uri.user_ctx = this;
    const esp_err_t registered = httpd_register_uri_handler(m_stream, &uri);
    if (registered != ESP_OK)
    {
        httpd_stop(m_stream);
        httpd_stop(m_control);
        m_stream = nullptr;
        m_control = nullptr;
        return registered;
    }
    // 运动单独使用服务任务；相机取帧、抓拍发送及参数设置不占用此请求队列。
    config.server_port = 82;
    config.ctrl_port += 1;
    config.stack_size = 4096;
    config.task_priority = tskIDLE_PRIORITY + 6;
    config.max_uri_handlers = 5;
    config.max_open_sockets = 3;
    // 不因临时只读请求淘汰仍有效的控制长连接。
    config.lru_purge_enable = false;
    config.close_fn = closeMotorSocket;
    // TCP 保活作为连接巡检的补充，清理切换网络后遗留的半开连接。
    config.keep_alive_enable = true;
    config.keep_alive_idle = 3;
    config.keep_alive_interval = 1;
    config.keep_alive_count = 2;
    config.recv_wait_timeout = 1;
    config.send_wait_timeout = 1;
    ESP_LOGI(TAG, "httpd_start() >> port=82 priority=6 stack_bytes=4096 max_clients=3 timeout_s=1");
    esp_err_t motionResult = httpd_start(&m_motion, &config);
    ESP_LOGI(TAG, "httpd_start() << result=%s port=82", esp_err_to_name(motionResult));
    if (motionResult == ESP_OK)
    {
        // 同一路径的 GET 只读状态，POST 才能续期或停车，均不接触相机锁。
        for (httpd_method_t method : {HTTP_GET, HTTP_POST})
        {
            httpd_uri_t motorUri = {};
            motorUri.uri = "/motor";
            motorUri.method = method;
            motorUri.handler = onMotor;
            motorUri.user_ctx = this;
            ESP_LOGI(TAG, "httpd_register_uri_handler() >> port=82 uri=/motor method=%s",
                     method == HTTP_GET ? "GET" : "POST");
            motionResult = httpd_register_uri_handler(m_motion, &motorUri);
            ESP_LOGI(TAG, "httpd_register_uri_handler() << result=%s", esp_err_to_name(motionResult));
            if (motionResult != ESP_OK) break;
        }
    }
    if (motionResult == ESP_OK)
    {
        httpd_uri_t socketUri = {};
        socketUri.uri = "/motor/ws";
        socketUri.method = HTTP_GET;
        socketUri.handler = onMotorSocket;
        socketUri.user_ctx = this;
        socketUri.is_websocket = true;
        socketUri.ws_pre_handshake_cb = openMotorSocket;
        ESP_LOGI(TAG, "httpd_register_uri_handler() >> port=82 uri=/motor/ws websocket=1");
        motionResult = httpd_register_uri_handler(m_motion, &socketUri);
        ESP_LOGI(TAG, "httpd_register_uri_handler() << result=%s", esp_err_to_name(motionResult));
    }
    if (motionResult == ESP_OK)
    {
        // 与车轮命令在同一 HTTP 任务串行执行，避免舵机操作占用相机服务。
        for (httpd_method_t method : {HTTP_GET, HTTP_POST})
        {
            httpd_uri_t armUri = {};
            armUri.uri = "/arm";
            armUri.method = method;
            armUri.handler = onArm;
            armUri.user_ctx = this;
            ESP_LOGI(TAG, "httpd_register_uri_handler() >> port=82 uri=/arm method=%d", int(method));
            motionResult = httpd_register_uri_handler(m_motion, &armUri);
            ESP_LOGI(TAG, "httpd_register_uri_handler() << result=%s", esp_err_to_name(motionResult));
            if (motionResult != ESP_OK) break;
        }
    }
    if (motionResult == ESP_OK)
    {
        // 连接巡检由独立定时器投递，视频取帧和主任务采样不能阻止旧控制连接释放。
        const esp_timer_create_args_t timer = {
            .callback = queueMotionCheck,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "motion_link",
            .skip_unhandled_events = true,
        };
        ESP_LOGI(TAG, "esp_timer_create() >> name=motion_link");
        motionResult = esp_timer_create(&timer, &m_motionTimer);
        ESP_LOGI(TAG, "esp_timer_create() << result=%s", esp_err_to_name(motionResult));
        if (motionResult == ESP_OK)
        {
            ESP_LOGI(TAG, "esp_timer_start_periodic() >> name=motion_link period_us=500000");
            motionResult = esp_timer_start_periodic(m_motionTimer, 500000);
            ESP_LOGI(TAG, "esp_timer_start_periodic() << result=%s", esp_err_to_name(motionResult));
        }
    }
    if (motionResult != ESP_OK)
    {
        // 巡检未启动成功时也释放已创建的定时器，不保留半启动的连接回收流程。
        if (m_motionTimer != nullptr)
        {
            ESP_LOGI(TAG, "esp_timer_delete() >> name=motion_link");
            const esp_err_t deleted = esp_timer_delete(m_motionTimer);
            ESP_LOGI(TAG, "esp_timer_delete() << result=%s", esp_err_to_name(deleted));
            m_motionTimer = nullptr;
        }
        // 运动服务不可用时不留下半启动状态，关闭已创建的服务并保持电机停止。
        m_motor.stop("motion server failed");
        if (m_motion != nullptr) httpd_stop(m_motion);
        httpd_stop(m_stream);
        httpd_stop(m_control);
        m_motion = nullptr;
        m_stream = nullptr;
        m_control = nullptr;
        ESP_LOGE(TAG, "startServers() << result=%s", esp_err_to_name(motionResult));
        return motionResult;
    }
    ESP_LOGI(TAG, "startServers() << result=OK control=80 stream=81 motor=82 socket_budget=%d",
             CONFIG_LWIP_MAX_SOCKETS);
    return ESP_OK;
}

esp_err_t CameraWeb::onSocketOpen(httpd_handle_t server, int socketFd)
{
    const int enabled = 1;
    // HTTP 头、分块长度和 MJPEG 边界都是小包；禁用 Nagle，避免等待合并后才发送。
    if (setsockopt(socketFd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled)) != 0)
    {
        ESP_LOGE(TAG, "onSocketOpen() << result=SET_NODELAY_FAILED socket=%d errno=%d", socketFd, errno);
        return ESP_FAIL;
    }
    int actual = 0;
    socklen_t length = sizeof(actual);
    if (getsockopt(socketFd, IPPROTO_TCP, TCP_NODELAY, &actual, &length) != 0 || actual != 1)
    {
        // 读取实际状态，不能仅凭设置调用成功就宣称新连接已使用低延迟模式。
        ESP_LOGE(TAG, "onSocketOpen() << result=VERIFY_NODELAY_FAILED socket=%d errno=%d value=%d",
                 socketFd, errno, actual);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "onSocketOpen() << result=OK socket=%d tcp_nodelay=%d", socketFd, actual);
    return ESP_OK;
}

esp_err_t CameraWeb::onRequest(httpd_req_t* req)
{
    auto* app = static_cast<CameraWeb*>(req->user_ctx);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
#if CONFIG_CAMERA_PERF_TEST
    if (std::strncmp(req->uri, "/perf", 5) == 0 &&
        (req->uri[5] == '\0' || req->uri[5] == '?'))
    {
        return app->sendPerf(req);
    }
#endif
    if (std::strcmp(req->uri, "/") == 0)
    {
        // HTML 随固件编译嵌入，不依赖电脑上的开发服务器或外部 CDN。
        extern const char pageStart[] asm("_binary_index_html_start");
        httpd_resp_set_type(req, "text/html; charset=utf-8");
        return httpd_resp_send(req, pageStart, HTTPD_RESP_USE_STRLEN);
    }
    if (std::strcmp(req->uri, "/led") == 0)
    {
        return app->setLed(req);
    }
    if (std::strcmp(req->uri, "/status") == 0)
    {
        return app->sendStatus(req);
    }
    if (std::strcmp(req->uri, "/control") == 0)
    {
        return app->setControl(req);
    }
    if (std::strcmp(req->uri, "/stop") == 0)
    {
        // 代次变化使旧循环退出；下次启动的视频使用新的代次，不受旧请求影响。
        app->m_epoch.fetch_add(1);
        return httpd_resp_sendstr(req, "OK");
    }
    if (std::strcmp(req->uri, "/capture") == 0)
    {
        const int64_t started = esp_timer_get_time();
        int64_t lockUs;
        camera_fb_t* frame = nullptr;
        {
            std::lock_guard<std::mutex> lock(app->m_mutex);
            lockUs = esp_timer_get_time() - started;
            // 最多清理两个旧缓冲并重试；连续损坏时重新应用当前传感器配置。
            for (unsigned attempt = 0; attempt < 5; ++attempt)
            {
                ESP_LOGI(TAG, "capture esp_camera_fb_get() >> attempt=%u", attempt);
                frame = esp_camera_fb_get();
                ESP_LOGI(TAG, "capture esp_camera_fb_get() << present=%d", int(frame != nullptr));
                if (frame == nullptr || isValidJpeg(frame)) break;
                app->m_jpegErrors.fetch_add(1);
                ESP_LOGW(TAG, "capture JPEG rejected bytes=%u", unsigned(frame->len));
                ESP_LOGI(TAG, "capture esp_camera_fb_return() >>");
                esp_camera_fb_return(frame);
                ESP_LOGI(TAG, "capture esp_camera_fb_return() <<");
                frame = nullptr;
                if (attempt == 1 && !app->applySetting("size", app->m_size))
                {
                    // 重新设置失败立即返回错误，不能继续发送无法解码的内容。
                    break;
                }
            }
        }
        const int64_t gotFrame = esp_timer_get_time();
        // 帧缓冲由驱动拥有，使用 RAII 保证发送失败时也归还。
        std::unique_ptr<camera_fb_t, decltype(&esp_camera_fb_return)>
            holder(frame, esp_camera_fb_return);
        if (frame == nullptr)
        {
            ESP_LOGW(TAG, "capture esp_camera_fb_get() << result=NO_FRAME elapsed_ms=%lld",
                     (long long)((gotFrame - started) / 1000));
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Capture failed");
        }
        ESP_LOGI(TAG, "capture esp_camera_fb_get() << bytes=%u capture_ms=%lld lock_ms=%lld",
                 unsigned(frame->len), (long long)((gotFrame - started - lockUs) / 1000),
                 (long long)(lockUs / 1000));
        httpd_resp_set_type(req, "image/jpeg");
        ESP_LOGI(TAG, "capture httpd_resp_send() >> bytes=%u", unsigned(frame->len));
        const esp_err_t result = httpd_resp_send(req, reinterpret_cast<const char*>(frame->buf), frame->len);
        const int sendErrno = errno;
        ESP_LOGI(TAG, "capture httpd_resp_send() << result=%s errno=%d send_ms=%lld",
                 esp_err_to_name(result), result == ESP_OK ? 0 : sendErrno,
                 (long long)((esp_timer_get_time() - gotFrame) / 1000));
        return result;
    }
    return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
}

#if CONFIG_CAMERA_PERF_TEST
// 输入：可选的相机开关查询；输出：固定 64 KiB 数据；作用：隔离采图与 TCP 传输的影响。
esp_err_t CameraWeb::sendPerf(httpd_req_t* req)
{
    ESP_LOGI(TAG, "sendPerf() >> uri=%s", req->uri);
    char query[64] = {};
    char mode[12] = {};
    char clock[12] = {};
    char channel[8] = {};
    char frequency[12] = {};
    char drive[8] = {};
    char pll[8] = {};
    char buffers[8] = {};
    char dvp[8] = {};
    char sample[8] = {};
    httpd_req_get_url_query_str(req, query, sizeof(query));
    httpd_query_key_value(query, "camera", mode, sizeof(mode));
    httpd_query_key_value(query, "clock", clock, sizeof(clock));
    httpd_query_key_value(query, "channel", channel, sizeof(channel));
    httpd_query_key_value(query, "xclk", frequency, sizeof(frequency));
    httpd_query_key_value(query, "drive", drive, sizeof(drive));
    httpd_query_key_value(query, "pll", pll, sizeof(pll));
    httpd_query_key_value(query, "buffers", buffers, sizeof(buffers));
    httpd_query_key_value(query, "dvp", dvp, sizeof(dvp));
    httpd_query_key_value(query, "sample", sample, sizeof(sample));
    if (clock[0] != '\0' || pll[0] != '\0' || dvp[0] != '\0')
    {
        // S3 相机由 LCD_CAM 提供时钟；在任何硬件操作前拒绝旧板的 LEDC/OV2640 测量参数。
        ESP_LOGW(TAG, "sendPerf() << result=UNSUPPORTED_CLOCK_TEST");
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "LEDC clock and OV2640 pll/dvp tests are unavailable on S3");
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        // 只允许在没有预览时切换硬件，避免销毁仍由视频发送任务持有的帧。
        if (m_streaming)
        {
            ESP_LOGW(TAG, "sendPerf() << result=STREAM_BUSY");
            httpd_resp_set_status(req, "409 Conflict");
            return httpd_resp_sendstr(req, "Stop stream first");
        }
        esp_err_t result = ESP_OK;
        if (buffers[0] != '\0')
        {
            // 比较帧缓冲数时完整重建驱动，其他初始化参数使用同一组构建配置。
            const int count = std::atoi(buffers);
            if (count < 1 || count > 3)
            {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid buffer count");
            }
            if (count != m_frameCount)
            {
                ESP_LOGI(TAG, "perf rebuild camera >> buffers=%d", count);
                result = esp_camera_deinit();
                if (result == ESP_OK)
                {
                    m_frameCount = count;
                    result = initCamera();
                }
                ESP_LOGI(TAG, "perf rebuild camera << result=%s", esp_err_to_name(result));
            }
        }
        if (std::strcmp(mode, "off") == 0 && esp_camera_sensor_get() != nullptr)
        {
            ESP_LOGI(TAG, "esp_camera_deinit() >>");
            result = esp_camera_deinit();
            ESP_LOGI(TAG, "esp_camera_deinit() << result=%s", esp_err_to_name(result));
        }
        else if (std::strcmp(mode, "on") == 0 && esp_camera_sensor_get() == nullptr)
        {
            result = initCamera();
        }
        if (result == ESP_OK && esp_camera_sensor_get() != nullptr && frequency[0] != '\0')
        {
            // OV3660 时钟测量只接受驱动支持的 16/20 MHz，不使用旧板低频参数。
            const int hz = std::atoi(frequency);
            // 驱动同步保存时钟值，避免测量换频后曝光换算仍使用旧频率。
            ESP_LOGI(TAG, "sensor set_xclk() >> timer=0 hz=%d", hz);
            result = hz == 16000000 || hz == 20000000 ?
                esp_camera_sensor_get()->set_xclk(esp_camera_sensor_get(), LEDC_TIMER_0, hz / 1000000) :
                ESP_ERR_INVALID_ARG;
            ESP_LOGI(TAG, "sensor set_xclk() << result=%s configured_hz=%u", esp_err_to_name(result),
                     unsigned(esp_camera_sensor_get()->xclk_freq_hz));
            if (result == ESP_OK)
            {
                // 外部时钟变化后重新选择匹配的 OV3660 PLL，避免缓存时序与真实配置脱节。
                sensor_t* sensor = esp_camera_sensor_get();
                ESP_LOGI(TAG, "sensor set_framesize() >> size=%d", int(sensor->status.framesize));
                result = sensor->set_framesize(sensor, sensor->status.framesize) == 0 ? ESP_OK : ESP_FAIL;
                ESP_LOGI(TAG, "sensor set_framesize() << result=%s", esp_err_to_name(result));
                m_autoVts = 0;
            }
        }
        if (result == ESP_OK && esp_camera_sensor_get() != nullptr && drive[0] != '\0')
        {
            // 保持频率与内存不变，只比较 GPIO15 输出驱动强度，测试后恢复默认档位。
            gpio_drive_cap_t strength = GPIO_DRIVE_CAP_2;
            if (std::strcmp(drive, "0") == 0)
            {
                strength = GPIO_DRIVE_CAP_0;
            }
            else if (std::strcmp(drive, "2") != 0)
            {
                result = ESP_ERR_INVALID_ARG;
            }
            if (result == ESP_OK)
            {
                ESP_LOGI(TAG, "gpio_set_drive_capability() >> gpio=15 level=%s", drive);
                result = gpio_set_drive_capability(static_cast<gpio_num_t>(board::CAM_XCLK), strength);
                ESP_LOGI(TAG, "gpio_set_drive_capability() << result=%s", esp_err_to_name(result));
            }
        }
        // 测量接口可能关闭或重建相机，及时使缓存失效或发布重建后的状态。
        cacheStatus();
        if (result != ESP_OK)
        {
            ESP_LOGE(TAG, "sendPerf() << result=%s", esp_err_to_name(result));
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera switch failed");
        }
        if (sample[0] != '\0')
        {
            const int count = std::atoi(sample);
            if (count < 2 || count > 30 || esp_camera_sensor_get() == nullptr)
            {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid sample count");
            }
            int captured = 0;
            int64_t firstUs = 0;
            int64_t lastUs = 0;
            // 测量窗口内不发送图像，取帧后立即归还，单独得到采集通路的节拍上限。
            // 首帧可能已缓存，不把它算作新采样；遇到超时立即退出，最多等待一次驱动超时。
            for (int index = 0; index < count; ++index)
            {
                ESP_LOGI(TAG, "bench esp_camera_fb_get() >> index=%d", index);
                camera_fb_t* frame = esp_camera_fb_get();
                const int64_t now = esp_timer_get_time();
                ESP_LOGI(TAG, "bench esp_camera_fb_get() << index=%d result=%s", index,
                         frame != nullptr ? "OK" : "NO_FRAME");
                if (frame == nullptr)
                {
                    break;
                }
                if (captured == 0)
                {
                    firstUs = now;
                }
                lastUs = now;
                ++captured;
                esp_camera_fb_return(frame);
            }
            char json[160];
            const double fps = captured > 1 && lastUs > firstUs ?
                (captured - 1) * 1000000.0 / (lastUs - firstUs) : 0;
            std::snprintf(json, sizeof(json), "{\"captured\":%d,\"fps\":%.3f,\"elapsed_ms\":%lld,\"buffers\":%d}",
                          captured, fps, (long long)((lastUs - firstUs) / 1000), m_frameCount);
            ESP_LOGI(TAG, "sendPerf() << %s", json);
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_sendstr(req, json);
        }
    }
#if CONFIG_CAMERA_WIFI_APSTA
    if (channel[0] != '\0')
    {
        // 双模式信道由路由器连接协调，性能测试不能单独更改热点信道。
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "APSTA channel follows router; manual channel changes disabled");
    }
    wifi_sta_list_t clients = {};
    const esp_err_t listResult = esp_wifi_ap_get_sta_list(&clients);
    if (listResult == ESP_OK && clients.num > 0)
    {
        ESP_LOGI(TAG, "perf AP client rssi_dbm=%d phy_n=%u internal_free=%u",
                 int(clients.sta[0].rssi), unsigned(clients.sta[0].phy_11n),
                 unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
    }
#endif
    // 不使用图像数据；固定负载在整个发送期间保持有效，结果不受 JPEG 内容影响。
    std::unique_ptr<char[]> data(new (std::nothrow) char[4096]);
    if (!data)
    {
        ESP_LOGE(TAG, "sendPerf() << result=NO_MEMORY");
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No memory");
    }
    std::memset(data.get(), 0x5a, 4096);
    httpd_resp_set_type(req, "application/octet-stream");
    const int64_t started = esp_timer_get_time();
    unsigned sentBytes = 0;
    esp_err_t result = ESP_OK;
    // 总量固定 64 KiB，任一分块失败立即退出，不用重试掩盖吞吐问题。
    for (int index = 0; index < 16 && result == ESP_OK; ++index)
    {
        result = httpd_resp_send_chunk(req, data.get(), 4096);
        if (result == ESP_OK)
        {
            sentBytes += 4096;
        }
    }
    const int sendErrno = errno;
    if (result == ESP_OK)
    {
        result = httpd_resp_send_chunk(req, nullptr, 0);
    }
    ESP_LOGI(TAG, "sendPerf() << result=%s errno=%d queued_bytes=%u send_ms=%lld camera=%s",
             esp_err_to_name(result), result == ESP_OK ? 0 : sendErrno, sentBytes,
             (long long)((esp_timer_get_time() - started) / 1000),
             esp_camera_sensor_get() == nullptr ? "off" : "on");
    return result;
}
#endif

// 输入：请求和来源要求；输出：是否允许；作用：HTTP 诊断可无来源，浏览器长连接必须同源设备。
bool CameraWeb::isMotorOrigin(httpd_req_t* req, bool required)
{
    char origin[96] = {};
    char host[64] = {};
    char allowed[80] = {};
    if (httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) != ESP_OK)
    {
        // 运动页面从同一设备的 80 端口加载，不接受缺少有效设备地址的请求。
        return false;
    }
    // 当前工程使用 IPv4/主机名和 HTTP；去掉运动端口，得到网页所在的 80 端口来源。
    const size_t length = std::strcspn(host, ":");
    std::snprintf(allowed, sizeof(allowed), "http://%.*s", static_cast<int>(length), host);
    if (httpd_req_get_hdr_value_len(req, "Origin") > 0)
    {
        if (httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof(origin)) != ESP_OK ||
            std::strcmp(origin, allowed) != 0)
        {
            // 仅允许本设备网页跨端口读取握手结果，不向任意网站开放运动通道。
            return false;
        }
        return true;
    }
    return !required;
}

// 输入：无；输出：无；作用：应用层把网络停车事件分别通知车轮与机械臂，不让模块互相依赖。
void CameraWeb::requestMotionStop()
{
    m_motor.requestStop();
    // 原子作废旧舵机指令，不在 Wi-Fi 回调中等待 I2C 或解除承重扭矩。
    m_arm.invalidate();
}

// 输入：运动 HTTP 请求；输出：响应结果；作用：保留低频诊断和备用停车，不用于网页周期续期。
esp_err_t CameraWeb::onMotor(httpd_req_t* req)
{
    auto* app = static_cast<CameraWeb*>(req->user_ctx);
    ESP_LOGI(TAG, "httpd_resp_set_hdr() >> name=Connection value=close");
    const esp_err_t header = httpd_resp_set_hdr(req, "Connection", "close");
    ESP_LOGI(TAG, "httpd_resp_set_hdr() << result=%s", esp_err_to_name(header));
    if (header != ESP_OK) return ESP_FAIL;
    if (!isMotorOrigin(req, false))
    {
        ESP_LOGW(TAG, "onMotor() << result=ORIGIN_REJECTED");
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Use the device webpage on port 80");
        // 来源拒绝后由 HTTP 服务立即关闭会话，不遗留未使用的连接位置。
        return ESP_FAIL;
    }
    // SDK 保留字符串地址直到响应发送完成；缓冲必须留在当前请求调用栈中。
    char origin[96] = {};
    if (httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof(origin)) == ESP_OK)
    {
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", origin);
        httpd_resp_set_hdr(req, "Vary", "Origin");
    }
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    const esp_err_t result = req->method == HTTP_GET ? app->sendMotor(req) : app->setMotor(req);
    if (result != ESP_OK) return result;
    // Content-Length 响应已发完；由服务队列回收会话，不能只发送 close 响应头。
    const int socketFd = httpd_req_to_sockfd(req);
    ESP_LOGI(TAG, "httpd_sess_trigger_close() >> socket=%d reason=short_response", socketFd);
    const esp_err_t closed = httpd_sess_trigger_close(req->handle, socketFd);
    if (closed == ESP_OK) ESP_LOGI(TAG, "httpd_sess_trigger_close() << result=ESP_OK");
    else ESP_LOGW(TAG, "httpd_sess_trigger_close() << result=%s", esp_err_to_name(closed));
    return closed == ESP_OK ? ESP_OK : ESP_FAIL;
}

// 输入：机械臂请求；输出：HTTP 结果；作用：限制来源后分派状态与单次控制命令。
esp_err_t CameraWeb::onArm(httpd_req_t* req)
{
    auto* app = static_cast<CameraWeb*>(req->user_ctx);
    ESP_LOGI(TAG, "httpd_resp_set_hdr() >> name=Connection value=close");
    const esp_err_t header = httpd_resp_set_hdr(req, "Connection", "close");
    ESP_LOGI(TAG, "httpd_resp_set_hdr() << result=%s", esp_err_to_name(header));
    if (header != ESP_OK) return ESP_FAIL;
    if (!isMotorOrigin(req, false))
    {
        // 与现有车轮接口一致，仅允许设备网页或无浏览器来源的本地调试工具。
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Use the device webpage on port 80");
        // 来源拒绝后由 HTTP 服务立即关闭会话，不遗留未使用的连接位置。
        return ESP_FAIL;
    }
    char origin[96] = {};
    if (httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof(origin)) == ESP_OK)
    {
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", origin);
        httpd_resp_set_hdr(req, "Vary", "Origin");
    }
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    const esp_err_t result = req->method == HTTP_GET ? app->sendArm(req) : app->setArm(req);
    if (result != ESP_OK) return result;
    // Content-Length 响应已发完；由服务队列回收会话，不能只发送 close 响应头。
    const int socketFd = httpd_req_to_sockfd(req);
    ESP_LOGI(TAG, "httpd_sess_trigger_close() >> socket=%d reason=short_response", socketFd);
    const esp_err_t closed = httpd_sess_trigger_close(req->handle, socketFd);
    if (closed == ESP_OK) ESP_LOGI(TAG, "httpd_sess_trigger_close() << result=ESP_OK");
    else ESP_LOGW(TAG, "httpd_sess_trigger_close() << result=%s", esp_err_to_name(closed));
    return closed == ESP_OK ? ESP_OK : ESP_FAIL;
}

// 输入：请求；输出：状态 JSON；作用：公布软件限位、代次与最后目标，释放后的角度也不是测量值。
esp_err_t CameraWeb::sendArm(httpd_req_t* req)
{
    char json[512];
    const int length = formatArm(json, sizeof(json), m_arm.getState(), 0, ESP_OK);
    if (length < 0 || length >= int(sizeof(json))) return ESP_FAIL;
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, length);
}

// 输入：action=set/release 表单；输出：命令结果；作用：完整校验后执行，越界不裁剪、不写硬件。
esp_err_t CameraWeb::setArm(httpd_req_t* req)
{
    ESP_LOGI(TAG, "setArm() >> bytes=%u", unsigned(req->content_len));
    char body[160] = {};
    esp_err_t result = ESP_OK;
    if (req->content_len == 0 || req->content_len >= sizeof(body)) result = ESP_ERR_INVALID_ARG;
    size_t offset = 0;
    while (result == ESP_OK && offset < req->content_len)
    {
        // 接收完整表单后才执行；中断不重放也不部分执行，超时由运动服务器限制。
        const int count = httpd_req_recv(req, body + offset, req->content_len - offset);
        if (count <= 0) result = ESP_ERR_INVALID_ARG;
        else offset += size_t(count);
    }
    char action[16] = {};
    if (result == ESP_OK && httpd_query_key_value(body, "action", action, sizeof(action)) != ESP_OK)
        result = ESP_ERR_INVALID_ARG;
    if (result == ESP_OK && !m_connected.load()) result = ESP_ERR_INVALID_STATE;
    if (result == ESP_OK && std::strcmp(action, "release") == 0)
    {
        // 明确释放操作无需代次，总能撤销旧指令；释放后承重机械臂可能下垂。
        result = m_arm.release();
    }
    else if (result == ESP_OK && (std::strcmp(action, "set") == 0 || std::strcmp(action, "increase") == 0 || std::strcmp(action, "decrease") == 0))
    {
        // 增减命令每次只推进 1～15 度，不在设备端排队或自动持续运动。
        const bool relative = std::strcmp(action, "set") != 0;
        const char* names[] = {"joint", relative ? "step" : "angle", "generation"};
        unsigned long values[3] = {};
        for (size_t index = 0; index < 3 && result == ESP_OK; ++index)
        {
            char field[16] = {};
            if (httpd_query_key_value(body, names[index], field, sizeof(field)) != ESP_OK)
                result = ESP_ERR_INVALID_ARG;
            const size_t size = std::strlen(field);
            // 只接受十进制数字，拒绝符号、空白、尾随字符及 uint32 溢出。
            if (size == 0) result = ESP_ERR_INVALID_ARG;
            for (size_t digit = 0; digit < size; ++digit)
            {
                if (field[digit] < '0' || field[digit] > '9') result = ESP_ERR_INVALID_ARG;
            }
            errno = 0;
            char* end = nullptr;
            values[index] = std::strtoul(field, &end, 10);
            if (errno != 0 || end == field || *end != '\0' || values[index] > UINT32_MAX) result = ESP_ERR_INVALID_ARG;
        }
        if (result == ESP_OK && (values[0] > 1 || values[1] > 180)) result = ESP_ERR_INVALID_ARG;
        if (result == ESP_OK && relative && (values[1] < 1 || values[1] > 15)) result = ESP_ERR_INVALID_ARG;
        if (result == ESP_OK)
        {
            // 方向由动作名确定，避免表单正负号与 URL 编码产生歧义。
            const int angle = std::strcmp(action, "decrease") == 0 ? -int(values[1]) : int(values[1]);
            result = m_arm.setTarget(int(values[0]), angle, uint32_t(values[2]), relative);
        }
    }
    else if (result == ESP_OK) result = ESP_ERR_INVALID_ARG;
    if (result == ESP_OK)
    {
        ESP_LOGI(TAG, "setArm() << result=ESP_OK");
        return sendArm(req);
    }
    ESP_LOGW(TAG, "setArm() << result=%s", esp_err_to_name(result));
    if (result == ESP_ERR_INVALID_ARG)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid fields or angle outside configured software limits");
    httpd_resp_set_status(req, result == ESP_ERR_INVALID_STATE ? "409 Conflict" : "503 Service Unavailable");
    return httpd_resp_sendstr(req, "Arm unavailable, stale generation or driver failure; read /arm before another command");
}

// 输入：升级请求；输出：准入结果；作用：握手前保留唯一控制连接，不自动启动电机。
esp_err_t CameraWeb::openMotorSocket(httpd_req_t* req)
{
    auto* app = static_cast<CameraWeb*>(req->user_ctx);
    ESP_LOGI(TAG, "openMotorSocket() >>");
    if (!isMotorOrigin(req, true))
    {
        ESP_LOGW(TAG, "openMotorSocket() << result=ORIGIN_REJECTED");
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Use the device webpage on port 80");
        return ESP_FAIL;
    }
    if (app->m_motorFd >= 0)
    {
        // 控制页面独占连接，额外页面不能抢占正在运动的操作者。
        ESP_LOGW(TAG, "openMotorSocket() << result=CONTROLLER_BUSY");
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "Another control page is connected");
        return ESP_FAIL;
    }
    app->m_motorFd = httpd_req_to_sockfd(req);
    app->m_hasMotionSocket.store(true);
    app->m_motorRxUs = esp_timer_get_time();
    app->m_motorClosing = false;
    app->m_motorSeq = 0;
    app->m_reportUs = 0;
    app->m_reportGen = 0;
    app->m_isReportedMoving = false;
    ESP_LOGI(TAG, "openMotorSocket() << result=ESP_OK socket=%d", app->m_motorFd);
    return ESP_OK;
}

// 输入：连接、序号和业务结果；输出：发送结果；作用：单次非阻塞写入完整 WS 文本帧。
esp_err_t CameraWeb::reportMotor(int socketFd, uint32_t sequence, esp_err_t result)
{
    const MotorState state = m_motor.getState();
    char text[126];
    const int length = std::snprintf(text, sizeof(text),
        "{\"e\":%d,\"r\":%d,\"m\":%d,\"f\":%d,\"g\":%lu,\"s\":%lu,\"c\":%d}",
        int(state.enabled), int(state.ready), int(state.moving), state.fault,
        static_cast<unsigned long>(state.generation), static_cast<unsigned long>(sequence), result);
    if (length <= 0 || length >= static_cast<int>(sizeof(text)))
    {
        // 短文本帧不超过 125 字节，拒绝截断数据，不能破坏后续帧边界。
        ESP_LOGE(TAG, "reportMotor() << result=STATE_TOO_LARGE");
        return ESP_FAIL;
    }
    const esp_err_t sent = sendSocketText(socketFd, text, length);
    if (sent == ESP_ERR_TIMEOUT) return ESP_OK;
    if (sent != ESP_OK) return sent;
    m_reportUs = esp_timer_get_time();
    m_reportGen = state.generation;
    m_isReportedMoving = state.moving;
    return ESP_OK;
}

// 输入：状态及回报编号；输出：JSON 长度；作用：同一快照区分输出角度和待完成目标。
int CameraWeb::formatArm(char* text, size_t capacity, const ArmState& state, uint32_t sequence, esp_err_t result)
{
    return std::snprintf(text, capacity,
        "{\"type\":\"arm\",\"s\":%lu,\"c\":%d,\"enabled\":%s,\"ready\":%s,\"fault\":%d,\"generation\":%lu,"
        "\"angles\":[%d,%d],\"targets\":[%d,%d],\"holding\":[%s,%s],\"slewing\":[%s,%s],\"minimum\":[%d,%d],\"maximum\":[%d,%d]}",
        static_cast<unsigned long>(sequence), result, state.enabled ? "true" : "false", state.ready ? "true" : "false", state.fault,
        static_cast<unsigned long>(state.generation), state.angles[0], state.angles[1], state.targets[0], state.targets[1],
        state.holding[0] ? "true" : "false", state.holding[1] ? "true" : "false", state.slewing[0] ? "true" : "false", state.slewing[1] ? "true" : "false",
        state.minimum[0], state.minimum[1], state.maximum[0], state.maximum[1]);
}

// 输入：连接、序号及机械臂结果；输出：发送结果；作用：回报与车轮协议隔离，不等待角度执行完成。
esp_err_t CameraWeb::reportArm(int socketFd, uint32_t sequence, esp_err_t result)
{
    char text[512];
    const int length = formatArm(text, sizeof(text), m_arm.getState(), sequence, result);
    if (length <= 0 || length >= int(sizeof(text))) return ESP_FAIL;
    const esp_err_t sent = sendSocketText(socketFd, text, length);
    // 拥塞时不排队；客户端停止续租后本地租约将自动终止推进。
    return sent == ESP_ERR_TIMEOUT ? ESP_OK : sent;
}

// 输入：完整文本；输出：发送结果；作用：单包发送 WS 帧，支持 125 字节以上的机械臂快照。
esp_err_t CameraWeb::sendSocketText(int socketFd, const char* text, size_t length)
{
    if (length > 512) return ESP_ERR_INVALID_SIZE;
    std::array<uint8_t, 516> packet{};
    packet[0] = 0x81;
    const size_t header = length <= 125 ? 2 : 4;
    if (header == 2) packet[1] = uint8_t(length);
    else
    {
        // 扩展长度使用网络大端；服务端帧不需要客户端掩码。
        packet[1] = 126;
        packet[2] = uint8_t(length >> 8);
        packet[3] = uint8_t(length & 0xff);
    }
    std::memcpy(packet.data() + header, text, length);
    ESP_LOGD(TAG, "send() >> socket=%d bytes=%u flags=MSG_DONTWAIT", socketFd, unsigned(header + length));
    const int count = send(socketFd, packet.data(), header + length, MSG_DONTWAIT);
    const int sendErr = count < 0 ? errno : 0;
    ESP_LOGD(TAG, "send() << result=%d errno=%d", count, sendErr);
    if (count < 0 && (sendErr == EAGAIN || sendErr == EWOULDBLOCK)) return ESP_ERR_TIMEOUT;
    if (count != int(header + length))
    {
        // 半帧不能续发另一帧；失败交给服务关闭连接并触发失联停止。
        ESP_LOGW(TAG, "sendSocketText() << result=SEND_FAILED bytes=%d errno=%d", count, sendErr);
        return ESP_FAIL;
    }
    return ESP_OK;
}

// 输入：12 字节二进制指令；输出：接收结果；作用：顺序验证后执行，正常续期每 500 ms 合并回报。
esp_err_t CameraWeb::onMotorSocket(httpd_req_t* req)
{
    auto* app = static_cast<CameraWeb*>(req->user_ctx);
    const int socketFd = httpd_req_to_sockfd(req);
    std::array<uint8_t, 12> data{};
    httpd_ws_frame_t frame = {};
    frame.payload = data.data();
    ESP_LOGD(TAG, "httpd_ws_recv_frame() >> socket=%d max_bytes=12", socketFd);
    const esp_err_t received = httpd_ws_recv_frame(req, &frame, data.size());
    ESP_LOGD(TAG, "httpd_ws_recv_frame() << result=%s bytes=%u", esp_err_to_name(received), unsigned(frame.len));
    if (received != ESP_OK || frame.type != HTTPD_WS_TYPE_BINARY || frame.len != data.size() ||
        socketFd != app->m_motorFd || app->m_motorClosing || data[0] > 8 ||
        (data[0] <= 2 && (data[1] > 200 || data[2] > 200 || data[3] > 200)) ||
        (data[0] >= 3 && (data[1] > 1 || data[3] != 0 ||
          ((data[0] == 3 || data[0] == 4) && (data[2] < 1 || data[2] > 15)) ||
          (data[0] == 6 && data[2] > 180) ||
          ((data[0] == 5 || data[0] == 7 || data[0] == 8) && (data[1] != 0 || data[2] != 0)))))
    {
        // 畸形帧或非当前控制连接都不能执行运动；返回失败由关闭回调投递停车。
        ESP_LOGW(TAG, "onMotorSocket() << result=INVALID_FRAME socket=%d", socketFd);
        return ESP_FAIL;
    }
    uint32_t generation = 0;
    uint32_t sequence = 0;
    for (size_t index = 0; index < 4; ++index)
    {
        // 明确按网络大端解码，不依赖 ESP32 和浏览器端的本机字节序。
        generation = (generation << 8) | data[4 + index];
        sequence = (sequence << 8) | data[8 + index];
    }
    if (sequence == 0 || sequence <= app->m_motorSeq)
    {
        ESP_LOGW(TAG, "onMotorSocket() << result=STALE_SEQUENCE");
        return ESP_FAIL;
    }
    // 只有通过长度、类型、连接身份和序号校验的指令才续期连接租约。
    app->m_motorRxUs = esp_timer_get_time();
    app->m_motorSeq = sequence;
    esp_err_t result = ESP_OK;
    if (data[0] >= 3)
    {
        // 机械臂与车轮共用传输连接，但状态代次和业务结果分开处理。
        if (data[0] == 5) result = app->m_arm.hold();
        else if (data[0] == 7) result = app->m_arm.release();
        else if (data[0] != 8)
        {
            if (!app->m_connected.load()) result = ESP_ERR_INVALID_STATE;
            else result = app->m_arm.setTarget(data[1], data[0] == 4 ? -int(data[2]) : int(data[2]), generation, data[0] != 6);
        }
        // 机械臂拒绝命令只回报机械臂错误，不因此切断仍正常的车轮连接。
        return app->reportArm(socketFd, sequence, result);
    }
    if (data[0] == 2)
    {
        // 停车不要求代次，始终优先撤销当前输出；回报用于确认下一次按住的代次。
        result = app->m_motor.stop("websocket stop");
    }
    else if (data[0] == 1)
    {
        if (!app->m_connected.load())
        {
            app->requestMotionStop();
            result = ESP_ERR_INVALID_STATE;
        }
        else
        {
            result = app->m_motor.move(int(data[1]) - 100, int(data[2]) - 100,
                                       int(data[3]) - 100, generation);
        }
    }
    // 操作 0 只读取快照，不写外设，也不延长有效运动期限。
    const MotorState state = app->m_motor.getState();
    if (data[0] != 1 || result != ESP_OK || state.generation != app->m_reportGen ||
        state.moving != app->m_isReportedMoving ||
        esp_timer_get_time() - app->m_reportUs >= 500000)
    {
        // 查询、停车、故障立即回报；相同运动心跳不逐条发送响应。
        const esp_err_t sent = app->reportMotor(socketFd, sequence, result);
        // 业务拒绝已由状态回报，停止动作与传输连接故障分别处理。
        // 保持连接以便重新查询、停车或控制机械臂；只有回报发送失败才关闭。
        return sent;
    }
    return ESP_OK;
}

// 输入：关闭的连接；输出：无；作用：仅控制连接断开才停车，不让诊断 GET 的关闭干扰车辆。
void CameraWeb::closeMotorSocket(httpd_handle_t server, int socketFd)
{
    auto& app = CameraWeb::instance();
    if (socketFd == app.m_motorFd)
    {
        app.m_hasMotionSocket.store(false);
        app.m_motorFd = -1;
        app.m_motorRxUs = 0;
        app.m_motorClosing = false;
        app.m_motorSeq = 0;
        app.requestMotionStop();
        ESP_LOGW(TAG, "closeMotorSocket() >> controller_disconnected socket=%d", socketFd);
    }
    ESP_LOGD(TAG, "close() >> socket=%d", socketFd);
    const int result = close(socketFd);
    ESP_LOGD(TAG, "close() << result=%d", result);
}

// 输入：实例；输出：无；作用：定时投递至 HTTP 任务，同一时间最多保留一次巡检。
void CameraWeb::queueMotionCheck(void* arg)
{
    auto* app = static_cast<CameraWeb*>(arg);
    // 没有控制连接时无需给运动 HTTP 任务投递空巡检；有连接时仍保持原 500 ms 检查周期。
    if (!app->m_hasMotionSocket.load() || app->m_checkQueued.exchange(true)) return;
    ESP_LOGI(TAG, "httpd_queue_work() >> service=motion work=checkMotionSocket");
    const esp_err_t result = httpd_queue_work(app->m_motion, checkMotionSocket, app);
    if (result == ESP_OK) ESP_LOGI(TAG, "httpd_queue_work() << result=ESP_OK");
    else
    {
        // 队列临时繁忙时等待下一次 500 ms 巡检，不增加积压工作。
        app->m_checkQueued.store(false);
        ESP_LOGW(TAG, "httpd_queue_work() << result=%s", esp_err_to_name(result));
    }
}

// 输入：实例；输出：无；作用：仅在 HTTP 服务任务中判定过期并回收旧 WebSocket。
void CameraWeb::checkMotionSocket(void* arg)
{
    auto* app = static_cast<CameraWeb*>(arg);
    if (app->m_motorFd >= 0 && !app->m_motorClosing &&
        esp_timer_get_time() - app->m_motorRxUs >= 3000000)
    {
        // 网页空闲时每秒查询，三秒没有有效帧即视为失联；正常操作者不会被新页面抢占。
        app->requestMotionStop();
        ESP_LOGI(TAG, "httpd_sess_trigger_close() >> socket=%d reason=controller_timeout", app->m_motorFd);
        const esp_err_t result = httpd_sess_trigger_close(app->m_motion, app->m_motorFd);
        if (result == ESP_OK)
        {
            // 保留描述符直到真实关闭回调，防止提前复用造成旧回调误伤新连接。
            app->m_motorClosing = true;
            ESP_LOGI(TAG, "httpd_sess_trigger_close() << result=ESP_OK");
        }
        else ESP_LOGW(TAG, "httpd_sess_trigger_close() << result=%s", esp_err_to_name(result));
    }
    app->m_checkQueued.store(false);
}

// 输入：无；输出：无；作用：采样系统状态并计算相邻窗口差值，视频阻塞时仍可发布 CPU 状态。
void CameraWeb::updateRuntime()
{
    ESP_LOGI(TAG, "updateRuntime() >>");
    std::array<double, 2> busyPct{};
    bool isCpuValid = false;
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS && CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER && \
    !CONFIG_FREERTOS_SMP && CONFIG_FREERTOS_NUMBER_OF_CORES == 2
    std::array<uint32_t, 2> idleUs{};
    // 每核仅取空闲累计量，不分配任务列表或长时间暂停调度。
    for (int core = 0; core < 2; ++core)
    {
        ESP_LOGI(TAG, "ulTaskGetIdleRunTimeCounterForCore() >> core=%d", core);
        idleUs[core] = static_cast<uint32_t>(ulTaskGetIdleRunTimeCounterForCore(core));
        ESP_LOGI(TAG, "ulTaskGetIdleRunTimeCounterForCore() << core=%d idle_us=%lu",
                 core, static_cast<unsigned long>(idleUs[core]));
    }
    const int64_t cpuNowUs = esp_timer_get_time();
    const int64_t cpuElapsedUs = cpuNowUs - m_cpuAtUs;
    // 首轮、过短窗口或间隔超过一次计数回绕时不伪造负载，下一轮重新建立有效差值。
    if (m_cpuAtUs > 0 && cpuElapsedUs >= 1000000 && cpuElapsedUs < UINT32_MAX)
    {
        isCpuValid = true;
        for (size_t core = 0; core < idleUs.size(); ++core)
        {
            const uint32_t deltaUs = idleUs[core] - m_idleLast[core];
            // 计数在任务切换时结算，允许两个 tick 的边界误差；异常样本整轮显示不可用。
            if (deltaUs > static_cast<uint64_t>(cpuElapsedUs + 2 * portTICK_PERIOD_MS * 1000))
            {
                isCpuValid = false;
            }
            busyPct[core] = std::clamp(100.0 - deltaUs * 100.0 / cpuElapsedUs, 0.0, 100.0);
        }
    }
    m_idleLast = idleUs;
    m_cpuAtUs = cpuNowUs;
#endif
    ESP_LOGI(TAG, "esp_rom_get_cpu_ticks_per_us() >>");
    const unsigned cpuMhz = esp_rom_get_cpu_ticks_per_us();
    ESP_LOGI(TAG, "esp_rom_get_cpu_ticks_per_us() << mhz=%u", cpuMhz);
    ESP_LOGI(TAG, "heap_caps_get_free_size() >> caps=INTERNAL|8BIT");
    const size_t internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "heap_caps_get_free_size() << bytes=%u", unsigned(internalFree));
    ESP_LOGI(TAG, "heap_caps_get_free_size() >> caps=SPIRAM|8BIT");
    const size_t psramFree = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "heap_caps_get_free_size() << bytes=%u", unsigned(psramFree));
    bool isRssiValid = false;
    int rssiDbm = 0;
    int apClients = -1;
    const char* wifiMode = "apsta";
    const char* rssiSource = "ap";
    wifi_sta_list_t clients{};
    ESP_LOGI(TAG, "esp_wifi_ap_get_sta_list() >>");
    const esp_err_t listResult = esp_wifi_ap_get_sta_list(&clients);
    ESP_LOGI(TAG, "esp_wifi_ap_get_sta_list() << result=%s clients=%d",
             esp_err_to_name(listResult), listResult == ESP_OK ? clients.num : -1);
    if (listResult == ESP_OK)
    {
        // STA 尚未取得地址时，显示热点客户端中最弱的 RSSI；没有客户端则显示未知。
        apClients = clients.num;
        for (int index = 0; index < clients.num; ++index)
        {
            if (!isRssiValid || clients.sta[index].rssi < rssiDbm) rssiDbm = clients.sta[index].rssi;
            isRssiValid = true;
        }
    }
    if (m_staReady.load())
    {
        wifi_ap_record_t access{};
        ESP_LOGI(TAG, "esp_wifi_sta_get_ap_info() >>");
        const esp_err_t wifiResult = esp_wifi_sta_get_ap_info(&access);
        ESP_LOGI(TAG, "esp_wifi_sta_get_ap_info() << result=%s rssi_dbm=%d",
                 esp_err_to_name(wifiResult), wifiResult == ESP_OK ? access.rssi : 0);
        if (wifiResult == ESP_OK)
        {
            // 明确标注信号来源，路由器信号不能混作热点客户端信号。
            rssiSource = "sta";
            rssiDbm = access.rssi;
            isRssiValid = true;
        }
    }
    double bytesPerSec = 0;
    double captureMs = 0;
    double sendMs = 0;
    uint64_t frames = 0;
    int64_t elapsedUs = 0;
    {
        // SDK 采样完成后才短暂取快照锁；视频只在完整一帧提交后更新四个累计量。
        std::lock_guard<std::mutex> lock(m_statusMutex);
        const int64_t videoNowUs = esp_timer_get_time();
        elapsedUs = m_runtimeAtUs == 0 ? 0 : videoNowUs - m_runtimeAtUs;
        const std::array<uint64_t, 4> totals{m_videoBytes, m_videoFrames, m_captureUs, m_sendUs};
        if (elapsedUs > 0)
        {
            // 窗口包含没有发送的时间，长时间没有完整帧时速率归零，不沿用旧的高吞吐值。
            frames = totals[1] - m_videoLast[1];
            bytesPerSec = (totals[0] - m_videoLast[0]) * 1000000.0 / elapsedUs;
            if (frames > 0)
            {
                captureMs = (totals[2] - m_videoLast[2]) / (1000.0 * frames);
                sendMs = (totals[3] - m_videoLast[3]) / (1000.0 * frames);
            }
        }
        m_videoLast = totals;
        m_runtimeAtUs = videoNowUs;
        std::snprintf(m_runtimeJson.data(), m_runtimeJson.size(),
            "{\"cpu_valid\":%s,\"cpu_mhz\":%u,\"cpu0_pct\":%.1f,\"cpu1_pct\":%.1f,"
            "\"internal_free\":%u,\"psram_free\":%u,\"wifi_mode\":\"%s\",\"rssi_source\":\"%s\",\"rssi_valid\":%s,"
            "\"rssi_dbm\":%d,\"ap_clients\":%d,\"video_Bps\":%.1f,\"frames\":%llu,"
            "\"capture_ms\":%.3f,\"send_ms\":%.3f,\"window_ms\":%lld,\"jpeg_errors\":%u,"
            "\"frame_timeouts\":%u,\"camera_resets\":%u}",
            isCpuValid ? "true" : "false", cpuMhz, busyPct[0], busyPct[1],
            unsigned(internalFree), unsigned(psramFree), wifiMode, rssiSource, isRssiValid ? "true" : "false",
            rssiDbm, apClients, bytesPerSec, static_cast<unsigned long long>(frames),
            captureMs, sendMs, static_cast<long long>(elapsedUs / 1000), unsigned(m_jpegErrors.load()),
            unsigned(m_frameTimeouts.load()), unsigned(m_cameraResets.load()));
    }
#if CONFIG_CAMERA_OLED_ENABLED
    // 无网页访问、无视频时也主动采样曝光；相机忙则沿用快照，不等待取帧。
    {
        std::unique_lock<std::mutex> lock(m_mutex, std::try_to_lock);
        if (lock.owns_lock() && !m_streaming && esp_timer_get_time() - m_expReadUs >= 2000000)
        {
            cacheStatus();
        }
    }
    updateOled(isCpuValid, busyPct);
#endif
    ESP_LOGI(TAG, "updateRuntime() << cpu_valid=%d cpu0_pct=%.1f cpu1_pct=%.1f video_Bps=%.1f frames=%llu window_ms=%lld",
             int(isCpuValid), busyPct[0], busyPct[1], bytesPerSec,
             static_cast<unsigned long long>(frames), static_cast<long long>(elapsedUs / 1000));
}

// 输入：OV3660；输出：读数是否有效；作用：读取实际快门、HTS/VTS 与 PLL，估算积分时间。
bool CameraWeb::readExposure(sensor_t* sensor)
{
    ESP_LOGI(TAG, "readExposure() >> sensor=OV3660");
    m_expReadUs = esp_timer_get_time();
    m_isExposureValid = false;
    if (sensor == nullptr || sensor->id.PID != OV3660_PID)
    {
        // 型号检查先于任何寄存器访问，禁止沿用旧 OV2640 的 bank 寄存器。
        ESP_LOGW(TAG, "readExposure() << result=UNSUPPORTED_SENSOR");
        return false;
    }
    const int regs[] = {0x300a, 0x303a, 0x303b, 0x303c, 0x303d, 0x380c,
                        0x380e, 0x3503, 0x3500, 0x3a00, 0x3103, 0x3500};
    const int masks[] = {0xffff, 0xff, 0xff, 0xff, 0xff, 0xffff,
                         0xffff, 0xff, 0xfffff, 0xff, 0xff, 0xfffff};
    int values[std::size(regs)]{};
    // 快门跨三个字节；末尾复读一致才发布，自动曝光恰逢更新时等待下一次采样。
    for (size_t index = 0; index < std::size(regs); ++index)
    {
        ESP_LOGI(TAG, "sensor get_reg() >> reg=0x%04x mask=0x%x", regs[index], masks[index]);
        values[index] = sensor->get_reg(sensor, regs[index], masks[index]);
        ESP_LOGI(TAG, "sensor get_reg() << reg=0x%04x value=%d", regs[index], values[index]);
        if (values[index] < 0 || values[index] > masks[index])
        {
            ESP_LOGW(TAG, "readExposure() << result=READ_FAILED reg=0x%04x", regs[index]);
            return false;
        }
    }
    if (values[0] != OV3660_PID || values[5] == 0 || values[6] <= 4 ||
        values[8] != values[11] || values[8] == 0 || (values[10] & 0x02) == 0 ||
        sensor->pixformat != PIXFORMAT_JPEG || sensor->xclk_freq_hz <= 0)
    {
        // 全零 SCCB、非 PLL 时钟源或跨帧变化都不能显示为有效曝光时间。
        ESP_LOGW(TAG, "readExposure() << result=INVALID_OR_CHANGING_REGISTERS");
        return false;
    }
    const int pre2[] = {2, 3, 4, 6};
    const int seld2[] = {2, 2, 4, 5};
    const int mult = values[2] & 0x1f;
    const int sysDiv = std::max(1, values[3] & 0x0f);
    const int root = (values[4] & 0x40) != 0 ? 2 : 1;
    // 采用组件 ov3660.c::calc_sysclk 的同一时钟模型；DVP/PCLK 分频不等于积分时钟。
    const double pllHz = (values[1] & 0x80) != 0 ? double(sensor->xclk_freq_hz) :
        double(sensor->xclk_freq_hz) * mult * root * 4.0 /
        pre2[(values[4] >> 4) & 3] / sysDiv / seld2[values[4] & 3];
    const double sysHz = pllHz / 4.0;
    if (!std::isfinite(sysHz) || sysHz <= 0)
    {
        ESP_LOGW(TAG, "readExposure() << result=INVALID_PLL_CLOCK");
        return false;
    }
    m_lineUs = values[5] * 1000000.0 / sysHz;
    m_expLines = values[8] / 16.0;
    m_isAutoExposure = (values[7] & 1) == 0;
    m_maxShutter = std::min(65531, values[6] - 4);
    m_expMaxLines = m_maxShutter;
    if (m_isAutoExposure || m_autoVts == 0)
    {
        // 驱动切换分辨率后重新记录基础 VTS；手动冻结扩展的 VTS 不覆盖自动模式原值。
        m_autoVts = values[6];
    }
    if (m_isAutoExposure)
    {
        m_autoFrames = values[9] & 0x04;
    }
    sensor->status.aec = m_isAutoExposure ? 1 : 0;
    m_isExposureValid = true;
    ESP_LOGI(TAG, "readExposure() << result=OK auto=%d shutter=%.4f hts=%d vts=%d sys_hz=%.0f line_us=%.3f exposure_ms=%.3f",
             int(m_isAutoExposure), m_expLines, values[5], values[6], sysHz, m_lineUs,
             m_expLines * m_lineUs / 1000.0);
    return true;
}

// 输入：传感器和整数曝光行数；输出：是否成功；作用：由 OV3660 驱动写快门并回读验证。
bool CameraWeb::setExposure(sensor_t* sensor, int lines)
{
    ESP_LOGI(TAG, "setExposure() >> sensor=OV3660 lines=%d", lines);
    if (!m_isExposureValid || m_isAutoExposure || lines < 1 || lines > m_expMaxLines)
    {
        // 只允许 VTS 减四行余量内的手动值，拒绝驱动隐式夹紧后冒充目标已生效。
        ESP_LOGW(TAG, "setExposure() << result=INVALID_STATE_OR_RANGE");
        return false;
    }
    ESP_LOGI(TAG, "sensor set_aec_value() >> lines=%d", lines);
    const int result = sensor->set_aec_value(sensor, lines);
    ESP_LOGI(TAG, "sensor set_aec_value() << result=%d", result);
    const bool hasReadback = readExposure(sensor);
    if (result != 0 || !hasReadback || std::abs(m_expLines - lines) >= 0.0625)
    {
        // 部分写入也回读真实值，上层会返回失败并刷新当前状态。
        ESP_LOGW(TAG, "setExposure() << result=WRITE_OR_READBACK_FAILED");
        return false;
    }
    ESP_LOGI(TAG, "setExposure() << result=OK exposure_ms=%.3f", m_expLines * m_lineUs / 1000.0);
    return true;
}

// 输入：传感器和目标自动模式；输出：是否成功；作用：冻结真实快门，保留 AGC 并恢复夜间模式。
bool CameraWeb::setAutoExposure(sensor_t* sensor, bool enabled)
{
    ESP_LOGI(TAG, "setAutoExposure() >> enabled=%d", int(enabled));
    if (!readExposure(sensor))
    {
        ESP_LOGW(TAG, "setAutoExposure() << result=READ_FAILED");
        return false;
    }
    if (m_isAutoExposure == enabled)
    {
        // 重复请求是幂等的，不重置手动范围或覆盖保存的自动 VTS。
        ESP_LOGI(TAG, "setAutoExposure() << result=UNCHANGED");
        return true;
    }
    int result = 0;
    if (!enabled)
    {
        // 先关闭 AEC，再读取被冻结的快门，避免自动更新后用旧采样值覆盖当前曝光。
        ESP_LOGI(TAG, "sensor set_exposure_ctrl() >> enabled=0");
        result = sensor->set_exposure_ctrl(sensor, 0);
        ESP_LOGI(TAG, "sensor set_exposure_ctrl() << result=%d", result);
        if (result == 0 && !readExposure(sensor)) result = -1;
    }
    if (result == 0)
    {
        // 长自动曝光可能超过基础 VTS；冻结时扩展帧长以保留当前积分时间，开启时还原。
        const int frozenVts = std::clamp(std::max(m_autoVts, int(std::ceil(m_expLines)) + 4), 5, 65535);
        const int regs[] = {0x380e, 0x3a00};
        const int masks[] = {0xffff, 0x04};
        const int values[] = {enabled ? m_autoVts : frozenVts, enabled ? m_autoFrames : 0};
        for (size_t index = 0; index < std::size(regs) && result == 0; ++index)
        {
            ESP_LOGI(TAG, "sensor set_reg() >> reg=0x%04x mask=0x%x value=%d",
                     regs[index], masks[index], values[index]);
            result = sensor->set_reg(sensor, regs[index], masks[index], values[index]);
            ESP_LOGI(TAG, "sensor set_reg() << reg=0x%04x result=%d", regs[index], result);
        }
    }
    if (enabled && result == 0)
    {
        // 自动模式最后开启，不修改 0x3503 的 AGC 位，不写入 OV2640 DSP 寄存器。
        ESP_LOGI(TAG, "sensor set_exposure_ctrl() >> enabled=1");
        result = sensor->set_exposure_ctrl(sensor, 1);
        ESP_LOGI(TAG, "sensor set_exposure_ctrl() << result=%d", result);
    }
    const bool hasReadback = readExposure(sensor);
    if (result != 0 || !hasReadback || m_isAutoExposure != enabled)
    {
        ESP_LOGW(TAG, "setAutoExposure() << result=WRITE_OR_READBACK_FAILED");
        return false;
    }
    ESP_LOGI(TAG, "setAutoExposure() << result=OK enabled=%d", int(enabled));
    return true;
}

// 输入：无；输出：无；作用：相机初始化、设置变更和帧率更新时发布小型状态快照。
void CameraWeb::cacheStatus()
{
    // 调用方已持有相机锁，或仍在单线程初始化阶段；这里只短暂持有快照锁。
    ESP_LOGI(TAG, "esp_camera_sensor_get() >> status_snapshot");
    sensor_t* sensor = esp_camera_sensor_get();
    ESP_LOGI(TAG, "esp_camera_sensor_get() << ready=%d", int(sensor != nullptr));
    if (sensor == nullptr)
    {
        // 测量模式关闭相机时清空快照，不能把历史数据描述为当前有效参数。
        std::lock_guard<std::mutex> lock(m_statusMutex);
        m_statusJson[0] = '\0';
#if CONFIG_CAMERA_OLED_ENABLED
        m_isOledExpValid = false;
#endif
        return;
    }
    if (esp_timer_get_time() - m_expReadUs >= 2000000)
    {
        // 复用现有状态/帧率发布节拍；HTTP 网络发送和快照锁内不访问 SCCB。
        readExposure(sensor);
    }
    const auto& state = sensor->status;
    std::lock_guard<std::mutex> lock(m_statusMutex);
#if CONFIG_CAMERA_OLED_ENABLED
    // 与网页发布同一份曝光值；失败和过期样本不能冒充当前读数。
    m_oledExpMs = std::llround(m_expLines * m_lineUs) / 1000.0;
    m_oledExpAtUs = m_expReadUs;
    m_isOledExpValid = m_isExposureValid;
    m_isOledAuto = m_isAutoExposure;
#endif
    std::snprintf(m_statusJson.data(), m_statusJson.size(),
        "{\"size\":%d,\"quality\":%d,\"max_fps\":%d,\"actual_fps\":%.2f,"
        "\"brightness\":%d,\"contrast\":%d,\"saturation\":%d,"
        "\"hmirror\":%d,\"vflip\":%d,\"streaming\":%s,"
        "\"auto_exposure\":%s,\"exposure_valid\":%s,\"exposure_us\":%lld,"
        "\"exposure_min_us\":%lld,\"exposure_max_us\":%lld,\"line_us\":%.3f",
        m_size, state.quality, m_maxFps, m_actualFps,
        state.brightness, state.contrast, state.saturation, state.hmirror, state.vflip,
        m_streaming ? "true" : "false", m_isAutoExposure ? "true" : "false",
        m_isExposureValid ? "true" : "false",
        std::llround(m_expLines * m_lineUs),
        static_cast<long long>(std::ceil(m_lineUs)),
        static_cast<long long>(std::floor(m_expMaxLines * m_lineUs)), m_lineUs);
}

esp_err_t CameraWeb::sendStatus(httpd_req_t* req)
{
    {
        // 空闲时也刷新自动曝光；视频取帧占锁时立即使用快照，不让状态请求等待取帧。
        std::unique_lock<std::mutex> lock(m_mutex, std::try_to_lock);
        if (lock.owns_lock() && !m_streaming && esp_timer_get_time() - m_expReadUs >= 2000000)
        {
            cacheStatus();
        }
    }
    char json[2048];
    std::array<char, 512> cached;
    std::array<char, 512> runtime;
    bool isVideoActive;
    int64_t runtimeAgeMs;
    // 只读已发布快照；相机取帧即使等待四秒，也不会阻塞本次状态读取。
    const MotorState motor = m_motor.getState();
    {
        std::lock_guard<std::mutex> lock(m_statusMutex);
        cached = m_statusJson;
        runtime = m_runtimeJson;
        isVideoActive = m_isVideoActive;
        runtimeAgeMs = m_runtimeAtUs == 0 ? -1 : (esp_timer_get_time() - m_runtimeAtUs) / 1000;
    }
    if (cached[0] == '\0')
    {
        // 即使相机不可用，错误响应也在释放快照锁后发送，避免慢客户端阻塞发布。
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera not ready");
    }
    // 使用原子快照提供两个接口的访问状态；不暴露任何 Wi-Fi 密码。
    const bool apReady = m_apReady.load();
    const bool staReady = m_staReady.load();
    const esp_ip4_addr_t staIp = {staReady ? m_staIp.load() : 0};
    std::snprintf(json, sizeof(json),
        "%s,\"connected\":%s,\"network\":{\"mode\":\"apsta\",\"ap_ready\":%s,"
        "\"sta_connected\":%s,\"ap_ip\":\"192.168.4.1\",\"sta_ip\":\"" IPSTR "\"},\"runtime\":%s,\"runtime_age_ms\":%lld,\"video_active\":%s,\"led_ready\":%s,\"led_on\":%s,\"led_brightness\":%d,\"led_fault\":%d,"
        "\"motor\":{\"enabled\":%s,\"ready\":%s,\"moving\":%s,\"fault\":%d,"
        "\"generation\":%lu,\"forward\":%d,\"right\":%d,\"turn\":%d,\"wheels\":[%d,%d,%d,%d]}}",
        cached.data(), (apReady || staReady) ? "true" : "false",
        apReady ? "true" : "false", staReady ? "true" : "false", IP2STR(&staIp),
        runtime[0] == '\0' ? "{}" : runtime.data(), static_cast<long long>(runtimeAgeMs),
        isVideoActive ? "true" : "false",
        m_ledReady.load() ? "true" : "false", m_ledOn.load() ? "true" : "false", m_ledBrightness.load(), int(m_ledFault.load()),
        motor.enabled ? "true" : "false", motor.ready ? "true" : "false", motor.moving ? "true" : "false",
        motor.fault, static_cast<unsigned long>(motor.generation), motor.forward, motor.right, motor.turn,
        motor.wheels[0], motor.wheels[1], motor.wheels[2], motor.wheels[3]);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

// 输入：请求；输出：独立电机 JSON；作用：避免运动续租依赖相机状态读取。
esp_err_t CameraWeb::sendMotor(httpd_req_t* req)
{
    const MotorState state = m_motor.getState();
    char json[384];
    std::snprintf(json, sizeof(json),
        "{\"enabled\":%s,\"ready\":%s,\"moving\":%s,\"fault\":%d,\"generation\":%lu,"
        "\"forward\":%d,\"right\":%d,\"turn\":%d,\"wheels\":[%d,%d,%d,%d]}",
        state.enabled ? "true" : "false", state.ready ? "true" : "false",
        state.moving ? "true" : "false", state.fault,
        static_cast<unsigned long>(state.generation), state.forward, state.right, state.turn,
        state.wheels[0], state.wheels[1], state.wheels[2], state.wheels[3]);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

// 输入：三轴百分数和指令代次表单；输出：HTTP 结果；作用：严格校验后续租或停车。
esp_err_t CameraWeb::setMotor(httpd_req_t* req)
{
    char body[160] = {};
    if (req->content_len == 0 || req->content_len >= sizeof(body))
    {
        // 有问题的控制请求撤销旧运动，不能依靠旧指令继续行驶。
        m_motor.stop("invalid body");
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid motor body length");
    }
    size_t received = 0;
    while (received < req->content_len)
    {
        // 网络分段接收直到正文完整；中断立即停车，不对部分参数执行运动。
        const int count = httpd_req_recv(req, body + received, req->content_len - received);
        if (count <= 0)
        {
            m_motor.stop("incomplete command");
            return ESP_FAIL;
        }
        received += count;
    }
    const char* names[] = {"forward", "right", "turn"};
    int values[3] = {};
    for (size_t axis = 0; axis < 3; ++axis)
    {
        char text[16] = {};
        char* end = nullptr;
        errno = 0;
        const esp_err_t parsed = httpd_query_key_value(body, names[axis], text, sizeof(text));
        const long value = std::strtol(text, &end, 10);
        if (parsed != ESP_OK || errno != 0 || end == text || *end != '\0' || value < -100 || value > 100)
        {
            m_motor.stop("invalid axis");
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected axes in -100..100");
        }
        values[axis] = static_cast<int>(value);
    }
    if (values[0] == 0 && values[1] == 0 && values[2] == 0)
    {
        // 全零停车不要求代次，即使网页状态已经过期也能撤销当前运动。
        const esp_err_t result = m_motor.stop("web stop");
        if (result != ESP_OK)
        {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Motor stop unconfirmed; check hardware");
        }
        return sendMotor(req);
    }
    if (m_motorFd >= 0)
    {
        // 长连接控制期间只保留 HTTP 备用停车和只读诊断，避免两个通道同时改变方向。
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Use the active WebSocket controller");
    }
    char text[16] = {};
    char* end = nullptr;
    errno = 0;
    const esp_err_t parsed = httpd_query_key_value(body, "generation", text, sizeof(text));
    const unsigned long generation = std::strtoul(text, &end, 10);
    if (parsed != ESP_OK || text[0] == '-' || errno != 0 || end == text || *end != '\0')
    {
        m_motor.stop("invalid generation");
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected unsigned generation");
    }
    if (!m_connected.load())
    {
        // 网络接口未就绪时只能停车，不接受新运动。
        requestMotionStop();
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "Network unavailable");
    }
    const esp_err_t result = m_motor.move(values[0], values[1], values[2], generation);
    if (result == ESP_ERR_INVALID_STATE)
    {
        // 迟到包只拒绝，不再触发停车，防止旧包干扰已经建立的新代次。
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Motor unavailable or stale generation; release and retry");
    }
    if (result != ESP_OK)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Motor driver failed; check hardware");
    }
    return sendMotor(req);
}

esp_err_t CameraWeb::setControl(httpd_req_t* req)
{
    char body[96] = {};
    char name[24] = {};
    char text[16] = {};
    if (req->content_len == 0 || req->content_len >= sizeof(body))
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid body length");
    }
    size_t received = 0;
    // TCP 数据可能分多次到达；三秒接收超时由 HTTP 服务器统一限制。
    while (received < req->content_len)
    {
        const int count = httpd_req_recv(req, body + received, req->content_len - received);
        if (count <= 0)
        {
            return ESP_FAIL;
        }
        received += count;
    }
    if (httpd_query_key_value(body, "var", name, sizeof(name)) != ESP_OK ||
        httpd_query_key_value(body, "val", text, sizeof(text)) != ESP_OK)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected var and val");
    }
    char* end = nullptr;
    errno = 0;
    const long value = std::strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || value < INT_MIN || value > INT_MAX)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid integer");
    }
    ESP_LOGI(TAG, "setControl() >> name=%s value=%ld", name, value);
    bool applied;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        // 参数设置与取帧互斥，避免传感器寄存器在另一个任务采集时被并发修改。
        applied = applySetting(name, static_cast<int>(value));
        cacheStatus();
    }
    ESP_LOGI(TAG, "setControl() << result=%s", applied ? "OK" : "REJECTED");
    if (!applied)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unsupported setting or range");
    }
    return sendStatus(req);
}

bool CameraWeb::applySetting(const char* name, int value)
{
    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor == nullptr)
    {
        // 测量模式可能主动停止相机，拒绝控制请求，避免访问已销毁的传感器对象。
        return false;
    }
    if (std::strcmp(name, "auto_exposure") == 0)
    {
        // 只接受明确的布尔值，切换完成后以硬件回读为准。
        return (value == 0 || value == 1) && setAutoExposure(sensor, value == 1);
    }
    if (std::strcmp(name, "exposure_us") == 0)
    {
        if (!readExposure(sensor) || m_isAutoExposure ||
            value < std::ceil(m_lineUs) || value > std::floor(m_expMaxLines * m_lineUs))
        {
            // 接口以整数微秒接收；仅开放当前时序可表达的范围，不把错误值隐式夹紧。
            return false;
        }
        const int lines = std::clamp(static_cast<int>(std::llround(value / m_lineUs)), 1, m_expMaxLines);
        return setExposure(sensor, lines);
    }
    if (std::strcmp(name, "max_fps") == 0)
    {
        if (value < 1 || value > 30)
        {
            return false;
        }
        m_maxFps = value;
        return true;
    }
    if (std::strcmp(name, "size") == 0)
    {
        framesize_t size;
        // 网页编号不依赖驱动枚举的底层数值；保留项目 03 的六档分辨率，不额外增加新界面选项。
        switch (value)
        {
            case 0: size = FRAMESIZE_QQVGA; break;
            case 1: size = FRAMESIZE_QVGA; break;
            case 2: size = FRAMESIZE_VGA; break;
            case 3: size = FRAMESIZE_SVGA; break;
            case 4: size = FRAMESIZE_XGA; break;
            case 5: size = FRAMESIZE_UXGA; break;
            default: return false;
        }
        if (!readExposure(sensor))
        {
            // 保留手动曝光前先取得真实时序，避免根据历史快照进行单位转换。
            return false;
        }
        const bool wasAuto = m_isAutoExposure;
        const double keepUs = m_expLines * m_lineUs;
        if (sensor->set_framesize(sensor, size) != 0)
        {
            return false;
        }
        m_size = value;
        m_expMaxLines = 0;
        // 新模式的 VTS 由驱动重建，恢复自动曝光时应使用新模式的基础帧长。
        m_autoVts = 0;
        if (!readExposure(sensor))
        {
            // 分辨率已经切换，状态仍发布真实模式，但曝光不可用时不再尝试写入寄存器。
            return false;
        }
        if (!wasAuto)
        {
            // 手动模式尽量保持切换前的毫秒值；新模式范围不足时取最近的有效行数。
            const int lines = std::clamp(static_cast<int>(std::llround(keepUs / m_lineUs)), 1, m_expMaxLines);
            return setExposure(sensor, lines);
        }
        return true;
    }
    if (std::strcmp(name, "quality") == 0)
    {
        // 数值越小画质越高；最低限制 10，避免极端高码率挤满帧缓冲。
        return value >= 10 && value <= 63 && sensor->set_quality(sensor, value) == 0;
    }
    if (std::strcmp(name, "brightness") == 0)
    {
        return value >= -2 && value <= 2 && sensor->set_brightness(sensor, value) == 0;
    }
    if (std::strcmp(name, "contrast") == 0)
    {
        return value >= -2 && value <= 2 && sensor->set_contrast(sensor, value) == 0;
    }
    if (std::strcmp(name, "saturation") == 0)
    {
        return value >= -2 && value <= 2 && sensor->set_saturation(sensor, value) == 0;
    }
    if (std::strcmp(name, "hmirror") == 0)
    {
        return (value == 0 || value == 1) && sensor->set_hmirror(sensor, value) == 0;
    }
    if (std::strcmp(name, "vflip") == 0)
    {
        return (value == 0 || value == 1) && sensor->set_vflip(sensor, value) == 0;
    }
    return false;
}

void CameraWeb::finishStream()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_streaming = false;
    m_actualFps = 0;
    {
        // 只清理活动标志，累计字节跨连接保留，主任务继续正确计算相邻窗口差值。
        std::lock_guard<std::mutex> statusLock(m_statusMutex);
        m_isVideoActive = false;
    }
    cacheStatus();
}

esp_err_t CameraWeb::onStream(httpd_req_t* req)
{
    auto* app = static_cast<CameraWeb*>(req->user_ctx);
    const unsigned epoch = app->m_epoch.load();
    if (!app->m_connected.load())
    {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "Wi-Fi disconnected");
    }
    ESP_LOGI(TAG, "onStream() >> socket=%d wifi_connected=%d internal_free=%u",
             httpd_req_to_sockfd(req), int(app->m_connected.load()),
             unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    unsigned frames = 0;
    // 记录本次连接完整发出的帧数，区分首帧失败和持续传输后的失败。
    unsigned totalFrames = 0;
    // 连续故障期间最多恢复两次，避免持续硬件故障导致无限重试。
    unsigned badFrames = 0;
    unsigned recoveries = 0;
    // 连续正常 30 帧后重置恢复预算，允许长连接处理分散的瞬时采集故障。
    unsigned goodFrames = 0;
    int64_t window = 0;
    int64_t nextFrame = esp_timer_get_time();
    esp_err_t result = ESP_OK;
    while (app->m_connected.load() && epoch == app->m_epoch.load())
    {
        // 使用单调时钟限速；向上取整到 tick，保证发送频率不超过设定上限。
        const int64_t remaining = nextFrame - esp_timer_get_time();
        if (remaining > 0)
        {
            const int64_t tickUs = 1000000 / configTICK_RATE_HZ;
            vTaskDelay(static_cast<TickType_t>((remaining + tickUs - 1) / tickUs));
        }
        if (epoch != app->m_epoch.load())
        {
            break;
        }
        const int64_t started = esp_timer_get_time();
        camera_fb_t* frame = nullptr;
        int maxFps;
        int64_t captureStart;
        int64_t captureUs;
        {
            std::lock_guard<std::mutex> lock(app->m_mutex);
            maxFps = app->m_maxFps;
            if (!app->m_streaming)
            {
                // 只在开始时发布状态，不为每一帧重复格式化 JSON。
                app->m_streaming = true;
                {
                    // 运行统计不用相机锁读取活动状态，发送阻塞时 CPU 采样仍可继续。
                    std::lock_guard<std::mutex> statusLock(app->m_statusMutex);
                    app->m_isVideoActive = true;
                }
                app->cacheStatus();
            }
            captureStart = esp_timer_get_time();
            frame = esp_camera_fb_get();
            captureUs = esp_timer_get_time() - captureStart;
        }
        std::unique_ptr<camera_fb_t, decltype(&esp_camera_fb_return)>
            holder(frame, esp_camera_fb_return);
        if (frame == nullptr || frame->format != PIXFORMAT_JPEG)
        {
            ESP_LOGE(TAG, "取帧失败 capture_ms=%lld lock_wait_ms=%lld frame_present=%d",
                     (long long)(captureUs / 1000),
                     (long long)((captureStart - started) / 1000), int(frame != nullptr));
            app->m_frameTimeouts.fetch_add(1);
            holder.reset();
            goodFrames = 0;
            // 单次采集超时先执行有界恢复；只有连续故障耗尽预算才结束网络流。
            if (recoveries >= 2 || app->recoverCamera() != ESP_OK)
            {
                result = ESP_FAIL;
                break;
            }
            ++recoveries;
            continue;
        }
        if (!isValidJpeg(frame))
        {
            app->m_jpegErrors.fetch_add(1);
            goodFrames = 0;
            ESP_LOGW(TAG, "stream JPEG rejected bytes=%u consecutive=%u", unsigned(frame->len), badFrames + 1);
            holder.reset();
            if (++badFrames >= 3)
            {
                // 先归还驱动帧再设置寄存器，避免恢复时占住旧帧缓冲。
                if (recoveries >= 2 || app->recoverCamera() != ESP_OK)
                {
                    result = ESP_FAIL;
                    break;
                }
                ++recoveries;
                badFrames = 0;
            }
            continue;
        }
        badFrames = 0;
        if (++goodFrames >= 30)
        {
            recoveries = 0;
            goodFrames = 30;
        }
        if (totalFrames == 0)
        {
            // 首帧记录尺寸和 JPEG 起始标志，后续正常帧不逐帧刷屏。
            const bool hasSoi = frame->len >= 2 && frame->buf[0] == 0xff && frame->buf[1] == 0xd8;
            ESP_LOGI(TAG, "首帧已取得 bytes=%u width=%u height=%u jpeg_soi=%d capture_ms=%lld lock_wait_ms=%lld",
                     unsigned(frame->len), unsigned(frame->width), unsigned(frame->height), int(hasSoi),
                     (long long)(captureUs / 1000),
                     (long long)((captureStart - started) / 1000));
        }
        char header[128];
        const size_t jpegBytes = frame->len;
        const int length = std::snprintf(header, sizeof(header),
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            unsigned(jpegBytes));
        // 网络层直接读取 DMA 的 PSRAM 帧，不再将完整 JPEG 复制到第二个 PSRAM 发送包。
        // 归还前由 holder 独占此帧，三缓冲允许另外两帧继续采集；底层 TCP/Wi-Fi 仍有自身复制。
        const int64_t sendStart = esp_timer_get_time();
        const char* stage = "frame_header";
        errno = 0;
        result = httpd_resp_send_chunk(req, header, length);
        if (result == ESP_OK)
        {
            stage = "jpeg_body";
            result = httpd_resp_send_chunk(req, reinterpret_cast<const char*>(frame->buf), jpegBytes);
        }
        if (result == ESP_OK)
        {
            stage = "frame_end";
            result = httpd_resp_send_chunk(req, "\r\n", 2);
        }
        // 返回后立即计时并保存 errno，避免首帧日志或状态查询污染发送耗时和错误信息。
        const int sendError = errno;
        const int64_t sentUs = esp_timer_get_time() - sendStart;
        holder.reset();
        if (result != ESP_OK)
        {
            // 必须先保存 errno，再查询无线状态，避免后续接口覆盖原始发送错误。
            const int sendErrno = sendError;
            const int64_t sendUs = esp_timer_get_time() - sendStart;
            ESP_LOGW(TAG, "视频发送失败 stage=%s result=%s errno=%d completed_frames=%u bytes=%u send_ms=%lld",
                     stage, esp_err_to_name(result), sendErrno, totalFrames,
                     unsigned(jpegBytes), (long long)(sendUs / 1000));
#if CONFIG_CAMERA_WIFI_APSTA
            // AP 没有上游路由器信号强度；改为记录热点状态及当前接入设备数量。
            wifi_sta_list_t clients = {};
            const esp_err_t wifiResult = esp_wifi_ap_get_sta_list(&clients);
            ESP_LOGW(TAG, "发送失败时 AP ready=%d clients=%d list_result=%s internal_free=%u",
                     int(app->m_apReady.load()), wifiResult == ESP_OK ? clients.num : -1,
                     esp_err_to_name(wifiResult), unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
#else
            wifi_ap_record_t access = {};
            const esp_err_t wifiResult = esp_wifi_sta_get_ap_info(&access);
            if (wifiResult == ESP_OK)
            {
                ESP_LOGW(TAG, "发送失败时 Wi-Fi 仍关联 rssi_dbm=%d internal_free=%u",
                         access.rssi, unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
            }
            else
            {
                ESP_LOGW(TAG, "发送失败时无法取得 Wi-Fi 状态 result=%s", esp_err_to_name(wifiResult));
            }
#endif
            break;
        }
        if (totalFrames == 0)
        {
            ESP_LOGI(TAG, "首帧发送完成 send_ms=%lld",
                     (long long)((esp_timer_get_time() - sendStart) / 1000));
        }
        ++totalFrames;
        const int64_t now = esp_timer_get_time();
        {
            // 每完整提交一帧只累加数值，不格式化 JSON 或额外打印逐帧日志。
            std::lock_guard<std::mutex> lock(app->m_statusMutex);
            app->m_videoBytes += jpegBytes;
            ++app->m_videoFrames;
            app->m_captureUs += captureUs;
            app->m_sendUs += sentUs;
        }
        if (window == 0)
        {
            // 以第一帧完成时间为起点，只统计之后的帧间隔，避免首个窗口多算一帧。
            window = now;
        }
        else
        {
            ++frames;
        }
        if (now - window >= 1000000)
        {
            std::lock_guard<std::mutex> lock(app->m_mutex);
            app->m_actualFps = frames * 1000000.0 / (now - window);
            app->cacheStatus();
            frames = 0;
            window = now;
        }
        // 传输本身已经较慢时不追赶历史帧，避免恢复后瞬间突发发送。
        nextFrame = started + (1000000 + maxFps - 1) / maxFps;
    }
    app->finishStream();
    if (result == ESP_OK)
    {
        result = httpd_resp_send_chunk(req, "--frame--\r\n", HTTPD_RESP_USE_STRLEN);
        if (result == ESP_OK)
        {
            result = httpd_resp_send_chunk(req, nullptr, 0);
        }
    }
    ESP_LOGI(TAG, "onStream() << result=%s completed_frames=%u",
             esp_err_to_name(result), totalFrames);
    return result;
}
