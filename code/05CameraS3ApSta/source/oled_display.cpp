#include "oled_display.h"
#include "board_pins.h"
#include <algorithm>
#include <cstdio>
#include "esp_log.h"
#if CONFIG_CAMERA_OLED_ENABLED
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

// 显示驱动专用日志标签，不再混入相机服务的日志。
static constexpr char TAG[] = "oled_display";

#if CONFIG_CAMERA_OLED_ENABLED
// 输入：命令/数据标志及字节数组；输出：SDK 结果；作用：同步发送 SH1106 I2C 控制字节和内容。
esp_err_t OledDisplay::write(bool isData, const uint8_t* bytes, size_t count)
{
    if (m_device == nullptr) return ESP_ERR_INVALID_STATE;
    // 分块控制栈内存占用；每次同步传输完成才复用缓冲，失败即结束本帧，不循环重试。
    for (size_t offset = 0; offset < count; offset += 64)
    {
        const size_t length = std::min(size_t{64}, count - offset);
        uint8_t packet[65];
        packet[0] = isData ? 0x40 : 0x00;
        std::copy_n(bytes + offset, length, packet + 1);
        ESP_LOGI(TAG, "i2c_master_transmit() >> data=%d offset=%u bytes=%u timeout_ms=100",
                 int(isData), unsigned(offset), unsigned(length + 1));
        const esp_err_t result = i2c_master_transmit(m_device, packet, length + 1, 100);
        ESP_LOGI(TAG, "i2c_master_transmit() << result=%s", esp_err_to_name(result));
        if (result != ESP_OK) return result;
    }
    return ESP_OK;
}

// 输入：无；输出：初始化结果；作用：使用独立 I2C0 探测并初始化 SH1106 128×64 屏幕。
esp_err_t OledDisplay::start()
{
    ESP_LOGI(TAG, "start() >> controller=SH1106 sda=%d scl=%d", int(board::OLED_SDA), int(board::OLED_SCL));
    i2c_master_bus_config_t bus{};
    bus.i2c_port = I2C_NUM_0;
    bus.sda_io_num = board::OLED_SDA;
    bus.scl_io_num = board::OLED_SCL;
    bus.clk_source = I2C_CLK_SRC_DEFAULT;
    bus.glitch_ignore_cnt = 7;
    bus.flags.enable_internal_pullup = true;
    ESP_LOGI(TAG, "i2c_new_master_bus() >> port=0 sda=%d scl=%d glitch=7 pullup=1",
             int(board::OLED_SDA), int(board::OLED_SCL));
    esp_err_t result = i2c_new_master_bus(&bus, &m_bus);
    ESP_LOGI(TAG, "i2c_new_master_bus() << result=%s", esp_err_to_name(result));
    if (result != ESP_OK)
    {
        ESP_LOGW(TAG, "start() << result=%s", esp_err_to_name(result));
        return result;
    }
    // 四针模块无外部复位线，等待供电稳定；只探测 SH1106 的两个常用地址，避免扫描其他硬件。
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t address = 0x3c;
    for (; address <= 0x3d; ++address)
    {
        ESP_LOGI(TAG, "i2c_master_probe() >> address=0x%02x timeout_ms=50", address);
        result = i2c_master_probe(m_bus, address, 50);
        ESP_LOGI(TAG, "i2c_master_probe() << result=%s", esp_err_to_name(result));
        // 地址未应答可试下一地址；总线超时等错误不继续投递操作。
        if (result != ESP_ERR_NOT_FOUND) break;
    }
    if (result == ESP_OK)
    {
        i2c_device_config_t device{};
        device.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        device.device_address = address;
        device.scl_speed_hz = 100000;
        ESP_LOGI(TAG, "i2c_master_bus_add_device() >> address=0x%02x hz=100000", address);
        result = i2c_master_bus_add_device(m_bus, &device, &m_device);
        ESP_LOGI(TAG, "i2c_master_bus_add_device() << result=%s", esp_err_to_name(result));
    }
    if (result == ESP_OK)
    {
        // SH1106 启用内部 DC/DC，使用 64 行扫描；不再使用旧 SSD1306 的水平寻址或外部 5V 配置。
        const uint8_t commands[] = {0xae, 0xd5, 0x80, 0xa8, 0x3f, 0xd3, 0x00,
            0x40, 0xad, 0x8b, 0xa1, 0xc8, 0xda, 0x12,
            0x81, 0x7f, 0xd9, 0x1f, 0xdb, 0x40, 0xa4, 0xa6};
        result = write(false, commands, sizeof(commands));
        if (result == ESP_OK)
        {
            // 等待升压稳定后清空显存再点亮，避免上电显示随机内容。
            vTaskDelay(pdMS_TO_TICKS(100));
            result = update({});
        }
    }
    if (result != ESP_OK)
    {
        // 屏幕缺失或首帧失败时释放 I2C0；可选 OLED 失败不阻止后续相机和网络启动。
        if (m_device != nullptr)
        {
            ESP_LOGI(TAG, "i2c_master_bus_rm_device() >> oled");
            const esp_err_t removed = i2c_master_bus_rm_device(m_device);
            ESP_LOGI(TAG, "i2c_master_bus_rm_device() << result=%s", esp_err_to_name(removed));
            m_device = nullptr;
        }
        ESP_LOGI(TAG, "i2c_del_master_bus() >> port=0");
        const esp_err_t freed = i2c_del_master_bus(m_bus);
        ESP_LOGI(TAG, "i2c_del_master_bus() << result=%s", esp_err_to_name(freed));
        m_bus = nullptr;
        ESP_LOGW(TAG, "start() << result=%s", esp_err_to_name(result));
        return result;
    }
    ESP_LOGI("camera_main", "OLED SH1106 ready address=0x%02x sda=%d scl=%d hz=100000",
             address, int(board::OLED_SDA), int(board::OLED_SCL));
    ESP_LOGI(TAG, "start() << result=OK address=0x%02x", address);
    return ESP_OK;
}

// 输入：显存、起始页及文本；输出：无；作用：按 6×16 单元绘制，最多显示 21 个字符。
void OledDisplay::drawText(std::array<uint8_t, 1024>& frame, int page, const char* value)
{
    const uint8_t digits[10][5] = {
        {0x3e,0x51,0x49,0x45,0x3e}, {0x00,0x42,0x7f,0x40,0x00},
        {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4b,0x31},
        {0x18,0x14,0x12,0x7f,0x10}, {0x27,0x45,0x45,0x45,0x39},
        {0x3c,0x4a,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1e}};
    // 仅需数字和固定状态标签，不引入完整字库；达到屏幕右边界后停止。
    for (int index = 0; value[index] != '\0' && index < 21; ++index)
    {
        std::array<uint8_t, 5> glyph{};
        const char letter = value[index];
        if (letter >= '0' && letter <= '9')
        {
            std::copy_n(digits[letter - '0'], 5, glyph.begin());
        }
        else
        {
            // 固定标签与单位逐项映射，未知字符保留空白。
            switch (letter)
            {
                case 'C': glyph = {0x3e,0x41,0x41,0x41,0x22}; break;
                case 'P': glyph = {0x7f,0x09,0x09,0x09,0x06}; break;
                case 'U': glyph = {0x3f,0x40,0x40,0x40,0x3f}; break;
                case 'E': glyph = {0x7f,0x49,0x49,0x49,0x41}; break;
                case 'X': glyph = {0x63,0x14,0x08,0x14,0x63}; break;
                case 'A': glyph = {0x7e,0x11,0x11,0x11,0x7e}; break;
                case 'O': glyph = {0x3e,0x41,0x41,0x41,0x3e}; break;
                case 'N': glyph = {0x7f,0x02,0x04,0x08,0x7f}; break;
                case 'F': glyph = {0x7f,0x09,0x09,0x09,0x01}; break;
                case 'm': glyph = {0x7c,0x04,0x18,0x04,0x78}; break;
                case 's': glyph = {0x48,0x54,0x54,0x54,0x20}; break;
                case ':': glyph = {0x00,0x36,0x36,0x00,0x00}; break;
                case '.': glyph = {0x00,0x60,0x60,0x00,0x00}; break;
                case '-': glyph = {0x08,0x08,0x08,0x08,0x08}; break;
                case '%': glyph = {0x23,0x13,0x08,0x64,0x62}; break;
                default: break;
            }
        }
        // 垂直放大两倍，字宽保持五列，长积分时间也能完整显示。
        for (int column = 0; column < 5; ++column)
        {
            uint16_t pixels = 0;
            for (int bit = 0; bit < 7; ++bit)
            {
                if ((glyph[column] & (1U << bit)) != 0)
                {
                    pixels |= 3U << (bit * 2);
                }
            }
            const size_t offset = page * 128 + index * 6 + column;
            frame[offset] = pixels & 0xff;
            frame[offset + 128] = pixels >> 8;
        }
    }
}

// 输入：当前 CPU 窗口；输出：无；作用：使用快照刷新 OLED，无效或过期曝光显示 --。
esp_err_t OledDisplay::update(const OledState& state)
{
    if (m_device == nullptr)
    {
        // 初始化失败时不发送数据，由主任务维持正常的低频采样。
        return ESP_ERR_INVALID_STATE;
    }
    const double exposureMs = state.exposureMs;
    const bool isExpValid = state.isExpValid;
    const bool isAuto = state.isAuto;
    const bool isCpuValid = state.isCpuValid;
    const std::array<double, 2>& busyPct = state.busyPct;
    std::array<uint8_t, 1024> frame{};
    char line[32];
    // 四行独占两页；首轮 CPU 尚无差值基线时不显示伪造的零负载。
    for (int core = 0; core < 2; ++core)
    {
        if (isCpuValid)
        {
            std::snprintf(line, sizeof(line), "CPU%d: %5.1f%%", core, busyPct[core]);
        }
        else
        {
            std::snprintf(line, sizeof(line), "CPU%d: --", core);
        }
        drawText(frame, core * 2, line);
    }
    if (isExpValid)
    {
        std::snprintf(line, sizeof(line), "EXP: %.2f ms", exposureMs);
    }
    else
    {
        std::snprintf(line, sizeof(line), "EXP: -- ms");
    }
    drawText(frame, 4, line);
    drawText(frame, 6, isExpValid ? (isAuto ? "AEC: ON" : "AEC: OFF") : "AEC: --");
    esp_err_t result = ESP_OK;
    // SH1106 显存为 132 列，常见 128 列面板从第 2 列开始；每页显式定位，不能沿用 SSD1306 寻址。
    for (int page = 0; page < 8 && result == ESP_OK; ++page)
    {
        const uint8_t address[] = {static_cast<uint8_t>(0xb0 + page), 0x02, 0x10};
        result = write(false, address, sizeof(address));
        if (result == ESP_OK) result = write(true, frame.data() + page * 128, 128);
    }
    if (result == ESP_OK)
    {
        const uint8_t displayOn = 0xaf;
        result = write(false, &displayOn, 1);
    }
    if (result != ESP_OK)
    {
        ESP_LOGW(TAG, "update() << result=%s", esp_err_to_name(result));
    }
    return result;
}
#else
// 输入：无；输出：成功；作用：禁用构建不访问 GPIO 或 I2C。
esp_err_t OledDisplay::start()
{
    ESP_LOGI(TAG, "start() >> enabled=0");
    ESP_LOGI(TAG, "start() << result=DISABLED");
    return ESP_OK;
}

// 输入：显示快照；输出：无；作用：禁用构建保持空操作。
esp_err_t OledDisplay::update(const OledState& state)
{
    (void)state;
    return ESP_OK;
}
#endif
