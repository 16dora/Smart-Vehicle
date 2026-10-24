#include "oled_display.h"
#include <algorithm>
#include <cstdio>
#include "esp_log.h"
#if CONFIG_CAMERA_OLED_ENABLED
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if !CONFIG_ESP_CONSOLE_NONE
#error "OLED UART0 pin reuse requires CONFIG_ESP_CONSOLE_NONE"
#endif
#endif

// 显示驱动专用日志标签，不再混入相机服务的日志。
static constexpr char TAG[] = "oled_display";

#if CONFIG_CAMERA_OLED_ENABLED
// 输入：命令/数据标志及字节数组；输出：SDK 结果；作用：OLED 只写，CS 由硬件接地。
esp_err_t OledDisplay::write(bool isData, const uint8_t* bytes, size_t count)
{
    ESP_LOGI(TAG, "gpio_set_level() >> gpio=15 level=%d", int(isData));
    esp_err_t result = gpio_set_level(GPIO_NUM_15, isData ? 1 : 0);
    ESP_LOGI(TAG, "gpio_set_level() << result=%s", esp_err_to_name(result));
    if (result != ESP_OK)
    {
        // 命令/数据选择失败时不能继续发送，以免破坏控制寄存器。
        return result;
    }
    // ESP32 无 DMA 时单次 FIFO 仅支持 64 字节；CS 接地，分块之间保持 D/C 不变。
    for (size_t offset = 0; offset < count; offset += 64)
    {
        const size_t length = std::min(size_t{64}, count - offset);
        spi_transaction_t transfer{};
        transfer.length = length * 8;
        transfer.tx_buffer = bytes + offset;
        ESP_LOGI(TAG, "spi_device_polling_transmit() >> data=%d offset=%u bytes=%u",
                 int(isData), unsigned(offset), unsigned(length));
        result = spi_device_polling_transmit(m_device, &transfer);
        ESP_LOGI(TAG, "spi_device_polling_transmit() << result=%s", esp_err_to_name(result));
        if (result != ESP_OK)
        {
            // 中途失败停止后续分块，下一刷新重新指定地址并发送完整显存。
            return result;
        }
    }
    return result;
}

// 输入：无；输出：初始化结果；作用：使用板载 RC 复位与双电源初始化 SSD1306。
esp_err_t OledDisplay::start()
{
    ESP_LOGI(TAG, "start() >>");
    gpio_config_t pins{};
    pins.pin_bit_mask = 1ULL << GPIO_NUM_15;
    pins.mode = GPIO_MODE_OUTPUT;
    ESP_LOGI(TAG, "gpio_config() >> mask=%llu mode=output pull=none interrupt=disabled", pins.pin_bit_mask);
    esp_err_t result = gpio_config(&pins);
    ESP_LOGI(TAG, "gpio_config() << result=%s", esp_err_to_name(result));
    if (result != ESP_OK)
    {
        ESP_LOGW(TAG, "start() << result=%s", esp_err_to_name(result));
        return result;
    }
    spi_bus_config_t bus{};
    bus.mosi_io_num = GPIO_NUM_1;
    bus.miso_io_num = -1;
    bus.sclk_io_num = GPIO_NUM_3;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 64;
    // 低频状态屏不使用 DMA，显存可以保持在主任务栈中。
    ESP_LOGI(TAG, "spi_bus_initialize() >> host=SPI2 mosi=1 miso=-1 sclk=3 wp=-1 hd=-1 max=64 dma=disabled");
    result = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
    ESP_LOGI(TAG, "spi_bus_initialize() << result=%s", esp_err_to_name(result));
    if (result != ESP_OK)
    {
        ESP_LOGW(TAG, "start() << result=%s", esp_err_to_name(result));
        return result;
    }
    spi_device_interface_config_t device{};
    device.clock_speed_hz = 1000000;
    device.mode = 0;
    device.spics_io_num = -1;
    device.queue_size = 1;
    ESP_LOGI(TAG, "spi_bus_add_device() >> host=SPI2 hz=1000000 mode=0 cs=-1 queue=1");
    result = spi_bus_add_device(SPI2_HOST, &device, &m_device);
    ESP_LOGI(TAG, "spi_bus_add_device() << result=%s", esp_err_to_name(result));
    if (result == ESP_OK)
    {
        // 等待板载 RC 复位和双电源稳定；外部 5V 接 VCC，关闭内部升压泵。
        vTaskDelay(pdMS_TO_TICKS(100));
        const uint8_t commands[] = {0xae, 0xd5, 0x80, 0xa8, 0x3f, 0xd3, 0x00,
            0x40, 0x8d, 0x10, 0x20, 0x00, 0xa1, 0xc8, 0xda, 0x12,
            0x81, 0x7f, 0xd9, 0x22, 0xdb, 0x20, 0xa4, 0xa6};
        result = write(false, commands, sizeof(commands));
    }
    if (result != ESP_OK)
    {
        // 初始化未完成时释放已申请资源，可选硬件失败不影响相机。
        if (m_device != nullptr)
        {
            ESP_LOGI(TAG, "spi_bus_remove_device() >> oled");
            const esp_err_t removed = spi_bus_remove_device(m_device);
            ESP_LOGI(TAG, "spi_bus_remove_device() << result=%s", esp_err_to_name(removed));
            m_device = nullptr;
        }
        ESP_LOGI(TAG, "spi_bus_free() >> host=SPI2");
        const esp_err_t freed = spi_bus_free(SPI2_HOST);
        ESP_LOGI(TAG, "spi_bus_free() << result=%s", esp_err_to_name(freed));
        ESP_LOGW(TAG, "start() << result=%s", esp_err_to_name(result));
        return result;
    }
    update({});
    ESP_LOGI(TAG, "start() << result=OK");
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
void OledDisplay::update(const OledState& state)
{
    if (m_device == nullptr)
    {
        // 配置关闭或初始化失败时不发送数据。
        return;
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
    const uint8_t address[] = {0x21, 0x00, 0x7f, 0x22, 0x00, 0x07};
    esp_err_t result = write(false, address, sizeof(address));
    if (result == ESP_OK)
    {
        // 每两秒只发送一帧；传输失败不立即循环重试，下一采样周期重画完整显存。
        result = write(true, frame.data(), frame.size());
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
}
#else
// 输入：无；输出：成功；作用：禁用构建不访问 GPIO 或 SPI。
esp_err_t OledDisplay::start()
{
    ESP_LOGI(TAG, "start() >> enabled=0");
    ESP_LOGI(TAG, "start() << result=DISABLED");
    return ESP_OK;
}

// 输入：显示快照；输出：无；作用：禁用构建保持空操作。
void OledDisplay::update(const OledState& state)
{
    (void)state;
}
#endif
