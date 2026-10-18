#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// UART0 connects to the board's CH340 USB serial adapter.
static void send_text(const char *text)
{
    uart_write_bytes(UART_NUM_0, text, strlen(text));
}

void app_main(void)
{
    const uart_config_t config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3,
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0));
    send_text("\r\nREADY smart_vehicle UART0 115200 8N1\r\n");

    char line[128];
    size_t length = 0;
    bool overflow = false;
    unsigned sequence = 0;
    int64_t next_heartbeat = 0;
    while (true) {
        uint8_t byte;
        if (uart_read_bytes(UART_NUM_0, &byte, 1, pdMS_TO_TICKS(20)) > 0) {
            if (byte == '\n') {
                if (overflow) {
                    send_text("ERROR line too long (max 127 bytes)\r\n");
                } else {
                    line[length] = '\0';
                    send_text("ECHO ");
                    send_text(line);
                    send_text("\r\n");
                }
                length = 0;
                overflow = false;
            } else if (byte != '\r' && !overflow) {
                if (length < sizeof(line) - 1) {
                    line[length++] = (char)byte;
                } else {
                    overflow = true;
                }
            }
        }
        const int64_t now = esp_timer_get_time();
        if (now >= next_heartbeat) {
            char status[96];
            snprintf(status, sizeof(status), "HEARTBEAT seq=%u uptime_ms=%lld\r\n",
                     sequence++, (long long)(now / 1000));
            send_text(status);
            next_heartbeat = now + 1000000;
        }
    }
}
