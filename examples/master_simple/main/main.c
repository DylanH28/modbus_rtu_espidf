#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "modbus_rtu.h"

static const char *TAG = "example_master";

void app_main(void)
{
    modbus_rtu_t *mb = NULL;

    modbus_rtu_uart_config_t ucfg = {
        .uart_num = UART_NUM_1,
        .baudrate = 115200,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .data_bits = UART_DATA_8_BITS,
        .tx_io = GPIO_NUM_17,
        .rx_io = GPIO_NUM_16,
        .rts_io = UART_PIN_NO_CHANGE,
        .use_uart_rs485_mode = true,
        .de_re_io = GPIO_NUM_NC,
        .de_re_active_high = true,
    };

    modbus_rtu_master_config_t mcfg = {
        .response_timeout_ms = 200,
        .inter_frame_timeout_us = 2000,
        .txrx_turnaround_us = 200,
        .strict_unit_id = true,
        .strict_function = true,
    };

    ESP_ERROR_CHECK(modbus_rtu_master_create(&ucfg, &mcfg, &mb));

    modbus_rtu_exception_t ex = {0};

    while (1) {
        uint16_t regs[4] = {0};
        esp_err_t err = modbus_rtu_read_holding_registers(mb, 1, 0x0000, 4, regs, 4, &ex);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "HR[0..3]=%u %u %u %u", regs[0], regs[1], regs[2], regs[3]);
        } else if (err == ESP_ERR_MODBUS_RTU_EXCEPTION) {
            ESP_LOGW(TAG, "Modbus exception: fc=%u ex=%u", ex.function, ex.exception_code);
        } else {
            ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
