#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "modbus_rtu.h"

static const char *TAG = "example_slave";
static uint16_t holding[16];

static esp_err_t read_holding(uint16_t addr, uint16_t qty, uint16_t *dest, void *user)
{
    (void)user;
    if ((addr + qty) > 16) return ESP_ERR_INVALID_SIZE;
    for (uint16_t i = 0; i < qty; ++i) dest[i] = holding[addr + i];
    return ESP_OK;
}

static esp_err_t write_holding(uint16_t addr, uint16_t qty, const uint16_t *src, void *user)
{
    (void)user;
    if ((addr + qty) > 16) return ESP_ERR_INVALID_SIZE;
    for (uint16_t i = 0; i < qty; ++i) holding[addr + i] = src[i];
    return ESP_OK;
}

void app_main(void)
{
    holding[0] = 123;
    holding[1] = 456;

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

    modbus_rtu_slave_config_t scfg = {
        .unit_id = 1,
        .inter_frame_timeout_us = 2000,
        .rx_poll_delay_ms = 1,
        .txrx_turnaround_us = 200,
        .max_adu_size = 256,
    };

    modbus_rtu_slave_cb_t cb = {
        .read_holding = read_holding,
        .write_holding = write_holding,
    };

    ESP_ERROR_CHECK(modbus_rtu_slave_create(&ucfg, &scfg, &cb, NULL, &mb));
    ESP_ERROR_CHECK(modbus_rtu_slave_start(mb));

    ESP_LOGI(TAG, "Slave running. Try reading holding regs from a master.");

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
