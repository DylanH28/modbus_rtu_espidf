#include "modbus_rtu_internal.h"

#include "driver/uart.h"
#include "esp_rom_sys.h"

static const char *TAG = "mb_port";

static inline void de_re_set(mb_port_t *p, bool tx)
{
    if (!p) return;
    if (p->rs485_mode) return;
    if (p->de_re_io == GPIO_NUM_NC) return;

    bool level = tx ? p->de_re_active_high : !p->de_re_active_high;
    gpio_set_level(p->de_re_io, level);
}

esp_err_t mb_port_init(mb_port_t *p, const modbus_rtu_uart_config_t *uart_cfg,
                       int inter_frame_timeout_us, int txrx_turnaround_us)
{
    if (!p || !uart_cfg) return ESP_ERR_INVALID_ARG;

    p->uart_num = uart_cfg->uart_num;
    p->rs485_mode = uart_cfg->use_uart_rs485_mode;
    p->de_re_io = uart_cfg->de_re_io;
    p->de_re_active_high = uart_cfg->de_re_active_high;
    p->txrx_turnaround_us = (txrx_turnaround_us < 0) ? 0 : txrx_turnaround_us;
    p->inter_frame_timeout_us = (inter_frame_timeout_us <= 0) ? 2000 : inter_frame_timeout_us;

    uart_config_t ucfg = {
        .baud_rate = uart_cfg->baudrate ? uart_cfg->baudrate : 115200,
        .data_bits = uart_cfg->data_bits ? uart_cfg->data_bits : UART_DATA_8_BITS,
        .parity    = uart_cfg->parity,
        .stop_bits = uart_cfg->stop_bits ? uart_cfg->stop_bits : UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err;

    err = uart_driver_install(
        p->uart_num,
        uart_cfg->rx_buf_size ? uart_cfg->rx_buf_size : MB_RXBUF_DEFAULT,
        uart_cfg->tx_buf_size ? uart_cfg->tx_buf_size : MB_TXBUF_DEFAULT,
        uart_cfg->uart_event_queue_size ? uart_cfg->uart_event_queue_size : MB_EVTQ_DEFAULT,
        NULL,
        0
    );
    if (err != ESP_OK) { ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(err)); return err; }

    err = uart_param_config(p->uart_num, &ucfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(err)); return err; }

    err = uart_set_pin(p->uart_num, uart_cfg->tx_io, uart_cfg->rx_io, uart_cfg->rts_io, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) { ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(err)); return err; }

    if (p->rs485_mode) {
        err = uart_set_mode(p->uart_num, UART_MODE_RS485_HALF_DUPLEX);
        if (err != ESP_OK) { ESP_LOGE(TAG, "uart_set_mode RS485: %s", esp_err_to_name(err)); return err; }
    } else {
        if (p->de_re_io != GPIO_NUM_NC) {
            gpio_config_t io = {
                .pin_bit_mask = (1ULL << p->de_re_io),
                .mode = GPIO_MODE_OUTPUT,
                .pull_down_en = 0,
                .pull_up_en = 0,
                .intr_type = GPIO_INTR_DISABLE
            };
            err = gpio_config(&io);
            if (err != ESP_OK) { ESP_LOGE(TAG, "gpio_config: %s", esp_err_to_name(err)); return err; }
            de_re_set(p, false);
        }
    }

    uart_flush_input(p->uart_num);
    de_re_set(p, false);
    return ESP_OK;
}

void mb_port_deinit(mb_port_t *p)
{
    if (!p) return;
    uart_driver_delete(p->uart_num);
}

esp_err_t mb_port_write_adu(mb_port_t *p, const uint8_t *adu, size_t adu_len)
{
    if (!p || !adu || adu_len == 0) return ESP_ERR_INVALID_ARG;

    uart_flush_input(p->uart_num);

    if (p->txrx_turnaround_us) esp_rom_delay_us((uint32_t)p->txrx_turnaround_us);

    de_re_set(p, true);
    int w = uart_write_bytes(p->uart_num, (const char*)adu, (int)adu_len);
    if (w < 0 || (size_t)w != adu_len) {
        de_re_set(p, false);
        return ESP_ERR_MODBUS_RTU_PORT;
    }
    uart_wait_tx_done(p->uart_num, pdMS_TO_TICKS(50));

    if (p->txrx_turnaround_us) esp_rom_delay_us((uint32_t)p->txrx_turnaround_us);
    de_re_set(p, false);
    return ESP_OK;
}

// Read a single RTU frame:
// - read bytes in small chunks
// - if idle >= inter_frame_timeout_us after having received data => end-of-frame
// - overall_timeout_ms caps total waiting time
esp_err_t mb_port_read_frame(mb_port_t *p, uint8_t *buf, size_t buf_len,
                             size_t *out_len, int overall_timeout_ms)
{
    if (!p || !buf || !out_len || buf_len < 5) return ESP_ERR_INVALID_ARG;
    *out_len = 0;

    const int64_t start_us = mb_time_us();
    int64_t last_rx_us = 0;
    bool got_any = false;

    while (1) {
        int64_t now_us = mb_time_us();
        int64_t elapsed_ms = (now_us - start_us) / 1000;
        if (overall_timeout_ms >= 0 && elapsed_ms >= overall_timeout_ms) {
            return ESP_ERR_MODBUS_RTU_TIMEOUT;
        }

        int to_read = (int)(buf_len - *out_len);
        if (to_read <= 0) return ESP_ERR_NO_MEM;

        int r = uart_read_bytes(p->uart_num, buf + *out_len, to_read, pdMS_TO_TICKS(5));
        if (r > 0) {
            *out_len += (size_t)r;
            last_rx_us = mb_time_us();
            got_any = true;

            if (*out_len >= buf_len) return ESP_ERR_NO_MEM;
        } else {
            if (got_any) {
                int64_t idle_us = mb_time_us() - last_rx_us;
                if (idle_us >= p->inter_frame_timeout_us) {
                    return ESP_OK;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
