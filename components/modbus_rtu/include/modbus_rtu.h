#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ------------ Errors ------------
#define ESP_ERR_MODBUS_RTU_BASE           0x31000
#define ESP_ERR_MODBUS_RTU_TIMEOUT        (ESP_ERR_MODBUS_RTU_BASE + 1)
#define ESP_ERR_MODBUS_RTU_CRC            (ESP_ERR_MODBUS_RTU_BASE + 2)
#define ESP_ERR_MODBUS_RTU_BAD_RESPONSE   (ESP_ERR_MODBUS_RTU_BASE + 3)
#define ESP_ERR_MODBUS_RTU_EXCEPTION      (ESP_ERR_MODBUS_RTU_BASE + 4)
#define ESP_ERR_MODBUS_RTU_PORT           (ESP_ERR_MODBUS_RTU_BASE + 5)

typedef struct {
    uint8_t function;
    uint8_t exception_code; // standard Modbus exception code
} modbus_rtu_exception_t;

typedef struct modbus_rtu_s modbus_rtu_t;

// ------------ UART / RS485 config ------------
typedef struct {
    uart_port_t uart_num;

    int baudrate;
    uart_parity_t parity;
    uart_stop_bits_t stop_bits;
    uart_word_length_t data_bits;

    gpio_num_t tx_io;
    gpio_num_t rx_io;
    gpio_num_t rts_io;       // optional (GPIO_NUM_NC or UART_PIN_NO_CHANGE)
    bool use_uart_rs485_mode;

    // Manual DE/RE GPIO (optional):
    // If use_uart_rs485_mode == false and de_re_io != GPIO_NUM_NC, we toggle it:
    //   - TX: level = de_re_active_high
    //   - RX: level = !de_re_active_high
    gpio_num_t de_re_io;
    bool de_re_active_high;

    int rx_buf_size;          // default if 0
    int tx_buf_size;          // default if 0
    int uart_event_queue_size;// default if 0
} modbus_rtu_uart_config_t;

// ------------ Master config ------------
typedef struct {
    int response_timeout_ms;
    int inter_frame_timeout_us;
    int txrx_turnaround_us;  // for manual DE/RE
    bool strict_unit_id;
    bool strict_function;
} modbus_rtu_master_config_t;

// ------------ Slave callbacks ------------
typedef esp_err_t (*modbus_rtu_read_bits_cb_t)(uint16_t addr, uint16_t qty, uint8_t *dest_bits, void *user);
typedef esp_err_t (*modbus_rtu_write_bits_cb_t)(uint16_t addr, uint16_t qty, const uint8_t *src_bits, void *user);

typedef esp_err_t (*modbus_rtu_read_regs_cb_t)(uint16_t addr, uint16_t qty, uint16_t *dest_regs, void *user);
typedef esp_err_t (*modbus_rtu_write_regs_cb_t)(uint16_t addr, uint16_t qty, const uint16_t *src_regs, void *user);

// Custom function hook (slave)
typedef esp_err_t (*modbus_rtu_custom_fc_cb_t)(
    uint8_t unit_id,
    uint8_t function,
    const uint8_t *request_pdu, size_t request_pdu_len,
    uint8_t *response_pdu, size_t response_pdu_max, size_t *response_pdu_len,
    void *user
);

typedef struct {
    modbus_rtu_read_bits_cb_t  read_coils;
    modbus_rtu_write_bits_cb_t write_coils;

    modbus_rtu_read_bits_cb_t  read_discrete_inputs;

    modbus_rtu_read_regs_cb_t  read_holding;
    modbus_rtu_write_regs_cb_t write_holding;

    modbus_rtu_read_regs_cb_t  read_input;

    modbus_rtu_custom_fc_cb_t  custom_function;
} modbus_rtu_slave_cb_t;

// ------------ Slave config ------------
typedef struct {
    uint8_t unit_id;             // 1..247
    int inter_frame_timeout_us;
    int rx_poll_delay_ms;
    int txrx_turnaround_us;      // for manual DE/RE
    size_t max_adu_size;         // default 256
} modbus_rtu_slave_config_t;

// ------------ Create/destroy ------------
esp_err_t modbus_rtu_master_create(const modbus_rtu_uart_config_t *uart_cfg,
                                  const modbus_rtu_master_config_t *master_cfg,
                                  modbus_rtu_t **out);

esp_err_t modbus_rtu_slave_create(const modbus_rtu_uart_config_t *uart_cfg,
                                 const modbus_rtu_slave_config_t *slave_cfg,
                                 const modbus_rtu_slave_cb_t *callbacks,
                                 void *user_ctx,
                                 modbus_rtu_t **out);

esp_err_t modbus_rtu_slave_start(modbus_rtu_t *mb);
esp_err_t modbus_rtu_slave_stop(modbus_rtu_t *mb);

void modbus_rtu_destroy(modbus_rtu_t *mb);

// ------------ Master helpers ------------
esp_err_t modbus_rtu_read_coils(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, uint16_t qty,
                               uint8_t *out_bits, size_t out_bits_len, modbus_rtu_exception_t *ex);

esp_err_t modbus_rtu_read_discrete_inputs(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, uint16_t qty,
                                         uint8_t *out_bits, size_t out_bits_len, modbus_rtu_exception_t *ex);

esp_err_t modbus_rtu_read_holding_registers(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, uint16_t qty,
                                           uint16_t *out_regs, size_t out_regs_len, modbus_rtu_exception_t *ex);

esp_err_t modbus_rtu_read_input_registers(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, uint16_t qty,
                                         uint16_t *out_regs, size_t out_regs_len, modbus_rtu_exception_t *ex);

esp_err_t modbus_rtu_write_single_coil(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, bool on,
                                      modbus_rtu_exception_t *ex);

esp_err_t modbus_rtu_write_single_register(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, uint16_t value,
                                          modbus_rtu_exception_t *ex);

esp_err_t modbus_rtu_write_multiple_coils(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, uint16_t qty,
                                         const uint8_t *bits, size_t bits_len, modbus_rtu_exception_t *ex);

esp_err_t modbus_rtu_write_multiple_registers(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, uint16_t qty,
                                             const uint16_t *regs, size_t regs_len, modbus_rtu_exception_t *ex);

esp_err_t modbus_rtu_mask_write_register(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr,
                                        uint16_t and_mask, uint16_t or_mask,
                                        modbus_rtu_exception_t *ex);

esp_err_t modbus_rtu_readwrite_multiple_registers(modbus_rtu_t *mb, uint8_t unit_id,
                                                 uint16_t read_addr, uint16_t read_qty,
                                                 uint16_t write_addr, uint16_t write_qty,
                                                 const uint16_t *write_regs, size_t write_regs_len,
                                                 uint16_t *out_read_regs, size_t out_read_regs_len,
                                                 modbus_rtu_exception_t *ex);

// Low-level transaction: request PDU in, response PDU out (no unit-id/CRC)
esp_err_t modbus_rtu_master_transaction(modbus_rtu_t *mb, uint8_t unit_id,
                                       const uint8_t *request_pdu, size_t request_pdu_len,
                                       uint8_t *response_pdu, size_t response_pdu_max, size_t *response_pdu_len,
                                       modbus_rtu_exception_t *ex);

// ------------ Bit helpers ------------
size_t modbus_rtu_bits_pack(const uint8_t *src_bits, size_t bit_count, uint8_t *out_bytes, size_t out_len);
size_t modbus_rtu_bits_unpack(const uint8_t *src_bytes, size_t byte_count, uint8_t *out_bits, size_t out_bits_len);

// ------------ CRC ------------
uint16_t modbus_rtu_crc16(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
