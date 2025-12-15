#pragma once
#include "modbus_rtu.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_log.h"

typedef enum { MB_ROLE_MASTER = 1, MB_ROLE_SLAVE = 2 } mb_role_t;

typedef struct {
    uart_port_t uart_num;
    bool rs485_mode;

    gpio_num_t de_re_io;
    bool de_re_active_high;

    int txrx_turnaround_us;
    int inter_frame_timeout_us;
} mb_port_t;

struct modbus_rtu_s {
    mb_role_t role;
    mb_port_t port;

    // master
    modbus_rtu_master_config_t master_cfg;
    SemaphoreHandle_t master_mutex;

    // slave
    modbus_rtu_slave_config_t slave_cfg;
    modbus_rtu_slave_cb_t cb;
    void *user_ctx;
    TaskHandle_t slave_task;
    volatile bool slave_running;
};

#define MB_ADU_MAX_DEFAULT 256
#define MB_RXBUF_DEFAULT   512
#define MB_TXBUF_DEFAULT   256
#define MB_EVTQ_DEFAULT    16

enum {
    MB_FC_READ_COILS              = 0x01,
    MB_FC_READ_DISCRETE_INPUTS    = 0x02,
    MB_FC_READ_HOLDING_REGS       = 0x03,
    MB_FC_READ_INPUT_REGS         = 0x04,
    MB_FC_WRITE_SINGLE_COIL       = 0x05,
    MB_FC_WRITE_SINGLE_REG        = 0x06,
    MB_FC_WRITE_MULTIPLE_COILS    = 0x0F,
    MB_FC_WRITE_MULTIPLE_REGS     = 0x10,
    MB_FC_MASK_WRITE_REG          = 0x16,
    MB_FC_READWRITE_MULTIPLE_REGS = 0x17,
};

enum {
    MB_EX_ILLEGAL_FUNCTION    = 0x01,
    MB_EX_ILLEGAL_DATA_ADDR   = 0x02,
    MB_EX_ILLEGAL_DATA_VALUE  = 0x03,
    MB_EX_SLAVE_DEVICE_FAIL   = 0x04,
};

static inline int64_t mb_time_us(void) { return esp_timer_get_time(); }

esp_err_t mb_port_init(mb_port_t *p, const modbus_rtu_uart_config_t *uart_cfg,
                       int inter_frame_timeout_us, int txrx_turnaround_us);

void      mb_port_deinit(mb_port_t *p);

esp_err_t mb_port_write_adu(mb_port_t *p, const uint8_t *adu, size_t adu_len);

esp_err_t mb_port_read_frame(mb_port_t *p, uint8_t *buf, size_t buf_len,
                             size_t *out_len, int overall_timeout_ms);
