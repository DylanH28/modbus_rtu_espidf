#include "modbus_rtu_internal.h"

#ifndef CONFIG_MODBUS_RTU_LOG_LEVEL
#define CONFIG_MODBUS_RTU_LOG_LEVEL 3
#endif

#if CONFIG_MODBUS_RTU_LOG_LEVEL >= 4
#define MB_LOGD(...) ESP_LOGD(__VA_ARGS__)
#else
#define MB_LOGD(...)
#endif
#if CONFIG_MODBUS_RTU_LOG_LEVEL >= 3
#define MB_LOGI(...) ESP_LOGI(__VA_ARGS__)
#else
#define MB_LOGI(...)
#endif
#if CONFIG_MODBUS_RTU_LOG_LEVEL >= 2
#define MB_LOGW(...) ESP_LOGW(__VA_ARGS__)
#else
#define MB_LOGW(...)
#endif
#if CONFIG_MODBUS_RTU_LOG_LEVEL >= 1
#define MB_LOGE(...) ESP_LOGE(__VA_ARGS__)
#else
#define MB_LOGE(...)
#endif

static const char *TAG = "modbus_rtu";

static inline void put_u16_be(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)(v & 0xFF); }
static inline uint16_t get_u16_be(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

static esp_err_t mb_build_adu(uint8_t unit_id, const uint8_t *pdu, size_t pdu_len,
                             uint8_t *adu, size_t adu_max, size_t *adu_len)
{
    if (!pdu || !adu || !adu_len) return ESP_ERR_INVALID_ARG;
    if (pdu_len + 1 + 2 > adu_max) return ESP_ERR_NO_MEM;

    adu[0] = unit_id;
    memcpy(&adu[1], pdu, pdu_len);

    uint16_t crc = modbus_rtu_crc16(adu, 1 + pdu_len);
    adu[1 + pdu_len + 0] = (uint8_t)(crc & 0xFF); // CRC Lo
    adu[1 + pdu_len + 1] = (uint8_t)(crc >> 8);   // CRC Hi
    *adu_len = 1 + pdu_len + 2;
    return ESP_OK;
}

static esp_err_t mb_parse_and_validate_adu(const uint8_t *adu, size_t adu_len,
                                          uint8_t expected_unit_id,
                                          const uint8_t *req_pdu, size_t req_pdu_len,
                                          const modbus_rtu_master_config_t *mcfg,
                                          uint8_t *out_unit_id,
                                          uint8_t *out_pdu, size_t out_pdu_max, size_t *out_pdu_len,
                                          modbus_rtu_exception_t *ex)
{
    if (!adu || adu_len < 5 || !out_unit_id || !out_pdu || !out_pdu_len) return ESP_ERR_INVALID_ARG;
    *out_pdu_len = 0;

    uint16_t got = (uint16_t)(adu[adu_len - 2] | (adu[adu_len - 1] << 8));
    uint16_t calc = modbus_rtu_crc16(adu, adu_len - 2);
    if (got != calc) return ESP_ERR_MODBUS_RTU_CRC;

    uint8_t unit_id = adu[0];
    uint8_t fc = adu[1];

    if (mcfg && mcfg->strict_unit_id && unit_id != expected_unit_id) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;
    if (mcfg && mcfg->strict_function && req_pdu && req_pdu_len >= 1) {
        if ((fc & 0x7F) != req_pdu[0]) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;
    }

    if (fc & 0x80) {
        if (ex) { ex->function = (uint8_t)(fc & 0x7F); ex->exception_code = adu[2]; }
        return ESP_ERR_MODBUS_RTU_EXCEPTION;
    }

    size_t pdu_len = adu_len - 1 - 2;
    if (pdu_len > out_pdu_max) return ESP_ERR_NO_MEM;

    memcpy(out_pdu, &adu[1], pdu_len);
    *out_pdu_len = pdu_len;
    *out_unit_id = unit_id;
    return ESP_OK;
}

esp_err_t modbus_rtu_master_create(const modbus_rtu_uart_config_t *uart_cfg,
                                  const modbus_rtu_master_config_t *master_cfg,
                                  modbus_rtu_t **out)
{
    if (!uart_cfg || !master_cfg || !out) return ESP_ERR_INVALID_ARG;
    *out = NULL;

    modbus_rtu_t *mb = (modbus_rtu_t*)calloc(1, sizeof(modbus_rtu_t));
    if (!mb) return ESP_ERR_NO_MEM;

    mb->role = MB_ROLE_MASTER;
    mb->master_cfg = *master_cfg;

    if (mb->master_cfg.response_timeout_ms <= 0) mb->master_cfg.response_timeout_ms = 200;
    if (mb->master_cfg.inter_frame_timeout_us <= 0) mb->master_cfg.inter_frame_timeout_us = 2000;
    if (mb->master_cfg.txrx_turnaround_us < 0) mb->master_cfg.txrx_turnaround_us = 0;

    mb->master_mutex = xSemaphoreCreateMutex();
    if (!mb->master_mutex) { free(mb); return ESP_ERR_NO_MEM; }

    esp_err_t err = mb_port_init(&mb->port, uart_cfg, mb->master_cfg.inter_frame_timeout_us,
                                mb->master_cfg.txrx_turnaround_us);
    if (err != ESP_OK) { vSemaphoreDelete(mb->master_mutex); free(mb); return err; }

    *out = mb;
    return ESP_OK;
}

esp_err_t modbus_rtu_slave_create(const modbus_rtu_uart_config_t *uart_cfg,
                                 const modbus_rtu_slave_config_t *slave_cfg,
                                 const modbus_rtu_slave_cb_t *callbacks,
                                 void *user_ctx,
                                 modbus_rtu_t **out)
{
    if (!uart_cfg || !slave_cfg || !callbacks || !out) return ESP_ERR_INVALID_ARG;
    if (slave_cfg->unit_id == 0 || slave_cfg->unit_id > 247) return ESP_ERR_INVALID_ARG;

    *out = NULL;
    modbus_rtu_t *mb = (modbus_rtu_t*)calloc(1, sizeof(modbus_rtu_t));
    if (!mb) return ESP_ERR_NO_MEM;

    mb->role = MB_ROLE_SLAVE;
    mb->slave_cfg = *slave_cfg;
    if (mb->slave_cfg.inter_frame_timeout_us <= 0) mb->slave_cfg.inter_frame_timeout_us = 2000;
    if (mb->slave_cfg.rx_poll_delay_ms <= 0) mb->slave_cfg.rx_poll_delay_ms = 1;
    if (mb->slave_cfg.max_adu_size == 0) mb->slave_cfg.max_adu_size = MB_ADU_MAX_DEFAULT;

    mb->cb = *callbacks;
    mb->user_ctx = user_ctx;

    esp_err_t err = mb_port_init(&mb->port, uart_cfg, mb->slave_cfg.inter_frame_timeout_us,
                                mb->slave_cfg.txrx_turnaround_us);
    if (err != ESP_OK) { free(mb); return err; }

    *out = mb;
    return ESP_OK;
}

void modbus_rtu_destroy(modbus_rtu_t *mb)
{
    if (!mb) return;
    if (mb->role == MB_ROLE_SLAVE) modbus_rtu_slave_stop(mb);
    if (mb->master_mutex) vSemaphoreDelete(mb->master_mutex);
    mb_port_deinit(&mb->port);
    free(mb);
}

esp_err_t modbus_rtu_master_transaction(modbus_rtu_t *mb, uint8_t unit_id,
                                       const uint8_t *request_pdu, size_t request_pdu_len,
                                       uint8_t *response_pdu, size_t response_pdu_max, size_t *response_pdu_len,
                                       modbus_rtu_exception_t *ex)
{
    if (!mb || mb->role != MB_ROLE_MASTER) return ESP_ERR_INVALID_STATE;
    if (!request_pdu || request_pdu_len < 1) return ESP_ERR_INVALID_ARG;
    if (!response_pdu || !response_pdu_len) return ESP_ERR_INVALID_ARG;

    *response_pdu_len = 0;
    if (ex) { ex->function = 0; ex->exception_code = 0; }

    bool expect_response = (unit_id != 0);

    if (xSemaphoreTake(mb->master_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) return ESP_ERR_TIMEOUT;

    uint8_t adu_tx[MB_ADU_MAX_DEFAULT];
    size_t adu_tx_len = 0;
    esp_err_t err = mb_build_adu(unit_id, request_pdu, request_pdu_len, adu_tx, sizeof(adu_tx), &adu_tx_len);
    if (err != ESP_OK) { xSemaphoreGive(mb->master_mutex); return err; }

    err = mb_port_write_adu(&mb->port, adu_tx, adu_tx_len);
    if (err != ESP_OK) { xSemaphoreGive(mb->master_mutex); return err; }

    if (!expect_response) { xSemaphoreGive(mb->master_mutex); return ESP_OK; }

    uint8_t adu_rx[MB_ADU_MAX_DEFAULT];
    size_t adu_rx_len = 0;
    err = mb_port_read_frame(&mb->port, adu_rx, sizeof(adu_rx), &adu_rx_len, mb->master_cfg.response_timeout_ms);
    if (err != ESP_OK) { xSemaphoreGive(mb->master_mutex); return err; }

    uint8_t rx_unit = 0;
    size_t pdu_len = 0;
    err = mb_parse_and_validate_adu(adu_rx, adu_rx_len, unit_id, request_pdu, request_pdu_len,
                                   &mb->master_cfg, &rx_unit, response_pdu, response_pdu_max, &pdu_len, ex);
    if (err == ESP_OK) *response_pdu_len = pdu_len;

    xSemaphoreGive(mb->master_mutex);
    return err;
}

// -------- Master helpers --------
static esp_err_t mb_read_bits(modbus_rtu_t *mb, uint8_t unit_id, uint8_t fc, uint16_t addr, uint16_t qty,
                             uint8_t *out_bits, size_t out_bits_len, modbus_rtu_exception_t *ex)
{
    if (!out_bits) return ESP_ERR_INVALID_ARG;
    if (qty < 1 || qty > 2000) return ESP_ERR_INVALID_ARG;
    if (out_bits_len < qty) return ESP_ERR_INVALID_SIZE;

    uint8_t req[5];
    req[0] = fc;
    put_u16_be(&req[1], addr);
    put_u16_be(&req[3], qty);

    uint8_t rsp[MB_ADU_MAX_DEFAULT];
    size_t rsp_len = 0;
    esp_err_t err = modbus_rtu_master_transaction(mb, unit_id, req, sizeof(req), rsp, sizeof(rsp), &rsp_len, ex);
    if (err != ESP_OK) return err;

    if (rsp_len < 2 || rsp[0] != fc) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;
    uint8_t byte_count = rsp[1];
    if (rsp_len != (size_t)(2 + byte_count)) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;

    modbus_rtu_bits_unpack(&rsp[2], byte_count, out_bits, out_bits_len);
    return ESP_OK;
}

static esp_err_t mb_read_regs(modbus_rtu_t *mb, uint8_t unit_id, uint8_t fc, uint16_t addr, uint16_t qty,
                             uint16_t *out_regs, size_t out_regs_len, modbus_rtu_exception_t *ex)
{
    if (!out_regs) return ESP_ERR_INVALID_ARG;
    if (qty < 1 || qty > 125) return ESP_ERR_INVALID_ARG;
    if (out_regs_len < qty) return ESP_ERR_INVALID_SIZE;

    uint8_t req[5];
    req[0] = fc;
    put_u16_be(&req[1], addr);
    put_u16_be(&req[3], qty);

    uint8_t rsp[MB_ADU_MAX_DEFAULT];
    size_t rsp_len = 0;
    esp_err_t err = modbus_rtu_master_transaction(mb, unit_id, req, sizeof(req), rsp, sizeof(rsp), &rsp_len, ex);
    if (err != ESP_OK) return err;

    if (rsp_len < 2 || rsp[0] != fc) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;
    uint8_t byte_count = rsp[1];
    if (byte_count != (uint8_t)(qty * 2)) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;
    if (rsp_len != (size_t)(2 + byte_count)) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;

    for (uint16_t i = 0; i < qty; ++i) out_regs[i] = get_u16_be(&rsp[2 + i * 2]);
    return ESP_OK;
}

esp_err_t modbus_rtu_read_coils(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, uint16_t qty,
                               uint8_t *out_bits, size_t out_bits_len, modbus_rtu_exception_t *ex)
{
    return mb_read_bits(mb, unit_id, MB_FC_READ_COILS, addr, qty, out_bits, out_bits_len, ex);
}

esp_err_t modbus_rtu_read_discrete_inputs(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, uint16_t qty,
                                         uint8_t *out_bits, size_t out_bits_len, modbus_rtu_exception_t *ex)
{
    return mb_read_bits(mb, unit_id, MB_FC_READ_DISCRETE_INPUTS, addr, qty, out_bits, out_bits_len, ex);
}

esp_err_t modbus_rtu_read_holding_registers(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, uint16_t qty,
                                           uint16_t *out_regs, size_t out_regs_len, modbus_rtu_exception_t *ex)
{
    return mb_read_regs(mb, unit_id, MB_FC_READ_HOLDING_REGS, addr, qty, out_regs, out_regs_len, ex);
}

esp_err_t modbus_rtu_read_input_registers(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, uint16_t qty,
                                         uint16_t *out_regs, size_t out_regs_len, modbus_rtu_exception_t *ex)
{
    return mb_read_regs(mb, unit_id, MB_FC_READ_INPUT_REGS, addr, qty, out_regs, out_regs_len, ex);
}

esp_err_t modbus_rtu_write_single_coil(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, bool on,
                                      modbus_rtu_exception_t *ex)
{
    uint8_t req[5];
    req[0] = MB_FC_WRITE_SINGLE_COIL;
    put_u16_be(&req[1], addr);
    put_u16_be(&req[3], on ? 0xFF00 : 0x0000);

    uint8_t rsp[16];
    size_t rsp_len = 0;
    esp_err_t err = modbus_rtu_master_transaction(mb, unit_id, req, sizeof(req), rsp, sizeof(rsp), &rsp_len, ex);
    if (err != ESP_OK) return err;

    if (rsp_len != sizeof(req) || memcmp(rsp, req, sizeof(req)) != 0) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;
    return ESP_OK;
}

esp_err_t modbus_rtu_write_single_register(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, uint16_t value,
                                          modbus_rtu_exception_t *ex)
{
    uint8_t req[5];
    req[0] = MB_FC_WRITE_SINGLE_REG;
    put_u16_be(&req[1], addr);
    put_u16_be(&req[3], value);

    uint8_t rsp[16];
    size_t rsp_len = 0;
    esp_err_t err = modbus_rtu_master_transaction(mb, unit_id, req, sizeof(req), rsp, sizeof(rsp), &rsp_len, ex);
    if (err != ESP_OK) return err;

    if (rsp_len != sizeof(req) || memcmp(rsp, req, sizeof(req)) != 0) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;
    return ESP_OK;
}

esp_err_t modbus_rtu_write_multiple_coils(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, uint16_t qty,
                                         const uint8_t *bits, size_t bits_len, modbus_rtu_exception_t *ex)
{
    if (!bits) return ESP_ERR_INVALID_ARG;
    if (qty < 1 || qty > 1968) return ESP_ERR_INVALID_ARG;
    if (bits_len < qty) return ESP_ERR_INVALID_SIZE;

    uint8_t packed[256];
    size_t byte_count = modbus_rtu_bits_pack(bits, qty, packed, sizeof(packed));
    if (byte_count == 0) return ESP_ERR_INVALID_SIZE;

    size_t req_len = 6 + byte_count;
    uint8_t req[6 + 256];
    req[0] = MB_FC_WRITE_MULTIPLE_COILS;
    put_u16_be(&req[1], addr);
    put_u16_be(&req[3], qty);
    req[5] = (uint8_t)byte_count;
    memcpy(&req[6], packed, byte_count);

    uint8_t rsp[16];
    size_t rsp_len = 0;
    esp_err_t err = modbus_rtu_master_transaction(mb, unit_id, req, req_len, rsp, sizeof(rsp), &rsp_len, ex);
    if (err != ESP_OK) return err;

    if (rsp_len != 5 || rsp[0] != MB_FC_WRITE_MULTIPLE_COILS) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;
    if (get_u16_be(&rsp[1]) != addr || get_u16_be(&rsp[3]) != qty) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;
    return ESP_OK;
}

esp_err_t modbus_rtu_write_multiple_registers(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr, uint16_t qty,
                                             const uint16_t *regs, size_t regs_len, modbus_rtu_exception_t *ex)
{
    if (!regs) return ESP_ERR_INVALID_ARG;
    if (qty < 1 || qty > 123) return ESP_ERR_INVALID_ARG;
    if (regs_len < qty) return ESP_ERR_INVALID_SIZE;

    size_t byte_count = qty * 2;
    size_t req_len = 6 + byte_count;
    uint8_t req[6 + 246];

    req[0] = MB_FC_WRITE_MULTIPLE_REGS;
    put_u16_be(&req[1], addr);
    put_u16_be(&req[3], qty);
    req[5] = (uint8_t)byte_count;
    for (uint16_t i = 0; i < qty; ++i) put_u16_be(&req[6 + i * 2], regs[i]);

    uint8_t rsp[16];
    size_t rsp_len = 0;
    esp_err_t err = modbus_rtu_master_transaction(mb, unit_id, req, req_len, rsp, sizeof(rsp), &rsp_len, ex);
    if (err != ESP_OK) return err;

    if (rsp_len != 5 || rsp[0] != MB_FC_WRITE_MULTIPLE_REGS) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;
    if (get_u16_be(&rsp[1]) != addr || get_u16_be(&rsp[3]) != qty) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;
    return ESP_OK;
}

esp_err_t modbus_rtu_mask_write_register(modbus_rtu_t *mb, uint8_t unit_id, uint16_t addr,
                                        uint16_t and_mask, uint16_t or_mask,
                                        modbus_rtu_exception_t *ex)
{
    uint8_t req[7];
    req[0] = MB_FC_MASK_WRITE_REG;
    put_u16_be(&req[1], addr);
    put_u16_be(&req[3], and_mask);
    put_u16_be(&req[5], or_mask);

    uint8_t rsp[16];
    size_t rsp_len = 0;
    esp_err_t err = modbus_rtu_master_transaction(mb, unit_id, req, sizeof(req), rsp, sizeof(rsp), &rsp_len, ex);
    if (err != ESP_OK) return err;

    if (rsp_len != sizeof(req) || memcmp(rsp, req, sizeof(req)) != 0) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;
    return ESP_OK;
}

esp_err_t modbus_rtu_readwrite_multiple_registers(modbus_rtu_t *mb, uint8_t unit_id,
                                                 uint16_t read_addr, uint16_t read_qty,
                                                 uint16_t write_addr, uint16_t write_qty,
                                                 const uint16_t *write_regs, size_t write_regs_len,
                                                 uint16_t *out_read_regs, size_t out_read_regs_len,
                                                 modbus_rtu_exception_t *ex)
{
    if (!write_regs || !out_read_regs) return ESP_ERR_INVALID_ARG;
    if (read_qty < 1 || read_qty > 125) return ESP_ERR_INVALID_ARG;
    if (write_qty < 1 || write_qty > 121) return ESP_ERR_INVALID_ARG;
    if (write_regs_len < write_qty || out_read_regs_len < read_qty) return ESP_ERR_INVALID_SIZE;

    size_t write_byte_count = write_qty * 2;
    size_t req_len = 10 + write_byte_count;
    uint8_t req[10 + 242];

    req[0] = MB_FC_READWRITE_MULTIPLE_REGS;
    put_u16_be(&req[1], read_addr);
    put_u16_be(&req[3], read_qty);
    put_u16_be(&req[5], write_addr);
    put_u16_be(&req[7], write_qty);
    req[9] = (uint8_t)write_byte_count;
    for (uint16_t i = 0; i < write_qty; ++i) put_u16_be(&req[10 + i * 2], write_regs[i]);

    uint8_t rsp[MB_ADU_MAX_DEFAULT];
    size_t rsp_len = 0;
    esp_err_t err = modbus_rtu_master_transaction(mb, unit_id, req, req_len, rsp, sizeof(rsp), &rsp_len, ex);
    if (err != ESP_OK) return err;

    if (rsp_len < 2 || rsp[0] != MB_FC_READWRITE_MULTIPLE_REGS) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;
    uint8_t byte_count = rsp[1];
    if (byte_count != (uint8_t)(read_qty * 2) || rsp_len != (size_t)(2 + byte_count)) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;

    for (uint16_t i = 0; i < read_qty; ++i) out_read_regs[i] = get_u16_be(&rsp[2 + i * 2]);
    return ESP_OK;
}

// -------- Slave engine --------
static void mb_build_exception_pdu(uint8_t function, uint8_t ex_code, uint8_t *out_pdu, size_t *out_len)
{
    out_pdu[0] = (uint8_t)(function | 0x80);
    out_pdu[1] = ex_code;
    *out_len = 2;
}

static esp_err_t mb_slave_reply(modbus_rtu_t *mb, uint8_t unit_id, const uint8_t *pdu, size_t pdu_len)
{
    uint8_t adu[MB_ADU_MAX_DEFAULT];
    size_t adu_len = 0;
    esp_err_t err = mb_build_adu(unit_id, pdu, pdu_len, adu, sizeof(adu), &adu_len);
    if (err != ESP_OK) return err;
    return mb_port_write_adu(&mb->port, adu, adu_len);
}

static esp_err_t mb_slave_handle_request(modbus_rtu_t *mb, const uint8_t *adu, size_t adu_len)
{
    if (adu_len < 5) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;

    uint16_t got = (uint16_t)(adu[adu_len - 2] | (adu[adu_len - 1] << 8));
    uint16_t calc = modbus_rtu_crc16(adu, adu_len - 2);
    if (got != calc) return ESP_ERR_MODBUS_RTU_CRC;

    uint8_t unit_id = adu[0];
    if (unit_id == 0) return ESP_OK; // broadcast ignored
    if (unit_id != mb->slave_cfg.unit_id) return ESP_OK;

    const uint8_t *pdu = &adu[1];
    size_t pdu_len = adu_len - 1 - 2;
    if (pdu_len < 1) return ESP_ERR_MODBUS_RTU_BAD_RESPONSE;

    uint8_t fc = pdu[0];
    uint8_t rsp_pdu[MB_ADU_MAX_DEFAULT];
    size_t rsp_len = 0;

    switch (fc) {
        case MB_FC_READ_HOLDING_REGS:
        case MB_FC_READ_INPUT_REGS: {
            if (pdu_len != 5) { mb_build_exception_pdu(fc, MB_EX_ILLEGAL_DATA_VALUE, rsp_pdu, &rsp_len); break; }
            uint16_t addr = get_u16_be(&pdu[1]);
            uint16_t qty  = get_u16_be(&pdu[3]);
            if (qty < 1 || qty > 125) { mb_build_exception_pdu(fc, MB_EX_ILLEGAL_DATA_VALUE, rsp_pdu, &rsp_len); break; }

            uint16_t regs[125];
            esp_err_t cb_err = ESP_ERR_NOT_SUPPORTED;
            if (fc == MB_FC_READ_HOLDING_REGS && mb->cb.read_holding) cb_err = mb->cb.read_holding(addr, qty, regs, mb->user_ctx);
            if (fc == MB_FC_READ_INPUT_REGS   && mb->cb.read_input)   cb_err = mb->cb.read_input(addr, qty, regs, mb->user_ctx);

            if (cb_err == ESP_OK) {
                rsp_pdu[0] = fc;
                rsp_pdu[1] = (uint8_t)(qty * 2);
                for (uint16_t i = 0; i < qty; ++i) put_u16_be(&rsp_pdu[2 + i * 2], regs[i]);
                rsp_len = 2 + qty * 2;
            } else if (cb_err == ESP_ERR_NOT_SUPPORTED) mb_build_exception_pdu(fc, MB_EX_ILLEGAL_FUNCTION, rsp_pdu, &rsp_len);
            else mb_build_exception_pdu(fc, MB_EX_ILLEGAL_DATA_ADDR, rsp_pdu, &rsp_len);
            break;
        }

        default: {
            if (mb->cb.custom_function) {
                size_t out_len = 0;
                esp_err_t cerr = mb->cb.custom_function(unit_id, fc, pdu, pdu_len, rsp_pdu, sizeof(rsp_pdu), &out_len, mb->user_ctx);
                if (cerr == ESP_OK && out_len >= 1) { rsp_len = out_len; break; }
            }
            mb_build_exception_pdu(fc, MB_EX_ILLEGAL_FUNCTION, rsp_pdu, &rsp_len);
            break;
        }
    }

    if (rsp_len) return mb_slave_reply(mb, unit_id, rsp_pdu, rsp_len);
    return ESP_OK;
}

static void mb_slave_task(void *arg)
{
    modbus_rtu_t *mb = (modbus_rtu_t*)arg;
    uint8_t *rx = (uint8_t*)malloc(mb->slave_cfg.max_adu_size);
    if (!rx) { mb->slave_running = false; vTaskDelete(NULL); return; }

    mb->slave_running = true;
    MB_LOGI(TAG, "Slave started (unit_id=%u)", mb->slave_cfg.unit_id);

    while (mb->slave_running) {
        size_t rx_len = 0;
        esp_err_t err = mb_port_read_frame(&mb->port, rx, mb->slave_cfg.max_adu_size, &rx_len, 1000);
        if (err == ESP_OK && rx_len > 0) (void)mb_slave_handle_request(mb, rx, rx_len);
        vTaskDelay(pdMS_TO_TICKS(mb->slave_cfg.rx_poll_delay_ms));
    }

    free(rx);
    mb->slave_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t modbus_rtu_slave_start(modbus_rtu_t *mb)
{
    if (!mb || mb->role != MB_ROLE_SLAVE) return ESP_ERR_INVALID_STATE;
    if (mb->slave_task) return ESP_ERR_INVALID_STATE;

    mb->slave_running = true;
    BaseType_t ok = xTaskCreate(mb_slave_task, "mb_slave", 4096, mb, 10, &mb->slave_task);
    return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t modbus_rtu_slave_stop(modbus_rtu_t *mb)
{
    if (!mb || mb->role != MB_ROLE_SLAVE) return ESP_ERR_INVALID_STATE;
    if (!mb->slave_task) return ESP_OK;
    mb->slave_running = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}
