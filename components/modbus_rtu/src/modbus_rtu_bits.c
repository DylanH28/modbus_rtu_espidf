#include "modbus_rtu.h"

size_t modbus_rtu_bits_pack(const uint8_t *src_bits, size_t bit_count, uint8_t *out_bytes, size_t out_len)
{
    if (!src_bits || !out_bytes) return 0;
    size_t needed = (bit_count + 7) / 8;
    if (out_len < needed) return 0;

    for (size_t i = 0; i < needed; ++i) out_bytes[i] = 0;

    for (size_t i = 0; i < bit_count; ++i) {
        if (src_bits[i]) out_bytes[i / 8] |= (1u << (i % 8));
    }
    return needed;
}

size_t modbus_rtu_bits_unpack(const uint8_t *src_bytes, size_t byte_count, uint8_t *out_bits, size_t out_bits_len)
{
    if (!src_bytes || !out_bits) return 0;
    size_t bit_count = byte_count * 8;
    if (out_bits_len < bit_count) bit_count = out_bits_len;

    for (size_t i = 0; i < bit_count; ++i) {
        out_bits[i] = (src_bytes[i / 8] >> (i % 8)) & 0x01;
    }
    return bit_count;
}
