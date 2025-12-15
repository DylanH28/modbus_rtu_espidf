idf_component_register(
    SRCS
        "src/modbus_rtu.c"
        "src/modbus_rtu_crc.c"
        "src/modbus_rtu_port_uart.c"
        "src/modbus_rtu_bits.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer freertos
)
