# modbus_rtu_espidf

A small Modbus RTU **master + slave** library as an ESP-IDF component.

- ESP-IDF v5.x compatible
- RTU framing + CRC16 + exceptions
- Thread-safe master transactions (mutex)
- UART RS-485 half-duplex mode OR manual DE/RE GPIO
- Slave engine with callbacks for coils/registers + custom function hook

## Supported function codes

0x01 0x02 0x03 0x04 0x05 0x06 0x0F 0x10 0x16 0x17

## Use

Copy `components/modbus_rtu` into your project’s `components/`.

Or use this repository as-is and build examples:
- `examples/master_simple`
- `examples/slave_simple`
