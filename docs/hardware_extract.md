# Hardware Extract

**Language / 语言：** **English** | [简体中文](./hardware_extract.zh-CN.md)

## Sources

- Schematic: `Circuit/ESP-drone_V1.3.pdf`
- user-fixed constraints from the implementation brief

## Locked Board Mapping

| Signal | GPIO / Interface | Role | Enabled | Source |
|---|---|---|---|---|
| `MOTOR1` | `IO5` | Brushed motor output `M1` | Yes | User-fixed + schematic net |
| `MOTOR2` | `IO6` | Brushed motor output `M2` | Yes | User-fixed + schematic net |
| `MOTOR3` | `IO3` | Brushed motor output `M3` | Yes | User-fixed + schematic net |
| `MOTOR4` | `IO4` | Brushed motor output `M4` | Yes | User-fixed + schematic net |
| `LED_G` | `IO46` | Green status LED | Yes | User-fixed + schematic net |
| `LED_R` | `IO8` | Red status LED | Yes | User-fixed + schematic net |
| `LED_Y` | `IO7` | Yellow status LED | Yes | User-fixed + schematic net |
| `BAT_ADC` | `IO2` | Battery voltage ADC input | Yes | User-fixed + schematic net |
| `U0TXD/U0RXD` | `UART0` | IMU UART to `ATK-MS901M` | Yes | User-fixed + schematic symbol |
| Camera reserved nets | Various | Camera expansion only | No | User instruction |

## Electrical Notes

- `BAT_ADC` uses a `100k / 100k` divider, so the divider ratio is `2.0`.
- `UART0` is reserved for the IMU. It must not be reused for console or debug text.
- `USB CDC` is reserved for the host console, CLI protocol, and debug event stream.
- camera-related nets are ignored by the current firmware generation

## Board-Level Rules

- GPIO definitions must live only in `main/board/board_config.h` and `main/board/board_config.c`.
- no business-logic module is allowed to hardcode GPIO numbers
- motor identity, motor physical position, and motor spin direction are separate concepts and must not be conflated
