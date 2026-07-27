# Hardware Notes

## Current target
STM32F411CEUx (BlackPill dev board) for firmware bring-up.
Migration target: STM32H743 for ML compute headroom + custom PCB.

## Pinout
| Function        | Pin  | Peripheral | Notes            |
|-----------------|------|------------|------------------|
| Status LED      | PC13 | GPIO       | Active low on BlackPill |
| IMU             | TBD  | SPI/I2C    |                  |
| Barometer       | TBD  | I2C        |                  |
| ESC 1..4        | TBD  | TIM (PWM)  | DShot/OneShot TBD |
| SD / flash log  | TBD  | SPI/SDIO   |                  |

## Clocking
- HSE: 25 MHz crystal (BlackPill). SYSCLK 96 MHz.
- Nucleo/Discovery boards commonly use 8 MHz HSE — adjust `use_hse()` in init.

## Open decisions (see architecture register)
- ESC protocol: DShot vs OneShot
- IMU redundancy: mechanical isolation approach
- Log storage: SD (SDIO) vs onboard SPI flash
