# Hardware Notes

## Current target
STM32F411CEUx (BlackPill dev board) for firmware bring-up.
Migration target: STM32H743 for ML compute headroom + custom PCB.

## Pinout
| Function        | Pin  | Peripheral | Notes            |
|-----------------|------|------------|------------------|
| Status LED      | PC13 | GPIO       | Active low on BlackPill |
| IMU SCK         | PA5  | SPI1 (AF5) | ICM-42688-P, 12 MHz, mode 0 |
| IMU MISO        | PA6  | SPI1 (AF5) |                  |
| IMU MOSI        | PA7  | SPI1 (AF5) |                  |
| IMU CS          | PA4  | GPIO       | Active low, driven by firmware |
| IMU INT1 (DRDY) | PA1  | EXTI1      | Rising edge; paces the control chain |
| IMU DMA         | —    | DMA2 S0/S3 ch3 | SPI1 RX / TX     |
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
