# Hardware Notes

## Current target
STM32F411CEUx (BlackPill dev board) for firmware bring-up.
Migration target: the custom PCB in `Hardware/RipWing`, which uses an
STM32H723VGTx (the schematic's MCU). The secondary-sensor pins below follow
the PCB wherever the F411 has the same pin.

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
| IMU2 SCK        | PB13 | SPI2 (AF5) | BMI088, 6 MHz, mode 0 (PCB: PB13) |
| IMU2 MISO       | PB14 | SPI2 (AF5) | SDO1 + SDO2; internal pull-up (PCB: PB14) |
| IMU2 MOSI       | PB15 | SPI2 (AF5) | (PCB: PB15) |
| IMU2 accel CS   | PB12 | GPIO       | CSB1, active low (PCB: PB12) |
| IMU2 gyro CS    | PA8  | GPIO       | CSB2, active low (PCB: PD10; no port D on the F411) |
| IMU2 INT1/INT3  | —    | —          | Not connected; polled at 500 Hz (PCB: PD11 / PD13) |
| I2C1 SCL        | PB8  | I2C1 (AF4) | 400 kHz, open drain, internal pull-up (PCB: PB8, 2.2k) |
| I2C1 SDA        | PB9  | I2C1 (AF4) | (PCB: PB9, 2.2k) |
| Barometer       | I2C1 | 0x46       | BMP581, CSB=VDD, SDO=GND; polled at 50 Hz (PCB INT: PB1) |
| Magnetometer    | I2C1 | 0x30       | MMC5983MA, CS=VDD; polled at 100 Hz (PCB INT: PB0) |
| Battery voltage | PB0  | ADC1_IN8   | Pack via 10k/1k divider (÷11), 100 nF at the pin (PCB: PC0) |
| Battery current | PB1  | ADC1_IN9   | ESC current pin via 1k, 100 nF (PCB: PC1); scale per ESC |
| ESC 1..4        | TBD  | TIM (PWM)  | DShot/OneShot TBD |
| SD / flash log  | TBD  | SPI/SDIO   |                  |

## Clocking
- HSE: 25 MHz crystal (BlackPill). SYSCLK 96 MHz.
- Nucleo/Discovery boards commonly use 8 MHz HSE — adjust `use_hse()` in init.

## Secondary sensors
The BMI088, BMP581 and MMC5983MA are optional at boot: each is probed by
chip ID and skipped with a warning if absent, so a BlackPill without the
breakouts boots normally. The chip drivers are in `crates/sensors` (host
tested against register-level fakes); the firmware glue, bus recovery and
health tracking are in `firmware/src/drivers/sensors.rs`.

PCB notes from the schematic:
- IMU2's supply (`+3V3_IMU2`, TPS7A2033 U11) has its enable on PE8 with a
  100k pull-up, so it is on by default. Firmware can power-cycle it to
  recover a hung BMI088 once the H723 port exists. IMU1 has the same on PE7.
- The magnetometer uses KiCad's MMC5883MA symbol with the value MMC5983MA.
  Check the footprint and pinout against the MMC5983MA datasheet before
  ordering boards.
- Sensor axis maps (`IMU2_AXIS`, `MAG_AXIS` in `board.rs`) are identity
  until verified on the mounted board.

## Open decisions (see architecture register)
- ESC protocol: DShot vs OneShot
- IMU redundancy: mechanical isolation approach
- Log storage: SD (SDIO) vs onboard SPI flash
