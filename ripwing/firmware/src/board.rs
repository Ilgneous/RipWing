//! Board-specific pin assignments and hardware constants.
//!
//! Centralize all pin choices here. When you move from the BlackPill dev
//! board to the custom PCB, this is the only file that should change.
//!
//! Current target: WeAct BlackPill, STM32F411CEU6.

// ---- Clock ----------------------------------------------------------------

/// External crystal frequency on the WeAct BlackPill [MHz].
///
/// IMPORTANT: if the board hangs at boot with no defmt output and no LED,
/// this value is the first suspect — `.freeze()` blocks forever waiting for
/// an HSE that never stabilizes. Nucleo/Discovery boards commonly use 8 MHz.
/// To rule the crystal out entirely, switch `init` to the internal
/// oscillator (drop `use_hse(...)` and keep `.sysclk(...)`).
pub const HSE_FREQ_MHZ: u32 = 25;

/// Target system clock [MHz]. The F411 tops out at 100 MHz; 96 divides
/// cleanly for USB later.
pub const SYSCLK_MHZ: u32 = 96;

// ---- Pin map (BlackPill) --------------------------------------------------
//
// Assigned:
//   PC13  onboard LED (ACTIVE LOW — pin low lights the LED)
//   PA13  SWDIO  } debug probe, do not reuse
//   PA14  SWCLK  }
//
// Reserved for bring-up instrumentation (§4.7). Pick any free GPIO and
// scope them to measure task period and execution time directly:
//   dbg_control  toggled high for the duration of the control loop body
//   dbg_sample   toggled on each IMU sample
//
// Suggested free pins on the BlackPill (not used by anything above):
//   PB0, PB1, PB12, PB13 — all broken out on the headers.
//
// IMU — ICM-42688-P breakout on SPI1:
//   PA5   SPI1_SCK   (AF5)
//   PA6   SPI1_MISO  (AF5)
//   PA7   SPI1_MOSI  (AF5)
//   PA4   IMU CS     GPIO, active low (driven manually, not hardware NSS)
//   PA1   IMU INT1   data-ready, EXTI1 rising edge. PA1 because EXTI0..4
//                    each have a dedicated vector (no shared-handler
//                    demux on the hot path) and PA0 is the BlackPill KEY
//                    button.
//   DMA2 Stream0 ch3 = SPI1_RX, DMA2 Stream3 ch3 = SPI1_TX
//
// Secondary sensors, on breakouts. Same buses and pins as the H723 PCB
// (Hardware/RipWing) wherever the F411 has them, so the wiring carries over:
//
//   BMI088 (IMU2) on SPI2, PS tied low (SPI mode):
//   PB13  SPI2_SCK   (AF5)            PCB: PB13
//   PB14  SPI2_MISO  (AF5)            PCB: PB14  (SDO1 and SDO2 together)
//   PB15  SPI2_MOSI  (AF5)            PCB: PB15
//   PB12  accel CS (CSB1), active low PCB: PB12
//   PA8   gyro CS (CSB2), active low  PCB: PD10 (no port D on the F411)
//   INT1 (accel) / INT3 (gyro) data-ready: not connected here, the
//   firmware polls. PCB: PD11 / PD13.
//
//   BMP581 + MMC5983MA on I2C1, 400 kHz:
//   PB8   I2C1_SCL   (AF4, open drain) PCB: PB8, 2.2k pull-up
//   PB9   I2C1_SDA   (AF4, open drain) PCB: PB9, 2.2k pull-up
//   BMP581: CSB to VDD (I²C), SDO to GND -> address 0x46.
//   MMC5983MA: CS to VDD (I²C), address 0x30.
//   The internal ~40k pull-ups are enabled so a missing breakout reads as
//   a NACK instead of a floating bus; breakouts usually add their own.
//   BARO_INT / MAG_INT: not connected here (PCB: PB1 / PB0). PB1 would
//   share EXTI line 1 with the ICM's data-ready on PA1, which is one more
//   reason these slow sensors are polled.
//
// Unassigned (fill in as hardware is chosen):
//   ESC 1..4   TIM channels for DShot
//   RC link    UART
//   Log store  SPI/SDIO

/// Onboard LED is active low on the WeAct BlackPill: driving the pin LOW
/// turns the LED ON. `toggle()` works regardless, but any code that sets an
/// explicit level needs to know this.
pub const LED_ACTIVE_LOW: bool = true;

// ---- IMU ------------------------------------------------------------------

use stm32f4xx_hal::gpio::{Input, Output, PushPull, PA1, PA4};

/// IMU chip select (active low).
pub type ImuCsPin = PA4<Output<PushPull>>;
/// IMU INT1, configured by the driver as data-ready.
pub type ImuDrdyPin = PA1<Input>;

/// SPI clock for the IMU [MHz]. The ICM-42688-P allows 24 MHz; 12 MHz
/// leaves margin for flying-lead wiring to a breakout and still moves a
/// full 18-byte frame in 12 µs. Raise on the custom PCB.
pub const IMU_SPI_MHZ: u32 = 12;

/// Rotation from IMU chip axes to the vehicle body frame, as a signed axis
/// permutation: body[i] = sign[i] * chip[axis[i]]. Identity means the chip
/// is mounted with its x forward and z up. Set this when the IMU is mounted
/// on the airframe — getting it wrong flips the sign of a feedback loop.
pub const IMU_AXIS: [usize; 3] = [0, 1, 2];
pub const IMU_SIGN: [f32; 3] = [1.0, 1.0, 1.0];

// ---- Secondary sensors ----------------------------------------------------

use stm32f4xx_hal::gpio::{PA8, PB12};

/// BMI088 accelerometer chip select (CSB1, active low).
pub type Imu2AccelCsPin = PB12<Output<PushPull>>;
/// BMI088 gyroscope chip select (CSB2, active low).
pub type Imu2GyroCsPin = PA8<Output<PushPull>>;

/// SPI2 clock for the BMI088 [MHz]. The chip allows 10 MHz, but SPI2 runs
/// from the 48 MHz APB1 clock and the HAL rounds the divider to the nearest
/// power of two: asking for 10 MHz gives 48/4 = 12 MHz, over the limit.
/// 6 MHz divides exactly (48/8).
pub const IMU2_SPI_MHZ: u32 = 6;

/// I2C1 clock for the barometer and magnetometer [kHz].
pub const I2C1_KHZ: u32 = 400;

/// BMI088 chip axes to body frame (see `IMU_AXIS`). Identity until the
/// board is mounted; on the PCB the BMI088 and ICM-42688-P may not share
/// an orientation, so check both against a known rotation.
pub const IMU2_AXIS: [usize; 3] = [0, 1, 2];
pub const IMU2_SIGN: [f32; 3] = [1.0, 1.0, 1.0];

/// MMC5983MA chip axes to body frame. Identity until verified: rotate the
/// board through north and check the heading the field implies.
pub const MAG_AXIS: [usize; 3] = [0, 1, 2];
pub const MAG_SIGN: [f32; 3] = [1.0, 1.0, 1.0];

// ---- Instrumentation ------------------------------------------------------
//
// When you wire scope pins, define their types here so the rest of the
// firmware refers to them by role rather than by pin number, e.g.:
//
//   use stm32f4xx_hal::gpio::{Output, PushPull, PB0, PB1};
//   pub type DbgControlPin = PB0<Output<PushPull>>;
//   pub type DbgSamplePin  = PB1<Output<PushPull>>;
