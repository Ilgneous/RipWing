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
// Unassigned (fill in as hardware is chosen):
//   Barometer  I2C
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

// ---- Instrumentation ------------------------------------------------------
//
// When you wire scope pins, define their types here so the rest of the
// firmware refers to them by role rather than by pin number, e.g.:
//
//   use stm32f4xx_hal::gpio::{Output, PushPull, PB0, PB1};
//   pub type DbgControlPin = PB0<Output<PushPull>>;
//   pub type DbgSamplePin  = PB1<Output<PushPull>>;
