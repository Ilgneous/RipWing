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
// Unassigned (fill in as hardware is chosen):
//   IMU        SPI or I2C + data-ready interrupt pin
//   Barometer  I2C
//   ESC 1..4   TIM channels for DShot
//   RC link    UART
//   Log store  SPI/SDIO

/// Onboard LED is active low on the WeAct BlackPill: driving the pin LOW
/// turns the LED ON. `toggle()` works regardless, but any code that sets an
/// explicit level needs to know this.
pub const LED_ACTIVE_LOW: bool = true;

// ---- Instrumentation ------------------------------------------------------
//
// When you wire scope pins, define their types here so the rest of the
// firmware refers to them by role rather than by pin number, e.g.:
//
//   use stm32f4xx_hal::gpio::{Output, PushPull, PB0, PB1};
//   pub type DbgControlPin = PB0<Output<PushPull>>;
//   pub type DbgSamplePin  = PB1<Output<PushPull>>;
