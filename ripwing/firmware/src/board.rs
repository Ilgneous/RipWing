//! Board-specific pin assignments and peripheral aliases.
//!
//! Centralize all pin choices here. When you move from a dev board
//! to your custom PCB, this is the only file that should change.

// Example (fill in as you wire peripherals):
//
// use stm32f4xx_hal::gpio::{Pin, Alternate};
//
// pub type ImuSpi = ...;   // SPI bus the IMU sits on
// pub type BaroI2c = ...;  // I2C bus the barometer sits on
//
// pub const MOTOR_PWM_FREQ_HZ: u32 = 480; // ESC PWM / DShot config
