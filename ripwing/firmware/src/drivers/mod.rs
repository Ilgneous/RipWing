//! Hardware device drivers.
//!
//! * [`imu`] — ICM-42688-P (IMU1): data-ready + DMA, paces the control chain.
//! * [`sensors`] — BMI088 (IMU2), BMP581 baro, MMC5983MA mag: polled. The
//!   chip drivers themselves live in the host-tested `ripwing-sensors` crate.
//! * [`power`] — battery voltage and current on ADC1, read by the safety
//!   monitor itself.

pub mod imu;
pub mod power;
pub mod sensors;
