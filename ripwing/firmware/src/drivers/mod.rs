//! Hardware device drivers.
//!
//! * [`imu`] — ICM-42688-P (IMU1): data-ready + DMA, paces the control chain.
//! * [`sensors`] — BMI088 (IMU2), BMP581 baro, MMC5983MA mag: polled. The
//!   chip drivers themselves live in the host-tested `ripwing-sensors` crate.

pub mod imu;
pub mod sensors;
