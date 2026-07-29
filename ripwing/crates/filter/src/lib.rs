//! Sensor signal conditioning for RipWing (case study §6.4).
//!
//! A two-stage filter on the sensor path: a nonlinear median-of-3 to discard
//! outliers, then a linear `N`-tap moving average to reduce broadband noise.
//! `no_std`, no HAL, no allocation, constant execution time — safe on the
//! hard real-time path and fully testable on the host.
//!
//! ```ignore
//! use ripwing_filter::ImuFilter;
//!
//! // Gyro: short window, it feeds the lag-sensitive rate loop.
//! // Accel: longer window, slower signal and noisier under vibration.
//! let mut filt: ImuFilter<4, 8> = ImuFilter::new();
//!
//! let filtered = filt.update(&raw);   // raw is borrowed, not consumed
//! log(&raw);                          // detector wants the spikes
//! fuse(&filtered);                    // control wants the clean signal
//! ```
//!
//! Window length is the tunable. See [`average`] for the lag-versus-noise
//! table; pick the smallest window that makes the noise tolerable, because
//! delay inside a feedback loop is phase margin spent.

#![no_std]

pub mod average;
pub mod median;
pub mod sensor;

pub use average::{group_delay_samples, group_delay_seconds, MovingAverage};
pub use median::{median3, MedianOf3};
pub use sensor::{ImuFilter, ScalarFilter, Vec3Filter};

#[cfg(test)]
mod tests;
