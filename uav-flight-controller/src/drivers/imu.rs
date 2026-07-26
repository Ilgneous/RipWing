//! IMU driver (e.g., MPU6000 / ICM-42688-P).
//!
//! Responsible for raw gyro + accel sample acquisition. Dual-redundant
//! IMUs are a project goal — keep the interface generic so two instances
//! can run side by side and be cross-checked.
