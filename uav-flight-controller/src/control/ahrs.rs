//! Attitude and heading reference system.
//!
//! Fuses gyro + accel (+ mag) into an orientation estimate.
//! Madgwick or Mahony are good no_std starting points; a
//! complementary filter is even simpler to bring up first.
