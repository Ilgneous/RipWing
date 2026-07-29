//! Inference wrapper for the quantized anomaly-detection model.
//!
//! Populated in the edge-deployment phase. Options on Cortex-M:
//!   - FFI into TFLite-Micro's C++ runtime, or
//!   - a Rust-native crate (e.g. `tract`) if it fits flash/RAM.
//! Runs at lower priority than the control loop; target >50 Hz.
