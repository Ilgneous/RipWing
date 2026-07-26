//! Edge anomaly detection subsystem.
//!
//! Observes flight telemetry and flags sensor degradation, motor
//! faults, or aerodynamic anomalies. The ML path MONITORS only —
//! flight-critical actuation stays in the classical control path.

pub mod model;
