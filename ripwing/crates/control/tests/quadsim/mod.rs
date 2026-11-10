//! Shared support for the QuadSim cross-check tests.
//!
//! * [`plant`]   — Rust port of QuadSim's 10-DOF rotational dynamics.
//! * [`fixture`] — loader for the Python reference trajectories.
//! * [`harness`] — closed loop: Rust controller + mixer driving the plant.
//!
//! The reference data is produced by
//! `QuadSim/analysis/export_step_references.py`; regenerate it (and re-run
//! these tests) whenever the Python plant, controller or tuned gains change.

// Each integration-test binary uses a different subset of this module.
#![allow(dead_code)]

pub mod fixture;
pub mod harness;
pub mod plant;
