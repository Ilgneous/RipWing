//! Motor mixing for a quad-X frame.
//!
//! Maps (thrust, roll, pitch, yaw) control demands to four motor throttles.
//! The signs encode the X geometry and prop-spin directions; verify them
//! against your actual motor layout before the first powered test.
//!
//! Frame: body FLU (x forward, y left, z up) — the same frame the IMU
//! mapping, the attitude estimator and QuadSim use. A positive demand on an
//! axis produces a positive torque about that axis (right-hand rule):
//!
//!   +roll   about +x: left side up, right side down
//!   +pitch  about +y: nose DOWN (rear motors up)
//!   +yaw    about +z: nose LEFT (counter-clockwise seen from above)
//!
//! Note this is not the Betaflight convention (FRD, where +pitch is nose up
//! and +yaw is nose right). Mixing FLU rates through FRD signs gives
//! positive feedback on pitch and yaw; the QuadSim plant tests in
//! `tests/quadsim_plant_port.rs` pin these signs down.
//!
//! Motor index / position (viewed from above, front at the top):
//!
//! ```text
//!     3 (FL,CW)   0 (FR,CCW)
//!            \   /
//!             \ /
//!             / \
//!            /   \
//!     2 (RL,CCW)  1 (RR,CW)
//! ```
//!
//! Yaw comes from rotor drag: a prop spinning CW (seen from above) pushes
//! the airframe CCW, so +yaw speeds up the CW pair (1, 3) and slows the CCW
//! pair (0, 2). Motor numbering is the conventional Betaflight-style quad-X
//! order; adjust to match how your ESCs are wired.
//!
//! Units: the mixer is linear, so it is exact if throttle is proportional
//! to rotor *thrust* (ω²/ω_max²). The QuadSim tests model the ESC that way,
//! which turns the simulator's torque-domain gains into these units by a
//! single scale factor per axis. A real ESC's throttle→thrust curve is not
//! linear; linearizing it belongs at the DShot boundary, not here.

use ripwing_common::MotorCommand;

/// Mix control demands into four normalized [0.0, 1.0] motor throttles.
pub fn mix_quad_x(thrust: f32, roll: f32, pitch: f32, yaw: f32) -> MotorCommand {
    // Each motor gets base thrust plus/minus each axis contribution.
    let mut m = [
        thrust - roll - pitch - yaw, // 0 FR CCW
        thrust - roll + pitch + yaw, // 1 RR CW
        thrust + roll + pitch - yaw, // 2 RL CCW
        thrust + roll - pitch + yaw, // 3 FL CW
    ];

    // Saturate to the valid throttle range. A more sophisticated mixer
    // rescales all four to preserve control authority when one saturates;
    // that is a later refinement. For now, clamp each independently.
    for v in m.iter_mut() {
        *v = v.clamp(0.0, 1.0);
    }

    MotorCommand { throttle: m }
}
