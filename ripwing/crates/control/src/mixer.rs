//! Motor mixing for a quad-X frame.
//!
//! Maps (thrust, roll, pitch, yaw) control demands to four motor throttles.
//! The signs encode the X geometry and prop-spin directions; verify them
//! against your actual motor layout before the first powered test.
//!
//! Motor index / position (viewed from above, front = +pitch):
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
//! This is the conventional Betaflight-style quad-X ordering; adjust to
//! match how your ESCs are wired.

use ripwing_common::MotorCommand;

/// Mix control demands into four normalized [0.0, 1.0] motor throttles.
pub fn mix_quad_x(thrust: f32, roll: f32, pitch: f32, yaw: f32) -> MotorCommand {
    // Each motor gets base thrust plus/minus each axis contribution.
    let mut m = [
        thrust - roll + pitch + yaw, // 0 FR CCW
        thrust - roll - pitch - yaw, // 1 RR CW
        thrust + roll - pitch + yaw, // 2 RL CCW
        thrust + roll + pitch - yaw, // 3 FL CW
    ];

    // Saturate to the valid throttle range. A more sophisticated mixer
    // rescales all four to preserve control authority when one saturates;
    // that is a later refinement. For now, clamp each independently.
    for v in m.iter_mut() {
        *v = v.clamp(0.0, 1.0);
    }

    MotorCommand { throttle: m }
}
