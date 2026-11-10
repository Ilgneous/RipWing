//! Minimal unit quaternion — only what the estimators need.
//!
//! Kept local rather than pulling in nalgebra: the estimator needs four
//! operations, and having them spelled out here makes the filter readable
//! line by line against the math, which is the point of starting with a
//! complementary filter at all.

/// Rotation quaternion `w + xi + yj + zk`, representing the rotation from
/// the body frame to the reference (level) frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quat {
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// From ZYX Euler angles [rad].
    pub fn from_euler(roll: f32, pitch: f32, yaw: f32) -> Self {
        let (sr, cr) = (libm::sinf(roll * 0.5), libm::cosf(roll * 0.5));
        let (sp, cp) = (libm::sinf(pitch * 0.5), libm::cosf(pitch * 0.5));
        let (sy, cy) = (libm::sinf(yaw * 0.5), libm::cosf(yaw * 0.5));
        Self {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    /// To ZYX Euler angles `[roll, pitch, yaw]` [rad].
    ///
    /// Pitch is clamped at ±90° (where roll and yaw become degenerate). The
    /// safety monitor trips at ~80° tilt, so the vehicle should never be
    /// flown near the singularity; the clamp only keeps `asin` in-domain
    /// when rounding pushes its argument a hair past 1.
    pub fn to_euler(&self) -> [f32; 3] {
        let Self { w, x, y, z } = *self;
        let roll = libm::atan2f(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        let sinp = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0);
        let pitch = libm::asinf(sinp);
        let yaw = libm::atan2f(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        [roll, pitch, yaw]
    }

    /// The reference-frame "up" axis expressed in the body frame: the third
    /// row of the body-to-reference rotation matrix. This is the direction
    /// the accelerometer *should* point if the attitude estimate is right.
    #[inline]
    pub fn up_in_body(&self) -> [f32; 3] {
        let Self { w, x, y, z } = *self;
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ]
    }

    /// Advance by a body-frame angular rate `omega` [rad/s] over `dt` [s]:
    /// first-order integration of `q̇ = ½ q ⊗ (0, ω)`, then renormalize.
    ///
    /// First order is plenty at 1 kHz: per-step rotation is at most a few
    /// milliradians, so the truncation error is far below sensor noise, and
    /// the renormalization stops it from accumulating as scale drift.
    #[inline]
    pub fn integrate(&mut self, omega: [f32; 3], dt: f32) {
        let [gx, gy, gz] = omega;
        let h = 0.5 * dt;
        let Self { w, x, y, z } = *self;
        self.w += h * (-x * gx - y * gy - z * gz);
        self.x += h * (w * gx + y * gz - z * gy);
        self.y += h * (w * gy - x * gz + z * gx);
        self.z += h * (w * gz + x * gy - y * gx);
        self.normalize();
    }

    pub fn norm(&self) -> f32 {
        libm::sqrtf(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
    }

    /// Scale back to unit length. A degenerate (zero or non-finite)
    /// quaternion resets to identity rather than propagating NaN into every
    /// downstream consumer.
    pub fn normalize(&mut self) {
        let n = self.norm();
        if n.is_finite() && n > 1e-6 {
            let inv = 1.0 / n;
            self.w *= inv;
            self.x *= inv;
            self.y *= inv;
            self.z *= inv;
        } else {
            *self = Self::IDENTITY;
        }
    }
}

impl Default for Quat {
    fn default() -> Self {
        Self::IDENTITY
    }
}
