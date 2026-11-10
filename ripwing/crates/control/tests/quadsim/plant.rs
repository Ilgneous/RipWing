//! Port of QuadSim's rotational dynamics (`QuadcopterPlant10DOF.derivatives`
//! in `QuadSim/models/quad_sim_10dof.py`).
//!
//! Scope: the 10 states that rotation depends on — Euler angles, body
//! rates, rotor speeds. The Python model's other couplings into rotation are
//! blade flapping (needs body velocity) and per-rotor ground effect (needs
//! height). The reference data is generated with both switched off
//! (`flap_coeff = 0`, `ground_effect_model = 'none'`), and with them off
//! translation cannot affect rotation, so this port is exact rather than an
//! approximation of the Python plant.
//!
//! Kept line-for-line close to the Python so the two can be diffed by eye.
//! Rotor sign convention (as in QuadSim): `spin_dir[i]` is the sign of rotor
//! i's drag torque on the airframe, so the rotor spins about −spin_dir[i]·ẑ.
//! Per rotor, yaw torque = s_i·(k_m·ω² + J_r·ω̇), and the net rotor angular
//! momentum is −J_r·Σ s_i·ω_i.
//!
//! f64 throughout, like NumPy, so plant integration error does not mask
//! the f32 controller differences the step-response tests are looking for.
//!
//! Frames: body FLU (x forward, y left, z up); ZYX Euler angles.
//! Motor order is QuadSim's: M0 front-right, M1 front-left, M2 rear-left,
//! M3 rear-right. (The firmware mixer uses a different numbering; the
//! harness maps between them.)

/// Number of states: [phi, theta, psi, p, q, r, w0, w1, w2, w3].
pub const N: usize = 10;
pub type State = [f64; N];

/// Physical parameters (7-inch prop). `Default` mirrors QuadSim's constructor defaults;
/// `plant_params_match_quadsim` checks that against the exported header.
#[derive(Clone, Copy, Debug)]
pub struct PlantParams {
    pub mass: f64,
    pub g: f64,
    pub arm_length: f64,
    pub ixx: f64,
    pub iyy: f64,
    pub izz: f64,
    /// Thrust coefficient [N/(rad/s)²].
    pub k_f: f64,
    /// Drag-torque coefficient [N·m/(rad/s)²].
    pub k_m: f64,
    /// Rotor speed limit [rad/s] (25 000 rpm).
    pub max_omega: f64,
    pub motor_time_constant: f64,
    pub rotor_rate_damp_coeff: f64,
    pub rotor_inertia: f64,
}

impl Default for PlantParams {
    fn default() -> Self {
        Self {
            mass: 1.2,
            g: 9.81,
            arm_length: 0.225,
            ixx: 0.0123,
            iyy: 0.0123,
            izz: 0.0224,
            k_f: 3.10e-6,
            k_m: 3.95e-8,
            max_omega: 25_000.0 * 2.0 * core::f64::consts::PI / 60.0,
            motor_time_constant: 0.035,
            rotor_rate_damp_coeff: 0.005,
            rotor_inertia: 2.75e-5,
        }
    }
}

pub struct Plant {
    pub p: PlantParams,
    /// Motor (x, y) in the body frame, QuadSim order.
    pub motor_pos: [[f64; 2]; 4],
    /// Sign of each rotor's drag torque about body +z (QuadSim `spin_dir`).
    pub spin_dir: [f64; 4],
}

impl Plant {
    pub fn new(p: PlantParams) -> Self {
        let d = p.arm_length / 2f64.sqrt();
        Self {
            p,
            motor_pos: [[d, -d], [d, d], [-d, d], [-d, -d]],
            spin_dir: [-1.0, 1.0, -1.0, 1.0],
        }
    }

    pub fn omega_hover(&self) -> f64 {
        (self.p.mass * self.p.g / (4.0 * self.p.k_f)).sqrt()
    }

    /// Level, at rest, rotors at hover speed (QuadSim `hover_state`).
    pub fn hover_state(&self) -> State {
        let mut s = [0.0; N];
        s[6..10].fill(self.omega_hover());
        s
    }

    /// Per-motor max thrust k_f·ω_max² [N]: the thrust at throttle 1.0.
    pub fn max_motor_thrust(&self) -> f64 {
        self.p.k_f * self.p.max_omega * self.p.max_omega
    }

    /// Time derivative of the state under COMMANDED rotor speeds `cmd`.
    // Indexed loops on purpose: they mirror the per-motor NumPy expressions.
    #[allow(clippy::needless_range_loop)]
    pub fn derivatives(&self, x: &State, cmd: &[f64; 4]) -> State {
        let p = &self.p;
        let (roll, pitch) = (x[0], x[1]);
        let w = [x[3], x[4], x[5]];
        let mut rotor = [0.0; 4];
        let mut c = [0.0; 4];
        for i in 0..4 {
            rotor[i] = x[6 + i].clamp(0.0, p.max_omega);
            c[i] = cmd[i].clamp(0.0, p.max_omega);
        }

        // Rotor thrust and drag torque -> body torque.
        let mut tau = [0.0; 3];
        for i in 0..4 {
            let f = p.k_f * rotor[i] * rotor[i];
            tau[0] += self.motor_pos[i][1] * f;
            tau[1] -= self.motor_pos[i][0] * f;
            tau[2] += self.spin_dir[i] * p.k_m * rotor[i] * rotor[i];
        }

        // Rotor rate damping (tip-path plane lags a rotating hub).
        tau[0] -= p.rotor_rate_damp_coeff * w[0];
        tau[1] -= p.rotor_rate_damp_coeff * w[1];

        // Rotor gyroscopics: -(w x [0, 0, H]), H = -J_r * sum(s_i * w_i)
        // because rotor i spins about -s_i * z.
        let h: f64 = -(0..4).map(|i| self.spin_dir[i] * rotor[i]).sum::<f64>() * p.rotor_inertia;
        tau[0] -= w[1] * h;
        tau[1] += w[0] * h;

        // Motor first-order lag, and the reaction torque from accelerating
        // the rotors: +J_r * sum(s_i * w_dot_i), the same direction as the
        // extra drag torque the new speed will produce.
        let mut rotor_dot = [0.0; 4];
        let mut react = 0.0;
        for i in 0..4 {
            rotor_dot[i] = (c[i] - rotor[i]) / p.motor_time_constant;
            react += self.spin_dir[i] * rotor_dot[i];
        }
        tau[2] += p.rotor_inertia * react;

        // Euler's equation, diagonal inertia: I w_dot = tau - w x (I w).
        let iw = [p.ixx * w[0], p.iyy * w[1], p.izz * w[2]];
        let gyro = [
            w[1] * iw[2] - w[2] * iw[1],
            w[2] * iw[0] - w[0] * iw[2],
            w[0] * iw[1] - w[1] * iw[0],
        ];
        let w_dot = [
            (tau[0] - gyro[0]) / p.ixx,
            (tau[1] - gyro[1]) / p.iyy,
            (tau[2] - gyro[2]) / p.izz,
        ];

        // ZYX Euler kinematics.
        let (sphi, cphi) = roll.sin_cos();
        let (sth, cth) = pitch.sin_cos();
        let tth = sth / cth;
        let euler_dot = [
            w[0] + sphi * tth * w[1] + cphi * tth * w[2],
            cphi * w[1] - sphi * w[2],
            sphi / cth * w[1] + cphi / cth * w[2],
        ];

        let mut d = [0.0; N];
        d[0..3].copy_from_slice(&euler_dot);
        d[3..6].copy_from_slice(&w_dot);
        d[6..10].copy_from_slice(&rotor_dot);
        d
    }

    /// Classical RK4 with zero-order-hold commands (QuadSim `step_rk4`).
    pub fn step_rk4(&self, x: &State, cmd: &[f64; 4], dt: f64) -> State {
        let add = |a: &State, k: &State, h: f64| -> State {
            let mut o = *a;
            for i in 0..N {
                o[i] += h * k[i];
            }
            o
        };
        let k1 = self.derivatives(x, cmd);
        let k2 = self.derivatives(&add(x, &k1, 0.5 * dt), cmd);
        let k3 = self.derivatives(&add(x, &k2, 0.5 * dt), cmd);
        let k4 = self.derivatives(&add(x, &k3, dt), cmd);
        let mut o = *x;
        for i in 0..N {
            o[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
        o
    }
}
