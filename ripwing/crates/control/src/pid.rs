//! Single-axis PID controller for the rate loop.
//!
//! This is the code that *applies* the gains your Python harness found.
//! It is deliberately more than the textbook `kp*e + ki*∫e + kd*de/dt`,
//! because a controller that flies needs three things a tuning script
//! usually omits:
//!
//!   1. Integral anti-windup. When the output saturates, the integral
//!      must stop accumulating error it cannot act on, or it overshoots
//!      badly on recovery. We clamp the integral term.
//!   2. Derivative filtering. The D term amplifies noise. We low-pass the
//!      derivative with a one-pole filter (the alpha parameter), separate
//!      from the median-then-average that cleans the measurement upstream.
//!   3. Output saturation. The command is clamped to a fixed range so the
//!      mixer always receives a bounded value.
//!
//! Everything here is `f32` to match the F411's single-precision FPU, so
//! host tests exercise the same precision the hardware will.

/// Tunable gains and limits for one PID axis.
///
/// The three gains come straight from your simulator's optimizer. The
/// limits are safety/behaviour bounds you set once for the airframe.
#[derive(Clone, Copy, Debug)]
pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    /// Symmetric clamp on the integral term (anti-windup). Units: same as
    /// output. Set to the output range or a fraction of it.
    pub integral_limit: f32,
    /// Symmetric clamp on the final output.
    pub output_limit: f32,
    /// Derivative low-pass coefficient in [0.0, 1.0].
    ///   1.0 = no filtering (raw derivative, noisy)
    ///   ->0 = heavy filtering (smooth, more lag)
    /// A common starting point is 0.1–0.3.
    pub d_filter_alpha: f32,
}

impl Default for PidConfig {
    fn default() -> Self {
        Self {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            integral_limit: f32::INFINITY,
            output_limit: f32::INFINITY,
            d_filter_alpha: 1.0,
        }
    }
}

/// The mutable running state of one PID axis. Separate from config so the
/// gains stay immutable while the integrator and filter evolve each tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct PidState {
    integral: f32,
    /// Previous *measurement* (not error) — see note in `update`.
    prev_measurement: f32,
    /// Filtered derivative, retained for the one-pole low-pass.
    filtered_derivative: f32,
    initialized: bool,
}

/// A single PID axis: immutable config + evolving state.
#[derive(Clone, Copy, Debug)]
pub struct Pid {
    pub config: PidConfig,
    pub state: PidState,
}

impl Pid {
    pub fn new(config: PidConfig) -> Self {
        Self {
            config,
            state: PidState::default(),
        }
    }

    /// Reset the running state (integrator, derivative memory). Call on
    /// arm, mode change, or any time continuity would be wrong.
    pub fn reset(&mut self) {
        self.state = PidState::default();
    }

    /// Current value of the integral accumulator. Exposed for tests and
    /// telemetry; the controller manages it internally otherwise.
    pub fn state_integral(&self) -> f32 {
        self.state.integral
    }

    /// Advance the controller one step.
    ///
    /// * `setpoint`    — desired value for this axis
    /// * `measurement` — measured value for this axis
    /// * `dt`          — timestep in seconds (e.g. 0.001 for 1 kHz)
    ///
    /// Returns the clamped control output.
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;

        // ---- Proportional ------------------------------------------------
        let p = self.config.kp * error;

        // ---- Integral with anti-windup -----------------------------------
        // Accumulate, then clamp. Clamping *after* accumulation is the
        // simple, robust form: the integrator can never wind past the
        // limit, so recovery from saturation is prompt.
        self.state.integral += error * dt;
        self.state.integral = clamp_sym(self.state.integral, self.config.integral_limit);
        let i = self.config.ki * self.state.integral;

        // ---- Derivative on measurement, low-pass filtered ----------------
        // We differentiate the *measurement*, not the error. Differentiating
        // error causes a huge spike ("derivative kick") whenever the
        // setpoint steps, because d(setpoint)/dt is an impulse. Using the
        // measurement removes that kick while giving identical damping.
        //
        // On the first call we have no previous measurement, so the
        // derivative is defined as zero to avoid a startup transient.
        let raw_derivative = if self.state.initialized {
            // Negative sign: derivative-on-measurement flips the sign vs.
            // derivative-on-error.
            -(measurement - self.state.prev_measurement) / dt
        } else {
            0.0
        };

        // One-pole low-pass: y += alpha * (x - y).
        self.state.filtered_derivative +=
            self.config.d_filter_alpha * (raw_derivative - self.state.filtered_derivative);
        let d = self.config.kd * self.state.filtered_derivative;

        // ---- Book-keeping ------------------------------------------------
        self.state.prev_measurement = measurement;
        self.state.initialized = true;

        // ---- Sum and saturate --------------------------------------------
        clamp_sym(p + i + d, self.config.output_limit)
    }
}

/// Clamp `x` to the symmetric range [-limit, +limit]. An infinite limit
/// (the default) is a no-op, so an unconfigured limit means "no clamp".
#[inline]
fn clamp_sym(x: f32, limit: f32) -> f32 {
    if x > limit {
        limit
    } else if x < -limit {
        -limit
    } else {
        x
    }
}
