"""
10-DOF Quadcopter Dynamics with Rotor Dynamics and Blade Flapping
=================================================================

DOF ACCOUNTING (why "10-DOF" and why 16 states)
-----------------------------------------------
- 6 rigid-body DOF (3 translation + 3 rotation)  -> 12 states (each DOF
  contributes a coordinate and a rate).
- 4 rotor DOF (each rotor spins about its own axis) -> 4 states. We track only
  each rotor *speed*, not its azimuth: azimuth is an ignorable coordinate
  (nothing in the model depends on blade phase after averaging over a rev).

  Total: 10 DOF, 16 states. Using standard symbols in the packed vector:
     [ x y z | u v w | phi theta psi | p q r | motor_speed[0..3] ]
     position    velocity   Euler angles   body rates   rotor speeds
  (roll=phi, pitch=theta, yaw=psi; body rates p,q,r = roll/pitch/yaw rates)

WHY BLADE FLAPPING IS QUASI-STATIC (not extra states)
-----------------------------------------------------
A flapping blade is a stiff pendulum spinning at rotor speed; its flap response
settles within roughly one rotor revolution. At hover here (~15,600 rpm =
260 rev/s) that is ~4 ms — two orders of magnitude faster than body dynamics
(~0.1–1 s) and an order faster than the motor lag (50 ms). By timescale
separation the flap angles are treated as *algebraic functions of the flight
state* rather than differential states. This is the standard treatment
(Hoffmann et al. 2007; Mahony, Kumar & Corke 2012; Pounds et al. 2010).

THE PHYSICS OF FLAPPING, IN ONE PARAGRAPH
-----------------------------------------
When the vehicle translates, the advancing blade sees higher airspeed (rotor
tip speed + vehicle speed) and the retreating blade lower. The advancing blade
generates more lift and flaps up; gyroscopic precession of the spinning rotor
turns that asymmetry into a tilt of the rotor tip-path plane *away from* the
direction of motion (peak displacement lags peak force by 90 deg of azimuth).
Two rigid-body consequences:
  1. The thrust vector tilts backward against the velocity -> a horizontal
     force opposing motion, proportional to (thrust x velocity). This "rotor
     drag" / H-force is the dominant natural velocity damping of a real
     multirotor — far larger than fuselage parasitic drag at low speed.
  2. The tilted thrust acts above the CoM and the blade roots transmit a hub
     moment -> a pitch/roll moment that tips the airframe away from the
     direction of motion (the documented "pitch-up in forward flight").
Additionally, when the *body* rotates at rate (p,q), the tip-path plane lags
the hub plane, producing a moment opposing the rotation: rotor rate damping.

NEW TERMS RELATIVE TO THE 6-DoF MODEL (QuadcopterPlant)
-------------------------------------------------------
  motor lag        w_i_dot = (w_cmd_i - w_i) / motor_time_constant       (first order)
  flapping force   F_b     = -flap_coeff * T * [v_bx, v_by, 0]
  flapping moment  body_torque     = flap_moment_arm * (z_hat x F_b)
  rate damping     body_torque    += -rotor_rate_damp_coeff * [p, q, 0]
  rotor gyroscopic body_torque    += -(body_rates x [0,0,rotor_angular_momentum_z]),  rotor_angular_momentum_z = rotor_inertia * sum(s_i w_i)
  yaw reaction     torque_z  += -rotor_inertia * sum(s_i * w_i_dot)

The 6-DoF model is recovered exactly by zeroing the new coefficients (this is
enforced by a regression test), so everything already validated about the base
model carries over.

WHAT THIS MEANS FOR PID TUNING
------------------------------
The command now reaches the body ONLY through the rotor states: the input
matrix B of the linearized model is zero in every body row (except a tiny yaw
reaction feedthrough) and 1/motor_time_constant in the rotor rows. The motor pole at
1/motor_time_constant rad/s is a hard bandwidth ceiling for the rate loop that the 6-DoF
model simply did not contain — gains that looked fine before can ring or go
unstable here, which is precisely why tuning should happen on this model.

Representative parameter values are provided; identify them for YOUR airframe
(motor step response for motor_time_constant, coast-down decel for flap_coeff) before
trusting absolute gain numbers.
"""

import numpy as np

from QuadSim.models.quad_sim import (
    QuadcopterPlant,
    ControlAllocator,
    CascadedPIDController,
    Simulation,
)


class QuadcopterPlant10DOF(QuadcopterPlant):
    """
    16-state / 10-DOF plant. Input is now the *commanded* rotor speeds
    (rad/s); actual rotor speeds are states with first-order lag.
    """

    N_STATES = 16

    def __init__(self,
                 motor_time_constant=0.05,      # motor+ESC+prop time constant [s]
                 flap_coeff=0.008,        # TPP tilt per airspeed [rad/(m/s)]
                 flap_moment_arm=0.05,         # effective flap moment arm [m]
                                      #   (rotor-plane height above CoM plus an
                                      #    equivalent arm for hub stiffness)
                 rotor_rate_damp_coeff=0.005,   # rotor rate damping [N m/(rad/s)]
                 rotor_inertia=3.0e-5,      # rotor+prop spin inertia [kg m^2]
                 rotor_radius=0.127,        # propeller radius [m] (~10 in prop)
                 ground_effect_gain=0.30,   # C_a: max thrust surplus at z=0 [-]
                 ground_effect_decay=2.2,   # C_b: decay rate in z/R [-]
                 ground_effect_model='exponential',   # or 'cheeseman' / 'none'
                 landing_gear_height=0.06,  # rotor plane height above the
                                            # skids [m]; z=0 means landed
                 **kwargs):
        super().__init__(**kwargs)
        self.motor_time_constant = motor_time_constant
        self.flap_coeff = flap_coeff
        self.flap_moment_arm = flap_moment_arm
        self.rotor_rate_damp_coeff = rotor_rate_damp_coeff
        self.rotor_inertia = rotor_inertia
        self.rotor_radius = rotor_radius
        self.ground_effect_gain = ground_effect_gain
        self.ground_effect_decay = ground_effect_decay
        self.ground_effect_model = ground_effect_model
        self.landing_gear_height = landing_gear_height

    # ------------------------------------------------------------------
    def rotor_heights(self, position_z, body_to_world):
        """
        Height of each rotor hub above the ground plane [m].

        Ground effect is a PER-ROTOR phenomenon: when the vehicle banks, the
        low rotor sits deeper in ground effect than the high one. That
        asymmetry is what turns ground effect into a moment generator (and is
        why a drone can tip during landing), so a single body-level thrust
        scaling would miss the most important failure mode.

        The rotor hubs lie in the body x-y plane (motor_pos has z=0), so each
        hub's world height is the CoM height plus the world-z component of its
        rotated body-frame offset. `landing_gear_height` shifts the reference
        so z=0 means "skids on the ground", not "rotor disk on the ground".
        """
        hub_world_offsets = (body_to_world @ self.motor_pos.T).T   # (4,3)
        return position_z + hub_world_offsets[:, 2] + self.landing_gear_height

    def ground_effect_ratio(self, rotor_height):
        """
        Thrust multiplier K_G = T_IGE / T_OGE for each rotor (>= 1).

        Default model (He & Leang, exponential form):
            K_G = 1 + C_a * exp(-C_b * z / R)

        This is used instead of the classical Cheeseman-Bennett relation
            K_G = 1 / (1 - (R / 4z)^2)
        for a specific reason. Cheeseman-Bennett was derived for full-size
        helicopters, whose fuselage hangs BELOW the rotor so z/R is never
        small. It is singular at z/R = 0.25 and predicts infinite thrust at
        the ground. A low-profile quadcopter routinely operates below that
        height during touchdown, which is exactly the regime that matters for
        landing. The exponential model predicts a FINITE maximum surplus at
        z = 0 (K_G -> 1 + C_a), so a landing simulation stays well-posed all
        the way down.

        `ground_effect_model='cheeseman'` selects the classical relation
        (clamped off its singularity) for comparison; 'none' disables it.
        """
        z_over_r = np.maximum(rotor_height, 0.0) / self.rotor_radius
        if self.ground_effect_model == 'none':
            return np.ones_like(z_over_r)
        if self.ground_effect_model == 'cheeseman':
            z_clamped = np.maximum(z_over_r, 0.35)   # stay off the singularity
            return 1.0 / (1.0 - (1.0 / (4.0 * z_clamped))**2)
        # Default: exponential, finite at the ground.
        return 1.0 + self.ground_effect_gain * np.exp(
            -self.ground_effect_decay * z_over_r)

    # ------------------------------------------------------------------
    def hover_state(self):
        """16-state rest-at-origin with rotors already at hover speed."""
        s = np.zeros(16)
        s[12:16] = self.hover_motor_speed
        return s

    # ------------------------------------------------------------------
    def derivatives(self, state, motor_speed_cmd):
        """
        Full 16-state equations of motion. `motor_speed_cmd` are COMMANDED rotor
        speeds [rad/s]; actual speeds live in state[12:16].
        """
        vel = state[3:6]
        roll, pitch, yaw = state[6:9]                              # Euler angles (phi, theta, psi)
        body_rates = state[9:12]                                   # body rates p,q,r
        rotor_speeds = np.clip(state[12:16], 0.0, self.max_motor_speed)   # actual rotor speeds
        cmd = np.clip(motor_speed_cmd, 0.0, self.max_motor_speed)

        # --- baseline rotor thrust/torque (same aerodynamic law as 6-DoF) ---
        body_to_world = self.rotation_matrix(roll, pitch, yaw)

        # --- ground effect (per rotor) -------------------------------------
        # Each rotor's thrust is scaled by its own K_G, because a banked
        # vehicle has one rotor closer to the ground than another. Recomputing
        # thrust/torque per rotor here (rather than calling the aggregate
        # motor_forces_torques) is what lets the asymmetry produce a real
        # roll/pitch moment.
        per_rotor_thrust = self.thrust_coeff * rotor_speeds**2
        if self.ground_effect_model != 'none':
            heights = self.rotor_heights(state[2], body_to_world)
            thrust_gain = self.ground_effect_ratio(heights)
            per_rotor_thrust = per_rotor_thrust * thrust_gain
        thrust = float(np.sum(per_rotor_thrust))

        # Roll/pitch torque from the (now possibly asymmetric) thrust set,
        # plus the unchanged yaw drag torque.
        torque_roll = float(np.sum(self.motor_pos[:, 1] * per_rotor_thrust))
        torque_pitch = float(-np.sum(self.motor_pos[:, 0] * per_rotor_thrust))
        motor_drag_torque = self.drag_torque_coeff * rotor_speeds**2
        torque_yaw = float(np.sum(self.spin_dir * motor_drag_torque))
        body_torque = np.array([torque_roll, torque_pitch, torque_yaw])

        # --- quasi-static blade flapping ---------------------------------
        # In-plane body airspeed drives the tip-path-plane tilt.
        velocity_body = body_to_world.T @ vel
        flap_force_body = -self.flap_coeff * thrust * np.array([velocity_body[0], velocity_body[1], 0.0])
        # Moment from the tilted thrust acting above the CoM + hub stiffness,
        # collapsed into one effective arm:  body_torque = h * (z_hat x F).
        flap_torque = self.flap_moment_arm * np.cross(np.array([0.0, 0.0, 1.0]), flap_force_body)

        # --- rotor rate damping (TPP lags a rotating hub) -----------------
        rate_damp_torque = -self.rotor_rate_damp_coeff * np.array([body_rates[0], body_rates[1], 0.0])

        # --- rotor gyroscopics --------------------------------------------
        # Net rotor angular momentum about body z (cancels when CW/CCW pairs
        # are balanced; appears during yaw commands).
        rotor_angular_momentum_z = self.rotor_inertia * np.sum(self.spin_dir * rotor_speeds)
        gyro_torque = -np.cross(body_rates, np.array([0.0, 0.0, rotor_angular_momentum_z]))

        # --- motor first-order lag ----------------------------------------
        motor_speeds_dot = (cmd - rotor_speeds) / self.motor_time_constant
        # Reaction torque on the airframe from accelerating the rotors.
        tau_react = np.array([0.0, 0.0,
                              -self.rotor_inertia * np.sum(self.spin_dir * motor_speeds_dot)])

        tau_total = body_torque + flap_torque + rate_damp_torque + gyro_torque + tau_react

        # --- Newton (world frame) -----------------------------------------
        thrust_world = body_to_world @ np.array([0.0, 0.0, thrust])
        flap_force_world = body_to_world @ flap_force_body
        gravity = np.array([0.0, 0.0, -self.m * self.g])
        drag = -self.linear_drag_coeff * vel
        accel = (thrust_world + flap_force_world + gravity + drag) / self.m

        # --- Euler (body frame) -------------------------------------------
        body_rates_dot = self.I_inv @ (tau_total - np.cross(body_rates, self.I @ body_rates))
        body_rate_to_euler_rate = self.euler_rate_matrix(roll, pitch)

        d = np.zeros(16)
        d[0:3] = vel
        d[3:6] = accel
        d[6:9] = body_rate_to_euler_rate @ body_rates
        d[9:12] = body_rates_dot
        d[12:16] = motor_speeds_dot
        return d

    # ------------------------------------------------------------------
    def linearize(self, state_trim=None, motor_speed_trim=None, eps_state=1e-6,
                  eps_input=1e-3):
        """
        16-state linearization about a trim point (default: hover with rotors
        at hover speed and commands equal to rotor speeds). Same central
        finite-difference scheme as the 6-DoF version, generalized dimensions.

        Structure worth knowing for control design:
          - B is ~zero in all body rows: commands act on the body ONLY through
            the rotor states (the motor lag is the bandwidth bottleneck).
          - A[12:16,12:16] = -(1/motor_time_constant) I ; B[12:16,:] = (1/motor_time_constant) I.

        The default trim sits at 5 m, deliberately OUT of ground effect. Hover
        trim at z = 0 is not an equilibrium once ground effect is modeled (the
        extra thrust accelerates the vehicle upward), so linearizing there
        would expand about a non-equilibrium point. To study the ground-effect
        regime, pass an explicit low-altitude `state_trim` with rotor speeds
        retrimmed for that height.
        """
        if state_trim is None:
            state_trim = self.hover_state()
            state_trim[2] = 5.0
        if motor_speed_trim is None:
            motor_speed_trim = np.full(4, self.hover_motor_speed)
        state_trim = np.asarray(state_trim, dtype=float)
        motor_speed_trim = np.asarray(motor_speed_trim, dtype=float)

        n, m_in = 16, 4
        A = np.zeros((n, n))
        B = np.zeros((n, m_in))
        for j in range(n):
            dx = np.zeros(n); dx[j] = eps_state
            fp = self.derivatives(state_trim + dx, motor_speed_trim)
            fm = self.derivatives(state_trim - dx, motor_speed_trim)
            A[:, j] = (fp - fm) / (2.0 * eps_state)
        for j in range(m_in):
            du = np.zeros(m_in); du[j] = eps_input
            fp = self.derivatives(state_trim, motor_speed_trim + du)
            fm = self.derivatives(state_trim, motor_speed_trim - du)
            B[:, j] = (fp - fm) / (2.0 * eps_input)
        return A, B, state_trim, motor_speed_trim


class Simulation10DOF(Simulation):
    """
    Simulation with a sensible 16-state default initial condition and an
    optional ground contact constraint.

    Ground contact matters here specifically because ground effect is a
    near-ground phenomenon: without a floor, a commanded landing simply flies
    through z = 0 into negative altitude, and any "did it land cleanly?"
    metric measures nonsense. The constraint is deliberately simple (an
    inelastic floor that zeroes downward velocity and holds the vehicle at
    z = 0), which is enough to answer touchdown questions without pretending
    to model gear compliance, friction, or bounce.
    """

    def __init__(self, *args, enforce_ground=False, **kwargs):
        super().__init__(*args, **kwargs)
        self.enforce_ground = enforce_ground

    def run(self, duration, setpoint_fn, x0=None):
        if x0 is None:
            # Start with rotors at hover speed to avoid an artificial spin-up
            # transient contaminating every experiment.
            x0 = self.plant.hover_state()
        if not self.enforce_ground:
            super().run(duration, setpoint_fn, x0=x0)
            return

        # --- rollout with an inelastic floor at z = 0 ---
        state = x0.copy()
        self.controller.reset()
        n_steps = int(duration / self.dt)
        for k in range(n_steps):
            t = k * self.dt
            setpoint = setpoint_fn(t)
            motor_speeds = self.controller.compute_commands(state, setpoint, self.dt)

            self.t_hist.append(t)
            self.state_hist.append(state.copy())
            self.motor_speed_hist.append(motor_speeds.copy())
            self.setpoint_hist.append([setpoint['x'], setpoint['y'], setpoint['z']])

            state = self.plant.step_rk4(state, motor_speeds, self.dt)

            # Inelastic contact: cannot pass through the ground, and resting on
            # it kills downward velocity and residual attitude rates.
            if state[2] < 0.0:
                state[2] = 0.0
                if state[5] < 0.0:
                    state[5] = 0.0
                state[3:5] = 0.0        # no sliding
                state[9:12] = 0.0       # gear reaction stops rotation

        self.t_hist = np.array(self.t_hist)
        self.state_hist = np.array(self.state_hist)
        self.motor_speed_hist = np.array(self.motor_speed_hist)
        self.setpoint_hist = np.array(self.setpoint_hist)


def step_maneuver(t):
    """Same smoothed climb + x-step reference as the 6-DoF demo."""
    z = 5.0 * (1.0 - np.exp(-t / 1.5))
    x = 2.0 * (1.0 - np.exp(-max(t - 6.0, 0.0) / 1.5)) if t >= 6.0 else 0.0
    return {'x': x, 'y': 0.0, 'z': z, 'yaw': 0.0}


def main():
    plant = QuadcopterPlant10DOF()
    alloc = ControlAllocator(plant)
    ctrl = CascadedPIDController(plant, alloc)
    sim = Simulation10DOF(plant, ctrl, control_hz=200.0)
    sim.run(12.0, step_maneuver)

    final = sim.state_hist[-1]
    print(f"Final position:  x={final[0]:+.3f}  y={final[1]:+.3f}  z={final[2]:+.3f} [m]")
    print(f"Final attitude:  roll={np.degrees(final[6]):+.2f}  "
          f"pitch={np.degrees(final[7]):+.2f}  yaw={np.degrees(final[8]):+.2f} [deg]")
    cmd_rpm = sim.motor_speed_hist * 60 / (2 * np.pi)
    act_rpm = sim.state_hist[:, 12:16] * 60 / (2 * np.pi)
    print(f"Peak commanded rpm: {cmd_rpm.max():.0f} | peak actual rpm: {act_rpm.max():.0f}")
    fig = sim.plot_dashboard(save_path='quad_dashboard_10dof.png')
    print("Dashboard saved to quad_dashboard_10dof.png")


if __name__ == '__main__':
    main()
