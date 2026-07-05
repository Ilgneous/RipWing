"""
6-DoF Nonlinear Quadcopter Dynamics Simulator
==============================================

A modular simulator that strictly separates the physical drone model (the Plant)
from the control law (the Controller). This separation is deliberate: the Plant
exposes a clean `(state, motor_rpms) -> state_derivative` interface, so any
controller that produces four motor commands can be dropped in without touching
the physics. Swapping the CascadedPIDController for an LQR controller later means
implementing one method, `compute_commands`, with the same signature.

State vector convention (12 states), all in the WORLD frame except body rates:
    index  symbol  meaning
    0..2   x y z   position in world/inertial frame [m]   (NED-style, +z up here)
    3..5   u v w   linear velocity in WORLD frame    [m/s]
    6..8   phi theta psi  Euler angles roll/pitch/yaw [rad] (ZYX convention)
    9..11  p q r   body angular rates                [rad/s]

Frame conventions
-----------------
- World frame: x forward, y left, z UP. Gravity acts in -z.
- Body frame:  x forward, y left, z up (FLU). Thrust is along +body-z.
- Euler angles use the ZYX (yaw-pitch-roll) intrinsic rotation sequence.
- Motors are in an "X" configuration, numbered:
      M0 front-right (CW)     M1 front-left (CCW)
      M3 rear-right  (CCW)    M2 rear-left  (CW)
  Spin directions alternate so net reaction torque cancels in hover.

Units are SI throughout.
"""

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib import gridspec


# ============================================================================
#  THE PLANT  --  pure physics, no knowledge of any controller
# ============================================================================
class QuadcopterPlant:
    """
    Full nonlinear 6-DoF rigid-body model of a quadcopter using the
    Newton-Euler equations.

    Inputs to the dynamics are the four motor angular speeds (rad/s). Each motor
    produces:
        thrust f_i = k_f * omega_i^2      (along +body z)
        drag   tau_i = k_m * omega_i^2    (about body z, sign = spin direction)

    The plant integrates its own state with an internal RK4 step, and also
    exposes `derivatives()` so an external integrator (e.g. solve_ivp) can be
    used instead.
    """

    def __init__(self,
                 mass=1.2,                 # total mass [kg]
                 arm_length=0.225,         # motor distance from center [m]
                 Ixx=0.0123, Iyy=0.0123, Izz=0.0224,  # diagonal inertia [kg m^2]
                 k_f=1.1e-6,               # thrust coefficient [N / (rad/s)^2]
                 k_m=1.9e-8,               # drag-torque coefficient [N m / (rad/s)^2]
                 drag_lin=0.10,            # translational aero drag [N / (m/s)]
                 g=9.81,                   # gravity [m/s^2]
                 max_rpm=25000.0):         # actuator saturation limit [rpm]
        # --- mass / inertia ---
        self.m = mass
        self.L = arm_length
        self.I = np.diag([Ixx, Iyy, Izz])         # body-frame inertia tensor
        self.I_inv = np.linalg.inv(self.I)
        self.Ixx, self.Iyy, self.Izz = Ixx, Iyy, Izz

        # --- aerodynamics ---
        self.k_f = k_f
        self.k_m = k_m
        self.drag_lin = drag_lin
        self.g = g

        # --- actuator limits ---
        self.max_rpm = max_rpm
        self.max_omega = max_rpm * 2.0 * np.pi / 60.0   # rad/s

        # Motor geometry for the "X" frame. Columns are motors 0..3.
        # Each motor's position in the body x-y plane (z=0).
        d = self.L / np.sqrt(2.0)
        # (x, y) of each motor; +x forward, +y left
        self.motor_pos = np.array([
            [ d, -d, 0.0],   # M0 front-right
            [ d,  d, 0.0],   # M1 front-left
            [-d,  d, 0.0],   # M2 rear-left
            [-d, -d, 0.0],   # M3 rear-right
        ])
        # Spin direction of each motor (+1 = CCW about +z, produces -z reaction
        # torque on the airframe; we track the reaction torque sign here).
        # Alternating signs so hover yaw torque cancels.
        self.spin_dir = np.array([-1.0, +1.0, -1.0, +1.0])

        # Hover speed (each motor) for convenience / controller feed-forward.
        self.omega_hover = np.sqrt(self.m * self.g / (4.0 * self.k_f))

    # ----------------------------------------------------------------------
    #  Kinematics helpers
    # ----------------------------------------------------------------------
    @staticmethod
    def rotation_matrix(phi, theta, psi):
        """
        Body-to-world rotation matrix for ZYX (yaw-pitch-roll) Euler angles.
        Maps a vector expressed in the body frame into the world frame.
        """
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth,  sth  = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)
        R = np.array([
            [cpsi*cth, cpsi*sth*sphi - spsi*cphi, cpsi*sth*cphi + spsi*sphi],
            [spsi*cth, spsi*sth*sphi + cpsi*cphi, spsi*sth*cphi - cpsi*sphi],
            [-sth,     cth*sphi,                  cth*cphi],
        ])
        return R

    @staticmethod
    def euler_rate_matrix(phi, theta):
        """
        Maps body angular rates (p, q, r) to Euler angle rates
        (phi_dot, theta_dot, psi_dot) for the ZYX convention.

        Singular at theta = +/- pi/2 (gimbal lock); fine for normal flight.
        """
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth,  sth  = np.cos(theta), np.sin(theta)
        tth = sth / cth
        W = np.array([
            [1.0, sphi*tth,      cphi*tth],
            [0.0, cphi,         -sphi],
            [0.0, sphi/cth,      cphi/cth],
        ])
        return W

    # ----------------------------------------------------------------------
    #  Force / torque from the motors
    # ----------------------------------------------------------------------
    def motor_forces_torques(self, omegas):
        """
        Given the four motor speeds [rad/s], return:
            total_thrust : scalar thrust along +body z [N]
            body_torque  : (3,) torque vector in the body frame [N m]
        """
        omegas = np.clip(omegas, 0.0, self.max_omega)
        f = self.k_f * omegas**2          # per-motor thrust [N]
        tau_drag = self.k_m * omegas**2   # per-motor yaw drag torque [N m]

        total_thrust = np.sum(f)

        # Roll/pitch torques from differential thrust across the arms.
        # tau = sum( r_i x F_i ), with F_i = f_i * z_hat (body frame).
        tau = np.zeros(3)
        for i in range(4):
            r = self.motor_pos[i]
            F = np.array([0.0, 0.0, f[i]])
            tau += np.cross(r, F)

        # Yaw torque from aerodynamic drag of the spinning rotors.
        tau[2] += np.sum(self.spin_dir * tau_drag)

        return total_thrust, tau

    # ----------------------------------------------------------------------
    #  Equations of motion
    # ----------------------------------------------------------------------
    def derivatives(self, state, omegas):
        """
        Newton-Euler equations of motion.

        Parameters
        ----------
        state  : (12,) current state vector (see module docstring)
        omegas : (4,)  motor speeds [rad/s]

        Returns
        -------
        dstate : (12,) time derivative of the state
        """
        # Unpack
        pos   = state[0:3]
        vel   = state[3:6]                 # world-frame linear velocity
        phi, theta, psi = state[6:9]
        omega_body = state[9:12]           # body rates p, q, r

        # --- forces ---
        thrust, tau = self.motor_forces_torques(omegas)
        R = self.rotation_matrix(phi, theta, psi)

        # Thrust is along +body z; rotate into world frame.
        thrust_world = R @ np.array([0.0, 0.0, thrust])
        gravity = np.array([0.0, 0.0, -self.m * self.g])
        drag = -self.drag_lin * vel        # simple linear translational drag

        accel = (thrust_world + gravity + drag) / self.m   # world-frame accel

        # --- rotational dynamics (Euler's equation in the body frame) ---
        # I * omega_dot = tau - omega x (I omega)
        omega_dot = self.I_inv @ (tau - np.cross(omega_body, self.I @ omega_body))

        # --- Euler angle kinematics ---
        W = self.euler_rate_matrix(phi, theta)
        euler_dot = W @ omega_body

        # Assemble
        dstate = np.zeros(12)
        dstate[0:3]  = vel
        dstate[3:6]  = accel
        dstate[6:9]  = euler_dot
        dstate[9:12] = omega_dot
        return dstate

    # ----------------------------------------------------------------------
    #  Integrators
    # ----------------------------------------------------------------------
    def step_rk4(self, state, omegas, dt):
        """
        Advance the state by dt using a fixed-step classical RK4 integrator.
        Motor speeds are held constant (zero-order hold) across the step,
        which matches how a discrete controller actually drives the plant.
        """
        k1 = self.derivatives(state, omegas)
        k2 = self.derivatives(state + 0.5 * dt * k1, omegas)
        k3 = self.derivatives(state + 0.5 * dt * k2, omegas)
        k4 = self.derivatives(state + dt * k3, omegas)
        return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

    def step_ivp(self, state, omegas, dt):
        """
        Alternative: advance the state with scipy.integrate.solve_ivp
        (adaptive RK45) over [0, dt] with zero-order-hold motor speeds.
        Slower but useful as a cross-check against the custom RK4.
        """
        sol = solve_ivp(
            fun=lambda t, s: self.derivatives(s, omegas),
            t_span=(0.0, dt),
            y0=state,
            method="RK45",
            rtol=1e-8, atol=1e-10,
        )
        return sol.y[:, -1]

    # ----------------------------------------------------------------------
    #  Linearization for LQR / state-space design
    # ----------------------------------------------------------------------
    def linearize(self, state_trim=None, omega_trim=None, eps_state=1e-6,
                  eps_input=1e-3):
        """
        Numerically linearize the continuous-time dynamics about a trim point
        using central finite differences:

            x_dot = f(x, u)   ~=   A (x - x_trim) + B (u - u_trim)

        where A = df/dx and B = df/du evaluated at (state_trim, omega_trim).

        Parameters
        ----------
        state_trim : (12,) trim state. Defaults to hover at the origin
                     (all zeros), i.e. level attitude, zero velocity/rates.
        omega_trim : (4,)  trim motor speeds. Defaults to the four-motor hover
                     speed `omega_hover` that exactly cancels gravity.
        eps_state  : perturbation size for the state Jacobian columns.
        eps_input  : perturbation size for the input Jacobian columns. Larger
                     than eps_state because dynamics are quadratic in omega, so
                     too small a step loses precision in float64.

        Returns
        -------
        A : (12, 12) state matrix  df/dx
        B : (12, 4)  input matrix  df/du   (input = motor speeds in rad/s)
        state_trim : the trim state used
        omega_trim : the trim input used

        Notes
        -----
        - The input here is raw motor speed (rad/s). If you'd rather design LQR
          in terms of the (thrust, tau) wrench, post-multiply B by the
          allocation Jacobian d(omega)/d(wrench) at trim, or linearize a wrapper
          that takes the wrench and calls the allocator. Working directly in
          motor speeds keeps B tied to the true actuators.
        - These are CONTINUOUS-time matrices. For a discrete LQR, discretize
          (e.g. scipy.signal.cont2discrete) at your control rate first.
        - Central differences give O(eps^2) accuracy and, importantly, make the
          A-matrix rows for the trivially-linear states (position fed by
          velocity) come out exactly right.
        """
        if state_trim is None:
            state_trim = np.zeros(12)
        if omega_trim is None:
            omega_trim = np.full(4, self.omega_hover)
        state_trim = np.asarray(state_trim, dtype=float)
        omega_trim = np.asarray(omega_trim, dtype=float)

        n_states = 12
        n_inputs = 4
        A = np.zeros((n_states, n_states))
        B = np.zeros((n_states, n_inputs))

        # --- A = df/dx via central differences, column by column ---
        for j in range(n_states):
            dx = np.zeros(n_states)
            dx[j] = eps_state
            f_plus  = self.derivatives(state_trim + dx, omega_trim)
            f_minus = self.derivatives(state_trim - dx, omega_trim)
            A[:, j] = (f_plus - f_minus) / (2.0 * eps_state)

        # --- B = df/du via central differences, column by column ---
        for j in range(n_inputs):
            du = np.zeros(n_inputs)
            du[j] = eps_input
            f_plus  = self.derivatives(state_trim, omega_trim + du)
            f_minus = self.derivatives(state_trim, omega_trim - du)
            B[:, j] = (f_plus - f_minus) / (2.0 * eps_input)

        return A, B, state_trim, omega_trim


# ============================================================================
#  CONTROL ALLOCATION  --  maps desired wrench to motor speeds
# ============================================================================
class ControlAllocator:
    """
    Inverts the motor geometry to turn a desired (thrust, tau_x, tau_y, tau_z)
    wrench into four motor speeds. This is kept separate so both the PID and a
    future LQR controller can share it.

    For the X-configuration with arm half-spacing d = L/sqrt(2):
        thrust = k_f (w0^2 + w1^2 + w2^2 + w3^2)
        tau_x  = k_f d (-w0^2 + w1^2 + w2^2 - w3^2)   (roll, about +x)
        tau_y  = k_f d ( w0^2 + w1^2 - w2^2 - w3^2)   (pitch, about +y)
        tau_z  = k_m (-w0^2 + w1^2 - w2^2 + w3^2)     (yaw)
    We solve for the squared speeds, clip to >= 0, and sqrt.
    """

    def __init__(self, plant: QuadcopterPlant):
        self.plant = plant
        d = plant.L / np.sqrt(2.0)
        kf, km = plant.k_f, plant.k_m
        sd = plant.spin_dir

        # Mixing matrix M maps [w0^2..w3^2] -> [thrust, tau_x, tau_y, tau_z].
        M = np.zeros((4, 4))
        for i in range(4):
            x, y, _ = plant.motor_pos[i]
            M[0, i] = kf                 # thrust
            M[1, i] = kf * y             # roll  torque = y * f
            M[2, i] = -kf * x            # pitch torque = -x * f  (sign per cross product)
            M[3, i] = km * sd[i]         # yaw   torque
        self.M = M
        self.M_inv = np.linalg.inv(M)

    def allocate(self, thrust, tau):
        """
        thrust : desired total thrust [N]
        tau    : (3,) desired body torque [N m]
        returns: (4,) motor speeds [rad/s], clipped to actuator limits.
        """
        wrench = np.array([thrust, tau[0], tau[1], tau[2]])
        omega_sq = self.M_inv @ wrench
        omega_sq = np.clip(omega_sq, 0.0, None)        # no negative squared speed
        omegas = np.sqrt(omega_sq)
        omegas = np.clip(omegas, 0.0, self.plant.max_omega)
        return omegas


# ============================================================================
#  THE CONTROLLER  --  cascaded PID, knows nothing about the physics internals
# ============================================================================
class PID:
    """Minimal scalar PID with output clamping and integral anti-windup."""

    def __init__(self, kp, ki, kd, out_limit=None, integ_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.integ_limit = integ_limit
        self.integ = 0.0
        self.prev_error = 0.0
        self._initialized = False

    def reset(self):
        self.integ = 0.0
        self.prev_error = 0.0
        self._initialized = False

    def update(self, error, dt, derivative=None):
        # Integral term with anti-windup clamp.
        self.integ += error * dt
        if self.integ_limit is not None:
            self.integ = np.clip(self.integ, -self.integ_limit, self.integ_limit)

        # Derivative term: use measured derivative if provided (avoids
        # derivative kick on setpoint changes), else finite-difference.
        if derivative is None:
            if not self._initialized:
                derivative = 0.0
                self._initialized = True
            else:
                derivative = (error - self.prev_error) / dt
        self.prev_error = error

        out = self.kp * error + self.ki * self.integ + self.kd * derivative
        if self.out_limit is not None:
            out = np.clip(out, -self.out_limit, self.out_limit)
        return out


class CascadedPIDController:
    """
    Cascaded position/attitude controller.

    Outer loop (position):
        position error  -> desired world acceleration (PID per axis)
        desired x,y accel + desired yaw -> desired roll/pitch angles
        desired z accel -> total thrust
    Inner loop (attitude/rate):
        attitude error -> desired body rates (P on angle)
        rate error     -> body torques (PID on rate)
    The desired thrust + torques go through the ControlAllocator to get motor
    speeds.

    To swap in LQR later: implement a class with the same `compute_commands(
    state, setpoint, dt) -> omegas` signature and feed it to the Simulation.
    """

    def __init__(self, plant: QuadcopterPlant, allocator: ControlAllocator):
        self.plant = plant
        self.alloc = allocator
        g = plant.g

        # ---- Outer loop: position -> desired acceleration ----
        # Limit horizontal accel so we don't command absurd tilt angles.
        self.pid_x = PID(kp=0.45, ki=0.02, kd=0.85, out_limit=0.5 * g, integ_limit=2.0)
        self.pid_y = PID(kp=0.45, ki=0.02, kd=0.85, out_limit=0.5 * g, integ_limit=2.0)
        self.pid_z = PID(kp=2.0,  ki=0.9,  kd=2.6,  out_limit=8.0,      integ_limit=4.0)

        # ---- Inner loop: attitude angle -> desired body rate (P) ----
        self.k_att_roll  = 8.0
        self.k_att_pitch = 8.0
        self.k_att_yaw   = 3.0

        # ---- Inner loop: body rate -> torque (PID) ----
        self.pid_p = PID(kp=0.040, ki=0.002, kd=0.0008, out_limit=2.0, integ_limit=0.5)
        self.pid_q = PID(kp=0.040, ki=0.002, kd=0.0008, out_limit=2.0, integ_limit=0.5)
        self.pid_r = PID(kp=0.060, ki=0.004, kd=0.0010, out_limit=1.0, integ_limit=0.5)

        # Max tilt the position loop is allowed to request [rad].
        self.max_tilt = np.radians(30.0)

    def reset(self):
        for pid in (self.pid_x, self.pid_y, self.pid_z,
                    self.pid_p, self.pid_q, self.pid_r):
            pid.reset()

    def compute_commands(self, state, setpoint, dt):
        """
        state    : (12,) current plant state
        setpoint : dict with keys 'x', 'y', 'z', 'yaw' (desired)
        dt       : control timestep [s]
        returns  : (4,) motor speeds [rad/s]
        """
        # Unpack state
        pos = state[0:3]
        vel = state[3:6]
        phi, theta, psi = state[6:9]
        p, q, r = state[9:12]

        xd, yd, zd = setpoint['x'], setpoint['y'], setpoint['z']
        psi_d = setpoint.get('yaw', 0.0)

        # ---------------- OUTER LOOP: position -> accel -> attitude ---------
        ax_des = self.pid_x.update(xd - pos[0], dt, derivative=-vel[0])
        ay_des = self.pid_y.update(yd - pos[1], dt, derivative=-vel[1])
        az_des = self.pid_z.update(zd - pos[2], dt, derivative=-vel[2])

        # Total thrust: gravity feed-forward + vertical PID, projected onto the
        # body z-axis so tilt doesn't sag altitude. T = m(g + az)/ (cos roll cos pitch)
        tilt_factor = max(np.cos(phi) * np.cos(theta), 0.5)
        thrust = self.plant.m * (self.plant.g + az_des) / tilt_factor
        thrust = np.clip(thrust, 0.0, 4.0 * self.plant.k_f * self.plant.max_omega**2)

        # Convert desired horizontal accelerations into desired tilt angles.
        # Rotate (ax, ay) into a yaw-aligned frame, then small-angle map to tilt.
        cpsi, spsi = np.cos(psi), np.sin(psi)
        ax_body =  cpsi * ax_des + spsi * ay_des
        ay_body = -spsi * ax_des + cpsi * ay_des

        theta_des =  np.clip(ax_body / self.plant.g, -self.max_tilt, self.max_tilt)  # pitch -> +x
        phi_des   = np.clip(-ay_body / self.plant.g, -self.max_tilt, self.max_tilt)  # roll  -> +y

        # ---------------- INNER LOOP: attitude -> rate -> torque -----------
        # Wrap yaw error into [-pi, pi].
        yaw_err = (psi_d - psi + np.pi) % (2*np.pi) - np.pi

        p_des = self.k_att_roll  * (phi_des - phi)
        q_des = self.k_att_pitch * (theta_des - theta)
        r_des = self.k_att_yaw   * yaw_err

        tau_x = self.pid_p.update(p_des - p, dt)
        tau_y = self.pid_q.update(q_des - q, dt)
        tau_z = self.pid_r.update(r_des - r, dt)
        tau = np.array([tau_x, tau_y, tau_z])

        # ---------------- ALLOCATION: wrench -> motor speeds ---------------
        omegas = self.alloc.allocate(thrust, tau)
        return omegas


# ============================================================================
#  THE SIMULATION  --  ties Plant + Controller together and logs history
# ============================================================================
class Simulation:
    """
    Runs the closed loop. The controller runs at `control_hz`; the plant is
    integrated at the same rate with zero-order-hold motor commands (simple and
    sufficient for this fidelity). Everything is logged for plotting.
    """

    def __init__(self, plant, controller, control_hz=200.0):
        self.plant = plant
        self.controller = controller
        self.dt = 1.0 / control_hz

        # History buffers
        self.t_hist = []
        self.state_hist = []
        self.omega_hist = []
        self.setpoint_hist = []

    def run(self, duration, setpoint_fn, x0=None):
        """
        duration    : total sim time [s]
        setpoint_fn : function(t) -> dict('x','y','z','yaw')
        x0          : optional initial 12-state (defaults to rest at origin)
        """
        if x0 is None:
            x0 = np.zeros(12)
        state = x0.copy()
        self.controller.reset()

        n_steps = int(duration / self.dt)
        for k in range(n_steps):
            t = k * self.dt
            sp = setpoint_fn(t)

            omegas = self.controller.compute_commands(state, sp, self.dt)

            # Log BEFORE stepping so state/command/time line up.
            self.t_hist.append(t)
            self.state_hist.append(state.copy())
            self.omega_hist.append(omegas.copy())
            self.setpoint_hist.append([sp['x'], sp['y'], sp['z']])

            # Advance the plant.
            state = self.plant.step_rk4(state, omegas, self.dt)

        # Convert to arrays for convenience.
        self.t_hist = np.array(self.t_hist)
        self.state_hist = np.array(self.state_hist)
        self.omega_hist = np.array(self.omega_hist)
        self.setpoint_hist = np.array(self.setpoint_hist)

    # ----------------------------------------------------------------------
    #  Visualization dashboard
    # ----------------------------------------------------------------------
    def plot_dashboard(self, save_path=None):
        """Three-panel dashboard: 3D trajectory, Euler angles, motor RPMs."""
        t = self.t_hist
        S = self.state_hist
        sp = self.setpoint_hist
        rpm = self.omega_hist * 60.0 / (2.0 * np.pi)   # rad/s -> rpm

        fig = plt.figure(figsize=(15, 9))
        gs = gridspec.GridSpec(2, 2, figure=fig, height_ratios=[1.2, 1.0],
                               hspace=0.30, wspace=0.25)

        # --- 3D trajectory ---
        ax3d = fig.add_subplot(gs[0, :], projection='3d')
        ax3d.plot(S[:, 0], S[:, 1], S[:, 2], color='tab:blue', lw=2,
                  label='actual path')
        ax3d.plot(sp[:, 0], sp[:, 1], sp[:, 2], '--', color='tab:red', lw=1.5,
                  label='commanded')
        ax3d.scatter(*S[0, 0:3], color='green', s=40, label='start')
        ax3d.scatter(*S[-1, 0:3], color='black', s=40, label='end')
        ax3d.set_xlabel('X [m]'); ax3d.set_ylabel('Y [m]'); ax3d.set_zlabel('Z [m]')
        ax3d.set_title('3D Trajectory')
        ax3d.legend(loc='upper left', fontsize=8)

        # --- Euler angles ---
        axang = fig.add_subplot(gs[1, 0])
        axang.plot(t, np.degrees(S[:, 6]), label='roll  φ', color='tab:red')
        axang.plot(t, np.degrees(S[:, 7]), label='pitch θ', color='tab:green')
        axang.plot(t, np.degrees(S[:, 8]), label='yaw  ψ', color='tab:blue')
        axang.set_xlabel('time [s]'); axang.set_ylabel('angle [deg]')
        axang.set_title('Euler Angles'); axang.grid(alpha=0.3); axang.legend(fontsize=8)

        # --- Motor RPMs ---
        axrpm = fig.add_subplot(gs[1, 1])
        for i in range(4):
            axrpm.plot(t, rpm[:, i], label=f'motor {i}')
        axrpm.axhline(self.plant.max_rpm, color='k', ls=':', lw=1,
                      label='saturation')
        axrpm.set_xlabel('time [s]'); axrpm.set_ylabel('RPM')
        axrpm.set_title('Motor Speeds'); axrpm.grid(alpha=0.3); axrpm.legend(fontsize=8)

        fig.suptitle('Quadcopter 6-DoF Simulation Dashboard', fontsize=14,
                     fontweight='bold')

        if save_path:
            fig.savefig(save_path, dpi=130, bbox_inches='tight')
        return fig


# ============================================================================
#  DEMO  --  hover to Z=5, then translate to X=2
# ============================================================================
def step_maneuver(t):
    """
    Commanded setpoint: climb to 5 m, then step to X = 2 m at t = 6 s.

    The raw commands are step changes, but we pass them through a first-order
    smoother (rate-limited approach) so the position loop isn't asked to track a
    discontinuity. This mirrors how a real waypoint controller feeds smoothed
    references and keeps the integrator from winding up on the climb.
    """
    # Smooth climb: approaches 5 m with a ~1.5 s time constant.
    z = 5.0 * (1.0 - np.exp(-t / 1.5))
    # Smooth X step after t = 6 s.
    x = 2.0 * (1.0 - np.exp(-max(t - 6.0, 0.0) / 1.5)) if t >= 6.0 else 0.0
    return {'x': x, 'y': 0.0, 'z': z, 'yaw': 0.0}


def demo_lqr_design():
    """
    Optional: show how `linearize()` feeds an LQR design. Requires scipy.
    This does NOT run in main() by default -- it's a reference for the swap.

    Steps:
      1. Linearize the plant about hover -> A, B (continuous-time).
      2. Pick Q (state penalty) and R (input penalty).
      3. Solve the continuous algebraic Riccati equation for K.
      4. The control law is then  u = omega_trim - K (x - x_trim),
         clipped to actuator limits. Wrap that in a class with the same
         `compute_commands(state, setpoint, dt) -> omegas` signature used by
         CascadedPIDController and hand it to Simulation unchanged.
    """
    from scipy.linalg import solve_continuous_are

    plant = QuadcopterPlant()
    A, B, x_trim, u_trim = plant.linearize()

    # Penalize position and attitude errors more than velocities/rates.
    Q = np.diag([10, 10, 10,    # x y z
                 1, 1, 1,       # velocities
                 5, 5, 5,       # roll pitch yaw
                 0.1, 0.1, 0.1])  # body rates
    R = np.eye(4) * 0.01

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P

    eig = np.linalg.eigvals(A - B @ K)
    print("LQR gain K (4x12) synthesized about hover.")
    print(f"Closed-loop stable: {np.all(eig.real < 0)} "
          f"(max real eig = {eig.real.max():.4f})")
    return K, x_trim, u_trim


def main():
    plant = QuadcopterPlant()
    allocator = ControlAllocator(plant)
    controller = CascadedPIDController(plant, allocator)
    sim = Simulation(plant, controller, control_hz=200.0)

    sim.run(duration=12.0, setpoint_fn=step_maneuver)

    # Quick numeric summary so the run is self-checking.
    final = sim.state_hist[-1]
    print(f"Final position:  x={final[0]:+.3f}  y={final[1]:+.3f}  z={final[2]:+.3f} [m]")
    print(f"Final attitude:  roll={np.degrees(final[6]):+.2f}  "
          f"pitch={np.degrees(final[7]):+.2f}  yaw={np.degrees(final[8]):+.2f} [deg]")
    max_rpm = (sim.omega_hist * 60.0 / (2*np.pi)).max()
    print(f"Peak motor speed: {max_rpm:.0f} rpm "
          f"({'SATURATED' if max_rpm >= plant.max_rpm else 'within limits'})")

    sim.plot_dashboard(save_path='quad_dashboard.png')
    print("Dashboard saved to quad_dashboard.png")


if __name__ == '__main__':
    main()
