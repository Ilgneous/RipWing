"""
PID tuning harness for the 10-DOF quadcopter model
==================================================

Tunes the CascadedPIDController against the 10-DOF plant (motor lag + blade
flapping), the model whose bandwidth limits and disturbances actually resemble
a real vehicle. Two stages, mirroring how real quads are tuned:

  STAGE 1 — INNER LOOP (attitude + rate).
      Scenario: 15-degree roll step while holding altitude.
      Tuned: k_att (attitude P), and the rate-loop kp/ki/kd (roll & pitch
      share gains by symmetry).
      Cost: ITAE of the roll error
            + overshoot penalty
            + steady-state RATE RIPPLE penalty (the motor-pole limit cycle —
              the failure mode the 6-DoF model cannot even represent)
            + actuator saturation penalty.

  STAGE 2 — OUTER LOOP (position), inner gains frozen.
      Scenario: 2 m x-step + altitude hold.
      Tuned: kp/kd for x-y, kp/kd for z.
      Cost: ITAE of position error + overshoot penalty + tilt-limit penalty.

Optimizer: Nelder-Mead over log10(gains) — gains are inherently positive and
span decades, so log-space makes the simplex well-behaved.

Outputs:
  tuned_gains.json        the gain set to transplant into the controller
  tuning_comparison.png   before/after step responses on the 10-DOF plant

NOTE ON TRANSFER TO FIRMWARE: these are continuous-signal gains evaluated at
200 Hz. Keep the same loop rate on target, or re-verify after changing it, and
re-identify tau_motor / k_flap for your airframe before trusting absolute
numbers (motor step response and coast-down tests respectively).
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

from QuadSim.models.quad_sim import ControlAllocator, CascadedPIDController
from QuadSim.models.quad_sim_10dof import QuadcopterPlant10DOF, Simulation10DOF


# ---------------------------------------------------------------------------
#  Realism shims: sensor noise + trim disturbance
# ---------------------------------------------------------------------------
# Without these, the optimizer exploits the simulator: noise-free measurements
# make huge derivative gains free damping (first runs returned kd_z ~ 400 and
# rate kp 260x default). Real gyros/estimators are noisy; noise is what makes
# derivative gain expensive and finite. A constant torque bias (shifted CoM /
# battery) is what gives the integral gain a job.

class NoisyController:
    """Wraps a controller, corrupting its state MEASUREMENT (not the truth)."""

    def __init__(self, inner, seed=0,
                 pos_std=0.02, vel_std=0.05,
                 att_std=np.radians(0.4), gyro_std=np.radians(1.0)):
        self.inner = inner
        self.rng = np.random.default_rng(seed)
        self.stds = (pos_std, vel_std, att_std, gyro_std)

    def reset(self):
        self.inner.reset()

    def compute_commands(self, state, setpoint, dt):
        s = state.copy()
        ps, vs, as_, gs = self.stds
        s[0:3] += self.rng.normal(0, ps, 3)
        s[3:6] += self.rng.normal(0, vs, 3)
        s[6:9] += self.rng.normal(0, as_, 3)
        s[9:12] += self.rng.normal(0, gs, 3)
        return self.inner.compute_commands(s, setpoint, dt)


class DisturbedPlant(QuadcopterPlant10DOF):
    """10-DOF plant with a constant body-torque bias (e.g. shifted battery)."""

    def __init__(self, tau_bias=(0.02, 0.0, 0.0), **kw):
        super().__init__(**kw)
        self.tau_bias = np.asarray(tau_bias, dtype=float)

    def derivatives(self, state, omega_cmd):
        d = super().derivatives(state, omega_cmd)
        d[9:12] += self.I_inv @ self.tau_bias
        return d


# ---------------------------------------------------------------------------
#  Gain plumbing
# ---------------------------------------------------------------------------
DEFAULTS = {
    'k_att': 8.0,
    'kp_rate': 0.040, 'ki_rate': 0.002, 'kd_rate': 0.0008,
    'kp_xy': 0.45, 'kd_xy': 0.85,
    'kp_z': 2.0, 'kd_z': 2.6,
}

def build_controller(plant, g):
    """Construct a controller with the given gain dict applied.

    Two hard-won realism fixes live here (found when noise was added):

    1. Yaw output limit must sit INSIDE the plant's physical yaw envelope.
       Yaw authority is only ~±0.26 N·m over the entire motor range (k_m is
       ~50x smaller than k_f); the original out_limit of 1.0 N·m let the
       controller demand 4x more yaw torque than physically exists. The
       allocator then "solves" for it by slamming the CW pair to zero and the
       CCW pair to max, destroying thrust/roll/pitch — a positive-feedback
       yaw ratchet. Real FCs desaturate/prioritize yaw last for this reason.
    2. Rate-loop D terms get a low-pass (real FCs always filter D). The rate
       PIDs numerically differentiate a noisy error at 200 Hz, which
       amplifies measurement noise by sqrt(2)/dt ≈ 280x.
    """
    c = CascadedPIDController(plant, ControlAllocator(plant))
    c.k_att_roll = c.k_att_pitch = g['k_att']
    for pid in (c.pid_p, c.pid_q):
        pid.kp, pid.ki, pid.kd = g['kp_rate'], g['ki_rate'], g['kd_rate']
        pid.d_lpf_hz = 30.0
    for pid in (c.pid_x, c.pid_y):
        pid.kp, pid.kd = g['kp_xy'], g['kd_xy']
    c.pid_z.kp, c.pid_z.kd = g['kp_z'], g['kd_z']
    # Yaw: clamp inside physical authority (~half the full envelope), filter D.
    yaw_authority = 2.0 * plant.k_m * plant.max_omega**2
    c.pid_r.out_limit = 0.5 * yaw_authority
    c.pid_r.d_lpf_hz = 30.0
    return c


def simulate(g, setpoint_fn, duration, x0_overrides=None,
             noisy=True, disturbed=False, seed=0):
    plant = DisturbedPlant() if disturbed else QuadcopterPlant10DOF()
    ctrl = build_controller(plant, g)
    if noisy:
        ctrl = NoisyController(ctrl, seed=seed)
    sim = Simulation10DOF(plant, ctrl, control_hz=200.0)
    x0 = plant.hover_state()
    for i, v in (x0_overrides or {}).items():
        x0[i] = v
    sim.run(duration, setpoint_fn, x0=x0)
    return sim, plant


# ---------------------------------------------------------------------------
#  Scenarios & costs
# ---------------------------------------------------------------------------
ROLL_STEP_DEG = 15.0
T_STEP = 0.3
T_RETURN = 1.2   # doublet: step back to level at t=1.2 s

def roll_target_deg(t):
    """Roll doublet: 0 -> 15 deg at T_STEP, back to 0 at T_RETURN.

    A doublet rather than an indefinite hold: holding a tilt forever means
    indefinite lateral acceleration, so by t~2.5s the vehicle slides at
    ~10 m/s where flapping moments (growing with speed) dominate and the
    scenario stops measuring the attitude loop. Doublets are the standard
    flight-test input for exactly this reason.
    """
    return ROLL_STEP_DEG if (T_STEP <= t < T_RETURN) else 0.0

def attitude_scenario(t):
    return {'x': 0, 'y': 0, 'z': 2.0, 'yaw': 0,
            'phi': np.radians(roll_target_deg(t)),
            'theta': 0.0}

def position_scenario(t):
    return {'x': (2.0 if t >= 0.5 else 0.0), 'y': 0.0, 'z': 2.0, 'yaw': 0.0}



def _fail(return_metrics, keys):
    """Uniform failure return: honors return_metrics so callers can subscript."""
    if return_metrics:
        d = {k: float('nan') for k in keys}
        d['cost'] = 1e6
        return d
    return 1e6

ATT_KEYS = ('itae', 'overshoot_pct', 'ripple', 'steady_err', 'thrash_rpm',
            'saturation', 'rise_time')
POS_KEYS = ('itae_x', 'itae_z', 'overshoot_pct', 'thrash_rpm', 'tilt_pk_deg',
            'rise_time')

def attitude_cost(g, return_metrics=False):
    try:
        sim, plant = simulate(g, attitude_scenario, 2.5, {2: 2.0},
                              noisy=True, disturbed=True)
    except Exception:
        return _fail(return_metrics, ATT_KEYS)
    S, t = sim.state_hist, sim.t_hist
    # Tip-over guard on roll/pitch ONLY: yaw legitimately wraps past pi/2.
    if not np.all(np.isfinite(S)) or np.abs(S[:, 6:8]).max() > np.pi / 2:
        return _fail(return_metrics, ATT_KEYS)
    roll = np.degrees(S[:, 6])
    target = np.array([roll_target_deg(ti) for ti in t])
    err = roll - target
    m = t >= T_STEP
    itae = np.trapezoid(np.abs(err[m]) * (t[m] - T_STEP), t[m])
    # Overshoot during the "up" phase of the doublet only.
    up = (t >= T_STEP) & (t < T_RETURN)
    overshoot = max(0.0, (roll[up].max() - ROLL_STEP_DEG) / ROLL_STEP_DEG)
    ms = t >= 2.0
    # Motor-buzz limit cycle on the TRUE rates (10-DOF-specific failure mode).
    ripple = np.degrees(np.std(S[ms, 9]))                     # deg/s
    # Steady-state error vs level target after the doublet: this is what makes
    # ki earn its keep against the constant torque bias.
    steady_err = np.mean(np.abs(err[t >= 2.0]))
    # Motor thrash: noise amplified into the commands (punishes huge kd/kp).
    cmd_rpm = sim.omega_hist * 60 / (2 * np.pi)
    thrash = np.std(cmd_rpm[ms] - cmd_rpm[ms].mean(axis=0))   # rpm
    sat = np.mean(sim.omega_hist >= plant.max_omega - 1e-6)
    # Yaw wander: the 10-DOF yaw-reaction feedthrough (-J_r * sum s_i domega_i)
    # turns noisy motor commands into yaw torque noise. Penalize so the tuner
    # doesn't accept gains whose transients slosh yaw around.
    yaw_pen = np.mean(np.abs(np.degrees(S[:, 8])))
    cost = (itae + 8.0 * overshoot + 0.5 * ripple + 2.0 * steady_err
            + 0.01 * thrash + 5.0 * sat + 0.05 * yaw_pen)
    if return_metrics:
        rise = rise_time(t, roll, 0.0, ROLL_STEP_DEG, T_STEP)
        return dict(itae=itae, overshoot_pct=100 * overshoot, ripple=ripple,
                    steady_err=steady_err, thrash_rpm=thrash,
                    saturation=sat, rise_time=rise, cost=cost)
    return cost


def position_cost(g, return_metrics=False):
    try:
        sim, plant = simulate(g, position_scenario, 6.0, {2: 2.0}, noisy=True)
    except Exception:
        return _fail(return_metrics, POS_KEYS)
    S, t = sim.state_hist, sim.t_hist
    # Tip-over guard on roll/pitch ONLY: yaw legitimately wraps past pi/2.
    if not np.all(np.isfinite(S)) or np.abs(S[:, 6:8]).max() > np.pi / 2:
        return _fail(return_metrics, POS_KEYS)
    x = S[:, 0]
    z = S[:, 2]
    m = t >= 0.5
    itae_x = np.trapezoid(np.abs(x[m] - 2.0) * (t[m] - 0.5), t[m])
    itae_z = np.trapezoid(np.abs(z[m] - 2.0) * (t[m] - 0.5), t[m])
    overshoot = max(0.0, (x.max() - 2.0) / 2.0)
    tilt_pk = np.abs(S[:, 6:8]).max()
    tilt_pen = max(0.0, np.degrees(tilt_pk) - 25.0) * 0.2    # discourage >25 deg
    # Motor thrash penalty (punishes noise-amplifying derivative gains).
    ms = t >= 4.0
    cmd_rpm = sim.omega_hist * 60 / (2 * np.pi)
    thrash = np.std(cmd_rpm[ms] - cmd_rpm[ms].mean(axis=0))
    cost = itae_x + 2.0 * itae_z + 6.0 * overshoot + tilt_pen + 0.01 * thrash
    if return_metrics:
        rise = rise_time(t, x, 0.0, 2.0, 0.5)
        return dict(itae_x=itae_x, itae_z=itae_z,
                    overshoot_pct=100 * overshoot, thrash_rpm=thrash,
                    rise_time=rise, cost=cost)
    return cost


def rise_time(t, y, y0, y1, t0):
    """10%-90% rise time after the step at t0."""
    span = y1 - y0
    m = t >= t0
    tt, yy = t[m], y[m]
    try:
        t10 = tt[np.argmax(yy >= y0 + 0.1 * span)]
        t90 = tt[np.argmax(yy >= y0 + 0.9 * span)]
        return t90 - t10
    except Exception:
        return np.nan


# ---------------------------------------------------------------------------
#  Staged optimization (Nelder-Mead in log-gain space)
# ---------------------------------------------------------------------------
def optimize_stage(cost_fn, base_gains, keys, maxfev=120):
    x0 = np.log10([base_gains[k] for k in keys])

    def wrapped(x):
        g = dict(base_gains)
        for k, v in zip(keys, 10.0 ** x):
            g[k] = v
        return cost_fn(g)

    res = minimize(wrapped, x0, method='Nelder-Mead',
                   options=dict(maxfev=maxfev, xatol=1e-3, fatol=1e-4))
    tuned = dict(base_gains)
    for k, v in zip(keys, 10.0 ** res.x):
        tuned[k] = v
    return tuned, res


def main():
    rng_report = {}

    print("=== STAGE 1: inner loop (attitude + rate) ===")
    before1 = attitude_cost(DEFAULTS, return_metrics=True)
    g1, res1 = optimize_stage(attitude_cost, DEFAULTS,
                              ['k_att', 'kp_rate', 'ki_rate', 'kd_rate'])
    after1 = attitude_cost(g1, return_metrics=True)
    print(f"  evaluations: {res1.nfev}")
    for k in ['k_att', 'kp_rate', 'ki_rate', 'kd_rate']:
        print(f"  {k:8s}: {DEFAULTS[k]:.5f} -> {g1[k]:.5f}")
    print(f"  rise {before1['rise_time']:.3f}->{after1['rise_time']:.3f} s | "
          f"ovs {before1['overshoot_pct']:.1f}->{after1['overshoot_pct']:.1f}% | "
          f"ripple {before1['ripple']:.3f}->{after1['ripple']:.3f} deg/s | "
          f"cost {before1['cost']:.3f}->{after1['cost']:.3f}")

    print("\n=== STAGE 2: outer loop (position), inner frozen ===")
    before2 = position_cost(g1, return_metrics=True)
    g2, res2 = optimize_stage(position_cost, g1,
                              ['kp_xy', 'kd_xy', 'kp_z', 'kd_z'])
    after2 = position_cost(g2, return_metrics=True)
    print(f"  evaluations: {res2.nfev}")
    for k in ['kp_xy', 'kd_xy', 'kp_z', 'kd_z']:
        print(f"  {k:8s}: {g1[k]:.4f} -> {g2[k]:.4f}")
    print(f"  rise {before2['rise_time']:.3f}->{after2['rise_time']:.3f} s | "
          f"ovs {before2['overshoot_pct']:.1f}->{after2['overshoot_pct']:.1f}% | "
          f"cost {before2['cost']:.3f}->{after2['cost']:.3f}")

    with open('tuned_gains.json', 'w') as f:
        json.dump({'defaults': DEFAULTS, 'tuned': g2,
                   'stage1_metrics': {'before': before1, 'after': after1},
                   'stage2_metrics': {'before': before2, 'after': after2}},
                  f, indent=2, default=float)
    print("\nSaved tuned_gains.json")

    # ------------------ before/after figure -----------------------------
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
    simb, _ = simulate(DEFAULTS, attitude_scenario, 2.5, {2: 2.0})
    sima, _ = simulate(g2, attitude_scenario, 2.5, {2: 2.0})
    ax = axes[0]
    ax.plot(simb.t_hist, np.degrees(simb.state_hist[:, 6]), label='default gains')
    ax.plot(sima.t_hist, np.degrees(sima.state_hist[:, 6]), label='tuned gains')
    ax.axhline(ROLL_STEP_DEG, color='k', ls=':', lw=1)
    ax.set_title('Attitude step (10-DOF plant)')
    ax.set_xlabel('t [s]'); ax.set_ylabel('roll [deg]')
    ax.grid(alpha=0.3); ax.legend(fontsize=9)

    simb, _ = simulate(DEFAULTS, position_scenario, 6.0, {2: 2.0})
    sima, _ = simulate(g2, position_scenario, 6.0, {2: 2.0})
    ax = axes[1]
    ax.plot(simb.t_hist, simb.state_hist[:, 0], label='default gains')
    ax.plot(sima.t_hist, sima.state_hist[:, 0], label='tuned gains')
    ax.axhline(2.0, color='k', ls=':', lw=1)
    ax.set_title('2 m position step (10-DOF plant)')
    ax.set_xlabel('t [s]'); ax.set_ylabel('x [m]')
    ax.grid(alpha=0.3); ax.legend(fontsize=9)

    fig.suptitle('PID tuning on the 10-DOF model: before vs after',
                 fontweight='bold')
    fig.tight_layout()
    fig.savefig('tuning_comparison.png', dpi=120, bbox_inches='tight')
    print("Saved tuning_comparison.png")


if __name__ == '__main__':
    main()
