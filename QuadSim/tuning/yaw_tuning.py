"""
Yaw-axis PID tuning for the 10-DOF quadcopter model
===================================================

STAGE 3 of the tuning sequence in pid_tuning.py, run after it: the roll/pitch
inner loop and the position loop are frozen at the gains already in
tuned_gains.json, and only the yaw path is tuned:

    k_att_yaw   heading error -> yaw-rate demand (P)
    kp/ki/kd_yaw   yaw-rate error -> yaw torque (PID)

Why yaw gets its own stage
--------------------------
Yaw is a different plant from roll/pitch. Its torque comes from rotor DRAG
(k_m), not thrust lever arms, so its authority is ~25x weaker (~±0.54 N·m over
the whole motor range with the 7" prop) and the controller output is clamped
to half of that. Yaw also has a direct feedthrough the other axes lack: the
rotor-reaction torque J_r·Σ s_i·dω_i/dt, which turns every motor-command
change into an instantaneous yaw kick. Tuning yaw jointly with roll/pitch lets
the optimizer trade one axis against the other through that coupling. A
separate stage keeps each cost meaningful.

Scenario: a 30° heading doublet (0 -> 30° at 0.3 s, back to 0 at 2.0 s)
while holding position and altitude, with
  * the same sensor noise as stages 1-2 (NoisyController), and
  * a constant yaw torque bias (0.01 N·m, roughly one motor tilted 1°),
    which the integral term has to cancel.

Cost: ITAE of the heading error after each step + overshoot + steady-state
heading error + yaw-rate ripple + motor thrash + saturation + roll/pitch
coupling. Only the yaw keys are written back into tuned_gains.json; the
stage 1-2 gains and metrics are left as they are.

Run from the RipWing repo root:
    python -m QuadSim.tuning.yaw_tuning
"""

import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from QuadSim.models.quad_sim_10dof import Simulation10DOF
from QuadSim.tuning.pid_tuning import (DEFAULTS, YAW_KEYS, DisturbedPlant,
                                       NoisyController, build_controller,
                                       optimize_stage, rise_time)

QUADSIM = Path(__file__).resolve().parents[1]
GAINS_JSON = QUADSIM / 'results/tuning_data/tuned_gains.json'
FIG_FILE = QUADSIM / 'results/figures/yaw_tuning.png'

YAW_STEP_DEG = 30.0
T_STEP, T_RETURN = 0.3, 2.0
DURATION = 3.5
YAW_TORQUE_BIAS = 0.01          # N·m
SETTLE = 0.6                    # s after each step before "steady" windows
MAXFEV = 600
# Second Nelder-Mead start. The cost is nearly flat in ki_yaw (the integral
# only trims a fraction of a degree of bias error, against a few % more
# overshoot), and from the defaults the log-space simplex walks ki down that
# flat direction to ~0. Starting with a real integrator finds the other basin;
# the lower-cost result wins.
ALT_START = {'k_att_yaw': 8.0, 'kp_yaw': 0.08, 'ki_yaw': 0.1, 'kd_yaw': 0.001}


def yaw_target_deg(t):
    return YAW_STEP_DEG if T_STEP <= t < T_RETURN else 0.0


def yaw_scenario(t):
    return {'x': 0.0, 'y': 0.0, 'z': 2.0, 'yaw': np.radians(yaw_target_deg(t))}


def simulate(g, seed=0, noisy=True, disturbed=True):
    plant = (DisturbedPlant(tau_bias=(0.0, 0.0, YAW_TORQUE_BIAS)) if disturbed
             else DisturbedPlant(tau_bias=(0.0, 0.0, 0.0)))
    ctrl = build_controller(plant, g)
    if noisy:
        ctrl = NoisyController(ctrl, seed=seed)
    sim = Simulation10DOF(plant, ctrl, control_hz=200.0)
    x0 = plant.hover_state()
    x0[2] = 2.0
    sim.run(DURATION, yaw_scenario, x0=x0)
    return sim, plant


METRIC_KEYS = ('itae', 'overshoot_pct', 'steady_err', 'ripple', 'thrash_rpm',
               'saturation', 'tilt_deg', 'rise_time')


def yaw_cost(g, return_metrics=False, seed=0):
    try:
        sim, plant = simulate(g, seed=seed)
    except Exception:
        sim = None
    S = None if sim is None else sim.state_hist
    if S is None or not np.all(np.isfinite(S)) or np.abs(S[:, 6:8]).max() > np.pi / 2:
        if return_metrics:
            d = {k: float('nan') for k in METRIC_KEYS}
            d['cost'] = 1e6
            return d
        return 1e6
    t = sim.t_hist
    psi = np.degrees(S[:, 8])
    target = np.array([yaw_target_deg(ti) for ti in t])
    err = psi - target

    # ITAE, with time measured from the most recent step so the return
    # phase is weighted like the first step rather than 1.7 s later.
    t_since = np.where(t >= T_RETURN, t - T_RETURN, t - T_STEP)
    m = t >= T_STEP
    itae = np.trapezoid(np.abs(err[m]) * t_since[m], t[m])

    up = (t >= T_STEP) & (t < T_RETURN)
    down = t >= T_RETURN
    overshoot = max(0.0, (psi[up].max() - YAW_STEP_DEG) / YAW_STEP_DEG,
                    -psi[down].min() / YAW_STEP_DEG)

    steady = ((t >= T_STEP + SETTLE) & (t < T_RETURN)) | (t >= T_RETURN + SETTLE)
    steady_err = np.mean(np.abs(err[steady]))
    ripple = np.degrees(np.std(S[t >= T_RETURN + SETTLE, 11]))   # yaw rate
    cmd_rpm = sim.motor_speed_hist * 60 / (2 * np.pi)
    ms = t >= T_RETURN + SETTLE
    thrash = np.std(cmd_rpm[ms] - cmd_rpm[ms].mean(axis=0))
    sat = np.mean((sim.motor_speed_hist >= plant.max_motor_speed - 1e-6)
                  | (sim.motor_speed_hist <= 1e-6))
    # Yaw that disturbs roll/pitch (via the allocator or the rotor-reaction
    # path) is not free. Mean |tilt|, deg.
    tilt = np.mean(np.abs(np.degrees(S[:, 6:8])).sum(axis=1))

    cost = (itae + 8.0 * overshoot + 2.0 * steady_err + 0.2 * ripple
            + 0.01 * thrash + 5.0 * sat + 1.0 * tilt)
    if return_metrics:
        rise = rise_time(t, psi, 0.0, YAW_STEP_DEG, T_STEP)
        return dict(itae=itae, overshoot_pct=100 * overshoot,
                    steady_err=steady_err, ripple=ripple, thrash_rpm=thrash,
                    saturation=sat, tilt_deg=tilt, rise_time=rise, cost=cost)
    return cost


def fmt(m):
    return (f"rise {m['rise_time']:.3f} s | ovs {m['overshoot_pct']:.1f}% | "
            f"steady {m['steady_err']:.3f} deg | ripple {m['ripple']:.2f} deg/s | "
            f"thrash {m['thrash_rpm']:.0f} rpm | tilt {m['tilt_deg']:.3f} deg | "
            f"cost {m['cost']:.3f}")


def main():
    data = json.loads(GAINS_JSON.read_text())
    base = {**DEFAULTS, **data['tuned']}
    # Start from the controller defaults, not a previous yaw tune, so
    # re-running is reproducible.
    start = {**base, **{k: DEFAULTS[k] for k in YAW_KEYS}}

    print('=== STAGE 3: yaw (heading P + yaw-rate PID), others frozen ===')
    before = yaw_cost(start, return_metrics=True)
    runs = []
    for name, x0 in (('defaults', start), ('alt start', {**start, **ALT_START})):
        g, r = optimize_stage(yaw_cost, x0, list(YAW_KEYS), maxfev=MAXFEV)
        print(f'  from {name:9s}: cost {r.fun:.3f} after {r.nfev} evals '
              f'({"converged" if r.success else r.message}) '
              + ' '.join(f'{k}={g[k]:.4g}' for k in YAW_KEYS))
        runs.append((r.fun, g, r))
    _, tuned, res = min(runs, key=lambda x: x[0])
    after = yaw_cost(tuned, return_metrics=True)
    for k in YAW_KEYS:
        print(f'  {k:10s}: {start[k]:.5f} -> {tuned[k]:.5f}')
    print(f'  before: {fmt(before)}')
    print(f'  after : {fmt(after)}')

    # Robustness: the optimizer saw one noise seed; check others.
    seeds = range(1, 6)
    cb = [yaw_cost(start, seed=s) for s in seeds]
    ca = [yaw_cost(tuned, seed=s) for s in seeds]
    print(f'  cost over seeds 1-5: default {np.mean(cb):.3f} ± {np.std(cb):.3f}'
          f' -> tuned {np.mean(ca):.3f} ± {np.std(ca):.3f}')

    for k in YAW_KEYS:
        data['tuned'][k] = tuned[k]
        data['defaults'][k] = DEFAULTS[k]
    data['stage3_yaw_metrics'] = {
        'before': before, 'after': after,
        'seed_cost_before': [float(c) for c in cb],
        'seed_cost_after': [float(c) for c in ca],
        'nfev': int(res.nfev), 'converged': bool(res.success)}
    GAINS_JSON.write_text(json.dumps(data, indent=2, default=float) + '\n')
    print(f'\nSaved {GAINS_JSON.relative_to(QUADSIM.parent)}')

    # ------------------------- figure -------------------------------------
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.4))
    for g, label in ((start, 'default yaw gains'), (tuned, 'tuned yaw gains')):
        sim, _ = simulate(g)
        t, S = sim.t_hist, sim.state_hist
        axes[0].plot(t, np.degrees(S[:, 8]), label=label)
        axes[1].plot(t, np.degrees(S[:, 11]), label=label)
        axes[2].plot(t, np.degrees(S[:, 6]), label=f'roll, {label}', lw=1)
        axes[2].plot(t, np.degrees(S[:, 7]), label=f'pitch, {label}', lw=1)
    ts = np.linspace(0, DURATION, 800)
    axes[0].plot(ts, [yaw_target_deg(x) for x in ts], 'k:', lw=1, label='setpoint')
    axes[0].set_ylabel('heading [deg]')
    axes[1].set_ylabel('yaw rate [deg/s]')
    axes[2].set_ylabel('attitude [deg]')
    axes[0].set_title('30° heading doublet')
    axes[1].set_title('Yaw rate')
    axes[2].set_title('Roll/pitch coupling')
    for ax in axes:
        ax.set_xlabel('t [s]')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle('Yaw tuning on the 10-DOF model (noise + 0.01 N·m yaw bias)',
                 fontweight='bold')
    fig.tight_layout()
    FIG_FILE.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG_FILE, dpi=120, bbox_inches='tight')
    print(f'Saved {FIG_FILE.relative_to(QUADSIM.parent)}')


if __name__ == '__main__':
    main()
