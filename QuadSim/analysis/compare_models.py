"""
6-DoF vs 10-DOF model comparison
================================

Demonstrates the concrete behavioral differences the upgrade buys, each tied to
a documented real-quadrotor phenomenon. Without flight-test data, "improvement"
means: the 10-DOF model reproduces physics that real vehicles exhibit and the
6-DoF model structurally cannot.

  A. VELOCITY DAMPING — a level real quad sheds speed mainly through rotor
     (flapping) drag, not parasitic drag. 6-DoF under-damps badly.
  B. ACTUATOR LAG — commanded vs actual rotor speed during a step. The 6-DoF
     model assumed the dashed and solid lines are identical.
  C. GAIN-TUNING TRAP — with hot rate gains, the 6-DoF model predicts a clean
     response while the 10-DOF model rings against the motor pole. Tuning on
     the 6-DoF model produces gains that misbehave on the real vehicle.
  D. FLAPPING DISTURBANCE IN TRANSLATION — forward flight generates a pitch
     moment the controller must actively reject; visible as a pitch offset
     relative to the 6-DoF prediction.

Output: model_comparison.png + printed quantitative metrics.
"""

import numpy as np
import matplotlib.pyplot as plt

from QuadSim.models.quad_sim import (QuadcopterPlant, ControlAllocator,
                      CascadedPIDController, Simulation)
from QuadSim.models.quad_sim_10dof import QuadcopterPlant10DOF, Simulation10DOF


def make(plant_cls, hot_rate_gain=1.0):
    plant = plant_cls()
    ctrl = CascadedPIDController(plant, ControlAllocator(plant))
    for pid in (ctrl.pid_p, ctrl.pid_q):
        pid.kp *= hot_rate_gain
    sim_cls = Simulation10DOF if plant_cls is QuadcopterPlant10DOF else Simulation
    return plant, ctrl, sim_cls(plant, ctrl, control_hz=200.0)


def x0_for(plant, overrides):
    x0 = plant.hover_state() if hasattr(plant, 'hover_state') else np.zeros(12)
    for idx, val in overrides.items():
        x0[idx] = val
    return x0


def main():
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.subplots_adjust(hspace=0.35, wspace=0.25)

    # ---------------- A: velocity damping ------------------------------
    level_hold = lambda t: {'x': 0, 'y': 0, 'z': 5, 'yaw': 0,
                            'phi': 0.0, 'theta': 0.0}
    p6, _, sim6 = make(QuadcopterPlant)
    sim6.run(4.0, level_hold, x0=x0_for(p6, {2: 5.0, 3: 4.0}))
    p10, _, sim10 = make(QuadcopterPlant10DOF)
    sim10.run(4.0, level_hold, x0=x0_for(p10, {2: 5.0, 3: 4.0}))

    ax = axes[0, 0]
    ax.plot(sim6.t_hist, sim6.state_hist[:, 3], label='6-DoF (parasitic drag only)')
    ax.plot(sim10.t_hist, sim10.state_hist[:, 3], label='10-DOF (+ flapping rotor drag)')
    ax.set_title('A. Velocity decay, level attitude hold')
    ax.set_xlabel('t [s]'); ax.set_ylabel('forward velocity [m/s]')
    ax.grid(alpha=0.3); ax.legend(fontsize=9)

    # half-life metric
    def half_life(t, v):
        idx = np.argmax(v <= v[0] / 2)
        return t[idx] if idx > 0 else np.inf
    hl6 = half_life(sim6.t_hist, sim6.state_hist[:, 3])
    hl10 = half_life(sim10.t_hist, sim10.state_hist[:, 3])

    # ---------------- B: actuator lag ----------------------------------
    roll_step = lambda t: {'x': 0, 'y': 0, 'z': 2, 'yaw': 0,
                           'phi': (np.radians(15) if t >= 0.3 else 0.0),
                           'theta': 0.0}
    p10b, _, sim10b = make(QuadcopterPlant10DOF)
    sim10b.run(1.2, roll_step, x0=x0_for(p10b, {2: 2.0}))
    t = sim10b.t_hist
    cmd_rpm = sim10b.omega_hist[:, 1] * 60 / (2 * np.pi)
    act_rpm = sim10b.state_hist[:, 13] * 60 / (2 * np.pi)

    ax = axes[0, 1]
    ax.plot(t, cmd_rpm, '--', label='commanded (what 6-DoF assumes is real)')
    ax.plot(t, act_rpm, label='actual rotor speed (10-DOF state)')
    ax.set_title('B. Motor lag during a roll step (motor 1)')
    ax.set_xlabel('t [s]'); ax.set_ylabel('RPM')
    ax.grid(alpha=0.3); ax.legend(fontsize=9)

    # ---------------- C: the gain-tuning trap ---------------------------
    # Sweep the rate-loop gain. On the 6-DoF model, higher gain only ever
    # helps (no actuator pole). On the 10-DOF model, high gain excites a
    # sustained rate limit cycle against the motor pole -- the "motor buzz"
    # every pilot who over-tunes rate PIDs knows. The angle trace looks fine
    # in both; the RATE and the motor commands reveal it.
    HOTS = [1, 2, 4, 8, 16, 32, 48]
    buzz6, buzz10 = [], []
    for HOT in HOTS:
        p6h, _, sim6h = make(QuadcopterPlant, hot_rate_gain=HOT)
        sim6h.run(2.0, roll_step, x0=x0_for(p6h, {2: 2.0}))
        m = sim6h.t_hist > 1.0
        buzz6.append(np.std(sim6h.state_hist[m, 9]))

        p10h, _, sim10h = make(QuadcopterPlant10DOF, hot_rate_gain=HOT)
        sim10h.run(2.0, roll_step, x0=x0_for(p10h, {2: 2.0}))
        m = sim10h.t_hist > 1.0
        buzz10.append(np.std(sim10h.state_hist[m, 9]))

    ax = axes[1, 0]
    ax.semilogx(HOTS, np.degrees(buzz6), 'o-', label='6-DoF: "any gain is fine"')
    ax.semilogx(HOTS, np.degrees(buzz10), 's-',
                label='10-DOF: limit cycle vs motor pole')
    ax.set_title('C. The tuning trap: steady-state rate oscillation vs rate gain')
    ax.set_xlabel('rate-loop kp multiplier'); ax.set_ylabel('roll-rate ripple std [deg/s]')
    ax.grid(alpha=0.3, which='both'); ax.legend(fontsize=9)
    r6, r10 = np.degrees(buzz6[-1]), np.degrees(buzz10[-1])

    # ---------------- D: steady tilt in cruise --------------------------
    # Track a constant-velocity reference. The 10-DOF vehicle must hold ~2x
    # the pitch: thrust has to overcome flapping rotor drag in addition to
    # parasitic drag. A PID tuned on the 6-DoF model underestimates the tilt
    # (and hence motor headroom) needed for forward flight.
    fwd = lambda t: {'x': 1.5 * t, 'y': 0, 'z': 5, 'yaw': 0}
    p6d, _, sim6d = make(QuadcopterPlant)
    sim6d.run(8.0, fwd, x0=x0_for(p6d, {2: 5.0}))
    p10d, _, sim10d = make(QuadcopterPlant10DOF)
    sim10d.run(8.0, fwd, x0=x0_for(p10d, {2: 5.0}))

    ax = axes[1, 1]
    ax.plot(sim6d.t_hist, np.degrees(sim6d.state_hist[:, 7]), label='6-DoF pitch')
    ax.plot(sim10d.t_hist, np.degrees(sim10d.state_hist[:, 7]), label='10-DOF pitch')
    ax.set_title('D. Steady pitch tracking a 1.5 m/s cruise\n'
                 '(flapping drag demands ~2x the tilt)')
    ax.set_xlabel('t [s]'); ax.set_ylabel('pitch [deg]')
    ax.grid(alpha=0.3); ax.legend(fontsize=9)

    fig.suptitle('What the 10-DOF model captures that 6-DoF cannot',
                 fontsize=14, fontweight='bold')
    fig.savefig('model_comparison.png', dpi=120, bbox_inches='tight')
    print("Saved model_comparison.png\n")

    # ---------------- quantitative summary ------------------------------
    print("--- Quantitative comparison ---")
    print(f"[A] velocity half-life:   6-DoF = {hl6:.2f} s   10-DOF = {hl10:.2f} s")
    lag_pk = np.max(np.abs(cmd_rpm - act_rpm))
    print(f"[B] peak cmd-vs-actual rotor speed gap: {lag_pk:.0f} rpm")
    print(f"[C] rate ripple at 48x gain: 6-DoF = {r6:.2f} deg/s   10-DOF = {r10:.2f} deg/s "
          f"({r10/max(r6,1e-9):.0f}x larger)")
    m6 = sim6d.t_hist > 5.0
    m10 = sim10d.t_hist > 5.0
    p6s = np.degrees(sim6d.state_hist[m6, 7].mean())
    p10s = np.degrees(sim10d.state_hist[m10, 7].mean())
    print(f"[D] steady cruise pitch:  6-DoF = {p6s:+.2f} deg   10-DOF = {p10s:+.2f} deg")


if __name__ == '__main__':
    main()
