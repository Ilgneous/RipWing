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
  C. GAIN-TUNING TRAP — sweeping the rate gain up, the 10-DOF model breaks
     into a limit cycle against the motor pole at a lower gain than the 6-DoF
     model, which is limited only by the 200 Hz sample rate. Tuning to the
     6-DoF edge produces gains that buzz on the real vehicle.
  D. FLAPPING DISTURBANCE IN TRANSLATION — forward flight generates a pitch
     moment the controller must actively reject; visible as a pitch offset
     relative to the 6-DoF prediction.

Output: results/figures/model_comparison.png + printed quantitative metrics.

Run from the RipWing repo root:
    python -m QuadSim.analysis.compare_models
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

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
    cmd_rpm = sim10b.motor_speed_hist[:, 1] * 60 / (2 * np.pi)
    act_rpm = sim10b.state_hist[:, 13] * 60 / (2 * np.pi)

    ax = axes[0, 1]
    ax.plot(t, cmd_rpm, '--', label='commanded (what 6-DoF assumes is real)')
    ax.plot(t, act_rpm, label='actual rotor speed (10-DOF state)')
    ax.set_title('B. Motor lag during a roll step (motor 1)')
    ax.set_xlabel('t [s]'); ax.set_ylabel('RPM')
    ax.grid(alpha=0.3); ax.legend(fontsize=9)

    # ---------------- C: the gain-tuning trap ---------------------------
    # Sweep the rate-loop gain and measure the steady-state roll-rate ripple
    # once the step has settled (t > 2 s of a 3 s run; measuring earlier
    # counts the slow low-gain step response as "ripple"). Past some gain
    # each model breaks into a sustained limit cycle -- the "motor buzz" of
    # an over-tuned rate loop. In the 6-DoF model only the 200 Hz sample
    # rate sets that limit; in the 10-DOF model the motor pole adds lag and
    # the buzz starts at a lower gain. The angle trace looks fine in both;
    # the RATE reveals it.
    HOTS = [1, 4, 16, 32, 48, 64, 72, 80, 88, 96, 112, 128, 160, 192]
    BUZZ_DEG_S = 1.0                     # ripple above this = limit cycle
    buzz6, buzz10 = [], []
    for HOT in HOTS:
        for plant_cls, out in ((QuadcopterPlant, buzz6),
                               (QuadcopterPlant10DOF, buzz10)):
            ph, _, simh = make(plant_cls, hot_rate_gain=HOT)
            simh.run(3.0, roll_step, x0=x0_for(ph, {2: 2.0}))
            m = simh.t_hist > 2.0
            out.append(np.degrees(np.std(simh.state_hist[m, 9])))

    def onset(buzz):
        # Lowest gain from which EVERY higher gain buzzes. (At x1 the slow
        # default response is still settling at t = 2 s; that is not a
        # limit cycle and is excluded by this definition.)
        h_on = np.inf
        for h, b in reversed(list(zip(HOTS, buzz))):
            if b <= BUZZ_DEG_S:
                break
            h_on = h
        return h_on
    on6, on10 = onset(buzz6), onset(buzz10)

    # Where the tuned rate gain sits on this axis.
    tuned_json = (Path(__file__).resolve().parents[1]
                  / 'results/tuning_data/tuned_gains.json')
    import json
    tuned_kp = json.loads(tuned_json.read_text())['tuned']['kp_rate']
    tuned_hot = tuned_kp / CascadedPIDController(
        QuadcopterPlant(), ControlAllocator(QuadcopterPlant())).pid_p.kp

    ax = axes[1, 0]
    ax.semilogx(HOTS, buzz6, 'o-', label=f'6-DoF: sample-rate limit only (buzz from x{on6})')
    ax.semilogx(HOTS, buzz10, 's-',
                label=f'10-DOF: + motor pole (buzz from x{on10})')
    ax.axvline(tuned_hot, color='k', ls=':', lw=1)
    ax.text(tuned_hot * 1.08, 0.6 * max(max(buzz6), max(buzz10)),
            f'tuned gain\n(x{tuned_hot:.1f})', fontsize=8)
    ax.set_title('C. The tuning trap: steady-state rate oscillation vs rate gain')
    ax.set_xlabel('rate-loop kp multiplier (x default)')
    ax.set_ylabel('settled roll-rate ripple std [deg/s]')
    ax.grid(alpha=0.3, which='both'); ax.legend(fontsize=9)

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
    out_file = Path(__file__).resolve().parents[1] / 'results/figures/model_comparison.png'
    out_file.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_file, dpi=120, bbox_inches='tight')
    print(f"Saved {out_file}\n")

    # ---------------- quantitative summary ------------------------------
    print("--- Quantitative comparison ---")
    print(f"[A] velocity half-life:   6-DoF = {hl6:.2f} s   10-DOF = {hl10:.2f} s")
    lag_pk = np.max(np.abs(cmd_rpm - act_rpm))
    print(f"[B] peak cmd-vs-actual rotor speed gap: {lag_pk:.0f} rpm")
    print(f"[C] limit-cycle onset (ripple > {BUZZ_DEG_S} deg/s): 6-DoF = x{on6}   "
          f"10-DOF = x{on10}   (tuned gain = x{tuned_hot:.1f})")
    for h, b6, b10 in zip(HOTS, buzz6, buzz10):
        print(f"      x{h:<4d} ripple 6-DoF {b6:6.2f}   10-DOF {b10:6.2f} deg/s")
    m6 = sim6d.t_hist > 5.0
    m10 = sim10d.t_hist > 5.0
    p6s = np.degrees(sim6d.state_hist[m6, 7].mean())
    p10s = np.degrees(sim10d.state_hist[m10, 7].mean())
    print(f"[D] steady cruise pitch:  6-DoF = {p6s:+.2f} deg   10-DOF = {p10s:+.2f} deg")


if __name__ == '__main__':
    main()
