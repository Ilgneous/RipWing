"""
Ground Effect Study: what IGE does to landing, and how to handle it.

Four experiments, each isolating one consequence:
  [A] Thrust ratio vs height       -- the model, and why not Cheeseman-Bennett
  [B] Landing descent profile      -- the "float" that prevents clean touchdown
  [C] Bank-angle restoring moment  -- the per-rotor asymmetry near the ground
  [D] Hover trim shift             -- throttle needed to hold height vs altitude

Run:  python3 ground_effect_study.py
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from quad_sim import ControlAllocator, CascadedPIDController
from quad_sim_10dof import QuadcopterPlant10DOF, Simulation10DOF


def descend_to_ground(t):
    """
    Precision landing profile: a 0.15 m/s commanded descent ramp, the way an
    autoland would fly it. A gentle approach is the revealing case -- an
    aggressive descent punches through ground effect, while a slow one lets
    the thrust surplus hold the vehicle off the pad.
    """
    return {'x': 0.0, 'y': 0.0, 'z': max(1.0 - 0.15 * t, -0.2), 'yaw': 0.0}


def run_landing(model, start_altitude=1.0, duration=14.0):
    """
    Commanded landing WITH a ground contact constraint. The floor is essential:
    without it a descent command flies straight through z=0 and any touchdown
    metric is meaningless.
    """
    plant = QuadcopterPlant10DOF(ground_effect_model=model)
    ctrl = CascadedPIDController(plant, ControlAllocator(plant))
    sim = Simulation10DOF(plant, ctrl, control_hz=200.0, enforce_ground=True)
    x0 = plant.hover_state()
    x0[2] = start_altitude
    sim.run(duration, descend_to_ground, x0=x0)
    return sim, plant


def touchdown_metrics(sim):
    """Time to first ground contact and the impact velocity at that moment."""
    z = sim.state_hist[:, 2]
    vz = sim.state_hist[:, 5]
    contact = np.flatnonzero(z <= 1e-9)
    if contact.size == 0:
        return None, None
    i = contact[0]
    # Impact speed is the descent rate on the step just before contact.
    return sim.t_hist[i], abs(vz[max(i - 1, 0)])


def main():
    fig = plt.figure(figsize=(15, 10))
    gs = fig.add_gridspec(2, 2, hspace=0.30, wspace=0.25)

    # ---------------- [A] thrust ratio vs height ----------------
    ax = fig.add_subplot(gs[0, 0])
    p_exp = QuadcopterPlant10DOF(ground_effect_model='exponential')
    p_che = QuadcopterPlant10DOF(ground_effect_model='cheeseman')
    z_over_r = np.linspace(0.0, 3.0, 400)
    heights = z_over_r * p_exp.rotor_radius
    ax.plot(z_over_r, p_exp.ground_effect_ratio(heights),
            lw=2, color='tab:blue', label='exponential (used here)')
    ax.plot(z_over_r, p_che.ground_effect_ratio(heights),
            lw=2, ls='--', color='tab:red', label='Cheeseman-Bennett (clamped)')
    ax.axvline(0.25, color='k', ls=':', lw=1)
    ax.text(0.28, 1.22, 'C-B singularity\nat z/R = 0.25', fontsize=8)
    ax.axhline(1.0, color='gray', lw=0.8)
    ax.set_xlabel('rotor height / rotor radius  (z/R)')
    ax.set_ylabel('thrust ratio  $K_G = T_{IGE}/T_{OGE}$')
    ax.set_title('[A] Ground effect model: finite at the ground')
    ax.grid(alpha=0.3); ax.legend(fontsize=8); ax.set_ylim(0.95, 1.45)

    # ---------------- [B] landing descent ----------------
    ax = fig.add_subplot(gs[0, 1])
    results = {}
    for label, model, color in [('with ground effect', 'exponential', 'tab:blue'),
                                ('without', 'none', 'tab:gray')]:
        sim, plant = run_landing(model)
        results[label] = sim
        ax.plot(sim.t_hist, sim.state_hist[:, 2], lw=2, color=color, label=label)
    ax.axhline(0.0, color='k', lw=1, ls='-')
    ax.set_xlabel('time [s]'); ax.set_ylabel('altitude [m]')
    ax.set_title('[B] Precision landing: 0.15 m/s commanded descent')
    ax.grid(alpha=0.3); ax.legend(fontsize=8)

    t_on, v_on = touchdown_metrics(results['with ground effect'])
    t_off, v_off = touchdown_metrics(results['without'])
    if t_on is not None and t_off is not None:
        ax.annotate(f'touchdown delayed {t_on - t_off:+.2f} s\n(vehicle floats on IGE)',
                    xy=(t_on, 0.0), xytext=(2.0, 0.62), fontsize=8,
                    arrowprops=dict(arrowstyle='->', lw=1))
    # Show the commanded profile for reference.
    t_ref = results['without'].t_hist
    ax.plot(t_ref, [max(descend_to_ground(tt)['z'], 0.0) for tt in t_ref],
            ls=':', color='tab:red', lw=1.4, label='commanded')
    ax.legend(fontsize=8)

    # ---------------- [C] restoring moment vs bank ----------------
    ax = fig.add_subplot(gs[1, 0])
    plant = QuadcopterPlant10DOF()
    cmd = np.full(4, plant.hover_motor_speed)
    bank_deg = np.linspace(-20, 20, 81)
    for alt, color in [(0.02, 'tab:blue'), (0.10, 'tab:orange'), (0.50, 'tab:green')]:
        moments = []
        for b in bank_deg:
            s = plant.hover_state()
            s[2] = alt
            s[6] = np.radians(b)
            moments.append(plant.derivatives(s, cmd)[9])   # roll accel
        ax.plot(bank_deg, np.degrees(moments), lw=2, color=color,
                label=f'altitude {alt:.2f} m')
    ax.axhline(0.0, color='k', lw=0.8)
    ax.set_xlabel('roll angle [deg]')
    ax.set_ylabel('roll acceleration [deg/s$^2$]')
    ax.set_title('[C] Per-rotor asymmetry: negative slope = self-leveling')
    ax.grid(alpha=0.3); ax.legend(fontsize=8)

    # ---------------- [D] hover trim shift ----------------
    ax = fig.add_subplot(gs[1, 1])
    plant = QuadcopterPlant10DOF()
    altitudes = np.linspace(0.0, 0.8, 200)
    trim_rpm = []
    for alt in altitudes:
        # Solve for the motor speed that exactly holds altitude at this height.
        body_to_world = plant.rotation_matrix(0.0, 0.0, 0.0)
        h = plant.rotor_heights(alt, body_to_world)
        gain = plant.ground_effect_ratio(h).mean()
        speed = np.sqrt(plant.m * plant.g / (4.0 * plant.thrust_coeff * gain))
        trim_rpm.append(speed * 60.0 / (2.0 * np.pi))
    trim_rpm = np.array(trim_rpm)
    oge_rpm = plant.hover_motor_speed * 60.0 / (2.0 * np.pi)
    ax.plot(altitudes, trim_rpm, lw=2, color='tab:purple')
    ax.axhline(oge_rpm, color='gray', ls='--', lw=1.2,
               label=f'free-air hover ({oge_rpm:.0f} rpm)')
    ax.set_xlabel('altitude [m]'); ax.set_ylabel('hover trim speed [rpm]')
    ax.set_title('[D] Throttle needed to hold altitude')
    ax.grid(alpha=0.3); ax.legend(fontsize=8)

    fig.suptitle('Ground Effect: Model, Landing Impact, and Control Consequences',
                 fontsize=14, fontweight='bold')
    fig.savefig('ground_effect_study.png', dpi=120, bbox_inches='tight')

    # ---------------- numeric summary ----------------
    print("[A] K_G at the ground (z=0):  "
          f"exponential={p_exp.ground_effect_ratio(np.array([0.0]))[0]:.3f}  "
          f"Cheeseman(clamped)={p_che.ground_effect_ratio(np.array([0.0]))[0]:.3f}")
    print(f"[A] K_G at one rotor radius:  "
          f"{p_exp.ground_effect_ratio(np.array([p_exp.rotor_radius]))[0]:.3f}")

    print(f"[B] touchdown WITH ground effect:  t={t_on:.2f} s, "
          f"impact speed {v_on:.3f} m/s")
    print(f"[B] touchdown WITHOUT:             t={t_off:.2f} s, "
          f"impact speed {v_off:.3f} m/s")
    print(f"[B] IGE delays touchdown by {t_on - t_off:+.2f} s and changes "
          f"impact speed by {v_on - v_off:+.3f} m/s")

    plant = QuadcopterPlant10DOF()
    s = plant.hover_state(); s[2] = 0.02; s[6] = np.radians(10.0)
    roll_accel = plant.derivatives(s, np.full(4, plant.hover_motor_speed))[9]
    print(f"[C] roll accel at +10 deg bank, 2 cm alt: "
          f"{np.degrees(roll_accel):+.2f} deg/s^2 (negative = restoring)")

    print(f"[D] hover trim at ground: {trim_rpm[0]:.0f} rpm vs "
          f"{oge_rpm:.0f} rpm free air "
          f"({100*(trim_rpm[0]-oge_rpm)/oge_rpm:+.1f}%)")
    print("\nSaved ground_effect_study.png")


if __name__ == '__main__':
    main()
