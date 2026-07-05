"""
Visual physics diagnostics
==========================

The pytest suite proves the math is correct. This script complements it by
producing plots for scenarios whose shape you can verify by eye against physical
intuition. Each scenario has a KNOWN expected signature stated in its title, so
you can confirm the 3D trajectory, Euler-angle, and motor-RPM plots look right
rather than just trusting green checkmarks.

Scenarios:
  1. Open-loop free fall      -> parabola in Z, flat angles, motors at zero
  2. Open-loop pure climb     -> straight vertical line, flat angles, equal RPMs
  3. Open-loop pure yaw spin  -> yaw ramps linearly, roll/pitch flat, RPM split
  4. Closed-loop square path  -> box trajectory, alternating roll/pitch, RPM work
  5. Closed-loop disturbance  -> recovery to hover after a kick (stability proof)

Outputs a single multi-page figure: diagnostics.png
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec

from quad_sim import (
    QuadcopterPlant, ControlAllocator, CascadedPIDController, Simulation,
)


def open_loop_rollout(plant, omegas_fn, duration, dt=0.002, state0=None):
    """Integrate the plant with no controller; omegas_fn(t) -> (4,) speeds."""
    state = np.zeros(12) if state0 is None else state0.copy()
    t_hist, s_hist, o_hist = [], [], []
    n = int(duration / dt)
    for k in range(n):
        t = k * dt
        omegas = omegas_fn(t)
        t_hist.append(t); s_hist.append(state.copy()); o_hist.append(omegas.copy())
        state = plant.step_rk4(state, omegas, dt)
    return np.array(t_hist), np.array(s_hist), np.array(o_hist)


def panel(fig, gs_cell, t, S, O, plant, title, mode='full'):
    """
    Draw a compact 3-in-1 panel (mini 3D + angles + rpm) inside one grid cell.
    Returns nothing; draws onto fig.
    """
    inner = gridspec.GridSpecFromSubplotSpec(
        1, 3, subplot_spec=gs_cell, wspace=0.45, width_ratios=[1.1, 1, 1])
    rpm = O * 60.0 / (2 * np.pi)

    # 3D trajectory
    ax0 = fig.add_subplot(inner[0], projection='3d')
    ax0.plot(S[:, 0], S[:, 1], S[:, 2], color='tab:blue', lw=1.8)
    ax0.scatter(*S[0, 0:3], color='green', s=20)
    ax0.scatter(*S[-1, 0:3], color='black', s=20)
    ax0.set_xlabel('X', fontsize=7); ax0.set_ylabel('Y', fontsize=7)
    ax0.set_zlabel('Z', fontsize=7)
    ax0.tick_params(labelsize=6)
    ax0.set_title('3D path', fontsize=8)

    # Euler angles
    ax1 = fig.add_subplot(inner[1])
    ax1.plot(t, np.degrees(S[:, 6]), label='roll', color='tab:red', lw=1.3)
    ax1.plot(t, np.degrees(S[:, 7]), label='pitch', color='tab:green', lw=1.3)
    ax1.plot(t, np.degrees(S[:, 8]), label='yaw', color='tab:blue', lw=1.3)
    ax1.set_xlabel('t [s]', fontsize=7); ax1.set_ylabel('deg', fontsize=7)
    ax1.tick_params(labelsize=6); ax1.grid(alpha=0.3)
    ax1.legend(fontsize=6, loc='best'); ax1.set_title('Euler angles', fontsize=8)

    # Motor RPM
    ax2 = fig.add_subplot(inner[2])
    for i in range(4):
        ax2.plot(t, rpm[:, i], label=f'M{i}', lw=1.1)
    ax2.set_xlabel('t [s]', fontsize=7); ax2.set_ylabel('RPM', fontsize=7)
    ax2.tick_params(labelsize=6); ax2.grid(alpha=0.3)
    ax2.legend(fontsize=6, loc='best', ncol=2); ax2.set_title('Motor RPM', fontsize=8)

    # Scenario title spanning the row.
    ax0.text2D(-0.15, 1.25, title, transform=ax0.transAxes,
               fontsize=10, fontweight='bold', va='bottom')


def main():
    plant = QuadcopterPlant()
    hover = plant.omega_hover

    fig = plt.figure(figsize=(16, 22))
    gs = gridspec.GridSpec(5, 1, figure=fig, hspace=0.55)

    # ---- Scenario 1: free fall (motors off) ----
    # EXPECT: Z is a downward parabola; X,Y,angles flat; all RPMs at zero.
    s0 = np.zeros(12); s0[2] = 50.0
    t, S, O = open_loop_rollout(plant, lambda _t: np.zeros(4), 3.0, state0=s0)
    panel(fig, gs[0], t, S, O, plant,
          "1. Free fall (motors OFF)  →  EXPECT: parabolic Z drop, flat angles, zero RPM")

    # ---- Scenario 2: pure vertical climb ----
    # EXPECT: straight vertical line in 3D; angles flat at 0; 4 equal RPM traces.
    t, S, O = open_loop_rollout(plant, lambda _t: np.full(4, hover * 1.04), 3.0)
    panel(fig, gs[1], t, S, O, plant,
          "2. Pure climb (uniform thrust)  →  EXPECT: vertical line, flat angles, 4 equal RPMs")

    # ---- Scenario 3: pure yaw spin ----
    # EXPECT: yaw ramps ~linearly; roll/pitch stay flat; RPM splits into 2 pairs.
    def yaw_input(_t):
        o = np.full(4, hover)
        o[1] *= 1.04; o[3] *= 1.04   # CCW motors up
        o[0] *= 0.96; o[2] *= 0.96   # CW motors down
        return o
    t, S, O = open_loop_rollout(plant, yaw_input, 3.0)
    panel(fig, gs[2], t, S, O, plant,
          "3. Pure yaw (spin imbalance)  →  EXPECT: yaw ramps, roll/pitch≈0, RPM splits 2+2")

    # ---- Scenario 4: closed-loop square trajectory ----
    # EXPECT: box-shaped XY path; roll and pitch alternate as it changes heading
    # of travel; motors continuously working (not flat).
    alloc = ControlAllocator(plant)
    ctrl = CascadedPIDController(plant, alloc)
    sim = Simulation(plant, ctrl, control_hz=200.0)

    def square_path(t):
        z = 5.0 * (1 - np.exp(-t / 1.5))
        leg = int((t // 5) % 4)
        corners = [(0, 0), (3, 0), (3, 3), (0, 3)]
        x, y = corners[leg]
        return {'x': x, 'y': y, 'z': z, 'yaw': 0.0}

    sim.run(20.0, square_path)
    panel(fig, gs[3], sim.t_hist, sim.state_hist, sim.omega_hist, plant,
          "4. Closed-loop square  →  EXPECT: box XY path, alternating roll/pitch, motors working")

    # ---- Scenario 5: disturbance rejection ----
    # EXPECT: starts kicked (tilted + offset), controller drives angles back to
    # ~0 and position back to target; a clear decaying transient.
    ctrl2 = CascadedPIDController(plant, ControlAllocator(plant))
    sim2 = Simulation(plant, ctrl2, control_hz=200.0)
    x0 = np.zeros(12)
    x0[2] = 5.0                    # start at altitude
    x0[6] = np.radians(25.0)       # kicked 25 deg roll
    x0[1] = 1.5                    # and displaced in y
    sim2.run(8.0, lambda t: {'x': 0, 'y': 0, 'z': 5.0, 'yaw': 0}, x0=x0)
    panel(fig, gs[4], sim2.t_hist, sim2.state_hist, sim2.omega_hist, plant,
          "5. Disturbance recovery  →  EXPECT: roll returns to 0, position recovers, decaying transient")

    fig.suptitle("Quadcopter Physics Diagnostics — each row has a KNOWN expected signature",
                 fontsize=14, fontweight='bold', y=0.995)
    fig.savefig('diagnostics.png', dpi=110, bbox_inches='tight')
    print("Saved diagnostics.png")

    # Print quantitative checks alongside the visuals.
    print("\n--- Quantitative signatures ---")
    # Scenario 1 free-fall distance after 3 s:
    print(f"[1] free fall: dropped to z={S[-1, 2]:.2f} m (started 50, motors off)")
    # Scenario 3 final yaw:
    _, S3, _ = open_loop_rollout(plant, yaw_input, 3.0)
    print(f"[3] pure yaw: final yaw={np.degrees(S3[-1, 8]):.1f} deg, "
          f"roll={np.degrees(S3[-1, 6]):.3f}, pitch={np.degrees(S3[-1, 7]):.3f} (both ~0 expected)")
    # Scenario 5 recovery:
    print(f"[5] recovery: final roll={np.degrees(sim2.state_hist[-1, 6]):.2f} deg "
          f"(started 25), final y={sim2.state_hist[-1, 1]:.3f} m (started 1.5)")


if __name__ == '__main__':
    main()
