"""
Export reference trajectories for the Rust control crate's plant-port tests
==========================================================================

The Rust firmware's controller is checked in `cargo test` against a Rust port
of this simulator's ROTATIONAL dynamics (ripwing/crates/control/tests/). This
script produces the Python side of that comparison:

  openloop            the 10-DOF plant driven by a scripted motor-command
                      sequence. Checks the plant PORT in isolation: the Rust
                      plant must reproduce this to ~machine precision.
  <scenario>_<gains>  closed-loop step responses of the Python
                      CascadedPIDController's inner loop (attitude P -> rate
                      PID -> allocator) on the same plant. The Rust
                      AttitudeRateController + mixer must reproduce these.

Rotational subsystem only
-------------------------
The plant is QuadcopterPlant10DOF with `flap_coeff = 0` and ground effect off.
Those two terms are the only paths by which translation (velocity, height)
feeds the rotational dynamics; with them zeroed the attitude/rate/rotor states
evolve independently of position, so a 10-state Rust port (Euler angles, body
rates, rotor speeds) is EXACT rather than approximate. What remains is the
part that bounds the rate loop: motor lag, rotor rate damping, rotor
gyroscopics, yaw reaction torque, and the nonlinear rigid-body equations.

Thrust: the altitude PID is disabled (gains zeroed), so the commanded thrust
is the controller's own gravity feed-forward  m*g / max(cos(phi)cos(theta), 0.5).
That keeps the closed loop free of translational state while still exercising
the real `compute_commands` code path unmodified.

Output format (self-describing, so the Rust test never hard-codes a gain):
  @plant key=value ...          plant parameters
  @case  name key=value ...     per-case controller parameters
  name,t,phi_d,theta_d,psi_d,phi,theta,psi,p,q,r,w0,w1,w2,w3,c0,c1,c2,c3
Rows are logged BEFORE each step (as Simulation does): state at t and the
command computed from it. Motor columns use THIS model's motor order
(M0 FR, M1 FL, M2 RL, M3 RR); w = actual rotor speed, c = command [rad/s].

Run from the RipWing repo root:
    python -m QuadSim.analysis.export_step_references
"""

from pathlib import Path

import numpy as np

from QuadSim.models.quad_sim import Simulation
from QuadSim.models.quad_sim_10dof import QuadcopterPlant10DOF
from QuadSim.tuning.pid_tuning import DEFAULTS, build_controller

OUT = (Path(__file__).resolve().parents[2]
       / 'ripwing/crates/control/tests/data/quadsim_step_refs.csv')
GAINS_JSON = Path(__file__).resolve().parents[1] / 'results/tuning_data/tuned_gains.json'

CONTROL_HZ = 200.0          # the rate the gains were tuned at
DURATION = 2.5
T_STEP, T_RETURN = 0.3, 1.2  # same doublet timing as tuning/pid_tuning.py
STEP_DEG = 15.0
YAW_STEP_DEG = 30.0


class RotationalPlant(QuadcopterPlant10DOF):
    """10-DOF plant with the translation->rotation couplings switched off."""

    def __init__(self):
        super().__init__(flap_coeff=0.0, ground_effect_model='none')


def fmt(x):
    return repr(float(x))


def row(name, t, sp, state, cmd):
    vals = [t, *sp, *state[6:12], *state[12:16], *cmd]
    return name + ',' + ','.join(fmt(v) for v in vals)


def plant_header(p):
    kv = dict(mass=p.m, g=p.g, arm_length=p.L, Ixx=p.Ixx, Iyy=p.Iyy, Izz=p.Izz,
              k_f=p.thrust_coeff, k_m=p.drag_torque_coeff,
              max_omega=p.max_motor_speed, omega_hover=p.hover_motor_speed,
              motor_time_constant=p.motor_time_constant,
              rotor_rate_damp_coeff=p.rotor_rate_damp_coeff,
              rotor_inertia=p.rotor_inertia)
    return '@plant ' + ' '.join(f'{k}={fmt(v)}' for k, v in kv.items())


# ---------------------------------------------------------------------------
#  Open loop: plant port check
# ---------------------------------------------------------------------------
def openloop_cmd(t, h):
    """Scripted pulses exciting each axis, then all at once.

    Amplitudes push the attitude to a few tens of degrees so the nonlinear
    Euler kinematics and the omega x I*omega term are actually exercised,
    and the differential spin excites rotor gyroscopics / yaw reaction.
    Motor order: M0 FR, M1 FL, M2 RL, M3 RR.
    """
    roll = np.array([-1, 1, 1, -1])    # +tau_x: left side up
    pitch = np.array([-1, -1, 1, 1])   # +tau_y: rear up (FLU)
    yaw = np.array([-1, 1, -1, 1])     # spin_dir: +tau_z
    d = np.zeros(4)
    if 0.05 <= t < 0.20:
        d += 0.03 * roll
    if 0.20 <= t < 0.35:
        d -= 0.03 * roll
    if 0.30 <= t < 0.45:
        d += 0.06 * pitch
    if 0.45 <= t < 0.60:
        d -= 0.03 * pitch
    if 0.50 <= t < 0.90:
        d += 0.25 * yaw
    if 0.70 <= t < 0.80:
        d += 0.05 * roll - 0.04 * pitch
    if t >= 1.0:
        d += 2.0               # 3x hover > max_omega: exercises the speed clamp
    return h * (1.0 + d)


def openloop(lines):
    plant = RotationalPlant()
    dt = 1.0 / CONTROL_HZ
    state = plant.hover_state()
    lines.append('@case openloop dt=' + fmt(dt))
    for k in range(int(1.2 / dt)):
        t = k * dt
        cmd = openloop_cmd(t, plant.hover_motor_speed)
        lines.append(row('openloop', t, (0.0, 0.0, 0.0), state, cmd))
        state = plant.step_rk4(state, cmd, dt)


# ---------------------------------------------------------------------------
#  Closed loop: controller check
# ---------------------------------------------------------------------------
def doublet(t, deg):
    return np.radians(deg) if T_STEP <= t < T_RETURN else 0.0


SCENARIOS = {
    'roll':  lambda t: (doublet(t, STEP_DEG), 0.0, 0.0),
    'pitch': lambda t: (0.0, doublet(t, STEP_DEG), 0.0),
    'yaw':   lambda t: (0.0, 0.0, np.radians(YAW_STEP_DEG) if t >= T_STEP else 0.0),
}


def case_header(name, ctrl, dt):
    kv = dict(dt=dt, k_att_roll=ctrl.k_att_roll, k_att_pitch=ctrl.k_att_pitch,
              k_att_yaw=ctrl.k_att_yaw)
    for ax, pid in (('p', ctrl.pid_p), ('q', ctrl.pid_q), ('r', ctrl.pid_r)):
        kv.update({f'{ax}.kp': pid.kp, f'{ax}.ki': pid.ki, f'{ax}.kd': pid.kd,
                   f'{ax}.out_limit': pid.out_limit,
                   f'{ax}.integ_limit': pid.integ_limit,
                   f'{ax}.d_lpf_hz': pid.d_lpf_hz})
    return f'@case {name} ' + ' '.join(f'{k}={fmt(v)}' for k, v in kv.items())


def closed_loop(lines, name, gains, scenario):
    plant = RotationalPlant()
    ctrl = build_controller(plant, gains)
    ctrl.pid_z.kp = ctrl.pid_z.ki = ctrl.pid_z.kd = 0.0   # thrust = feed-forward
    sim = Simulation(plant, ctrl, control_hz=CONTROL_HZ)
    sp_fn = SCENARIOS[scenario]

    def setpoint(t):
        phi, theta, psi = sp_fn(t)
        return {'x': 0.0, 'y': 0.0, 'z': 2.0, 'yaw': psi,
                'phi': phi, 'theta': theta}

    x0 = plant.hover_state()
    x0[2] = 2.0
    sim.run(DURATION, setpoint, x0=x0)

    lines.append(case_header(name, ctrl, sim.dt))
    for t, s, c in zip(sim.t_hist, sim.state_hist, sim.motor_speed_hist):
        lines.append(row(name, t, sp_fn(t), s, c))
    roll = np.degrees(sim.state_hist[:, 6:9])
    print(f'  {name:22s} peak |att| = {np.abs(roll).max(axis=0).round(2)} deg')


def main():
    import json
    tuned = json.loads(GAINS_JSON.read_text())['tuned']

    lines = [
        '# QuadSim reference trajectories for ripwing-control plant-port tests.',
        '# GENERATED by QuadSim/analysis/export_step_references.py -- do not edit.',
        '# Columns: case,t,phi_d,theta_d,psi_d,phi,theta,psi,p,q,r,'
        'w0,w1,w2,w3,c0,c1,c2,c3',
        '# Motor order M0 FR, M1 FL, M2 RL, M3 RR (QuadSim order). Units SI.',
        plant_header(RotationalPlant()),
    ]
    openloop(lines)
    for scenario in ('roll', 'pitch', 'yaw'):
        closed_loop(lines, f'{scenario}_tuned', tuned, scenario)
    closed_loop(lines, 'roll_default', DEFAULTS, 'roll')

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text('\n'.join(lines) + '\n')
    print(f'Wrote {OUT} ({len(lines)} lines)')


if __name__ == '__main__':
    main()
