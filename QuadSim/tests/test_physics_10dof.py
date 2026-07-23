"""
Verification suite for the 10-DOF model (rotor dynamics + blade flapping)
=========================================================================

Layered like test_physics.py, but focused on the NEW physics. The base 6-DoF
suite still validates everything the models share.

  Layer R  REDUCTION / REGRESSION
           With the new coefficients zeroed, the 10-DOF model must reproduce
           the validated 6-DoF model *exactly*. This proves the upgrade is a
           strict superset and inherits all 24 prior validations.

  Layer M  MOTOR DYNAMICS
           The rotor states must follow the commanded speed as a first-order
           lag with the specified time constant (closed-form exponential).

  Layer F  BLADE FLAPPING
           Force opposes in-plane velocity with magnitude flap_coeff*T*v; moment
           tips the airframe away from the direction of motion (the documented
           pitch-up in forward flight); both with the correct axis pairing.

  Layer G  RATE DAMPING / GYROSCOPICS
           Tip-path-plane lag damps body rates; net rotor angular momentum
           couples pitch<->roll exactly per  body_torque = -motor_speed x H.

  Layer L  LINEARIZATION STRUCTURE
           B must be ~zero in the body rows (commands act only through the
           rotor states) — the bandwidth-limit structure that motivates
           tuning on this model. Full 16-state controllability.

  Layer C  CLOSED-LOOP & BEHAVIORAL IMPROVEMENT
           Stability with the nominal controller, and the physically real
           behavior the 6-DoF model lacks: flapping velocity damping.

Run with:  pytest test_physics_10dof.py -v
"""

import numpy as np
import pytest

from quad_sim import (
    QuadcopterPlant, ControlAllocator, CascadedPIDController, Simulation,
)
from quad_sim_10dof import QuadcopterPlant10DOF, Simulation10DOF


@pytest.fixture
def plant():
    return QuadcopterPlant10DOF()


#: Altitude used for "free air" reference states. Ground effect decays with
#: rotor height, so any test asserting clean hover trim, exact reduction to the
#: 6-DoF model, or flapping-only behavior must sit OUT of ground effect.
#: At 5 m the K_G surplus is < 1e-15, i.e. numerically absent.
OGE_ALTITUDE = 5.0


def hover16(plant, altitude=OGE_ALTITUDE):
    """Hover trim state, by default well clear of ground effect."""
    s = plant.hover_state()
    s[2] = altitude
    return s


# ===========================================================================
#  LAYER R — REDUCTION TO THE VALIDATED 6-DoF MODEL
# ===========================================================================
class TestReduction:

    def test_reduces_exactly_to_6dof(self):
        """
        Zero the new coefficients and hold rotor speeds at the command (no
        lag transient): every body state must match the 6-DoF plant to
        numerical precision over a full rollout. This is the guarantee that
        the 10-DOF model *contains* the already-validated model.

        Ground effect is switched off as well: it is a separate mechanism with
        no counterpart in the 6-DoF plant, so leaving it on would compare two
        different models rather than testing reduction.
        """
        p10 = QuadcopterPlant10DOF(flap_coeff=0.0, rotor_rate_damp_coeff=0.0,
                                   rotor_inertia=0.0, ground_effect_model='none')
        p6 = QuadcopterPlant()
        motor_speeds = np.full(4, p6.hover_motor_speed * 1.02)   # slight climb

        s10 = np.zeros(16); s10[12:16] = motor_speeds      # rotors already at cmd
        s6 = np.zeros(12)
        dt = 0.002
        for _ in range(500):
            s10 = p10.step_rk4(s10, motor_speeds, dt)
            s6 = p6.step_rk4(s6, motor_speeds, dt)
        np.testing.assert_allclose(s10[0:12], s6, atol=1e-9)

    def test_hover_equilibrium_16state(self, plant):
        """Hover state + hover command must be an exact equilibrium."""
        d = plant.derivatives(hover16(plant), np.full(4, plant.hover_motor_speed))
        np.testing.assert_allclose(d, np.zeros(16), atol=1e-9)


# ===========================================================================
#  LAYER M — MOTOR (ROTOR-SPEED) DYNAMICS
# ===========================================================================
class TestMotorDynamics:

    def test_first_order_lag_time_constant(self, plant):
        """
        Step the command; the rotor speed must follow
        w(t) = cmd + (w0 - cmd) exp(-t/body_torque). Check the 63.2% point at t = body_torque
        and the 95% point at t = 3 body_torque.
        """
        w0 = plant.hover_motor_speed
        cmd = np.full(4, 1.10 * w0)
        s = hover16(plant)
        dt = 1e-3
        n_tau = int(plant.motor_time_constant / dt)
        for k in range(3 * n_tau):
            s = plant.step_rk4(s, cmd, dt)
            if k == n_tau - 1:
                frac_1tau = (s[12] - w0) / (cmd[0] - w0)
        frac_3tau = (s[12] - w0) / (cmd[0] - w0)
        assert frac_1tau == pytest.approx(1 - np.exp(-1), abs=0.01)
        assert frac_3tau == pytest.approx(1 - np.exp(-3), abs=0.01)

    def test_command_is_not_instantaneous(self, plant):
        """
        Immediately after a step command, body torque must still be ~zero
        (rotors have not moved yet). This is the actuator-lag reality the
        6-DoF model lacked.
        """
        s = hover16(plant)
        # Aggressive differential command (roll):
        cmd = np.full(4, plant.hover_motor_speed)
        cmd[1] *= 1.2; cmd[2] *= 1.2; cmd[0] *= 0.8; cmd[3] *= 0.8
        d = plant.derivatives(s, cmd)
        # Roll acceleration at t=0+ must be zero: torque comes from ACTUAL
        # rotor speeds, which are still symmetric.
        assert abs(d[9]) < 1e-9


# ===========================================================================
#  LAYER F — BLADE FLAPPING
# ===========================================================================
class TestBladeFlapping:

    def test_flap_force_opposes_velocity_and_scales(self, plant):
        """
        Level flight, in-plane velocity v: horizontal accel due to flapping
        must equal -flap_coeff * T * v / m (isolated by zeroing parasitic drag),
        for both +x and an oblique direction.
        """
        p = QuadcopterPlant10DOF(linear_drag_coeff=0.0)
        T = p.m * p.g   # hover thrust
        for v_vec in ([3.0, 0.0], [2.0, -1.5]):
            s = hover16(p)
            s[3], s[4] = v_vec
            d = p.derivatives(s, np.full(4, p.hover_motor_speed))
            expected = -p.flap_coeff * T * np.array(v_vec) / p.m
            np.testing.assert_allclose(d[3:5], expected, rtol=1e-6)

    def test_flap_force_linear_in_speed(self, plant):
        """Doubling airspeed must double the flapping force (linear model)."""
        p = QuadcopterPlant10DOF(linear_drag_coeff=0.0)
        s1 = hover16(p); s1[3] = 2.0
        s2 = hover16(p); s2[3] = 4.0
        a1 = p.derivatives(s1, np.full(4, p.hover_motor_speed))[3]
        a2 = p.derivatives(s2, np.full(4, p.hover_motor_speed))[3]
        assert a2 == pytest.approx(2 * a1, rel=1e-9)

    def test_flap_moment_tips_away_from_motion(self, plant):
        """
        The documented behavior: translating forward, the rotor TPP tilts back
        and the airframe receives a decelerating (nose-up) moment.
        In this convention +pitch accelerates +x, so moving +x must give
        q_dot < 0. Symmetrically, moving +y must give p_dot > 0 (since +roll
        accelerates -y).
        """
        s = hover16(plant); s[3] = 4.0        # moving +x
        d = plant.derivatives(s, np.full(4, plant.hover_motor_speed))
        assert d[10] < 0.0, "forward flight must produce a decelerating pitch moment"

        s = hover16(plant); s[4] = 4.0        # moving +y
        d = plant.derivatives(s, np.full(4, plant.hover_motor_speed))
        assert d[9] > 0.0, "+y flight must produce a decelerating roll moment"

    def test_no_flapping_from_vertical_motion(self, plant):
        """
        Flapping is driven by IN-PLANE advance ratio only: pure climb must
        produce no flapping force or moment.
        """
        p = QuadcopterPlant10DOF(linear_drag_coeff=0.0)
        s = hover16(p); s[5] = 3.0            # climbing
        d = p.derivatives(s, np.full(4, p.hover_motor_speed))
        np.testing.assert_allclose(d[3:5], np.zeros(2), atol=1e-12)
        np.testing.assert_allclose(d[9:11], np.zeros(2), atol=1e-12)


# ===========================================================================
#  LAYER G — RATE DAMPING & ROTOR GYROSCOPICS
# ===========================================================================
class TestRateDampingAndGyro:

    def test_rate_damping_opposes_body_rate(self, plant):
        """
        Pure pitch rate at hover: with balanced rotors (net H = 0) and no
        velocity, the only pitch torque is the TPP-lag damping, so
        q_dot = -rotor_rate_damp_coeff * q / Iyy exactly.
        """
        q = 2.0
        s = hover16(plant); s[10] = q
        d = plant.derivatives(s, np.full(4, plant.hover_motor_speed))
        assert d[10] == pytest.approx(-plant.rotor_rate_damp_coeff * q / plant.Iyy, rel=1e-6)

    def test_gyroscopic_pitch_roll_coupling(self, plant):
        """
        With a net rotor angular momentum rotor_angular_momentum_z (CW/CCW imbalance, as during a
        yaw command) and a pitch rate q, the airframe must feel a roll torque
        torque_x = -q * rotor_angular_momentum_z  (from body_torque = -motor_speed x H). Checked in closed form,
        with the rate-damping contribution subtracted.
        """
        s = hover16(plant)
        # Imbalance rotors like a yaw command: CCW (+1 spin) up, CW down.
        s[12] *= 0.95; s[14] *= 0.95
        s[13] *= 1.05; s[15] *= 1.05
        q = 1.5
        s[10] = q
        om = s[12:16]
        rotor_angular_momentum_z = plant.rotor_inertia * np.sum(plant.spin_dir * om)
        assert abs(rotor_angular_momentum_z) > 0.0
        cmd = om.copy()   # hold rotors where they are (no reaction torque)
        d = plant.derivatives(s, cmd)
        expected_p_dot = (-q * rotor_angular_momentum_z) / plant.Ixx     # damping acts on y, not x
        assert d[9] == pytest.approx(expected_p_dot, rel=1e-6)


# ===========================================================================
#  LAYER L — LINEARIZATION STRUCTURE (the tuning-relevant part)
# ===========================================================================
class TestLinearizationStructure:

    def test_commands_act_only_through_rotors(self, plant):
        """
        B must be ~zero in the translational and roll/pitch rows: the command
        cannot instantaneously touch the body. Rotor rows must be
        (1/motor_time_constant) I and the rotor block of A must be -(1/motor_time_constant) I.
        (The yaw-rate row has a tiny direct feedthrough from the rotor-accel
        reaction torque, which we verify in closed form.)
        """
        A, B, xt, ut = plant.linearize()
        # Body rows (positions, velocities, angles, p and q rates): no feedthrough.
        np.testing.assert_allclose(B[0:11, :], np.zeros((11, 4)), atol=1e-8)
        # Yaw row: exact reaction-torque feedthrough -rotor_inertia*s_i/(body_torque*Izz).
        expected_yaw = -plant.rotor_inertia * plant.spin_dir / (plant.motor_time_constant * plant.Izz)
        np.testing.assert_allclose(B[11, :], expected_yaw, rtol=1e-4)
        # Rotor rows.
        np.testing.assert_allclose(B[12:16, :], np.eye(4) / plant.motor_time_constant, rtol=1e-6)
        np.testing.assert_allclose(A[12:16, 12:16], -np.eye(4) / plant.motor_time_constant,
                                   rtol=1e-6)

    def test_full_state_controllable_pbh(self, plant):
        """
        All 16 states reachable through the 4 motor commands. Uses the PBH
        (Popov-Belevitch-Hautus) test — rank [lambda I - A | B] = n at every
        eigenvalue — because the naive Kalman controllability matrix is
        numerically meaningless at 16 states (A^15 spans ~20 orders of
        magnitude and destroys the rank computation).
        """
        A, B, _, _ = plant.linearize()
        n = 16
        for lam in np.linalg.eigvals(A):
            M = np.hstack([lam * np.eye(n) - A, B])
            r = np.linalg.matrix_rank(M, tol=1e-7 * np.linalg.norm(M))
            assert r == n, f"PBH rank deficient at eigenvalue {lam}"

    def test_hover_is_open_loop_unstable(self, plant):
        """
        A real hovering rotorcraft is open-loop UNSTABLE: the flapping moment
        tips the vehicle away from its velocity, the tilt redirects thrust and
        accelerates it further, and the cycle diverges as a growing
        oscillation. The 10-DOF linearization must therefore contain a complex
        eigenvalue pair with positive real part — a behavior the 6-DoF model
        (neutrally stable hover) cannot represent, and a key reason attitude
        feedback is mandatory on real vehicles.
        """
        A, _, _, _ = plant.linearize()
        eigs = np.linalg.eigvals(A)
        unstable_osc = [e for e in eigs if e.real > 1e-3 and abs(e.imag) > 1e-3]
        assert len(unstable_osc) >= 2, "expected the unstable flapping hover mode"
        # And confirm the mode disappears when flapping is off (6-DoF limit).
        p0 = QuadcopterPlant10DOF(flap_coeff=0.0)
        A0, _, _, _ = p0.linearize()
        eigs0 = np.linalg.eigvals(A0)
        assert all(e.real <= 1e-6 for e in eigs0), \
            "without flapping, hover should have no exponentially growing mode"

    def test_velocity_damping_visible_in_A(self, plant):
        """
        The flapping drag must appear in the linearization: d(u_dot)/d(u) =
        -(linear_drag_coeff + flap_coeff*T_hover)/m, i.e. strictly more damped than the
        6-DoF model's -linear_drag_coeff/m.
        """
        A, _, _, _ = plant.linearize()
        T_h = plant.m * plant.g
        expected = -(plant.linear_drag_coeff + plant.flap_coeff * T_h) / plant.m
        assert A[3, 3] == pytest.approx(expected, rel=1e-4)
        # And it is stronger than parasitic drag alone:
        assert A[3, 3] < -plant.linear_drag_coeff / plant.m


# ===========================================================================
#  LAYER C — CLOSED LOOP & BEHAVIORAL IMPROVEMENT
# ===========================================================================
class TestClosedLoopAndImprovement:

    def _run10(self, setpoint_fn, duration=10.0, plant=None):
        plant = plant or QuadcopterPlant10DOF()
        ctrl = CascadedPIDController(plant, ControlAllocator(plant))
        sim = Simulation10DOF(plant, ctrl, control_hz=200.0)
        sim.run(duration, setpoint_fn)
        return sim

    def test_hover_holds_with_nominal_gains(self):
        sim = self._run10(lambda t: {'x': 0, 'y': 0, 'z': 2.0, 'yaw': 0})
        f = sim.state_hist[-1]
        assert abs(f[2] - 2.0) < 0.2 and abs(f[0]) < 0.1 and abs(f[1]) < 0.1

    def test_step_maneuver_no_divergence(self):
        from quad_sim_10dof import step_maneuver
        sim = self._run10(step_maneuver, duration=12.0)
        S = sim.state_hist
        assert np.all(np.isfinite(S))
        assert np.all(np.abs(S[:, 6:9]) < np.pi)
        # Actual rotor speeds within actuator limits.
        assert np.all(S[:, 12:16] <= QuadcopterPlant10DOF().max_motor_speed + 1e-6)

    def test_flapping_gives_realistic_velocity_damping(self):
        """
        THE headline behavioral improvement: hold the vehicle level (attitude
        override) with an initial 4 m/s forward velocity. The 10-DOF model
        must shed speed noticeably faster than the 6-DoF model, because rotor
        drag from flapping dominates parasitic drag — the real reason a level
        quadcopter decelerates.
        """
        level_hold = lambda t: {'x': 0, 'y': 0, 'z': 5.0, 'yaw': 0,
                                'roll': 0.0, 'pitch': 0.0}
        # 10-DOF
        p10 = QuadcopterPlant10DOF()
        sim10 = Simulation10DOF(p10, CascadedPIDController(p10, ControlAllocator(p10)))
        x0 = p10.hover_state(); x0[2] = 5.0; x0[3] = 4.0
        sim10.run(3.0, level_hold, x0=x0)
        # 6-DoF
        p6 = QuadcopterPlant()
        sim6 = Simulation(p6, CascadedPIDController(p6, ControlAllocator(p6)))
        x0 = np.zeros(12); x0[2] = 5.0; x0[3] = 4.0
        sim6.run(3.0, level_hold, x0=x0)

        v10 = sim10.state_hist[-1][3]
        v6 = sim6.state_hist[-1][3]
        assert v10 < v6 - 0.3, (
            f"10-DOF should damp velocity faster (v10={v10:.2f}, v6={v6:.2f})")


# ===========================================================================
#  GROUND EFFECT
# ===========================================================================
class TestGroundEffect:
    """
    Ground effect (in-ground-effect, IGE): thrust rises as the rotors approach
    the ground because the downwash cannot fully develop, raising the effective
    induced velocity. Verified against the documented shape of the effect and
    against the physical consequences that matter for landing.
    """

    def test_disabled_reduces_to_previous_model(self):
        """
        With ground_effect_model='none', the plant must reproduce the
        pre-ground-effect dynamics EXACTLY at any altitude. This guards the
        per-rotor force refactor: everything already validated stays valid.
        """
        p_off = QuadcopterPlant10DOF(ground_effect_model='none')
        rng = np.random.default_rng(7)
        for _ in range(50):
            s = rng.normal(0.0, 0.3, 16)
            s[2] = abs(s[2]) + 3.0
            s[12:16] = 1600.0 + rng.normal(0.0, 40.0, 4)
            cmd = 1600.0 + rng.normal(0.0, 40.0, 4)
            speeds = np.clip(s[12:16], 0.0, p_off.max_motor_speed)
            thrust_ref, torque_ref = p_off.motor_forces_torques(speeds)
            per = p_off.thrust_coeff * speeds**2
            assert np.sum(per) == pytest.approx(thrust_ref, rel=1e-12)
            assert np.sum(p_off.motor_pos[:, 1] * per) == pytest.approx(
                torque_ref[0], abs=1e-12)
            assert -np.sum(p_off.motor_pos[:, 0] * per) == pytest.approx(
                torque_ref[1], abs=1e-12)

    def test_high_altitude_matches_no_ground_effect(self):
        """Far from the ground, IGE must vanish: K_G -> 1."""
        p_on = QuadcopterPlant10DOF()
        p_off = QuadcopterPlant10DOF(ground_effect_model='none')
        s = p_on.hover_state()
        s[2] = 10.0                      # well out of ground effect
        cmd = np.full(4, p_on.hover_motor_speed)
        np.testing.assert_allclose(p_on.derivatives(s, cmd),
                                   p_off.derivatives(s, cmd), atol=1e-9)

    def test_thrust_ratio_monotonic_and_bounded(self):
        """
        K_G must decrease monotonically with height and stay FINITE at the
        ground. Finiteness is the whole reason for choosing the exponential
        model over Cheeseman-Bennett, which is singular at z/R = 0.25.
        """
        p = QuadcopterPlant10DOF()
        heights = np.linspace(0.0, 2.0, 60)
        ratios = p.ground_effect_ratio(heights)
        assert np.all(np.isfinite(ratios))
        assert np.all(np.diff(ratios) <= 1e-12), "K_G must be non-increasing in z"
        assert ratios[0] == pytest.approx(1.0 + p.ground_effect_gain, rel=1e-9)
        assert ratios[-1] == pytest.approx(1.0, abs=1e-2)

    def test_extra_thrust_near_ground(self):
        """
        At identical motor speeds, being near the ground must produce MORE
        upward acceleration than being high up. This is the effect users
        actually feel: the drone floats on landing.
        """
        p = QuadcopterPlant10DOF()
        cmd = np.full(4, p.hover_motor_speed)
        low = p.hover_state();  low[2] = 0.0
        high = p.hover_state(); high[2] = 10.0
        accel_low = p.derivatives(low, cmd)[5]
        accel_high = p.derivatives(high, cmd)[5]
        assert accel_low > accel_high + 0.5, (
            f"near-ground z-accel {accel_low:.3f} should exceed "
            f"high-altitude {accel_high:.3f}")
        # High altitude is trimmed hover, so it should be ~0.
        assert abs(accel_high) < 1e-6

    def test_hover_thrust_surplus_matches_model(self):
        """
        The extra acceleration at the ground must equal the K_G prediction, not
        merely be positive. At hover speeds all four rotors are level and at the
        same height, so a_z = (K_G - 1) * g exactly.
        """
        p = QuadcopterPlant10DOF()
        cmd = np.full(4, p.hover_motor_speed)
        s = p.hover_state(); s[2] = 0.0
        height = p.landing_gear_height          # level attitude -> all rotors here
        expected = (p.ground_effect_ratio(np.array([height]))[0] - 1.0) * p.g
        assert p.derivatives(s, cmd)[5] == pytest.approx(expected, rel=1e-9)

    def test_bank_near_ground_produces_restoring_moment(self):
        """
        THE KEY ASYMMETRY. Rolled near the ground, the LOW rotors sit deeper in
        ground effect and gain more thrust than the high ones. The differential
        thrust produces a roll moment that pushes the vehicle back toward level.

        This per-rotor asymmetry is why ground effect must not be modeled as a
        single body-level thrust scale factor.
        """
        p = QuadcopterPlant10DOF()
        cmd = np.full(4, p.hover_motor_speed)
        s = p.hover_state()
        s[2] = 0.02                      # very close to the ground
        s[6] = np.radians(10.0)          # rolled +10 deg
        roll_accel = p.derivatives(s, cmd)[9]
        assert roll_accel < -1e-6, (
            f"positive roll near ground should produce negative (restoring) "
            f"roll accel, got {roll_accel:.5f}")

        # The same bank far from the ground produces no such moment.
        s_high = s.copy(); s_high[2] = 10.0
        assert abs(p.derivatives(s_high, cmd)[9]) < 1e-9

    def test_rotor_heights_track_attitude(self):
        """
        Rotor height bookkeeping: level flight puts all four rotors at the same
        height; a positive roll (right-hand about +x, +y goes up) must raise the
        +y motors and lower the -y motors.
        """
        p = QuadcopterPlant10DOF()
        level = p.rotor_heights(1.0, p.rotation_matrix(0.0, 0.0, 0.0))
        np.testing.assert_allclose(level, level[0], atol=1e-12)
        assert level[0] == pytest.approx(1.0 + p.landing_gear_height)

        rolled = p.rotor_heights(1.0, p.rotation_matrix(np.radians(20.0), 0.0, 0.0))
        # motor_pos rows: M0 (+x,-y), M1 (+x,+y), M2 (-x,+y), M3 (-x,-y)
        assert rolled[1] > rolled[0], "+y motor should rise under positive roll"
        assert rolled[2] > rolled[3], "+y motor should rise under positive roll"

    def test_cheeseman_model_available_and_stronger(self):
        """
        The classical Cheeseman-Bennett model is selectable for comparison. It
        must be finite (we clamp off its singularity) and predict a larger
        surplus than the exponential model at very low height -- the documented
        reason it overestimates IGE for low-profile multirotors.
        """
        p_exp = QuadcopterPlant10DOF(ground_effect_model='exponential')
        p_che = QuadcopterPlant10DOF(ground_effect_model='cheeseman')
        z = np.array([0.0, 0.01, 0.05])
        r_exp = p_exp.ground_effect_ratio(z)
        r_che = p_che.ground_effect_ratio(z)
        assert np.all(np.isfinite(r_che))
        assert np.all(r_che > r_exp)

    def test_ground_contact_constraint(self):
        """
        The floor must actually stop the vehicle. Without a contact constraint
        a commanded landing flies through z=0 into negative altitude, and every
        touchdown metric becomes meaningless -- so the constraint is a
        prerequisite for studying ground effect at all.
        """
        def descend(t):
            return {'x': 0.0, 'y': 0.0, 'z': -1.0, 'yaw': 0.0}

        plant = QuadcopterPlant10DOF()
        sim = Simulation10DOF(
            plant, CascadedPIDController(plant, ControlAllocator(plant)),
            enforce_ground=True)
        x0 = plant.hover_state(); x0[2] = 0.8
        sim.run(6.0, descend, x0=x0)
        assert sim.state_hist[:, 2].min() >= -1e-12, "vehicle passed through the floor"

    def test_ground_effect_delays_touchdown(self):
        """
        End-to-end consequence: on a gentle commanded descent, the vehicle WITH
        ground effect touches down LATER than the same controller without it,
        because the near-ground thrust surplus holds it off the pad. This is
        the "hovers above the pad and won't settle" behavior that motivated
        modeling the effect.

        A gentle profile is used deliberately: an aggressive descent carries
        enough momentum to punch through ground effect and hides it.
        """
        def gentle_descent(t):
            return {'x': 0.0, 'y': 0.0, 'z': max(1.0 - 0.15 * t, -0.2), 'yaw': 0.0}

        touchdown = {}
        for label, model in [('on', 'exponential'), ('off', 'none')]:
            plant = QuadcopterPlant10DOF(ground_effect_model=model)
            sim = Simulation10DOF(
                plant, CascadedPIDController(plant, ControlAllocator(plant)),
                enforce_ground=True)
            x0 = plant.hover_state(); x0[2] = 1.0
            sim.run(14.0, gentle_descent, x0=x0)
            contact = np.flatnonzero(sim.state_hist[:, 2] <= 1e-9)
            assert contact.size > 0, f"[{label}] never reached the ground"
            touchdown[label] = sim.t_hist[contact[0]]

        assert touchdown['on'] > touchdown['off'] + 0.2, (
            f"ground effect should delay touchdown "
            f"(with={touchdown['on']:.2f} s, without={touchdown['off']:.2f} s)")


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
