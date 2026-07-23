"""
Physics validation suite for the quadcopter simulator
=====================================================

The philosophy here: a 6-DoF sim can *look* plausible on a plot while being
silently wrong (a flipped sign, a wrong unit, a transposed rotation matrix).
Visual inspection won't catch those. Analytical invariants will.

The tests are layered from most fundamental to most integrated:

  Layer 1  ANALYTICAL INVARIANTS
           Closed-form facts that must hold exactly (to numerical tolerance),
           independent of any controller. These catch sign/unit/frame bugs.
           e.g. "hover thrust exactly cancels gravity", "free-fall acceleration
           is -g", "rotation matrices are orthonormal".

  Layer 2  OPEN-LOOP PHYSICAL BEHAVIOR
           Run the *plant alone* (no controller) with hand-chosen motor inputs
           and assert the qualitative motion matches Newtonian intuition.
           e.g. "extra thrust climbs", "asymmetric thrust rolls the correct
           direction", "gyroscopic coupling has the right sign".

  Layer 3  INTEGRATOR + ENERGY
           Verify the numerics themselves: RK4 vs adaptive solve_ivp agree,
           energy is conserved in a conservative (drag-free) configuration,
           and the linearization matches the nonlinear model near trim.

  Layer 4  CLOSED-LOOP SANITY
           With the controller in the loop, assert the regulated behavior is
           physically reasonable (settles, doesn't diverge, stays within
           actuator limits, respects symmetry).

Run with:  pytest test_physics.py -v
"""

import numpy as np
import pytest

from QuadSim.models.quad_sim import (
    QuadcopterPlant,
    ControlAllocator,
    CascadedPIDController,
    Simulation,
)


# Convenience: a fresh plant for each test (no shared mutable state).
@pytest.fixture
def plant():
    return QuadcopterPlant()


# Standard hover input: all four motors at the gravity-cancelling speed.
def hover_omegas(plant):
    return np.full(4, plant.omega_hover)


# ===========================================================================
#  LAYER 1 — ANALYTICAL INVARIANTS
# ===========================================================================
class TestAnalyticalInvariants:
    """Closed-form facts. If any of these fail, there is a sign/unit/frame bug."""

    def test_hover_cancels_gravity(self, plant):
        """
        At hover speed with level attitude and zero velocity, the net linear
        acceleration must be exactly zero — thrust cancels gravity. This is the
        single most important invariant; if it fails, nothing else matters.
        """
        state = np.zeros(12)
        d = plant.derivatives(state, hover_omegas(plant))
        accel = d[3:6]
        np.testing.assert_allclose(accel, np.zeros(3), atol=1e-9)

    def test_hover_produces_no_torque(self, plant):
        """
        Symmetric hover thrust with alternating spin directions must produce
        zero net torque about every body axis (roll, pitch, AND yaw). A yaw
        torque here would mean the spin-direction signs are wrong.
        """
        thrust, tau = plant.motor_forces_torques(hover_omegas(plant))
        np.testing.assert_allclose(tau, np.zeros(3), atol=1e-9)

    def test_freefall_is_minus_g(self, plant):
        """With motors off, vertical acceleration must equal -g exactly."""
        state = np.zeros(12)
        d = plant.derivatives(state, np.zeros(4))
        assert d[5] == pytest.approx(-plant.g, abs=1e-9)
        # And no horizontal acceleration at rest.
        np.testing.assert_allclose(d[3:5], np.zeros(2), atol=1e-9)

    def test_thrust_scales_with_omega_squared(self, plant):
        """
        Aerodynamic thrust must scale with omega^2 (f = k_f omega^2). Doubling
        every motor speed must quadruple total thrust.
        """
        t1, _ = plant.motor_forces_torques(np.full(4, 500.0))
        t2, _ = plant.motor_forces_torques(np.full(4, 1000.0))
        assert t2 == pytest.approx(4.0 * t1, rel=1e-12)

    def test_rotation_matrix_orthonormal(self, plant):
        """
        Any valid rotation matrix must be orthonormal (R R^T = I) with
        determinant +1. Test across a spread of nontrivial angles.
        """
        for phi, theta, psi in [(0.3, -0.2, 1.1),
                                (-0.5, 0.4, -2.0),
                                (0.1, 0.1, 0.1)]:
            R = plant.rotation_matrix(phi, theta, psi)
            np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
            assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-12)

    def test_rotation_matrix_known_values(self, plant):
        """
        A 90-degree yaw must map body-x (forward) onto world-y. This pins down
        the sign/handedness convention, not just orthonormality.
        """
        R = plant.rotation_matrix(0.0, 0.0, np.pi / 2)
        body_x = np.array([1.0, 0.0, 0.0])
        world = R @ body_x
        np.testing.assert_allclose(world, [0.0, 1.0, 0.0], atol=1e-12)

    def test_thrust_direction_follows_tilt(self, plant):
        """
        When pitched forward (theta > 0), the thrust vector must acquire a
        forward (+x world) component while losing some vertical component.
        This verifies thrust is correctly rotated body->world.
        """
        state = np.zeros(12)
        state[7] = np.radians(20.0)  # pitch up 20 deg
        d = plant.derivatives(state, hover_omegas(plant))
        # With a forward pitch, expect a nonzero +x acceleration...
        assert d[3] > 0.1, "pitching forward should accelerate +x"
        # ...and the vertical thrust component drops, so net z-accel goes negative
        # (gravity no longer fully cancelled).
        assert d[5] < 0.0, "tilting reduces vertical thrust, should sag"

    def test_allocator_inverts_mixer(self, plant):
        """
        The control allocator must be a true inverse of the plant's force/torque
        map: ask for a known wrench, allocate motor speeds, push them back
        through the plant, and recover the same thrust and torque.
        """
        alloc = ControlAllocator(plant)
        thrust_cmd = plant.m * plant.g            # hover thrust
        tau_cmd = np.array([0.05, -0.03, 0.01])   # arbitrary small torque
        omegas = alloc.allocate(thrust_cmd, tau_cmd)
        thrust_out, tau_out = plant.motor_forces_torques(omegas)
        assert thrust_out == pytest.approx(thrust_cmd, rel=1e-6)
        np.testing.assert_allclose(tau_out, tau_cmd, atol=1e-6)


# ===========================================================================
#  LAYER 2 — OPEN-LOOP PHYSICAL BEHAVIOR (plant alone, no controller)
# ===========================================================================
class TestOpenLoopBehavior:
    """
    Drive the plant directly with hand-picked motor speeds and integrate for a
    short time. Assert the qualitative motion matches Newtonian intuition.
    These catch coupling-direction and integrator bugs.
    """

    def _rollout(self, plant, omegas, dt=0.002, n=500, state0=None):
        """Integrate the plant open-loop and return the final state + history."""
        state = np.zeros(12) if state0 is None else state0.copy()
        hist = [state.copy()]
        for _ in range(n):
            state = plant.step_rk4(state, omegas, dt)
            hist.append(state.copy())
        return state, np.array(hist)

    def test_extra_thrust_climbs(self, plant):
        """All motors above hover -> the drone must gain altitude."""
        omegas = hover_omegas(plant) * 1.05
        final, _ = self._rollout(plant, omegas)
        assert final[2] > 0.05, "above-hover thrust should climb"
        assert final[5] > 0.0, "vertical velocity should be positive"

    def test_reduced_thrust_descends(self, plant):
        """All motors below hover -> the drone must lose altitude."""
        omegas = hover_omegas(plant) * 0.95
        final, _ = self._rollout(plant, omegas)
        assert final[2] < -0.05, "below-hover thrust should descend"

    def test_differential_thrust_rolls(self, plant):
        """
        Spin the left motors (M1 front-left, M2 rear-left) faster than the right
        (M0, M3). Net positive roll torque should develop a positive roll angle
        AND a positive roll rate. This pins the roll-axis sign.
        """
        omegas = hover_omegas(plant).copy()
        omegas[1] *= 1.02  # front-left up
        omegas[2] *= 1.02  # rear-left up
        omegas[0] *= 0.98  # front-right down
        omegas[3] *= 0.98  # rear-right down
        final, hist = self._rollout(plant, omegas, n=200)
        # Body roll rate p (index 9) and roll angle phi (index 6) same sign.
        assert final[9] > 0.0, "left-heavy thrust should produce +roll rate"
        assert final[6] > 0.0, "and accumulate a +roll angle"

    def test_yaw_torque_from_spin_imbalance(self, plant):
        """
        Speed up the two CCW motors and slow the two CW motors. The net rotor
        drag imbalance must produce a yaw rate, with no roll/pitch contamination
        (since thrust stays symmetric front/back and left/right).
        """
        omegas = hover_omegas(plant).copy()
        # spin_dir = [-1, +1, -1, +1]; motors 1 and 3 are +1 (CCW).
        omegas[1] *= 1.03
        omegas[3] *= 1.03
        omegas[0] *= 0.97
        omegas[2] *= 0.97
        final, _ = self._rollout(plant, omegas, n=200)
        assert abs(final[11]) > 1e-3, "spin imbalance should yaw the airframe"
        # Roll and pitch rates should stay tiny — this is a pure-yaw input.
        assert abs(final[9]) < 1e-2, "no spurious roll from a yaw command"
        assert abs(final[10]) < 1e-2, "no spurious pitch from a yaw command"

    def test_gyroscopic_coupling_sign(self, plant):
        """
        Euler's equation has the cross term -omega x (I omega). With Ixx == Iyy
        but Izz different, a simultaneous roll+yaw rate must induce a pitch
        acceleration of a predictable sign. This tests the rotational coupling
        term specifically (a common place for sign bugs).

            q_dot = [(Izz - Ixx)/Iyy] * p * r
        """
        state = np.zeros(12)
        p, r = 2.0, 3.0
        state[9] = p   # roll rate
        state[11] = r  # yaw rate
        d = plant.derivatives(state, hover_omegas(plant))
        q_dot = d[10]
        expected = (plant.Izz - plant.Ixx) / plant.Iyy * p * r
        assert q_dot == pytest.approx(expected, rel=1e-6)

    def test_horizontal_drag_opposes_velocity(self, plant):
        """
        Give the body a horizontal velocity with motors at hover. Translational
        drag must decelerate it (acceleration opposite to velocity).
        """
        state = np.zeros(12)
        state[3] = 5.0  # moving +x at 5 m/s
        d = plant.derivatives(state, hover_omegas(plant))
        assert d[3] < 0.0, "drag should decelerate forward motion"


# ===========================================================================
#  LAYER 3 — INTEGRATOR ACCURACY & ENERGY CONSERVATION
# ===========================================================================
class TestIntegratorAndEnergy:
    """Verify the numerics, not just the equations."""

    def test_rk4_matches_solve_ivp(self, plant):
        """
        The custom RK4 step and scipy's adaptive RK45 must agree closely over a
        short horizon with identical inputs. A mismatch means the RK4 is wrong.
        """
        omegas = hover_omegas(plant) * 1.03
        state_rk4 = np.zeros(12)
        state_ivp = np.zeros(12)
        dt = 0.001
        for _ in range(200):
            state_rk4 = plant.step_rk4(state_rk4, omegas, dt)
            state_ivp = plant.step_ivp(state_ivp, omegas, dt)
        np.testing.assert_allclose(state_rk4, state_ivp, atol=1e-6)

    def test_projectile_matches_kinematics(self, plant):
        """
        With motors OFF, the center of mass is a ballistic projectile. Compare
        the integrated trajectory to the closed-form solution z = z0 + v0 t -
        0.5 g t^2 (ignoring drag is invalid here, so disable it).
        """
        p = QuadcopterPlant(drag_lin=0.0)  # remove drag for clean kinematics
        state = np.zeros(12)
        state[2] = 100.0   # start high
        state[5] = 5.0     # initial upward velocity
        dt, T = 0.001, 1.0
        n = int(T / dt)
        s = state.copy()
        for _ in range(n):
            s = p.step_rk4(s, np.zeros(4), dt)
        z_analytic = 100.0 + 5.0 * T - 0.5 * p.g * T**2
        assert s[2] == pytest.approx(z_analytic, abs=1e-3)

    def test_energy_conserved_without_drag(self, plant):
        """
        In free ballistic flight with no drag and no thrust, total mechanical
        energy (kinetic + potential) must be conserved. RK4 should hold this to
        a tight tolerance over the horizon.
        """
        p = QuadcopterPlant(drag_lin=0.0)
        state = np.zeros(12)
        state[2] = 50.0
        state[3] = 4.0    # some horizontal velocity
        state[5] = 6.0    # some vertical velocity

        def energy(s):
            ke = 0.5 * p.m * np.sum(s[3:6] ** 2)
            pe = p.m * p.g * s[2]
            return ke + pe

        e0 = energy(state)
        s = state.copy()
        for _ in range(1000):
            s = p.step_rk4(s, np.zeros(4), 0.001)
        e1 = energy(s)
        # Relative energy drift should be well under 0.1%.
        assert abs(e1 - e0) / e0 < 1e-3

    def test_linearization_matches_nonlinear(self, plant):
        """
        Near the hover trim, the linearized model A,B must predict the same
        state derivative as the full nonlinear model for a small perturbation.
        This validates the Jacobian used for LQR.
        """
        A, B, x_trim, u_trim = plant.linearize()
        # Small perturbation in state and input.
        dx = np.zeros(12)
        dx[6] = 0.01   # 0.01 rad roll
        dx[3] = 0.1    # 0.1 m/s forward
        du = np.zeros(4)
        du[0] = 2.0    # bump one motor

        nonlinear = plant.derivatives(x_trim + dx, u_trim + du)
        linear = A @ dx + B @ du   # f(trim) is ~0 at hover
        np.testing.assert_allclose(nonlinear, linear, atol=1e-3)


# ===========================================================================
#  LAYER 4 — CLOSED-LOOP SANITY (controller in the loop)
# ===========================================================================
class TestClosedLoop:
    """
    With the full controller running, assert the regulated behavior is
    physically sensible. These are the behaviors the plots should show.
    """

    def _run(self, setpoint_fn, duration=12.0, x0=None):
        plant = QuadcopterPlant()
        alloc = ControlAllocator(plant)
        ctrl = CascadedPIDController(plant, alloc)
        sim = Simulation(plant, ctrl, control_hz=200.0)
        sim.run(duration, setpoint_fn, x0=x0)
        return sim

    def test_hover_holds_position(self):
        """Commanded to hold the origin at 2 m, the drone must stay near it."""
        sim = self._run(lambda t: {'x': 0, 'y': 0, 'z': 2.0, 'yaw': 0})
        final = sim.state_hist[-1]
        assert abs(final[2] - 2.0) < 0.2, "altitude should settle near target"
        assert abs(final[0]) < 0.1 and abs(final[1]) < 0.1, "no horizontal drift"

    def test_altitude_step_settles(self):
        """Climb to 5 m and stay there — final altitude within 5% of target."""
        sim = self._run(lambda t: {'x': 0, 'y': 0, 'z': 5.0, 'yaw': 0})
        final_z = sim.state_hist[-1][2]
        assert final_z == pytest.approx(5.0, rel=0.05)

    def test_no_divergence(self):
        """
        Over the full maneuver, no state may blow up. NaNs or huge values mean
        the loop went unstable.
        """
        from QuadSim.models.quad_sim import step_maneuver
        sim = self._run(step_maneuver)
        S = sim.state_hist
        assert np.all(np.isfinite(S)), "states must remain finite"
        assert np.all(np.abs(S[:, 0:3]) < 100.0), "positions must stay bounded"
        assert np.all(np.abs(S[:, 6:9]) < np.pi), "attitudes must stay sane"

    def test_motors_within_saturation(self):
        """
        For a gentle maneuver, motor speeds must never exceed the actuator
        limit. If they do, the demo gains/commands are too aggressive.
        """
        from QuadSim.models.quad_sim import step_maneuver
        sim = self._run(step_maneuver)
        assert np.all(sim.omega_hist <= plant_max_omega()), \
            "motors should not saturate on a gentle maneuver"

    def test_x_translation_requires_pitch(self):
        """
        To move forward in +x, the drone MUST pitch (can't translate without
        tilting — that's the underactuated nature of a quadcopter). Verify a
        nonzero pitch excursion occurs during an x-step.
        """
        from QuadSim.models.quad_sim import step_maneuver
        sim = self._run(step_maneuver)
        S = sim.state_hist
        max_pitch = np.abs(S[:, 7]).max()
        assert max_pitch > np.radians(0.5), \
            "forward translation must involve pitching"

    def test_lateral_symmetry(self):
        """
        Commanding +y should mirror commanding -y: same magnitude response,
        opposite sign roll. Tests that the controller/plant aren't biased.
        """
        sim_pos = self._run(lambda t: {'x': 0, 'y': 1.0, 'z': 3.0, 'yaw': 0})
        sim_neg = self._run(lambda t: {'x': 0, 'y': -1.0, 'z': 3.0, 'yaw': 0})
        yp = sim_pos.state_hist[-1][1]
        yn = sim_neg.state_hist[-1][1]
        assert yp == pytest.approx(-yn, rel=0.05), "y response should be symmetric"


def plant_max_omega():
    """Helper for the saturation test (module-level so it's picklable)."""
    return QuadcopterPlant().max_omega


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
