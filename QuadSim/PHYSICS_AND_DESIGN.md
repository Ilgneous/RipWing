# Quadcopter Simulator — Physics & Design Notes

This document explains the physics behind `quad_sim.py` and `diagnostics.py`, why
each modeling choice was made, and how the code maps onto the standard
literature. It is written to be read *alongside* the code: section headings
reference specific classes and methods.

The goal is twofold: (1) so you can defend every line of the model against a
textbook, and (2) so you know which simplifications are deliberate and where the
"real" physics would go if you needed more fidelity later.

---

## 0. How to use this document (suggested reading order)

The three references you found sit at very different levels. Read them in this
order, mapping each to the code as you go:

1. **GMU lecture slides** (`QuadcopterDynamics.pdf`) — start here. Lightest
   math, builds physical intuition for frames, thrust, and torque. Read before
   touching the equations.
2. **Gibiansky blog** ("Quadcopter Dynamics") — the workhorse. This is the
   derivation `quad_sim.py` most closely follows: thrust ∝ ω², drag torque ∝ ω²,
   the inertial-frame translational equation, and the body-frame Euler rotational
   equation. Read this with `QuadcopterPlant.derivatives()` open side by side.
3. **MIT VNAV Lecture 6** (Carlone) — the rigorous version. Formalizes the
   Newton-Euler equation as a single matrix expression, proves the
   rotation-matrix derivative Ṙ = R[ω]×, and introduces **differential
   flatness** — the property that makes trajectory generation tractable. Read
   this once you're comfortable with #2 and want the "why is this the right
   structure" answer.

A concrete study path tied to the code is in the last section (§9).

Additional references worth having (full citations in §10):

- **Mellinger & Kumar (2011)**, "Minimum snap trajectory generation and control
  for quadrotors" — the foundational paper for the *cascaded* control structure
  you're using and for flatness-based trajectory design. This is the paper the
  MIT notes cite for differential flatness.
- **Beard & McLain**, *Small Unmanned Aircraft* (textbook) — the most complete
  single reference for the full sensing→estimation→control→guidance stack, which
  is your eventual firmware pipeline.

---

## 1. Coordinate frames and the state vector

### What the code does
`quad_sim.py` uses a 12-element state vector (see module docstring):

```
[ x  y  z | u  v  w | φ  θ  ψ | p  q  r ]
  position   velocity   Euler      body rates
  (world)    (world)    (ZYX)      (body)
```

- World frame: x forward, y left, z **up**; gravity acts in −z.
- Body frame: FLU (x forward, y left, z up); thrust along +body-z.
- Euler angles use the **ZYX** (yaw-pitch-roll) intrinsic convention.

### Why
This is the standard rigid-body state for an underactuated flyer: **6 DoF**
(3 translational + 3 rotational) but only **4 inputs** (rotor speeds). That
4-vs-6 gap is the entire reason control is interesting — you cannot move
sideways without first tilting, so translation and rotation are *coupled*. The
Gibiansky blog and MIT notes both open with this point; it is the defining
feature of the vehicle.

**A convention difference worth knowing:** the Gibiansky blog uses a ZYZ Euler
sequence and the MIT notes work directly with the rotation matrix R ∈ SO(3)
(no Euler angles at all). This code uses ZYX because it is the most common
aerospace convention and keeps roll/pitch/yaw human-readable on the plots. The
*physics* is identical regardless of parameterization; only the entries of the
rotation matrix and the body-rate↔Euler-rate map change.

### Where it lives
- `QuadcopterPlant.rotation_matrix()` — body→world rotation R for ZYX.
- `QuadcopterPlant.euler_rate_matrix()` — maps body rates (p,q,r) to Euler
  rates (φ̇,θ̇,ψ̇). This is the W matrix; see §4.

---

## 2. Thrust and torque from the rotors

### What the code does
In `QuadcopterPlant.motor_forces_torques()`:

```
f_i   = k_f · ω_i²      (per-motor thrust, along +body z)
τ_i   = k_m · ω_i²      (per-motor reaction/drag torque about body z)
```

Total thrust is Σf_i. Roll/pitch torque comes from differential thrust across
the arms (τ = Σ rᵢ × Fᵢ), and yaw torque is the net rotor-drag imbalance
(Σ spin_dirᵢ · k_m ω_i²).

### Why thrust ∝ ω²
This is **momentum theory** (actuator-disk model). A rotor accelerates a column
of air downward; thrust equals the rate of momentum imparted to that air. Working
through hover induced-velocity (vₕ = √(T/2ρA)) and the motor power relation, the
clean result is that thrust scales with the *square* of angular velocity, T = kω².
The Gibiansky blog derives this from the brushless-motor voltage/torque equations
and conservation of energy; the MIT notes state it directly as
`Tᵢ = c_f · ωᵢ|ωᵢ|·e₃`. Both land on the same ω² law.

The drag (reaction) torque has the *same* ω² form for the same aerodynamic
reason — it is the torque the airframe feels as a reaction to spinning the
prop against air resistance. That is why `k_m` is ~50× smaller than `k_f`
here: yaw authority is much weaker than thrust, which the test suite checks
explicitly (`test_linearization_matches_nonlinear` shows the yaw column of B is
~16× weaker than roll/pitch).

### Why alternating spin directions
`spin_dir = [-1, +1, -1, +1]`. Diagonal motors spin the same way; adjacent
motors spin opposite ways. In hover all four reaction torques cancel, so the
airframe doesn't spin up. To *command* yaw you deliberately unbalance the
CW vs CCW pairs — speeding up the two CCW props and slowing the two CW props
yaws the body without changing net thrust or roll/pitch. This is exactly what
`diagnostics.py` scenario 3 ("pure yaw") demonstrates, and what
`test_yaw_torque_from_spin_imbalance` verifies produces yaw with no roll/pitch
contamination.

### The "X" frame mixing
Because the motors sit at the corners of a square (an "X" relative to the
forward axis), each motor contributes to roll, pitch, AND yaw simultaneously.
The geometry is encoded in `motor_pos` and turned into a 4×4 mixing matrix in
`ControlAllocator`. The columns map [ω₀²…ω₃²] → [thrust, τx, τy, τz]:

```
thrust =  k_f (ω₀² + ω₁² + ω₂² + ω₃²)
τx (roll)  =  k_f · y_i · ωᵢ²   summed   (left/right thrust difference)
τy (pitch) = −k_f · x_i · ωᵢ²   summed   (front/back thrust difference)
τz (yaw)   =  k_m · spin_dirᵢ · ωᵢ²  summed
```

This matches the Gibiansky "+"-frame relations (τφ, τθ, τψ) rotated 45° into the
X configuration. The blog uses a + frame where each torque axis is driven by
just two motors; the X frame spreads it across all four, which is why real
quadcopters fly X (more authority per axis, camera doesn't see the props).

---

## 3. Translational dynamics (Newton)

### What the code does
In `derivatives()`:

```
m · a_world = R · [0,0,T] + [0,0,−mg] + (−drag_lin · v)
```

Thrust is generated along +body-z, rotated into the world frame by R, then
gravity and a linear drag term are added.

### Why this form
This is **Newton's second law in the inertial frame** — the natural frame for
translation because gravity and position are defined there. It matches Gibiansky's
`m·ẍ = [0,0,−mg] + R·T_B + F_D` exactly, and the MIT notes' `m·v̇ = −mge₃ +
R·f_thrust` (they fold drag into a later lab).

The key physical insight is the `R · [0,0,T]` term: **the body can only push
along its own z-axis.** To accelerate in +x world, the only option is to pitch
the body so that the thrust vector tilts forward. That is the underactuation from
§1 made concrete, and it is why `test_x_translation_requires_pitch` asserts that
any forward motion *must* be accompanied by a pitch excursion — if it weren't,
the physics would be wrong.

### The drag simplification
`drag = −drag_lin · v` is a deliberately simple linear model. Real aerodynamic
drag is quadratic in velocity and direction-dependent, but at the speeds this sim
targets (<10 m/s) the linear approximation is standard and matches Gibiansky's
"highly simplified view of fluid friction." The MIT notes note that at <10 m/s the
neglected effects (blade flapping, hub forces, ground effect) are "more than one
order of magnitude smaller" than thrust and rotor drag — which is the
justification for leaving them out. If you later need high-speed fidelity, this
is the first term to upgrade.

---

## 4. Rotational dynamics (Euler) and the body↔Euler-rate map

### What the code does
Two separate pieces:

**Euler's rigid-body equation** (in `derivatives()`):
```
ω̇ = I⁻¹ · ( τ − ω × (I·ω) )
```

**Body-rate to Euler-rate map** (`euler_rate_matrix()`, applied in `derivatives()`):
```
[φ̇, θ̇, ψ̇] = W(φ,θ) · [p, q, r]
```

### Why rotational dynamics live in the body frame
The inertia tensor I is constant *only* in the body frame (it rotates with the
vehicle). If you wrote Euler's equation in the world frame, I would be
time-varying and the equation would be a mess. So rotational dynamics are
expressed in the body frame, which is why the state stores body rates (p,q,r)
rather than Euler-angle rates.

### The gyroscopic coupling term −ω × (I·ω)
This cross-product term is the part people get wrong. It is the **gyroscopic /
Coriolis coupling**: when the body is already rotating about two axes, conservation
of angular momentum induces an acceleration about the third. For a diagonal
inertia it expands to terms like `q̇ = [(Izz−Ixx)/Iyy]·p·r`. This is a classic
sign-bug location, which is why `test_gyroscopic_coupling_sign` checks that exact
closed form against the code. Both Gibiansky and MIT write Euler's equation in the
identical `I⁻¹(τ − ω×Iω)` form.

### Why a separate W matrix for the angles
Body rates (p,q,r) are *not* the same as Euler-angle rates (φ̇,θ̇,ψ̇). The MIT
notes are emphatic on this: "note that the angular velocity vector ω ≠ θ̇." The
gyro measures body rates; the plots want human-readable Euler angles; the W matrix
(`euler_rate_matrix`) converts between them. It becomes singular at θ = ±90°
("gimbal lock"), which is the fundamental limitation of any Euler-angle
parameterization and the reason serious flight stacks (and the MIT notes) prefer
to integrate the rotation matrix R or a quaternion directly. For this sim's
gentle maneuvers, Euler angles are fine and far more readable.

> **Upgrade path:** if you ever fly aggressive maneuvers (near-vertical pitch),
> switch the attitude state from Euler angles to a quaternion or rotation matrix
> and integrate `Ṙ = R[ω]×` (proved in MIT Lecture 6 §6.1.1). The translational
> and torque code stays identical; only the attitude kinematics change.

---

## 5. Integration: why RK4 (and why a solve_ivp cross-check)

### What the code does
`step_rk4()` advances the state with fixed-step classical RK4, holding motor
speeds constant across the step (zero-order hold). `step_ivp()` does the same with
scipy's adaptive RK45 as a cross-check.

### Why
The Gibiansky blog uses plain forward Euler (`x += dt·ẋ`). That works for a demo
but accumulates error quickly. RK4 is the standard upgrade: it samples the
derivative four times per step and is 4th-order accurate, so for the same step
size it is dramatically more accurate, and for the same accuracy it permits much
larger steps. The zero-order hold on motor speeds is not a hack — it mirrors
reality, where a discrete controller sets motor commands once per control tick and
they're held until the next.

The two integrators exist so they can be cross-checked against each other
(`test_rk4_matches_solve_ivp`): if the hand-written RK4 had a bug, it would
disagree with scipy's well-tested adaptive solver. Energy conservation in
drag-free ballistic flight (`test_energy_conserved_without_drag`) is the other
numerical check — a leaky integrator would bleed or inject energy.

---

## 6. Control architecture: cascaded PID

### What the code does
`CascadedPIDController` has two nested loops:

- **Outer (position) loop:** position error → desired world acceleration (PID
  per axis) → desired roll/pitch angles + total thrust.
- **Inner (attitude/rate) loop:** attitude error → desired body rates (P) →
  body torques (PID).

The desired (thrust, τ) wrench then goes through `ControlAllocator` to get the
four motor speeds.

### Why cascaded (nested) rather than one flat controller
This is the single most important design choice, and it follows directly from the
underactuation. You cannot directly command position — you can only command motor
speeds, which produce torques, which change attitude, which *finally* redirects
thrust to move position. So the controller is structured as a chain that mirrors
the physics:

```
position → attitude → body rate → torque → motor speed
 (slow)                                      (fast)
```

The inner loops run conceptually faster and stabilize attitude; the outer loop
treats the (fast, well-stabilized) attitude as something it can command. This
**time-scale separation** is why cascaded control works and is the structure used
in essentially every real flight controller (PX4, Betaflight, Ardupilot) and in
the Mellinger-Kumar paper. The Gibiansky blog only closes the *attitude* loop
(it explicitly says position can't be controlled with a gyro alone); this sim adds
the outer position loop on top, which requires the position/velocity feedback the
sim provides perfectly.

### Why thrust is divided by cos(φ)cos(θ)
In `compute_commands`, the thrust is `m(g + a_z) / (cosφ·cosθ)`. When the drone
tilts, the vertical component of thrust drops, so altitude would sag. Dividing by
cosφ·cosθ projects the needed thrust back up so the *vertical* component stays
correct. This exact compensation is in the Gibiansky blog (T = mg/(cosθcosφ)) and
is what keeps altitude flat during the X-translation in the demo.

### Why the integral anti-windup
The `PID` class clamps its integral term. If the drone is far from target, the
integral can accumulate a huge correction that overshoots badly once the error
finally closes — "integral windup." The Gibiansky blog devotes a section to this
and disables the integral until near steady-state; this code instead clamps the
integral magnitude (`integ_limit`), a simpler and common alternative. The smoothed
setpoints in `step_maneuver` (first-order ramp instead of a hard step) serve the
same goal from the other side: don't ask the loop to track a discontinuity.

### Why separate the allocator
`ControlAllocator` is factored out specifically so the *same* allocation step
serves both the PID controller and a future LQR controller. The controller's job
ends at "here is the wrench I want (thrust + 3 torques)"; turning that wrench into
four motor speeds is pure geometry that both controllers share. This is the
`compute_commands(state, setpoint, dt) → omegas` interface boundary that makes the
PID→LQR swap a drop-in.

---

## 7. Linearization and the LQR path

### What the code does
`QuadcopterPlant.linearize()` numerically computes the Jacobians A = ∂f/∂x and
B = ∂f/∂u about a trim point (default: hover) via central finite differences,
returning continuous-time state-space matrices.

### Why
PID gains are tuned by hand (as the Gibiansky blog admits — "tuned by hand and
intuition"). LQR instead computes an *optimal* gain from the linearized dynamics
and a cost function, but it needs the A/B matrices. Linearizing about hover is
valid because hover is the operating point the vehicle spends most of its time
near, and small-signal behavior around it is well-approximated as linear.

Central differences are used rather than hand-derived Jacobians because they're
less error-prone and automatically correct for the trivially-linear couplings
(position fed by velocity comes out as an exact identity block). The result is
validated three ways: structural checks (identity blocks, ±g tilt coupling),
controllability (rank 12), and that the linear prediction matches the nonlinear
model for small perturbations (`test_linearization_matches_nonlinear`).

> **Note on discrete vs continuous:** `linearize()` returns *continuous-time*
> A/B. Your embedded controller runs at a fixed rate, so for a discrete LQR you'd
> first discretize (e.g. `scipy.signal.cont2discrete`) at the control frequency.
> This matters when you port to firmware.

---

## 8. What the diagnostics verify, and why those scenarios

`diagnostics.py` runs five scenarios chosen so each has a *known* expected plot
shape — the point is to verify the plots are realistic, not just that numbers
come out:

| Scenario | Physical principle | Expected signature |
|---|---|---|
| Free fall (motors off) | Newton: a = −g | Parabolic Z, flat angles, zero RPM |
| Pure climb | Symmetric thrust > weight | Vertical line, flat angles, 4 equal RPMs |
| Pure yaw | Spin-drag imbalance | Yaw ramps, roll/pitch ≈ 0, RPM splits 2+2 |
| Square path | Cascaded control + coupling | Box XY path, alternating roll/pitch |
| Disturbance recovery | Closed-loop stability | 25° kick decays to 0, position recovers |

The first three are **open-loop** (plant only, no controller) precisely so they
isolate the *physics* from the controller — if free fall isn't a clean parabola,
the bug is in the dynamics or integrator, not the gains. The last two are
**closed-loop** and confirm the controller produces physically sensible regulated
behavior. This mirrors the layered structure of `test_physics.py` (analytical
invariants → open-loop behavior → integrator → closed-loop), which is the same
validate-from-the-bottom-up philosophy: prove the conservation laws and known
analytical facts first, because those catch the sign/unit/frame bugs that plots
can hide.

---

## 9. A concrete study path tied to the code

If you want to *learn* the material (not just use the sim), this order maps each
concept to the code that implements it:

1. **Frames & state** (GMU slides; MIT §6.1 intro). Read the `quad_sim.py`
   module docstring and `rotation_matrix()`. Convince yourself why position is in
   the world frame but angular rates are in the body frame.
2. **Thrust & torque** (Gibiansky "Quadcopter Dynamics" through the torque
   section). Read `motor_forces_torques()` and `ControlAllocator`. Derive the
   mixing matrix yourself and check it against `M` in the code.
3. **Translational EOM** (Gibiansky "Equations of Motion"; MIT eq. 6.7–6.10).
   Read the first half of `derivatives()`. Run diagnostics scenarios 1–2.
4. **Rotational EOM** (same sources; focus on Euler's equation). Read the second
   half of `derivatives()` and `euler_rate_matrix()`. Run `test_gyroscopic_
   coupling_sign` and scenario 3.
5. **Integration** (any numerical-methods reference on RK4). Read `step_rk4()`.
   Run `test_rk4_matches_solve_ivp` and `test_energy_conserved_without_drag`.
6. **Control** (Gibiansky PD/PID sections; then Mellinger-Kumar for the cascaded
   structure). Read `CascadedPIDController`. Run scenarios 4–5.
7. **Linearization & LQR** (any linear-systems text; MIT §6.2 for flatness as the
   "why trajectory generation works" capstone). Read `linearize()` and
   `demo_lqr_design()`.

---

## 10. References

**The three you found (in recommended reading order):**

- GMU SYST 460 lecture slides, *Quadcopter Dynamics*.
  https://catsr.vse.gmu.edu/SYST460/QuadcopterDynamics.pdf
- A. Gibiansky, *Quadcopter Dynamics, Simulation, and Control* (blog).
  https://andrew.gibiansky.com/blog/physics/quadcopter-dynamics/
  — The derivation this code follows most closely.
- L. Carlone, *MIT 16.485 VNAV Lecture 6: Quadrotor Dynamics*.
  https://vnav.mit.edu/material/06-Control1-notes.pdf
  — Rigorous Newton-Euler, Ṙ = R[ω]× proof, and differential flatness.

**Foundational papers / textbooks worth adding:**

- D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control for
  quadrotors," *IEEE ICRA*, 2011, pp. 2520–2525. The standard reference for the
  cascaded control structure and flatness-based trajectory generation; cited by
  the MIT notes.
- R. W. Beard and T. W. McLain, *Small Unmanned Aircraft: Theory and Practice*,
  Princeton University Press, 2012. The most complete single textbook for the
  full dynamics → estimation → control → guidance stack (i.e. your eventual
  firmware pipeline).
- For the rigid-body rotational dynamics specifically, any classical mechanics
  text covering Euler's equations (e.g. Goldstein, *Classical Mechanics*) is the
  ground truth behind the `I⁻¹(τ − ω×Iω)` term.

> A note on source quality: prefer primary sources (the Mellinger-Kumar paper,
> the MIT notes, a real mechanics textbook) over aggregated blog posts when you
> need to be *certain* about a sign or convention. Blogs are excellent for
> intuition and a first pass — the Gibiansky post is genuinely good — but they
> occasionally simplify in ways that bite you when porting to hardware. When the
> code and a source disagree, re-derive from the Newton-Euler equation in the
> MIT notes; that is the most authoritative of the set.
