# 10-DOF Quadcopter Model — Physics, Verification & PID Tuning

Companion to `PHYSICS_AND_DESIGN.md` (which covers the 6-DoF base model).
This document covers what was *added* in `quad_sim_10dof.py`, how each addition
was verified (`test_physics_10dof.py`), what the extra fidelity actually buys
(`compare_models.py` → `model_comparison.png`), and how the model is used to
tune the PID loops (`pid_tuning.py` → `tuned_gains.json`,
`tuning_comparison.png`).

---

## 1. What "10-DOF" means here

The rigid body still has 6 degrees of freedom. The additional 4 DOF are the
**rotor speeds**, promoted from algebraic inputs to *states* with first-order
dynamics:

```
state (16) = [ x y z | u v w | φ θ ψ | p q r | ω₀ ω₁ ω₂ ω₃ ]
                          12 body states            4 rotor states
```

In the 6-DoF model, a commanded motor speed takes effect instantly. In
reality the ESC + motor + prop is a lag: the prop has inertia, and the ESC
takes finite time to change its speed. This is modeled as

```
ω̇ᵢ = (ω_cmd,ᵢ − ωᵢ) / τ_m          τ_m = 50 ms
```

Everything downstream (thrust, torque, flapping) now uses the *actual* rotor
speed, not the commanded one. This single change is what puts a hard ceiling
on achievable control bandwidth — see §5, panel C.

## 2. Blade flapping — the physics

A hovering rotor in still air produces thrust along its shaft. A rotor
**translating** through the air does not: the advancing blade (moving into the
relative wind) sees higher airspeed and makes more lift; the retreating blade
sees less. The blade "flaps" up on the advancing side, and the **rotor tip-path
plane tilts back** relative to the shaft, away from the direction of motion.
Two consequences for the airframe:

1. **A horizontal force opposing the velocity** (the thrust vector tilts back):
   `F_flap = −k_flap · T · v_xy(body)`. It scales with thrust because it *is*
   the tilted thrust vector, and linearly with speed for small advance ratios.
2. **A pitching/rolling moment away from the direction of motion**, because
   that force acts at the rotor plane, a height `h` above the center of mass:
   `τ_flap = h · (ẑ × F_flap)`.

This is modeled **quasi-statically**: flapping dynamics settle within about
one rotor revolution (~4 ms at 260 rev/s), which is far faster than the body
dynamics (~100 ms) — a textbook timescale separation. The literature does the
same for control-oriented models: Mahony, Kumar & Corke (2012) give exactly
this quasi-static formulation; Hoffmann et al. (2007) measured the effect on
the Stanford STARMAC testbed and showed it dominates attitude behavior at
speed; Pounds et al. (2010) used the h-offset moment deliberately for passive
stability. Full flapping dynamics (Leishman) are only needed for rotor-blade
structural analysis, not vehicle control.

Two smaller rotor effects are included with the same quasi-static logic:

- **Rotor gyroscopics**: the spinning rotors carry net angular momentum
  `H_z = J_r Σ sᵢωᵢ`; body rotation produces `τ = −ω × [0,0,H_z]`.
- **Yaw reaction from rotor acceleration**: `τ_z += −J_r Σ sᵢ·ω̇ᵢ` — spinning
  a prop up torques the airframe the other way. With motor lag this becomes a
  *feedthrough* from commands to yaw, which matters (see §6, the yaw story).
- **Rotor rate damping**: `τ −= k_damp·[p,q,0]`, the aerodynamic damping a
  rotor disk presents to body rotation.

Parameters (`tau_motor=0.05 s`, `k_flap=0.008 rad/(m/s)`, `h=0.05 m`,
`J_r=3·10⁻⁵ kg·m²`) are **representative values from the literature ranges,
not identified from your airframe**. Before trusting tuned gains on hardware:
identify `τ_m` from a bench step response of one motor+prop (log ESC telemetry
or a tachometer), and `k_flap` from the deceleration of a coast-down glide.

## 2b. Ground effect (in-ground-effect, IGE)

### The physics
Close to the ground the rotor downwash cannot fully develop: the wake is
turned outward by the surface, which reduces the induced velocity through the
disk and therefore reduces induced drag. At a fixed rotor speed the rotor
produces **more thrust** near the ground than in free air. The effect is
governed by rotor height normalized by rotor radius (z/R) and is essentially
gone by z/R ≈ 2–3.

### Why the exponential model, not Cheeseman-Bennett
The classical relation every helicopter text gives is

```
K_G = T_IGE / T_OGE = 1 / (1 - (R / 4z)^2)          (Cheeseman & Bennett 1955)
```

It has a **singularity at z/R = 0.25** and predicts infinite thrust at the
ground. That is tolerable for a full-size helicopter, whose fuselage hangs
below the rotor so the disk is never within a quarter radius of the ground.
It is *not* tolerable for a low-profile quadcopter, which spends the final
moments of every landing exactly in that regime — precisely the regime you
care about. The literature is explicit that Cheeseman-Bennett does not
transfer directly to multirotors.

This model therefore uses the exponential form (He & Leang):

```
K_G = 1 + C_a * exp(-C_b * z / R)
```

which predicts a **finite** maximum surplus at the ground, `K_G → 1 + C_a`.
Defaults are `C_a = 0.30` (30% surplus at touchdown) and `C_b = 2.2`, giving
+3% at one rotor radius and < 1% beyond two — consistent with the reported
range for small two-blade props. Cheeseman-Bennett remains selectable via
`ground_effect_model='cheeseman'` for comparison, clamped off its singularity.

### Why it is applied PER ROTOR
Ground effect is computed for each rotor at its own height, not as one
body-level thrust scale factor. When the vehicle banks, the low rotors sit
deeper in ground effect and gain more thrust than the high ones, and that
differential thrust is a **moment**. Modeling it as a single scalar would
capture the "floats on landing" symptom but miss the attitude coupling
entirely — including the fact that near the ground the asymmetry acts as a
strong restoring (self-leveling) moment.

### Model interface
```
rotor_heights(position_z, body_to_world)  -> (4,) hub heights [m]
ground_effect_ratio(rotor_height)         -> (4,) K_G per rotor
ground_effect_model : 'exponential' (default) | 'cheeseman' | 'none'
landing_gear_height : shifts z=0 to mean "skids down", not "disk on the deck"
```

`Simulation10DOF(..., enforce_ground=True)` adds an inelastic floor. This is
not decoration: without a contact constraint a commanded landing flies
straight through z = 0 into negative altitude and every touchdown metric is
meaningless.

### What it does (ground_effect_study.py → ground_effect_study.png)

| Panel | Measurement | Result |
|---|---|---|
| A | K_G vs z/R | 1.30 at the ground, 1.033 at one rotor radius, finite everywhere (Cheeseman-Bennett gives 2.04 even clamped) |
| B | 0.15 m/s precision landing | touchdown delayed **1.50 s**; impact speed cut from 0.159 to 0.034 m/s (−79%) |
| C | Roll accel vs bank angle near ground | −325 °/s² at 10° bank at 2 cm altitude: a strong restoring moment that vanishes by 0.5 m |
| D | Hover trim vs altitude | 4.9% less throttle needed at the ground (14,852 vs 15,620 rpm) |

Panel B is the answer to the original question: on a *gentle* descent the
vehicle floats and refuses to settle. An aggressive descent partly hides the
effect because momentum carries the vehicle through, which is itself worth
knowing — it means a naive "just descend faster" fix trades touchdown
precision for impact energy.

### Consequences for the flight controller
- **Altitude hold needs integral action or feed-forward near the ground.** A
  proportional-only height loop sees a persistent thrust surplus it never
  commanded and settles high.
- **Tuning is unaffected at altitude.** The tuning scenarios run at 2 m, where
  K_G = 1.000 to floating-point precision, so the tuned gains from §6 remain
  valid. Ground effect is a *landing-phase* disturbance, not a gain-schedule
  driver for the attitude loop.
- **The self-leveling moment is real but is not a stability guarantee.** It
  helps at touchdown, but it also means attitude response near the ground
  differs from free air — if you gain-schedule anything by altitude, this is
  the term that justifies it.
- **Firmware path:** the cleanest treatment is a feed-forward thrust
  correction using a height estimate (rangefinder/lidar), i.e. divide
  commanded thrust by the predicted K_G. The literature's stronger option is a
  nonlinear disturbance observer, which recovers most of the benefit without
  needing an accurate K_G model.

## 3. Where it lives in the code

| Physics | Code |
|---|---|
| Motor lag | `QuadcopterPlant10DOF.derivatives`: rotor block `(cmd−ω)/τ` |
| Flapping force | `−k_flap·T·[v_bx, v_by, 0]` added to body forces |
| Flapping moment | `h_flap · (ẑ × F_flap)` added to torques |
| Rotor gyroscopics | `−ω × [0,0,H_z]`, `H_z = J_r Σ sᵢωᵢ` |
| Yaw reaction | `−J_r Σ sᵢω̇ᵢ` on τ_z |
| Rate damping | `−k_damp·[p,q,0]` |
| Ground effect (per rotor) | `rotor_heights()`, `ground_effect_ratio()`, applied to `per_rotor_thrust` in `derivatives` |
| Ground contact | `Simulation10DOF(enforce_ground=True)` |
| 16-state trim/Jacobians | `hover_state()`, `linearize()` override (trims at 5 m, out of ground effect) |

The class inherits `QuadcopterPlant`, so the 6-DoF thrust/torque/Newton-Euler
core is shared, not duplicated.

## 4. Verification (test_physics_10dof.py — 27 tests)

Same bottom-up philosophy as the 6-DoF suite: analytical facts first.

- **Exact reduction**: with all new coefficients zeroed, the 16-state
  derivatives match the 6-DoF model to 1e-9. The upgrade cannot have broken
  the base physics.
- **Motor lag closed form**: 63.2% of a step at t=τ, 95% at 3τ.
- **Flapping closed forms**: force = −k_flap·T·v/m exactly, linear in speed,
  zero for vertical motion; moment tips the body *away* from the direction of
  motion (moving +x ⇒ q̇<0, moving +y ⇒ ṗ>0 — sign conventions verified
  against the geometry, not assumed).
- **Rate damping & gyroscopics**: checked against hand-derived closed forms
  (e.g. ṗ = −q·H_z/Ixx).
- **Linearization structure**: B-matrix body rows ≈ 0 (commands act through
  the rotor states now), rotor rows = I/τ, and the yaw feedthrough term
  −J_r·sᵢ/(τ·Izz) present with the right sign.
- **Controllability**: the naive Kalman rank test *fails numerically* (the
  16-state system spans ~16 orders of magnitude); the PBH test passes at
  every eigenvalue. Kept both in the test as a documented lesson.
- **Hover is open-loop unstable** (eigenvalues +0.59 ± 1.34j): the flapping
  moment creates the classic unstable oscillatory hover mode every real
  rotorcraft has. Zero `k_flap` and the mode vanishes. The 6-DoF model
  *cannot represent this* — closed-loop control just looks easier there.

Ground effect adds nine more tests: exact reduction when disabled, vanishing
at altitude, monotonic and finite K_G, extra thrust near the ground matching
`(K_G - 1)·g` analytically, the bank-angle restoring moment, rotor height
bookkeeping under attitude, Cheeseman-Bennett availability, the floor
constraint, and the end-to-end touchdown delay.

One consequence worth noting: adding ground effect **invalidated four existing
tests** that trimmed hover at z = 0. That was correct behavior, not a
regression — hover trim at zero altitude genuinely is no longer an equilibrium
once IGE exists. The fix was to move all free-air reference states (and the
default `linearize()` trim) to 5 m, which is the physically honest reading of
"hover trim".

Both suites: **51/51 passing** (24 legacy 6-DoF + 27 ten-DOF).

## 5. What the fidelity buys (compare_models.py → model_comparison.png)

| Panel | Experiment | 6-DoF | 10-DOF | Meaning |
|---|---|---|---|---|
| A | Velocity decay, level hold, v₀=4 m/s | no decay (>4 s) | half-life 2.33 s | Flapping is the dominant horizontal damping; the 6-DoF drone coasts forever |
| B | Roll step, commanded vs actual rotor speed | identical | 1659 rpm peak gap | Motor lag is a real actuator, not a wire |
| C | Rate-loop gain sweep ×1…×48 | ripple flat at 0.08°/s | 5.36°/s limit-cycle "buzz" at ×48 | The motor pole caps usable gain; 6-DoF rewards infinite gain |
| D | Steady 1.5 m/s cruise | 0.76° pitch | 1.61° pitch | ~2× tilt needed to overcome flapping drag — trim attitude predictions differ materially |

Panel C is the punchline for tuning: **gains tuned on the 6-DoF model are
untrustworthy precisely in the direction tuning pushes them** (higher), because
the 6-DoF plant hides the actuator pole that turns high gain into oscillation.

## 6. PID tuning on the 10-DOF plant (pid_tuning.py)

Staged Nelder-Mead in log-gain space: Stage 1 tunes the inner loop
(`k_att, kp/ki/kd_rate`) on a 15° **roll doublet**; Stage 2 tunes the outer
loop (`kp/kd_xy, kp/kd_z`) on a 2 m position step with the inner loop frozen.

Getting a *trustworthy* harness required four fixes, each a transferable
lesson:

1. **Sensor noise is mandatory.** The first noise-free run returned
   `kd_z ≈ 400` and rate kp 260× default — the optimizer exploited perfect
   measurements, where derivative gain is free damping. Realistic noise
   (0.02 m pos, 0.05 m/s vel, 0.4° att, 1°/s gyro) plus a motor-thrash
   penalty makes derivative gain cost what it costs on hardware.
2. **A constant torque bias (0.02 N·m, shifted-battery style) gives the
   integral term a job** — otherwise ki is unidentifiable.
3. **Yaw demands must be capped inside the physical yaw envelope.** Yaw
   authority is only ±0.26 N·m over the *entire* motor range (k_m ≪ k_f).
   The original `out_limit=1.0` let the controller demand 4× the physics;
   the allocator "solved" it by slamming the CW pair to zero and the CCW
   pair to max, destroying roll/pitch/thrust — a positive-feedback yaw
   ratchet that noise reliably triggered (39% of commands clipped at zero).
   Real FCs prioritize yaw last for exactly this reason. Fix:
   `pid_r.out_limit = 0.5 × (2·k_m·ω_max²)`.
4. **D-terms need a low-pass** (added to the `PID` class, default-off,
   30 Hz here): numerically differentiating a noisy error at 200 Hz
   amplifies noise ~280×. Every real flight controller filters D.

Also: the attitude scenario is a **doublet**, not an indefinite hold — a held
tilt means indefinite lateral acceleration, and by ~2.5 s the vehicle slides
fast enough that flapping moments dominate and the test stops measuring the
attitude loop. Doublets are the standard flight-test input for this reason.

A residual behavior worth knowing: during aggressive transients, yaw wanders
tens of degrees. That is real 10-DOF physics — the yaw-reaction feedthrough
(−J_r Σ sᵢω̇ᵢ) converts noisy motor commands into yaw torque noise comparable
to the capped yaw authority ("yaw washout" on small quads). A small yaw
penalty in the cost keeps the optimizer from exploiting it.

### Results

| | default | tuned |
|---|---|---|
| kp_rate | 0.040 | 0.567 |
| ki_rate | 0.002 | ~0 (2e-5) |
| kd_rate | 0.0008 | 0.00015 |
| k_att | 8.0 | 5.59 |
| kp_xy / kd_xy | 0.45 / 0.85 | 0.55 / 0.95 |
| kp_z / kd_z | 2.0 / 2.6 | 3.58 / 2.85 |
| **doublet overshoot** | **58%** | **1.1%** |
| **rate ripple** | **20.5°/s** | **1.1°/s** |
| rise time | 0.28 s | 0.33 s |

The optimizer's choices are physically interpretable: with high rate
stiffness, the torque bias is rejected through a 0.36° attitude offset, so
`ki_rate → 0`; with noise priced in, `kd_rate` *shrinks* rather than exploding.
Cross-seed validation (5 unseen noise realizations): ITAE improves 7–8×
consistently (16.6–18.7 → 2.2–2.4 deg·s) — the gains are not overfit to one
noise draw.

## 7. Transfer to firmware

- The tuned gains assume a 200 Hz control loop and *these* plant parameters.
  Re-identify `τ_m` and `k_flap` for your airframe first (§2), re-run
  `pid_tuning.py`, and expect the ratios (not the absolute numbers) to carry.
- For the LQR path: `linearize()` returns continuous A(16×16)/B(16×4);
  discretize at your control rate with `scipy.signal.cont2discrete` before
  computing the discrete LQR gain.
- The D-term low-pass belongs in your firmware PID too (fixed-point friendly:
  one multiply-accumulate per axis).
- The yaw-authority cap translates directly: clamp yaw torque demands before
  allocation, or implement prioritized desaturation (thrust > roll/pitch > yaw).

## 8. References

- G. M. Hoffmann, H. Huang, S. L. Waslander, C. J. Tomlin, "Quadrotor
  Helicopter Flight Dynamics and Control: Theory and Experiment," *AIAA GNC*,
  2007. — Measured flapping effects on STARMAC; the empirical basis for §2.
- R. Mahony, V. Kumar, P. Corke, "Multirotor Aerial Vehicles: Modeling,
  Estimation, and Control of Quadrotor," *IEEE Robotics & Automation
  Magazine*, 19(3), 2012. — The quasi-static flapping model used here; the
  best single tutorial reference for this whole document.
- P. Pounds, R. Mahony, P. Corke, "Modelling and control of a large quadrotor
  robot," *Control Engineering Practice*, 18(7), 2010. — Rotor height offset
  and passive stability.
- J. G. Leishman, *Principles of Helicopter Aerodynamics*, Cambridge
  University Press. — Full flapping dynamics, if you ever need beyond
  quasi-static.
- I. C. Cheeseman and W. E. Bennett, "The Effect of the Ground on a Helicopter
  Rotor in Forward Flight," ARC R&M 3021, 1955. — The classical IGE relation;
  included here for comparison and to document why it is *not* the default.
- X. He and K. K. Leang, "Quasi-Steady In-Ground-Effect Model for Single and
  Multirotor Aerial Vehicles," *AIAA Journal*, 58(12), 2020. — The exponential
  IGE model used here; predicts finite thrust at the ground and covers
  multirotor configurations.
- X. He, ... K. K. Leang, "In-Ground-Effect Modeling and Nonlinear-Disturbance
  Observer for Multirotor UAV Control," *ASME J. Dyn. Sys., Meas., Control*,
  141(7), 2019. — IGE compensation via feed-forward and a nonlinear
  disturbance observer; the reference for the firmware path above.
- P. Sanchez-Cuevas, G. Heredia, A. Ollero, "Characterization of the Aerodynamic
  Ground Effect and Its Influence in Multirotor Control," *International Journal
  of Aerospace Engineering*, 2017. — Experimental quadrotor/octorotor IGE data.
