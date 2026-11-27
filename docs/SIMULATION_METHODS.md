# Design and Verification of a Webots Simulation Environment for UAV Navigation in Static and Dynamic Scenarios

> [!NOTE]
> **Historical document — v1 configuration.** The numbers below describe
> the configuration as it was when this was written (1.075 m scan plane (0.075 m mount)). The current
> build flies at **1.0 m** with a **0.14 m** LiDAR mount, so the scan plane
> rides at **1.14 m**; see `protos/DnsQuadcopter.proto` (`lidarHeight`) and
> `config/scenarios/L00_C.json`. Kept unedited as the record of what was
> measured at the time.


Simulation Environment Development — Drone Navigation System, Taylor's
University. Supervisor: (name withheld by request).

This document records the simulation configuration and the engineering reasons
behind it. Measured results and their evidence files are in
[VALIDATION_EVIDENCE.md](VALIDATION_EVIDENCE.md); the teammate-facing contracts
are in [TEAM_INTEGRATION.md](TEAM_INTEGRATION.md).

---

## 1. Objectives

The simulation environment has to support a specific research question — does a
PPO-trained navigation policy behave differently when an obstacle *moves* rather
than stands still — without becoming the thing under study. That imposes four
requirements:

1. **Fidelity sufficient to be non-trivial.** The drone must fly on rotor thrust
   against rigid-body physics. A kinematic integrator would make every result an
   artefact of the integrator.
2. **An interface identical to the training environment.** The policies were
   fitted against a planar benchmark; if the simulator's contract differs even
   slightly, any behavioural difference is confounded.
3. **Exact repeatability.** Two runs of the same seed must produce the same
   episode, or nothing measured across episodes means anything.
4. **A discriminative scene.** If every controller succeeds, the environment
   measures nothing.

Requirement 4 was not satisfied by the first layout, and fixing it is described
in §5.

---

## 2. Choice of simulator

Webots R2025a was selected over the obvious alternatives for four concrete
reasons:

- **Python controllers natively**, so the environment, the PPO bridge and the
  tests share one language and one process model.
- **The sensor and rotor models needed already ship as base nodes** — `Lidar`,
  `Propeller`, `InertialUnit`, `GPS`, `Gyro`. The airframe is therefore built
  from base nodes only, with no `EXTERNPROTO` download, so a world loads with no
  network access.
- **A `Supervisor` API that can reset an experiment exactly**, including
  obstacle poses and phase — requirement 3.
- **Worlds are plain text**, so they can be *generated* from a configuration
  file rather than hand-maintained (§3).

The alternative that came closest was Gazebo, which has stronger ROS
integration; that was not useful here because no part of this project uses ROS,
and its Python story is considerably weaker.

---

## 3. Configuration-driven world generation

Worlds are written by `dns/worldgen.py` from a scenario JSON, not edited by
hand:

```bash
python -m dns.worldgen --config config/scenario_warehouse.json \
    --out worlds/DNS_Warehouse.wbt
```

The reason is drift. Arena size, start, goal, obstacle geometry and sensor
configuration appear in the world file, in the collision model, in the sensing
model and in the figures. Maintaining a `.wbt` by hand guarantees that these
disagree eventually, usually silently. With generation, `config/*.json` is the
single source of truth and the scenario validator (§10) rejects a configuration
before it can produce a misleading run.

The consequence to remember: **editing a `.wbt` directly is pointless**, because
the next generation overwrites it.

---

## 4. Geometry and coordinate conventions

### 4.1 Frames

| | |
|---|---|
| DNS +x | forward |
| DNS +y | **right** of the drone |
| altitude | held at 1.0 m |
| heading | held at 0 rad |

Webots is ENU with +x east, +y north, +z up. The planar mapping is
`x_dns = x_webots`, `y_dns = −y_webots`, defined once in `dns/scenario.py`
(`to_webots` / `to_dns`, `bearing_to_webots` / `bearing_to_dns`) and applied
nowhere else by hand. A sign error in this transform would invalidate every
result, which is why checks 10–13 of the validation suite exercise all four
commanded directions independently.

### 4.2 Arena

| Property | Value |
|---|---|
| Extent | `low_xy = [0.0, −3.0]`, `high_xy = [10.0, 3.0]` → **10.0 × 6.0 m** |
| Wall height | 2.5 m |
| Wall thickness | 0.1 m |
| Floor | 1 m reference grid |
| Start | (0.85, 0.00) m, green pad |
| Goal | (9.15, 0.00) m, blue pad, success radius **0.4 m** |
| Cruise altitude | 1.0 m |

**The 10 × 6 m extent is deliberate and is not a free parameter.** Spencer's
checkpoints were fitted in `planar.py` on exactly `LOW=[0,-3]`, `HIGH=[10,3]`,
and `navigation.observation` divides the relative goal vector by 10 while
capping LiDAR range at 5 m. Enlarging the room to, say, 12 × 8 m would move the
goal feature and the free-space statistics outside the distribution the policies
saw, so any difference between conditions would be confounded by the arena
change rather than caused by the obstacle layout. A larger hall is a legitimate
out-of-distribution generalisation study and must be reported as one.

**Wall height was raised from 1.2 m to 2.5 m.** At 1.2 m only 0.125 m of wall
stood above the 1.075 m scan plane. Any airframe tilt beyond about 6° therefore
sent the beams over the top, and the sensor reported open space where a solid
wall stood — in up to 5 of 12 sectors simultaneously. At 2.5 m the walls stay in
view across the full altitude and tilt envelope (1.36 m of headroom above the
1.14 m scan plane against a maximum autopilot tilt of 25.78°).

---

## 5. Static obstacles

| id | Shape | Centre (DNS) | Footprint | Height | Research purpose |
|---|---|---|---|---|---|
| `rack_a` | box | (3.50, −1.00) | 0.8 × 2.4 m | 2.0 m | Blocks the left half of the aisle, forcing the first lateral deviation. Its flat 2.4 m face is the primary LiDAR calibration target — a plane returns a predictable range profile, a cylinder does not. |
| `rack_b` | box | (6.50, +1.00) | 0.8 × 2.4 m | 2.0 m | Mirrors rack A, forcing a second deviation the other way, so a policy cannot succeed by committing to one side. |
| `column_1` | box | (2.00, +2.30) | 0.4 × 0.4 m | 2.5 m | Off-route building column; a precisely known square return for sector calibration, plus a wall-clearance hazard. |
| `column_2` | box | (8.00, −2.30) | 0.4 × 0.4 m | 2.5 m | Mirror column near the opposite boundary, late in the route. |
| `crate_stack` | box, yaw 0.35 rad | (7.90, +0.90) | 0.7 × 0.7 m | 1.6 m | Low pallet load off the final approach; tests whether clearance is kept while converging on the goal. |

### 5.1 Why the layout changed

An earlier layout placed the racks clear of the centre line by 0.30 m. The
goal-direct baseline then threaded the gap and scored 3/3, so the static
condition measured nothing — a failure of requirement 4 in §1.

`rack_a` now occupies y ∈ [−2.20, +0.20] at x ∈ [3.1, 3.9], and `rack_b`
occupies y ∈ [−0.20, +2.20] at x ∈ [6.1, 6.9]. Each overlaps y = 0 by 0.20 m.
After inflating by the 0.18 m drone radius, passing `rack_a` requires
y ≥ +0.38 m and passing `rack_b` requires y ≤ −0.38 m, which forces an S-shaped
route. The recorded trajectories (`assets/figures/trajectories.png`) show
exactly that S.

### 5.2 Navigability

The scene must be hard, not impossible. Free widths at the two rack stations are
2.44 m on the main side and 0.44 m on the far side; the cart blocks 0.91 m of
the 5.64 m usable width at x = 5.1 m, leaving at least 2.37 m beside it. Every
passage exceeds the 0.36 m drone diameter, so a feasible route always exists —
including a narrower alternative on the far side of each rack.

---

## 6. Dynamic obstacle

| Property | Value |
|---|---|
| id | `cart` — powered transfer cart |
| Footprint | 0.9 × 0.55 m, height 1.5 m |
| Station | x = 5.10 m, crossing the route perpendicular to travel |
| Motion | `linear`, axis y, amplitude **A = 1.9 m**, speed **v = 0.45 m/s** |
| Period | T = 4A/v = **16.889 s** |
| Phase | per-seed, `numpy default_rng(seed).uniform(0, 2π)` |

### 6.1 Motion model

`linear` motion is a constant-speed triangle wave, not a sinusoid. For normalised
phase

$$u(t) = \left(\frac{t}{T} + \frac{\varphi}{2\pi}\right) \bmod 1,
\qquad T = \frac{4A}{v}$$

the lateral offset is

$$\Delta y(t) = A\,\bigl(1 - 4\,\lvert u(t) - 0.5 \rvert\bigr)$$

which runs from −A at u = 0 through +A at u = 0.5 and back. A full cycle covers
4A of travel, so specifying an explicit speed fixes the period and vice versa.
The implementation is `dns/scenario.py`, and `scenario_default.json` additionally
supports an `oscillate` type with `Δy = A sin(2πt/T + φ)` for a sinusoidal
profile.

Constant speed was chosen over a sinusoid deliberately: it makes the timing
problem uniform across the crossing rather than concentrating the difficulty at
the fast mid-stroke, and it makes "the cart is travelling at 0.45 m/s" a
checkable statement at every instant rather than only on average.

### 6.2 What kind of motion this is

The cart is **kinematically driven**: the Supervisor writes its pose each step
from the equation above. It is not a rigid body under contact forces. It cannot
be pushed, it does not push, and it carries no momentum. The drone's collision
against it is detected geometrically.

This is a modelling choice with a clear cost, and it must not be presented as
physical realism. It buys exact repeatability — the cart replays across resets
to 2.554e-15 m — which is what makes paired comparison between the static and
dynamic conditions meaningful at all.

### 6.3 The static counterpart

`config/scenario_warehouse_static.json` is identical except that the cart is
parked at (5.10, −2.50) against the wall with `amplitude_m: 0.0`. Keeping it
*present but stationary* rather than deleting it holds the obstacle count and
the LiDAR scene constant, so the two modes differ in exactly one factor:
motion.

---

## 7. Drone model

The airframe is a **custom research PROTO**, `protos/DnsQuadcopter.proto`. It is
not a Crazyflie, a Mavic or any other commercial platform, and it does not
represent physical hardware. There is no flight-controller board, microcontroller
or physical sensor in this model — only Webots base nodes. The Webots Mavic2Pro
sample is cited in the PROTO's comments purely as construction precedent for
graphical-only rotor geometry.

### 7.1 Mass and geometry

| Property | Value |
|---|---|
| Total mass | 0.55 kg (0.35 kg body box + 4 × 0.05 kg rotor groups as point masses) |
| Inertia | Ixx = Iyy = 0.00272, Izz = 0.00513 kg·m² |
| Rotor hubs | (±a, ±a) with a = 0.09 m → hub radius 0.1273 m |
| Rotor radius | 0.05 m → tip radius 0.1773 m |
| `boundingObject` | Cylinder, radius **0.18 m** |

The collision radius is not arbitrary: the 0.1773 m tip radius rounds to the
0.18 m that `navigation.RADIUS` uses, so Webots contact physics and the
geometric collision model agree on the same disc.

Rotor layout is an X configuration in the Webots robot frame (+x front, +y left,
+z up): m0 front-left CCW, m1 front-right CW, m2 rear-left CW, m3 rear-right CCW.

### 7.2 Propulsion

Webots' `Propeller` node produces, along the shaft axis,

$$T = t_1 \lvert\omega\rvert\,\omega - t_2 \lvert\omega\rvert V,
\qquad Q = q_1 \lvert\omega\rvert\,\omega - q_2 \lvert\omega\rvert V$$

with `thrustConstant` t₁ = 1.5 × 10⁻⁵ N·s² and `torqueConstant`
q₁ = 2.4 × 10⁻⁷ N·m·s². CW rotors are driven with negative ω and carry
t₁ = −`thrustConstant`, so every rotor produces +z thrust while Q keeps the sign
of ω and cancels in hover.

Derived and measured operating points:

| Quantity | Value | Source |
|---|---|---|
| Hover thrust per rotor | 1.3489 N | mg/4, confirmed in `results/calibration_warehouse.json` |
| Hover rotor speed | ≈ 299.9 rad/s | √(1.3489 / t₁) |
| Maximum rotor thrust | 5.4 N at 600 rad/s | `results/thrust_curve.json` |
| Thrust-to-weight | **4.003** | measured |
| Yaw moment per newton | 0.016 m | measured |
| Linear damping | 0.695 s⁻¹ | `results/drag_calibration.json`, terminal velocity 14.11 m/s rotors-off |

### 7.3 Two non-physical parameters, declared

`motorMaxTorque` is set to 2000 N·m and is **not a physical motor rating**.
Rotor speed inside Webots' propeller model ramps at a rate proportional to
`maxTorque`: at the usual 30 N·m the rotors needed about 8 s to reach a
commanded 424 rad/s and the vehicle could not take off. At 2000 they reach 97%
of the commanded speed in 0.1 s, which moves the physically meaningful limit to
`motorAcceleration` = 5000 rad/s² (0 → 600 rad/s in 0.12 s), a realistic figure
for a small quadcopter. Measured in `tests/client_spinup.py`,
`results/spinup.json`.

The helix `Solid` nodes are **graphical only** — no physics, no
`boundingObject`. Giving them their own collider places it inside the airframe's
own bounding cylinder; the resulting permanent contact absorbed several newtons
of thrust and stopped the vehicle taking off. This was isolated by the bisection
ladder retained in `worlds/_shell_*.wbt` and `worlds/_quad_*.wbt` with its logs
in `results/`.

---

## 8. Flight control

The policy issues one horizontal velocity setpoint every 0.2 s. A quadcopter
cannot track that instantaneously — it must tilt before it can accelerate — so
the setpoint is held while `dns/autopilot.py` closes three nested loops at the
8 ms basic timestep (125 Hz):

```
velocity  ->  tilt target        (outer, slow)
attitude  ->  body torque        (inner, fast)
altitude  ->  collective thrust  (independent)
```

| Gain / limit | Value |
|---|---|
| `kp_altitude` | 2.0 s⁻¹ (altitude error → climb rate) |
| `kp_climb` | 5.0 s⁻¹ (climb-rate error → acceleration) |
| `max_climb_rate` | 1.0 m/s |
| `integral_limit` | 0.6 m·s⁻¹·s |
| `max_tilt_rad` | 0.45 rad ≈ **25.78°**, ≈ 4.75 m/s² of lateral authority |

Nothing in this module writes the drone's position. Every motion comes out of
rotor thrust applied by Webots to a rigid body — which is what makes check 14 of
the validation suite (a commanded 1.2 m/s reading 0.2801 m/s after one step)
the expected result rather than a defect.

Sign conventions were cross-checked against Webots' own `mavic2pro.c` sample and
re-verified empirically by `tests/client_calibrate.py`: positive pitch tilts the
nose down and accelerates along +x; positive roll drops the right side and
accelerates along −y(Webots) = +y(DNS). Both horizontal axes therefore use a
positive tilt target for a positive demand.

---

## 9. Sensing

Full specification and limitations: [TEAM_INTEGRATION.md §B](TEAM_INTEGRATION.md).
Mount study: [SENSOR_ATTITUDE.md](SENSOR_ATTITUDE.md). In summary:

- Webots `Lidar`, `type "fixed"`, `projection "cylindrical"`, 360° horizontal
  field of view, **720 beams** (0.5°, 60 per sector), **1 layer**, 0.05–5.0 m,
  zero noise.
- Mounted 0.14 m above the body origin → scan plane ≈ 1.14 m at cruise.
- **Gimbal-stabilised**: counter-rotated each 8 ms tick so the plane stays
  horizontal. Idealised — zero mass, zero lag, no joint limits.
- Binned into 12 × 30° sectors by Spencer's `navigation.pack_scan`; sector 0
  forward, 3 right, 6 back, 9 left, angles increasing clockwise.

Beam count was raised from 360 to 720 because beam pitch bounds the worst-case
grazing error: an obstacle overlapping a sector boundary by less than one beam
pitch can be missed by that sector entirely and read as clear-to-cap. Halving
the pitch reduced the effect without removing it; the residual is a ±1 sector
bearing ambiguity near boundaries.

Three sensing modes exist and are kept strictly separate so nothing fabricated
can leak into a result that looks measured:

| Mode | What it is |
|---|---|
| `lidar` | the real device, binned |
| `none` | every sector `(0.0, False)` = unobserved — the correct default before sensor processing is delivered |
| `geometric` | **ground-truth** ray-cast from supervisor state, 3 sub-rays per sector. A debug reference, **not a sensor**; it over-states clearance by up to 0.24 m, always optimistically. |

---

## 10. Timing, collision and validation of the configuration

### 10.1 Timing

| | |
|---|---|
| Physics step | 8 ms |
| Control period | 0.2 s = **25 steps**, exactly |
| Command rate | 5 Hz, matching `navigation.DT` |
| Episode horizon | 90 steps = 18.0 s (`navigation.HORIZON`) |

`Scenario.steps_per_command` refuses a configuration where the command period is
not an exact multiple of the basic timestep, because a fractional step would
make the 0.2 s contract approximate. Measured timing error is below 3 × 10⁻¹⁶ s.

### 10.2 Collision geometry

Collision mode is `geometric`: a swept-segment test of the drone's 0.18 m disc
against obstacle footprints and the arena walls, matching `planar.py` so the
contract is identical to PPO training. Boxes are tested against their **exact
rectangular footprint** via a half-size slab test, including the crate's 0.35 rad
yaw; the bounding radius `hypot(sx/2, sy/2)` is only a broad-phase convenience.
`physical` (Webots contact points) and `either` modes also exist.

### 10.3 Scenario validation

`dns/scenario.py` refuses to load a configuration that would produce a
misleading run. The validator rejects, among others:

- an obstacle shorter than the scan plane (solid to collision, invisible to the
  sensor)
- overlapping obstacles
- a start point inside a wall, or a goal on top of an obstacle
- a LiDAR mount inside the rotor disc
- a command period that is not a whole number of timesteps
- unknown dynamics / sensing / collision modes

Each of these is covered by the 46 unit tests.

### 10.4 Reset and reproducibility

`reset(seed, scene)` restores drone pose, altitude, heading, linear and angular
velocity, obstacle poses and phase, and zeroes the episode clock. Measured: two
resets start within **2.758 × 10⁻⁶ m**; a three-command trajectory reproduces to
**7.091 × 10⁻³ m**. Physical mode is not bit-exact because the settle phase ends
on a tolerance test — the expected behaviour of a feedback-controlled rigid
body, not drift. Re-running the full 90-episode experiment reproduced
`experiment_summary.csv` byte for byte.

---

## 11. Limitations

Stated in full, with evidence, in
[VALIDATION_EVIDENCE.md §7](VALIDATION_EVIDENCE.md). The modelling assumptions
that belong to this document specifically:

1. **Kinematic obstacle motion** (§6.2) — no contact dynamics for the cart.
2. **Idealised gimbal** — massless, no lag, no joint limits.
3. **One scan layer** — the sensor is blind to anything not crossing 1.14 m.
4. **Fixed altitude and yaw** — the problem is solved in the horizontal plane.
5. **Zero sensor noise** — noise should be introduced deliberately as a
   robustness study rather than left implicit.
6. **Two declared non-physical parameters** (§7.3) — `motorMaxTorque` and the
   graphical-only helices.
7. **One fixed obstacle layout** — the arena is held at 10 × 6 m for the reason
   in §4.2, and obstacle positions do not vary. Randomised layouts and an
   out-of-distribution arena study are future work, not completed work.
8. **No real-world validation.** No physical drone was flown.
