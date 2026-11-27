# Team Integration

How to drive this simulator from the PPO side, and what it emits on the sensor
side. Written so that neither teammate has to ask the simulation owner for a
basic specification.

Paths are relative to the repository root. The trimmed copies shipped inside the
supervisor/Drive bundle live in `evidence/PPO_Interface.md` and
`evidence/Sensor_Interface.md`; this page is the authoritative version.

---

## 0. Prerequisites for both teammates

| | |
|---|---|
| Simulator | Webots **R2025a**; set `WEBOTS_HOME` if not auto-detected |
| Python | 3.11+ (verified on 3.15.2) with `numpy` |
| PPO package | Spencer's `DroneNavPPO`, **not vendored here** |

`dns/spencer_link.py` puts Spencer's directory on `sys.path` at runtime so the
observation contract — `DT`, `SPEED`, `RADIUS`, `OBS_DIM`, `pack_scan`,
`validate` — can never drift from the trained checkpoints. Nothing in
`DroneNavPPO/` is copied, shadowed or modified. By default it is expected one
level above this repository:

```
<parent>/
├── webots_dns/        <- this repository
└── DroneNavPPO/
    └── DroneNavPPO/   <- navigation.py lives here
```

Anywhere else, point `DNS_PPO_DIR` at the directory containing `navigation.py`:

```bash
export DNS_PPO_DIR=/path/to/DroneNavPPO/DroneNavPPO     # bash
$env:DNS_PPO_DIR = "C:\path\to\DroneNavPPO\DroneNavPPO" # PowerShell
```

Only `python tests/unit_tests.py` runs without it.

---

# A. PPO integration — for Spencer

**Status:** verified against your unmodified `handoff.py` and `navigation.py`.
The SHA-256 digests your own `evidence/seed*/config.json` records for
`navigation.py`, `planar.py`, `ppo.py` and `train.py` still match the files on
disk.

## A1. The short version

The Webots backend implements your `Backend` protocol, so it drops straight into
`Navigation`:

```python
from dns.launcher import webots_session
with webots_session(world="worlds/DNS_Warehouse.wbt"):
    from dns.backend import WebotsBackend
    from navigation import Navigation          # yours, unmodified
    env = Navigation(WebotsBackend())
    obs, info = env.reset(seed=12, scene="moving")
    obs, reward, terminated, truncated, info = env.step([0.5, -0.25])
```

`obs` is the same 52-feature vector your `observation()` builds, because it *is*
built by your `observation()`. Nothing is normalised on our side.

## A2. Running it

```bash
# interface + physics check -- 22 assertions
python scripts/with_webots.py --timeout 420 --world worlds/DNS_Warehouse.wbt \
    tests/client_validation.py -- --config config/scenario_warehouse.json \
    --dynamics physical --sensing lidar

# the full 90-episode experiment
python scripts/with_webots.py --timeout 3000 --world worlds/DNS_Warehouse.wbt \
    tests/client_experiments.py -- \
    --dynamic-config config/scenario_warehouse.json \
    --static-config  config/scenario_warehouse_static.json \
    --out results/experiments_warehouse
```

`scripts/with_webots.py` starts Webots on a free IPC port, runs the client as a
child process with `libController` on its path, and shuts Webots down. The
child-process-plus-timeout design is deliberate: `Supervisor()` blocks forever
if no matching robot is waiting.

## A3. Coordinate convention

| | |
|---|---|
| +x | forward |
| +y | **right** of the drone |
| altitude | held at 1.0 m |
| heading | held at 0 rad |

Webots is ENU; the mapping is `x_dns = x_webots`, `y_dns = -y_webots`, defined
once in `dns/scenario.py` and never applied by hand anywhere else.

## A4. Start, goal, termination

| | Value |
|---|---|
| start | (0.85, 0.00) m |
| goal | (9.15, 0.00) m |
| success | drone centre within **0.4 m** of the goal (`navigation.GOAL_TOL`) |
| horizon | 90 steps = 18.0 s (`navigation.HORIZON`) |
| collision | swept test of the drone's 0.18 m disc against exact obstacle footprints and the walls |

Exactly one of `goal`, `collision`, `timeout` ends every episode. No episode is
dropped, so the denominator of any rate is the full cell.

## A5. Action and timing

- You pass **physical velocity** `[vx, vy]` in m/s, already scaled. We do not
  scale again.
- Per-axis cap **1.2 m/s** (`navigation.SPEED`). Exceeding it raises
  `BackendError` rather than silently clipping.
- One `step` advances exactly **0.200000 s** of simulation time (25 × 8 ms);
  measured timing error is below 3e-16 s.
- The command is held for the whole interval while a cascaded autopilot tracks
  it at 125 Hz underneath.

## A6. What comes back

`navigation.Frame`, all in metres and m/s, DNS axes:

| field | meaning |
|---|---|
| `time_s` | simulation seconds since reset (reset returns 0.0) |
| `xy` | drone position |
| `goal_xy` | goal position |
| `velocity` | **measured** from the GPS speed vector |
| `ranges` | 12 sector distances, metres, capped at 5.0 |
| `valid` | per-sector coverage flag |
| `collision` | ground-truth collision for this step |

**`velocity` is a measurement, not your command echoed back.** After one 0.2 s
step a commanded `[1.2, 0.0]` reads `[0.2801, -0.0008]` — that is the vehicle's
physical rise time. Measured exactly equal to commanded means you are in
`dynamics="kinematic"`, an interface-test mode that is labelled as such in every
record and is **not a flight model**.

## A7. Reset

`reset(seed, scene)` restores drone position, altitude, heading, linear and
angular velocity, obstacle poses and phase, and zeroes the episode clock.
Measured: two resets start within **2.758e-06 m** of each other and a
three-command trajectory reproduces to **7.091e-03 m**. Physical mode is not
bit-exact because the settle phase ends on a tolerance test — expected
behaviour for a feedback-controlled rigid body, not drift.

`seed` is recorded and, in the dynamic scenario, selects the cart's initial
crossing phase via `numpy default_rng(seed).uniform(0, 2*pi)`. It does **not**
randomise obstacle positions — see A9.

## A8. Worked example

`results/two_frames_webots_physical_lidar.json` is a real packet from the
warehouse world. It passes your verifier unmodified:

```bash
python "$DNS_PPO_DIR/handoff.py" --file results/two_frames_webots_physical_lidar.json
# PASS: command, timing, 52 features, reward, and episode flags
```

A readable dump, including the sector table and a self-consistency block:

```bash
python scripts/show_handoff.py results/two_frames_webots_physical_lidar.json
```

Note the packet layout: `frames[i]` carries **exactly** the `navigation.Frame`
fields, because your verifier does `Frame(**raw)` and any extra key is a
`TypeError`. Our richer per-step record — altitude, attitude, mount mode,
contact state, commanded velocity — lives under `dns_records`, a key your code
never reads.

## A9. Before you quote any number

- **One fixed obstacle layout.** The dynamic condition varies *when* the cart
  arrives, not *where* obstacles are. Nothing here supports a generalisation
  claim. Randomised layouts are the next piece of work on our side.
- **Physical mode is near-deterministic.** Repeated episodes on a fixed layout
  measure run-to-run variation, not scenario variation. Vary the seed — and
  therefore the phase — instead.
- **The scene is harder than the planar baseline.** Racks are long flat surfaces
  rather than discs and they straddle the straight line to the goal. Your
  checkpoints reach the goal in every static episode and in **46.7%–60%** of
  dynamic episodes; the spread *between seeds* is as wide as the gap between
  conditions.
- **Our baselines are untuned for this scene** and are not a ranking of methods.
  The reactive rule's repulsion radius (1.45 m) exceeds the crate-to-goal
  distance (1.54 m), so it stalls **1.61 m** short without ever colliding.
- **One scan layer, idealised gimbal.** Both are modelling choices, labelled in
  every record.

---

# B. LiDAR integration — for the sensor and perception teammate

**Status:** sensor installed, angular mapping measured, output validated against
an independent analytic ray-cast.

You own the processing. This covers what the simulator mounts, what it emits,
and which limitations you will have to design around.

## B1. Sensor configuration as built

| Property | Value | Notes |
|---|---|---|
| Device | Webots `Lidar`, `type "fixed"` | base node, not a custom model |
| Horizontal field of view | **360°** (6.283185 rad) | `projection "cylindrical"`, required by Webots above π |
| Beams per sweep | **720** | 0.5° spacing, 60 per 30° sector |
| Layers | **1** | `verticalFieldOfView 0.01` — a razor-thin plane |
| Range | 0.05 m min, **5.0 m** max | cap matches `navigation.RANGE` |
| Noise | 0.0 | ideal; add noise deliberately for a robustness study |
| Mount height | **0.14 m** above the body origin | ≈1.14 m above the floor at cruise |
| Mount mode | **gimbal-stabilised** | counter-rotated every 8 ms so the plane stays horizontal |

**The beam count changed from 360 to 720.** If you read the raw range image,
call `lidar.getHorizontalResolution()` rather than assuming a number; the packet
records it as `lidar_beams`.

## B2. Angular convention

| Sector | Bearing | Direction |
|---|---|---|
| 0 | 0° | forward |
| 3 | 90° | right |
| 6 | 180° | back |
| 9 | 270° | left |

Angles increase **clockwise** from the drone's forward axis. Sector *i* is
centred on *i* × 30° and spans ±15°.

**The index-to-bearing mapping was measured, not assumed.** The implementation
(`dns/sensing.py`) uses `direction = +1`, `offset = 180.0°`, the value fitted in
`results/calibration.json` (RMS **0.0147 m**, 641 finite beams). The warehouse
calibration fits `offset = 180.5°` (RMS **0.0245 m**, 671 of 720 beams finite;
`results/calibration_warehouse.json`). The half-degree between the two fits is one
beam pitch. Do not assume the raw Webots beam index order equals clockwise
navigation bearing — it does not, and the offset is why.

```bash
python scripts/with_webots.py --world worlds/DNS_Warehouse.wbt \
    tests/client_calibrate.py -- --config config/scenario_warehouse.json \
    --out results/calibration_warehouse.json
```

## B3. What the packet contains

`results/two_frames_webots_physical_lidar.json` — two consecutive frames from a
real run of `DNS_Warehouse.wbt`, **exactly 0.200000 s apart** in simulation time.

```
time_s     simulation seconds since reset (frame 0 is 0.0)
xy         drone position [m], DNS frame
goal_xy    goal position [m]
velocity   measured drone velocity [m/s]
ranges     12 sector distances [m], raw physical units, capped at 5.0
valid      12 booleans
collision  ground-truth collision flag
```

Richer per-step state — altitude, roll, pitch, yaw, commanded velocity, mount
mode, beam count, Webots contact flag — sits under `dns_records`.

**Nothing is normalised.** Scaling happens once, downstream, in the PPO
observation builder. Do not divide by 5 before handing values on.

## B4. The encoding you must respect

| Case | Encoding |
|---|---|
| surface seen at *d* metres | `(d, True)` |
| observed, nothing within 5 m | `(5.0, True)` — *observed clear*, not unknown |
| sector genuinely unobserved | `(0.0, False)` |

A beam beyond `maxRange` returns `inf` from Webots and becomes `(5.0, True)`.
That is correct in open space and **wrong whenever the beam escapes the room**,
which is the failure mode described in B5.

## B5. Known limitations — design against these

### B5.1 Attitude

A quadcopter tilts to accelerate, up to **25.78°**. With a body-fixed mount the
scan plane tilts with it and measurement degrades sharply: worst-case sector
range error rose to **0.197 m** against **0.071 m** for the stabilised mount
over identical manoeuvres. Before the walls were raised, a body-fixed plane at
1.075 m under 1.2 m walls lost the wall entirely past about 6° of tilt and
reported "clear to 5 m" over a solid surface in up to 5 of 12 sectors at once.

The delivered configuration is **stabilised**, so you receive a horizontal
plane. The gimbal is **idealised**: zero mass, zero lag, no joint limits. A
physical stabilised mount has all three, so attitude robustness remains open on
hardware. `sensing.mount` switches back to `body_fixed` to study the
degradation; every record carries the mode it was produced under. Full study:
[`SENSOR_ATTITUDE.md`](SENSOR_ATTITUDE.md).

### B5.2 Single scan layer

The sensor sees only what crosses its plane at ≈1.14 m. An obstacle shorter than
that is solid to the collision test and invisible to the LiDAR. The scenario
validator now refuses to load such a scene, which closes the worst case, but
height remains a modelling constraint rather than something the sensor
discovers.

### B5.3 Grazing at sector boundaries

Where an obstacle overlaps a sector boundary by less than one beam pitch, that
sector can miss it entirely and report the 5 m cap while the **neighbouring**
sector holds the return. The surface is not lost, only re-attributed. Halving
the beam pitch (360 → 720) reduced this but did not remove it.

Practical consequence: **do not treat a single clear sector flanked by a near
return as verified free space.** Obstacle bearing carries a ±1 sector ambiguity
near boundaries.

### B5.4 `geometric` mode is not a sensor

`sensing.mode = "geometric"` is a debug reference that ray-casts from supervisor
ground truth using only 3 sub-rays per sector. It systematically **over-states
clearance** relative to the real 60-beam scan — up to 0.24 m in the measured
comparison, always optimistic. Never compare a `lidar` episode against a
`geometric` one as if the clearance signal were the same.

## B6. Measured sensor quality

Against an independent analytic ray-cast of the same scene — a separate
implementation using the slab method on the true footprints, including occlusion
— across a dynamic episode with the cart crossing:

- worst per-sector disagreement in any step: **0.2900 m** (a boundary graze)
- mean worst-per-step disagreement: **0.0475 m** over 90 steps × 12 sectors
- where the cart is the nearest surface along its own bearing, the measured
  range tracks its true clearance to **0.009 m** mean, 0.037 m max

Occlusion behaves correctly: when a rack stands between the drone and the cart,
the sector reports the rack. That is why validation compares against the full
analytic scene rather than against the cart's own distance.

## B7. Calibration scene

`DNS_Warehouse.wbt` doubles as the calibration scene. Useful targets:

- `rack_a`, `rack_b` — 2.4 m flat faces, the cleanest planar returns
- `column_1`, `column_2` — 0.4 × 0.4 m squares at known positions
- the four walls at x = 0, x = 10, y = −3, y = +3

With the drone at the start pad (0.85, 0.00), sector 6 reads **0.850 m** against
the wall 0.85 m behind it, and sectors 3 and 9 read **3.000 m** against the side
walls 3.0 m away. Those three numbers are the quickest check that a pipeline has
the convention right.

---

## C. Flagged interface changes

Changes that affect a teammate are reported rather than applied silently:

| Change | From → To | Why | Action needed |
|---|---|---|---|
| LiDAR beam count | 360 → **720** | beam pitch bounds worst-case grazing error | read `getHorizontalResolution()`, don't hard-code |
| Arena wall height | 1.2 m → **2.5 m** | at 1.2 m, >6° tilt sent beams over the wall top | none, but old scans are not comparable |
| Experiment `protocol.json` `world` field | hard-coded → queried from Webots | the literal misreported which world was loaded | re-read provenance from regenerated files |
