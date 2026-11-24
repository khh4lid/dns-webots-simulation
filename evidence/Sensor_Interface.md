# Sensor Interface — DNS Webots LiDAR

**For:** the LiDAR / obstacle-detection teammate
**From:** simulation environment
**Status:** sensor installed, angular mapping measured, output validated
against an independent analytic ray-cast.

You own the processing. This document covers what the simulator mounts, what it
emits, and which of its limitations you will have to design around.

---

## 1. Sensor configuration as built

| Property | Value | Notes |
|---|---|---|
| Device | Webots `Lidar`, `type "fixed"` | base node, not a custom model |
| Horizontal field of view | **360 deg** (6.283185 rad) | `projection "cylindrical"`, which Webots requires above pi |
| Beams per sweep | **720** | 0.5 deg spacing, 60 per 30 deg sector |
| Layers | **1** | `verticalFieldOfView 0.01` — a razor-thin plane |
| Range | 0.05 m min, **5.0 m** max | cap matches `navigation.RANGE` |
| Noise | 0.0 | ideal; add noise deliberately for a robustness study |
| Mount height | **0.14 m** above the drone body origin | ~1.14 m above the floor at cruise altitude |
| Mount mode | **gimbal-stabilised** | counter-rotated every 8 ms tick so the scan plane stays horizontal |

**Beam count changed from 360 to 720.** If you read the raw range image, call
`lidar.getHorizontalResolution()` rather than assuming a number. The packet
records it as `lidar_beams`.

## 2. Angular convention

Agreed and implemented:

| Sector | Bearing | Direction |
|---|---|---|
| 0 | 0 deg | forward |
| 3 | 90 deg | right |
| 6 | 180 deg | back |
| 9 | 270 deg | left |

Angles increase **clockwise** from the drone's forward axis. Sector *i* is
centred on *i* x 30 deg and spans +/-15 deg.

**The index-to-bearing mapping was measured, not assumed.** Fitting it against
known arena geometry gives `direction = +1`, `offset = 180.50 deg`, RMS residual
**0.0245 m** over 720 beams. The residual half-degree is one beam pitch. Do not
assume the raw Webots beam index order equals clockwise navigation bearing — it
does not, and the offset is why.

Reproduce:

```bash
python scripts/with_webots.py --world worlds/DNS_Warehouse.wbt \
    tests/client_calibrate.py -- --config config/scenario_warehouse.json \
    --out results/calibration_warehouse.json
```

## 3. What the packet contains

`04_TEAM_HANDOFF/two_frames.json` — two consecutive frames from a real run of
`DNS_Warehouse.wbt`, **exactly 0.200000 s apart** in simulation time.

Per frame:

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
mode, beam count, Webots contact flag — is under `dns_records`.

**Nothing is normalised.** Scaling happens once, downstream, in the PPO
observation builder. Do not divide by 5 before handing values on.

## 4. The encoding you must respect

| Case | Encoding |
|---|---|
| surface seen at d metres | `(d, True)` |
| observed, nothing within 5 m | `(5.0, True)` — *observed clear*, not unknown |
| sector genuinely unobserved | `(0.0, False)` |

A beam that exceeds `maxRange` returns `inf` from Webots, and that becomes
`(5.0, True)`. This is correct in open space and **wrong whenever the beam
escapes the room**, which is exactly the failure mode described next.

## 5. Known limitations — please design against these

### 5.1 Attitude (fixed here, but understand it)

A quadcopter tilts to accelerate, up to 25.8 deg. With a body-fixed mount the
scan plane tilts with it, and measurement degrades sharply: worst-case sector
range error rose to **0.197 m** against **0.071 m** for the stabilised mount
over identical manoeuvres. Before the arena walls were raised, a body-fixed
plane at 1.075 m under 1.2 m walls lost the wall entirely past about 6 deg of
tilt and reported "clear to 5 m" over a solid surface in up to 5 of 12 sectors
at once.

The delivered configuration is **stabilised**, so you receive a horizontal
plane. The gimbal is **idealised**: zero mass, zero lag, no joint limits. A
physical stabilised mount has all three, so treat attitude robustness as still
open on hardware. `sensing.mount` in the scenario file switches back to
`body_fixed` if you want to study the degradation; every record carries the
mode it was produced under.

### 5.2 Single scan layer

The sensor sees only what crosses its plane at ~1.14 m. An obstacle shorter than
that is solid to the collision test and invisible to the LiDAR. The scenario
validator now refuses to load a scene containing one, which closes the worst
case, but height remains a modelling constraint rather than something the
sensor discovers.

### 5.3 Grazing at sector boundaries

Where an obstacle overlaps a sector boundary by less than one beam pitch, that
sector can miss it entirely and report the full 5 m cap while the **neighbouring**
sector holds the return. The surface is not lost, only re-attributed. Halving
the beam pitch (360 -> 720) reduced this but did not remove it.

Practical consequence: **do not treat a single clear sector flanked by a near
return as verified free space.** Obstacle bearing carries a +/-1 sector ambiguity
near boundaries.

### 5.4 The `geometric` mode is not a sensor

`sensing.mode = "geometric"` is a debug reference that ray-casts from supervisor
ground truth using only 3 sub-rays per sector. It systematically **over-states
clearance** relative to the real 60-beam scan — up to 0.24 m in the measured
comparison, always in the optimistic direction. Never compare a `lidar` episode
against a `geometric` one as if the clearance signal were the same.

## 6. How good is the sensor, measured

Against an independent analytic ray-cast of the same scene — a separate
implementation using the slab method on the true obstacle footprints, including
occlusion — across a dynamic episode with the cart crossing:

- worst per-sector disagreement in any step: **0.29 m** (a boundary graze)
- mean worst-per-step disagreement: **0.048 m**
- on the steps where the cart is the nearest surface along its own bearing,
  the measured range tracks its true clearance to **0.009 m mean**, 0.037 m max

Occlusion behaves correctly: when a rack stands between the drone and the cart,
the sector reports the rack. That is why the validation compares against the
full analytic scene rather than against the cart's own distance.

## 7. Calibration scene

`DNS_Warehouse.wbt` doubles as the calibration scene. Useful targets:

- `rack_a`, `rack_b` — 2.4 m flat faces, the cleanest planar returns
- `column_1`, `column_2` — 0.4 x 0.4 m squares at known positions
- the four walls at x = 0, x = 10, y = -3, y = +3

With the drone at the start pad (0.85, 0.0), sector 6 reads **0.850 m** against
the wall 0.85 m behind it, and sectors 3 and 9 read **3.000 m** against the side
walls 3.0 m away. Those three numbers are the quickest sanity check that your
pipeline has the convention right.
