# Why the lidar is gimballed: four measured sensor defects

> [!NOTE]
> **Historical document — v1 configuration.** The numbers below describe
> the configuration as it was when this was written (1.2 m walls). The current
> build flies at **1.0 m** with a **0.14 m** LiDAR mount, so the scan plane
> rides at **1.14 m**; see `protos/DnsQuadcopter.proto` (`lidarHeight`) and
> `config/scenarios/L00_C.json`. Kept unedited as the record of what was
> measured at the time.


**Status:** resolved, with evidence. **Owner:** Khaled (simulation environment).
**Audience:** the supervisor, Spencer (PPO), sensor/obstacle-detection teammate.

This note records four problems found in the first real lidar integration, how
each was measured, what was changed, and what residual error remains. All
numbers come from a running Webots R2025a simulation; nothing here is estimated.

Reproduce everything with:

```bash
cd webots_dns
python -m dns.worldgen
python scripts/with_webots.py --timeout 600 tests/client_lidar_attitude.py
python scripts/with_webots.py --timeout 420 tests/client_calibrate.py
```

Raw output: `results/lidar_attitude.json`, `results/calibration.json`,
`results/gimbal_probe.json`.

---

## Why this mattered enough to stop and fix

Spencer's contract (`navigation.py`) describes a **flat** world: 12 sectors of
30 degrees, each holding the nearest obstacle distance in a horizontal plane,
and `pack_scan` encodes an out-of-range beam as `(5.0, True)` — *observed clear
to 5 m*. `planar.py`, which trained the policies, ray-casts in 2-D against
height-less circles.

A quadcopter has no choice but to tilt in order to accelerate: the autopilot's
tilt authority is `Gains.max_tilt_rad = 0.45` rad (25.8 deg), and the measured
manoeuvres below reach 19 deg. A single-layer lidar bolted to that airframe
does not deliver a horizontal plane, so the sensor silently broke the contract
it was supposed to satisfy — in the unsafe direction.

---

## Defect 1 — a tilted scan plane flew over the walls and reported open space

**Mechanism.** The original mount put the lidar at 1.0 + 0.075 = 1.075 m while
the arena walls were 1.2 m, leaving **0.125 m of wall above the scan plane**.
Rolling or pitching by `asin(0.125 / d)` — about 6 deg at d = 1.2 m — lifts the
beams on the rising side over the wall top. Webots then returns `inf`, and
`pack_scan` turns that into `(5.0, True)`.

**Measurement.** `tests/client_lidar_attitude.py` flies hover / hard-right /
hard-left / hard-forward / diagonal commands and, at every control step,
compares the 12 measured sectors against an analytic ray-cast taken from the
drone's true pose **on the lidar's own bearings** (so the reference is not
coarser than the measurement). A sector is scored `phantom-clear` when it
reports the 5 m cap with `valid=True` while ground truth puts a surface at
<= 4.5 m.

| Condition (original geometry) | Phantom-clear sectors | Paired RMS |
|---|---|---|
| Level hover, tilt <= 2 deg | **0 of 12** | 0.035 m |
| Tilt > 5 deg | **mean 3.4 of 12, max 5** | up to 0.472 m |

The pattern matched the geometry exactly, which is what confirmed the cause:

| Manoeuvre | Attitude | Sectors lost | Direction lost |
|---|---|---|---|
| hard right | roll +17.5 deg (right side drops, left rises) | 8, 9, 10 | left |
| hard left | roll −17.5 deg (right side rises) | 2, 3, 4 | right |
| hard forward | pitch +17.5 deg (nose down, tail rises) | 4–8 | rear |

The 1.8 m obstacles were never lost — only the 1.2 m walls — which is the
headroom mechanism and nothing else.

**Independent confirmation.** `tests/client_gimbal_probe.py` tilted *the mount
alone* by 17.2 deg while the airframe stayed level at 0.00 deg. Finite beams
fell 311 -> 194 and the 0.85 m wall vanished, leaving a minimum range of
2.218 m — the 2.123 m obstacle tangent divided by cos 17.2 deg, i.e. the slant
range to the one object still tall enough to be hit.

---

## Defect 2 — the levelled plane range-found the drone's own rotors

Stabilising the mount introduced a second, opposite failure. Holding the plane
level while the body rolls means the plane dips **relative to the body** on the
low side, by `tip_radius * tan(tilt)`. The rotor discs sit at z = 0.015 out to
a 0.1773 m tip radius, so the plane drops 0.1773 * tan(19 deg) = 0.061 m — just
past the 0.06 m the lidar had above them.

The signature was unmistakable: during the diagonal manoeuvre, sectors 7 and 8
(210 and 240 deg, bracketing the Webots rear-left rotor at DNS ~225 deg) read a
constant **0.177 and 0.178 m** — the rotor tip radius — against a ground truth
of 0.98 and 1.34 m.

**Fix.** Raise the mount to **0.14 m**, giving 0.125 m of clearance against the
0.0857 m worst-case dip — a 46% margin. `Scenario._check_lidar_clearances()`
now asserts this at load time so the geometry cannot silently regress, and the
same check rejects a body-fixed mount riding above its own walls.

---

## Defect 3 — grazing incidence at sector boundaries (inherent, not fixed)

Two sectors still disagreed with ground truth after the mount was levelled, and
critically **the same disagreement appeared in both mount modes**, so it was not
attitude. The cause is angular quantisation:

With the drone at [0.85, 0.901], obstacle `obs_a` ([3.5, −0.35], r = 0.55) sits
at relative bearing −25.3 deg with an angular half-width of
asin(0.55 / 2.93) = 10.8 deg, so it spans −36.1 to −14.4 deg. Sector 0 ends at
−15 deg: the obstacle overlaps it by **0.6 deg**. At the original 1 deg beam
pitch no beam landed inside that sliver, so sector 0 read clear to cap while
sector 1 correctly held the return at 2.929 m.

**This is a property of 30 deg binning, not a bug.** The surface is not lost,
only re-attributed to a neighbouring sector. Two consequences are documented
rather than engineered away:

1. Beam pitch was halved — **720 beams, 0.5 deg spacing, 60 beams per sector**
   (was 360/1.0 deg/30). This halved the baseline hover RMS from 0.0346 m to
   **0.0171 m**, confirming the residual was discretisation.
2. The acceptance test now scores a **3-sector sliding minimum** ("is the
   nearest surface in roughly this direction seen?"), which is still fully
   sensitive to attitude faults but immune to +-1 sector re-attribution. Strict
   per-sector counts are still reported alongside.

**Limitation for the sensor teammate and for PPO:** obstacle *bearing* carries
a +-1 sector ambiguity near boundaries, and a grazed sector reports the full
5 m cap rather than a partial range. Do not treat a single clear sector flanked
by a near return as verified free space.

---

## Defect 4 — `geometric` mode is optimistic by up to 0.28 m

`sensing.geometric_scan` samples only 3 sub-rays per sector (−10, 0, +10 deg),
mirroring `planar.py`. The real lidar uses 60 and finds the true minimum over
the full +-15 deg, so it legitimately reports **shorter** ranges:

| Sector | lidar (m) | geometric 3-ray (m) | diff |
|---|---|---|---|
| 1 | 2.679 | 2.960 | −0.281 |
| 8 | 2.273 | 2.489 | −0.216 |
| 10 | 1.912 | 2.167 | −0.255 |
| 3, 6, 9 | exact | exact | 0.000 |

Every difference is negative. This explains the 0.29 m spread in the earlier
`results/calibration.json` that looked like a mapping error: **the mapping was
correct and the reference was coarse.** Treat `geometric` as a debug reference
that slightly over-states clearance, never as a sensor model, and never compare
`lidar` and `geometric` episodes as if the clearance signal were identical.

---

## What the mount modes now cost

29 control steps per mode, tilts up to 19 deg, current geometry:

| Mount | Lost-surface steps | Invented-surface steps | Worst paired RMS | Median RMS |
|---|---|---|---|---|
| `body_fixed` | 0 | 3 (strict) | 0.196 m | 0.027 m |
| **`stabilised`** | **0** | **0** | **0.044 m** | **0.023 m** |

Both modes pass the window criterion now that the walls are 2.5 m, but the
body-fixed plane is still 4.5x worse in the tail because it measures *slant*
ranges and samples surfaces at a tilt-dependent height.

---

## Changes made, and their justification

| Change | From | To | Why |
|---|---|---|---|
| Lidar mount | body-fixed | **ideal gimbal**, levelled every 8 ms tick | Makes the flat 12-sector contract actually true; matches the 2-D world the policies trained in |
| Mount height | 0.075 m | **0.14 m** | Clear the rotor discs when the plane is levelled against max tilt |
| Wall height | 1.2 m | **2.5 m** | A 1.2 m parapet is not a credible boundary for a drone cruising at 1.0 m, and it left only 0.125 m of headroom |
| Beams | 360 (1.0 deg) | **720 (0.5 deg)** | Halves worst-case grazing error; halved measured baseline RMS |

**Flagged interface change for Spencer and the sensor teammate:** the beam
count moved from 360 to 720. The 12-sector output, its units, the 5 m cap and
the `(0.0, False)` / `(5.0, True)` encoding are all unchanged, so nothing
downstream of `pack_scan` is affected — but anyone consuming the *raw* range
image must read `lidar.getHorizontalResolution()` rather than assume 360. The
packet records it as `lidar_beams`.

### How the gimbal is implemented

`DnsQuadcopter.proto` wraps the `Lidar` in `DEF DNS_LIDAR_MOUNT Pose` whose
rotation is the exposed PROTO field `lidarMountRotation`. A `Pose` carries no
mass, inertia or collider, so switching mount modes leaves the calibrated
0.55 kg mass budget and the flight dynamics untouched.

`tests/client_gimbal_probe.py` established the write path empirically:

- `drone.getField("lidarMountRotation")` — **works**, updates in place, the
  device handle survives, and the readback matches.
- `drone.getFromProtoDef("DNS_LIDAR_MOUNT")` — **read-only**; Webots raises
  *"called on a read-only PROTO internal field"*.

`Autopilot.level_mount_rotation()` takes the body's **world orientation
matrix** from `Node.getOrientation()` rather than `InertialUnit` roll/pitch
angles, so the result commits to no Euler convention. This matters: `Rz Ry Rx`
and `Rz Rx Ry` differ by a roll-pitch commutator that **vanishes for pure-roll
and pure-pitch manoeuvres**, so a convention error would have passed every
single-axis test and only shown up on diagonal commands.

### Honest residuals

- **One tick of lag.** The mount is written from the orientation measured at the
  start of the step, so it trails the airframe by 8 ms (order `omega * dt`).
  Bounded by the 0.044 m worst-case RMS above.
- **Ideal gimbal.** Zero mass, zero actuator lag, no joint limits. A real
  stabilised mount has all three. This is a *simulation model choice*, labelled
  `lidar_mount: "stabilised"` in every record, not a claim about hardware.
- **Grazing ambiguity.** Defect 3 is reduced, not eliminated.
- **Single layer.** Still one 0.01 rad layer, so the sensor is blind to
  anything not intersecting its plane — obstacles are 1.8 m cylinders and the
  plane rides at 1.14 m, which `_check_lidar_clearances` does *not* yet verify
  per-obstacle.

### What this does not prove

A validated sensor model is not a validated policy. These tests show the
simulator reports what its own geometry contains; they say nothing about
whether a PPO policy navigates well, and nothing about a physical drone.
