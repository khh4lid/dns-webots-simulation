# For the sensor and perception teammate

**Owner of this document:** Khaled (@khh4lid) ·
**You:** name to confirm (handle to confirm)

> [!NOTE]
> I own sensor **mounting and configuration** inside the simulator. You own raw
> sensor **processing**. Nothing here touches your algorithm, and none of your
> code is in this repository.

## The sensor as configured

| Property | Value | Where it comes from |
|---|---|---|
| Device | Webots `Lidar`, name `"lidar"` | `protos/DnsQuadcopter.proto` |
| Beams | **720** over 360° (0.5° spacing) | read at run time via `getHorizontalResolution()` |
| Range cap | 5.0 m (`maxRange`); `minRange` 0.05 m | PROTO, asserted against the scenario |
| Vertical FOV | 0.01 rad — a single razor-thin layer | PROTO |
| Noise | 0.0 (ideal) | PROTO field `lidarNoise` |
| Mount height | 0.14 m above the body origin | PROTO, asserted |
| Scan plane | **1.14 m** above the floor (1.0 m cruise + 0.14 m mount) | derived |
| Stabilisation | ideal gimbal; the plane is held level every control tick | `Autopilot.stabilise_lidar` |
| Sectors | 12 × 30° | `navigation.SECTORS` |

**Never hard-code 720.** Read it with `getHorizontalResolution()`; the beam
count is a PROTO field and the whole pipeline follows it.

## Index → bearing

```python
bearing_dns(i) = direction * (2 * pi * i / resolution) + offset
#   direction = +1.0,  offset = +pi  (i.e. +180.000 deg)
```

Both the sweep direction and the angular origin were **measured, not assumed**:
`fit_lidar_mapping` searches the two unknowns against an analytic ray cast from
a known pose in known geometry and reports the residual. Webots documents the
range image as "scan lines left to right", which fixes the ordering but not the
sign or the origin once the device sits on a robot with its own axes.

Sector *i* is centred at `i * 30°` measured from +x rotating towards +y, so
**0 = front, 3 = right, 6 = back, 9 = left**. Binning is Spencer's
`navigation.pack_scan`, unmodified. The convention is verified per layout by
placing a single synthetic beam on each cardinal bearing and asserting the
sector it lands in.

> [!TIP]
> `+y` is **right** in the DNS frame and `-y` in Webots world coordinates. The
> sign lives in exactly one place, `dns/scenario.py`; everything else calls
> `to_dns`, `to_webots` and `bearing_to_webots`.

## Cap and `inf` handling

Webots returns `inf` beyond `maxRange`. `pack_scan` turns a *valid* `inf` into
"observed clear to 5.0 m" — the `(5.0, True)` encoding. A sector with no beam at
all is `(0.0, False)`, meaning *unknown*, which is different from *clear*.
Never treat 0.0 as a near obstacle.

## How accurate is it?

Measured against an independent analytic ray caster, sampled at the **same 720
bearings** and binned with the same `pack_scan`:

| Comparison | n | MAE | RMSE | p95 | max |
|---|---|---|---|---|---|
| Cart sector, pooled over 18 runs | 650 | **0.01935 m** | 0.08344 m | 0.04578 m | 1.48734 m |

Per-run and per-layout breakdowns: `results/v2_S6/lidar_cart_consistency.csv`.

A methodological note you will care about: comparing the sensor against a
reference that samples only **three** sub-rays per sector (the model in
`planar.py`) inflates the error to ~0.2–0.5 m, because the 60-beam sensor sees
obstacle edges the 3-ray reference misses. Sampling the reference at the full
720 bearings drops it to the table above. If you build your own reference,
match the sampling density.

## Ground-truth data you can use offline

`results/v2_S6/runs/<run_id>/lidar.npz`, for variant 00 of every
(layout, mode, controller) cell — 60 runs:

```python
import numpy as np
d = np.load("results/v2_S6/runs/L00_C_ppo_seed7_v00/lidar.npz")
d["t_s"]                   # (T,)     simulated time
d["pose_xy"]               # (T, 2)   drone position, DNS frame
d["sector_ranges_m"]       # (T, 12)  the packed sectors the policy received
d["sector_valid"]          # (T, 12)  bool
d["obstacle_centres_xy"]   # (T, N, 2) supervisor ground truth, cart included
```

Regenerate the analytic reference for any frame with
`scripts/analyse_lidar_cart.py::reference_scan`.

## One open anomaly — please look at this

In v1's L00 Mode B run, `min_sensed_clearance_m` was **0.0536 m** while the
supervisor ground-truth clearance at the same time was **0.62 m**. A sector
reporting 5 cm with nothing within 60 cm is unexplained. It matters because
`gap_rule`'s repulsion scales with `1.45 − range`, so a spurious short return
produces a very large repulsive force.

Reproduce it:

```powershell
python scripts/with_webots.py --boot-wait 12 --timeout 400 `
    --world worlds/DNS_L00_C.wbt tests/client_capture.py -- `
    --layout L00 --mode B --controller reactive --variant 0
```

Candidate explanations I have not ruled out: a beam striking the airframe
(rotor tip radius 0.1773 m, `minRange` 0.05 m); a grazing return at an obstacle
edge; or a sector-binning edge case. It is logged as an open issue rather than
explained away.

## Plugging in your processing

Your code does not have to live here and nothing in this repo needs to change.
Take `sector_ranges_m` (or the raw range image, by reading the device directly)
as input and return 12 sector ranges plus a validity mask in the same encoding.
The swap point is `dns/sensing.lidar_scan`. If you want a hook added, open an
issue and I will add one rather than you patching the environment.
