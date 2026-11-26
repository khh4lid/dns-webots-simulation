# Validation and Evidence

> [!NOTE]
> **Historical document — v1 configuration.** The numbers below describe
> the configuration as it was when this was written (1.2 m walls). The current
> build flies at **1.0 m** with a **0.14 m** LiDAR mount, so the scan plane
> rides at **1.14 m**; see `protos/DnsQuadcopter.proto` (`lidarHeight`) and
> `config/scenarios/L00_C.json`. Kept unedited as the record of what was
> measured at the time.


Verification methodology, measured results, experiment provenance, reproduction
commands, and the limits of what any of it supports.

Every number on this page is traceable to a file in `results/`. Where a check is
exact by construction rather than independently measured, that is stated —
an exact zero is a claim about the implementation, not about physical realism.

---

## 1. What kind of claim is each result?

The distinction matters more than the numbers, so it comes first.

| Class | Meaning | Example here |
|---|---|---|
| **Measured** | Read back from the simulator after the fact | drone velocity, LiDAR ranges, flown paths |
| **Simulator ground truth** | Computed from supervisor state, not sensed | `min_truth_clearance_m`, analytic ray-cast |
| **Exact by construction** | Two sides of the check share a definition | cart position vs its own motion equation |
| **Derived** | Computed from measured values | success rates, Wilson intervals, RMSE |

Software verification — *does the simulator do what it says* — is separate from
research evaluation — *how does a controller behave in it*. Sections 3–5 are
verification. Section 6 is evaluation, and it carries much weaker claims.

---

## 2. Test inventory

| Suite | Count | Needs Webots | Needs Spencer's PPO package | Evidence |
|---|---|---|---|---|
| `tests/unit_tests.py` | 46 | no | no | stdout |
| `tests/client_validation.py` | 22 | yes | yes (import only) | `results/validation_physical_lidar.json` |
| `tests/client_moving_obstacle.py` | 13 | yes | yes (import only) | `results/moving_obstacle_warehouse.json` |
| `tests/client_experiments.py` | 90 episodes | yes | yes (checkpoints) | `results/experiments_warehouse/` |

The 46 unit tests are the only suite that runs with nothing but `numpy` — they
cover geometry, ray-casting, obstacle motion, scenario validation and the
observation contract as pure functions. Everything else needs a live simulator.

---

## 3. Simulator verification — 22 checks

Configuration: `config/scenario_warehouse.json`, `worlds/DNS_Warehouse.wbt`,
physical dynamics, LiDAR sensing, geometric collision, 8 ms physics step,
25 steps per 0.2 s command. Full record: `results/validation_physical_lidar.json`.

| # | What was tested | Why it matters | Measured result | Status |
|---|---|---|---|---|
| 1 | World loads with the configured timing | A wrong timestep silently changes the control rate | `basicTimeStep` 8 ms, 25 steps per command | PASS |
| 2 | Vehicle parameters | Thrust margin bounds what the autopilot can do | 0.55 kg, thrust/weight 4.00, collision radius 0.18 m | PASS |
| 3 | Start inside arena | Catches a config that spawns inside a wall | `[0.85, -0.0]` within `[0,-3]..[10,3]` | PASS |
| 4 | Start matches config | The world is generated, so it must agree | config `[0.85, 0.00]`, actual `[0.85, -0.0]` | PASS |
| 5 | Goal matches config | Same | `[9.15, 0.00]`, tolerance 0.4 m | PASS |
| 6 | Reset zeroes the clock | Episodes must start comparable | `time_s` 0.0 | PASS |
| 7 | Reset altitude | The policy assumes level cruise | 1.0011 m against a 1.0 m target | PASS |
| 8 | Command rate is 5 Hz | PPO was trained at `DT` = 0.2 s | deltas all 0.2 s, max error **2.78e-16 s** | PASS |
| 9 | Timestamps monotonic | Guards against a repeated or reversed frame | t = 0.0, 0.2, 0.4, 0.6, 0.8, 1.0 | PASS |
| 10 | +x forward | A sign error here invalidates every result | cmd `[0.6, 0]` → moved `[0.4657, 0.0]` m | PASS |
| 11 | −x back | " | cmd `[-0.6, 0]` → moved `[-0.4655, -0.0]` m | PASS |
| 12 | +y right | " | cmd `[0, 0.6]` → moved `[-0.0002, 0.4657]` m | PASS |
| 13 | −y left | " | cmd `[0, -0.6]` → moved `[0.0, -0.4655]` m | PASS |
| 14 | Velocity is measured, not echoed | A simulator that returns the command looks perfect and teaches nothing | commanded `[1.2, 0]` reads `[0.2801, -0.0008]` m/s after one step | PASS |
| 15 | No false collision at start | " | collision `False` at `[0.85, -0.0]` | PASS |
| 16 | Collision fires on wall contact | " | reported at `[0.255, 0.0]` after 5 steps | PASS |
| 17 | Reset restores the start pose | Episodes must be comparable | gap **2.758e-06 m** (tolerance 1e-03) | PASS |
| 18 | Reset is reproducible | Physical mode is not bit-exact; bound the drift | max divergence **7.091e-03 m** over 3 identical runs (tolerance 2e-02) | PASS |
| 19 | Observation is 52-dimensional | Spencer's contract | shape `(52,)` | PASS |
| 20 | Observation finite and bounded | A NaN would poison training | range `[-0.0001, 1.0000]` | PASS |
| 21 | Step through `navigation.Navigation` | Proves the wrapper works end to end | action `[0.5,-0.25]` → reward −0.017423, outcome `running` | PASS |
| 22 | `navigation.validate` accepts every frame | His verifier, not ours | accepted (it raises otherwise) | PASS |

Checks 10–14 are the reason to trust anything else. A simulator that teleports
the drone or echoes the command back would pass a naive smoke test and produce
entirely fictional research.

### On check 14

After one 0.2 s step a commanded 1.2 m/s reads 0.2801 m/s. That is the intended
result: a 0.55 kg quadcopter must tilt before it can accelerate sideways, so it
cannot reach a lateral setpoint instantly. A simulator reporting 1.2 m/s here
would be integrating the command, not flying.

---

## 4. Moving obstacle — 13 checks

Record: `results/moving_obstacle_warehouse.json`, 90 control steps over 18.0 s.
Figure: `assets/figures/moving_obstacle_validation.png`.

| What was tested | Measured result | Class | Status |
|---|---|---|---|
| Supervisor position matches the motion model | max deviation **0.000e+00 m** over 90 steps | exact by construction | PASS |
| Stays in the declared envelope | y ∈ [−1.8600, +1.8800] against declared ±1.90 m | measured | PASS |
| Stays inside the arena | x ∈ [5.10, 5.10], y within walls inset by r | measured | PASS |
| Other axis untouched | x held at 5.1 m | measured | PASS |
| Travel speed matches config | median **0.4500 m/s** vs 0.45 configured | measured | PASS |
| Crosses the route | centre crossed y = 0 twice in 18.0 s | measured | PASS |
| Replays exactly across resets | max divergence **2.554e-15 m** | measured | PASS |
| Obstacle migrates across sectors | swept 3 sectors `[0, 1, 11]`, bearing 0.4°–359.9° | measured | PASS |
| **Scan matches an analytic ray-cast** | worst per-sector 0.2900 m, **mean 0.0475 m** over 90 steps × 12 sectors, occlusion included | measured vs ground truth | PASS |
| Measured range tracks the cart | on the 17 of 90 steps where the cart is nearest along its own bearing | measured | PASS |
| Range follows the crossing | range in the cart's sector varied by 1.850 m | measured | PASS |
| Drone held altitude | 1.0000–1.0030 m against 1.0 m target | measured | PASS |
| Collision with the mover is detected | reported at `[4.705, 0.067]` after 17 steps | measured | PASS |

**The 0.000e+00 m result is not a physics claim.** The cart is *kinematically
driven* from the same closed-form path the checker evaluates, so exact agreement
is guaranteed by construction. It verifies that the implementation applies its
model without drift or step-skipping — nothing more. The independent facts in
this group are the measured median speed (0.4500 vs 0.45), the 2.554e-15 m
replay determinism, and above all the analytic-ray-cast comparison, which is
computed from a separate geometric routine that never reads the LiDAR.

---

## 5. Sensor configuration and calibration

Record: `results/calibration_warehouse.json`. Figure:
`assets/figures/lidar_validation.png`.

| Quantity | Value | How obtained |
|---|---|---|
| Beams | 720 over 360° (0.5° spacing, 60 per sector) | configured |
| Finite returns in the probe frame | 671 of 720 | measured |
| Angular offset | **180.5°**, direction +1 | **measured**, not assumed |
| Angular fit residual | RMS **0.0245 m** | measured |
| Worst per-sector difference vs ray-cast | 0.2362 m | measured |
| Mount height | 0.14 m above the body, scan plane at 1.14 m | configured |
| Wall headroom at cruise | 1.36 m below the 2.5 m wall top | derived |
| Maximum autopilot tilt | 25.78° | measured |

The beam count was raised from 360 to 720 because beam pitch bounds the
worst-case grazing error: a cylinder overlapping a sector boundary by less than
one beam pitch can be missed by that sector entirely and read as clear-to-cap.
The wall height was raised from 1.2 m to 2.5 m for the same class of reason — at
1.2 m only 0.125 m of wall stood above the scan plane, so any airframe tilt
beyond about 6° sent beams over the top and the sensor reported open space where
a wall stood. Both are recorded in `results/lidar_attitude*.json`.

Mount comparison (`stabilised` vs `body_fixed`) is written up in
[`SENSOR_ATTITUDE.md`](SENSOR_ATTITUDE.md). In the warehouse scene the
body-fixed mount produced 4 steps out of 29 with a too-short reading; the
stabilised mount is the configured default for that reason.

---

## 6. Research evaluation — 90 episodes

Record: `results/experiments_warehouse/` (`experiment_results.csv`, 90 rows;
`experiment_summary.csv`; `protocol.json`; `traces.json`).
Figures: `assets/figures/success_rate.png`, `assets/figures/trajectories.png`.

**Read section 7 before quoting any of this.**

Five controllers — three of them trained PPO checkpoints, two of them
hand-written baselines — were run in two modes that differ in exactly one
factor: whether the transfer cart sweeps the aisle or is parked against the wall.

| Mode | Controller | Kind | Episodes | Goal | Collision | Timeout | Success | 95% CI |
|---|---|---|---|---|---|---|---|---|
| B static | PPO seed 7 | `ppo_checkpoint` | 3 | 3 | 0 | 0 | 100% | 43.9–100% |
| B static | PPO seed 19 | `ppo_checkpoint` | 3 | 3 | 0 | 0 | 100% | 43.9–100% |
| B static | PPO seed 43 | `ppo_checkpoint` | 3 | 3 | 0 | 0 | 100% | 43.9–100% |
| B static | reactive rule | **scripted** | 3 | 0 | 0 | 3 | 0% | 0–56.2% |
| B static | goal-direct | **scripted** | 3 | 0 | 3 | 0 | 0% | 0–56.2% |
| C dynamic | PPO seed 7 | `ppo_checkpoint` | 15 | 9 | 6 | 0 | 60.0% | 35.8–80.2% |
| C dynamic | PPO seed 19 | `ppo_checkpoint` | 15 | 8 | 7 | 0 | 53.3% | 30.1–75.2% |
| C dynamic | PPO seed 43 | `ppo_checkpoint` | 15 | 7 | 8 | 0 | 46.7% | 24.8–69.9% |
| C dynamic | reactive rule | **scripted** | 15 | 0 | 5 | 10 | 0% | 0–20.4% |
| C dynamic | goal-direct | **scripted** | 15 | 0 | 15 | 0 | 0% | 0–20.4% |

Totals: 90 episodes, 45 per mode-group, no episode excluded. Every rate is
successes ÷ episodes in that cell.

**Only `ppo_seed7`, `ppo_seed19` and `ppo_seed43` are PPO.** `reactive` and
`direct` are short hand-written rules (`evaluate.gap_rule` and
`evaluate.go_to_goal` in Spencer's package) included as a sanity floor. Calling
all five "controllers" is correct; calling all five "PPO" is not.

### What this does and does not show

It shows the environment discriminates. Moving the cart drops the PPO
goal-reaching rate from 3/3 to roughly half, and the failures are collisions
rather than timeouts — the policies keep flying at the goal and meet the cart.
A scene where every controller succeeds measures nothing, and an earlier layout
did exactly that: with the racks clear of the centre line by 0.30 m the
goal-direct baseline threaded the gap and scored 3/3. The racks now overlap
y = 0 by 0.20 m each, which forces an S-shaped route.

It does **not** show that PPO beats the baselines as methods. The baselines were
not tuned for this scene, and one of them fails for a reason that is purely
geometric: the reactive rule's fixed 1.45 m repulsion radius exceeds the 1.54 m
crate-to-goal distance, so it stalls 1.61 m short of the goal and times out.
That is a statement about an untuned constant, not about reactive navigation.

The static cells hold three episodes each. A 3/3 result carries a 95% interval
of 43.9–100%, which separates nothing. Physical mode is near-deterministic in a
fixed layout, so repeats there measure run-to-run variation rather than scenario
variation; the dynamic cells vary the cart's initial crossing phase per seed and
are the only genuinely distinct scenario instances.

---

## 7. Limitations

1. **One fixed obstacle layout throughout.** Mode C varies *when* the cart
   arrives, not *where* obstacles are. Nothing here supports a claim about
   spatial generalisation, and `protocol.json` records that constraint
   explicitly. Varying layouts is future work, not completed work.
2. **Three checkpoints.** Their dynamic-mode spread (46.7–60%) is as wide as the
   effect being measured.
3. **Baselines are untuned.** See above — a sanity floor, not a comparison.
4. **Small static samples.** Three episodes per static cell.
5. **The LiDAR mount is an idealised gimbal** — massless, no lag, no limits.
   Real stabilisation has all three.
6. **One scan layer.** The sensor is blind to anything not crossing its plane;
   the scenario validator now rejects obstacles shorter than the scan height
   rather than letting them be silently invisible.
7. **Obstacle motion is kinematic**, driven through the Supervisor rather than
   by contact forces. The cart pushes nothing and cannot be pushed.
8. **Fixed altitude and fixed yaw.** The problem is solved in the horizontal
   plane; the autopilot holds 1.0 m and zero yaw.
9. **PPO training distribution.** The arena is kept at 10 × 6 m because the
   checkpoints were fitted on exactly that workspace. Enlarging the room is a
   legitimate out-of-distribution study and must be reported as one.
10. **No real-world validation.** No physical drone was flown. Simulation
    results do not transfer without a separate study.

---

## 8. Reproducing everything

Commands assume a shell at the repository root. Each was run to produce the
files cited above.

### 8.1 Environment

| | Verified configuration |
|---|---|
| OS | Windows 10 Pro (19045) |
| Simulator | Webots **R2025a** |
| Python | 3.15.2 |
| `numpy` | 2.3.5 |
| `matplotlib` | 3.11.2 |
| `Pillow` | 13.0.0 |
| `imageio-ffmpeg` | 0.8.1 |

```bash
pip install -r requirements.txt
```

Set `WEBOTS_HOME` if Webots is not auto-detected, and make Spencer's package
reachable (see [TEAM_INTEGRATION.md](TEAM_INTEGRATION.md)).

### 8.2 Without a simulator

```bash
python tests/unit_tests.py                      # expect 46/46
```

### 8.3 Regenerate the worlds

Worlds are written from the scenario JSON, so re-run this after any change to
`config/*.json` or `protos/DnsQuadcopter.proto`.

```bash
python -m dns.worldgen --config config/scenario_warehouse.json \
    --out worlds/DNS_Warehouse.wbt
python -m dns.worldgen                          # the bare arena
```

### 8.4 Simulator verification

```bash
python scripts/with_webots.py --timeout 420 --world worlds/DNS_Warehouse.wbt \
    tests/client_validation.py -- --config config/scenario_warehouse.json \
    --dynamics physical --sensing lidar        # expect 22/22, ~2 min
```

`--dynamics kinematic` and `--sensing none|geometric` exist for labelled
interface tests only. `kinematic` integrates the commanded velocity directly and
is **not a flight model**; `geometric` is a ground-truth ray-cast, **not a
sensor**. Never report either as a measurement.

### 8.5 The 90-episode experiment

```bash
python scripts/with_webots.py --timeout 3000 --world worlds/DNS_Warehouse.wbt \
    tests/client_experiments.py -- \
    --dynamic-config config/scenario_warehouse.json \
    --static-config  config/scenario_warehouse_static.json \
    --out results/experiments_warehouse        # ~2.6 min wall clock
```

Seeds are fixed in `protocol.json`; the crossing phase is
`numpy default_rng(seed).uniform(0, 2*pi)`, horizon 90 steps. Re-running this
reproduced `experiment_summary.csv` **byte for byte**.

### 8.6 Validation studies

```bash
python scripts/with_webots.py --world worlds/DNS_Warehouse.wbt \
    tests/client_moving_obstacle.py -- --config config/scenario_warehouse.json \
    --out results/moving_obstacle_warehouse.json

python scripts/with_webots.py --timeout 700 --world worlds/DNS_Warehouse.wbt \
    tests/client_lidar_attitude.py -- --config config/scenario_warehouse.json \
    --out results/lidar_attitude_warehouse.json

python scripts/with_webots.py --world worlds/DNS_Warehouse.wbt \
    tests/client_calibrate.py -- --config config/scenario_warehouse.json \
    --out results/calibration_warehouse.json
```

### 8.7 Screenshots, video and figures

```bash
python scripts/with_webots.py --boot-wait 12 --timeout 600 \
    --world worlds/DNS_Warehouse.wbt tests/client_screenshot.py \
    -- --config config/scenario_warehouse.json

python scripts/with_webots.py --timeout 520 --world worlds/DNS_Warehouse.wbt \
    tests/client_record_demo.py
python scripts/verify_mp4.py results/demo/DNS_Webots_Research_Demo.mp4 \
    --json results/demo/mp4_verification.json      # expect "verdict": "VALID"

python scripts/make_assets.py                      # all README figures
```

Two capture failures in this environment are **silent** — every API call
succeeds and a valid-looking file appears — so verification is mandatory:

1. **The Webots GUI viewport renders no geometry** when launched
   non-interactively here. `Supervisor.exportImage` returned three
   byte-identical 2.9 MB uniform fills at three different drone poses
   (`results/render_probe/render_probe.json`, `"distinct_images": 1`). Frames
   are therefore captured from an in-scene `Camera` node, which does render
   (`results/camera_probe/camera_probe.json`: 143 → 564 distinct colours).
2. **The bundled `ffmpeg.exe` does not run** (exit 53, no `ffprobe`), so
   encoding uses `imageio-ffmpeg`'s binary.

`scripts/verify_mp4.py` parses the ISO-BMFF container directly and reports frame
count and per-frame byte sizes; a blank capture collapses to one or two distinct
sizes. The delivered video measures 1255 frames, 1280 × 720, 50.2 s, 1028
distinct frame sizes. Never treat `movieIsReady()` / `movieFailed()` as evidence.

### 8.8 Interface handoff

```bash
python scripts/show_handoff.py results/two_frames_webots_physical_lidar.json
python "$DNS_PPO_DIR/handoff.py" --file results/two_frames_webots_physical_lidar.json
```

The second runs **Spencer's own** verifier against our packet, rather than
adjusting his code to fit ours.

---

## 9. Notes on provenance records

- `protocol.json` and several `results/*.json` embed the absolute paths in use
  when they were produced. That is intentional provenance; regenerating on
  another machine rewrites the paths and does not change the measurements.
- `results/moving_obstacle_warehouse.json` echoes `"radius_m": 0.5` for the
  cart. That field is **ignored for box obstacles** — `dns/scenario.py` collides
  boxes against their exact rectangular footprint and derives a bounding radius
  of `hypot(0.45, 0.275)` = 0.527 m. The echoed 0.5 is an unused dataclass
  default, not a measurement.
- Before 2026-11-24 the experiment `protocol.json` recorded a hard-coded
  `world: worlds/DNS_Research_World.wbt` regardless of the world actually
  loaded. `tests/client_experiments.py` now asks Webots via
  `robot.getWorldPath()`. The affected datasets were regenerated; the
  measurements were unchanged.
