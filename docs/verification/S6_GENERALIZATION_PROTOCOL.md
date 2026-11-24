# S6 — Spatial-generalization protocol (PRE-REGISTERED)

**Status: committed before any controller was run on L01–L05.**
Frozen scenario hashes: `07_SCENARIOS/HASHES.txt`.
Generator config hash: recorded in each scenario's `provenance.generator_config_sha256`.

This document fixes the matrix, the variants, the metrics, the termination
rules, the analysis plan and the reduction rule *in advance*. Nothing below was
chosen after seeing an outcome. If a bug forces a change, the change is logged
in `V1_CORRECTIONS.md`, everything affected is rerun, and the superseded results
are kept in `10_EVIDENCE/failed_runs/`.

---

## 1. Question

Spencer's three PPO checkpoints were fitted on one obstacle layout (L00). v1
measured them only on that layout, so it could not distinguish navigation
competence from layout-specific memorisation. This experiment asks: **how does
success change when the obstacle positions change, holding everything else
fixed?**

The environment is the instrument. A drop on new layouts is a measurement of the
policies' generalization limits, not a failure of the environment. Policy-level
conclusions belong to Spencer.

## 2. What is held fixed

Frozen by the Section 3 contracts and verified per layout by
`dns/worldcheck.py` and `scripts/contract_tests.py`:

| Quantity | Value |
|---|---|
| Arena bounds (DNS) | `[0.0, −3.0] → [10.0, 3.0]` m |
| Wall height | 2.5 m |
| Start | (0.85, 0.00) m |
| Goal | (9.15, 0.00) m |
| Goal tolerance | 0.4 m |
| Cruise altitude | 1.0 m |
| `basicTimeStep` | 8 ms |
| Observation interval | 0.200 s (25 timesteps) |
| Per-axis speed cap | 1.2 m/s |
| Horizon | 90 control steps |
| LiDAR | 720 beams, 360°, 5.0 m cap, 12 sectors, stabilised mount at 0.14 m |
| Collision rule | geometric (swept, exact box footprints) |
| Cart dynamics | linear, ±1.9 m along y, 0.45 m/s, period 16.8889 s |
| Drone collision radius | 0.18 m |

**Only obstacle placement varies.** No start jitter: the contract fixes the
start point, so jittering it to manufacture distinct Mode B variants was
rejected (see `DECISIONS.md`).

## 3. Layouts

Six layouts, generated and frozen before this run. L01–L05 are **TEST-ONLY** and
held out from any training.

| ID | Title | Dissimilarity to L00 (1−IoU) | En-route corridor width (m) | A* length (m) |
|---|---|---|---|---|
| L00 | Baseline (training layout) | 0.000 | 2.080 | 9.004 |
| L01 | Perturbed (±0.3 m, ±10°) | 0.411 | 1.868 | 8.963 |
| L02 | Mirrored (y → −y) | 0.926 | 2.080 | 9.004 |
| L03 | Aisle shift | 0.907 | 1.315 | 9.294 |
| L04 | Clutter | 0.865 | 0.850 | 9.087 |
| L05 | Hard (narrow gate + occluded goal) | 0.859 | 0.684 | 9.335 |

Full descriptors: `06_DATA/scenario_table.csv`.

**L02 is the memorization probe.** It is maximally dissimilar to L00 (0.926)
while being *identical* on every difficulty descriptor — same A* length
(9.004 m), same tortuosity (1.085), same en-route corridor width (2.080 m),
same straight-line intrusion (0.200 m) — because it is an exact reflection.
Any L00→L02 drop therefore cannot be a difficulty effect.

## 4. Modes

Defined once here and used with these meanings in every document.

- **Mode A** — interface/kinematic test mode. The drone is moved at exactly the
  commanded velocity (`dynamics.mode = kinematic`). Used only for contract
  testing; **no result in this experiment uses Mode A.**
- **Mode B (static)** — full flight dynamics; the cart is **parked** at the
  arena edge (y = −2.5 m) with amplitude 0. The layout is then entirely static.
- **Mode C (dynamic)** — full flight dynamics; the cart sweeps the aisle. Mode B
  and Mode C share an identical static obstacle set by construction, so B vs C
  isolates obstacle motion.

## 5. Matrix

5 controllers × 6 layouts × (Mode B: 3 episodes + Mode C: 15 episodes)
= **540 episodes**, of which 90 are the L00 regression/reproduction run.

Controllers — all Spencer's, run unmodified:

| Name | Kind | Source |
|---|---|---|
| `ppo_seed7` | trained checkpoint | `evidence/seed7/model.npz` via `ppo.PPO.load().predict` |
| `ppo_seed19` | trained checkpoint | `evidence/seed19/model.npz` |
| `ppo_seed43` | trained checkpoint | `evidence/seed43/model.npz` |
| `reactive` | untuned reference | `evaluate.gap_rule` (hand-written potential field) |
| `direct` | untuned reference | `evaluate.goal_direct` (fly at the goal) |

`direct` and `reactive` are **untuned references**, never used to rank methods.
`PPO.predict` is `tanh(actor(obs))` — deterministic, no action sampling, no RNG.

### Variants

- **Mode C — 15 distinct variants.** Variant *i* sets the cart's initial
  crossing phase to `numpy.random.default_rng(200_000_000 + i).uniform(0, 2π)`.
  This is v1's exact rule with v1's exact seeds, so the 15 phases are identical
  to v1's and L00 results are directly comparable. The 15 phases are distinct
  (verified: 15 unique values), so **n = 15 distinct configurations per cell.**
- **Mode B — 3 repeats of 1 configuration.** The episode seed is accepted by
  `WebotsBackend.reset(seed, scene)` but never used: there is no start jitter,
  no scene randomisation and no policy sampling. The three Mode B episodes per
  cell are therefore the *same* configuration run three times.
  **n = 1 distinct configuration per cell**; the three repeats are reported as
  run-to-run determinism evidence, not as sample size. This is the correction
  carried forward from Phase 0 item 4 (v1 reported these as 3/3).

## 6. Termination and outcome taxonomy

An episode ends when Spencer's `navigation.Navigation.step` returns
`terminated` or `truncated`. His rule, unmodified:

```
collision  if frame.collision
goal       elif |goal − xy| <= 0.4 m
timeout    elif steps >= 90
```

The reported outcome refines `collision` by attributing it to the nearest body
at the terminal pose (`dns/worldcheck.classify_contact`):

| Outcome | Meaning |
|---|---|
| `goal` | within 0.4 m of the goal |
| `collision_static` | swept collision, nearest body a static obstacle |
| `collision_cart` | swept collision, nearest body the moving cart |
| `collision_wall` | swept collision, nearest surface an arena wall |
| `timeout` | 90 control steps without goal or collision |

`out-of-bounds` is not a separate outcome: leaving the arena *is* a wall
collision under the geometric rule (`_geometric_collision` returns True as soon
as the drone's disc reaches `low + radius` / `high − radius`), so it is reported
as `collision_wall`. Stated here so the absence of an `out_of_bounds` column is
not read as a missing measurement.

## 7. Per-episode metrics

Logged for every episode into `06_DATA/results_v2.csv`:

| Column | Definition |
|---|---|
| `outcome` | as above |
| `time_to_goal_s` | `control_steps × 0.2`, blank unless `outcome == goal` |
| `path_length_m` | sum of per-step displacements (Spencer's `info["step_m"]`) |
| `path_efficiency` | A* reference length ÷ flown path length; ≤ 1 for an efficient route, blank unless `goal` |
| `min_clearance_m` | minimum supervisor ground-truth distance to the nearest obstacle **surface** over the episode (exact box footprints, moving cart included at its true pose) |
| `mean_clearance_m` | mean of the same quantity over control steps |
| `near_miss_count` | number of contiguous intervals with clearance < 0.30 m |
| `near_miss_duration_s` | total time with clearance < 0.30 m |
| `termination_step` | control step at which the episode ended |
| `min_sensed_clearance_m` | minimum over valid sectors of the LiDAR range (what the policy actually saw) |

Clearance is supervisor knowledge and never enters an observation.
Near-miss threshold 0.30 m is the task's default (the project has no value of
its own); it is > the 0.18 m collision radius so a near miss is not a collision.

## 8. Analysis plan

Fixed in advance:

1. **k/n per cell**, with n = 15 (Mode C) or n = 1 (Mode B) per the variant
   definitions above.
2. **PPO pooled over the three checkpoints** per layout and mode: Mode C pooled
   n = 45, Mode B pooled n = 3 cells of n = 1. Wilson 95% intervals on pooled
   proportions only.
3. **No significance tests. No ranking where intervals overlap.** Per-seed
   points are plotted but seeds are never ordered.
4. **Generalization gap** = pooled success on L00 − pooled success on L*, per
   mode, with both intervals shown.
5. **Memorization test** = L00 vs L02, pooled, per mode. Difficulty descriptors
   are identical by construction, so this isolates geometry.
6. **Exploratory only** (6 layouts is too few for a fitted trend): success vs
   dissimilarity and success vs en-route corridor width, plotted as points with
   no fitted line.
7. **Failure taxonomy**: outcome counts per layout per mode, with one video per
   failure type.

## 9. Reduction rule (pre-registered)

Measured cost: **1.74 s per episode** (v1: 156.69 s / 90 episodes) plus ~12 s
Webots boot per layout. Projected total for 540 episodes across 6 sessions:
**≈ 16 min of compute.** This is far inside the time available, so **no
reduction is applied** and the full matrix is run.

Had the projection exceeded the budget, the pre-registered reduction order was:
(1) drop `direct` and `reactive` to layouts L00, L02 and L05 only; (2) reduce
Mode C from 15 to 8 evenly spaced cart phases. Recorded here so that choosing
not to reduce is itself documented.

## 10. Blocking conditions

A layout is blocked from the experiment unless all of the following pass, and
the pass is recorded in `Verification_Report_v2.md`:

- every generator validator (wall margin, obstacle gap, keep-out, scan-plane
  height, cart lane clearance, straight-line blockage);
- A* feasibility for a disc of radius 0.18 + 0.02 m;
- same-seed byte-identical regeneration;
- `dns/worldcheck.check` — obstacle t = 0 poses, `basicTimeStep`, observation
  interval, LiDAR beams/FOV/cap, arena bounds, start, goal, tolerance;
- contract tests (`scripts/contract_tests.py`) for observation shape, dtype,
  range, two-frame structure, action bounds and sector indices;
- reference traversal: an A* waypoint follower reaches the goal in Mode B.

All six layouts passed these before this run. Evidence:
`06_DATA/scenario_table.csv`, `10_EVIDENCE/Verification_Report_v2.md`.
