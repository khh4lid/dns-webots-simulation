# V1_CORRECTIONS.md — what v1 stated, what is correct, and the evidence

v1 (`DNS_SIMULATION_RESEARCH_DELIVERABLE/`) is **frozen and unmodified**; its
SHA-256 manifest is `BASELINE_HASHES.txt`. This file records every correction
applied in v2 material. Each entry gives the v1 statement, the corrected
statement, and the file that proves it.

Numbering follows the task's Phase 3 list; C8 and C9 were found during the work.

---

## C1 — The `reactive` explanation was numerically wrong, and the mechanism was incomplete

**v1 stated:** "its repulsion radius (1.45 m) is larger than the crate-to-goal
distance (1.54 m)".

**Why that is wrong:** 1.45 is *not* larger than 1.54, so the sentence
contradicts its own conclusion. The 1.54 m figure is the crate stack's
**centre**-to-goal distance, while `evaluate.gap_rule` reacts to the nearest
**LiDAR return** per sector — that is, to obstacle *surfaces* and to the arena
walls. Spencer's code, unmodified:

```python
ranges = obs[4:16] * RANGE          # per-sector nearest return, metres
mask   = obs[16:28] > 0
force  = (np.maximum(1.45 - ranges, 0) * mask) @ DIRECTIONS
v      = goal_direct(obs) - 0.85 * force
```

**Corrected, measured values** (`06_DATA/reactive_gnron_analysis.json`):

| Quantity | Value |
|---|---|
| crate_stack **centre** → goal (L00) | **1.5403 m** |
| crate_stack **surface** → goal (L00) | **1.0582 m** |
| `gap_rule` repulsion radius | 1.45 m |
| Goal inside the repulsion zone? | **Yes** — 1.0582 m < 1.45 m |

So v1's *conclusion* (the goal lies inside the repulsion zone) is correct, but
its *arithmetic* was not: the comparison must be made against the surface
distance, 1.0582 m, not the centre distance, 1.5403 m. This is the
goals-non-reachable-with-obstacles-nearby condition of Ge & Cui
(IEEE T-RA 16(5), 2000, doi:10.1109/70.880813).

**A second correction — the mechanism.** The GNRON condition at the goal is
*not* what stops `reactive` on L00. Recomputing `gap_rule` from the logged
sector ranges of the real episode shows the controller never reaches the goal
neighbourhood at all:

| Measured on `L00_B_reactive_v00` | Value |
|---|---|
| First position where the command collapses | (5.13, 0.08) m — mid-aisle |
| Peak repulsive term magnitude | **1.8654** (attractive term is a unit vector) |
| Final distance to goal | **1.6167 m** |
| Outcome | timeout at the 90-step horizon |

The controller is slowed over the whole route by repulsion from the two racks
and the walls — the repulsive term exceeds the unit attractive term — and runs
out of horizon 1.62 m short. Evidence that the goal-zone condition alone is not
decisive: on **L01** the goal is still inside the repulsion zone (crate surface
1.1116 m < 1.45 m) and `reactive` nevertheless **reaches it** (3/3 in Mode B,
final distance 0.2283 m), because reaching the goal only requires entering the
0.4 m tolerance, not converging to the point.

**Corrected statement for v2:** *`reactive` fails on L00 because the summed
repulsion from the racks and walls (peak magnitude 1.87 against a unit
attractive term) slows it enough that it times out 1.62 m short of the goal.
The goal additionally lies inside its 1.45 m repulsion zone — the crate stack's
surface is 1.058 m away, not the 1.54 m centre distance v1 quoted — which is a
GNRON configuration, but on this layout the controller never gets close enough
for that to be the binding failure.*

**Evidence:** `06_DATA/reactive_gnron_analysis.json`,
`scripts/analyse_reactive.py`, figure E13.

---

## C2 — "Cart vs model 0.00e+00 m" is true by construction

**v1 stated** this as headline validation of the moving obstacle.

**Correct:** the cart is **kinematic** — its pose is written by the supervisor
from `Obstacle.centre_at(t)`, and the check compared that written pose against
the same function. The result is therefore exactly zero *by construction* and
carries no physical information.

**Reclassified** as an **implementation-consistency check**: it confirms the
supervisor writes the pose it intends to and that reset restores it, nothing
more. It is never used as headline evidence.

**Replacement dynamic evidence** (`10_EVIDENCE/Verification_Report_v2.md`):

1. **LiDAR–cart consistency** — measured sector range against an independent
   analytic ray caster applied to the logged cart pose, over full crossing
   cycles.
2. **Swept-collision behaviour against the moving cart** — in Mode C the
   `collision_cart` outcome is produced by the swept geometric test; it is the
   dominant PPO failure mode on L00 and is absent from Mode B.
3. **Webots physics velocity of the cart** — reported in
   `06_DATA/cart_physics_check.json`. The cart is repositioned by
   `Field.setSFVec3f`, so the physics engine reports no rigid-body velocity for
   it. **Physical contact response with the cart is therefore not modelled**:
   a cart-drone collision is decided by the geometric swept test, not by
   Webots contact dynamics. Stated as a limitation, not hidden.

---

## C3 — v1's Mode B counts have n = 1, not n = 3

**v1 reported:** `ppo_seed7 3/3`, `ppo_seed19 3/3`, `ppo_seed43 3/3`,
`reactive 0/3`, `direct 0/3` in Mode B.

**Measured** (`06_DATA/v1_distinctness.json`): the episode seed is accepted by
`WebotsBackend.reset(seed, scene)` and **never used** — there is no start
jitter, no scene randomisation, and `PPO.predict` is `tanh(actor(obs))` with no
sampling. Every configuration-defining field is identical across the three
episodes in each Mode B cell:

| Mode B cell | episodes | distinct configurations | path-length spread |
|---|---|---|---|
| ppo_seed7 | 3 | **1** | 0.0010 m |
| ppo_seed19 | 3 | **1** | 0.0006 m |
| ppo_seed43 | 3 | **1** | 0.0001 m |
| reactive | 3 | **1** | 0.0065 m |
| direct | 3 | **1** | 0.0989 m |

**Corrected reporting:** Mode B rates are **k/1 per controller**, and the three
episodes are reported as run-to-run determinism evidence. Pooled over the three
checkpoints, Mode B on L00 is **3/3 configurations, Wilson 95% [0.439, 1.000]** —
a far weaker claim than "3/3" suggested.

Mode C is unaffected: its 15 episodes per cell use 15 **distinct** cart phases
(verified: 15 unique values), so n = 15 is correct there.

A nuance worth recording: the small within-cell spread above is not randomness.
The v2 rerun reproduced every v1 episode's path length exactly (C4), so the
simulator is deterministic given the same episode *sequence*; the spread comes
from residual airframe state carried across the teleport-and-settle reset, not
from stochasticity.

---

## C4 — Seed spread must be pooled, never ranked

**v1 reported** per-seed Mode C results 9/15, 8/15, 7/15 and discussed the
spread as if it ordered the checkpoints.

**Correct:** those three rates have Wilson 95% intervals of roughly
[0.36, 0.80], [0.30, 0.75] and [0.25, 0.70] — they overlap almost completely,
so the checkpoints cannot be ordered. **Pooled: 24/45 = 0.533, Wilson 95%
[0.391, 0.671]** (`06_DATA/pooled_rates.csv`).

**Applied in v2:** every reported rate is pooled over the three checkpoints with
a Wilson interval; per-seed values appear as plotted points only (figure F2) and
are never ranked. No significance test is reported anywhere.

---

## C5 — The LiDAR error metrics were under-specified

**v1 stated:** "LiDAR angular map RMS 0.025 m" and "full scan vs independent
analytic ray caster mean error 0.048 m" without saying over what.

**Corrected definitions** (`10_EVIDENCE/Verification_Report_v2.md`,
`06_DATA/lidar_verification.csv`):

- Both are **per-beam** errors, **absolute** (unsigned), in metres.
- **0.025 m** is the RMS residual of the *angular calibration fit* — the search
  over sweep direction and angular origin — computed only over beams that
  returned a finite range **and** whose analytic expectation is inside the 5 m
  cap. It measures how well the index→bearing map is recovered.
- **0.048 m** is the mean absolute error of a *full* 720-beam scan against the
  analytic ray caster, over **all** beams including those whose line of sight is
  occluded by an obstacle edge. It is larger because grazing beams near an edge
  are where a half-beam-pitch angular error turns into a large range error.
- v2 additionally reports **MAE, RMSE, p95 and max per sector** so the tail is
  visible rather than averaged away.

The two numbers answer different questions and must not be compared directly.

---

## C6 — Modes A/B/C were never defined in one place

**Corrected:** defined once in `S6_GENERALIZATION_PROTOCOL.md` §4 and repeated
in every document:

- **Mode A** — kinematic interface-test mode; the drone is moved at exactly the
  commanded velocity. Used for contract testing only; **no reported result uses
  it**.
- **Mode B (static)** — full flight dynamics, cart parked at y = −2.5 m with
  zero amplitude.
- **Mode C (dynamic)** — full flight dynamics, cart sweeping ±1.9 m at 0.45 m/s.

---

## C7 — References

v1 cited 5 works. v2 cites a verified set (see `01_REPORT/references.md`); every
entry was checked against a DOI, arXiv ID or publisher page before being cited.

**One correction of substance:** the task brief suggested citing
Stable-Baselines3 "if the checkpoints are SB3, as `PPO.load` suggests". **They
are not.** `DroneNavPPO/ppo.py` is a from-scratch NumPy PPO-Clip implementation
(`class PPO`, `MLP` with two tanh layers, a hand-written Adam, and
`np.savez_compressed` checkpoints). SB3 is therefore **not cited**; Schulman et
al. is cited for the algorithm only.

---

## C8 — v1 shipped a protocol file describing a different run than its results

**Found during Phase 0.** In the frozen v1 package:

- `03_RESEARCH/results.csv` (SHA-256 `9c6af2aa…`) is byte-identical to
  `webots_dns/results/experiments_warehouse/experiment_results.csv`, whose
  sibling `protocol.json` records `wall_clock_s = 156.69` and
  `world = worlds/DNS_Warehouse.wbt`.
- `03_RESEARCH/experiment_protocol.json`, shipped alongside it, records
  `wall_clock_s = 159.78` and `world = worlds/DNS_Research_World.wbt`.

The two files therefore describe **different executions**. The outcome counts
are unaffected — the v2 rerun reproduces the shipped `results.csv` exactly
(C9) — but the shipped provenance record does not belong to the shipped data.

**Fixed in v2:** every run writes its own manifest next to its own data
(`06_DATA/runs/<run_id>/manifest.json`), each world carries the SHA-256 of the
scenario that generated it, and `dns/worldcheck.check` refuses to produce
results against a world whose node poses disagree with the scenario.

---

## C9 — The L00 reproduction (new evidence, not a correction)

Running v2's harness on the regenerated L00 world reproduces v1's
`results.csv` **exactly in all ten cells**, with a maximum per-episode path
length difference of **0 m**:

| Mode | ppo_seed7 | ppo_seed19 | ppo_seed43 | reactive | direct |
|---|---|---|---|---|---|
| B (static) | 3/3 → 3/3 | 3/3 → 3/3 | 3/3 → 3/3 | 0/3 → 0/3 | 0/3 → 0/3 |
| C (dynamic) | 9/15 → 9/15 | 8/15 → 8/15 | 7/15 → 7/15 | 0/15 → 0/15 | 0/15 → 0/15 |

**Evidence:** `06_DATA/v1_reproduction.csv`, figure E9.

---

## Not corrected — carried forward unchanged

These v1 numbers were re-verified and stand: observation interval 0.200000 s;
reset error 2.8 µm; the four v1 defects found and fixed (circle-vs-box
collision, camera tilt axis, cart t = 0 world pose, fixed-scenario calibration
tool), each of which is now a regression test in `tests/test_generator.py` or
`dns/worldcheck.py`.
