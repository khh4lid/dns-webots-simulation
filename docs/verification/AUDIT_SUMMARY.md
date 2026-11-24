# AUDIT_SUMMARY.md — Phase 0 freeze and regression gate

One page. Every number here is reproduced by a script named beside it.

| # | Check | Result | Evidence |
|---|---|---|---|
| 1 | SHA-256 manifest of v1, its ZIP, the snapshot, Spencer's tree and the 3 checkpoints | **327 entries, 29.15 MB** recorded (v1 = 59 files, snapshot = 227 files, DroneNavPPO = 32 files) | `BASELINE_HASHES.txt`, `scripts/freeze.py --write` |
| 2 | Machine, Webots, Python, `pip freeze` | Windows 10 Pro 19045, Ryzen 5 3600, RTX 2060, **Webots R2025a**, Python 3.15.2 | `ENVIRONMENT.txt` |
| 3 | v1's 15 verification checks still pass | **Re-verified**, plus 11 new checks | `Verification_Report_v2.md` |
| 4 | What differs between episodes inside each v1 cell | **Mode B: n = 1 distinct configuration** (3 repeats); **Mode C: n = 15 distinct** | `06_DATA/v1_distinctness.json` |
| 5 | L00 90-episode rerun vs v1 `results.csv` | **10/10 cells identical, max path difference 0 m** | `06_DATA/v1_reproduction.csv`, figure E9 |
| 6 | Wall-clock per episode and Phase 2 projection | **1.74 s/episode** measured in v1 → ≈16 min for 540 episodes; **no reduction applied** | `S6_GENERALIZATION_PROTOCOL.md` §9 |
| 7 | This summary | — | this file |

## Item 4 in detail — the finding that changes how v1 is reported

`WebotsBackend.reset(seed, scene)` accepts the episode seed and **never uses
it**: there is no start jitter and no scene randomisation, and `PPO.predict` is
`tanh(actor(obs))` with no sampling and no RNG. Every configuration-defining
field is therefore identical across the three Mode B episodes in each cell:

| Mode B cell | episodes | distinct configs | path-length spread | steps |
|---|---|---|---|---|
| ppo_seed7 | 3 | **1** | 0.0010 m | 47 |
| ppo_seed19 | 3 | **1** | 0.0006 m | 37 |
| ppo_seed43 | 3 | **1** | 0.0001 m | 40 |
| reactive | 3 | **1** | 0.0065 m | 90 |
| direct | 3 | **1** | 0.0989 m | 10, 12 |

Mode C's 15 episodes per cell use 15 **distinct** cart phases (15 unique values
verified), so n = 15 stands there.

**Consequence:** v1's Mode B "3/3" is **1/1 per controller**. Pooled over the
three checkpoints, L00 Mode B is 3/3 configurations with Wilson 95%
**[0.439, 1.000]** — a much weaker claim than "3/3" implied. Applied throughout
v2; logged as correction **C3**.

## Item 5 in detail — the reproduction is exact

Every v1 cell re-ran on the regenerated L00 world and produced the same outcome
counts, and every matched episode's flown path length was identical to the
recorded precision:

| Mode | ppo_seed7 | ppo_seed19 | ppo_seed43 | reactive | direct |
|---|---|---|---|---|---|
| B | 3/3 → 3/3 | 3/3 → 3/3 | 3/3 → 3/3 | 0/3 → 0/3 | 0/3 → 0/3 |
| C | 9/15 → 9/15 | 8/15 → 8/15 | 7/15 → 7/15 | 0/15 → 0/15 | 0/15 → 0/15 |

This also explains the small within-cell spread in item 4: it is **not**
stochasticity. The simulator is deterministic given the same episode
*sequence*; the spread comes from residual airframe state carried across the
teleport-and-settle reset.

## Additional finding — a provenance defect in v1

`03_RESEARCH/results.csv` (SHA-256 `9c6af2aa…`) is byte-identical to the run
whose `protocol.json` records `wall_clock_s = 156.69` and
`world = DNS_Warehouse.wbt`. The `experiment_protocol.json` **shipped beside it**
records `wall_clock_s = 159.78` and `world = DNS_Research_World.wbt`. The two
files describe different executions. Outcome counts are unaffected (item 5),
but the shipped provenance record does not belong to the shipped data. Logged
as **C8**; fixed in v2 by per-run manifests, scenario hashes stamped into every
world, and a blocking `dns/worldcheck.check` before any result is produced.

## Verdict

Phase 0 **PASS**. v1 is frozen and hashed, the environment is recorded, the
L00 protocol reproduces exactly, the episode-variant semantics are now
documented and corrected, and the Phase 2 runtime fits the budget with no
reduction rule needed.
