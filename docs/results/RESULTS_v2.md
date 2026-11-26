# Results — v2 S6 generalisation experiment

**540 episodes = 480 distinct configurations.** 5 controllers x 6 layouts x (Mode B: 3 runs of 1 configuration + Mode C: 15 cart phases).

Protocol, committed before anything ran: [S6_GENERALIZATION_PROTOCOL.md](../verification/S6_GENERALIZATION_PROTOCOL.md).

## Pooled PPO success

| Layout | Mode B (n=1/controller) | Mode C (n=45) |
|---|---|---|
| **L00** | 3/3 = 1.000 [0.439, 1.000] | 24/45 = 0.533 [0.391, 0.671] |
| **L01** | 3/3 = 1.000 [0.439, 1.000] | 20/45 = 0.444 [0.309, 0.588] |
| **L02** | 3/3 = 1.000 [0.439, 1.000] | 24/45 = 0.533 [0.391, 0.671] |
| **L03** | 2/3 = 0.667 [0.208, 0.939] | 20/45 = 0.444 [0.309, 0.588] |
| **L04** | 0/3 = 0.000 [0.000, 0.561] | 0/45 = 0.000 [0.000, 0.079] *(excluded — see ADR-014)* |
| **L05** | 0/3 = 0.000 [0.000, 0.561] | 0/45 = 0.000 [0.000, 0.079] |

Per-checkpoint values are in `results/v2_S6/pooled_rates.csv`. They are **never ranked**: on L00 Mode C they are 9/15, 8/15 and 7/15, a spread about the size of the effect, with Wilson intervals that overlap almost completely.

> [!IMPORTANT]
> **The Wilson intervals are approximate.** A Wilson interval assumes
> independent trials. The 15 cart-phase variants inside a cell are run back
> to back and are coupled through residual airframe state across the reset
> (`docs/verification/RESET_DETERMINISM.md`), so they are not strictly
> independent draws. The deliberate cart-phase variation is far larger than
> the coupling, but the intervals should be read as indicative rather than
> exact.


## Failure taxonomy — all PPO episodes

| Layout | goal | collision static | collision cart | collision wall | timeout |
|---|---|---|---|---|---|
| **L00** | 33 | 0 | 21 | 0 | 0 |
| **L01** | 29 | 0 | 25 | 0 | 0 |
| **L02** | 33 | 0 | 21 | 0 | 0 |
| **L03** | 26 | 13 | 15 | 0 | 0 |
| **L04** | 0 | 54 | 0 | 0 | 0 |
| **L05** | 0 | 0 | 0 | 0 | 54 |

On L00–L02 the dominant failure is a **cart collision** — a timing failure, not a geometric one. On L03–L05 static collisions take over as the corridors narrow.

## Layout descriptors

| Layout | IoU vs L00 | mean displacement (m) | A* (m) | tortuosity | corridor (m) | occupied (%) | cart blocks |
|---|---|---|---|---|---|---|---|
| **L00** | 1.000 | 0.0 | 9.004 | 1.085 | 2.08 | 7.75 | 0.340 |
| **L01** | 0.589 | 0.2122 | 8.963 | 1.080 | 1.87 | 7.75 | 0.274 |
| **L02** | 0.074 | 3.0 | 9.004 | 1.085 | 2.08 | 7.75 | 0.348 |
| **L03** | 0.093 | 1.1455 | 9.294 | 1.120 | 1.31 | 7.75 | 0.423 |
| **L04** | 0.135 | 0.9146 | 9.087 | 1.095 | 0.85 | 10.20 | 0.290 |
| **L05** | 0.141 | 1.85 | 9.335 | 1.125 | 0.68 | 11.57 | 0.465 |

## The mirror test

L02 is L00 reflected about y = 0. IoU **0.074**, mean obstacle displacement **3.0 m** — yet identical A* length (9.004 m), tortuosity (1.085), corridor width (2.08 m) and intrusion depth (0.200 m). Difficulty is identical **by construction**, so an L00-vs-L02 difference cannot be a difficulty effect.

Pooled Mode C: L00 24/45 = 0.533 [0.391, 0.671] and L02 24/45 = 0.533 [0.391, 0.671] — the same.

> **The claim, narrowly.** No evidence of left/right layout memorisation in this test. It is **not** a claim that the policy generalises: on L05, whose traversability was confirmed by a flown reference traversal reaching the goal in 56 steps, the same checkpoints score 0/45.

Visual proof: `assets/figures/E15_mirror_overlay.png` (the reflected tracks do not retrace the originals) and `assets/figures/E16_mirror_phases.png` (phase by phase).

## Why L04 is excluded

All five controllers score 0/90 — but so does the environment's own reference tracker, which is *given* the correct route. At a 0.22 m planning margin and <= 0.48 m/s it is stable (peak tilt 0.105 rad) and collision-free but times out at the 90-step horizon; faster, it collides at the same crate the PPO checkpoints hit; at a 0.12 m margin the airframe tumbled to 1.5905 rad against 0.45 rad of authority. So 'the policies fail' cannot be separated from 'this layout may be outside the airframe's envelope within the horizon'. Data kept in full. [ADR-014](../decisions/ADR-014-l04-excluded-generalisation-conclusion.md).

## Verification alongside the results

| Check | Result |
|---|---|
| L00 reproduction vs v1 | 10/10 cells, max path difference 0 m |
| World <-> scenario t=0 pose | <= 4.0e-06 m, all layouts |
| Contract tests | 26/26, all layouts |
| LiDAR <-> cart (C2) | MAE 0.01935 m, n=650 |
| Airframe tilt > 0.50 rad | 3 of 540 episodes |

Raw data: `results/v2_S6/results_v2.csv` and the per-run folders in `results/v2_S6/runs/`.
