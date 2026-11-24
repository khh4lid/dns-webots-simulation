# PPO interface, v2

*The v1 interface document is unchanged; this is the v2 revision.*

**Owner of this document:** TBC (@khh4lid) ·
**You:** TBC (TBC)

> [!IMPORTANT]
> **Nothing of yours is in this repository.** Your `DroneNavPPO/` tree, your
> training code and your checkpoints are yours to publish. This repo records the
> *interface* and the SHA-256 fingerprints of the files it ran against, so we can
> both prove which version produced which numbers.

## What you can do today

Evaluate any checkpoint on any of the six layouts, in either mode, without
editing a line of your code. Scenario selection happens entirely on the
environment side — the world file *is* the scenario.

```powershell
# one layout, both modes, all five controllers (90 episodes, ~3 min)
python scripts/with_webots.py --boot-wait 12 --timeout 1500 `
    --world worlds/DNS_L03_C.wbt tests/client_s6_experiment.py -- --layout L03

# the whole ladder (540 episodes, ~19 min)
python scripts/run_s6.py
```

`tests/client_s6_experiment.py` imports `evaluate.gap_rule`,
`evaluate.goal_direct` and `ppo.PPO.load(...).predict` directly from your tree
and calls them unmodified. To add a checkpoint, drop it in
`evidence/seed<N>/model.npz` and add `<N>` to `load_controllers(seeds=...)`.

## The contract I hold to

| Item | Value | Verified by |
|---|---|---|
| Observation | 52 floats, `float32`, all in [−1, 1] | contract test, every layout |
| Two-frame structure | current scan then previous scan; identical on reset | contract test |
| Action | 2 floats in [−1, 1], scaled by `SPEED = 1.2` m/s per axis | contract test |
| Observation interval | exactly 0.200000 s of simulated time = 25 × 8 ms | measured, not configured |
| Horizon | 90 control steps | `navigation.HORIZON` |
| LiDAR | 720 beams, 360°, 5 m cap, 12 sectors, 0/3/6/9 = front/right/back/left | contract test |
| Arena / start / goal / tolerance | identical in all six layouts | asserted against L00 |

**26/26 contract tests pass on every layout.** Run them yourself:
`python scripts/run_gates.py`. A layout that fails is blocked from the
experiment.

`handoff/two_frames.json` still verifies: `python handoff.py --file <packet>`.

## TEST-ONLY — please read

**L01–L05 are held out.** They exist to measure generalisation, and they only
measure it while no policy has been trained on them.

> [!WARNING]
> If you train on any of L01–L05, every generalisation number in this repository
> becomes a training-set score. If you want to train on new layouts, tell me and
> I will generate a **fresh held-out family** with a different seed — that is one
> config edit and about five minutes of compute.

L00 is the training layout and is not flagged test-only.

## Your results

| Layout | What changes | IoU vs L00 | Corridor (m) | PPO Mode C pooled [Wilson 95%] | Reference traversal |
|---|---|---|---|---|---|
| **L00** | the training layout | 1.000 | 2.08 | 24/45 = 0.533 [0.391, 0.671] | goal, 50 steps |
| **L01** | positions ±0.3 m, yaw ±10° | 0.589 | 1.87 | 20/45 = 0.444 [0.309, 0.588] | goal, 50 steps |
| **L02** | reflected about y = 0 | 0.074 | 2.08 | 24/45 = 0.533 [0.391, 0.671] | goal, 51 steps |
| **L03** | racks moved along x, sides swapped | 0.093 | 1.31 | 20/45 = 0.444 [0.309, 0.588] | goal, 52 steps |
| **L04** | extra crate stacks | 0.135 | 0.85 | 0/45 = 0.000 [0.000, 0.079] | **not reached** — excluded |
| **L05** | narrow gate + occluded goal | 0.141 | 0.68 | 0/45 = 0.000 [0.000, 0.079] | goal, 56 steps |

Pooled over your three checkpoints, n = 45 distinct cart phases per layout.
Per-seed values are in `results/v2_S6/pooled_rates.csv`. I **do not rank the
seeds** — their Wilson intervals overlap almost completely. On L00 Mode C they
are 9/15, 8/15 and 7/15, which is a spread of about the size of the effect.

Per-episode records are in `results/v2_S6/runs/<run_id>/` — `trajectory.csv`
(pose, velocity, command, clearance, per-step), `events.csv`, `obstacles.csv`,
and `lidar.npz` for variant 00 of each cell.

## Metric and termination definitions

Termination is **your** rule, unmodified: collision → goal (≤ 0.4 m) → timeout
(≥ 90 steps). I only refine the `collision` label by attributing it to the
nearest body at the terminal pose, giving `collision_static`, `collision_cart`
and `collision_wall`. Leaving the arena is a wall collision under the geometric
rule, so there is no separate out-of-bounds outcome.

- `path_efficiency` = A* reference length ÷ flown length, blank unless the goal
  was reached.
- `min_clearance_m` is supervisor ground truth to the nearest obstacle
  **surface**, exact box footprints, cart included at its true pose. It never
  enters an observation.
- A near miss is clearance < 0.30 m, counted as contiguous intervals.

## Two things not to assume

1. **Cart collisions are geometric, not physical.** The cart is driven
   kinematically, so Webots reports no rigid-body velocity for it and it cannot
   transfer momentum. `collision_cart` means the swept footprints intersected.
2. **Mode B's three episodes are one configuration.** The seed you pass to
   `reset(seed, scene)` is accepted and never used — no start jitter, no scene
   randomisation. With `PPO.predict` deterministic, the three episodes are
   repeats. I count n = 1 for Mode B rates (see
   [ADR-011](../decisions/ADR-011-distinct-configurations-wilson-intervals-no-seed.md)).

## What I'd like from you

1. **More training seeds.** Three cannot separate checkpoint variance from
   layout effects; the intervals are wide because n is small.
2. **A view on L04** — all five controllers score 0/90 and my reference tracker
   could not fly it either. See
   [ADR-014](../decisions/ADR-014-l04-excluded-generalisation-conclusion.md).
3. Confirmation that `ppo.py` is a from-scratch NumPy PPO and **not**
   Stable-Baselines3 — I read it that way and so do not cite SB3 anywhere.
