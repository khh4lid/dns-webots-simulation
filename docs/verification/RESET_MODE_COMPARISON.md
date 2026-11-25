# Old versus new reset — what changes, and what does not

Two L00 runs of the **same** 90-episode protocol, same scenarios, same
checkpoints, same criteria, differing only in the reset procedure.

| | `settle_tolerance` (published) | `deterministic` (new) |
|---|---|---|
| Results | `results/v2_S6/` | `results/v2_S6_deterministic/` |
| Episode independence | **no** — drift 1.6–287 mm in isolation | **yes** — ≤ 0.21 mm across predecessors |
| Status | the published v1/v2 numbers; unchanged | a second, independently reproducible run |

Neither overwrites the other. `settle_tolerance` is still the default.

## Reset validation — the evidence for the new mode

`tests/client_reset_validation.py` runs each target three times inside one
session — first with nothing before it, then after predecessor A, then after
predecessor B — and compares all three. **5 targets × 2 controllers
(`ppo_seed7`, `ppo_seed19`) × 3 runs = 15 episodes, 15 pairwise comparisons**
per mode, spanning five cart phases and both outcome types.

| | `settle_tolerance` | `deterministic` |
|---|---|---|
| Outcomes agree across predecessors | 5/5 | **5/5** |
| Step counts agree | **4/5** (v01: 60, 59, 59) | **5/5** |
| Worst trajectory difference | **2.880e-01 m** | **2.139e-04 m** |
| Median trajectory difference | 3.5e-02 m | ~2e-06 m |

Compared component by component after reset: translation, rotation, linear and
angular velocity, autopilot integrator, the 12 packed sector ranges and their
validity flags, episode clock, cart phase. Source:
`results/reset_validation/validation_*.json`.

A cross-session check (isolated session vs 16-episode prefix, L00/`ppo_seed7`/v01)
gave identical initial state — altitude 2.5e-12 m, everything else exactly 0 —
and trajectories agreeing to 1.0e-06 m.

## What changed in the 90-episode re-run

**18 of 90 episodes differ in some way; 3 changed outcome.**

| Run | `settle_tolerance` | `deterministic` |
|---|---|---|
| `L00_C_ppo_seed7_v11` | goal (53 steps) | **collision_cart** (19) |
| `L00_C_ppo_seed19_v05` | collision_cart (27) | **goal** (47) |
| `L00_C_ppo_seed19_v11` | goal (42) | **collision_cart** (17) |
| `L00_C_reactive_v05` | collision_static (89) | timeout (90) — cell still 0/15 |
| 8 × `direct` | collision_static (10) | collision_static (12) — same outcome |
| 6 others | same outcome, step count differs by 1–16 | |

### Effect on the reported rates

| Cell | `settle_tolerance` | `deterministic` |
|---|---|---|
| ppo_seed7 Mode C | 9/15 | **8/15** |
| ppo_seed19 Mode C | 8/15 | 8/15 |
| ppo_seed43 Mode C | 7/15 | 7/15 |
| **PPO pooled Mode C** | **24/45 = 0.533** [0.391, 0.671] | **23/45 = 0.511** [0.370, 0.650] |
| All Mode B cells | 3/3, 3/3, 3/3, 0/3, 0/3 | identical |
| reactive, direct | 0/15, 0/15 | identical |

**Read this carefully.** Three individual episodes flipped, which is a real and
previously unobserved effect — the earlier study saw none and said so. But the
pooled rate moved by one episode, from 0.533 to 0.511, and each value sits
comfortably inside the other's Wilson interval. The qualitative conclusions for
L00 are unchanged: PPO succeeds on roughly half the cart phases, the untuned
references score zero, and Mode B is unaffected.

### What this does *not* establish

- **Only L00 was re-run.** L01–L05 have **not** been re-run under the
  deterministic reset, so the ladder comparison, the mirror result and the L05
  figure all still rest on `settle_tolerance` data. They are internally
  consistent — one protocol, one reset, one session order — but a
  deterministic-mode ladder is outstanding work, not a completed check.
- **The three flips are not evidence that either number is "the right one".**
  They are evidence that episodes terminating near a boundary are sensitive to
  sub-millimetre initial-state differences, which is the honest characterisation.
- Repetitions under the deterministic reset are **still not independent
  samples**: they are exact repeats of one configuration, so they count once.
  Independence here means *reproducibility*, not statistical independence.

---

## Hidden state — what actually survives a reset

Every component a reset is supposed to restore, compared across the three runs
of each target (15 comparisons per mode). These are read **after** reset, before
the first action.

| Component | `settle_tolerance` worst | `deterministic` worst |
|---|---|---|
| Position (translation) | 4.572e-02 m | **2.525e-12 m** |
| Linear velocity | 4.170e-02 m/s | **3.139e-10 m/s** |
| Angular velocity | 2.669e-01 rad/s | **1.134e-08 rad/s** |
| Measured velocity in the observation | 1.504e-02 m/s | **0.0 exactly** |
| **Autopilot integrator** | 4.559e-02 | **0.0 exactly** |
| **12 packed sector ranges** | 6.478e-02 m | **0.0 exactly** |
| Sector validity flags | identical | identical |
| Roll / pitch / yaw (IMU) | 9.6e-04 / 2.2e-03 / 1.1e-03 rad | **0.0 exactly** |
| Episode clock | 0.0 | 0.0 |
| **Cart y at t = 0** | 0.0 | 0.0 |
| **Cart phase** | 0.0 | 0.0 |

Three findings worth stating plainly:

1. **The old reset leaked controller and sensor state, not just pose.** The
   autopilot integrator started up to 0.046 different and the sector ranges the
   policy received started up to 0.065 m different. That is a leak into the
   *observation*, which is why trajectories diverged rather than merely
   wobbling.
2. **Obstacle timing was never affected, by either mode.** Cart phase, cart
   position at t = 0 and the episode clock are identical to 0.0 in both. The
   750-step settle does advance the global simulation clock, but
   `backend._epoch` is taken *after* settling and obstacle motion is a function
   of `episode_time`, so the cart always starts at its t = 0 pose. The longer
   settle does not shift the hazard.
3. **The `rotation` field is not a real difference.** Its worst value is
   1.89 rad in *both* modes — but in every one of those cases roll, pitch and
   yaw differ by exactly 0.0. A Webots `SFRotation` is `[axis, angle]`, and the
   drone flies at fixed yaw so the angle is ≈ 0, which leaves the axis
   degenerate and free to be written differently for an identical orientation.
   The IMU Euler angles are the physically meaningful read-out and they agree
   exactly. Reported here rather than silently dropped.

## Does the 750-step settle create unrealistic behaviour?

No behaviour is invented: the hover controller runs normally and the rotors
spin up through the same model. What the procedure does add is that the pose is
re-asserted after each of the 750 steps, so the airframe cannot drift during
the settle. The episode therefore begins from a hover at exactly the start pose
with the rotors already at hover speed — a cleaner initial condition than the
tolerance loop produced, not a less physical one. The settle costs 6.0 s of
simulated time, the same budget the tolerance loop was allowed, and it is
excluded from the episode clock.

---

# Full ladder under the deterministic reset (25 Nov 2026)

All six layouts re-run, same scenarios, same checkpoints, same rules. Cohorts
kept apart: `results/v2_S6/` (sequential) and `results/v2_S6_deterministic/`.

**540 episodes compared · 15 changed outcome (2.8 %) · 70 changed step count.**

| Layout | Mode B old → new | PPO Mode C old → new | Outcome changes |
|---|---|---|---|
| L00 | 3/3 → **3/3** | 24/45 → **23/45** | 4 |
| L01 | 3/3 → **3/3** | 20/45 → **22/45** | 3 |
| L02 | 3/3 → **3/3** | 24/45 → **23/45** | 4 |
| L03 | 2/3 → **2/3** | 20/45 → **20/45** | 4 |
| L04 | 0/3 → **0/3** | 0/45 → **0/45** | 0 |
| L05 | 0/3 → **0/3** | 0/45 → **0/45** | 0 |

## What survives

- **Mode B is identical on every layout.** No cell moved.
- **The mirror result survives.** L00 and L02 were 24/45 and 24/45 under the
  sequential reset; they are **23/45 and 23/45** under the deterministic one.
  Still exactly equal. The conclusion — *no evidence of left/right layout
  memorisation in this test* — therefore holds under **both** reset procedures,
  which is a stronger statement than either run alone supports.
- **The zero results are untouched.** L04 and L05 are 0/45 under both. L05
  remains a valid failure on a layout whose traversability was confirmed.
- **The ordering of the ladder is unchanged**, and every interval overlaps its
  counterpart.

## What moved

Three cells shifted by one or two episodes (L00 −1, L01 +2, L02 −1); L03, L04
and L05 did not move at all. Fifteen individual episodes flipped, in both
directions — seven toward goal, eight away. The flips concentrate in Mode C,
where an episode can terminate near the cart, and are absent from L04 and L05,
where every episode fails well short of any boundary.

**Attribution.** These differences are caused by the reset procedure alone.
The scenario JSONs, the policy checkpoints, the evaluation rules and Spencer's
code are byte-identical between the cohorts; only `WebotsBackend.reset_mode`
differs.

## Which cohort to quote

Quote **both**, as the slides now do. The sequential cohort is the published
v1/v2 record and is what the v1 reproduction (10/10 cells, 0 m) was measured
against. The deterministic cohort is the independently reproducible one. Where
they differ the difference is at most two episodes in 45 and well inside the
intervals, so no qualitative claim depends on the choice.
