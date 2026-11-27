# Reset determinism — is a single episode reproducible on its own?

> [!IMPORTANT]
> **Status, 27 Nov 2026.**
> - **Default.** The default reset is still `settle_tolerance`, and every published number (`results/v2_S6/`, the README, the final deck) uses it.
> - **Experimental mode.** The `deterministic` mode described at the end of this page is in `dns/backend.py` on `main` as an opt-in setting. Branch `reset-fix-v2.1` points at a commit already contained in `main`.
> - **Gate result.** Its multi-target validation (5 targets × 2 controllers) meets the strict order-independence gate on 4 of 5 targets. The worst path drift is 2.139e-04 m against a 1e-5 m bar, so it is **not** the default and is not approved for published results.
> - **Second cohort.** Its 540-episode re-run, `results/v2_S6_deterministic/`, is a separately reported cohort; see [RESET_MODE_COMPARISON](RESET_MODE_COMPARISON.md).

**Short answer: no.** The 90-episode evaluation is reproducible as a *sequence*,
and that is what the v1 reproduction demonstrated. An individual episode re-run
in a fresh session is **not** bit-reproducible: it drifts from the logged run by
millimetres to tens of centimetres, and in one of four cases it changed the step
count.

This matters enough to state plainly, because it bounds what "reproducible"
means for this environment and it weakens the independence of the episodes
inside a cell.

## Why the question came up

Two facts sat uneasily together:

- the v1 protocol re-run reproduced **all ten cells with a maximum path
  difference of 0 m** — but it replayed the whole 90-episode session in order;
- while capturing footage, replaying **one** episode in a fresh session drifted
  **0.287 m** from the log, and only replaying the session prefix first brought
  it back to ~5e-06 m.

Both are consistent with the simulator being deterministic given the episode
*sequence* but not given an episode in isolation. That is testable.

## Method

`tests/client_reset_determinism.py`, driven by `scripts/run_reset_study.py`.
For each target variant, two Webots sessions:

- **isolated** — the target is the *first* episode after the world loads;
- **sequence** — the real session prefix (the whole Mode B block, then Mode C up
  to the target) is replayed first, then the target.

Both log the post-settle initial state and the full trajectory, and both are
compared against the logged run in `results/v2_S6/runs/`.
Layout L00, controller `ppo_seed7`, Mode C.

## Result

| Variant | Arm | Prefix | Outcome | Steps vs log | Max path difference |
|---|---|---|---|---|---|
| v00 | isolated | 0 | collision ✓ | 0 | **5.449e-03 m** |
| v00 | sequence | 15 | collision ✓ | 0 | 4.949e-06 m |
| v01 | isolated | 0 | goal ✓ | **−1** | **2.874e-01 m** |
| v01 | sequence | 16 | goal ✓ | 0 | 4.993e-06 m |
| v02 | isolated | 0 | goal ✓ | 0 | **2.271e-03 m** |
| v02 | sequence | 17 | goal ✓ | 0 | 4.845e-06 m |
| v05 | isolated | 0 | goal ✓ | 0 | **3.129e-02 m** |
| v05 | sequence | 20 | goal ✓ | 0 | 4.974e-06 m |
| v10 | isolated | 0 | collision ✓ | 0 | **1.583e-03 m** |
| v10 | sequence | 25 | collision ✓ | 0 | 4.974e-06 m |

Source: `results/reset_study/reset_study.csv` and `SUMMARY.json`. n = 5
variants, both arms, 10 Webots sessions.

**Isolated:** drift of 1.6 mm to 287 mm; outcome matched the log **5/5**, step
count matched **4/5**.
**With the sequence replayed:** every case agrees to ~5e-06 m, outcome **5/5**
and step count **5/5** — between two and five orders of magnitude tighter than the isolated runs.

## Mechanism

`WebotsBackend.reset` teleports the drone, calls `resetPhysics`, flushes three
steps and then **settles to a tolerance, not to a fixed point**:

```
speed < settle_speed_tol_mps (0.02)   and   |altitude − 1.0| < 0.02
```

Any state inside that box is accepted. Which state you land on depends on the
attitude and rates the previous episode left behind, so the post-settle
condition is a function of history. The tolerance is 20 mm/s and 20 mm; the
observed trajectory divergence is the same order, amplified over 60 control
steps by a closed-loop policy reacting to its own slightly different position.

## What this does and does not affect

**Not affected — the headline reproduction.** The v1 comparison re-ran the full
session in the same order, so its 10/10 cells at 0 m stands exactly as reported.
The same is true of the 540-episode experiment: re-running
`python scripts/run_s6.py` replays each layout's session in order.

**Affected — episode independence.** The 15 cart-phase variants inside a cell
are run back to back, so variant *i* starts from the state variant *i−1* left.
They are therefore **not fully independent draws**. The deliberate variation
(the cart phase, which shifts the hazard by up to 1.9 m) is larger than the
initial-state differences the coupling produces (up to 0.046 m in position;
[RESET_MODE_COMPARISON](RESET_MODE_COMPARISON.md)), but the coupling is not
zero, its effect on a trajectory can reach 0.287 m, and it should not be
described as zero.

**Affected — single-episode replay.** Any claim of the form "re-run this one
episode and you will get this trajectory" is false unless the session prefix is
replayed. `tests/client_capture.py --warmup` exists for exactly this reason, and
every video sidecar records its `max_path_delta_m` so the reader can check.

**Possible but not observed — an outcome flip.** All ten runs matched the
logged outcome, isolated included (5/5). But v01 changed its step count, and at
0.287 m of drift an episode terminating near the 0.4 m goal tolerance or near
an obstacle could plausibly flip. No such flip was seen, and none is claimed
either way; the sample is five isolated episodes on one layout with one
controller.

## Threat-to-validity statement for the report

> The simulator is deterministic given a fixed episode sequence: re-running the
> full 90-episode protocol reproduced all ten cells with a maximum per-episode
> path difference of 0 m. It is **not** deterministic for an episode run in
> isolation — the reset settles to a tolerance rather than a fixed point, so a
> single episode re-run in a fresh session drifted 1.6 mm to 287 mm from its
> logged trajectory (n = 5; outcome still matched the log in 5/5, step count in
> 4/5). Episodes within a cell are consequently coupled
> through residual airframe state, and are not strictly independent draws, so
> the Wilson intervals reported for them are approximate.

## How to remove it, if we want to

Settle to a *fixed point* rather than a tolerance: hold the hover until the
state stops changing to within a much tighter bound, or write the full rigid-body
state (pose, linear and angular velocity) directly and skip the settle. Either
would make individual episodes independently reproducible. Both change
`WebotsBackend.reset`, which would break bit-comparability with the v1 results,
so this is a deliberate **not-now**: the current behaviour is documented instead.

---

# Fix — deterministic reset (25 Nov 2026)

## What was changed

`WebotsBackend.reset_mode` selects the settle procedure.

| Mode | Procedure | Status |
|---|---|---|
| `settle_tolerance` | v1 behaviour: hover until `speed < 0.02 m/s` and `|alt − 1.0| < 0.02 m`, up to 6 s. **Default**, so all published v1/v2 numbers stay bit-comparable. | unchanged |
| `deterministic` | Clear the autopilot integrator, then run a **fixed 750 steps** (750 × 8 ms = 6.0 s, the same budget) of the hover controller, re-asserting pose and zero velocity after **every** step. | new |

Why it works: writing the pose and velocity each step means the state at the
end of every step is exactly the start state, whatever the controller did in
between. The only thing that evolves is the rotor spin-up, which is then a
deterministic function of the fixed step count. No tolerance is widened and no
drift is hidden — the loop simply has no history-dependent exit condition.

Nothing in Spencer's PPO model, `navigation.py` or the controllers was touched.

## Measured result — L00, `ppo_seed7`, variant v01

Same episode, run isolated (nothing before it) and after a 16-episode prefix:

| Quantity | `settle_tolerance` | `deterministic` |
|---|---|---|
| Steps, isolated vs sequence | 59 vs **60** | 59 vs **59** |
| Max trajectory difference between the two arms | **0.287 m** | **1.0e-06 m** (the 6-dp logging rounding) |
| Initial altitude difference | — | **2.5e-12 m** |
| Initial x, y, roll, pitch, vx, vy difference | — | **0.0, exactly** |
| Outcome | goal / goal | goal / goal |

Source: `results/reset_study/L00_C_ppo_seed7_v01_{isolated,sequence}_det.json`.

**On this target, the episode was independently reproducible under `deterministic`** (1.0e-06 m). The later multi-target validation found a worst case of 2.139e-04 m, which misses the strict 1e-5 m gate on 1 of 5 targets; see the status note at the top of this page.

## What has NOT been done

*(Both items below were done later on 25 Nov 2026: the full 540-episode ladder was
re-run into `results/v2_S6_deterministic/`, and the validation was extended to
5 targets × 2 controllers. See the status note at the top of this page.)*

- **The 90-episode experiment has not been re-run under the new mode.** The
  published L00 and S6 numbers were all produced with `settle_tolerance` and
  are unchanged; they remain valid as *sequence-reproducible* results and are
  labelled as such. A deterministic-mode re-run would produce a **different and
  not directly comparable** set of trajectories — the reset procedure differs —
  and must be written to a separate results folder, never over the existing one.
- Verified on **one** variant, two arms. It should be extended to the full
  n = 5 study and to a second controller before the mode is made the default.
