# ASSUMPTIONS.md — what was assumed, why, the risk, and how to confirm it

Each entry: the assumption, the reason it was necessary, what it would cost if
wrong, and the concrete step that would settle it.

---

### A1 — The start/goal keep-out is 0.75 m

**Why:** the project defines no value of its own; the task specifies 0.75 m as
the fallback. L00's own smallest start/goal surface gap (crate stack at
1.058 m) is larger, so adopting 0.75 m does not exclude the training layout.

**Risk:** low. A larger keep-out would only have made L01–L05 easier.

**To confirm:** agree a project value with Dr. Mehdi; regenerate — the
constraint is one line in `generator_config.yaml` and the layouts are
reproducible from it.

---

### A2 — The near-miss threshold is 0.30 m

**Why:** no project value exists; 0.30 m is the task's fallback and sits above
the 0.18 m collision radius, so a near miss is never also a collision.

**Risk:** low, and fully reversible — `min_clearance_m` and the per-step
`clearance_m` are logged for every episode, so any threshold can be recomputed
from `06_DATA/results_v2.csv` and `runs/*/trajectory.csv` without re-running.

---

### A3 — Mode B's three episodes are repeats, and n = 1

**Why:** measured (`AUDIT_SUMMARY.md` item 4) — the seed is unused and the
policies are deterministic.

**Risk:** this makes every Mode B claim weaker than v1's, deliberately. If
Spencer later adds stochastic action sampling, Mode B repeats would become
genuinely distinct and n would rise to 3.

**To confirm:** it is already confirmed by measurement; it would only change if
`PPO.predict` changed.

---

### A4 — The cart's lane may move in x, but its dynamics may not

**Why:** Section 2 forbids changing the moving-obstacle model. Holding
amplitude, speed, period and sweep centre fixed keeps Mode C's timing problem
comparable across the ladder.

**Risk:** the ladder varies static geometry only; it does **not** test
generalization to different obstacle *speeds*. Stated as a limitation.

**To confirm:** Phase 4 (motion variants C1–C3) would address it; **not run** —
see `EXECUTION_LOG.md`.

---

### A5 — L04's traversability within the 90-step horizon is UNCONFIRMED

**Why it matters:** all five controllers score 0/90 on L04. Before that can be
read as a generalization result, the layout has to be shown flyable.

**What was measured.** L04 passes every generator validator and is A*-feasible
with an en-route corridor of 0.850 m. The flown reference traversal, however,
reaches the goal at **no** tested combination:

| Planning margin | Tracker speed | Outcome | Peak tilt |
|---|---|---|---|
| 0.22 m | ≤ 0.48 m/s | timeout at 90 steps, **no collision** | 0.105 rad |
| 0.22 m | ≤ 0.84 m/s | collision at 13 steps | — |
| 0.22 m | ≤ 1.08 m/s | collision at 10 steps | — |
| 0.12 m | ≤ 1.08 m/s | timeout, airframe tumbled | **1.5905 rad** |

So the route is flyable slowly and safely, but not fast enough to finish inside
the 90-step (18 s) horizon; above that speed the tracker clips `crate_extra_3`
at x ≈ 3.1 m — **the same body all three PPO checkpoints hit**.

**Assumption taken:** L04 is reported in full but **excluded from the
generalization conclusion**, because "the policies do not generalize here"
cannot be separated from "this layout may be outside the airframe's feasible
envelope within the horizon".

**Risk:** if L04 is in fact comfortably flyable, I am discarding a real result.
If it is not, including it would have been a false finding. The conservative
choice is to exclude and say so.

**To confirm:** either (a) a better-tuned tracker, or (b) raise the horizon for
a diagnostic-only run and check whether the route completes. **Not done** — the
horizon is a frozen contract (`HORIZON = 90`) and changing it, even
diagnostically, after seeing outcomes would compromise the pre-registration.
This is the single clearest open item for Dr. Mehdi.

---

### A6 — The reference tracker's speed is below the contract cap

**Why:** the contract gives an *upper* bound of 1.2 m/s per axis; the tracker
schedules 0.45–0.90 of it on local clearance. Flying below a cap respects it.

**Risk:** none to the contract. It does mean the reference traversal is a
*traversability* check, never a performance number, and it is labelled as such
everywhere.

---

### A7 — Full LiDAR range images are archived for variant 00 only

**Why:** all 540 episodes would be ≈280 MB of float64 for data identical in
kind.

**Risk:** an analysis needing per-beam data for a non-archived episode must
re-run it. The run is reproducible, so this costs time, not information.

**Declared in:** `LIDAR_ARCHIVE_VARIANTS`, every run manifest, and
`DECISIONS.md` D11 — never presented as full coverage.

---

### A8 — Physical contact response with the cart is not modelled

**Why:** the cart is repositioned by `Field.setSFVec3f`, so Webots reports no
rigid-body velocity for it and it cannot transfer momentum.

**Risk:** a drone–cart collision is decided by the **geometric swept test**, not
by contact dynamics. A real cart would push the drone. Every `collision_cart`
outcome should be read as "the swept footprints intersected", not "a physical
impact was simulated".

**To confirm:** give the cart a `Physics` node and drive it with a motor rather
than by pose writes. Out of scope for this package; stated as a limitation in
the report.

---

### A9 — The 3 checkpoints are the population of interest

**Why:** Spencer supplied exactly three.

**Risk:** three training seeds is a very small sample. Pooling over them with
Wilson intervals is honest about the precision, but it cannot separate
checkpoint-to-checkpoint variance from layout effects.

**To confirm:** more training seeds — a request for Spencer, recorded in
`09_HANDOFFS/PPO_Interface_v2.md`.
