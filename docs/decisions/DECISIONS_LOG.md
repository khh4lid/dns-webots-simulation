# DECISIONS.md — choices made, alternatives considered, and why

Each entry: the decision, what else was on the table, and the reason. Where a
decision was forced by a measurement, the measurement is quoted.

---

### D1 — One Webots world per layout, selected by world file

**Alternatives:** (a) a single world with a superset of obstacle nodes,
repositioned per layout; (b) `Supervisor.worldLoad` to swap worlds in-session.

**Chosen:** one generated `.wbt` per layout × mode, launched by the harness.

**Why:** obstacle counts differ across layouts (6, 6, 6, 6, 9, 10), so a
superset world would leave unused nodes parked somewhere in the arena where
they would still be collidable and still return LiDAR. `worldLoad` restarts the
controller connection, which the extern-controller harness is not built for.
A world per layout also means the world file *is* the scenario record, which is
what lets `dns/worldcheck.py` verify node-by-node that the right geometry
loaded. **Spencer's `handoff.py` is untouched** — scenario selection happens
entirely on the environment side, as Section 2 requires.

### D2 — Mode B and Mode C share one loaded world

**Alternative:** a separate Webots session per mode.

**Chosen:** load the layout's `_C` world and point the backend at the `_B`
scenario for Mode B; the cart is repositioned to its parked pose on reset.

**Why:** Mode B and Mode C have an identical static obstacle set by
construction (asserted in `tests/test_generator.py`), so sharing the loaded
world makes the B-vs-C comparison bit-comparable on the same geometry and
halves the session count. The parked cart is reached through the normal
`_move_obstacles` path, so nothing special-cases Mode B.

### D3 — No start jitter; Mode B keeps 3 repeats of 1 configuration

**Alternative:** jitter the start point to manufacture three genuinely distinct
Mode B variants, which would raise Mode B's n from 1 to 3.

**Chosen:** leave the start fixed and report Mode B as n = 1 with three
repeats.

**Why:** the Section 3 contract freezes "arena bounds, flight altitude, start,
goal and tolerance identical to L00". Jittering the start to improve a sample
size would break a frozen contract to flatter a statistic. The repeats are kept
because they are the determinism evidence, and the analysis counts n = 1 per
the integrity rule that identical deterministic repeats count once.

### D4 — Mode C reuses v1's exact phase rule and seeds

**Alternative:** draw fresh phases for v2.

**Chosen:** `numpy.random.default_rng(200_000_000 + i).uniform(0, 2π)` for
i = 0…14, identical to v1.

**Why:** it makes the L00 cells directly comparable with v1's `results.csv`,
which is what turned the reproduction check into a strong result (10/10 cells
identical, max path difference 0 m, session replayed in order). Fresh phases would have made any
difference uninterpretable.

### D5 — The cart's dynamics are frozen; only its lane x is perturbed

**Alternative:** jitter the cart's sweep centre and amplitude along with
everything else in L01.

**Chosen:** jitter only the lane position x; amplitude (1.9 m), speed
(0.45 m/s), period (16.8889 s) and sweep centre (y = 0) are identical in every
layout.

**Why:** Section 2 forbids changing the moving-obstacle model. Holding the cart
dynamics fixed also keeps Mode C's timing problem comparable across the ladder,
so a success difference between layouts is attributable to static geometry
rather than to a different hazard.

### D6 — L05's narrow gate is constructed, then verified, not sampled

**Alternative:** rejection-sample until a narrow passage happened to appear.

**Chosen:** place the gate explicitly (rack → gap *w* → crate → gap → column →
wall), then accept the layout only if the **measured** widest-path bottleneck
falls in `target_corridor_width_m` = [0.50, 0.78] m.

**Why:** two constraints pull against each other — `min_obstacle_gap_m` is
0.50 m, which is also the floor of the narrow passage the family is supposed to
contain — so the acceptable band is thin and uniform sampling almost never hits
it. Measured: 16 of 20 draws died on `free_point` before the corridor test was
ever reached. Constructing the gate and verifying the descriptor keeps the
difficulty claim empirical while making the search terminate. Realised value:
**0.684 m**, inside the band.

### D7 — Corridor width is reported twice: whole-route and en-route

**Alternative:** report only the widest-path bottleneck over the whole route.

**Chosen:** report both, and use the en-route figure for difficulty.

**Why:** the start and goal are fixed by contract at 0.85 m from an arena wall,
so the whole-route bottleneck is capped at 1.70 m by the *endpoints* regardless
of layout. Measured: L00, L01 and L02 all report exactly 1.650 m, which says
nothing about their obstacles. Excluding a goal-tolerance-radius neighbourhood
of each endpoint gives 2.080 / 1.868 / 2.080 / 1.315 / 0.850 / 0.684 m — a
ladder that actually varies with the layout.

### D8 — The analytic clearance field replaces a distance transform

**Alternative:** SciPy's `distance_transform_edt` on a binary occupancy grid.

**Chosen:** evaluate `PlacedObstacle.surface_distance` exactly on every grid
cell.

**Why:** it is exact rather than a raster approximation, it removes a
dependency SciPy was not already installed for, and — most importantly — it
uses *the same geometry code as the collision test and the ray caster*. A
layout cannot be declared feasible by one geometry model and collided by
another, which is precisely the v1 circle-vs-box defect.

### D9 — The reference tracker is a profile ladder, not one tuned setting

**Alternative:** hand-tune a single tracker until every layout passed.

**Chosen:** try documented (planning margin, speed, slew) combinations in order
and report which one succeeded.

**Why:** tuning one profile until all six layouts passed would be fitting the
test tool to the desired answer. A ladder makes "how much gentleness this
layout needed" part of the result. Five of six layouts pass on the first
profile with peak tilt 0.2496 rad, well inside the 0.45 rad authority.

### D10 — L04 is reported but excluded from the generalization conclusion

**Alternative:** report L04's 0/90 as a generalization result, or regenerate
L04 until it behaved.

**Chosen:** report the data in full and mark its traversability **unconfirmed**,
excluding it from the generalization claim.

**Why:** all five controllers score 0/90 on L04 *and* the reference tracker
failed to fly it (peak tilt 1.5905 rad against 0.45 rad of authority). With the
reference traversal unconfirmed I cannot separate "the policies do not
generalize here" from "this layout may not be flyable by this airframe", and
reporting the former would be a claim the evidence does not support.
Regenerating L04 after seeing its outcomes would violate the pre-registration,
so the layout is kept exactly as frozen. See `ASSUMPTIONS.md` A5 for what would
resolve it.

### D11 — Full LiDAR range images are archived for a declared subset

**Alternative:** archive all 540 episodes, or none.

**Chosen:** archive the 12-sector packed scan plus pose and ground truth for
**variant 00 of every (layout, mode, controller) cell** — 60 runs.

**Why:** all 540 would be roughly 280 MB of float64 for data that is identical
in kind. The cap is declared in `LIDAR_ARCHIVE_VARIANTS`, in every run manifest
and here, so the subsetting is never mistaken for full coverage. Per-step
trajectory, obstacle and event logs **are** written for all 540.

### D12 — Stable-Baselines3 is not cited

**Alternative:** cite Raffin et al. (2021), as the brief tentatively suggested.

**Chosen:** do not cite it.

**Why:** `DroneNavPPO/ppo.py` is a from-scratch NumPy PPO-Clip implementation —
a hand-written `MLP`, a hand-written `Adam`, and `np.savez_compressed`
checkpoints. `PPO.load` is Spencer's own classmethod, not SB3's. Citing SB3
would misattribute his work.
