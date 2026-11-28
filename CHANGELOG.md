# Changelog

All notable changes to the simulation environment.

## [Unreleased] — 2026-11-28 — benchmark phase: plan, benchmark sheet, traditional baselines

Documentation and teammate deliverables only. No simulation code, scenario, world, result or evidence file changed.

### Added
- **`docs/benchmark/BENCHMARK_TVT2025.md`.** The team's benchmark: Wang, Zhou & Wu, *IEEE Transactions on Vehicular Technology*, 2026. It holds:
  - the verified citation and an abstract-level summary;
  - an extraction sheet to fill from the full text, which has not been read yet;
  - a side-by-side with our verified setting;
  - eleven gap hypotheses;
  - the fair-comparison rule: no "better than the benchmark" claim unless both methods ran under the same protocol.
- **`docs/RESEARCH_PLAN.md`.** The pipeline (Webots for evaluation, a fast surrogate for training), v3 contracts, work packages per teammate (ENV, SENS, PPO, BASE, EVAL), experiments E1–E6, decision gates D1–D4, a relative timeline and the paper outline.
- **`baselines/traditional/`.** Ariana's A\*/RRT prototype, as received:
  - her notebook, progress report and slides, unmodified;
  - a README with her results, notes on reading them, and the plan to port the planners into the Webots arena.

### Changed
- **README.** A *Research direction* section, team rows and next steps for each work package, entry points for the baselines and the plan, and a repository-map entry for `baselines/`.

## [Unreleased] — 2026-11-27 — documentation sync with the final meeting deck

Documentation only. No code, scenario, world, result or evidence file changed.

### Changed
- **README** aligned with the final deck (`docs/meetings/2026-11-27/DNS_SimEnv_FINAL_2026-11-27.pdf`) and with the evidence files:
  - new *Status, stable version and next steps* section;
  - new *Reset reproducibility: two cohorts kept apart* section;
  - a *Where things are* table and contributing notes in the quick start;
  - a link to the final deck.
- **Reset documents** (`RESET_DETERMINISM.md`, `RESET_MODE_COMPARISON.md`) carry a status note:
  - the deterministic reset is an opt-in mode already contained in `main`, not the default;
  - it misses the strict 1e-5 m order-independence gate on 1 of 5 targets (worst 2.139e-04 m);
  - its cohort is reported separately and never mixed with the published one.
- **`docs/FIGURES.md`** points at repository paths instead of the one-off meeting-package folders.

### Fixed — statements the evidence does not support
- **Release links.** The README linked the clips to a `v2.0` release and called the PPTX a release asset, but no GitHub release has been published. The links now point at the committed GIFs.
- **Reset start pose.** "Reset repeatability 2.8 µm" was the start-pose check (`docs/VALIDATION_EVIDENCE.md`, check 17), not episode repeatability. It is now labelled as such.
- **Isolated-episode drift.** The drift range was given as 2.3–287 mm; `results/reset_study/reset_study.csv` gives 1.6–287 mm.
- **Unqualified "0 m".** The "0 m" reproduction is now qualified everywhere as "when the session is replayed in order".
- **Mirror result.** It is stated as no sign of left–right memorisation in this test, explicitly not proof of its absence.
- **L04.** Only the fast-profile tracker attempt is backed by `results/layout_gate/L04.json`, and the README says so beside the final deck's summary. The exclusion is not a claim that L04 is impassable.
- **Obstacle detectability.** "Nothing is invisible to the sensor" is replaced by the single-plane, 5 m range and line-of-sight limits.
- **LiDAR angular offset.** The implementation uses 180.0° (`results/calibration.json`); the warehouse calibration fits 180.5°. The docs had quoted one or the other as *the* offset.
- **Problems found by checking.** Six problems found (five fixed, one open: the reset), not "five defects".
- **"Orders of magnitude".** "The cart-phase variation dominates the coupling by orders of magnitude" is replaced by the measured comparison.

## [2.0] — 2026-11-25 — S6 generalisation and transparency release

### Added
- **Seeded scenario generator** (`dns/generator.py`): one YAML config plus a
  seed produces a byte-identical scenario JSON, hashed, from which the Webots
  world is written. Byte-identity is tested across separate processes.
- **Six-layout distribution-shift ladder** L00–L05, each validated,
  A*-feasible and frozen before any controller ran. En-route corridor widths
  2.08 / 1.87 / 2.08 / 1.31 / 0.85 / 0.68 m.
- **S6 generalisation experiment**: 540 episodes = 480 distinct
  configurations — 5 controllers × 6 layouts × (Mode B 3 runs + Mode C 15 cart
  phases). The protocol was committed before anything ran.
- **Feasibility and difficulty tooling** (`dns/feasibility.py`): exact
  clearance field, A*, widest-path bottleneck corridor width, IoU
  dissimilarity, cart-interaction descriptors.
- **World ↔ scenario verification** (`dns/worldcheck.py`): refuses to produce
  results when the loaded world's node poses, time step, observation interval
  or LiDAR configuration disagree with the scenario.
- **Blocking layout gate**: 26 contract tests, the world check and a flown A*
  reference traversal, per layout.
- **LiDAR–cart consistency** against an independent analytic ray caster sampled
  at the same 720 bearings: cart-sector MAE 0.0193 m over 650 comparisons.
- 26 generator and validator unit tests; real Webots captures, four GIFs, the
  demonstration video, and 14 architecture decision records.

### Changed
- Repository reorganised into `results/v1_L00` and `results/v2_S6`,
  `config/scenarios`, `docs/` sub-folders and `assets/` sub-folders. Moves used
  `git mv`, so file history follows the files.
- `tools/` merged into `scripts/`.
- `scripts/build_deliverable.py` no longer bakes in a machine-specific Webots
  path; it probes the usual install locations.

### Fixed — corrections to v1 (full list in `docs/verification/V1_CORRECTIONS.md`)
- **C1** The reactive explanation compared the 1.45 m repulsion radius against
  the crate's *centre* distance (1.54 m); the controller reacts to the
  *surface*, 1.0582 m away. Restated with the measured mechanism.
- **C2** "Cart vs model 0.00e+00 m" is true by construction; reclassified as an
  implementation-consistency check and replaced with real dynamic evidence.
- **C3** v1's Mode B "3/3" is 1/1 per controller — the episode seed is accepted
  but never used.
- **C4** Per-seed results are pooled with Wilson intervals and never ranked.
- **C8** v1 shipped a `protocol.json` describing a different run than its own
  `results.csv`.

### Verified
- v1's L00 protocol reproduces **10/10 cells with a maximum per-episode
  path difference of 0 m when the session is replayed in order**. A single
  episode re-run in isolation drifts up to 0.287 m -- see
  `docs/verification/RESET_DETERMINISM.md`.
- Maximum obstacle t = 0 pose error across all six layouts: ≤ 4.0e-06 m.
- 3 of 540 episodes exceeded 0.50 rad airframe tilt.

### Known limitations
- **L04 is excluded** from the generalisation conclusion pending a decision: no
  controller and no reference-tracker setting reaches the goal within the
  90-step horizon, so its 0/90 is not attributable.
- Physical contact response with the cart is not modelled.
- The LiDAR gimbal is ideal, which is a best case rather than a real sensor.
- One unexplained sensor reading: 0.0536 m against 0.62 m ground-truth clearance.

## [1.0] — 2026-11-24 — first research milestone
- One obstacle layout, 90 episodes (Mode B static / Mode C dynamic), five
  controllers; the v1 verification report; the Webots environment, drone PROTO,
  cascaded autopilot, LiDAR sector pipeline and the teammate handoffs.
- Tagged `v1.0` and recoverable in full.
