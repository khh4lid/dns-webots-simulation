# Changelog

All notable changes to the simulation environment.

## [2.0] — 2026-11-25 — S6 generalisation and transparency release

### Added
- **Seeded scenario generator** (`dns/generator.py`): one YAML config plus a
  seed produces a byte-identical scenario JSON, hashed, from which the Webots
  world is written. Byte-identity is tested across separate processes.
- **Six-layout distribution-shift ladder** L00–L05, each validated,
  A*-feasible and frozen before any controller ran. En-route corridor widths
  2.08 / 1.87 / 2.08 / 1.32 / 0.85 / 0.68 m.
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
- v1's L00 protocol reproduces exactly: **10/10 cells, maximum per-episode path
  difference 0 m**.
- Maximum obstacle t = 0 pose error across all six layouts: ≤ 4.0e-06 m.
- 2 of 540 episodes exceeded 0.50 rad airframe tilt.

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
