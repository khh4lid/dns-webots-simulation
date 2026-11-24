# DNS — PRE-MEETING MASTER TASK (FINAL)

> Verbatim record of the task prompt that produced this package.
> Saved as required by Section 0. Research meeting with Dr. Mehdi: Friday 27 November 2026.

Close the spatial-generalization gap (S6), correct v1, and document everything

## 0. ROLE AND MISSION

You are BERD, my research engineer on the DNS UAV-navigation project. My contribution is the Webots simulation environment. You delivered v1 (DNS_SIMULATION_RESEARCH_DELIVERABLE). Your own conclusion was that the biggest remaining gap is obstacle-position randomization (S6): without it there is no evidence of spatial generalization.

Mission for Friday — produce v2:
1. close S6 with a reproducible scenario generator and a pre-registered spatial-generalization experiment;
2. correct and reframe the weak points of v1 (Phase 3);
3. document every step with real evidence: logs, tables, figures, charts, videos, GIFs and sensor visualizations;
4. package it so Dr. Mehdi can see what I built, check every claim, rerun it, and see how Spencer and the sensor teammate can use it today.

Execute end-to-end. Do not stop at a plan. Do not restart. Do not modify v1. Save this prompt as 10_EVIDENCE/TASK_PROMPT.md.

## 1. STARTING POINT (from your v1 report — carry forward, re-verify, explain any change)

Platform and files
- Windows; project root D:\Download\drone_navigation_system\
- v1 package: DNS_SIMULATION_RESEARCH_DELIVERABLE\ (59 files, 5.9 MB) + .zip (5.3 MB) — frozen, do not modify
- baseline snapshot: webots_dns_BASELINE_SNAPSHOT_2026-11-24 (227 files)
- v1 contents used below: 02_DEMONSTRATION\DNS_Webots_Research_Demo.mp4; 03_RESEARCH\DNS_Simulation_Research_Report.pdf (11 pages, 5 references); Verification_Report.md (15/15 PASS); results.csv (90 rows); 04_TEAM_HANDOFF\ (PPO_Interface.md, two_frames.json, Sensor_Interface.md, lidar_calibration.json)
- video capture: the Webots built-in recorder produced blank frames on this machine; the working method is the offscreen Camera node (real renders, 1:1 frames)

Baseline world — L00, the layout Spencer's checkpoints were trained on
- arena 10.0 × 6.0 m (state the exact bounds as implemented), walls 2.5 m, flight altitude 1.0 m, 1 m floor grid
- start (0.85, 0.00) m; goal (9.15, 0.00) m; success tolerance 0.4 m
- rack_a (3.5, −1.0), 0.8 × 2.4 × 2.0 m; rack_b (6.5, 1.0), 0.8 × 2.4 × 2.0 m — each intrudes 0.20 m across the straight start–goal line (intentional: an open line let "direct" succeed and the static mode measured nothing)
- column_1 (2.0, 2.3) and column_2 (8.0, −2.3), 0.4 × 0.4 × 2.5 m
- crate_stack (7.9, 0.9), yaw 0.35 rad, 0.7 × 0.7 × 1.6 m
- cart (5.1, 0.0), 0.9 × 0.55 × 1.5 m, linear ±1.9 m along y at 0.45 m/s, period 16.89 s, kinematic

Interfaces (unchanged)
- PPO path: Spencer's handoff.py, unmodified, SHA-256 matching his recorded fingerprints; two-frame observation, frames 0.2 s apart (two_frames.json)
- LiDAR: 360°, 720 beams (read via getHorizontalResolution()), 5 m cap, 12 sectors; sector 0 = front, 3 = right, 6 = back, 9 = left; observation interval 0.200000 s

v1 measured results
- timing 0.200000 s (error < 3e-16); reset error 2.8 µm; LiDAR angular map RMS 0.025 m; full scan vs independent analytic ray caster mean error 0.048 m (with occlusion); cart vs its motion model 0.00e+00 m at 0.4500 m/s
- controllers: ppo_seed7, ppo_seed19, ppo_seed43 = trained PPO checkpoints (PPO.load); direct and reactive = hand-written rules from Spencer's code (not learned)
- 90-episode experiment on L00 (Mode B static / Mode C dynamic): ppo_seed7 3/3 and 9/15; ppo_seed19 3/3 and 8/15; ppo_seed43 3/3 and 7/15; reactive 0/3 and 0/15; direct 0/3 and 0/15
- the earlier 15-episode experiment is superseded — never mix its numbers with these

Defects found and fixed in v1 (keep each as a regression test)
- shelves modelled as circles → false collision about 0.5 m into an empty aisle; replaced by exact box distance, slab ray intersection and a swept collision test
- the camera generator rotated about the vertical axis instead of tilting (unnoticed because runs were headless)
- the world file placed the cart at its motion centre instead of its t = 0 pose (1.9 m mismatch; it corrupted a calibration reference by 1.34 m)
- the calibration tool loaded a fixed scenario (it compared one world's measurements with another world's geometry)

Disclosed v1 limitations: one static layout; three seeds (spread 47–60% ≈ the effect); untuned baselines; ideal gimbal; single scan layer; kinematic obstacle motion.

## 2. SCOPE BOUNDARIES

My responsibilities — stay inside them: simulation platform selection and setup; navigation environment design; start and goal definition; static obstacle development; moving obstacle modelling and validation.

Never:
- retrain, fine-tune or modify Spencer's checkpoints, handoff.py, training code, or the direct/reactive controllers (his code — run them as-is and label them untuned references);
- modify the sensor teammate's processing algorithm;
- change the contracts in Section 3, or basicTimeStep, the drone model, controller gains, flight altitude or arena bounds in L00–L05 (ablations run in separate bench copies);
- modify v1 or the baseline snapshot;
- delete, hide or overwrite failed runs.

Allowed: running all five controllers read-only; reading the LiDAR data path; building environment test tools (generator, validators, analytic ray caster, clearance monitor, A* reference-traversal follower), labelled as test tools, not navigation methods.

If handoff.py cannot select a scenario without being edited, implement scenario selection on the environment side (world file or supervisor) — never inside his code.

## 3. FROZEN CONTRACTS — make them executable

Convert PPO_Interface.md, two_frames.json, Sensor_Interface.md and lidar_calibration.json into automated contract tests (if not already done) and run them on L00 and on every new layout:
- observation shape, dtype, range and two-frame structure; action bounds and speed limit;
- observation interval exactly 0.200 s of simulated time and an integer multiple of basicTimeStep;
- 720 beams, 360°, 5 m cap, 12 sectors, 0/3/6/9 = front/right/back/left;
- arena bounds, flight altitude, start, goal and tolerance identical to L00 (primary set).
A contract failure on a new layout blocks that layout from the experiment until it is fixed.

## 4. INTEGRITY RULES (non-negotiable)

1. Every number in the report, slides, README and talking points is traceable in 10_EVIDENCE/CLAIMS_EVIDENCE.csv (claim → value → unit → n → data file → script → figure → run IDs). Carry v1 numbers forward with their v1 source files.
2. Never fabricate, estimate or "illustrate" results. Not run = "not run" + reason.
3. n counts distinct, documented configurations only. Identical deterministic repeats count once. Report k/n; Wilson 95% intervals for pooled results; no significance tests; no ranking where intervals overlap.
4. Single source of truth: every world, verification tool, ray caster and figure reads geometry from the scenario JSON by ID, and every run asserts that the loaded world matches the scenario hash (this is the class of bug found in v1).
5. Label checks that are true by construction (e.g. the kinematic cart vs its own motion model) as implementation-consistency checks — never as physical validation or headline evidence.
6. Pre-register the experiment and freeze the scenario set before running any controller on new layouts. Never alter layouts after seeing outcomes; if a bug forces a change, log it, regenerate, rerun everything affected and keep the superseded results.
7. Provenance label on every visual: WEBOTS CAPTURE (offscreen Camera), LOGGED-DATA RENDER, or SCHEMATIC.
8. "Verification" in simulation — never "real-world validation". Do not claim the generator is novel. No publication guarantees.
9. Policy-level conclusions belong to Spencer; the environment "exposes" behaviour. Untuned references are never used to rank methods.
10. Cite only references verified via DOI, arXiv or the publisher page.

## 5. PHASES

### PHASE 0 — Freeze and regression gate (MUST, keep it short)

1. SHA-256 manifest of v1, its ZIP, the snapshot, handoff.py and the checkpoints → 10_EVIDENCE/BASELINE_HASHES.txt; recheck at the end. Use git (branch premeeting-2026-11-27) if the project is a repository; otherwise rely on the snapshot + manifest and create a second snapshot at the freeze.
2. Record the Windows version, CPU/GPU, exact Webots version, Python version and pip freeze → ENVIRONMENT.txt.
3. Rerun the 15 v1 verification checks; they must still pass.
4. Document exactly what differs between episodes inside each v1 cell — the 3 Mode B and the 15 Mode C episodes per controller (start jitter? cart phase? policy sampling seed? deterministic flag?). If any are identical configurations, say so; it changes how v1 is reported (Phase 3).
5. Rerun the L00 90-episode protocol (if runtime forbids, at least all PPO cells) and compare with results.csv: identical outcomes are determinism evidence; any difference must be explained.
6. Measure mean wall-clock time per episode and project the Phase 2 runtime. If it exceeds the time available, pre-register a reduction rule before running (e.g. fewer, evenly spaced cart phases; reactive on a subset of layouts).
7. AUDIT_SUMMARY.md (≤ 1 page) → Checkpoint A.

### PHASE 1 — S6 scenario generator (MUST, top priority)

Same arena, assets, start, goal, tolerance, altitude, cart dynamics and contracts as L00. Config (07_SCENARIOS/generator_config.yaml) + seed → canonical scenario JSON (byte-identical for the same config + seed; unit-tested) → world file or supervisor loader, selectable by ID.

Layout set — a graded distribution-shift ladder (one seed per family minimum; extra seeds only if time allows):
- L00 Baseline — the training layout (reference, unchanged)
- L01 Perturbed — same obstacles, positions jittered within ±0.3 m and yaw within ±10° (near-distribution)
- L02 Mirrored — L00 reflected across y = 0 (y → −y, yaw → −yaw, cart initial direction mirrored): identical geometric difficulty, flipped geometry — a direct memorization probe
- L03 Aisle shift — racks repositioned along x and alternated in side, each still blocking the straight line by a sampled overlap
- L04 Clutter — L00-type assets plus extra crate stacks under minimum-gap constraints
- L05 Hard — a narrow passage close to (but above) the minimum traversable width and/or an occluded goal

Validators (resample on failure; log rejection counts):
- inside the arena with wall margin; no overlaps; minimum gaps where a passage is intended;
- start/goal keep-out (project value, else 0.75 m) clear of static obstacles and the cart's swept area;
- every obstacle taller than the 1.0 m scan plane + margin;
- cart lane clear of static obstacles; keep the lane at x = 5.1 if free, otherwise relocate it and document why; cart speed, amplitude and period as in L00;
- straight start–goal line blocked in L01–L05 (as in L00) so static mode stays informative; record the intrusion depth;
- v1 regression checks asserted for every layout: world ↔ scenario hash match, cart t = 0 pose consistency, camera pose sanity.

Feasibility and descriptors (per layout):
- occupancy grid (0.05 m), obstacles inflated by drone radius + margin; A* start→goal must exist;
- descriptors: A* length, tortuosity, minimum corridor width (distance transform), occupied area %, obstacle count, start–goal line of sight, straight-line intrusion depth, number of cart crossings of the A* path, fraction of time the cart blocks it (< 100%);
- dissimilarity to L00: 1 − IoU of the static occupancy grids, plus mean obstacle displacement — "how different" is reported separately from "how hard";
- reference traversal (environment test tool): an A* waypoint follower using the same command interface and speed limit flies each layout in Webots (static) to prove physical traversability — needed because direct and reactive both fail on L00.
Mark L01–L05 TEST-ONLY (held out from any training).

Outputs: scenario JSONs + hashes, worlds or loader, scenario_table.csv, asset_library.csv, a scaled top-down map per layout, validator and generator unit tests.

### PHASE 2 — S6 spatial-generalization experiment (MUST)

Write and commit protocol/S6_GENERALIZATION_PROTOCOL.md (matrix, variants, metrics, termination rules, analysis plan, reduction rule) before running anything.

- Controllers: ppo_seed7, ppo_seed19, ppo_seed43 through handoff.py, unmodified; direct and reactive as untuned references.
- Matrix: 5 controllers × L01–L05 × (Mode B: 3 variants + Mode C: 15 variants), identical to the L00 protocol so results are comparable = 450 episodes (+ the L00 regression run from Phase 0). Every variant documented (e.g. start jitter, cart phase) and distinct.
- Per episode: outcome (success / collision-static / collision-cart / timeout / out-of-bounds), time to goal, path length, path efficiency (A* length ÷ flown length), minimum and mean clearance, near-miss count and duration (project threshold, else 0.30 m), termination step.
- Analysis: k/n per cell; PPO pooled over the three seeds per layout and mode with Wilson 95% intervals; generalization gap vs L00; L00 vs L02 as the memorization test; success vs dissimilarity and vs difficulty (exploratory, six layouts); failure taxonomy with one video per failure type.
- Framing: the environment exposes the policies' generalization limits; a drop on new layouts is a valid finding, not an environment failure.

Outputs: results_v2.csv (including the L00 rows), per-run folders, figures F2–F3.

### PHASE 3 — Correct and reframe v1 (MUST)

1. The reactive explanation is numerically wrong as written: "its repulsion radius (1.45 m) is larger than the crate-to-goal distance (1.54 m)". From the reported geometry, 1.54 m is centre-to-goal; the crate-stack surface is about 1.06–1.14 m from the goal (depending on the yaw sign). If the controller measures repulsion from the obstacle surface or the nearest LiDAR return, the goal lies inside the 1.45 m zone — a GNRON case (Ge & Cui, IEEE Transactions on Robotics and Automation 16(5), 2000, doi:10.1109/70.880813). Verify against Spencer's reactive code (read-only), restate it precisely with the reference points named, and add a figure (repulsion zone, goal-tolerance disk, stop point) — or remove the claim.
2. "Cart vs model 0.00e+00 m" is true by construction for kinematic motion. Keep it only as an implementation-consistency check. The headline dynamic evidence becomes:
   - LiDAR–cart consistency: range in the expected sector vs the analytic ray caster applied to the logged cart pose, over full cycles;
   - swept-collision detection against the moving cart (probe cases);
   - what Webots physics reports for the cart while it moves (node velocity) — if it is ≈ 0, state that physical contact response with the cart is not modelled and that collisions are decided by the geometric swept test.
3. Distinctness: apply Phase 0 item 4. If the v1 Mode B episodes were identical configurations, correct the v1 counts everywhere in v2 material (e.g. 1/1 per controller, not 3/3).
4. Seed spread: per-seed Mode C results 9/15, 8/15 and 7/15 have overlapping Wilson 95% intervals (about 36–80%, 30–75% and 25–70%); pooled 24/45 = 53% (about 39–67%), assuming 15 distinct configurations per seed. Report pooled results with intervals; never rank seeds.
5. Define the LiDAR metrics precisely: are 0.025 m (RMS) and 0.048 m (mean) per beam or per scan, absolute or signed, over which beams — and why they differ (angular calibration map vs full scan with occlusion). Add MAE, RMSE, p95 and max, per sector.
6. Define Mode A/B/C explicitly in every document.
7. Expand the references from 5 to a verified set (Section 8).
Log every correction in 10_EVIDENCE/V1_CORRECTIONS.md (what was stated → what is correct → evidence).

### PHASE 4 — Dynamic-obstacle extensions (SHOULD)

A separate experiment on L00 only, so motion is the only variable: C1 = the baseline cart (reference), C2 = single-pass linear crossing, C3 = two obstacles crossing with phase offsets. Measure commanded vs logged pose, LiDAR consistency, swept-collision behaviour and reset consistency; run the five controllers if time allows. State the kinematic status explicitly.

### PHASE 5 — Stabilization bracket (SHOULD)

v1 changed the gimbal, wall height, mount height and beam count together, and the gimbal is ideal. Hold walls, mount and beams at their final values and toggle stabilization ON/OFF only. Tilt sweep (supervisor-imposed static attitudes, e.g. 0, ±5, ±10, ±15° roll and pitch) plus one realistic maneuver. Report range error vs |tilt|, false-short returns (floor strikes) and sector-minimum error. Ideal ON vs OFF brackets a real gimbal (best case vs no stabilization) — say so.

### PHASE 6 — Sensor verification extension (SHOULD)

Extend the existing angular map with the four diagonal bearings and a beyond-cap distance (5.5 m) if not already covered; run the analytic ray-caster comparison on all six layouts (per-sector errors; visibility of every new obstacle at the scan plane); verify the index → bearing formula and write it into Sensor_Interface_v2.md.

## 6. DOCUMENTATION AND VISUAL EVIDENCE PROTOCOL — document everything

6.1 Execution log — 10_EVIDENCE/EXECUTION_LOG.md, one entry per step:

    STEP NN — title
    Objective:
    Inputs / assumptions:
    Commands run (exact, PowerShell or Python):
    Files created or changed (+ commit hash or snapshot ID):
    Verification performed:
    Result: PASS / FAIL / PARTIAL — key numbers with units
    Evidence: figure / video / GIF / CSV paths
    Issues, deviations, blockers:
    Wall-clock time / simulated time:

6.2 DECISIONS.md (choice, alternatives, reason); ASSUMPTIONS.md (assumption, reason, risk, how to confirm); V1_CORRECTIONS.md.

6.3 Run folders — runs\<run_id>\ containing manifest.json (run_id, timestamp, commit or snapshot, Webots version, scenario ID + hash, mode, variant, controller + hash, basicTimeStep, observation interval, termination, outcome), trajectory.csv, obstacles.csv, lidar.npz (range images, sectors, pose, ground truth) and events.csv. Run ID format: L02_C_ppo_seed19_v07. CSV columns carry units (t_s, x_m, yaw_rad).

6.4 Figures — generated by script from logged data (tools\make_figures.py); PNG 300 dpi + PDF/SVG; captions in 03_FIGURES\captions.md (what it shows, n, data file, script, commit, provenance).

Core figures for the report (3–5):
- F1 Distribution-shift ladder: L00–L05 at identical scale (metres) — boxes, inflated C-space, A* path, start/goal with tolerance disk, cart lane, scale bar; dissimilarity and difficulty printed on each panel.
- F2 Generalization results: success/collision/timeout per layout for Mode B and Mode C; PPO pooled with Wilson intervals plus per-seed points; direct/reactive as grey reference markers.
- F3 Representative trajectories: PPO on L00 vs L02 (mirror) vs one hard layout, success and failure, coloured by clearance.
- F4 Verification panel: observation Δt, reset error, LiDAR vs ray-caster error distribution, LiDAR–cart consistency over time, box vs circle geometry.
- F5 Sensor configuration (SV1) — or the stabilization bracket if Phase 5 is done.

Extended figures (evidence archive):
- E1 Descriptor table vs L00
- E2 Per-layout maps with corridor-width heatmap
- E3 Success vs dissimilarity and vs difficulty
- E4 Clearance distributions per layout
- E5 LiDAR error vs bearing (unrolled 720-beam strip with sector boundaries), per layout
- E6 Sector–time heatmap during a cart crossing
- E7 Collision probes (face / corner / edge / corridor) and the circle-vs-box false collision
- E8 Δt histogram and altitude hold
- E9 L00 reproduction (v1 vs v2 outcomes)
- E10 Trajectory small multiples (controller × layout)
- E11 Architecture / data-flow diagram: Webots world ↔ supervisor (scenario loader, logger) ↔ drone controller ↔ handoff.py / PPO (Spencer) ↔ LiDAR pipeline (sensor teammate), with the contracts written on the arrows
- E12 Work timeline built from EXECUTION_LOG.md
- E13 Reactive GNRON geometry (after the Phase 3 fix)
- E14 Motion variants C1–C3 (if Phase 4 is done)

Chart rules: one style file; SI units on every axis; equal aspect and a scale bar on maps; colour-blind-safe palette (e.g. Okabe–Ito); no 3D, pie or radar charts; text ≥ 8 pt at print size; the same colour means the same entity everywhere (each controller, the cart, static obstacles).

6.5 Videos — offscreen Camera-node capture (real Webots renders, 1:1 frames, labelled WEBOTS CAPTURE). Add overlays in post-processing (OpenCV or ffmpeg), because Supervisor labels and GUI Optional Rendering do not appear in Camera images. MP4 H.264, yuv420p, 1280×720, 30 fps, CRF ≈ 23, faststart; one JSON sidecar per video (run IDs, provenance, simulated-time span, playback speed).
- V1 Main demo (≤ 2 min, MUST): L00 → new layouts with PPO in Mode B and Mode C; overlay: layout, mode, controller, variant, sim time, outcome.
- V2 LiDAR view: Camera capture side by side with a synchronized LOGGED-DATA RENDER of the rays, the 12 sectors (0/3/6/9 labelled) and the cart.
- V3 Mirror test: L00 vs L02, same controller, side by side.
- V4 Failure reel: one clip per failure type, cause labelled.
- V5 Stabilization ON vs OFF (if Phase 5 is done).

6.6 GIFs — cut from the real MP4s (ffmpeg palettegen/paletteuse or imageio), 6–12 s, 10–15 fps, 720–960 px wide, ≤ 10 MB, looping:
- G1 Shift-ladder montage (L00 → L05 maps)
- G2 Mirror test
- G3 LiDAR sweep + sector heatmap during a cart crossing
- G4 PPO success on a new layout
- G5 PPO failure on a new layout
- G6 Stabilization ON vs OFF (if done)
Embed G1–G3 in README.md.

6.7 Sensor visualization pack — 09_HANDOFFS\sensor_pack\ (named SV so they do not clash with your S-stage IDs):
- SV1 Sensor schematic, top and side views: drone, LiDAR mount position and height, scan plane at 1.0 m, stabilization axes, 360° FOV, 0.5° beam spacing, 5 m cap ring, 12 sectors with indices (0 front, 3 right, 6 back, 9 left) and boundaries, clockwise arrow. Label it SCHEMATIC; dimensions must match the world file.
- SV2 Webots GUI screenshots with LiDAR rays / point cloud visible (Optional Rendering), level and tilted.
- SV3 Unrolled range-image strip vs bearing, with sector boundaries and ray-caster ground truth.
- SV4 Sector–time heatmap during a cart crossing.
- SV5 Sensor-suite table: every device on the drone (LiDAR, GPS, InertialUnit, gyro, compass, Camera) — Webots node, device name, sampling period, output, consumer (PPO / sensor pipeline / logging / video only).

6.8 Step screenshots — before/after each phase in 10_EVIDENCE\screenshots\.

6.9 Visual lab notebook — 10_EVIDENCE\GALLERY.md (optionally a self-contained GALLERY.html): one section per step with a thumbnail, a one-line result and data links.

6.10 QA pass — open every figure, play every video and GIF, render every PDF page to an image and inspect it, run every README command, check every link; log it.

## 7. DELIVERABLE — a new folder next to v1 (v1 untouched)

    D:\Download\drone_navigation_system\DNS_FRIDAY_MEETING_2026-11-27\
      00_START_HERE.md     1 page: what changed since v1, 3 key verified results, 2-command demo, folder map
      README.md            exact setup (Windows, Webots version, Python, packages) and reproduction
      reproduce.py         --fast: scenarios + figures from logs; --full: rerun simulations (+ reproduce.ps1)
      01_REPORT\           Progress_Report_v2.pdf (4–6 pages) + source; v1 report copy (unchanged, hashed)
      02_SLIDES\           ≤ 8-slide PDF + TALKING_POINTS.md
      03_FIGURES\          core\ (F1–F5), extended\ (E1–E14), captions.md
      04_VIDEOS\           V1_main_demo.mp4, extended\ (V2–V5), sidecars
      05_GIFS\             G1–G6
      06_DATA\             results_v2.csv, scenario_table.csv, asset_library.csv, verification CSVs, runs\
      07_SCENARIOS\        generator_config.yaml, L00–L05 JSON + hashes, worlds or loader
      08_WEBOTS_PROJECT\   runnable worlds, PROTOs, controllers, tools, tests
      09_HANDOFFS\         PPO_Interface_v2.md, Sensor_Interface_v2.md, two_frames.json,
                           lidar_calibration.json, contracts\, sensor_pack\, lidar_gt_dataset\
      10_EVIDENCE\         TASK_PROMPT.md, AUDIT_SUMMARY.md, EXECUTION_LOG.md, DECISIONS.md,
                           ASSUMPTIONS.md, V1_CORRECTIONS.md, Verification_Report_v2.md,
                           CLAIMS_EVIDENCE.csv, GALLERY.md, BASELINE_HASHES.txt, ENVIRONMENT.txt,
                           REPRO_CHECK.md, screenshots\, failed_runs\

Verification_Report_v2.md = the 15 v1 checks + new ones: generator determinism, overlaps, keep-outs, scan-plane height, world ↔ scenario hash, cart t = 0 consistency, contract tests on L01–L05, A* feasibility, reference traversal, L00 reproduction, LiDAR–cart consistency.

## 8. REPORT — Progress_Report_v2.pdf (4–6 pages excluding references)

The 11-page v1 report remains the full technical record. v2 is a concise progress report on what changed and what it shows (a full report v2 is a COULD).

Working title: "Design and Verification of a Reproducible Webots Simulation Environment for UAV Navigation Among Static and Dynamic Obstacles". Use "verification", not "validation".

Sections:
1. Scope and contribution — my role; v1 recap in five lines.
2. Platform justification — why Webots (built-in LiDAR model, supervisor API for scenario control and ground truth, physics-based flight, fixed-step timing, open source), with a brief cited comparison with Gazebo, AirSim, gym-pybullet-drones and Flightmare.
3. Environment and scenario generator — assets, L00, the shift ladder, validators, feasibility.
4. Verification — timing, reset, geometry, LiDAR vs ray caster, LiDAR–cart consistency, L00 reproduction, and the v1 defects found and fixed.
5. Spatial-generalization pilot — protocol, results with intervals, mirror test, failure taxonomy; exploratory.
6. Team integration — what Spencer and the sensor teammate can use now.
7. Limitations and threats to validity — the v1 list, what v2 resolved and what remains.
8. Next steps — 3–5 items.

Style: natural, precise academic English; past tense; first person for my work ("I designed…", "I verified…"); units and n on every number; no "novel", "robust", "state-of-the-art" or "guarantees" without measurement; every figure cited in the text; self-contained captions.

References — verify each before citing (target 12–15 in v2):
- closest: Ji, Bamdad & Cruz, "SWIFT-Nav: Stability-Aware Waypoint-Level TD3 with Fuzzy Arbitration for UAV Navigation in Cluttered Environments", ACRA 2025, arXiv:2512.16027 (Webots, DJI Mavic 2 Pro, LiDAR-based safety) — position my work as complementary infrastructure (seeded held-out layouts, dynamic-obstacle verification, documented sensor contract), not as a critique;
- Kalidas et al., Drones 7(4):245, 2023 (DRL UAV navigation with stationary and mobile obstacles);
- generalization: Cobbe et al., ICML 2019 and ICML 2020; Kirk et al., JAIR 2023; domain randomization: Tobin et al., IROS 2017;
- PPO: Schulman et al., arXiv:1707.06347; Stable-Baselines3: Raffin et al., JMLR 2021 (if the checkpoints are SB3, as PPO.load suggests);
- Webots: Michel, International Journal of Advanced Robotic Systems 1(1), 2004, doi:10.5772/5618;
- V&V: Sargent, Journal of Simulation 7(1), 2013; sim-to-real: Zhao, Peña Queralta & Westerlund, IEEE SSCI 2020;
- reporting: Henderson et al., AAAI 2018; Agarwal et al., NeurIPS 2021; Wilson, JASA 1927;
- determinism: Chance et al., arXiv:2104.06262;
- planning and reactive baselines: Hart, Nilsson & Raphael 1968 (A*); Lozano-Pérez 1983 (configuration space); Khatib, IJRR 1986 (potential fields); Ge & Cui, IEEE T-RA 2000 (GNRON);
- simulators, only if discussed: Koenig & Howard, IROS 2004; Shah et al., FSR 2017; Panerati et al., IROS 2021; Song et al., CoRL 2020.

## 9. HANDOFFS v2 (new versions; the v1 originals stay unchanged)

PPO_Interface_v2.md — for Spencer: scenario selection by ID without editing handoff.py; the scenario list, descriptors and TEST-ONLY flags; one-command evaluation of any checkpoint on the layout × mode matrix; metric and termination definitions; results_v2 summary; questions (more seeds? training on L01–L05 would require a fresh held-out set).

Sensor_Interface_v2.md — for the sensor teammate: the verified index → bearing formula; 720 beams via getHorizontalResolution(); sector boundaries with 0/3/6/9; cap and inf handling; scan plane at 1.0 m; stabilization behaviour (ideal); error statistics per sector; a ground-truth-labelled LiDAR dataset (lidar_gt_dataset\: range images, sectors, pose and ray-caster ground truth for all six layouts, static and cart crossings) for offline testing without touching their code; open questions.

## 10. MEETING MATERIALS

Slides (≤ 8, verified figures only):
1. Contribution
2. Environment + scenario generator (F1, G1)
3. Verification, including the v1 defects found and fixed (F4)
4. Sensor (SV1, V2)
5. Generalization results (F2)
6. Mirror test + trajectories (F3, G2)
7. Team integration
8. Limitations and next steps

TALKING_POINTS.md — a 60-second script with verified numbers only, plus evidence-linked answers to:
- How do you know the LiDAR readings and sector indices are correct?
- The cart error is zero — isn't that by construction? (Yes; here is the real dynamic evidence.)
- How do you guarantee every layout is solvable?
- Are three seeds and 15 episodes enough to compare anything?
- Why do direct and reactive score 0?
- Is the ideal gimbal realistic?
- What did verification catch? (the four v1 defects)
- What can Spencer and the sensor teammate do with this today?

## 11. TIMELINE (my local time) — hard freeze Thursday night

- Checkpoint A — Wed 25 Nov, midday: Phase 0 done; runtime projection; episode-variant definitions.
- Checkpoint B — Wed 25 Nov, night: L01–L05 generated, validated, feasible and frozen; protocol committed; Phase 2 running.
- Checkpoint C — Thu 26 Nov, afternoon: Phase 2 results; Phase 3 corrections; SHOULD items as time allows.
- Checkpoint D — Thu 26 Nov, 22:00: figures, videos, GIFs, report, slides, handoffs, packaging → FREEZE.
- Fri 27 Nov, morning: clean-copy reproduction following the README exactly → REPRO_CHECK.md; final CLAIMS_EVIDENCE.csv check.

At each checkpoint, send me a ≤ 10-line status (done / running / blocked / next) plus one key figure.

Priority if time runs short:
- MUST: Phase 0; Phase 1 (L01–L05 + validators + feasibility + reference traversal); Phase 2 (at least all PPO cells); Phase 3; F1–F4; V1; SV1; Progress_Report_v2; TALKING_POINTS; both handoffs v2; Verification_Report_v2; CLAIMS_EVIDENCE.csv.
- SHOULD: direct/reactive on all layouts; Phases 4–6; V2–V4; GIFs; slides; LiDAR GT dataset.
- COULD: extra seeds per family; randomized start/goal set; full report v2; GALLERY.html; V5.
A smaller verified package beats a larger unverified one.

## 12. DEFINITION OF DONE (each true, or marked not done with the reason)

- [ ] v1, snapshot, handoff.py and checkpoint hashes unchanged
- [ ] The 15 v1 checks pass; the L00 rerun matches results.csv (or differences are explained)
- [ ] Episode-variant definitions documented; v1 counts corrected if repeats were identical
- [ ] L01–L05: validators pass, A* feasible, same-seed determinism, reference traversal reported, contract tests pass, frozen before any controller ran
- [ ] S6 experiment run per the pre-registered protocol; Wilson intervals; mirror test reported; failure taxonomy
- [ ] V1_CORRECTIONS.md covers the reactive numbers, zero-error reframing, distinctness, seed spread, metric definitions, mode definitions and references
- [ ] LiDAR–cart consistency and the cart physics-velocity check reported
- [ ] Core figures, V1 and SV1 produced from real data with provenance labels
- [ ] Progress report 4–6 pages with verified references; talking points; both handoffs v2; Verification_Report_v2
- [ ] CLAIMS_EVIDENCE.csv covers every number in the report, slides, README and talking points
- [ ] Clean-copy reproduction passes (or the exact failure is documented); QA pass logged; failed runs kept

## 13. YOUR FINAL RESPONSE TO ME

1. Status table per phase — Done / Partial / Blocked, with evidence paths.
2. Key verified results — ≤ 10 bullets: value, unit, n, interval where relevant, file path.
3. v1 corrections applied — before → after.
4. Changes vs v1 — files added; hashes confirmed unchanged.
5. Failures and limitations — an honest list.
6. Blockers and exact requests — for Spencer, the sensor teammate, or me.
7. 60-second meeting script — verified numbers only.
8. Folder tree with file sizes.

Start now with Phase 0. Work autonomously; ask me only if a decision is irreversible or a contract conflict blocks progress. Otherwise take the most conservative option consistent with the contracts, record it in ASSUMPTIONS.md, and continue.
