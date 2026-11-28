# Benchmark paper: Wang, Zhou & Wu, IEEE TVT (2026)

The team's benchmark, chosen on the supervisor's direction on 28 Nov 2026.

> [!IMPORTANT]
> **Reading status.** The bibliographic facts and the abstract-level summary below are verified. **The full text has not been read yet:** IEEE Xplore is paywalled and ResearchGate offers no public full text. Every cell marked **[PDF]** must be filled from the paper itself before we state any comparison. Until then, section 5 lists gap *hypotheses*, not findings.

## 1. Citation

H. Wang, F. Zhou and Q. Wu, "A Robust and Intelligent Obstacle Avoidance Method for Uncrewed Aerial Vehicles Under the Complex and Dynamic Environment," *IEEE Transactions on Vehicular Technology*, vol. 75, no. 3, pp. 4019–4031, Mar. 2026 (published online 2025; IEEE Xplore document 11180066).

## 2. What the paper claims (abstract level, verified)

| | Summary (paraphrased) |
|---|---|
| **Setting** | UAVs working in confined spaces among static objects and moving pedestrians. The authors argue that UAV control there still relies on human pilots, and that existing dynamic-avoidance schemes adapt poorly. |
| **Idea** | Obstacle avoidance built on *active perception of environment uncertainty*. |
| **Method** | An environment-perception network lets the drone perceive and encode the dynamic scene. Reinforcement learning then avoids pedestrians despite uncertain observations. |
| **Evidence** | Simulation experiments. The method is reported to beat existing computational and deep-learning approaches in dynamic scenarios, and to exceed human testers. |

Sources: the IEEE Xplore abstract; TRID record 01997668.

**Indirect hint, to verify:** a 2026 preprint (UCON, arXiv:2609.29419) lists this paper among vision-based methods that are sensitive to motion blur and illumination. If that is right, the benchmark's input is a camera or depth image, not LiDAR.

## 3. Extraction sheet [PDF]

Fill one row per item, with the page or section, when the PDF is available. Our column is already verified.

| Item | Benchmark [PDF] | Page | Ours (verified in this repository) |
|---|---|---|---|
| Sensor and observation | | | 360° LiDAR: 720 beams, 5 m cap, one plane at 1.14 m, binned into 12 sectors; observation = 52 floats (two frames) |
| Uncertainty model | | | none yet (zero-noise LiDAR, ideal gimbal) |
| Perception network | | | none; hand-crafted sector binning (`navigation.pack_scan`) |
| RL algorithm, network size | | | PPO, from-scratch NumPy (Spencer); deterministic `tanh(actor(obs))` at test time |
| Action space and rate | | | 2 floats in [−1, 1] × 1.2 m/s per axis, every 0.2 s; altitude held at 1.0 m |
| Reward | | | Spencer's training reward; not documented in this repository yet (WP-PPO deliverable) |
| Simulator and vehicle dynamics | | | Webots R2025a; rigid-body quadrotor; 8 ms physics; 125 Hz attitude/altitude loop |
| Scene size and static obstacles | | | 10 × 6 m aisle; 2 racks, 2 columns, 1 crate stack (exact boxes) |
| Dynamic obstacles: number, speed, motion model | | | 1 scripted cart, 0.45 m/s, ±1.9 m, triangle wave, 15 phases |
| Start and goal | | | fixed: (0.85, 0) → (9.15, 0), success within 0.4 m |
| Training budget, seeds | | | 3 PPO checkpoints (seeds 7, 19, 43) |
| Evaluation episodes, statistics | | | 540 episodes (480 distinct); Wilson 95 % intervals; no significance tests |
| Metrics | | | success / collision (static, cart, wall) / timeout; path efficiency vs A\*; min clearance; near misses (< 0.30 m) |
| Baselines | | | 2 reference controllers (direct, reactive); traditional A\*/RRT being ported (`baselines/traditional/`) |
| Human-tester protocol | | | — |
| Generalisation tests | | | 6 seeded layouts incl. an exact mirror (IoU 0.074 with training) |
| Ablations | | | — |
| Real-world tests | | | none (simulation only) |
| Code or data release | | | private repository; scenarios hashed; every number traced in `evidence/CLAIMS_EVIDENCE.csv` |
| Stated limitations | | | see the README, *Limitations* |

## 4. Our setting compared with the benchmark

| Dimension | Benchmark | Ours today | Planned ([research plan](../RESEARCH_PLAN.md)) |
|---|---|---|---|
| Input | perception network over the sensed scene; possibly vision [PDF] | 360° 2D LiDAR | LiDAR only: richer encoding (more bins, temporal stack) and an explicit LiDAR uncertainty model |
| Environment | simulation [PDF: which] | Webots, full rigid-body dynamics | Webots for evaluation; a fast surrogate for training, with an equivalence check against Webots |
| Moving obstacles | pedestrians [PDF] | one scripted cart | several movers (carts, pedestrian-like bodies); scripted for evaluation, stochastic for training |
| Start and goal | [PDF] | fixed | randomised per episode under feasibility rules; held-out pairs for testing |
| Learning method | RL [PDF] | PPO | improved PPO (WP-PPO) |
| Evidence standard | [PDF] | verified world, reproducibility checks, intervals | the same standard, applied to every new experiment |

## 5. Candidate gaps: hypotheses until the PDF is read

For each, check what the paper actually does. Keep a hypothesis only if the paper confirms the gap.

| # | Hypothesis | Check in the paper | What we would do, and how we would measure it |
|---|---|---|---|
| H1 | The input is vision with a limited field of view, sensitive to light and motion blur | sensor model, field of view | 360° LiDAR, no blind side, independent of lighting. Measure success when movers approach from the side or from behind. |
| H2 | Start and goal are fixed, or drawn from a narrow set | scenario generation | Randomise start and goal per episode and test on held-out pairs. Report success on unseen pairs. |
| H3 | Pedestrian motion is simple or narrow | motion model, speed range | Sweep speed, count and motion model, including out-of-distribution speeds. Plot success against speed and count. |
| H4 | The vehicle is a point mass or uses simplified dynamics | dynamics model | Webots rigid-body quadrotor with an inner loop. Measure the gap between the surrogate and Webots. |
| H5 | The uncertainty model is generic, not sensor-specific | how uncertainty is injected | LiDAR range noise, dropout, body-tilt and occlusion. Plot success against noise level. |
| H6 | Statistical reporting is thin (few episodes or seeds, no intervals) | number of episodes and seeds, error bars | At least 5 training seeds, Wilson 95 % intervals, episode counts stated. |
| H7 | Test scenes resemble the training scenes | train/test split | Overlap-controlled held-out layouts (IoU), a mirror test and harder layouts. |
| H8 | Failures are not separated from impossible scenes | solvability checks | A reference traversal confirms each test scene is passable (as for L05), and unsolvable ones are excluded (as for L04). |
| H9 | Code and scenes are not released | availability statement | Hashed scenarios, per-run manifests, every claim traced to a file. |
| H10 | The perception network is costly on board | network size, inference time | Report policy inference time; the current observation is 52 numbers. |
| H11 | No real-flight validation | experiments section | **A limit we share.** State it; do not claim real-world performance. |

## 6. How we will compare fairly

Numbers from a different simulator, sensor and scene **cannot** be ranked against ours. There are two honest options. We choose after the PDF is read; this is decision gate D1 in the plan.

1. **Protocol-matched.** Rebuild the benchmark's scenario types and metrics in Webots with LiDAR input. Compare all of our methods under that protocol. Quote the benchmark's published numbers beside ours as context, not as a head-to-head result.
2. **Re-implementation.** If the paper gives enough detail, re-implement its method in our environment, adapting its input to LiDAR, and compare head-to-head with intervals. Any adaptation is reported as a change to their method.

Either way, we will not claim "better than the benchmark" unless both methods ran in the same environment under the same protocol.
