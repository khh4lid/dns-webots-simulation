# Simulation environment — dossier

What I was responsible for, what I delivered, and where the evidence is.

**Owner:** TBC (@khh4lid) ·
**Supervisor:** Dr. Mehdi Samieiyeganeh

My contribution is the **simulation environment**. I do not own the PPO policy
(Spencer) or raw sensor processing (the perception teammate), and neither is in
this repository.

| # | Responsibility | Delivered | Evidence | Status |
|---|---|---|---|---|
| 1 | **Simulator selection and setup** | Webots R2025a, extern-controller harness, world generated from config, offline-loadable | [ADR-001](../decisions/ADR-001-webots-r2025a-simulator.md), [ADR-013](../decisions/ADR-013-offscreen-camera-capture-built-in-recorder.md), `evidence/ENVIRONMENT.txt` | Done |
| 2 | **Navigation environment design** | 10 × 6 m arena, 2.5 m walls, 1.0 m cruise, cascaded autopilot at 125 Hz over an 8 ms step, 5 Hz command interface | [ADR-002](../decisions/ADR-002-keep-10-x-6-m-arena.md), [ADR-008](../decisions/ADR-008-5-hz-velocity-commands-125-hz-loop-8-ms-step.md) | Done |
| 3 | **Start and goal definition** | Start (0.85, 0.00), goal (9.15, 0.00), tolerance 0.4 m, identical in all six layouts and asserted per layout | contract tests, `tests/test_generator.py` | Done |
| 4 | **Static obstacle development** | Exact box footprints (not circles), asset library, seeded generator, six validated layouts, A* feasibility and difficulty descriptors | [ADR-004](../decisions/ADR-004-exact-box-footprints-bounding-circles.md), [ADR-009](../decisions/ADR-009-seeded-generator-graded-shift-ladder.md), `results/v2_S6/scenario_table.csv` | Done |
| 5 | **Moving obstacle modelling and validation** | Kinematic cart, ±1.9 m at 0.45 m/s, period 16.8889 s, swept collision test, LiDAR–cart consistency vs an independent ray caster | [ADR-006](../decisions/ADR-006-cart-is-kinematic.md), `results/v2_S6/lidar_cart_consistency.json` | Done, with a stated limitation (no physical contact response) |

## Beyond the five — what v2 added

- A **pre-registered** generalisation protocol, committed before any controller
  ran on L01–L05: [S6_GENERALIZATION_PROTOCOL.md](../verification/S6_GENERALIZATION_PROTOCOL.md)
- **540 episodes / 480 distinct configurations** across six layouts
- **Nine corrections to v1**, each with evidence: [V1_CORRECTIONS.md](../verification/V1_CORRECTIONS.md)
- A **blocking gate** — world-vs-scenario check, 26 contract tests, A*
  feasibility and a flown reference traversal — that a layout must pass before
  it can contribute a result

## What teammates can reuse immediately

| You want | Take | Guide |
|---|---|---|
| To score a checkpoint on six layouts | `tests/client_s6_experiment.py`, `scripts/run_s6.py` | [FOR_SPENCER](FOR_SPENCER.md) |
| Ground-truth-labelled LiDAR frames | `results/v2_S6/runs/*/lidar.npz` | [FOR_SENSOR_TEAM](FOR_SENSOR_TEAM.md) |
| A new held-out layout family | `config/scenarios/generator_config.yaml` + `python -m dns.generator` | [ADR-009](../decisions/ADR-009-seeded-generator-graded-shift-ladder.md) |
| To check a claim | `evidence/CLAIMS_EVIDENCE.csv` | — |

## What I would not claim

The environment **exposes** policy behaviour; it does not explain it.
Policy-level conclusions are Spencer's. `direct` and `reactive` are untuned
references from his code and are never used to rank methods. Everything here is
**verification in simulation** — never real-world validation.
