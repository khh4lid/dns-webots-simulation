# Architecture decision records

Why the simulation environment is built the way it is. Each record
states the context, the options that were on the table, what was
decided, the evidence behind it and what it costs.

| ADR | Decision | Status |
|---|---|---|
| [ADR-001](ADR-001-webots-r2025a-simulator.md) | Webots R2025a as the simulator | Accepted |
| [ADR-002](ADR-002-keep-10-x-6-m-arena.md) | Keep the 10 x 6 m arena | Accepted |
| [ADR-003](ADR-003-fixed-1.0-m-altitude-single-layer-360-deg-lidar.md) | Fixed 1.0 m altitude and a single-layer 360 deg LiDAR | Accepted |
| [ADR-004](ADR-004-exact-box-footprints-bounding-circles.md) | Exact box footprints instead of bounding circles | Accepted |
| [ADR-005](ADR-005-racks-deliberately-cross-straight-start-goal-lin.md) | The racks deliberately cross the straight start-goal line | Accepted |
| [ADR-006](ADR-006-cart-is-kinematic.md) | The cart is kinematic | Accepted with a stated limitation |
| [ADR-007](ADR-007-gimbal-stabilised-lidar-mount.md) | Gimbal-stabilised LiDAR mount | Accepted with a stated limitation |
| [ADR-008](ADR-008-5-hz-velocity-commands-125-hz-loop-8-ms-step.md) | 5 Hz velocity commands over a 125 Hz loop and an 8 ms step | Accepted |
| [ADR-009](ADR-009-seeded-generator-graded-shift-ladder.md) | A seeded generator and a graded shift ladder | Accepted |
| [ADR-010](ADR-010-mirror-test-memorisation-probe.md) | The mirror test as a memorisation probe | Accepted |
| [ADR-011](ADR-011-distinct-configurations-wilson-intervals-no-seed.md) | Distinct configurations, Wilson intervals, no seed ranking | Accepted |
| [ADR-012](ADR-012-one-source-geometry-checked-against-loaded-world.md) | One source of geometry, checked against the loaded world | Accepted |
| [ADR-013](ADR-013-offscreen-camera-capture-built-in-recorder.md) | Offscreen Camera capture instead of the built-in recorder | Accepted |
| [ADR-014](ADR-014-l04-excluded-generalisation-conclusion.md) | L04 excluded from the generalisation conclusion | Proposed — awaiting the supervisor's decision |

The running decision log with the smaller day-to-day choices is
[DECISIONS_LOG.md](DECISIONS_LOG.md); the assumption register is
[../verification/ASSUMPTIONS.md](../verification/ASSUMPTIONS.md).
