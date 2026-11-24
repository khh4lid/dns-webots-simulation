# ADR-001 — Webots R2025a as the simulator

**Status:** Accepted

## Context
The environment had to provide a planar LiDAR, ground-truth obstacle poses for
scoring, physics-based quadrotor flight, a fixed and exactly divisible time
step, and had to run on the Windows laptop this project is developed on. It
also had to be driveable from an external Python process, because the policy
side is Spencer's code and must not be edited to fit the simulator.

## Options considered
| Simulator | LiDAR model | Ground-truth / scene control | Quadrotor physics | Windows | Licence |
|---|---|---|---|---|---|
| **Webots R2025a** | built-in `Lidar` node, configurable beams/FOV/range | `Supervisor` API reads and writes any node pose | built-in `Propeller` thrust model; samples ship with the install | first-class native build | Apache 2.0 |
| Gazebo (Ignition/Harmonic) | built-in ray sensors | services and plugins | via plugins such as PX4 SITL | ROS-centric; Windows support is partial | Apache 2.0 |
| AirSim | built-in LiDAR | API over RPC | Unreal-based, high fidelity | yes | MIT; **archived by Microsoft, succeeded by Project AirSim** |
| gym-pybullet-drones | no built-in LiDAR (ray casts must be written) | direct PyBullet state access | quadrotor-specific, RL-first | yes | MIT |
| Isaac Sim | built-in RTX LiDAR | USD + Python API | yes | yes | proprietary; **requires an RTX GPU and a large install** |

Sources are listed in the repository references; each was checked against the
project's own documentation or repository rather than from memory.

## Decision
**Webots R2025a.** It is the only option in the table that supplied all four of
the things this contribution needs — a built-in configurable LiDAR, a
supervisor API for both scene control and ground truth, physics-based rotor
flight, and a native Windows build — without additional middleware.

The deciding criterion was the *supervisor API*. Every validator, the analytic
ray-caster comparison, the swept collision test and the world-vs-scenario check
all need to read and write exact body poses from outside the controller. That
is a first-class Webots feature, where in the alternatives it is either a
plugin layer or hand-written.

## Evidence
Determinism, which the whole experiment rests on, was measured rather than
assumed: re-running the v1 protocol on the regenerated world reproduced all ten
cells with a **maximum per-episode path difference of 0 m**
(`results/v2_S6/v1_reproduction.csv`, figure `E9_l00_reproduction.png`).

This is a **selection rationale, not a benchmark**: no competing simulator was
installed, instrumented or timed for this project, and no performance
comparison is claimed.

## Consequences
+ One dependency, no ROS, no GPU requirement; the world opens offline.
+ `Supervisor` gives exact ground truth for scoring without touching the sensor path.
− Webots' own movie recorder does not work in this environment (see ADR-013).
− Tied to Webots' rotor model; a different flight model would need re-tuning.

---

**Related:** ADR-008, ADR-012, ADR-013 · [ADR index](README.md) ·
[decision log](DECISIONS_LOG.md)
