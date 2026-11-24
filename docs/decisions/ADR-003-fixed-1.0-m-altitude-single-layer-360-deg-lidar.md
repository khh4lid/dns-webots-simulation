# ADR-003 — Fixed 1.0 m altitude and a single-layer 360 deg LiDAR

**Status:** Accepted

## Context
Spencer's observation contract is planar: 12 sectors, fixed yaw, no altitude
term. The environment has to match that or the policy is given information it
cannot represent.

## Options considered
1. Full 3-D navigation with a multi-layer LiDAR.
2. Planar navigation at a fixed altitude with one scan layer.

## Decision
**Planar, cruise altitude 1.0 m, one scan layer at 1.14 m** (1.0 m cruise plus
a 0.14 m mount). Every obstacle must rise above the scan plane with margin, so
no body can be solid to the collision test and invisible to the sensor — the
worst possible combination, because the policy would be punished for hitting
something it was never shown.

## Evidence
Enforced twice: `Scenario._check_obstacle_visibility` rejects any obstacle at
or below the scan plane plus margin, and the generator's `scan_plane_height`
validator rejects the same case at draw time
(`tests/test_generator.py::validator rejects an obstacle below the scan plane`).

## Consequences
+ The sensor sees everything the collision test can hit.
− No vertical avoidance; flying over an obstacle is not a strategy the
  environment can express. Listed as a limitation.

---

**Related:** ADR-007 · [ADR index](README.md) ·
[decision log](DECISIONS_LOG.md)
