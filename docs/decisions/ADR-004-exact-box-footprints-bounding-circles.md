# ADR-004 — Exact box footprints instead of bounding circles

**Status:** Accepted

## Context
v1 modelled the rectangular storage racks as circles for both the collision test
and the ray cast. A 0.8 x 2.4 m rack has a bounding radius of 1.265 m, so the
circle covers large volumes of empty aisle.

## Options considered
1. Keep circles — simpler maths, one closed-form segment test.
2. Exact convex footprints: box distance, slab ray intersection, swept test.

## Decision
**Exact footprints.** `PlacedObstacle.surface_distance` uses the exact box
distance, `ray_distance` uses the slab method in the box's own frame, and the
collision test sweeps the drone's motion with a ternary search on the convex
distance function.

## Evidence
The defect this fixes is reproduced as a regression test: a drone at (0.9, 1.1)
beside a rack centred at the origin has a **true surface gap of 0.50 m**, while
the bounding circle of radius 1.265 m calls it a collision
(`tests/test_generator.py::box surface distance is exact, not a bounding
circle`; panel (d) of `F4_verification_panel.png`).

## Consequences
+ Collision statistics mean what they say.
+ The same geometry code serves the collision test, the ray caster and the
  feasibility checker, so a layout cannot be feasible under one model and
  collided under another (ADR-012).
− Each collision test costs more than a closed-form circle check.

---

**Related:** ADR-012 · [ADR index](README.md) ·
[decision log](DECISIONS_LOG.md)
