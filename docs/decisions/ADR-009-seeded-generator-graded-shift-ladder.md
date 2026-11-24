# ADR-009 — A seeded generator and a graded shift ladder

**Status:** Accepted

## Context
v1 measured the policies on one layout, so it could not distinguish navigation
competence from layout-specific memorisation. Testing on new layouts requires
those layouts to be reproducible, validated and frozen before any controller
sees them.

## Options considered
1. Hand-author a few new worlds in the Webots editor.
2. A config-plus-seed generator emitting canonical scenario JSON, with the
   world written from the JSON.

## Decision
**Generator.** `config/scenarios/generator_config.yaml` plus a seed produces a
byte-identical scenario JSON, hashed, from which the world is written. Six
layouts form a graded ladder: L00 baseline, L01 perturbed, L02 mirrored,
L03 aisle shift, L04 clutter, L05 hard.

## Evidence
Byte-identity is tested **across separate processes**, which specifically guards
against seeding from Python's per-process-randomised `hash()`
(`tests/test_generator.py`). Measured en-route corridor widths across the
ladder: **2.08 / 1.87 / 2.08 / 1.31 / 0.85 / 0.68 m**. All six are A*-feasible and all block
the straight line.

## Consequences
+ Any layout can be regenerated from the config and checked against its hash.
+ L01-L05 are flagged TEST-ONLY; training on them would require a fresh
  held-out set.
− Six layouts with one seed per family is a pilot, not a distribution: it is far
  too few to fit a trend, and no trend is fitted.

---

**Related:** ADR-010, ADR-011, ADR-012 · [ADR index](README.md) ·
[decision log](DECISIONS_LOG.md)
