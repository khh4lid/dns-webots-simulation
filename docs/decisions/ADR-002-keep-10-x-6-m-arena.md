# ADR-002 — Keep the 10 x 6 m arena

**Status:** Accepted

## Context
A larger hall would make the navigation problem more interesting. But Spencer's
PPO checkpoints were fitted in `planar.py` on exactly `LOW=[0,-3]`,
`HIGH=[10,3]`, and `navigation.observation` divides the relative goal vector by
10 while capping LiDAR range at 5 m.

## Options considered
1. Enlarge to 12 x 8 m for a harder task.
2. Keep 10 x 6 m and vary only obstacle placement.
3. Randomise the arena size per layout.

## Decision
**Keep 10 x 6 m in every layout L00-L05.** Enlarging the room would move the
goal feature and the free-space statistics outside the distribution the
checkpoints saw, so any difference between layouts would be confounded by the
arena change rather than caused by the obstacle layout.

## Evidence
Asserted for every layout by `tests/test_generator.py::every layout keeps the
frozen L00 envelope`, which compares arena bounds, start, goal, tolerance,
altitude, time step, speed cap and the full LiDAR configuration against L00.

## Consequences
+ Layout is the single independent variable across the ladder.
− A larger-hall study is a separate, legitimate experiment that this release
  does not run, and would have to be reported as out-of-distribution.

---

**Related:** ADR-009 · [ADR index](README.md) ·
[decision log](DECISIONS_LOG.md)
