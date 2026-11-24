# ADR-012 — One source of geometry, checked against the loaded world

**Status:** Accepted

## Context
Two v1 defects shared a root cause. The calibration tool loaded a fixed scenario
and so compared one world's measurements against another world's geometry; and
the world writer placed the moving cart at its motion centre rather than its
t = 0 pose. Both were silent: every API call succeeded and plausible numbers
came out.

## Options considered
1. Careful review and a naming convention.
2. Make the scenario JSON the single source of geometry, and verify at run time
   that the loaded world implements it.

## Decision
**Single source plus a run-time check.** Every world, validator, ray caster,
figure and run reads geometry from the scenario JSON by id. `dns/worldcheck.py`
then asks the running simulator where its bodies are and **refuses to produce
results** if any obstacle's t = 0 pose, the time step, the observation interval
or the LiDAR beams/FOV/cap disagree with the scenario.

## Evidence
Maximum obstacle t = 0 pose error over all six layouts: **<= 4.0e-06 m**
(`results/layout_gate/SUMMARY.json`). Each world also carries the SHA-256 of the
scenario that produced it, and the experiment client asserts it before the first
episode.

## Consequences
+ A world that has drifted from its definition fails loudly instead of quietly
  producing misattributed results.
− Every geometry change means regenerating worlds; hand-editing a `.wbt` is no
  longer supported.

---

**Related:** ADR-004, ADR-006 · [ADR index](README.md) ·
[decision log](DECISIONS_LOG.md)
