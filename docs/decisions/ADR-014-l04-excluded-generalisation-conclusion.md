# ADR-014 — L04 excluded from the generalisation conclusion

**Status:** Proposed — awaiting the supervisor's decision

## Context
All five controllers score 0/90 on L04. Before that can be read as a
generalisation result, the layout has to be shown flyable by this airframe
within the 90-step horizon.

## Options considered
1. Report 0/90 as a generalisation result.
2. Regenerate L04 until it behaves.
3. Report the data in full, mark traversability unconfirmed, and exclude it from
   the conclusion.

## Decision
**Option 3.** Option 2 would alter a layout after seeing its outcomes, which
breaks the pre-registration. Option 1 would claim something the evidence does
not support.

## Evidence
L04 passes every generator validator and is A*-feasible with an en-route
corridor of 0.850 m. The flown reference traversal reaches the goal at no tested
setting: at a 0.22 m planning margin and <= 0.48 m/s the tracker is stable
(peak tilt 0.105 rad) and collision-free but **times out at 90 steps**; at
higher speeds it collides near x = 3.1 m, which is where all three PPO
checkpoints also collide. At a 0.12 m margin the airframe tumbled to 1.5905 rad
against 0.45 rad of tilt authority. The controllers themselves do **not**
destabilise: 0 of 90 L04 episodes exceeded 0.50 rad.

## Consequences
+ The generalisation claim rests only on layouts whose traversability was
  confirmed. L05, where the reference traversal reached the goal in 56 steps, is
  reported as a **valid** 0/45 failure.
− One rung of the ladder yields no conclusion, and the question of whether L04
  is flyable at all is still open.

**Decision needed from Dr. Mehdi:** keep L04 excluded, or authorise a
diagnostic-only rerun with a longer horizon (which would sit outside the frozen
contract and so could not feed the main result)?

---

**Related:** ADR-009, ADR-011 · [ADR index](README.md) ·
[decision log](DECISIONS_LOG.md)
