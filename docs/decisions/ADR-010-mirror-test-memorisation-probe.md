# ADR-010 — The mirror test as a memorisation probe

**Status:** Accepted

## Context
A success-rate drop on a new layout has two possible causes that a single
comparison cannot separate: the layout is harder, or the policy was keyed to the
old geometry. Separating them needs a layout that is maximally *different* while
being exactly as *hard*.

## Options considered
1. Compare against a randomly perturbed layout and reason about difficulty.
2. Construct an exact reflection, whose difficulty is identical by construction.

## Decision
**L02 is L00 reflected about y = 0**: y -> -y, yaw -> -yaw, and the cart's
crossing phase shifted by pi. The phase shift is used rather than a negative
amplitude because the triangle-wave offset satisfies f(u + 1/2) = -f(u)
exactly, so a half-period shift reflects the whole trajectory, whereas a
negative amplitude would corrupt the period formula (period = 4A/v).

## Evidence
L02 has IoU **0.074** with L00 and a mean obstacle
displacement of **3.00 m**, yet identical A* length
(9.004 m), tortuosity (1.085), en-route corridor width (2.080 m) and intrusion
depth (0.200 m) — asserted by `tests/test_generator.py::L02 matches L00 on every
difficulty descriptor`. Visual proof that it was flown mirrored:
`assets/figures/E15_mirror_overlay.png` and `assets/gifs/mirror_l00_vs_l02.gif`.

## Consequences
+ An L00-vs-L02 difference cannot be a difficulty effect.
− The test probes **left/right layout identity only**. It is not a general
  generalisation result, and is always reported alongside L05, where the same
  checkpoints score 0/45 on a layout whose traversability was confirmed.

---

**Related:** ADR-009, ADR-011 · [ADR index](README.md) ·
[decision log](DECISIONS_LOG.md)
