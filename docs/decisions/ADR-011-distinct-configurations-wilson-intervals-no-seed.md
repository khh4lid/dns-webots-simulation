# ADR-011 — Distinct configurations, Wilson intervals, no seed ranking

**Status:** Accepted

## Context
v1 reported Mode B as "3/3" per controller. Those three episodes are the same
configuration run three times: the episode seed is accepted by
`WebotsBackend.reset` and never used, there is no start jitter, and
`PPO.predict` is `tanh(actor(obs))` with no sampling.

## Options considered
1. Report episode counts as the sample size.
2. Count distinct documented configurations, and treat identical deterministic
   repeats as one.
3. Add start jitter so the three Mode B episodes become genuinely distinct.

## Decision
**Count distinct configurations.** Mode B is n = 1 per controller; Mode C is
n = 15 (fifteen distinct cart phases). Option 3 was rejected because the
contract freezes the start point, and breaking a frozen contract to improve a
sample size is the wrong trade.

Rates are pooled over the three checkpoints with Wilson 95% intervals. Per-seed
values are plotted but never ranked, and no significance test is reported.

## Evidence
`results/v2_S6/v1_distinctness.json` lists, per cell, how many distinct
configurations the episodes represent and the path-length spread across the
repeats (<= 0.0065 m for PPO). Pooled rates and intervals:
`results/v2_S6/pooled_rates.csv`.

## Consequences
+ Every reported rate states the n it is over, and 540 episodes are reported as
  480 distinct configurations.
− Mode B claims become much weaker than v1 implied: L00 Mode B pooled is 3/3
  with a Wilson interval of [0.439, 1.000].

---

**Related:** ADR-009 · [ADR index](README.md) ·
[decision log](DECISIONS_LOG.md)
