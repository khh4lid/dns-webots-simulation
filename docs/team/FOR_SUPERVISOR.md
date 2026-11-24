# For the supervisor — one page

**Student:** TBC (@khh4lid) ·
**Supervisor:** Dr. Mehdi Samieiyeganeh ·
**Contribution:** the Webots simulation environment

> [!NOTE]
> Names marked **TBC** were not supplied when this release was generated. They
> are placeholders, not omissions — tell me the spellings and I will fill them in.

## What was done since v1

v1 built and verified one obstacle layout and ran 90 episodes on it. It could
not tell navigation competence apart from layout-specific memorisation, because
there was only one layout.

v2 adds a **seeded scenario generator** and a **graded distribution-shift
ladder** of six layouts, then runs the same protocol across all of them:
**540 episodes = 480 distinct configurations**
(the difference is explained below). Every layout is validated, A*-feasible and
frozen with a SHA-256 before any controller sees it.

## Key results, with intervals

| Layout | What changes | IoU vs L00 | Corridor (m) | PPO Mode C pooled [Wilson 95%] | Reference traversal |
|---|---|---|---|---|---|
| **L00** | the training layout | 1.000 | 2.08 | 24/45 = 0.533 [0.391, 0.671] | goal, 50 steps |
| **L01** | positions ±0.3 m, yaw ±10° | 0.589 | 1.87 | 20/45 = 0.444 [0.309, 0.588] | goal, 50 steps |
| **L02** | reflected about y = 0 | 0.074 | 2.08 | 24/45 = 0.533 [0.391, 0.671] | goal, 51 steps |
| **L03** | racks moved along x, sides swapped | 0.093 | 1.31 | 20/45 = 0.444 [0.309, 0.588] | goal, 52 steps |
| **L04** | extra crate stacks | 0.135 | 0.85 | 0/45 = 0.000 [0.000, 0.079] | **not reached** — excluded |
| **L05** | narrow gate + occluded goal | 0.141 | 0.68 | 0/45 = 0.000 [0.000, 0.079] | goal, 56 steps |

*PPO pooled over the three checkpoints; n = 45 distinct cart phases per layout
in Mode C. Per-seed values are plotted but never ranked — their intervals
overlap. Source: `results/v2_S6/pooled_rates.csv`.*

Three things worth your attention:

1. **The mirror test (L00 vs L02).** L02 is an exact reflection of L00:
   maximally different geometry (IoU 0.074,
   mean obstacle displacement
   3.00 m) with
   *identical* difficulty by construction — same A* length, tortuosity,
   corridor width and intrusion depth. The pooled result is the same on both.
   I phrase this narrowly: **no evidence of left/right layout memorisation in
   this test.** It is not a claim that the policy generalises.

2. **L05 is a genuine failure.** 0/45, on a layout whose traversability was
   confirmed by a flown reference traversal (goal in 56 steps). This is the
   counterweight to the mirror result and should be read alongside it.

3. **The L00 reproduction.** Re-running v1's protocol reproduced all ten cells
   exactly, with a maximum per-episode path difference of **0 m**. That is the
   determinism evidence everything else rests on.

## Decisions I need from you

| # | Question | Why it matters |
|---|---|---|
| 1 | **L04: keep it excluded, or authorise a diagnostic rerun with a longer horizon?** | All five controllers score 0/90 there, but the reference traversal never reached the goal either — stable and collision-free at ≤ 0.48 m/s but timing out at the 90-step horizon, colliding above that. I cannot separate "the policies fail" from "this layout may not be flyable within the horizon", so I excluded it. A longer-horizon rerun would sit outside the frozen contract and could not feed the main result. See [ADR-014](../decisions/ADR-014-l04-excluded-generalisation-conclusion.md). |
| 2 | **Licence for this repository** | Currently *not decided*. The repository is private; no licence file has been added, deliberately — it is your call. |
| 3 | **Is three training seeds enough to report?** | Three checkpoints cannot separate checkpoint-to-checkpoint variance from layout effects. I report pooled rates with Wilson intervals and never rank seeds, but the precision is limited by n. |

## What I corrected from v1

Nine corrections, each with before → after → evidence, in
[V1_CORRECTIONS.md](../verification/V1_CORRECTIONS.md). The two that change
reported numbers:

- **v1's Mode B "3/3" is really 1/1.** The episode seed is accepted but never
  used, so the three episodes are one configuration run three times. Pooled
  Mode B on L00 is 3/3 with a Wilson interval of [0.439, 1.000] — a far weaker
  claim than "3/3" suggested.
- **The reactive explanation was arithmetically wrong.** It compared the 1.45 m
  repulsion radius against the crate's *centre* distance (1.54 m); the
  controller reacts to the *surface*, which is 1.0582 m away. The conclusion
  survives, the reasoning did not.

## Honest limitations

- Physical contact response with the moving cart is **not** modelled — a
  `collision_cart` outcome means the swept footprints intersected.
- The LiDAR gimbal is **ideal**, which is a best case, not a real sensor.
- Six layouts, one seed per family: a pilot, not a distribution. No trend is
  fitted and none should be read into the scatter.
- One unexplained sensor reading: 0.0536 m measured where ground-truth
  clearance was 0.62 m. Logged as an open issue, not swept up.

Full list: [ASSUMPTIONS.md](../verification/ASSUMPTIONS.md).
