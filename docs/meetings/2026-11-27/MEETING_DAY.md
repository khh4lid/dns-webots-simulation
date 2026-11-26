# Meeting day — Friday 27 November 2026

Presenter: **Khaled** (@khh4lid) · Supervisor: (name withheld) ·
Topic: the DNS simulation environment

---

## Open these, in this order

| # | File | Why |
|---|---|---|
| 1 | `docs/meetings/2026-11-27/DNS_SimEnv_SIMPLE_2026-11-27.pptx` | **Present from this.** 12 picture-first slides, ~10 minutes, speaker notes on every one. Videos embedded on slides 3 and 10. |
| 2 | `docs/meetings/2026-11-27/DNS_SimEnv_SIMPLE_2026-11-27.pdf` | The same deck as a PDF. Use it if PowerPoint misbehaves — nothing in the argument depends on motion. |
| 3 | `docs/meetings/2026-11-27/DNS_SimEnv_2026-11-27.pdf` | **The detailed deck — backup only.** 19 slides with the full numbers. Open it when a question needs more than the simple deck carries. |
| 4 | `assets/gifs/` | Fallback if PowerPoint will not play the MP4s. `V-TOP_L00_success.gif`, `V-SIDE_L00_altitude.gif`, `V-TOP_L02_mirror.gif`, `V-TOP_L05_failure.gif` — any browser plays these. |
| 5 | The repository README, **on the branch** | `https://github.com/khh4lid/dns-webots-simulation/tree/v2-transparency-release` — open it before the meeting so it is already loaded. |
| 6 | `docs/verification/RESET_DETERMINISM.md` | Have it open in a tab. The independence question lands here. |

Check before you walk in: the simple PDF opens, slides 3 and 10 play in the
simple PPTX, and the README page has actually rendered (images loaded, GIFs
animating).

> **Which deck when.** Open the **simple** deck and talk to it. The **detailed**
> deck is where you go *after* a question — it has the verification table, the
> per-layout descriptors and the reset comparison. Do not present from both.

**Arabic walkthrough:** `TALKING_POINTS_SIMPLE_AR.md` — one short explanation
per slide plus the English sentence to say.

### Simple-deck slide map

| Slide | Point |
|---|---|
| 1–2 | Who I am, what I own, and where my part sits in the team loop |
| 3 | The task: start to goal, LiDAR only |
| 4–9 | Six design choices — what I chose, what I did not, and why |
| 10 | The result: mirror map equal, hardest map zero |
| 11 | Four checks, and six defects found (five fixed) |
| 12 | Done / not yet / what I need |
| B1–B4 | All choices · verification numbers · the reset finding · why L04 is out |

---

## The first 30 seconds

> "My part of the project is the simulation environment — the Webots world the
> team's drone flies in, and the checks that say whether it can be believed.
>
> Version 1 had one obstacle layout, so it could not tell navigation from
> memorising one room. Version 2 adds a seeded generator and six layouts and
> runs the same protocol across all of them: 540 episodes.
>
> I want to show you two results and ask for four decisions."

Then go to slide 2. Do not open with the methodology.

---

## The four decisions to ask for

| # | Ask | Who | Backup slide |
|---|---|---|---|
| 1 | **L04: keep it excluded, or authorise a longer-horizon diagnostic?** My own reference tracker cannot fly it either, so 0/90 there is not attributable. | the supervisor | simple **B4** |
| 2 | **Approve a full re-evaluation under the fixed reset as v2.1?** The fix is written and on its own branch; it needs 540 episodes and a re-analysis. | the supervisor | simple **B3** |
| 3 | **More PPO training seeds.** Three cannot separate checkpoint variance from layout effects. | Spencer | simple slide 12 |
| 4 | **The 0.0536 m vs 0.62 m sensor reading.** Unexplained, and repulsion scales with `1.45 − range`. | sensor teammate | — |

Ask for all four explicitly. If only one gets answered, make it **1**, because
it is the one blocking a result.

---

## If something fails

| Failure | Do this |
|---|---|
| Video will not play | Present the simple PDF — the poster frames are already there. Say "the full clip is in the release; here is the frame." |
| PowerPoint missing or broken | The simple PDF is complete. Nothing in the argument depends on motion. |
| Projector dies | Talk simple slides 2, 3, 10 and 12 from `TALKING_POINTS_SIMPLE_AR.md`. The mirror result and the reset finding both survive being spoken. |
| Asked for a number not on a slide | `evidence/CLAIMS_EVIDENCE.csv` — claim → file → script. Slide numbers are traced in `evidence/SLIDES_SIMPLE_CLAIMS_CHECK.csv` (simple deck) and `evidence/SLIDES_CLAIMS_CHECK.csv` (detailed), both 0 unmatched. |
| Asked to show it running | `python scripts/with_webots.py --boot-wait 12 --timeout 400 --world worlds/DNS_L00_C.wbt tests/client_capture.py -- --layout L00 --mode C --controller ppo_seed7 --variant 0` (~1 min). |
| Asked why the PR is not merged | It is open for review; `gh` was not installed, so it goes through the browser. `GITHUB_WEB_STEPS.md` has the steps. Nothing is blocked on it. |
| Running short on time | Simple deck **5-minute path**: slides 1, 2, 3, 10, 12. |

---

## Say / do not say

**Say:** "reproduced exactly when the session is replayed in order" · "no
evidence of left/right layout memorisation **in this test**" · "verification in
simulation" · "the intervals are approximate because the episodes are coupled".

**Do not say:** "the policy generalises" · "exactly reproducible" without the
qualifier · "validated" for anything in simulation · any ranking of the three
checkpoints · that the reset fix is in this build.

---

## One thing to be ready for

If the supervisor asks *"so are your numbers wrong?"* — the answer is no, and be
precise about why: the 540 episodes were all measured under one reset, in a
fixed order, and that order reproduces to 5 µm. What the reset finding changes
is the **independence** assumption behind the intervals, not the measurements.
The deterministic re-run of all six layouts moved the pooled rate by at most two
episodes in 45 and left the mirror result intact — L00 and L02 are equal under
both resets. That comparison is `docs/verification/RESET_MODE_COMPARISON.md`.
