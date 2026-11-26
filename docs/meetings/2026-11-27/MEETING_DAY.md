# Meeting day — Friday 27 November 2026

Presenter: **Khaled** (@khh4lid) · Supervisor: (name withheld) ·
Topic: the DNS simulation environment

---

## Open these, in this order

| # | File | Why |
|---|---|---|
| 1 | `docs/meetings/2026-11-27/DNS_SimEnv_FINAL_2026-11-27.pptx` | **Present from this.** 16 slides, ~12 minutes, a flight-path tracker along the bottom, number cards, speaker notes on every slide. Clips embedded on slides 2, 11 and 12. |
| 2 | `docs/meetings/2026-11-27/DNS_SimEnv_FINAL_2026-11-27.pdf` | The same deck as a PDF. Use it if PowerPoint misbehaves — the poster frames are already in place and nothing in the argument depends on motion. |
| 3 | `docs/meetings/2026-11-27/NUMBERS_CHEATSHEET.pdf` | **Print this.** One A4 page: every number in the deck, grouped by its type, with its unit, meaning and source file. |
| 4 | `docs/meetings/2026-11-27/DNS_SimEnv_SIMPLE_2026-11-27.pdf` | Backup deck — the shorter picture-first version. |
| 5 | `docs/meetings/2026-11-27/DNS_SimEnv_2026-11-27.pdf` | Backup deck — the detailed 19-slide version with the full numbers. |
| 6 | `assets/gifs/` | Fallback if PowerPoint will not play the MP4s. Any browser plays these. |
| 7 | The repository README, **on the branch** | `https://github.com/khh4lid/dns-webots-simulation/tree/v2-transparency-release` — open it before the meeting so it is already loaded. |
| 8 | `docs/verification/RESET_DETERMINISM.md` | Have it open in a tab. The independence question lands here. |

Check before you walk in: the final PDF opens, slides 2, 11 and 12 play in the
final PPTX, and the README page has actually rendered.

> **Which deck when.** Present the **final** deck. The simple and detailed
> decks are backups for questions. Do not present from more than one.

**Arabic walkthrough:** `TALKING_POINTS_FINAL_AR.md` — per slide: what it
means, what each number on it represents, and the English sentence to say.

### Final-deck slide map

| Slide | Point | Tracker |
|---|---|---|
| 1 | Cover | at Start |
| 2 | The mission: Start to Goal, nothing touched | Mission |
| 3 | My role in the team loop | Mission |
| 4 | The map, drawn to scale | Map |
| 5 | Heights: everything is taller than the scan | Map |
| 6 | The drone itself | Drone |
| 7 | 720 beams, read as 12 directions | Sensors |
| 8 | Four obstacle types, each with a job | Obstacles |
| 9 | Six choices, and what I didn't choose | Choices |
| 10 | Six test maps, with their narrowest gaps | Tests |
| 11 | The three ways a flight can end | Tests |
| 12 | Results, with the mirror pair inset | Results |
| 14 | Four checks | Checks |
| 15 | Done / not yet / what I need | Next |
| 16 | Close — on GitHub | at Goal |
| B1–B6 | choices in full · reset finding · why Clutter is out · full results · verification detail · the timing ladder |  |

Slide 13 of the storyboard (the mirror comparison) is merged into slide 12 as
its inset, so the deck is 15 rendered main slides covering 16 beats; the
tracker numbering still reads 1 to 16.


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
| 1 | **Clutter map (L04): keep it excluded, or authorise a longer-horizon diagnostic?** My own reference tracker cannot fly it either, so 0/90 there is not attributable. | the supervisor | final **B3** |
| 2 | **Approve a full re-evaluation under the fixed reset as v2.1?** The fix is written and on its own branch; it needs 540 episodes and a re-analysis. | the supervisor | final **B2** |
| 3 | **More PPO training seeds.** Three cannot separate checkpoint variance from layout effects. | Spencer | final slide 15 |
| 4 | **The 0.0536 m vs 0.62 m sensor reading.** Unexplained, and repulsion scales with `1.45 − range`. | sensor teammate | — |

Ask for all four explicitly. If only one gets answered, make it **1**, because
it is the one blocking a result.

---

## If something fails

| Failure | Do this |
|---|---|
| Video will not play | Present the final PDF — the poster frames are already there. Say "the full clip is in the release; here is the frame." |
| PowerPoint missing or broken | The final PDF is complete. Nothing in the argument depends on motion. |
| Projector dies | Talk final slides 2, 4, 10 and 12 from `TALKING_POINTS_FINAL_AR.md`. The mirror result and the reset finding both survive being spoken. |
| Asked for a number not on a slide | `evidence/CLAIMS_EVIDENCE.csv` — claim → file → script. Slide numbers are traced in `evidence/SLIDES_FINAL_CLAIMS_CHECK.csv`, `SLIDES_SIMPLE_CLAIMS_CHECK.csv` and `SLIDES_CLAIMS_CHECK.csv`, all 0 unmatched. `NUMBERS_CHEATSHEET.pdf` has every number on one page. |
| Asked to show it running | `python scripts/with_webots.py --boot-wait 12 --timeout 400 --world worlds/DNS_L00_C.wbt tests/client_capture.py -- --layout L00 --mode C --controller ppo_seed7 --variant 0` (~1 min). |
| Asked why the PR is not merged | It is open for review; `gh` was not installed, so it goes through the browser. `GITHUB_WEB_STEPS.md` has the steps. Nothing is blocked on it. |
| Running short on time | Final deck **5-minute path**: slides 1, 2, 4, 7, 10, 12, 15. |

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
