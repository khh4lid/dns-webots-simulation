# Presenter kit — 27 November 2026

**Presenter:** NAME NEEDED · **Supervisor:** Dr. Mehdi Samieiyeganeh ·
**Deck:** `DNS_SimEnv_2026-11-27.pdf` / `.pptx` (12 slides + 6 backups)

> [!WARNING]
> The deck still carries visible **NAME NEEDED** placeholders — mine, Spencer's
> and the sensor teammate's. Fill them in `scripts/make_slides.py` (`PRESENTER`)
> and `scripts/make_team_docs.py` (`PEOPLE`), then re-run both, before this is
> treated as final.

---

## 60-second version

> I built the simulation environment for the DNS project — the Webots world the
> team's drone flies in, and the checks that say whether it can be believed.
>
> Version 1 had one obstacle layout, so it could not tell navigation from
> memorising one room. Version 2 adds a seeded generator and six layouts, and
> runs the same protocol across all of them: 540 episodes, 480 distinct
> configurations.
>
> Two results matter. On a layout that is an exact mirror of the training one —
> maximally different geometry, identical difficulty by construction — the
> policies score exactly what they score on the original, 24 of 45 both times.
> That is no evidence of left/right memorisation. But on the hardest layout,
> which my own reference tracker proved is flyable, they score zero of 45.
>
> I also corrected four things from v1, including my own headline: what I
> reported as 3 of 3 was really one configuration run three times.
>
> I need three decisions — one each from you, Spencer and the sensor teammate.

## 3-minute version

1. **Role (20 s).** The environment is mine: simulator choice, arena, start and
   goal, static obstacles, the moving cart. The PPO policy and the sensor
   processing are teammates' and are not in my repository.
2. **The task (20 s).** Cross a 10 × 6 m aisle at 1.0 m, sensing only through a
   360° LiDAR binned into 12 sectors, one velocity command every 0.2 s, 90-step
   horizon.
3. **Why Webots (20 s).** The supervisor API is the deciding feature — every
   validator and all ground-truth scoring needs exact body poses from outside
   the controller. Selection rationale, not a benchmark.
4. **Verification (40 s).** Timing exact to 3e−16 s, reset 2.8 µm, world-vs-
   scenario ≤ 4.0e−06 m, LiDAR vs an independent ray caster. It caught five
   real defects, all silent — the worst modelled rectangular racks as circles
   and reported collisions half a metre into empty aisle.
5. **The ladder (30 s).** Six layouts from one config and a seed, frozen before
   anything ran, corridor widths 2.08 down to 0.68 m, all A*-feasible.
6. **Results (40 s).** L01 and L03 overlap the baseline — I cannot distinguish
   them and do not claim to. L05 is 0/45 on a layout proven flyable. L04 is
   excluded because my reference tracker could not fly it either.
7. **The mirror (20 s).** IoU 0.074, 3.00 m mean displacement, identical A*
   length and corridor width. Same result. Narrow claim.
8. **Honesty and asks (30 s).** Four corrections, six limitations, three
   decisions.

---

## Timing — 12 minutes

| Slide | Content | Target | Running |
|---|---|---|---|
| 1 | Title | 0:30 | 0:30 |
| 2 | My role and what is not mine | 0:50 | 1:20 |
| 3 | The task — top and side views | 1:00 | 2:20 |
| 4 | Obstacle catalogue | 0:50 | 3:10 |
| 5 | Why Webots | 1:00 | 4:10 |
| 6 | The LiDAR contract | 1:00 | 5:10 |
| 7 | Verification and the five defects | 1:20 | 6:30 |
| 8 | The six-layout ladder | 0:50 | 7:20 |
| 9 | Results with intervals | 1:30 | 8:50 |
| 10 | The mirror test | 1:10 | 10:00 |
| 11 | Corrections and limitations | 1:00 | 11:00 |
| 12 | Three decisions | 1:00 | 12:00 |

Leave the slide-9 and slide-10 numbers on screen while answering — most
questions land there.

---

## Q&A, with the evidence

**How do you know the LiDAR readings and the sector indices are correct?**
The index→bearing map was *measured*, not assumed: a search over sweep
direction and angular origin fitted against an analytic ray cast from a known
pose. The sector convention is re-tested per layout by putting a single
synthetic beam on each cardinal bearing and asserting which sector it lands in —
26/26 contract tests on all six layouts.
→ `docs/interfaces/Sensor_Interface_v2.md`, backup B6.

**The cart error is zero — isn't that by construction?**
Yes, and that was my mistake in v1. The cart is driven kinematically from a
function of time, and I compared its pose against that same function. It is now
labelled an implementation-consistency check. The real evidence is the measured
sector range against an independent ray caster at the logged cart pose: **MAE
0.0193 m over 650 comparisons**.
→ `results/v2_S6/lidar_cart_consistency.json`, correction C2.

**How do you know every layout is solvable?**
Three independent gates: A* feasibility for a disc of the drone's radius; the
measured widest-path corridor width; and a *flown* reference traversal that
tracks the A* route through Spencer's own command interface. Five of six reach
the goal. The sixth, L04, does not — which is exactly why it is excluded.
→ `results/layout_gate/SUMMARY.json`, backup B4.

**Are three seeds and fifteen episodes enough to compare anything?**
Not to rank, which is why I never do. On L00 Mode C the three checkpoints give
9/15, 8/15 and 7/15 — intervals that overlap almost completely. I report the
pooled rate with a Wilson interval and plot the per-seed points without
ordering them. More seeds is my ask of Spencer.
→ `results/v2_S6/pooled_rates.csv`, ADR-011.

**Why do `direct` and `reactive` score 0?**
`direct` flies straight at the goal, and both racks cross that line by design —
it hits `rack_a` at x ≈ 2.98 m every time. `reactive` is a potential field;
v1 explained its failure by comparing the 1.45 m repulsion radius against the
crate's *centre* distance, 1.54 m. The controller reacts to the nearest LiDAR
return — the *surface* — at 1.0582 m. The measured cause is cumulative
repulsion from the racks and walls, peak magnitude 1.8654 against a unit
attractive term, timing out 1.6167 m short. Both are untuned references from
Spencer's code and I never use them to rank methods.
→ backup B3, correction C1.

**Is the ideal gimbal realistic?**
No — it is a best case. It makes the flat 12-sector contract actually true,
which matches the 2-D world the policies were trained in. An ideal mount and a
body-fixed one bracket a real gimbal; I have not yet run that bracket.
→ ADR-007.

**What did verification catch?**
Five defects, all silent. Racks collided as circles (false collisions 0.50 m
into clear aisle); the camera generator rotated about the vertical instead of
tilting, so every frame was empty sky; the world file wrote the cart at its
sweep centre instead of its t = 0 pose, a 1.9 m error that corrupted a
calibration reference by 1.34 m; the calibration tool loaded a different world
than it measured; and v1 shipped a protocol file describing a different run
than its own results CSV.
→ slide 7, `docs/verification/V1_CORRECTIONS.md`.

**Why is L04 excluded?**
All five controllers score 0/90 — but so does the reference tracker, which is
*given* the correct route. At a 0.22 m planning margin and ≤ 0.48 m/s it is
stable (peak tilt 0.105 rad) and collision-free but times out at the 90-step
horizon; faster, it clips the same crate the PPO checkpoints hit. So I cannot
separate "the policies fail" from "this layout may be outside the airframe's
envelope within the horizon". The data is kept in full.
→ backup B4, ADR-014. **This is decision 1 for you.**

**What can Spencer and the sensor teammate do with this today?**
Spencer: one command evaluates any checkpoint on all six layouts
(`python scripts/run_s6.py`), with L01–L05 flagged TEST-ONLY. The sensor
teammate: 60 runs of ground-truth-labelled LiDAR frames in
`results/v2_S6/runs/*/lidar.npz`, plus the verified index→bearing formula and a
reproducible anomaly to investigate.
→ `docs/team/FOR_SPENCER.md`, `docs/team/FOR_SENSOR_TEAM.md`.

---

## If something breaks

| Problem | Do this |
|---|---|
| Video will not play in PowerPoint | Present the **PDF** — every video slide keeps its poster frame. The GIFs in `assets/gifs/` also play in any browser. |
| No PowerPoint at all | Open `DNS_SimEnv_2026-11-27.pdf` — it is complete and self-contained. |
| The PR is not merged yet | Present from the branch: `github.com/khh4lid/dns-webots-simulation/tree/v2-transparency-release`. |
| Asked for a number not on a slide | `evidence/CLAIMS_EVIDENCE.csv` — every claim maps to its data file and script. |
| Asked to show it running live | `python scripts/with_webots.py --boot-wait 12 --timeout 400 --world worlds/DNS_L00_C.wbt tests/client_capture.py -- --layout L00 --mode C --controller ppo_seed7 --variant 0` (~1 min). |

## Do not say

- "the policy generalises" — say *no evidence of left/right memorisation in
  this test*, and pair it with L05
- "proves", "robust", "state-of-the-art"
- "validated" for anything in simulation — it is **verification**
- any ranking of the three checkpoints
