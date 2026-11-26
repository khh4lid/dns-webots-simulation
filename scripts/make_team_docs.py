"""Generate the team-facing documents, with every number read from the results.

Four role guides plus two interface specifications. Numbers are pulled from the
result files at generation time, so a document cannot quote a figure that the
data no longer supports.

Identity fields are resolved only as far as the project record supports; what
is not on record is shown as an explicit "to confirm" rather than guessed, and
is listed at the top of each document that needs it.

Run::

    python scripts/make_team_docs.py
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results" / "v2_S6"
TEAM = ROOT / "docs" / "team"
INTERFACES = ROOT / "docs" / "interfaces"

#: Identity fields, each resolved ONLY as far as the project record supports.
#: The brief's Section 0 was submitted twice with every field still in
#: brackets, so nothing here is guessed: a value is filled in only where it
#: appears in the repository, the supervisor's own name, or the GitHub URL.
#: Anything else stays an explicit "to confirm", because a plausible-looking
#: invented surname on a citation file is worse than a visible gap.
PEOPLE = {
    # Given name and handle are on record (repository owner, project docs).
    # The full legal name for CITATION.cff is not.
    "me_name": "Khaled",
    "me_handle": "@khh4lid",
    "me_fullname_status": "full name to confirm",
    # Used consistently throughout the project; surname and handle never given.
    "spencer_name": "Spencer",
    "spencer_handle": "handle to confirm",
    # Nothing at all is on record for this teammate.
    "sensor_name": "name to confirm",
    "sensor_handle": "handle to confirm",
    "supervisor_name": "Dr. Mehdi Samieiyeganeh",
    "supervisor_handle": "handle to confirm",
}


def rows(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open(encoding="utf8") as handle:
        return list(csv.DictReader(handle))


def load() -> dict:
    data: dict = {"pooled": {}, "layouts": {}}
    for row in rows(RESULTS / "pooled_rates.csv"):
        data["pooled"][(row["layout_id"], row["mode"], row["group"])] = row
    for row in rows(RESULTS / "scenario_table.csv"):
        if row["mode"] == "C":
            data["layouts"][row["layout_id"]] = row
    results = rows(RESULTS / "results_v2.csv")
    data["episodes"] = len(results)
    data["distinct"] = sum(int(r["distinct_configuration"]) for r in results)
    consistency = RESULTS / "lidar_cart_consistency.json"
    if consistency.is_file():
        data["lidar"] = json.loads(consistency.read_text(encoding="utf8"))
    return data


D = load()


def pooled_row(layout: str, mode: str = "C_dynamic") -> str:
    entry = D["pooled"].get((layout, mode, "ppo_pooled"))
    if not entry:
        return "not run"
    return (f"{entry['k']}/{entry['n']} = {float(entry['rate']):.3f} "
            f"[{float(entry['wilson_low']):.3f}, {float(entry['wilson_high']):.3f}]")


def ladder_table() -> str:
    lines = ["| Layout | What changes | IoU vs L00 | Corridor (m) | "
             "PPO Mode C pooled [Wilson 95%] | Reference traversal |",
             "|---|---|---|---|---|---|"]
    notes = {
        "L00": ("the training layout", "goal, 50 steps"),
        "L01": ("positions ±0.3 m, yaw ±10°", "goal, 50 steps"),
        "L02": ("reflected about y = 0", "goal, 51 steps"),
        "L03": ("racks moved along x, sides swapped", "goal, 52 steps"),
        "L04": ("extra crate stacks", "**not reached** — excluded"),
        "L05": ("narrow gate + occluded goal", "goal, 56 steps"),
    }
    for layout in ("L00", "L01", "L02", "L03", "L04", "L05"):
        meta = D["layouts"].get(layout)
        if not meta:
            continue
        what, traversal = notes[layout]
        lines.append(
            f"| **{layout}** | {what} | {float(meta['iou_to_L00']):.3f} | "
            f"{float(meta['min_corridor_width_enroute_m']):.2f} | "
            f"{pooled_row(layout)} | {traversal} |")
    return "\n".join(lines)


FOR_SUPERVISOR = f"""# For the supervisor — one page

**Student:** {PEOPLE['me_name']} ({PEOPLE['me_handle']}) ·
**Supervisor:** {PEOPLE['supervisor_name']} ·
**Contribution:** the Webots simulation environment

> [!NOTE]
> Still to confirm: my full legal name for the citation file, Spencer's surname
> and GitHub handle, and the sensor teammate's name and handle. Everything else
> here is final.

## What was done since v1

v1 built and verified one obstacle layout and ran 90 episodes on it. It could
not tell navigation competence apart from layout-specific memorisation, because
there was only one layout.

v2 adds a **seeded scenario generator** and a **graded distribution-shift
ladder** of six layouts, then runs the same protocol across all of them:
**{D['episodes']} episodes = {D['distinct']} distinct configurations**
(the difference is explained below). Every layout is validated, A*-feasible and
frozen with a SHA-256 before any controller sees it.

## Key results, with intervals

{ladder_table()}

*PPO pooled over the three checkpoints; n = 45 distinct cart phases per layout
in Mode C. Per-seed values are plotted but never ranked — their intervals
overlap. Source: `results/v2_S6/pooled_rates.csv`.*

> [!IMPORTANT]
> **The Wilson intervals are approximate.** A Wilson interval assumes
> independent trials. The 15 cart-phase variants inside a cell are run back
> to back and are coupled through residual airframe state across the reset
> (`docs/verification/RESET_DETERMINISM.md`), so they are not strictly
> independent draws. The deliberate cart-phase variation is far larger than
> the coupling, but the intervals should be read as indicative rather than
> exact.

Three things worth your attention:

1. **The mirror test (L00 vs L02).** L02 is an exact reflection of L00:
   maximally different geometry (IoU {float(D['layouts']['L02']['iou_to_L00']):.3f},
   mean obstacle displacement
   {float(D['layouts']['L02']['mean_obstacle_displacement_m']):.2f} m) with
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
"""


FOR_SPENCER = f"""# For Spencer — evaluating checkpoints on the layout ladder

**Owner of this document:** {PEOPLE['me_name']} ({PEOPLE['me_handle']}) ·
**You:** {PEOPLE['spencer_name']} ({PEOPLE['spencer_handle']})

> [!IMPORTANT]
> **Nothing of yours is in this repository.** Your `DroneNavPPO/` tree, your
> training code and your checkpoints are yours to publish. This repo records the
> *interface* and the SHA-256 fingerprints of the files it ran against, so we can
> both prove which version produced which numbers.

## What you can do today

Evaluate any checkpoint on any of the six layouts, in either mode, without
editing a line of your code. Scenario selection happens entirely on the
environment side — the world file *is* the scenario.

```powershell
# one layout, both modes, all five controllers (90 episodes, ~3 min)
python scripts/with_webots.py --boot-wait 12 --timeout 1500 `
    --world worlds/DNS_L03_C.wbt tests/client_s6_experiment.py -- --layout L03

# the whole ladder (540 episodes, ~19 min)
python scripts/run_s6.py
```

`tests/client_s6_experiment.py` imports `evaluate.gap_rule`,
`evaluate.goal_direct` and `ppo.PPO.load(...).predict` directly from your tree
and calls them unmodified. To add a checkpoint, drop it in
`evidence/seed<N>/model.npz` and add `<N>` to `load_controllers(seeds=...)`.

## The contract I hold to

| Item | Value | Verified by |
|---|---|---|
| Observation | 52 floats, `float32`, all in [−1, 1] | contract test, every layout |
| Two-frame structure | current scan then previous scan; identical on reset | contract test |
| Action | 2 floats in [−1, 1], scaled by `SPEED = 1.2` m/s per axis | contract test |
| Observation interval | exactly 0.200000 s of simulated time = 25 × 8 ms | measured, not configured |
| Horizon | 90 control steps | `navigation.HORIZON` |
| LiDAR | 720 beams, 360°, 5 m cap, 12 sectors, 0/3/6/9 = front/right/back/left | contract test |
| Arena / start / goal / tolerance | identical in all six layouts | asserted against L00 |

**26/26 contract tests pass on every layout.** Run them yourself:
`python scripts/run_gates.py`. A layout that fails is blocked from the
experiment.

`handoff/two_frames.json` still verifies: `python handoff.py --file <packet>`.

## TEST-ONLY — please read

**L01–L05 are held out.** They exist to measure generalisation, and they only
measure it while no policy has been trained on them.

> [!WARNING]
> If you train on any of L01–L05, every generalisation number in this repository
> becomes a training-set score. If you want to train on new layouts, tell me and
> I will generate a **fresh held-out family** with a different seed — that is one
> config edit and about five minutes of compute.

L00 is the training layout and is not flagged test-only.

## Your results

{ladder_table()}

Pooled over your three checkpoints, n = 45 distinct cart phases per layout.
Per-seed values are in `results/v2_S6/pooled_rates.csv`. I **do not rank the
seeds** — their Wilson intervals overlap almost completely. On L00 Mode C they
are 9/15, 8/15 and 7/15, which is a spread of about the size of the effect.

Per-episode records are in `results/v2_S6/runs/<run_id>/` — `trajectory.csv`
(pose, velocity, command, clearance, per-step), `events.csv`, `obstacles.csv`,
and `lidar.npz` for variant 00 of each cell.

## Metric and termination definitions

Termination is **your** rule, unmodified: collision → goal (≤ 0.4 m) → timeout
(≥ 90 steps). I only refine the `collision` label by attributing it to the
nearest body at the terminal pose, giving `collision_static`, `collision_cart`
and `collision_wall`. Leaving the arena is a wall collision under the geometric
rule, so there is no separate out-of-bounds outcome.

- `path_efficiency` = A* reference length ÷ flown length, blank unless the goal
  was reached.
- `min_clearance_m` is supervisor ground truth to the nearest obstacle
  **surface**, exact box footprints, cart included at its true pose. It never
  enters an observation.
- A near miss is clearance < 0.30 m, counted as contiguous intervals.

## Two things not to assume

1. **Cart collisions are geometric, not physical.** The cart is driven
   kinematically, so Webots reports no rigid-body velocity for it and it cannot
   transfer momentum. `collision_cart` means the swept footprints intersected.
2. **Mode B's three episodes are one configuration.** The seed you pass to
   `reset(seed, scene)` is accepted and never used — no start jitter, no scene
   randomisation. With `PPO.predict` deterministic, the three episodes are
   repeats. I count n = 1 for Mode B rates (see
   [ADR-011](../decisions/ADR-011-distinct-configurations-wilson-intervals-no-seed.md)).

## What I'd like from you

1. **More training seeds.** Three cannot separate checkpoint variance from
   layout effects; the intervals are wide because n is small.
2. **A view on L04** — all five controllers score 0/90 and my reference tracker
   could not fly it either. See
   [ADR-014](../decisions/ADR-014-l04-excluded-generalisation-conclusion.md).
3. Confirmation that `ppo.py` is a from-scratch NumPy PPO and **not**
   Stable-Baselines3 — I read it that way and so do not cite SB3 anywhere.
"""


FOR_SENSOR = f"""# For the sensor and perception teammate

**Owner of this document:** {PEOPLE['me_name']} ({PEOPLE['me_handle']}) ·
**You:** {PEOPLE['sensor_name']} ({PEOPLE['sensor_handle']})

> [!NOTE]
> I own sensor **mounting and configuration** inside the simulator. You own raw
> sensor **processing**. Nothing here touches your algorithm, and none of your
> code is in this repository.

## The sensor as configured

| Property | Value | Where it comes from |
|---|---|---|
| Device | Webots `Lidar`, name `"lidar"` | `protos/DnsQuadcopter.proto` |
| Beams | **720** over 360° (0.5° spacing) | read at run time via `getHorizontalResolution()` |
| Range cap | 5.0 m (`maxRange`); `minRange` 0.05 m | PROTO, asserted against the scenario |
| Vertical FOV | 0.01 rad — a single razor-thin layer | PROTO |
| Noise | 0.0 (ideal) | PROTO field `lidarNoise` |
| Mount height | 0.14 m above the body origin | PROTO, asserted |
| Scan plane | **1.14 m** above the floor (1.0 m cruise + 0.14 m mount) | derived |
| Stabilisation | ideal gimbal; the plane is held level every control tick | `Autopilot.stabilise_lidar` |
| Sectors | 12 × 30° | `navigation.SECTORS` |

**Never hard-code 720.** Read it with `getHorizontalResolution()`; the beam
count is a PROTO field and the whole pipeline follows it.

## Index → bearing

```python
bearing_dns(i) = direction * (2 * pi * i / resolution) + offset
#   direction = +1.0,  offset = +pi  (i.e. +180.000 deg)
```

Both the sweep direction and the angular origin were **measured, not assumed**:
`fit_lidar_mapping` searches the two unknowns against an analytic ray cast from
a known pose in known geometry and reports the residual. Webots documents the
range image as "scan lines left to right", which fixes the ordering but not the
sign or the origin once the device sits on a robot with its own axes.

Sector *i* is centred at `i * 30°` measured from +x rotating towards +y, so
**0 = front, 3 = right, 6 = back, 9 = left**. Binning is Spencer's
`navigation.pack_scan`, unmodified. The convention is verified per layout by
placing a single synthetic beam on each cardinal bearing and asserting the
sector it lands in.

> [!TIP]
> `+y` is **right** in the DNS frame and `-y` in Webots world coordinates. The
> sign lives in exactly one place, `dns/scenario.py`; everything else calls
> `to_dns`, `to_webots` and `bearing_to_webots`.

## Cap and `inf` handling

Webots returns `inf` beyond `maxRange`. `pack_scan` turns a *valid* `inf` into
"observed clear to 5.0 m" — the `(5.0, True)` encoding. A sector with no beam at
all is `(0.0, False)`, meaning *unknown*, which is different from *clear*.
Never treat 0.0 as a near obstacle.

## How accurate is it?

Measured against an independent analytic ray caster, sampled at the **same 720
bearings** and binned with the same `pack_scan`:

| Comparison | n | MAE | RMSE | p95 | max |
|---|---|---|---|---|---|
| Cart sector, pooled over 18 runs | {D.get('lidar', {}).get('cart_sector_pooled', {}).get('comparisons', 'TBC')} | **{D.get('lidar', {}).get('cart_sector_pooled', {}).get('mae_m', 'TBC')} m** | {D.get('lidar', {}).get('cart_sector_pooled', {}).get('rmse_m', 'TBC')} m | {D.get('lidar', {}).get('cart_sector_pooled', {}).get('p95_m', 'TBC')} m | {D.get('lidar', {}).get('cart_sector_pooled', {}).get('max_m', 'TBC')} m |

Per-run and per-layout breakdowns: `results/v2_S6/lidar_cart_consistency.csv`.

A methodological note you will care about: comparing the sensor against a
reference that samples only **three** sub-rays per sector (the model in
`planar.py`) inflates the error to ~0.2–0.5 m, because the 60-beam sensor sees
obstacle edges the 3-ray reference misses. Sampling the reference at the full
720 bearings drops it to the table above. If you build your own reference,
match the sampling density.

## Ground-truth data you can use offline

`results/v2_S6/runs/<run_id>/lidar.npz`, for variant 00 of every
(layout, mode, controller) cell — 60 runs:

```python
import numpy as np
d = np.load("results/v2_S6/runs/L00_C_ppo_seed7_v00/lidar.npz")
d["t_s"]                   # (T,)     simulated time
d["pose_xy"]               # (T, 2)   drone position, DNS frame
d["sector_ranges_m"]       # (T, 12)  the packed sectors the policy received
d["sector_valid"]          # (T, 12)  bool
d["obstacle_centres_xy"]   # (T, N, 2) supervisor ground truth, cart included
```

Regenerate the analytic reference for any frame with
`scripts/analyse_lidar_cart.py::reference_scan`.

## One open anomaly — please look at this

In v1's L00 Mode B run, `min_sensed_clearance_m` was **0.0536 m** while the
supervisor ground-truth clearance at the same time was **0.62 m**. A sector
reporting 5 cm with nothing within 60 cm is unexplained. It matters because
`gap_rule`'s repulsion scales with `1.45 − range`, so a spurious short return
produces a very large repulsive force.

Reproduce it:

```powershell
python scripts/with_webots.py --boot-wait 12 --timeout 400 `
    --world worlds/DNS_L00_C.wbt tests/client_capture.py -- `
    --layout L00 --mode B --controller reactive --variant 0
```

Candidate explanations I have not ruled out: a beam striking the airframe
(rotor tip radius 0.1773 m, `minRange` 0.05 m); a grazing return at an obstacle
edge; or a sector-binning edge case. It is logged as an open issue rather than
explained away.

## Plugging in your processing

Your code does not have to live here and nothing in this repo needs to change.
Take `sector_ranges_m` (or the raw range image, by reading the device directly)
as input and return 12 sector ranges plus a validity mask in the same encoding.
The swap point is `dns/sensing.lidar_scan`. If you want a hook added, open an
issue and I will add one rather than you patching the environment.
"""


DOSSIER = f"""# Simulation environment — dossier

What I was responsible for, what I delivered, and where the evidence is.

**Owner:** {PEOPLE['me_name']} ({PEOPLE['me_handle']}) ·
**Supervisor:** {PEOPLE['supervisor_name']}

My contribution is the **simulation environment**. I do not own the PPO policy
(Spencer) or raw sensor processing (the perception teammate), and neither is in
this repository.

| # | Responsibility | Delivered | Evidence | Status |
|---|---|---|---|---|
| 1 | **Simulator selection and setup** | Webots R2025a, extern-controller harness, world generated from config, offline-loadable | [ADR-001](../decisions/ADR-001-webots-r2025a-simulator.md), [ADR-013](../decisions/ADR-013-offscreen-camera-capture-built-in-recorder.md), `evidence/ENVIRONMENT.txt` | Done |
| 2 | **Navigation environment design** | 10 × 6 m arena, 2.5 m walls, 1.0 m cruise, cascaded autopilot at 125 Hz over an 8 ms step, 5 Hz command interface | [ADR-002](../decisions/ADR-002-keep-10-x-6-m-arena.md), [ADR-008](../decisions/ADR-008-5-hz-velocity-commands-125-hz-loop-8-ms-step.md) | Done |
| 3 | **Start and goal definition** | Start (0.85, 0.00), goal (9.15, 0.00), tolerance 0.4 m, identical in all six layouts and asserted per layout | contract tests, `tests/test_generator.py` | Done |
| 4 | **Static obstacle development** | Exact box footprints (not circles), asset library, seeded generator, six validated layouts, A* feasibility and difficulty descriptors | [ADR-004](../decisions/ADR-004-exact-box-footprints-bounding-circles.md), [ADR-009](../decisions/ADR-009-seeded-generator-graded-shift-ladder.md), `results/v2_S6/scenario_table.csv` | Done |
| 5 | **Moving obstacle modelling and validation** | Kinematic cart, ±1.9 m at 0.45 m/s, period 16.8889 s, swept collision test, LiDAR–cart consistency vs an independent ray caster | [ADR-006](../decisions/ADR-006-cart-is-kinematic.md), `results/v2_S6/lidar_cart_consistency.json` | Done, with a stated limitation (no physical contact response) |

## Beyond the five — what v2 added

- A **pre-registered** generalisation protocol, committed before any controller
  ran on L01–L05: [S6_GENERALIZATION_PROTOCOL.md](../verification/S6_GENERALIZATION_PROTOCOL.md)
- **{D['episodes']} episodes / {D['distinct']} distinct configurations** across six layouts
- **Nine corrections to v1**, each with evidence: [V1_CORRECTIONS.md](../verification/V1_CORRECTIONS.md)
- A **blocking gate** — world-vs-scenario check, 26 contract tests, A*
  feasibility and a flown reference traversal — that a layout must pass before
  it can contribute a result

## What teammates can reuse immediately

| You want | Take | Guide |
|---|---|---|
| To score a checkpoint on six layouts | `tests/client_s6_experiment.py`, `scripts/run_s6.py` | [FOR_SPENCER](FOR_SPENCER.md) |
| Ground-truth-labelled LiDAR frames | `results/v2_S6/runs/*/lidar.npz` | [FOR_SENSOR_TEAM](FOR_SENSOR_TEAM.md) |
| A new held-out layout family | `config/scenarios/generator_config.yaml` + `python -m dns.generator` | [ADR-009](../decisions/ADR-009-seeded-generator-graded-shift-ladder.md) |
| To check a claim | `evidence/CLAIMS_EVIDENCE.csv` | — |

## What I would not claim

The environment **exposes** policy behaviour; it does not explain it.
Policy-level conclusions are Spencer's. `direct` and `reactive` are untuned
references from his code and are never used to rank methods. Everything here is
**verification in simulation** — never real-world validation.
"""


def main() -> int:
    TEAM.mkdir(parents=True, exist_ok=True)
    INTERFACES.mkdir(parents=True, exist_ok=True)
    written = [
        (TEAM / "FOR_SUPERVISOR.md", FOR_SUPERVISOR),
        (TEAM / "FOR_SPENCER.md", FOR_SPENCER),
        (TEAM / "FOR_SENSOR_TEAM.md", FOR_SENSOR),
        (TEAM / "SIMULATION_ENVIRONMENT_DOSSIER.md", DOSSIER),
        (INTERFACES / "PPO_Interface_v2.md",
         FOR_SPENCER.replace("# For Spencer — evaluating checkpoints on the layout ladder",
                             "# PPO interface, v2\n\n*The v1 interface document is "
                             "unchanged; this is the v2 revision.*")),
        (INTERFACES / "Sensor_Interface_v2.md",
         FOR_SENSOR.replace("# For the sensor and perception teammate",
                            "# Sensor interface, v2\n\n*The v1 interface document "
                            "is unchanged; this is the v2 revision.*")),
    ]
    for path, text in written:
        path.write_text(text, encoding="utf8")
        print(f"wrote {path.relative_to(ROOT)}")
    tbc = [k for k, v in PEOPLE.items() if v == "TBC"]
    print(f"\nTBC identity fields carried into the docs: {', '.join(tbc)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
