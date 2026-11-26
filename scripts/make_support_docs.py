"""Generate CHANGELOG, CITATION.cff, CONTRIBUTING and the per-folder READMEs.

Every top-level folder gets a short README saying what is inside, who uses it
and how it is produced, so a newcomer can orient without reading code.

Run::

    python scripts/make_support_docs.py
"""
from __future__ import annotations

import csv
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results" / "v2_S6"

with (RESULTS / "results_v2.csv").open(encoding="utf8") as handle:
    ROWS = list(csv.DictReader(handle))
EPISODES = len(ROWS)
DISTINCT = sum(int(r["distinct_configuration"]) for r in ROWS)

CHANGELOG = f"""# Changelog

All notable changes to the simulation environment.

## [2.0] — 2026-11-25 — S6 generalisation and transparency release

### Added
- **Seeded scenario generator** (`dns/generator.py`): one YAML config plus a
  seed produces a byte-identical scenario JSON, hashed, from which the Webots
  world is written. Byte-identity is tested across separate processes.
- **Six-layout distribution-shift ladder** L00–L05, each validated,
  A*-feasible and frozen before any controller ran. En-route corridor widths
  2.08 / 1.87 / 2.08 / 1.31 / 0.85 / 0.68 m.
- **S6 generalisation experiment**: {EPISODES} episodes = {DISTINCT} distinct
  configurations — 5 controllers × 6 layouts × (Mode B 3 runs + Mode C 15 cart
  phases). The protocol was committed before anything ran.
- **Feasibility and difficulty tooling** (`dns/feasibility.py`): exact
  clearance field, A*, widest-path bottleneck corridor width, IoU
  dissimilarity, cart-interaction descriptors.
- **World ↔ scenario verification** (`dns/worldcheck.py`): refuses to produce
  results when the loaded world's node poses, time step, observation interval
  or LiDAR configuration disagree with the scenario.
- **Blocking layout gate**: 26 contract tests, the world check and a flown A*
  reference traversal, per layout.
- **LiDAR–cart consistency** against an independent analytic ray caster sampled
  at the same 720 bearings: cart-sector MAE 0.0193 m over 650 comparisons.
- 26 generator and validator unit tests; real Webots captures, four GIFs, the
  demonstration video, and 14 architecture decision records.

### Changed
- Repository reorganised into `results/v1_L00` and `results/v2_S6`,
  `config/scenarios`, `docs/` sub-folders and `assets/` sub-folders. Moves used
  `git mv`, so file history follows the files.
- `tools/` merged into `scripts/`.
- `scripts/build_deliverable.py` no longer bakes in a machine-specific Webots
  path; it probes the usual install locations.

### Fixed — corrections to v1 (full list in `docs/verification/V1_CORRECTIONS.md`)
- **C1** The reactive explanation compared the 1.45 m repulsion radius against
  the crate's *centre* distance (1.54 m); the controller reacts to the
  *surface*, 1.0582 m away. Restated with the measured mechanism.
- **C2** "Cart vs model 0.00e+00 m" is true by construction; reclassified as an
  implementation-consistency check and replaced with real dynamic evidence.
- **C3** v1's Mode B "3/3" is 1/1 per controller — the episode seed is accepted
  but never used.
- **C4** Per-seed results are pooled with Wilson intervals and never ranked.
- **C8** v1 shipped a `protocol.json` describing a different run than its own
  `results.csv`.

### Verified
- v1's L00 protocol reproduces **10/10 cells with a maximum per-episode
  path difference of 0 m when the session is replayed in order**. A single
  episode re-run in isolation drifts up to 0.287 m -- see
  `docs/verification/RESET_DETERMINISM.md`.
- Maximum obstacle t = 0 pose error across all six layouts: ≤ 4.0e-06 m.
- 3 of {EPISODES} episodes exceeded 0.50 rad airframe tilt.

### Known limitations
- **L04 is excluded** from the generalisation conclusion pending a decision: no
  controller and no reference-tracker setting reaches the goal within the
  90-step horizon, so its 0/90 is not attributable.
- Physical contact response with the cart is not modelled.
- The LiDAR gimbal is ideal, which is a best case rather than a real sensor.
- One unexplained sensor reading: 0.0536 m against 0.62 m ground-truth clearance.

## [1.0] — 2026-11-24 — first research milestone
- One obstacle layout, 90 episodes (Mode B static / Mode C dynamic), five
  controllers; the v1 verification report; the Webots environment, drone PROTO,
  cascaded autopilot, LiDAR sector pipeline and the teammate handoffs.
- Tagged `v1.0` and recoverable in full.
"""

CITATION = """cff-version: 1.2.0
title: >-
  DNS Webots simulation environment for UAV navigation among static and
  dynamic obstacles
message: >-
  If you use this simulation environment, please cite it. The author's given
  name and GitHub handle are confirmed; the full legal name is still to be
  supplied and is marked below rather than guessed.
type: software
version: "2.0"
date-released: "2026-11-25"
repository-code: "https://github.com/khh4lid/dns-webots-simulation"
abstract: >-
  A reproducible Webots simulation environment for UAV navigation research: a
  seeded scenario generator producing byte-identical layouts, a six-layout
  distribution-shift ladder with validated feasibility, a kinematic moving
  obstacle, a verified twelve-sector LiDAR contract, and a pre-registered
  spatial-generalisation experiment of 540 episodes over 480 distinct
  configurations.
keywords:
  - webots
  - uav
  - quadcopter
  - robotics-simulation
  - autonomous-navigation
  - lidar
  - reproducibility
  - reinforcement-learning
authors:
  - given-names: Khaled
    # family-names deliberately omitted: not on record for this release.
    # Supply it and re-run scripts/make_support_docs.py before citing.
    alias: khh4lid
    affiliation: "Taylor's University, Malaysia"
"""

CONTRIBUTING = """# Contributing

This repository holds **one** contribution to the DNS project: the Webots
simulation environment. The PPO policy and raw sensor processing live in their
owners' own repositories and are deliberately not vendored here.

## Ground rules

1. **Do not commit other people's code.** Record the interface and the SHA-256
   fingerprint of what you ran against instead. `python scripts/freeze.py
   --verify` proves the frozen baseline is untouched.
2. **Do not hand-edit a `.wbt`.** Worlds are generated from
   `config/scenarios/*.json` by `scripts/build_worlds.py`. A hand edit is
   silently overwritten, and `dns/worldcheck.py` rejects the mismatch.
3. **Every number in a document must come from a file in the repository** and
   appear in `evidence/CLAIMS_EVIDENCE.csv`. If a file disagrees with a
   document, the file wins and the document is fixed.
4. **Label every visual** WEBOTS CAPTURE, LOGGED-DATA RENDER or SCHEMATIC.
5. **Never delete a failed run.** Negative and surprising results are the point.

## Branches and pull requests

Branch from `main` as `feature/<name>`, `fix/<name>` or `docs/<name>`. No
force-pushes to `main`, no history rewriting. Open a PR and work the checklist:

```
[ ] python scripts/security_scan.py      -> VERDICT: CLEAN
[ ] python tests/test_generator.py       -> 26 passed, 0 failed
[ ] python scripts/run_gates.py          -> every layout PASS
[ ] python reproduce.py --fast           -> figures and tables rebuild
[ ] every new number is in evidence/CLAIMS_EVIDENCE.csv
[ ] every new visual has a provenance label and a caption
[ ] no file over 50 MB; GIFs under 8 MB; MP4s go to Release assets
[ ] evidence/RELEASE_STATE.md updated
```

## Where things go

| You are adding | Put it in |
|---|---|
| A new layout family | `config/scenarios/generator_config.yaml`, then regenerate |
| An environment test tool | `scripts/` |
| A Webots client | `tests/client_*.py` |
| A design decision | a new ADR via `scripts/make_adrs.py` |
| A figure | `scripts/make_figures.py` or `make_visuals.py` — never a hand-drawn image |
| A teammate-facing guide | `docs/team/` |

## Issues

Label with one of `env`, `ppo`, `sensor`, `verification`, `decision-needed`,
`question`, `bug`. An issue reporting a measurement should link the run id and
the file it came from.
"""

FOLDERS = {
    "assets": (
        "Every visual in the repository",
        "README and report images by kind: `figures/` (plots and panels), "
        "`gifs/` (looping clips), `renders/` (Webots captures and video "
        "posters), `diagrams/` (schematics), `decisions/` (one image per ADR).",
        "Generated by `scripts/make_figures.py`, `scripts/make_visuals.py` and "
        "`scripts/make_clips.py`. Nothing here is hand-drawn or stock."),
    "config": (
        "Scenario definitions",
        "`scenarios/` holds the generator config, the twelve canonical scenario "
        "JSONs (six layouts x two modes) and their SHA-256 list. The v1 configs "
        "sit alongside, unchanged.",
        "Generated by `python -m dns.generator --all`; byte-identical for the "
        "same config and seed."),
    "dns": (
        "The simulation package",
        "`scenario` (geometry and the frame transform), `backend` (Spencer's "
        "Backend protocol over Webots), `autopilot`, `sensing`, `generator`, "
        "`feasibility`, `worldcheck`, `worldgen`, `launcher`, `spencer_link`.",
        "Hand-written. `spencer_link` only puts Spencer's directory on "
        "`sys.path` -- nothing of his is copied or shadowed."),
    "docs": (
        "All documentation",
        "`team/` role guides, `interfaces/` frozen contracts, `decisions/` "
        "ADRs, `verification/` reports and corrections, `results/` tables, "
        "`meetings/`, `reports/`.",
        "Partly generated (`scripts/make_adrs.py`, `scripts/make_team_docs.py`) "
        "so the numbers quoted inside track the data files."),
    "evidence": (
        "Traceability",
        "`CLAIMS_EVIDENCE.csv` maps every claim to its data file and script; "
        "`BASELINE_HASHES.txt` and `ENVIRONMENT.txt` pin what was frozen and on "
        "what machine; `RELEASE_STATE.md` is the working state; `FILE_MAP.md` "
        "records the v2 migration.",
        "`scripts/make_claims.py` and `scripts/freeze.py`."),
    "protos": (
        "The drone model",
        "`DnsQuadcopter.proto` -- airframe, four `Propeller` rotors, the "
        "orientable LiDAR mount and the optional offscreen demonstration camera.",
        "Hand-written; every dimension is referenced by the scenario validators."),
    "results": (
        "Experiment output",
        "`v1_L00/` the 90-episode v1 experiment; `v2_S6/` the 540-episode "
        "ladder with per-run folders, pooled rates, descriptors and the "
        "verification analyses.",
        "Written by `tests/client_s6_experiment.py` via `scripts/run_s6.py`."),
    "scripts": (
        "Tooling",
        "Generator drivers, experiment runners, figure and clip builders, the "
        "security scan, the migration, the freeze/verify manifest and the "
        "claims table.",
        "Run any of them with `--help`."),
    "tests": (
        "Verification",
        "`test_generator.py` (26 unit tests, no Webots needed), "
        "`client_layout_gate.py` (the blocking gate), `client_s6_experiment.py`, "
        "`client_capture.py`, and the v1 clients.",
        "Webots clients run through `scripts/with_webots.py`."),
    "worlds": (
        "Generated Webots worlds",
        "One `.wbt` per layout and mode, each stamped with the SHA-256 of the "
        "scenario that produced it. The v1 worlds remain alongside.",
        "Generated by `scripts/build_worlds.py`. **Do not hand-edit** -- the "
        "world check will reject a drifted file."),
}


def main() -> int:
    (ROOT / "CHANGELOG.md").write_text(CHANGELOG, encoding="utf8")
    (ROOT / "CITATION.cff").write_text(CITATION, encoding="utf8")
    (ROOT / "CONTRIBUTING.md").write_text(CONTRIBUTING, encoding="utf8")
    for folder, (title, what, how) in FOLDERS.items():
        target = ROOT / folder
        if not target.is_dir():
            continue
        (target / "README.md").write_text(
            f"# `{folder}/` — {title}\n\n"
            f"**What is in here.** {what}\n\n"
            f"**How it is produced.** {how}\n\n"
            f"[Back to the repository README](../README.md)\n", encoding="utf8")
    print(f"wrote CHANGELOG.md, CITATION.cff, CONTRIBUTING.md "
          f"and {len(FOLDERS)} folder READMEs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
