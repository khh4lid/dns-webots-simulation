"""Stage A migration: move the repo into the v2 layout and import the v2 package.

Two kinds of operation, kept strictly apart:

* **MOVE** -- a file already tracked by git is relocated with ``git mv`` so its
  history follows it. Never a copy-and-delete, which would orphan the history.
* **IMPORT** -- a file that exists only in the v2 package
  (``DNS_FRIDAY_MEETING_2026-11-27``) is copied in. Spencer's code and
  checkpoints and the sensor teammate's work are **never** imported; only
  interface descriptions and hashes of their files.

The plan is data, printed as a table and written to ``evidence/FILE_MAP.md``,
so the mapping can be reviewed before anything moves (``--dry-run``, default).

Run::

    python scripts/migrate_v2.py              # show the plan
    python scripts/migrate_v2.py --apply      # perform it
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parent
PACKAGE = WORKSPACE / "DNS_FRIDAY_MEETING_2026-11-27"

#: New directories the v2 layout needs, each with the audience it serves.
NEW_DIRS = {
    "assets/figures": "README figures (core + extended), 300 dpi PNG",
    "assets/gifs": "looping GIFs embedded in the README",
    "assets/renders": "Webots Camera captures and video posters",
    "assets/diagrams": "schematics and architecture drawings",
    "assets/decisions": "one image per ADR",
    "config/scenarios": "generated scenario JSONs + HASHES.txt",
    "results/v1_L00": "the v1 90-episode experiment on the training layout",
    "results/v2_S6": "the v2 540-episode shift-ladder experiment",
    "docs/team": "role guides: supervisor, PPO, sensor, dossier",
    "docs/interfaces": "frozen contracts handed to teammates",
    "docs/decisions": "architecture decision records",
    "docs/verification": "verification report, corrections, audit",
    "docs/results": "full result tables and analysis",
    "docs/meetings/2026-11-27": "agenda and open questions",
    "docs/reports": "progress report PDF and source",
    "evidence/screenshots": "rendered-README and step screenshots",
}

#: (source relative to repo, destination relative to repo). git mv, history kept.
MOVES = [
    ("results/experiments_warehouse", "results/v1_L00"),
    # Untracked, so git mv does not apply; moved on the filesystem and picked
    # up as a new path by the first commit. 540 folders, 8.1 MB, largest file
    # 10.8 KB -- comfortably inside the size limits this release works to.
    ("results/s6/runs", "results/v2_S6/runs"),
    ("results/s6", "results/v2_S6/_session_logs"),
]

#: (source in the v2 package, destination in the repo, note)
IMPORTS = [
    # --- scenarios -------------------------------------------------------
    ("07_SCENARIOS/generator_config.yaml", "config/scenarios/generator_config.yaml",
     "the single input to the generator"),
    ("07_SCENARIOS/HASHES.txt", "config/scenarios/HASHES.txt",
     "SHA-256 of each canonical scenario"),
    # --- results ---------------------------------------------------------
    ("06_DATA/results_v2.csv", "results/v2_S6/results_v2.csv", "540 episodes"),
    ("06_DATA/pooled_rates.csv", "results/v2_S6/pooled_rates.csv",
     "pooled rates with Wilson intervals"),
    ("06_DATA/scenario_table.csv", "results/v2_S6/scenario_table.csv",
     "per-layout descriptors"),
    ("06_DATA/asset_library.csv", "results/v2_S6/asset_library.csv",
     "every obstacle placement"),
    ("06_DATA/v1_reproduction.csv", "results/v2_S6/v1_reproduction.csv",
     "v1 vs v2 cell comparison"),
    ("06_DATA/v1_distinctness.json", "results/v2_S6/v1_distinctness.json",
     "episode-variant analysis"),
    ("06_DATA/reactive_gnron_analysis.json",
     "results/v2_S6/reactive_gnron_analysis.json", "C1 evidence"),
    ("06_DATA/generator_report.json", "results/v2_S6/generator_report.json",
     "draw attempts and rejection counts"),
    # --- verification docs ----------------------------------------------
    ("10_EVIDENCE/V1_CORRECTIONS.md", "docs/verification/V1_CORRECTIONS.md",
     "C1-C9"),
    ("10_EVIDENCE/AUDIT_SUMMARY.md", "docs/verification/AUDIT_SUMMARY.md",
     "Phase 0 gate"),
    ("10_EVIDENCE/S6_GENERALIZATION_PROTOCOL.md",
     "docs/verification/S6_GENERALIZATION_PROTOCOL.md", "pre-registered"),
    ("10_EVIDENCE/DECISIONS.md", "docs/decisions/DECISIONS_LOG.md",
     "running decision log; ADRs expand on it"),
    ("10_EVIDENCE/ASSUMPTIONS.md", "docs/verification/ASSUMPTIONS.md",
     "assumption register"),
    ("10_EVIDENCE/BASELINE_HASHES.txt", "evidence/BASELINE_HASHES.txt",
     "frozen baseline manifest"),
    ("10_EVIDENCE/ENVIRONMENT.txt", "evidence/ENVIRONMENT.txt",
     "machine and package versions"),
    ("10_EVIDENCE/CLAIMS_EVIDENCE.csv", "evidence/CLAIMS_EVIDENCE.csv",
     "every number traced to its source"),
    ("10_EVIDENCE/TASK_PROMPT.md", "evidence/TASK_PROMPT_v2.md",
     "the brief this release was produced against"),
    # --- figures ---------------------------------------------------------
    ("03_FIGURES/captions.md", "docs/FIGURES.md", "figure catalogue"),
]

#: Scenario JSONs and worlds are imported by glob rather than listed one by one.
GLOB_IMPORTS = [
    ("07_SCENARIOS/L0*.json", "config/scenarios", "6 layouts x 2 modes"),
    ("03_FIGURES/core/*.png", "assets/figures", "core figures"),
    ("03_FIGURES/extended/*.png", "assets/figures", "extended figures"),
    ("09_HANDOFFS/sensor_pack/SV1_sensor_schematic.png", "assets/diagrams",
     "sensor schematic"),
]

#: Explicitly NOT imported, with the reason. Printed so the exclusion is visible.
EXCLUDED = [
    ("DroneNavPPO/**", "Spencer's code and checkpoints -- his to publish; "
                       "only the interface and SHA-256 hashes are recorded"),
    ("10_EVIDENCE/TASK_PROMPT.md verbatim brief", "imported as TASK_PROMPT_v2.md "
                                                  "for provenance"),
    ("06_DATA/runs/** (540 run folders)", "imported separately, size-checked"),
]


def run(command: list[str], apply: bool) -> None:
    if not apply:
        return
    done = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    if done.returncode != 0:
        print(f"    ! {' '.join(command)}: {done.stderr.strip()[:160]}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    apply = args.apply
    rows: list[tuple[str, str, str, str]] = []

    print(f"{'op':<8}{'source':<52}{'destination':<44}note")
    print("-" * 140)

    for relative, purpose in NEW_DIRS.items():
        target = ROOT / relative
        if apply:
            target.mkdir(parents=True, exist_ok=True)
            keep = target / ".gitkeep"
            if not any(target.iterdir()):
                keep.write_text("", encoding="utf8")
        rows.append(("MKDIR", "-", relative, purpose))

    for source, destination in MOVES:
        src, dst = ROOT / source, ROOT / destination
        if src.exists():
            tracked = subprocess.run(
                ["git", "ls-files", "--error-unmatch", source], cwd=ROOT,
                capture_output=True, text=True).returncode == 0
            if apply:
                dst.parent.mkdir(parents=True, exist_ok=True)
                if dst.exists():
                    shutil.rmtree(dst)
                if tracked:
                    run(["git", "mv", source, destination], apply)
                else:
                    shutil.move(str(src), str(dst))
            rows.append(("MOVE" if tracked else "MOVE*", source, destination,
                         "git mv, history kept" if tracked
                         else "untracked; filesystem move, new path at commit"))
        else:
            rows.append(("SKIP", source, destination, "source absent"))

    for source, destination, note in IMPORTS:
        src = PACKAGE / source
        dst = ROOT / destination
        if src.is_file():
            if apply:
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dst)
            rows.append(("IMPORT", f"pkg/{source}", destination, note))
        else:
            rows.append(("MISS", f"pkg/{source}", destination, "not in package"))

    for pattern, destination, note in GLOB_IMPORTS:
        matches = sorted(PACKAGE.glob(pattern))
        for src in matches:
            dst = ROOT / destination / src.name
            if apply:
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dst)
        rows.append(("IMPORT", f"pkg/{pattern}", f"{destination}/",
                     f"{note} ({len(matches)} files)"))

    for pattern, reason in EXCLUDED:
        rows.append(("EXCLUDE", pattern, "-", reason))

    for op, source, destination, note in rows:
        print(f"{op:<8}{source[:51]:<52}{destination[:43]:<44}{note[:60]}")

    lines = ["# FILE_MAP.md — where every v2 artefact lives in the repository",
             "",
             "Generated by `scripts/migrate_v2.py`. `MOVE` uses `git mv` so file",
             "history follows the file; `IMPORT` copies from the v2 meeting",
             "package; `EXCLUDE` records what is deliberately **not** committed.",
             "",
             "| op | source | destination | note |", "|---|---|---|---|"]
    for op, source, destination, note in rows:
        lines.append(f"| {op} | `{source}` | `{destination}` | {note} |")
    target = ROOT / "evidence" / "FILE_MAP.md"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text("\n".join(lines) + "\n", encoding="utf8")
    print(f"\nwrote {target.relative_to(ROOT)}  ({len(rows)} entries)")
    print("DRY RUN — rerun with --apply to perform it" if not apply else "APPLIED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
