"""Drive the reset-determinism study and summarise it.

For each target variant, one Webots session flies it **isolated** (first
episode after the world loads) and another flies it **in sequence** (the real
session prefix replayed first). Both are compared against the logged run.

Writes ``results/reset_study/SUMMARY.json`` and a CSV, plus the figure used in
the documentation.

Run::

    python scripts/run_reset_study.py
    python scripts/run_reset_study.py 0 1 2
"""
from __future__ import annotations

import csv
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results" / "reset_study"
RUNS = ROOT / "results" / "v2_S6" / "runs"
VARIANTS = (0, 1, 2, 5, 10)
LAYOUT, CONTROLLER = "L00", "ppo_seed7"


def run(variant: int, arm: str) -> bool:
    command = [
        sys.executable, str(ROOT / "scripts" / "with_webots.py"),
        "--boot-wait", "12", "--timeout", "900",
        "--world", f"worlds/DNS_{LAYOUT}_C.wbt",
        str(ROOT / "tests" / "client_reset_determinism.py"),
        "--", "--layout", LAYOUT, "--controller", CONTROLLER,
        "--variant", str(variant), "--arm", arm,
    ]
    done = subprocess.run(command, cwd=str(ROOT), capture_output=True,
                          text=True, timeout=1200)
    for line in done.stdout.splitlines():
        if line.startswith("["):
            print(" ", line, flush=True)
    if done.returncode != 0:
        print(f"  !! variant {variant} arm {arm} exited {done.returncode}")
        print(done.stdout[-800:])
    return done.returncode == 0


def logged(run_id: str) -> dict | None:
    path = RUNS / run_id / "trajectory.csv"
    if not path.is_file():
        return None
    with path.open(encoding="utf8") as handle:
        rows = list(csv.DictReader(handle))
    return {
        "track": np.array([[float(r["x_m"]), float(r["y_m"])] for r in rows]),
        "start": {"x_m": float(rows[0]["x_m"]), "y_m": float(rows[0]["y_m"]),
                  "altitude_m": float(rows[0]["altitude_m"]),
                  "roll_rad": float(rows[0]["roll_rad"]),
                  "pitch_rad": float(rows[0]["pitch_rad"])},
        "steps": len(rows) - 1,
    }


def _fmt(value) -> str:
    return "n/a" if value is None else f"{value:.3e}"


def main() -> int:
    variants = [int(v) for v in sys.argv[1:]] or list(VARIANTS)
    OUT.mkdir(parents=True, exist_ok=True)
    for variant in variants:
        for arm in ("isolated", "sequence"):
            run(variant, arm)

    with (ROOT / "results" / "v2_S6" / "results_v2.csv").open(encoding="utf8") as h:
        outcomes = {r["run_id"]: r["outcome"] for r in csv.DictReader(h)}

    rows = []
    for variant in variants:
        run_id = f"{LAYOUT}_C_{CONTROLLER}_v{variant:02d}"
        reference = logged(run_id)
        for arm in ("isolated", "sequence"):
            path = OUT / f"{run_id}_{arm}.json"
            if not path.is_file():
                continue
            report = json.loads(path.read_text(encoding="utf8"))
            start = report["start_state"]
            row = {
                "run_id": run_id, "variant": variant, "arm": arm,
                "prefix_episodes": report["prefix_episodes_replayed"],
                "outcome": report["outcome"],
                "logged_outcome": outcomes.get(run_id, ""),
                "outcome_matches": int(
                    outcomes.get(run_id, "").startswith(report["outcome"])),
                "steps": report["steps"],
                "logged_steps": reference["steps"] if reference else "",
                "steps_delta": (report["steps"] - reference["steps"]
                                if reference else ""),
                "max_path_delta_m": report["max_path_delta_vs_log_m"],
            }
            if reference:
                for key in ("altitude_m", "roll_rad", "pitch_rad"):
                    row[f"start_{key}_delta"] = round(
                        start[key] - reference["start"][key], 9)
            rows.append(row)

    if not rows:
        print("no results")
        return 1
    with (OUT / "reset_study.csv").open("w", newline="", encoding="utf8") as h:
        writer = csv.DictWriter(h, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    def arm_rows(arm): return [r for r in rows if r["arm"] == arm]

    summary = {
        "question": "Is a single episode reproducible on its own, or only when "
                    "the preceding episode sequence is replayed?",
        "layout": LAYOUT, "controller": CONTROLLER, "variants": variants,
        "arms": {},
    }
    for arm in ("isolated", "sequence"):
        group = arm_rows(arm)
        deltas = [r["max_path_delta_m"] for r in group
                  if r["max_path_delta_m"] is not None]
        summary["arms"][arm] = {
            "episodes": len(group),
            "outcome_matches_log": sum(r["outcome_matches"] for r in group),
            "step_count_matches_log": sum(1 for r in group
                                          if r["steps_delta"] == 0),
            "max_path_delta_m_median": (round(float(np.median(deltas)), 9)
                                        if deltas else None),
            "max_path_delta_m_worst": (round(float(np.max(deltas)), 9)
                                       if deltas else None),
        }
    (OUT / "SUMMARY.json").write_text(json.dumps(summary, indent=2),
                                      encoding="utf8")

    print(f"\n{'variant':>8}{'arm':>11}{'outcome':>12}{'logged':>12}"
          f"{'steps':>7}{'Δsteps':>8}{'max path Δ':>14}")
    for row in rows:
        print(f"{row['variant']:>8}{row['arm']:>11}{row['outcome']:>12}"
              f"{row['logged_outcome']:>12}{row['steps']:>7}"
              f"{str(row['steps_delta']):>8}"
              f"{_fmt(row['max_path_delta_m']):>14}")
    for arm, stats in summary["arms"].items():
        print(f"\n{arm}: outcome matched log {stats['outcome_matches_log']}"
              f"/{stats['episodes']}, step count matched "
              f"{stats['step_count_matches_log']}/{stats['episodes']}, "
              f"worst path delta {stats['max_path_delta_m_worst']} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
