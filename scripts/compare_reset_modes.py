"""Per-scenario comparison of the two reset modes, for the same protocol.

Both cohorts use identical scenario JSONs, identical policy checkpoints and
identical evaluation rules; the only difference is `WebotsBackend.reset_mode`.
Any difference reported here is therefore attributable to the reset procedure,
not to a policy change -- nothing in Spencer's code was touched.

Writes `results/v2_S6_deterministic/reset_mode_comparison.csv` and a summary.

Run::  python scripts/compare_reset_modes.py
"""
from __future__ import annotations

import csv
import json
import math
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT / "results" / "v2_S6" / "results_v2.csv"
NEW_DIR = ROOT / "results" / "v2_S6_deterministic"
LAYOUTS = ("L00", "L01", "L02", "L03", "L04", "L05")


def wilson(k: int, n: int, z: float = 1.959964) -> tuple[float, float]:
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def load_new() -> dict:
    rows = {}
    for path in sorted(NEW_DIR.glob("results_L*.csv")):
        with path.open(encoding="utf8") as handle:
            for row in csv.DictReader(handle):
                rows[row["run_id"]] = row
    return rows


def main() -> int:
    with OLD.open(encoding="utf8") as handle:
        old = {r["run_id"]: r for r in csv.DictReader(handle)}
    new = load_new()
    shared = sorted(set(old) & set(new))
    if not shared:
        print("no overlapping episodes yet")
        return 1

    rows, summary = [], []
    for run_id in shared:
        o, n = old[run_id], new[run_id]
        rows.append({
            "run_id": run_id, "layout_id": n["layout_id"], "mode": n["mode"],
            "controller": n["controller"], "variant": n["variant"],
            "outcome_old": o["outcome"], "outcome_new": n["outcome"],
            "outcome_changed": int(o["outcome"] != n["outcome"]),
            "steps_old": o["termination_step"], "steps_new": n["termination_step"],
            "steps_changed": int(o["termination_step"] != n["termination_step"]),
            "path_old_m": o["path_length_m"], "path_new_m": n["path_length_m"],
            "path_abs_diff_m": round(abs(float(o["path_length_m"])
                                         - float(n["path_length_m"])), 4),
            "min_clearance_old_m": o["min_clearance_m"],
            "min_clearance_new_m": n["min_clearance_m"],
        })

    for layout in LAYOUTS:
        group = [r for r in rows if r["layout_id"] == layout]
        if not group:
            continue
        for mode in ("B_static", "C_dynamic"):
            cell = [r for r in group if r["mode"] == mode]
            ppo = [r for r in cell if r["controller"].startswith("ppo_")]
            if not ppo:
                continue
            distinct = [r for r in ppo
                        if mode == "C_dynamic" or r["variant"] == "v00"]
            ko = sum(1 for r in distinct if r["outcome_old"] == "goal")
            kn = sum(1 for r in distinct if r["outcome_new"] == "goal")
            lo, ho = wilson(ko, len(distinct))
            ln, hn = wilson(kn, len(distinct))
            summary.append({
                "layout_id": layout, "mode": mode, "n_distinct": len(distinct),
                "ppo_pooled_old": f"{ko}/{len(distinct)}",
                "ppo_pooled_new": f"{kn}/{len(distinct)}",
                "wilson_old": f"[{lo:.3f}, {ho:.3f}]",
                "wilson_new": f"[{ln:.3f}, {hn:.3f}]",
                "episodes_outcome_changed": sum(r["outcome_changed"] for r in cell),
                "episodes_steps_changed": sum(r["steps_changed"] for r in cell),
                "episodes_compared": len(cell),
                "max_path_diff_m": max(r["path_abs_diff_m"] for r in cell),
            })

    with (NEW_DIR / "reset_mode_comparison.csv").open(
            "w", newline="", encoding="utf8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    with (NEW_DIR / "reset_mode_summary.csv").open(
            "w", newline="", encoding="utf8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summary[0]))
        writer.writeheader(); writer.writerows(summary)

    changed = [r for r in rows if r["outcome_changed"]]
    print(f"episodes compared: {len(rows)}   outcome changed: {len(changed)}   "
          f"step count changed: {sum(r['steps_changed'] for r in rows)}")
    print(f"\n{'layout':>7}{'mode':>11}{'PPO old':>10}{'PPO new':>10}"
          f"{'Δoutcome':>10}{'max Δpath':>11}")
    for s in summary:
        print(f"{s['layout_id']:>7}{s['mode']:>11}{s['ppo_pooled_old']:>10}"
              f"{s['ppo_pooled_new']:>10}{s['episodes_outcome_changed']:>10}"
              f"{s['max_path_diff_m']:>11.3f}")
    if changed:
        print("\nOutcome changes (reset procedure only; no policy was modified):")
        for r in changed:
            print(f"  {r['run_id']:<30} {r['outcome_old']:<18} -> "
                  f"{r['outcome_new']:<18} steps {r['steps_old']}->{r['steps_new']}")
    print(f"\nwrote {(NEW_DIR / 'reset_mode_comparison.csv').relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
