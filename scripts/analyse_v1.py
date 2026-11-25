"""Phase 0 / Phase 3 analysis of v1: episode distinctness and the L00 reproduction.

Answers three questions with numbers, and writes them where the report and the
corrections log can cite them:

1. **What actually differs between the episodes inside each v1 cell?**
   (Phase 0 item 4.) If the three Mode B episodes are the same configuration,
   every v1 Mode B rate has n = 1, not n = 3, and must be restated.
2. **Does the v2 rerun of L00 reproduce v1's results.csv?** (Phase 0 item 5.)
   Cell-by-cell outcome comparison plus per-episode path-length differences, so
   "identical" is a measurement rather than an impression.
3. **What do the v1 numbers become once n counts configurations?**
   (Phase 3 items 3 and 4.) Wilson 95% intervals on pooled proportions.

Outputs: ``06_DATA/v1_reproduction.csv``, ``06_DATA/v1_distinctness.json``,
``06_DATA/pooled_rates.csv``.

Run::

    python tools/analyse_v1.py
"""
from __future__ import annotations

import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = PROJECT_ROOT.parent
DATA = DATA

V1_RESULTS = WORKSPACE / "DNS_SIMULATION_RESEARCH_DELIVERABLE" / "03_RESEARCH" / "results.csv"
V2_RESULTS = DATA / "results_v2.csv"

CONTROLLERS = ("ppo_seed7", "ppo_seed19", "ppo_seed43", "reactive", "direct")


def wilson(successes: int, n: int, z: float = 1.959964) -> tuple[float, float]:
    """Wilson score interval. Sane at n = 3; the normal approximation is not."""
    if n == 0:
        return (float("nan"), float("nan"))
    p = successes / n
    denominator = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denominator
    half = (z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / denominator
    return (max(0.0, centre - half), min(1.0, centre + half))


def load(path: Path) -> list[dict]:
    with path.open(encoding="utf8") as handle:
        return list(csv.DictReader(handle))


# ---------------------------------------------------------------------------
# 1. distinctness
# ---------------------------------------------------------------------------
def distinctness(v1: list[dict]) -> dict:
    """Which fields vary across the episodes inside one v1 cell."""
    report: dict = {
        "question": "What differs between episodes inside each v1 cell?",
        "method": ("For each (mode, controller) cell, list the distinct values "
                   "of every field that could define a configuration."),
        "cells": {},
    }
    for mode in ("B_static", "C_dynamic"):
        for controller in CONTROLLERS:
            group = [r for r in v1 if r["mode"] == mode
                     and r["controller"] == controller]
            if not group:
                continue
            varying = {}
            for field in ("seed", "obstacle_phase_rad", "start_x_m", "start_y_m",
                          "goal_x_m", "goal_y_m", "scenario"):
                values = sorted({r[field] for r in group})
                varying[field] = values
            # Does any configuration-defining field actually differ?
            config_fields = ("obstacle_phase_rad", "start_x_m", "start_y_m",
                             "goal_x_m", "goal_y_m", "scenario")
            def key(row: dict) -> tuple:
                parts = []
                for field in config_fields:
                    value = row[field]
                    if value in ("", None):
                        parts.append(None)
                        continue
                    try:
                        # Round numeric fields so -0.0 and 0.0 are one value;
                        # v1's start_y_m is recorded as both.
                        parts.append(round(float(value), 6) + 0.0)
                    except ValueError:
                        parts.append(value)
                return tuple(parts)

            distinct_configs = len({key(r) for r in group})
            paths = sorted(float(r["path_length_m"]) for r in group)
            report["cells"][f"{mode}/{controller}"] = {
                "episodes": len(group),
                "distinct_configurations": distinct_configs,
                "field_values": varying,
                "outcomes": dict(Counter(r["outcome"] for r in group)),
                "path_length_spread_m": round(paths[-1] - paths[0], 6),
                "control_steps": sorted({int(r["control_steps"]) for r in group}),
            }
    return report


# ---------------------------------------------------------------------------
# 2. reproduction
# ---------------------------------------------------------------------------
def reproduction(v1: list[dict], v2: list[dict]) -> list[dict]:
    """Cell-by-cell v1 vs v2 L00 comparison."""
    v2_l00 = [r for r in v2 if r["layout_id"] == "L00"]
    rows = []
    for mode_v1, mode_v2 in (("B_static", "B_static"), ("C_dynamic", "C_dynamic")):
        for controller in CONTROLLERS:
            a = [r for r in v1 if r["mode"] == mode_v1
                 and r["controller"] == controller]
            b = [r for r in v2_l00 if r["mode"] == mode_v2
                 and r["controller"] == controller]
            if not a or not b:
                continue
            ka = sum(int(r["goal_reached"]) for r in a)
            kb = sum(int(r["goal_reached"]) for r in b)
            # Per-episode path comparison, matched on the episode index.
            deltas = []
            for first, second in zip(sorted(a, key=lambda r: r["experiment_id"]),
                                     sorted(b, key=lambda r: r["run_id"])):
                deltas.append(abs(float(first["path_length_m"])
                                  - float(second["path_length_m"])))
            rows.append({
                "mode": mode_v1,
                "controller": controller,
                "v1_k": ka, "v1_n": len(a),
                "v2_k": kb, "v2_n": len(b),
                "outcomes_match": int(ka == kb and len(a) == len(b)),
                "v1_outcomes": "|".join(f"{k}:{v}" for k, v in
                                        sorted(Counter(r["outcome"] for r in a).items())),
                "v2_outcomes": "|".join(f"{k}:{v}" for k, v in
                                        sorted(Counter(
                                            r["outcome"].replace("collision_static", "collision")
                                            .replace("collision_cart", "collision")
                                            .replace("collision_wall", "collision")
                                            for r in b).items())),
                "max_path_delta_m": round(max(deltas), 6) if deltas else "",
                "mean_path_delta_m": round(sum(deltas) / len(deltas), 6) if deltas else "",
            })
    return rows


# ---------------------------------------------------------------------------
# 3. pooled rates with intervals
# ---------------------------------------------------------------------------
def pooled(v2: list[dict]) -> list[dict]:
    """PPO pooled over the three checkpoints, per layout and mode.

    n counts DISTINCT CONFIGURATIONS: Mode C contributes 15 per checkpoint,
    Mode B contributes 1 per checkpoint (the three Mode B episodes are repeats
    of one configuration -- see distinctness()).
    """
    rows = []
    layouts = sorted({r["layout_id"] for r in v2})
    for layout in layouts:
        for mode in ("B_static", "C_dynamic"):
            # --- PPO pooled ---
            group = [r for r in v2 if r["layout_id"] == layout
                     and r["mode"] == mode
                     and r["controller_kind"] == "ppo_checkpoint"]
            if not group:
                continue
            distinct = [r for r in group if int(r["distinct_configuration"]) == 1]
            k = sum(int(r["goal_reached"]) for r in distinct)
            n = len(distinct)
            low, high = wilson(k, n)
            rows.append({
                "layout_id": layout, "mode": mode, "group": "ppo_pooled",
                "k": k, "n": n, "rate": round(k / n, 4) if n else "",
                "wilson_low": round(low, 4), "wilson_high": round(high, 4),
                "repeats_run": len(group),
                "note": "pooled over ppo_seed7/19/43; n = distinct configurations",
            })
            # --- per seed, reported but never ranked ---
            for controller in ("ppo_seed7", "ppo_seed19", "ppo_seed43",
                               "reactive", "direct"):
                cell = [r for r in v2 if r["layout_id"] == layout
                        and r["mode"] == mode and r["controller"] == controller
                        and int(r["distinct_configuration"]) == 1]
                if not cell:
                    continue
                kc = sum(int(r["goal_reached"]) for r in cell)
                lo, hi = wilson(kc, len(cell))
                rows.append({
                    "layout_id": layout, "mode": mode, "group": controller,
                    "k": kc, "n": len(cell),
                    "rate": round(kc / len(cell), 4),
                    "wilson_low": round(lo, 4), "wilson_high": round(hi, 4),
                    "repeats_run": len([r for r in v2
                                        if r["layout_id"] == layout
                                        and r["mode"] == mode
                                        and r["controller"] == controller]),
                    "note": ("untuned reference, never used to rank methods"
                             if controller in ("reactive", "direct")
                             else "single checkpoint; intervals overlap, do not rank"),
                })
    return rows


def main() -> int:
    DATA.mkdir(parents=True, exist_ok=True)
    v1 = load(V1_RESULTS)
    print(f"v1: {len(v1)} episodes from {V1_RESULTS.name}")

    report = distinctness(v1)
    (DATA / "v1_distinctness.json").write_text(json.dumps(report, indent=2),
                                               encoding="utf8")
    print("\n--- v1 episode distinctness ---")
    for name, cell in report["cells"].items():
        print(f"  {name:<26} episodes={cell['episodes']:>2} "
              f"distinct_configs={cell['distinct_configurations']:>2} "
              f"path_spread={cell['path_length_spread_m']:.4f} m "
              f"steps={cell['control_steps']}")

    if not V2_RESULTS.is_file():
        print(f"\n{V2_RESULTS} not present yet; skipping reproduction and pooling")
        return 0

    v2 = load(V2_RESULTS)
    print(f"\nv2: {len(v2)} episodes from {V2_RESULTS.name}")

    rows = reproduction(v1, v2)
    with (DATA / "v1_reproduction.csv").open("w", newline="", encoding="utf8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print("\n--- L00 reproduction: v1 vs v2 ---")
    print(f"  {'mode':<11}{'controller':<13}{'v1':>7}{'v2':>7}  match  "
          f"{'max dpath':>10}")
    for row in rows:
        print(f"  {row['mode']:<11}{row['controller']:<13}"
              f"{row['v1_k']}/{row['v1_n']:<5}{row['v2_k']}/{row['v2_n']:<5}"
              f"{'YES' if row['outcomes_match'] else 'NO':>6}  "
              f"{row['max_path_delta_m']:>10}")
    matched = sum(r["outcomes_match"] for r in rows)
    print(f"  => {matched}/{len(rows)} cells reproduce v1 exactly")

    pooled_rows = pooled(v2)
    with (DATA / "pooled_rates.csv").open("w", newline="", encoding="utf8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(pooled_rows[0]))
        writer.writeheader()
        writer.writerows(pooled_rows)
    print(f"\nwrote {DATA / 'pooled_rates.csv'} ({len(pooled_rows)} rows)")
    print("\n--- PPO pooled success (n = distinct configurations) ---")
    print(f"  {'layout':<8}{'mode':<11}{'k/n':>8}{'rate':>8}  Wilson 95%")
    for row in pooled_rows:
        if row["group"] != "ppo_pooled":
            continue
        print(f"  {row['layout_id']:<8}{row['mode']:<11}"
              f"{row['k']}/{row['n']:<6}{row['rate']:>8.3f}  "
              f"[{row['wilson_low']:.3f}, {row['wilson_high']:.3f}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
