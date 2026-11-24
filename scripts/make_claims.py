"""Build CLAIMS_EVIDENCE.csv: every number in the v2 material, traced to its source.

One row per claim: claim -> value -> unit -> n -> data file -> script -> figure
-> run IDs. Values are READ FROM THE DATA, never retyped, so the table cannot
drift from the results it indexes.

Run::

    python scripts/make_claims.py
"""
from __future__ import annotations

import csv
import json
from collections import Counter
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PACKAGE = PROJECT_ROOT.parent / "DNS_FRIDAY_MEETING_2026-11-27"
DATA = PACKAGE / "06_DATA"
EVIDENCE = PACKAGE / "10_EVIDENCE"

COLUMNS = ["claim_id", "claim", "value", "unit", "n", "data_file", "script",
           "figure", "run_ids", "appears_in"]


def read(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open(encoding="utf8") as handle:
        return list(csv.DictReader(handle))


def main() -> int:
    rows: list[dict] = []

    def add(cid, claim, value, unit, n, data_file, script, figure,
            run_ids="", appears_in="report,slides,talking_points"):
        rows.append({"claim_id": cid, "claim": claim, "value": value,
                     "unit": unit, "n": n, "data_file": data_file,
                     "script": script, "figure": figure, "run_ids": run_ids,
                     "appears_in": appears_in})

    results = read(DATA / "results_v2.csv")
    pooled = read(DATA / "pooled_rates.csv")
    table = {r["layout_id"]: r for r in read(DATA / "scenario_table.csv")
             if r["mode"] == "C"}
    reproduction = read(DATA / "v1_reproduction.csv")

    # ---- Phase 0 -------------------------------------------------------
    manifest = (EVIDENCE / "BASELINE_HASHES.txt")
    entries = sum(1 for line in manifest.read_text(encoding="utf8").splitlines()
                  if line and not line.startswith("#")) if manifest.is_file() else 0
    add("P0-1", "Files hashed in the frozen baseline manifest", entries,
        "files", entries, "10_EVIDENCE/BASELINE_HASHES.txt", "scripts/freeze.py", "")
    if reproduction:
        matched = sum(int(r["outcomes_match"]) for r in reproduction)
        worst = max(float(r["max_path_delta_m"] or 0) for r in reproduction)
        add("P0-2", "L00 cells reproducing v1 results.csv exactly",
            f"{matched}/{len(reproduction)}", "cells", len(reproduction),
            "06_DATA/v1_reproduction.csv", "tools/analyse_v1.py", "E9")
        add("P0-3", "Largest per-episode path-length difference, v1 vs v2 rerun",
            worst, "m", 90, "06_DATA/v1_reproduction.csv",
            "tools/analyse_v1.py", "E9")
    distinct = DATA / "v1_distinctness.json"
    if distinct.is_file():
        report = json.loads(distinct.read_text(encoding="utf8"))
        cell = report["cells"].get("B_static/ppo_seed7", {})
        add("P0-4", "Distinct configurations per v1 Mode B cell",
            cell.get("distinct_configurations", ""), "configurations", 3,
            "06_DATA/v1_distinctness.json", "tools/analyse_v1.py", "")
        cellc = report["cells"].get("C_dynamic/ppo_seed7", {})
        add("P0-5", "Distinct configurations per v1 Mode C cell",
            cellc.get("distinct_configurations", ""), "configurations", 15,
            "06_DATA/v1_distinctness.json", "tools/analyse_v1.py", "")

    # ---- Phase 1: layouts ---------------------------------------------
    for layout, meta in sorted(table.items()):
        add(f"P1-{layout}-corridor",
            f"{layout} en-route corridor width",
            meta.get("min_corridor_width_enroute_m", ""), "m", 1,
            "06_DATA/scenario_table.csv", "scripts/describe_layouts.py", "F1,E1")
        add(f"P1-{layout}-dissim",
            f"{layout} dissimilarity to L00 (1 - IoU of static footprints)",
            meta.get("dissimilarity_to_L00", ""), "dimensionless", 1,
            "06_DATA/scenario_table.csv", "scripts/describe_layouts.py", "F1,E1,E3")
        add(f"P1-{layout}-astar", f"{layout} A* reference path length",
            meta.get("astar_length_m", ""), "m", 1,
            "06_DATA/scenario_table.csv", "scripts/describe_layouts.py", "F1,E1")
        add(f"P1-{layout}-intrusion",
            f"{layout} straight start-goal line intrusion depth",
            meta.get("line_intrusion_m", ""), "m", 1,
            "06_DATA/scenario_table.csv", "scripts/describe_layouts.py", "E1")
        add(f"P1-{layout}-cartblock",
            f"{layout} fraction of the cart period blocking the A* route",
            meta.get("cart_blocking_fraction", ""), "dimensionless", 1,
            "06_DATA/scenario_table.csv", "scripts/describe_layouts.py", "E1")

    # ---- Phase 2: results ----------------------------------------------
    for entry in pooled:
        if entry["group"] != "ppo_pooled":
            continue
        layout, mode = entry["layout_id"], entry["mode"]
        ids = [r["run_id"] for r in results
               if r["layout_id"] == layout and r["mode"] == mode
               and r["controller_kind"] == "ppo_checkpoint"
               and int(r["distinct_configuration"]) == 1]
        add(f"P2-{layout}-{mode}",
            f"Pooled PPO success, {layout} {mode}",
            f"{entry['k']}/{entry['n']} = {entry['rate']}", "rate",
            entry["n"], "06_DATA/pooled_rates.csv", "tools/analyse_v1.py", "F2",
            run_ids=f"{len(ids)} runs: {ids[0]}…{ids[-1]}" if ids else "")
        add(f"P2-{layout}-{mode}-ci",
            f"Wilson 95% interval, pooled PPO {layout} {mode}",
            f"[{entry['wilson_low']}, {entry['wilson_high']}]", "rate",
            entry["n"], "06_DATA/pooled_rates.csv", "tools/analyse_v1.py", "F2")

    if results:
        add("P2-total", "Episodes run in the S6 experiment", len(results),
            "episodes", len(results), "06_DATA/results_v2.csv",
            "tools/run_s6.py", "F2")
        counts = Counter(r["outcome"] for r in results)
        for outcome, count in sorted(counts.items()):
            add(f"P2-outcome-{outcome}", f"Episodes ending in {outcome}",
                count, "episodes", len(results), "06_DATA/results_v2.csv",
                "tools/run_s6.py", "F2")
        tilt = sum(int(r["tilt_limit_exceeded"]) for r in results)
        add("P2-tilt", "Episodes exceeding 0.50 rad airframe tilt", tilt,
            "episodes", len(results), "06_DATA/results_v2.csv",
            "tools/run_s6.py", "")

    # ---- Phase 3: corrections ------------------------------------------
    gnron = DATA / "reactive_gnron_analysis.json"
    if gnron.is_file():
        report = json.loads(gnron.read_text(encoding="utf8"))
        l00 = report["layouts"]["L00"]["surface_vs_centre_to_goal"]["crate_stack"]
        add("P3-1a", "crate_stack CENTRE-to-goal distance on L00 (v1 quoted this)",
            l00["centre_to_goal_m"], "m", 1,
            "06_DATA/reactive_gnron_analysis.json", "scripts/analyse_reactive.py",
            "E13")
        add("P3-1b", "crate_stack SURFACE-to-goal distance on L00 (what gap_rule sees)",
            l00["surface_to_goal_m"], "m", 1,
            "06_DATA/reactive_gnron_analysis.json", "scripts/analyse_reactive.py",
            "E13")
        add("P3-1c", "gap_rule repulsion radius", report["repulsion_radius_m"],
            "m", 1, "DroneNavPPO/evaluate.py (read-only)",
            "scripts/analyse_reactive.py", "E13")
        run = report["layouts"]["L00"].get("reactive_mode_b", {})
        if run:
            add("P3-1d", "Peak repulsive term magnitude, reactive on L00 "
                         "(attractive term is a unit vector)",
                run["max_repel_norm"], "dimensionless", run["steps"],
                "06_DATA/reactive_gnron_analysis.json",
                "scripts/analyse_reactive.py", "E13",
                run_ids="L00_B_reactive_v00")
            add("P3-1e", "Final distance to goal, reactive on L00 Mode B",
                run["final"]["remaining_m"], "m", 1,
                "06_DATA/reactive_gnron_analysis.json",
                "scripts/analyse_reactive.py", "E13",
                run_ids="L00_B_reactive_v00")

    # ---- gates ----------------------------------------------------------
    summary = PROJECT_ROOT / "results" / "layout_gate" / "SUMMARY.json"
    if summary.is_file():
        gates = json.loads(summary.read_text(encoding="utf8"))
        for layout, entry in sorted(gates.items()):
            add(f"GATE-{layout}-contracts",
                f"{layout} contract tests passed",
                entry.get("contract_tests", ""), "tests", 26,
                "results/layout_gate/*.json", "tests/client_layout_gate.py", "")
            add(f"GATE-{layout}-pose",
                f"{layout} maximum obstacle t=0 pose error vs scenario",
                entry.get("max_t0_pose_error_m", ""), "m",
                entry.get("n_obstacles", ""), "results/layout_gate/*.json",
                "dns/worldcheck.py", "")

    DATA.mkdir(parents=True, exist_ok=True)
    target = EVIDENCE / "CLAIMS_EVIDENCE.csv"
    with target.open("w", newline="", encoding="utf8") as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {target} ({len(rows)} claims)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
