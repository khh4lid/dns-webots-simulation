"""Compute and tabulate the per-layout descriptors for the shift ladder.

Writes, into the v2 package:

``06_DATA/scenario_table.csv``
    One row per layout x mode: hashes, obstacle counts, A* feasibility and
    length, tortuosity, measured corridor width, occupied area, line-of-sight
    and intrusion, cart interaction, and dissimilarity to L00.
``06_DATA/asset_library.csv``
    One row per distinct asset placement across all layouts, so every body in
    every scene is accounted for with its size, height and purpose.

"How different" (dissimilarity to L00) and "how hard" (corridor width,
tortuosity, occupied area) are deliberately separate columns: a mirrored layout
is maximally different and exactly as hard, and conflating the two would make
that impossible to say.

Run::

    python scripts/describe_layouts.py
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dns.feasibility import describe                            # noqa: E402
from dns.scenario import Scenario                               # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = PROJECT_ROOT / "config" / "scenarios"
DATA = PROJECT_ROOT / "results" / "v2_S6"

LAYOUTS = ("L00", "L01", "L02", "L03", "L04", "L05")
MODES = ("C", "B")

COLUMNS = [
    "layout_id", "mode", "layout_title", "scenario_name", "test_only",
    "scenario_sha256", "seed", "draw_attempts",
    "n_obstacles", "n_static", "n_moving",
    "feasible", "astar_length_m", "straight_line_m", "tortuosity",
    "min_corridor_width_m", "min_corridor_width_enroute_m",
    "bottleneck_clearance_m", "min_path_clearance_m",
    "occupied_area_pct", "start_goal_line_of_sight", "line_blocked",
    "line_intrusion_m", "line_cspace_penetration_m", "line_min_segment_clearance_m",
    "cart_present", "cart_blocking_fraction", "cart_crossings", "cart_period_s",
    "iou_to_L00", "dissimilarity_to_L00", "mean_obstacle_displacement_m",
    "inflation_radius_m", "grid_step_m",
]


def main() -> int:
    DATA.mkdir(parents=True, exist_ok=True)
    reference = Scenario.load(SCENARIOS / "L00_C.json")

    rows = []
    assets = []
    for layout in LAYOUTS:
        for mode in MODES:
            path = SCENARIOS / f"{layout}_{mode}.json"
            if not path.is_file():
                print(f"SKIP {path.name}: not generated")
                continue
            raw = json.loads(path.read_text(encoding="utf8"))
            provenance = raw.get("provenance", {})
            scenario = Scenario.load(path)
            facts, _path, _grid = describe(scenario, reference=reference)

            row = {c: "" for c in COLUMNS}
            row.update({
                "layout_id": layout,
                "mode": mode,
                "layout_title": provenance.get("layout_title", ""),
                "scenario_name": scenario.name,
                "test_only": provenance.get("test_only", ""),
                "scenario_sha256": provenance.get("scenario_sha256", ""),
                "seed": provenance.get("seed") if provenance.get("seed") is not None else "",
                "draw_attempts": provenance.get("draw_attempts", ""),
            })
            for key, value in facts.items():
                target = {"iou_to_reference": "iou_to_L00",
                          "dissimilarity_to_reference": "dissimilarity_to_L00",
                          "mean_obstacle_displacement_m": "mean_obstacle_displacement_m",
                          }.get(key, key)
                if target in row:
                    row[target] = "" if value is None else value
            rows.append(row)

            if mode == "C":
                for obstacle in scenario.obstacles:
                    moving = (obstacle.motion.type != "static"
                              and obstacle.motion.amplitude_m)
                    assets.append({
                        "layout_id": layout,
                        "obstacle_id": obstacle.id,
                        "label": obstacle.label,
                        "shape": obstacle.shape,
                        "x_m": round(obstacle.xy[0], 4),
                        "y_m": round(obstacle.xy[1], 4),
                        "size_x_m": round(obstacle.size_xy[0], 4),
                        "size_y_m": round(obstacle.size_xy[1], 4),
                        "height_m": obstacle.height_m,
                        "yaw_rad": round(obstacle.yaw_rad, 6),
                        "moving": int(bool(moving)),
                        "motion_type": obstacle.motion.type,
                        "amplitude_m": obstacle.motion.amplitude_m,
                        "speed_mps": obstacle.motion.speed_mps,
                        "purpose": obstacle.purpose,
                    })

    with (DATA / "scenario_table.csv").open("w", newline="", encoding="utf8") as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    with (DATA / "asset_library.csv").open("w", newline="", encoding="utf8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(assets[0]))
        writer.writeheader()
        writer.writerows(assets)

    print(f"wrote {DATA / 'scenario_table.csv'} ({len(rows)} rows)")
    print(f"wrote {DATA / 'asset_library.csv'} ({len(assets)} rows)\n")

    header = (f"{'layout':<7}{'mode':<5}{'feas':<6}{'A*_m':>7}{'tort':>6}"
              f"{'corr_m':>8}{'enrt_m':>8}{'occ%':>7}{'LOS':>5}{'intr_m':>8}"
              f"{'cartblk':>8}{'xings':>6}{'dissim':>8}{'displ_m':>9}")
    print(header)
    print("-" * len(header))
    for row in rows:
        print(f"{row['layout_id']:<7}{row['mode']:<5}"
              f"{str(row['feasible']):<6}"
              f"{row['astar_length_m'] or 0:>7.3f}"
              f"{row['tortuosity'] or 0:>6.3f}"
              f"{row['min_corridor_width_m'] or 0:>8.3f}"
              f"{row['min_corridor_width_enroute_m'] or 0:>8.3f}"
              f"{row['occupied_area_pct']:>7.2f}"
              f"{str(row['start_goal_line_of_sight'])[:1]:>5}"
              f"{row['line_intrusion_m']:>8.3f}"
              f"{row['cart_blocking_fraction'] or 0:>8.3f}"
              f"{row['cart_crossings'] or 0:>6}"
              f"{row['dissimilarity_to_L00']:>8.3f}"
              f"{(row['mean_obstacle_displacement_m'] if row['mean_obstacle_displacement_m'] != '' else float('nan')):>9.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
