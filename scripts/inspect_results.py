"""Print per-episode rows from an experiment CSV, filtered by controller."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path,
                        default=PROJECT_ROOT / "results" / "experiments_warehouse"
                        / "experiment_results.csv")
    parser.add_argument("--controller", default=None)
    parser.add_argument("--mode", default=None)
    parser.add_argument("--limit", type=int, default=20)
    args = parser.parse_args()

    rows = list(csv.DictReader(args.file.open(encoding="utf8")))
    if args.controller:
        rows = [r for r in rows if r["controller"] == args.controller]
    if args.mode:
        rows = [r for r in rows if r["mode"] == args.mode]

    print(f"{'id':<22}{'outcome':<11}{'steps':>6}{'path_m':>9}"
          f"{'to_goal':>9}{'clear':>8}{'rmse':>8}")
    for row in rows[:args.limit]:
        print(f"{row['experiment_id']:<22}{row['outcome']:<11}"
              f"{row['control_steps']:>6}{float(row['path_length_m']):>9.2f}"
              f"{float(row['final_goal_distance_m']):>9.2f}"
              f"{float(row['min_truth_clearance_m'] or 0):>8.2f}"
              f"{float(row['velocity_tracking_rmse_mps'] or 0):>8.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
