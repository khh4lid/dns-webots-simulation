"""Print the moving-obstacle tracking table from results/moving_obstacle.json."""
from __future__ import annotations

import json
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]

data = json.loads((PROJECT_ROOT / "results" / "moving_obstacle.json").read_text(encoding="utf8"))
rows = data["steps"]

print("   t   obs_y    drone_xy            bearing  sect  true_clr   meas    delta")
for r in rows[::6]:
    print(f"{r['t']:5.1f}  {r['actual_xy'][1]:+6.3f}  "
          f"[{r['drone_xy'][0]:.3f},{r['drone_xy'][1]:+.3f}]  "
          f"{r['true_bearing_deg']:8.2f}  {r['obstacle_sector']:>4}  "
          f"{r['true_clearance_m']:7.3f}  {r['obstacle_sector_range_m']:6.3f}  "
          f"{r['obstacle_sector_range_m'] - r['true_clearance_m']:+7.3f}")

deltas = [r["obstacle_sector_range_m"] - r["true_clearance_m"] for r in rows]
print(f"\ndelta: min {min(deltas):+.3f}  max {max(deltas):+.3f}  "
      f"mean {sum(deltas) / len(deltas):+.3f}")
print(f"all deltas negative (lidar reads shorter): {all(d < 0 for d in deltas)}")
