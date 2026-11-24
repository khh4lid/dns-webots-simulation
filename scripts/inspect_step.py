"""Print one recorded step from results/lidar_attitude.json sector by sector.

Usage::

    python scripts/inspect_step.py --mount stabilised --manoeuvre diagonal --step 1
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path,
                        default=PROJECT_ROOT / "results" / "lidar_attitude.json")
    parser.add_argument("--mount", default="stabilised")
    parser.add_argument("--manoeuvre", default="diagonal")
    parser.add_argument("--step", type=int, default=1)
    args = parser.parse_args()

    data = json.loads(args.file.read_text(encoding="utf8"))
    rows = [r for r in data["steps"]
            if r["mount"] == args.mount and r["manoeuvre"] == args.manoeuvre
            and r["step"] == args.step]
    if not rows:
        print("no such step")
        return 1
    r = rows[0]
    print(f"mount {r['mount']}  manoeuvre {r['manoeuvre']}  step {r['step']}")
    print(f"xy {[round(v, 4) for v in r['xy']]}  altitude {r['altitude_m']:.4f} m")
    print(f"roll {r['roll_deg']:+.3f} deg  pitch {r['pitch_deg']:+.3f} deg  "
          f"yaw {r['yaw_deg']:+.4f} deg  tilt {r['tilt_deg']:.3f} deg")
    print(f"paired RMS {r['paired_rms_m']:.4f} m")
    print("\n sector  bearing  measured  truth    delta   flag")
    for i, (m, t) in enumerate(zip(r["measured_ranges_m"], r["truth_ranges_m"])):
        flag = ""
        if i in r["phantom_clear_sectors"]:
            flag = "PHANTOM-CLEAR"
        elif i in r["too_short_sectors"]:
            flag = "TOO-SHORT"
        print(f" {i:>6}  {i * 30:>5}   {m:>7.3f}  {t:>7.3f}  {m - t:>+7.3f}   {flag}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
