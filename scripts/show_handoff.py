"""Print a two-frame handoff packet in a human-readable form.

Usage::

    python scripts/show_handoff.py results/two_frames_webots_physical_lidar.json

Shows the sector table with its bearing labels so the angular convention is
visible at a glance, and re-checks the things the packet asserts about itself:
the frame separation, the action -> commanded-velocity scaling, and whether the
reported velocity is a measurement rather than the command echoed back.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]

SECTOR_LABEL = {0: "forward", 3: "right", 6: "back", 9: "left"}


def sector_table(ranges, valid) -> str:
    lines = ["    sector  bearing  range_m   valid  note"]
    for i, (r, v) in enumerate(zip(ranges, valid)):
        note = SECTOR_LABEL.get(i, "")
        if v and r >= 5.0 - 1e-6:
            note = (note + " clear to cap").strip()
        elif not v:
            note = (note + " UNOBSERVED").strip()
        lines.append(f"    {i:>6}  {i * 30:>5} deg  {r:>7.3f}  {str(bool(v)):>5}  {note}")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("packet", type=Path)
    args = parser.parse_args()
    path = args.packet if args.packet.is_absolute() else PROJECT_ROOT / args.packet
    p = json.loads(path.read_text(encoding="utf8"))

    print(f"file     : {path}")
    print(f"schema   : {p['schema']}")
    print(f"source   : {p.get('source', '?')}   scenario: {p.get('scenario', '?')}")
    print(f"dynamics : {p['dynamics']}      sensing: {p['sensing']}")
    print(f"seed     : {p.get('seed')}")

    print(f"mount    : {p.get('lidar_mount', '?')}   beams: {p.get('lidar_beams', '?')}")

    # ``frames`` is the strict navigation.Frame payload; ``dns_records`` carries
    # the provenance fields Spencer's Frame dataclass would reject.
    frames = p["frames"]
    rich = p.get("dns_records", frames)
    for i, (f, r) in enumerate(zip(frames, rich)):
        sim_t = r.get("sim_time_s")
        print(f"\n--- frame {i} (episode t={f['time_s']:.6f} s"
              + (f", wall-clock sim t={sim_t:.3f} s)" if sim_t is not None else ")"))
        print(f"    xy          : {[round(v, 5) for v in f['xy']]} m")
        remaining = r.get("remaining_m")
        print(f"    goal_xy     : {[round(v, 5) for v in f['goal_xy']]} m"
              + (f"   remaining {remaining:.4f} m" if remaining is not None else ""))
        if "commanded_velocity_mps" in r:
            print(f"    commanded v : "
                  f"{[round(v, 4) for v in r['commanded_velocity_mps']]} m/s")
        print(f"    measured  v : {[round(v, 5) for v in f['velocity']]} m/s")
        if "altitude_m" in r:
            print(f"    altitude    : {r['altitude_m']:.5f} m    "
                  f"roll {math.degrees(r.get('roll_rad', 0.0)):+.3f} "
                  f"pitch {math.degrees(r.get('pitch_rad', 0.0)):+.3f} "
                  f"yaw {math.degrees(r.get('yaw_rad', 0.0)):+.3f} deg")
        print(f"    collision   : {f['collision']}"
              + (f"   (webots contact: {r['contact']})" if "contact" in r else ""))
        print(sector_table(f["ranges"], f["valid"]))

    print("\n--- self-consistency")
    dt = frames[1]["time_s"] - frames[0]["time_s"]
    print(f"    frame separation      : {dt:.9f} s "
          f"({'OK' if abs(dt - 0.2) < 1e-6 else 'MISMATCH'}, expected 0.200000000)")

    action = p["action"]
    commanded = p["commanded_velocity_mps"]
    expect = [a * 1.2 for a in action]
    ok = all(abs(c - e) < 1e-9 for c, e in zip(commanded, expect))
    print(f"    action {action} x 1.2  : {commanded} "
          f"({'OK' if ok else 'MISMATCH'})")

    measured = frames[1]["velocity"]
    echoed = all(abs(m - c) < 1e-9 for m, c in zip(measured, commanded))
    print(f"    velocity is measured  : {'NO - command echoed back' if echoed else 'yes'}"
          f"  (measured {[round(v, 5) for v in measured]} vs commanded {commanded})")

    n_valid = sum(bool(v) for v in frames[1]["valid"])
    print(f"    sectors observed      : {n_valid}/12")
    obs = p.get("expected", {}).get("next_obs")
    if obs is not None:
        print(f"    observation length    : {len(obs)} "
              f"({'OK' if len(obs) == 52 else 'MISMATCH'}, expected 52)")

    # Spencer's verifier does Frame(**frames[i]) and reads packet["scene"], so
    # an extra or missing frame key breaks it with a TypeError/KeyError.
    expected_keys = {"time_s", "xy", "goal_xy", "velocity",
                     "ranges", "valid", "collision"}
    problems = []
    if "scene" not in p:
        problems.append("missing top-level 'scene'")
    for i, f in enumerate(frames):
        extra = set(f) - expected_keys
        missing = expected_keys - set(f)
        if extra:
            problems.append(f"frame {i} has extra keys {sorted(extra)}")
        if missing:
            problems.append(f"frame {i} missing {sorted(missing)}")
    print(f"    navigation.Frame keys : {'OK' if not problems else 'INCOMPATIBLE'}")
    for problem in problems:
        print(f"      ! {problem}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
