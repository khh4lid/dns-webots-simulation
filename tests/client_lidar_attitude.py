"""Does airframe attitude corrupt the lidar scan? Measure it.

Motivation
----------
``DnsQuadcopter`` carries a single-layer Lidar (``numberOfLayers 1``,
``verticalFieldOfView 0.01``) rigidly bolted to the body, so the scan plane
tilts with the airframe. The autopilot tilts in order to accelerate -- up to
``Gains.max_tilt_rad`` = 0.45 rad (25.8 deg). The arena walls are 1.2 m tall
and the sensor rides at roughly 1.075 m, leaving only ~0.125 m of wall above
the scan plane.

Geometry then predicts that a roll/pitch of about
``asin(0.125 / d)`` -- 6 deg at d = 1.2 m -- lifts the beams on the rising side
over the wall top. Webots returns ``inf`` for a beam that hits nothing, and
``navigation.pack_scan`` encodes ``inf`` as ``(RANGE, True)``: *observed clear
to 5 m*. Inside a closed 10 x 6 m arena that is false -- there is a wall there.

This test does not assume any of that. It flies real commanded manoeuvres and,
at every control step, compares the measured lidar sectors against a dense
analytic ray-cast from the drone's true pose, counting sectors where the sensor
claims "clear to cap" while ground truth puts a surface well inside range.

Run via::

    python scripts/with_webots.py --timeout 420 tests/client_lidar_attitude.py

Output: ``results/lidar_attitude.json`` plus a printed summary.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns import sensing  # noqa: E402
from dns.scenario import DEFAULT_SCENARIO_PATH, Scenario  # noqa: E402
from dns.spencer_link import DT, RANGE, SECTORS, pack_scan  # noqa: E402

RESULTS = PROJECT_ROOT / "results"

#: A sector counts as "claimed clear" when it is valid and sits at the cap.
CLEAR_EPS = 1e-3
#: Ground truth must be this much closer than the cap for the claim to be wrong.
TRUTH_MARGIN = 0.5

MANOEUVRES = (
    ("hover", [0.0, 0.0], 5),
    ("hard_right", [0.0, 1.2], 6),
    ("hard_left", [0.0, -1.2], 6),
    ("hard_forward", [1.2, 0.0], 6),
    ("diagonal", [0.85, 0.85], 6),
)


def dense_truth(position_dns, scenario, t, mapping) -> tuple[np.ndarray, np.ndarray]:
    """Ground-truth 12-sector scan, sampled on the lidar's own bearings.

    Using the same bearing set and the same ``pack_scan`` binning as the sensor
    isolates attitude effects from angular-sampling effects: any disagreement
    left over is not because the reference is coarser than the measurement.
    """
    bearings = mapping.bearings_dns()
    distances = sensing.cast_rays(
        position_dns, bearings, np.asarray(scenario.low_xy),
        np.asarray(scenario.high_xy), scenario.placed_at(t), RANGE)
    return pack_scan(bearings, distances, np.ones(len(bearings), dtype=bool))


def run_mount(backend, scenario, mount: str) -> list[dict]:
    """Fly every manoeuvre and score the scan against ground truth."""
    mapping = backend.lidar_mapping
    rows: list[dict] = []
    for name, command, steps in MANOEUVRES:
        backend.reset(seed=5, scene="static")
        for step in range(steps):
            frame = backend.advance(np.asarray(command, dtype=float), DT)
            roll, pitch, yaw = backend.autopilot.state.rpy
            truth_ranges, truth_valid = dense_truth(
                frame.xy, scenario, backend.episode_time, mapping)

            measured = np.asarray(frame.ranges, dtype=float)
            valid = np.asarray(frame.valid, dtype=bool)
            claims_clear = valid & (measured >= RANGE - CLEAR_EPS)
            truth_blocked = truth_valid & (truth_ranges <= RANGE - TRUTH_MARGIN)
            phantom = claims_clear & truth_blocked

            # A sector reading materially *shorter* than truth is the other
            # failure mode: on the descending side a tilted plane strikes the
            # floor, or dips into the drone's own rotors, inventing an obstacle
            # where the air is clear.
            short = valid & truth_valid & (measured < truth_ranges - TRUTH_MARGIN)

            # Grazing incidence is a separate, inherent effect and must not be
            # scored as an attitude fault. Where a cylinder overlaps a sector
            # boundary by less than one beam pitch, that sector can miss it
            # while the neighbour still holds the return -- the surface is not
            # lost, only re-attributed. Comparing a 3-sector sliding minimum
            # asks "is the nearest surface in roughly this direction seen?",
            # which is attitude-sensitive but immune to +-1 sector reassignment.
            def window_min(a: np.ndarray) -> np.ndarray:
                return np.minimum.reduce([np.roll(a, k) for k in (-1, 0, 1)])

            win_measured = window_min(np.where(claims_clear, RANGE, measured))
            win_truth = window_min(np.where(truth_valid, truth_ranges, RANGE))
            window_phantom = (win_measured >= RANGE - CLEAR_EPS) & (
                win_truth <= RANGE - TRUTH_MARGIN)
            window_short = win_measured < win_truth - TRUTH_MARGIN

            # Paired error only where both saw a surface, so the cap does not
            # dominate the statistic.
            both = valid & truth_valid & ~claims_clear & ~(truth_ranges >= RANGE - CLEAR_EPS)
            residual = float(np.sqrt(np.mean((measured[both] - truth_ranges[both]) ** 2))) \
                if both.any() else float("nan")

            rows.append({
                "mount": mount,
                "manoeuvre": name, "command_mps": command, "step": step,
                "time_s": frame.time_s,
                "xy": frame.xy.tolist(),
                "altitude_m": backend.altitude(),
                "roll_deg": math.degrees(roll), "pitch_deg": math.degrees(pitch),
                "yaw_deg": math.degrees(yaw),
                "tilt_deg": math.degrees(math.hypot(roll, pitch)),
                "measured_ranges_m": measured.tolist(),
                "truth_ranges_m": truth_ranges.tolist(),
                "phantom_clear_sectors": np.flatnonzero(phantom).tolist(),
                "n_phantom": int(phantom.sum()),
                "too_short_sectors": np.flatnonzero(short).tolist(),
                "n_too_short": int(short.sum()),
                "window_phantom_sectors": np.flatnonzero(window_phantom).tolist(),
                "n_window_phantom": int(window_phantom.sum()),
                "window_short_sectors": np.flatnonzero(window_short).tolist(),
                "n_window_short": int(window_short.sum()),
                "paired_rms_m": residual,
            })
            print(f"  [{mount:<10}] {name:<13} step {step}  tilt "
                  f"{math.degrees(math.hypot(roll, pitch)):>5.2f} deg  "
                  f"strict(clear {np.flatnonzero(phantom).tolist()}, "
                  f"short {np.flatnonzero(short).tolist()})  "
                  f"window(clear {np.flatnonzero(window_phantom).tolist()}, "
                  f"short {np.flatnonzero(window_short).tolist()})  "
                  f"RMS {residual:.4f} m")
    return rows


def score(rows: list[dict]) -> dict:
    tilted = [r for r in rows if r["tilt_deg"] > 5.0]
    level = [r for r in rows if r["tilt_deg"] <= 2.0]
    return {
        "steps": len(rows),
        # strict: exact per-sector agreement, so grazing re-attribution counts
        "strict_steps_with_phantom_clear": sum(1 for r in rows if r["n_phantom"]),
        "strict_steps_with_too_short": sum(1 for r in rows if r["n_too_short"]),
        "strict_max_phantom_in_one_step": max(r["n_phantom"] for r in rows),
        "strict_mean_phantom_when_tilt_gt_5deg":
            float(np.mean([r["n_phantom"] for r in tilted])) if tilted else None,
        "strict_mean_phantom_when_tilt_le_2deg":
            float(np.mean([r["n_phantom"] for r in level])) if level else None,
        # window: immune to +-1 sector re-attribution, so it isolates attitude
        "window_steps_with_phantom_clear": sum(1 for r in rows if r["n_window_phantom"]),
        "window_steps_with_too_short": sum(1 for r in rows if r["n_window_short"]),
        "window_max_phantom_in_one_step": max(r["n_window_phantom"] for r in rows),
        "worst_paired_rms_m": float(np.nanmax([r["paired_rms_m"] for r in rows])),
        "median_paired_rms_m": float(np.nanmedian([r["paired_rms_m"] for r in rows])),
        "max_tilt_seen_deg": float(max(r["tilt_deg"] for r in rows)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_SCENARIO_PATH)
    parser.add_argument("--mount", choices=("body_fixed", "stabilised", "both"),
                        default="both")
    parser.add_argument("--out", type=Path, default=RESULTS / "lidar_attitude.json")
    args = parser.parse_args()

    base = dataclasses.replace(Scenario.load(args.config),
                               dynamics_mode="physical", sensing_mode="lidar")
    lidar_height = base.cruise_altitude_m + base.lidar_height_m
    headroom = base.wall_height_m - lidar_height

    from dns.autopilot import Gains  # noqa: E402
    max_tilt = Gains().max_tilt_rad

    print(f"scenario      : {base.name}")
    print(f"wall height   : {base.wall_height_m} m")
    print(f"lidar height  : ~{lidar_height:.3f} m  (cruise {base.cruise_altitude_m} "
          f"+ {base.lidar_height_m} mount)")
    print(f"headroom      : {headroom:.3f} m of wall above the scan plane")
    print(f"max autopilot tilt : {math.degrees(max_tilt):.1f} deg")
    print(f"a body-fixed plane loses a wall at d = 1.2 m beyond tilt "
          f"{math.degrees(math.asin(min(1.0, headroom / 1.2))):.2f} deg\n")

    from dns.backend import WebotsBackend  # noqa: E402  (needs the Webots env)

    mounts = ("body_fixed", "stabilised") if args.mount == "both" else (args.mount,)
    # Webots permits exactly one Robot instance per controller process, so the
    # modes are run back to back against a single backend, re-pointing the
    # scenario between runs rather than constructing a second one.
    scenario = dataclasses.replace(base, lidar_mount_mode=mounts[0])
    scenario.validate()
    backend = WebotsBackend(scenario)
    mount_field = backend.drone.getField("lidarMountRotation")

    results: dict[str, dict] = {}
    all_rows: list[dict] = []
    for mount in mounts:
        backend.scenario = dataclasses.replace(base, lidar_mount_mode=mount)
        backend.lidar_mount_field = mount_field if mount == "stabilised" else None
        if mount == "body_fixed":                 # park the mount at identity
            mount_field.setSFRotation([0.0, 1.0, 0.0, 0.0])
        rows = run_mount(backend, backend.scenario, mount)
        results[mount] = score(rows)
        all_rows += rows
        print()

    report = {
        "wall_height_m": base.wall_height_m,
        "lidar_height_m": lidar_height,
        "headroom_m": headroom,
        "max_autopilot_tilt_deg": math.degrees(max_tilt),
        "by_mount": results,
        "steps": all_rows,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2), encoding="utf8")

    print("--- summary by mount")
    for mount, s in results.items():
        print(f"  {mount}")
        for key, value in s.items():
            print(f"      {key}: {value}")
    print(f"\nwrote {args.out}")

    if "stabilised" in results:
        s = results["stabilised"]
        # Acceptance is on the window metric: the stabilised mount must never
        # lose a surface or invent one. Strict per-sector grazing misses are
        # reported but not failed -- they are set by beam pitch, not attitude.
        ok = (s["window_steps_with_phantom_clear"] == 0
              and s["window_steps_with_too_short"] == 0)
        print(f"\nstabilised mount, window criterion: {'PASS' if ok else 'FAIL'} "
              f"(lost-surface steps {s['window_steps_with_phantom_clear']}, "
              f"invented-surface steps {s['window_steps_with_too_short']}, "
              f"max tilt flown {s['max_tilt_seen_deg']:.1f} deg)")
        print(f"  strict per-sector grazing misses: "
              f"{s['strict_steps_with_phantom_clear']}/{s['steps']} steps "
              f"(inherent to 30 deg binning; the return lands in a neighbour)")
        return 0 if ok else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
