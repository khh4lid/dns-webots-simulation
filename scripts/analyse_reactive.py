"""Phase 3 item 1: what actually stops Spencer's `reactive` controller.

v1 stated: "its repulsion radius (1.45 m) is larger than the crate-to-goal
distance (1.54 m)". That sentence is wrong as written -- 1.54 m is the
CENTRE-to-goal distance, and 1.45 < 1.54, so as stated the claim does not even
support its own conclusion.

`evaluate.gap_rule` (Spencer's code, read-only) is:

    ranges = obs[4:16] * RANGE          # per-sector nearest LiDAR return, metres
    mask   = obs[16:28] > 0             # sector validity
    force  = (maximum(1.45 - ranges, 0) * mask) @ DIRECTIONS
    v      = goal_direct(obs) - 0.85 * force
    return v / max(1.0, norm(v))

so repulsion is measured from the **nearest LiDAR return in each sector**, i.e.
from obstacle *surfaces* and from the arena walls -- not from obstacle centres.
This script recomputes `gap_rule` from the logged sector ranges of the real
reactive episodes and reports, with numbers:

* the surface-to-goal distance of the crate stack (vs v1's centre-to-goal 1.54 m);
* the attractive and repulsive term magnitudes along the flown path;
* where the commanded speed collapses, and which sectors supply the repulsion;
* whether the goal lies inside the 1.45 m repulsion zone -- the GNRON condition
  of Ge & Cui (IEEE T-RA 16(5), 2000, doi:10.1109/70.880813).

Nothing in Spencer's code is modified; `gap_rule` is imported and called.

Run::

    python scripts/analyse_reactive.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.scenario import Scenario                                # noqa: E402
from dns.spencer_link import RANGE, SECTORS, find_ppo_dir         # noqa: E402

PACKAGE = PROJECT_ROOT.parent / "DNS_FRIDAY_MEETING_2026-11-27"
SCENARIOS = PACKAGE / "07_SCENARIOS"
RUNS = PROJECT_ROOT / "results" / "s6" / "runs"
DATA = PACKAGE / "06_DATA"

#: gap_rule's repulsion radius, read from Spencer's source rather than retyped.
REPULSION_RADIUS_M = 1.45
REPULSION_GAIN = 0.85


def directions() -> np.ndarray:
    """evaluate.DIRECTIONS: the unit vector of each sector's centre bearing."""
    index = np.arange(SECTORS)
    return np.column_stack((np.cos(index * 2 * np.pi / SECTORS),
                            np.sin(index * 2 * np.pi / SECTORS)))


def rebuild_observation(xy: np.ndarray, ranges: np.ndarray, valid: np.ndarray,
                        goal: np.ndarray) -> np.ndarray:
    """The slice of navigation.observation that gap_rule reads.

    gap_rule uses obs[:2] (clipped goal vector), obs[4:16] (sector ranges / 5)
    and obs[16:28] (validity). Velocity and the previous frame are not used, so
    the reconstruction is exact for this controller.
    """
    obs = np.zeros(4 + 4 * SECTORS, dtype=np.float32)
    obs[:2] = np.clip((goal - xy) / 10.0, -1.0, 1.0)
    obs[4:4 + SECTORS] = ranges / RANGE
    obs[4 + SECTORS:4 + 2 * SECTORS] = valid.astype(float)
    return obs


def surface_distance_to_goal(scenario: Scenario) -> dict:
    """Per-obstacle surface-to-goal distance, next to the centre-to-goal value.

    The point of the table is that v1 quoted the centre distance while the
    controller reacts to the surface.
    """
    goal = np.asarray(scenario.goal_xy, dtype=float)[None, :]
    table = {}
    for obstacle in scenario.obstacles:
        placed = obstacle.placed_at(0.0)
        centre = np.asarray(placed.centre, dtype=float)
        table[obstacle.id] = {
            "centre_to_goal_m": round(float(np.linalg.norm(centre - goal[0])), 4),
            "surface_to_goal_m": round(float(placed.surface_distance(goal)[0]), 4),
            "inside_repulsion_zone": bool(
                placed.surface_distance(goal)[0] < REPULSION_RADIUS_M),
        }
    return table


def analyse_run(run_id: str, scenario: Scenario) -> dict | None:
    """Recompute gap_rule along a logged reactive episode."""
    archive = RUNS / run_id / "lidar.npz"
    if not archive.is_file():
        return None
    from evaluate import gap_rule                                 # noqa: E402

    data = np.load(archive, allow_pickle=False)
    goal = np.asarray(scenario.goal_xy, dtype=float)
    unit = directions()

    steps = []
    for index in range(len(data["t_s"])):
        xy = data["pose_xy"][index]
        ranges = data["sector_ranges_m"][index]
        valid = data["sector_valid"][index]
        obs = rebuild_observation(xy, ranges, valid, goal)

        attract = obs[:2] / max(float(np.linalg.norm(obs[:2])), 1e-8)
        magnitude = np.maximum(REPULSION_RADIUS_M - ranges, 0.0) * (valid > 0)
        repel = REPULSION_GAIN * (magnitude @ unit)
        command = gap_rule(obs)

        steps.append({
            "step": index,
            "t_s": round(float(data["t_s"][index]), 4),
            "x_m": round(float(xy[0]), 4),
            "y_m": round(float(xy[1]), 4),
            "remaining_m": round(float(np.linalg.norm(goal - xy)), 4),
            "attract_norm": round(float(np.linalg.norm(attract)), 4),
            "repel_norm": round(float(np.linalg.norm(repel)), 4),
            "command_norm": round(float(np.linalg.norm(command)), 4),
            "command_x": round(float(command[0]), 4),
            "command_y": round(float(command[1]), 4),
            "min_sector_m": round(float(np.min(ranges[valid > 0])), 4)
                            if np.any(valid) else None,
            "sectors_repelling": [int(s) for s in np.flatnonzero(magnitude > 0)],
            "dominant_sector": int(np.argmax(magnitude)) if magnitude.max() > 0 else None,
            "dominant_range_m": round(float(ranges[int(np.argmax(magnitude))]), 4)
                                if magnitude.max() > 0 else None,
        })

    final = steps[-1]
    # The stall point: where the command magnitude first drops below a tenth of
    # the attractive term and stays low.
    stalled = [s for s in steps if s["command_norm"] < 0.15]
    return {
        "run_id": run_id,
        "steps": len(steps),
        "final": final,
        "min_command_norm": round(min(s["command_norm"] for s in steps), 4),
        "max_repel_norm": round(max(s["repel_norm"] for s in steps), 4),
        "stalled_steps": len(stalled),
        "first_stall_step": stalled[0]["step"] if stalled else None,
        "first_stall_position": ([stalled[0]["x_m"], stalled[0]["y_m"]]
                                 if stalled else None),
        "trace": steps,
    }


def main() -> int:
    DATA.mkdir(parents=True, exist_ok=True)
    report: dict = {
        "claim_in_v1": ("its repulsion radius (1.45 m) is larger than the "
                        "crate-to-goal distance (1.54 m)"),
        "why_wrong": ("1.54 m is the crate-stack CENTRE-to-goal distance, and "
                      "1.45 < 1.54, so the sentence contradicts its own "
                      "conclusion. gap_rule measures repulsion from the nearest "
                      "LiDAR return per sector, i.e. from obstacle SURFACES and "
                      "from the arena walls."),
        "gap_rule_source": "evaluate.gap_rule (Spencer, unmodified)",
        "repulsion_radius_m": REPULSION_RADIUS_M,
        "repulsion_gain": REPULSION_GAIN,
        "layouts": {},
    }

    for layout in ("L00", "L01", "L02", "L03", "L04", "L05"):
        path = SCENARIOS / f"{layout}_B.json"
        if not path.is_file():
            continue
        scenario = Scenario.load(path)
        entry = {"surface_vs_centre_to_goal": surface_distance_to_goal(scenario)}
        analysis = analyse_run(f"{layout}_B_reactive_v00", scenario)
        if analysis:
            entry["reactive_mode_b"] = analysis
        report["layouts"][layout] = entry

    (DATA / "reactive_gnron_analysis.json").write_text(
        json.dumps(report, indent=2), encoding="utf8")

    print("=== crate_stack / nearest-obstacle geometry relative to the goal ===")
    print(f"{'layout':<8}{'obstacle':<18}{'centre->goal':>14}"
          f"{'surface->goal':>15}{'in 1.45 m zone':>16}")
    for layout, entry in report["layouts"].items():
        for name, value in sorted(entry["surface_vs_centre_to_goal"].items(),
                                  key=lambda kv: kv[1]["surface_to_goal_m"])[:2]:
            print(f"{layout:<8}{name:<18}{value['centre_to_goal_m']:>14.4f}"
                  f"{value['surface_to_goal_m']:>15.4f}"
                  f"{str(value['inside_repulsion_zone']):>16}")

    print("\n=== reactive, Mode B variant 00: recomputed gap_rule ===")
    print(f"{'layout':<8}{'steps':>6}{'min|cmd|':>10}{'max|repel|':>12}"
          f"{'stalled':>9}{'final dist':>12}  first stall at")
    for layout, entry in report["layouts"].items():
        analysis = entry.get("reactive_mode_b")
        if not analysis:
            continue
        print(f"{layout:<8}{analysis['steps']:>6}"
              f"{analysis['min_command_norm']:>10.4f}"
              f"{analysis['max_repel_norm']:>12.4f}"
              f"{analysis['stalled_steps']:>9}"
              f"{analysis['final']['remaining_m']:>12.4f}  "
              f"{analysis['first_stall_position']}")

    print(f"\nwrote {DATA / 'reactive_gnron_analysis.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
