"""Blocking gate for one layout: world check, contract tests, reference traversal.

A layout may not enter the S6 experiment until this passes. Three independent
questions are answered, in order:

1. **Does the world implement the scenario?** (``dns.worldcheck.check``) --
   obstacle t = 0 poses, basicTimeStep, observation interval, lidar beams / FOV
   / cap, arena bounds, start, goal, tolerance.
2. **Do the frozen contracts still hold?** -- observation shape, dtype and
   range; two-frame structure; action bounds and the per-axis speed cap; the
   0.200 s observation interval as *measured simulated time*; the 12-sector
   index convention with 0/3/6/9 = front/right/back/left.
3. **Is the layout physically traversable?** -- an A* waypoint follower, driven
   through Spencer's own ``Navigation`` interface at the same speed cap, must
   reach the goal in Mode B. This is needed because ``direct`` and ``reactive``
   both fail on L00, so a controller failing on a new layout is not by itself
   evidence that the layout is flyable.

The follower is an **environment test tool**, not a navigation method, and is
never reported as a baseline or compared with a controller.

Run::

    python scripts/with_webots.py --boot-wait 12 --timeout 600 \\
        --world worlds/DNS_L03_C.wbt tests/client_layout_gate.py -- --layout L03
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns import worldcheck                                       # noqa: E402
from dns.feasibility import astar, clearance_grid                # noqa: E402
from dns.scenario import Scenario                                # noqa: E402
from dns.sensing import sector_bearings_dns                      # noqa: E402
from dns.spencer_link import (                                   # noqa: E402
    DT, GOAL_TOL, HORIZON, OBS_DIM, RANGE, SECTORS, SPEED, Navigation,
    observation, pack_scan,
)

_ONLY: dict | None = None

SCENARIOS = PROJECT_ROOT / "config" / "scenarios"


# ---------------------------------------------------------------------------
# contract tests
# ---------------------------------------------------------------------------
def contract_tests(backend, scenario: Scenario) -> list[dict]:
    """Every frozen contract, as an executable check with a measured value."""
    checks: list[dict] = []

    def record(name: str, passed: bool, measured, expected, unit=""):
        checks.append({"check": name, "pass": bool(passed),
                       "measured": measured, "expected": expected, "unit": unit})

    # --- timing -----------------------------------------------------------
    steps = scenario.command_period_s * 1000.0 / scenario.basic_timestep_ms
    record("observation_interval_is_integer_timesteps",
           abs(steps - round(steps)) < 1e-9, steps, round(steps), "timesteps")

    env = Navigation(backend)
    obs, info = env.reset(200_000_000, "static")

    # --- observation ------------------------------------------------------
    record("observation_shape", obs.shape == (OBS_DIM,), list(obs.shape),
           [OBS_DIM])
    record("observation_dtype", obs.dtype == np.float32, str(obs.dtype),
           "float32")
    record("observation_finite", bool(np.all(np.isfinite(obs))), True, True)
    record("observation_range_within_unit_box",
           bool(np.all(np.abs(obs) <= 1.0 + 1e-6)),
           round(float(np.abs(obs).max()), 6), "<= 1.0")
    # The two-frame structure: on reset both halves are the same frame, so the
    # current-scan block and the previous-scan block must be identical.
    current, previous = obs[4:4 + 2 * SECTORS], obs[4 + 2 * SECTORS:]
    record("two_frame_structure_identical_on_reset",
           bool(np.allclose(current, previous)), True, True)

    # --- lidar / sector contract -----------------------------------------
    if backend.lidar is not None:
        record("lidar_beams_720",
               backend.lidar.getHorizontalResolution() == 720,
               backend.lidar.getHorizontalResolution(), 720, "beams")
        record("lidar_fov_360deg",
               abs(backend.lidar.getFov() - 2 * math.pi) < 1e-6,
               round(math.degrees(backend.lidar.getFov()), 6), 360.0, "deg")
        record("lidar_range_cap_5m",
               abs(backend.lidar.getMaxRange() - RANGE) < 1e-9,
               backend.lidar.getMaxRange(), RANGE, "m")
    record("sector_count_12", SECTORS == 12, SECTORS, 12, "sectors")

    # Sector index convention: a single synthetic beam placed on each cardinal
    # bearing must land in sector 0 / 3 / 6 / 9. This tests Spencer's pack_scan
    # binning against the documented convention, using his function unmodified.
    bearings = sector_bearings_dns(SECTORS)
    for index, label in ((0, "front"), (3, "right"), (6, "back"), (9, "left")):
        ranges, seen = pack_scan(np.array([bearings[index]]),
                                 np.array([1.234]), np.array([True]))
        hit = int(np.argmax(seen))
        record(f"sector_{index}_is_{label}",
               hit == index and abs(ranges[index] - 1.234) < 1e-9,
               hit, index, "sector index")
    record("sector_bearing_step_30deg",
           abs(math.degrees(bearings[1] - bearings[0]) - 30.0) < 1e-9,
           round(math.degrees(bearings[1] - bearings[0]), 6), 30.0, "deg")

    # --- action bounds and speed cap --------------------------------------
    rejected = False
    try:
        env.step([1.5, 0.0])
    except ValueError:
        rejected = True
    record("action_outside_unit_box_rejected", rejected, rejected, True)

    env.reset(200_000_000, "static")
    before = env.frame.time_s
    env.step([0.5, -0.25])
    measured_dt = env.frame.time_s - before
    record("observation_interval_0p2s_measured",
           abs(measured_dt - DT) < 1e-9, round(measured_dt, 12), DT, "s")
    record("per_axis_speed_cap",
           abs(scenario.max_speed_mps - SPEED) < 1e-9,
           scenario.max_speed_mps, SPEED, "m/s")

    # --- layout invariants vs L00 ----------------------------------------
    reference = Scenario.load(SCENARIOS / "L00_C.json")
    for name, got, want in (
            ("arena_low_xy", list(scenario.low_xy), list(reference.low_xy)),
            ("arena_high_xy", list(scenario.high_xy), list(reference.high_xy)),
            ("start_xy", list(scenario.start_xy), list(reference.start_xy)),
            ("goal_xy", list(scenario.goal_xy), list(reference.goal_xy)),
            ("goal_tolerance_m", scenario.goal_tolerance_m,
             reference.goal_tolerance_m),
            ("cruise_altitude_m", scenario.cruise_altitude_m,
             reference.cruise_altitude_m),
            ("basic_timestep_ms", scenario.basic_timestep_ms,
             reference.basic_timestep_ms),
            ("horizon_steps", HORIZON, 90)):
        record(f"matches_L00_{name}", got == want, got, want)

    return checks


# ---------------------------------------------------------------------------
# reference traversal (environment test tool)
# ---------------------------------------------------------------------------
#: Extra C-space margin the reference path is planned with, on top of the drone
#: radius. A* minimises length, so with only the 0.02 m analysis margin it
#: shaves obstacle corners at the inflation boundary; a physical quadcopter
#: lagging its velocity command then clips them. Planning with 0.12 m of margin
#: gives the tracker room to have tracking error without that being a property
#: of the layout. Measured: at 0.02 m the follower hit rack_a on L00 in 10 steps.
REFERENCE_PLAN_MARGIN_M = 0.12

#: Fraction of the per-axis speed cap the follower uses, scheduled on the local
#: ground-truth clearance: fast where the corridor is open, slow where it is
#: tight. The contract gives an upper bound of 1.2 m/s per axis; flying below it
#: still respects the contract, and the question here is whether the route is
#: traversable at all, not how fast.
#:
#: A single fixed 0.55 scale was tried first and failed on L04: 0.66 m/s over a
#: 9.09 m route needs ~69 of the 90 available control steps before any
#: corner-cutting, and the tracker timed out at 90 steps without ever coming
#: near an obstacle. Scheduling restores the margin in the open sections without
#: raising speed inside the narrow gaps, where tracking error is what matters.
REFERENCE_SPEED_MIN = 0.45
REFERENCE_SPEED_MAX = 0.90
#: Clearance above the drone radius at which the follower reaches full speed.
REFERENCE_SPEED_FULL_AT_M = 0.55

#: Pure-pursuit lookahead along the planned path, metres.
REFERENCE_LOOKAHEAD_M = 0.55

#: Maximum change in each normalised action component per 0.2 s control step.
#:
#: Required, not cosmetic. An A* path on a 0.05 m grid is a staircase, so the
#: raw pure-pursuit command changes direction abruptly at every tread. On L04
#: that produced a lateral demand of about -1.0 m/s in a single step; the
#: cascaded autopilot exceeded its 0.45 rad tilt authority, and the airframe
#: rolled to exactly -1.5703 rad (-90 deg) and locked there -- position frozen
#: for the remaining 50 control steps, reported as a timeout. Measured in
#: results/layout_gate/L04.json before this limit was added.
#:
#: 0.12 per step is 0.144 m/s per 0.2 s = 0.72 m/s^2, comfortably inside the
#: tilt the autopilot can hold. This is a property of the TEST TOOL, not of the
#: layout: no layout was changed to make the tracker succeed.
REFERENCE_SLEW_PER_STEP = 0.12

#: Passes of a 3-point moving average applied to the planned path before
#: tracking. Removes the grid staircase while a clearance re-check guarantees
#: the smoothed path never moves into an obstacle.
REFERENCE_SMOOTH_PASSES = 6


def smooth_path(points: np.ndarray, grid, radius_m: float,
                passes: int = REFERENCE_SMOOTH_PASSES) -> np.ndarray:
    """Moving-average the path, keeping every point at least ``radius_m`` clear.

    Smoothing a C-space path can only ever move it toward the straight line, so
    each candidate point is accepted only if its clearance still exceeds the
    planning radius; otherwise the original point is kept. The path therefore
    stays feasible by construction rather than by assumption.
    """
    result = points.copy()
    for _ in range(passes):
        candidate = result.copy()
        candidate[1:-1] = (result[:-2] + result[1:-1] + result[2:]) / 3.0
        for index in range(1, len(result) - 1):
            i, j = grid.index_of(candidate[index])
            if float(grid.clearance[j, i]) >= radius_m:
                result[index] = candidate[index]
    return result


#: Tracker profiles tried in order until one reaches the goal, as
#: (speed_min, speed_max, slew_per_step). The layout is judged traversable if
#: ANY documented profile reaches the goal, and the profile that succeeded is
#: reported with the result.
#:
#: A ladder rather than one hand-tuned setting, because tuning a single profile
#: until every layout passed would be fitting the test tool to the answer. The
#: profiles go from quick-and-coarse to slow-and-smooth; which one a layout
#: needs is itself informative about how tight that layout is.
REFERENCE_PROFILES = (
    (0.45, 0.90, 0.35),
    (0.40, 0.70, 0.25),
    (0.30, 0.55, 0.18),
    (0.25, 0.40, 0.12),
)


#: Planning margins tried, largest first. A wider margin keeps the reference
#: route further from obstacle corners, which is what a physical tracker needs;
#: it is only available where the corridor is wide enough to hold it.
REFERENCE_MARGINS = (0.22, 0.18, 0.12)


def reference_traversal(backend, scenario: Scenario) -> dict:
    """Try each (margin, tracker profile) pair; report the first that succeeds."""
    attempts = []
    margins = ([float(_ONLY["margin"])] if _ONLY else list(REFERENCE_MARGINS))
    profiles = ([REFERENCE_PROFILES[int(_ONLY["profile"])]] if _ONLY
                else list(REFERENCE_PROFILES))
    for margin in margins:
        for index, profile in enumerate(profiles):
            # A tumble leaves the airframe in an attitude the next reset cannot
            # settle within its 6 s budget, which would abort the whole ladder.
            # Treat that as this rung failing, not as a client error.
            try:
                result = _track_once(backend, scenario, *profile, margin=margin)
            except Exception as error:                           # noqa: BLE001
                attempts.append({"plan_margin_m": margin,
                                 "profile_index": index,
                                 "speed_min": profile[0], "speed_max": profile[1],
                                 "slew_per_step": profile[2],
                                 "outcome": f"aborted: {type(error).__name__}",
                                 "detail": str(error)[:180],
                                 "steps": None, "peak_abs_tilt_rad": None})
                continue
            attempts.append({"plan_margin_m": margin, "profile_index": index,
                             "speed_min": profile[0], "speed_max": profile[1],
                             "slew_per_step": profile[2],
                             "outcome": result["outcome"],
                             "steps": result.get("steps"),
                             "peak_abs_tilt_rad": result.get("peak_abs_tilt_rad")})
            if result["pass"]:
                result["profile_used"] = index
                result["profiles_tried"] = attempts
                return result
    failed = dict(attempts[-1])
    return {"pass": False, "outcome": attempts[-1]["outcome"],
            "steps": attempts[-1]["steps"],
            "reason": "no tracker profile reached the goal",
            "profiles_tried": attempts, **{k: v for k, v in failed.items()
                                           if k not in ("outcome", "steps")}}


def _track_once(backend, scenario: Scenario, speed_min: float,
                speed_max: float, slew: float, *,
                margin: float = REFERENCE_PLAN_MARGIN_M) -> dict:
    """Fly the A* route with a pure-pursuit tracker through Spencer's interface.

    Proves the layout is physically traversable by the actual vehicle under the
    actual command contract and speed cap -- not merely that a grid path exists.
    This is an **environment test tool**: it is given the ground-truth route,
    so it is never a baseline and is never compared with a controller.
    """
    radius = scenario.collision_radius_m + margin
    grid = clearance_grid(scenario, 0.0, 0.05)
    path = astar(grid, scenario.start_xy, scenario.goal_xy, radius)
    if path is None:
        # Fall back to the analysis margin: a layout traversable only with less
        # planning margin is still traversable, and the distinction is reported.
        radius = scenario.collision_radius_m + 0.02
        path = astar(grid, scenario.start_xy, scenario.goal_xy, radius)
        if path is None:
            return {"pass": False, "reason": "A* found no path",
                    "outcome": "infeasible"}
    points = smooth_path(np.asarray(path, dtype=float), grid, radius)
    # Arc length along the planned path, for the lookahead walk.
    segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
    arc = np.concatenate(([0.0], np.cumsum(segment)))

    env = Navigation(backend)
    env.reset(200_000_000, "static")
    trace = []
    commanded = np.zeros(2)
    progress = 0
    peak_tilt = 0.0
    while True:
        position = env.frame.xy
        # Pure pursuit on the dense path: take the closest path point, then walk
        # a fixed arc length forward. Following the path's own curvature keeps
        # the tracker inside the corridor A* planned, which a straight run at a
        # distant simplified waypoint does not.
        #
        # The search starts at the last progress index and never goes backwards:
        # a global argmin can latch onto a different branch where the route
        # doubles back, which commands the tracker away from the goal.
        window = np.linalg.norm(points[progress:] - position, axis=1)
        progress += int(np.argmin(window))
        target_arc = arc[progress] + REFERENCE_LOOKAHEAD_M
        ahead = int(np.searchsorted(arc, target_arc))
        target = points[min(ahead, len(points) - 1)]

        delta = target - position
        distance = float(np.linalg.norm(delta))
        # Speed scheduled on the local ground-truth clearance.
        i, j = grid.index_of(position)
        slack = float(grid.clearance[j, i]) - scenario.collision_radius_m
        ramp = float(np.clip(slack / REFERENCE_SPEED_FULL_AT_M, 0.0, 1.0))
        scale = speed_min + ramp * (speed_max - speed_min)
        action = delta / max(distance, 1e-9) * scale
        # Ease off on the final approach so the tracker settles inside the goal
        # tolerance instead of overshooting it.
        remaining = float(np.linalg.norm(
            np.asarray(scenario.goal_xy) - position))
        if remaining < 0.8:
            action = action * max(0.35, remaining / 0.8)
        action = np.clip(action, -1.0, 1.0)
        # Slew-limit the command so the autopilot is never asked for a step
        # change in lateral velocity it cannot hold within its tilt authority.
        action = commanded + np.clip(action - commanded, -slew, slew)
        commanded = action
        _obs, _reward, ended, cut, info = env.step(action)
        peak_tilt = max(peak_tilt, abs(float(backend.record.roll_rad)),
                        abs(float(backend.record.pitch_rad)))
        trace.append({"t_s": round(env.frame.time_s, 4),
                      "x_m": round(float(env.frame.xy[0]), 4),
                      "y_m": round(float(env.frame.xy[1]), 4),
                      "alt_m": round(float(backend.record.altitude_m), 4),
                      "roll_rad": round(float(backend.record.roll_rad), 4),
                      "pitch_rad": round(float(backend.record.pitch_rad), 4),
                      "ax": round(float(action[0]), 4),
                      "ay": round(float(action[1]), 4),
                      "vx": round(float(env.frame.velocity[0]), 4),
                      "vy": round(float(env.frame.velocity[1]), 4),
                      "clear_m": round(slack + scenario.collision_radius_m, 4)})
        if ended or cut:
            break
    return {"pass": info["outcome"] == "goal", "outcome": info["outcome"],
            "steps": env.steps, "duration_s": round(env.steps * DT, 4),
            "plan_margin_m": round(radius - scenario.collision_radius_m, 4),
            "speed_scale_of_cap": [speed_min, speed_max],
            "speed_schedule": "scaled on local ground-truth clearance",
            "lookahead_m": REFERENCE_LOOKAHEAD_M,
            "slew_per_step": slew,
            "smooth_passes": REFERENCE_SMOOTH_PASSES,
            "peak_abs_tilt_rad": round(peak_tilt, 4),
            "tilt_limit_rad": 0.45,
            "path_points": len(points),
            "final_goal_distance_m": round(info["remaining_m"], 4),
            "trace": trace}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", required=True)
    parser.add_argument("--only", default=None,
                        help="margin:profile_index, e.g. 0.22:3")
    parser.add_argument("--out", type=Path,
                        default=PROJECT_ROOT / "results" / "layout_gate")
    args = parser.parse_args()
    if args.only:
        global _ONLY
        margin, profile = args.only.split(":")
        _ONLY = {"margin": margin, "profile": profile}
    args.out.mkdir(parents=True, exist_ok=True)

    dynamic_path = SCENARIOS / f"{args.layout}_C.json"
    static_path = SCENARIOS / f"{args.layout}_B.json"
    dynamic = Scenario.load(dynamic_path)
    static = Scenario.load(static_path)
    digest = json.loads(dynamic_path.read_text(encoding="utf8"))[
        "provenance"]["scenario_sha256"]

    from dns.backend import WebotsBackend                         # noqa: E402

    backend = WebotsBackend(dynamic)
    world_report = worldcheck.check(backend, dynamic, expect_sha256=digest)

    # Contract tests and the reference traversal both run in Mode B: a parked
    # cart makes them repeatable, and traversability is a static property.
    backend.scenario = static
    backend.obstacle_nodes = backend.obstacle_nodes[:len(static.obstacles)]
    contracts = contract_tests(backend, static)
    traversal = reference_traversal(backend, static)

    failed = [c for c in contracts if not c["pass"]]
    verdict = "PASS" if not failed and traversal["pass"] else "FAIL"
    report = {
        "layout": args.layout,
        "verdict": verdict,
        "scenario_sha256": digest,
        "world_check": world_report,
        "contract_tests": contracts,
        "contract_tests_passed": len(contracts) - len(failed),
        "contract_tests_total": len(contracts),
        "reference_traversal": traversal,
    }
    (args.out / f"{args.layout}.json").write_text(
        json.dumps(report, indent=2), encoding="utf8")

    print(f"[{args.layout}] world check      : OK "
          f"(max t=0 pose error "
          f"{world_report['checks']['obstacle_t0_pose_max_error_m']:.2e} m)")
    print(f"[{args.layout}] contract tests   : "
          f"{len(contracts) - len(failed)}/{len(contracts)} pass")
    for check in failed:
        print(f"    FAIL {check['check']}: measured={check['measured']} "
              f"expected={check['expected']}")
    print(f"[{args.layout}] reference travers: {traversal['outcome']} "
          f"in {traversal.get('steps', 0)} steps "
          f"({traversal.get('duration_s', 0)} s), "
          f"{traversal.get('waypoints', 0)} waypoints")
    print(f"[{args.layout}] VERDICT          : {verdict}")
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
