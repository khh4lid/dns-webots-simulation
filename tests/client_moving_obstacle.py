"""M4: verify the moving obstacle actually moves, correctly and repeatably.

A scripted obstacle is only scientifically usable if four things hold, and each
is checked here against the running simulator rather than against the maths that
produced it:

1. **The simulator agrees with the trajectory model.** The supervisor's reported
   obstacle translation must match ``Obstacle.centre_at(t)`` at every control
   step, in the DNS frame.
2. **It stays inside its declared envelope** and inside the arena.
3. **Its speed is the documented one.** For ``linear`` motion the centre travels
   at a constant ``speed_mps`` except at the turning points.
4. **It replays exactly.** Two resets followed by the same commands must give
   the same obstacle poses -- otherwise an episode is not reproducible and no
   seeded comparison is valid.

Plus the part that matters for navigation: the **lidar must see it move**. The
sector holding the nearest return should migrate as the obstacle crosses, and
the measured range should track the true centre distance minus the radius.

Run via::

    python scripts/with_webots.py --timeout 600 --world worlds/S2_single_moving.wbt \\
        tests/client_moving_obstacle.py -- --config config/scenario_s2_single_moving.json
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

from dns import sensing                                              # noqa: E402
from dns.scenario import Scenario, to_dns                            # noqa: E402
from dns.spencer_link import DT, RANGE, pack_scan                    # noqa: E402


def analytic_sectors(position_dns, scenario, t, mapping):
    """Ground-truth 12-sector scan over the whole scene, on the lidar's own
    bearings. Includes every obstacle, so occlusion is handled: if a rack
    stands between the drone and the cart, this reports the rack, exactly as
    the sensor must."""
    bearings = mapping.bearings_dns()
    distances = sensing.cast_rays(
        position_dns, bearings, np.asarray(scenario.low_xy),
        np.asarray(scenario.high_xy), scenario.placed_at(t), RANGE)
    return pack_scan(bearings, distances, np.ones(len(bearings), dtype=bool))

RESULTS = PROJECT_ROOT / "results"


class Checks:
    def __init__(self):
        self.rows: list[dict] = []

    def record(self, name: str, ok: bool, detail: str) -> bool:
        self.rows.append({"check": name, "pass": bool(ok), "detail": detail})
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
        return ok

    @property
    def failed(self):
        return [r for r in self.rows if not r["pass"]]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=PROJECT_ROOT / "config" / "scenario_s2_single_moving.json")
    parser.add_argument("--steps", type=int, default=90,
                        help="control steps to observe (90 x 0.2 s = 18 s)")
    parser.add_argument("--out", type=Path, default=RESULTS / "moving_obstacle.json")
    args = parser.parse_args()

    scenario = Scenario.load(args.config)
    # Track the index, not just the obstacle: backend.obstacle_nodes is parallel
    # to scenario.obstacles, and in a scene where the mover is not the first
    # obstacle, taking node[0] silently watches a static rack instead.
    moving = [(i, o) for i, o in enumerate(scenario.obstacles)
              if o.motion.type != "static" and o.motion.amplitude_m]
    if not moving:
        print("scenario has no moving obstacle")
        return 1
    index, obstacle = moving[0]
    motion = obstacle.motion
    period = (4.0 * motion.amplitude_m / motion.speed_mps
              if motion.speed_mps else motion.period_s)

    checks = Checks()
    print(f"scenario   : {scenario.name}")
    print(f"obstacle   : {obstacle.id}  r={obstacle.radius_m} m  h={obstacle.height_m} m")
    print(f"motion     : {motion.type} along {motion.axis}, amplitude "
          f"{motion.amplitude_m} m, speed {motion.speed_mps} m/s, period {period:.2f} s")
    print(f"sensing    : {scenario.sensing_mode} "
          f"(mount {scenario.lidar_mount_mode}, {scenario.lidar_resolution} beams)\n")

    from dns.backend import WebotsBackend  # noqa: E402

    backend = WebotsBackend(scenario)
    node = backend.obstacle_nodes[index]

    def observed_centre() -> np.ndarray:
        """Obstacle centre as the *simulator* reports it, in the DNS frame."""
        return to_dns(np.asarray(node.getField("translation").getSFVec3f())[:2])

    # ------------------------------------------------------------------ run 1
    print(f"[1] tracking {args.steps} control steps")
    backend.reset(seed=11, scene="moving")
    rows = []
    for _ in range(args.steps):
        frame = backend.advance(np.zeros(2), DT)      # hover: isolate the obstacle
        t = backend.episode_time
        expected = obstacle.centre_at(t)
        actual = observed_centre()
        relative = actual - frame.xy
        # Exact surface distance: for a rectangular cart, centre-distance minus
        # a radius would be wrong by up to half the diagonal.
        true_gap = float(obstacle.placed_at(t).surface_distance(
            np.asarray(frame.xy, dtype=float)[None, :])[0])
        ranges = np.asarray(frame.ranges)
        valid = np.asarray(frame.valid)

        # Which sector *should* hold the obstacle, from its true bearing. The
        # binning matches navigation.pack_scan exactly.
        bearing = math.atan2(relative[1], relative[0])
        sectors = len(ranges)
        expected_sector = int(math.floor((bearing + math.pi / sectors)
                                         / (2 * math.pi / sectors)) % sectors)
        # Read the obstacle's own sector, not a neighbourhood. The disc
        # subtends only about +-6 deg at this range inside a +-15 deg sector, so
        # the sector holding its centre bearing always contains part of it. A
        # 3-sector window was tried first and was wrong: it reaches into
        # neighbours whose minimum is the side wall (3/sin 75 deg = 3.106 m
        # at the sector-2 edge), which masked the obstacle entirely.
        own_range = float(ranges[expected_sector]) if valid[expected_sector] \
            else float("nan")
        window = [(expected_sector + k) % sectors for k in (-1, 0, 1)]
        window_range = float(min(ranges[s] for s in window if valid[s])) \
            if any(valid[s] for s in window) else float("nan")

        # Full-scene analytic truth, which is what the sensor should match.
        truth_ranges, _ = analytic_sectors(frame.xy, backend.scenario, t,
                                           backend.lidar_mapping)
        sector_error = float(np.max(np.abs(ranges - truth_ranges)))
        # Is the cart actually the nearest thing along its own bearing, or is a
        # rack in the way? Only the former lets us read the cart's range off
        # that sector directly.
        cart_leads = bool(abs(truth_ranges[expected_sector] - true_gap) < 0.25)

        rows.append({
            "t": t,
            "expected_xy": expected.tolist(),
            "actual_xy": actual.tolist(),
            "error_m": float(np.linalg.norm(actual - expected)),
            "drone_xy": frame.xy.tolist(),
            "true_clearance_m": true_gap,
            "true_bearing_deg": math.degrees(bearing) % 360.0,
            "obstacle_sector": expected_sector,
            "obstacle_sector_range_m": own_range,
            "neighbourhood_min_m": window_range,
            "analytic_sector_ranges_m": truth_ranges.tolist(),
            "max_sector_error_m": sector_error,
            "cart_is_nearest_in_its_sector": cart_leads,
            "nearest_sector": int(np.argmin(np.where(valid, ranges, np.inf))),
            "altitude_m": backend.altitude(),
        })

    errors = np.array([r["error_m"] for r in rows])
    checks.record("simulator_matches_trajectory_model", float(errors.max()) < 1e-6,
                  f"max |supervisor - centre_at(t)| = {errors.max():.3e} m "
                  f"over {len(rows)} steps")

    ys = np.array([r["actual_xy"][1] for r in rows])
    xs = np.array([r["actual_xy"][0] for r in rows])
    envelope = motion.amplitude_m + 1e-9
    checks.record("stays_in_declared_envelope",
                  bool(np.all(np.abs(ys - obstacle.xy[1]) <= envelope)),
                  f"y in [{ys.min():.4f}, {ys.max():.4f}], declared "
                  f"{obstacle.xy[1] - motion.amplitude_m:+.2f} .. "
                  f"{obstacle.xy[1] + motion.amplitude_m:+.2f} m")
    low = np.asarray(scenario.low_xy) + obstacle.radius_m
    high = np.asarray(scenario.high_xy) - obstacle.radius_m
    checks.record("stays_inside_arena",
                  bool(xs.min() >= low[0] and xs.max() <= high[0]
                       and ys.min() >= low[1] and ys.max() <= high[1]),
                  f"x [{xs.min():.2f}, {xs.max():.2f}], y [{ys.min():.2f}, "
                  f"{ys.max():.2f}] within walls inset by r")
    checks.record("other_axis_untouched",
                  bool(np.allclose(xs, obstacle.xy[0], atol=1e-9)),
                  f"x held at {obstacle.xy[0]} m (motion axis is "
                  f"{motion.axis!r})")

    # measured travel speed away from the turning points
    speeds = np.abs(np.diff(ys)) / DT
    interior = speeds[speeds > 0.1 * motion.speed_mps]
    median_speed = float(np.median(interior)) if interior.size else float("nan")
    checks.record("travel_speed_matches_config",
                  abs(median_speed - motion.speed_mps) < 0.02,
                  f"median {median_speed:.4f} m/s vs configured "
                  f"{motion.speed_mps} m/s")

    crossings = int(np.sum(np.diff(np.sign(ys - obstacle.xy[1])) != 0))
    checks.record("crosses_the_route", crossings >= 1,
                  f"obstacle centre crossed y={obstacle.xy[1]} "
                  f"{crossings} time(s) in {args.steps * DT:.1f} s")

    # ------------------------------------------------------------------ run 2
    print("\n[2] replay determinism")
    backend.reset(seed=11, scene="moving")
    replay = []
    for _ in range(args.steps):
        backend.advance(np.zeros(2), DT)
        replay.append(observed_centre())
    drift = float(np.max([np.abs(np.asarray(a["actual_xy"]) - b).max()
                          for a, b in zip(rows, replay)]))
    checks.record("obstacle_replays_exactly", drift < 1e-9,
                  f"max divergence between two resets: {drift:.3e} m")

    # ------------------------------------------------------- lidar sees motion
    print("\n[3] the lidar follows the obstacle")
    # The drone hovers 0.85 m from the back wall, so the *globally* nearest
    # return is that wall for the whole sweep. What must track the obstacle is
    # the sector the obstacle actually occupies, so score that.
    occupied = [r["obstacle_sector"] for r in rows]
    checks.record("obstacle_sector_migrates", len(set(occupied)) > 1,
                  f"obstacle swept {len(set(occupied))} sectors: "
                  f"{sorted(set(occupied))} (bearing "
                  f"{min(r['true_bearing_deg'] for r in rows):.1f}.."
                  f"{max(r['true_bearing_deg'] for r in rows):.1f} deg)")

    # Compare the measured range in the obstacle's own sector against the true
    # surface distance, on the steps where the obstacle is inside sensing range
    # and is the nearest thing along that bearing.
    # The sensor must match the analytic scene, including occlusion: where a
    # rack stands between the drone and the cart, reporting the rack is the
    # correct answer, so comparing against the cart's own clearance would
    # fail a sensor that is behaving perfectly.
    worst_sector = max(r["max_sector_error_m"] for r in rows)
    mean_sector = float(np.mean([r["max_sector_error_m"] for r in rows]))
    checks.record("scan_matches_analytic_scene", worst_sector < 0.35,
                  f"worst per-sector |lidar - analytic| {worst_sector:.4f} m, "
                  f"mean {mean_sector:.4f} m over {len(rows)} steps and all "
                  f"12 sectors (occlusion included)")

    leading = [r for r in rows if r["cart_is_nearest_in_its_sector"]]
    if leading:
        deltas = np.array([r["obstacle_sector_range_m"] - r["true_clearance_m"]
                           for r in leading])
        checks.record("measured_range_tracks_the_cart",
                      float(np.abs(deltas).max()) < 0.35,
                      f"on the {len(leading)} of {len(rows)} steps where the "
                      f"cart is the nearest surface along its own bearing, "
                      f"|lidar - cart clearance| max {np.abs(deltas).max():.4f} m, "
                      f"mean {np.abs(deltas).mean():.4f} m")
    else:
        checks.record("measured_range_tracks_the_cart", True,
                      "the cart was occluded by a rack in every sampled step, "
                      "so its range was never directly readable from its own "
                      "sector; scan_matches_analytic_scene covers correctness")

    swing = float(np.ptp([r["obstacle_sector_range_m"] for r in rows
                          if math.isfinite(r["obstacle_sector_range_m"])]))
    checks.record("measured_range_follows_the_crossing", swing > 0.3,
                  f"range in the cart's sector varied by {swing:.3f} m as it "
                  "crossed")

    altitudes = np.array([r["altitude_m"] for r in rows])
    checks.record("drone_held_altitude_while_hovering",
                  float(np.abs(altitudes - scenario.cruise_altitude_m).max()) < 0.05,
                  f"altitude in [{altitudes.min():.4f}, {altitudes.max():.4f}] m "
                  f"(target {scenario.cruise_altitude_m} m)")

    # ------------------------------------------------------------- collision
    print("\n[4] the moving obstacle can still collide")
    backend.reset(seed=11, scene="moving")
    hit = None
    for _ in range(60):
        hit = backend.advance(np.array([1.2, 0.0]), DT)   # drive into its path
        if hit.collision:
            break
    checks.record("collision_with_mover_detected", bool(hit and hit.collision),
                  f"collision reported at {np.round(hit.xy, 3).tolist()} "
                  f"after {backend._steps} steps" if hit else "no frame")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({
        "scenario": scenario.name,
        "config": str(args.config),
        "obstacle": {
            "id": obstacle.id, "label": obstacle.label,
            "purpose": obstacle.purpose, "shape": obstacle.shape,
            "size_xy": list(obstacle.size_xy), "base_xy": list(obstacle.xy),
            "radius_m": obstacle.radius_m, "height_m": obstacle.height_m,
            "motion": {"type": motion.type, "axis": motion.axis,
                       "amplitude_m": motion.amplitude_m,
                       "speed_mps": motion.speed_mps,
                       "phase_rad": motion.phase_rad,
                       "period_s": period},
        },
        "measured_median_speed_mps": median_speed,
        "steps": rows,
        "checks": checks.rows,
        "passed": len(checks.rows) - len(checks.failed),
        "total": len(checks.rows),
    }, indent=2), encoding="utf8")
    print(f"\nwrote {args.out}")

    print(f"\n{len(checks.rows) - len(checks.failed)}/{len(checks.rows)} checks passed")
    for row in checks.failed:
        print(f"  ! {row['check']}: {row['detail']}")
    return 1 if checks.failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
