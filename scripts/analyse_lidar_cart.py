"""C2: LiDAR–cart consistency, computed offline from the logged runs.

v1's headline dynamic evidence was "cart vs model 0.00e+00 m", which is true by
construction for a kinematically driven body and therefore carries no physical
information (correction C2). This is the replacement: for every logged control
step of a Mode C episode, the **measured** 12-sector scan is compared against an
**independent analytic ray caster** evaluated at the same instant, using the
logged drone pose and the logged cart pose.

The ray caster (``dns.sensing.cast_rays``) shares no code with the Webots LiDAR
path: it intersects rays against the arena walls and the exact obstacle
footprints. It is sampled at the SAME 720 bearings the real sensor uses and
binned with Spencer's own ``pack_scan``, so the comparison is like-for-like.
An agreement between the two is then evidence that the simulated sensor sees
the moving cart where the scenario says the cart is.

Only sectors whose analytic expectation is inside the range cap are compared:
a beam that returns "clear to 5 m" carries no positional information, and
including it would dilute the error with free space.

Run::

    python scripts/analyse_lidar_cart.py
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from dns.scenario import Scenario                                # noqa: E402
from dns.sensing import LidarMapping, cast_rays                  # noqa: E402
from dns.spencer_link import pack_scan                           # noqa: E402

RUNS = ROOT / "results" / "v2_S6" / "runs"
SCENARIOS = ROOT / "config" / "scenarios"
OUT = ROOT / "results" / "v2_S6"


def reference_scan(position, low, high, placed, cap, bearings):
    """Analytic 12-sector scan sampled at the SAME bearings as the real LiDAR.

    ``dns.sensing.geometric_scan`` samples three sub-rays per sector, matching
    the 3-ray model in Spencer's ``planar.py``. The Webots LiDAR has 720 beams,
    i.e. 60 per sector, and reports the minimum over all of them. Comparing the
    two directly makes the reference miss obstacle edges the sensor does see,
    which showed up as per-sector errors of up to 4 m that are an artefact of
    the sampling density rather than a sensor disagreement.

    Casting 720 analytic rays on the measured index->bearing map and binning
    them with Spencer's own ``pack_scan`` makes the comparison like-for-like.
    """
    distances = cast_rays(position, bearings, low, high, placed, cap)
    ranges, _seen = pack_scan(bearings, distances,
                              np.ones(len(bearings), dtype=bool))
    return np.clip(ranges, 0.0, cap)


def cart_sector(position, cart_centre) -> int:
    """Which of the 12 sectors the cart's centre falls in, from the drone."""
    delta = np.asarray(cart_centre, dtype=float) - np.asarray(position, dtype=float)
    bearing = float(np.arctan2(delta[1], delta[0]))
    return int(np.floor((bearing + np.pi / 12) / (2 * np.pi / 12)) % 12)


def analyse(run_id: str, scenario: Scenario) -> dict | None:
    archive = RUNS / run_id / "lidar.npz"
    if not archive.is_file():
        return None
    data = np.load(archive, allow_pickle=False)
    low = np.asarray(scenario.low_xy, dtype=float)
    high = np.asarray(scenario.high_xy, dtype=float)
    cap = scenario.sensing_max_range_m

    movers = [i for i, o in enumerate(scenario.obstacles)
              if o.motion.type != "static" and o.motion.amplitude_m]
    # The measured index->bearing map, so the reference rays sit exactly where
    # the real beams do.
    bearings = LidarMapping.calibrated(scenario.lidar_resolution).bearings_dns()
    errors, cart_errors, rows = [], [], []

    for index in range(len(data["t_s"])):
        position = data["pose_xy"][index]
        measured = data["sector_ranges_m"][index]
        valid = data["sector_valid"][index]
        centres = data["obstacle_centres_xy"][index]

        # Rebuild the exact footprints at this instant from the LOGGED centres,
        # so the reference follows the cart rather than assuming where it was.
        placed = []
        for obstacle, centre in zip(scenario.obstacles, centres):
            body = obstacle.placed_at(0.0)
            placed.append(type(body)(centre=(float(centre[0]), float(centre[1])),
                                     shape=body.shape, radius_m=body.radius_m,
                                     half_size=body.half_size,
                                     yaw_rad=body.yaw_rad))
        expected = reference_scan(position, low, high, placed, cap, bearings)

        usable = valid & (expected < cap - 1e-6)
        if not np.any(usable):
            continue
        residual = np.abs(measured - expected)[usable]
        errors.extend(residual.tolist())

        if movers:
            sector = cart_sector(position, centres[movers[0]])
            if usable[sector]:
                cart_errors.append(float(abs(measured[sector] - expected[sector])))
                rows.append({
                    "run_id": run_id,
                    "t_s": round(float(data["t_s"][index]), 4),
                    "cart_y_m": round(float(centres[movers[0]][1]), 4),
                    "cart_sector": sector,
                    "measured_m": round(float(measured[sector]), 4),
                    "raycast_m": round(float(expected[sector]), 4),
                    "abs_error_m": round(cart_errors[-1], 5),
                })

    if not errors:
        return None
    errors = np.asarray(errors)
    summary = {
        "run_id": run_id,
        "control_steps": int(len(data["t_s"])),
        "sector_comparisons": int(errors.size),
        "all_sectors": {
            "mae_m": round(float(errors.mean()), 5),
            "rmse_m": round(float(np.sqrt((errors ** 2).mean())), 5),
            "p95_m": round(float(np.percentile(errors, 95)), 5),
            "max_m": round(float(errors.max()), 5),
        },
    }
    if cart_errors:
        cart = np.asarray(cart_errors)
        summary["cart_sector_only"] = {
            "comparisons": int(cart.size),
            "mae_m": round(float(cart.mean()), 5),
            "rmse_m": round(float(np.sqrt((cart ** 2).mean())), 5),
            "p95_m": round(float(np.percentile(cart, 95)), 5),
            "max_m": round(float(cart.max()), 5),
        }
    return {"summary": summary, "rows": rows}


def main() -> int:
    all_rows, summaries = [], []
    for layout in ("L00", "L01", "L02", "L03", "L04", "L05"):
        path = SCENARIOS / f"{layout}_C.json"
        if not path.is_file():
            continue
        scenario = Scenario.load(path)
        for controller in ("ppo_seed7", "ppo_seed19", "ppo_seed43"):
            result = analyse(f"{layout}_C_{controller}_v00", scenario)
            if result:
                result["summary"]["layout"] = layout
                summaries.append(result["summary"])
                all_rows.extend(result["rows"])

    if not summaries:
        print("no lidar archives found")
        return 1

    pooled = np.asarray([r["abs_error_m"] for r in all_rows])
    report = {
        "what": "measured 12-sector LiDAR vs an independent analytic ray caster "
                "evaluated at the logged drone and cart poses",
        "why": "replaces v1's 'cart vs model 0.00e+00 m', which was true by "
               "construction for a kinematically driven body (correction C2)",
        "method": "only sectors whose analytic expectation is inside the 5 m cap "
                  "are compared; an out-of-range beam carries no positional "
                  "information",
        "runs": summaries,
        "cart_sector_pooled": {
            "comparisons": int(pooled.size),
            "mae_m": round(float(pooled.mean()), 5),
            "rmse_m": round(float(np.sqrt((pooled ** 2).mean())), 5),
            "p95_m": round(float(np.percentile(pooled, 95)), 5),
            "max_m": round(float(pooled.max()), 5),
        },
    }
    (OUT / "lidar_cart_consistency.json").write_text(
        json.dumps(report, indent=2), encoding="utf8")
    with (OUT / "lidar_cart_consistency.csv").open("w", newline="",
                                                   encoding="utf8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)

    print(f"{'run':<28}{'steps':>7}{'cmp':>7}{'MAE_m':>9}{'RMSE_m':>9}{'p95_m':>9}{'max_m':>9}")
    for entry in summaries:
        a = entry["all_sectors"]
        print(f"{entry['run_id']:<28}{entry['control_steps']:>7}"
              f"{entry['sector_comparisons']:>7}{a['mae_m']:>9.4f}"
              f"{a['rmse_m']:>9.4f}{a['p95_m']:>9.4f}{a['max_m']:>9.4f}")
    cart = report["cart_sector_pooled"]
    print(f"\nCART SECTOR, pooled over {len(summaries)} runs: "
          f"n={cart['comparisons']}  MAE={cart['mae_m']:.4f} m  "
          f"RMSE={cart['rmse_m']:.4f} m  p95={cart['p95_m']:.4f} m  "
          f"max={cart['max_m']:.4f} m")
    print(f"wrote {(OUT / 'lidar_cart_consistency.json').relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
