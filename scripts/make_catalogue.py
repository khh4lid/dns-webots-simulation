"""Obstacle catalogue and the side-elevation chart.

Two slide assets that the videos cannot carry on their own:

``obstacle_catalogue``
    Four cards -- rack, column, crate stack, cart -- each with a real render
    crop, its footprint and height, whether it moves, why it is in the scene,
    its height against the 1.14 m scan plane, and how many appear in each of
    L00-L05. Counts are read from the asset library, never typed.

``side_elevation``
    x against z for one logged episode: obstacle heights as bars, the scan
    plane, and the drone's altitude trace. This is ADR-003 in one picture --
    every obstacle rises above the plane, so nothing is solid to the collision
    test and invisible to the sensor.

Run::

    python scripts/make_catalogue.py
"""
from __future__ import annotations

import csv
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import dns_style as S                                            # noqa: E402
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.patches import Rectangle                         # noqa: E402
from PIL import Image                                            # noqa: E402

FIGURES = ROOT / "assets" / "figures"
RENDERS = ROOT / "assets" / "renders"
RESULTS = ROOT / "results" / "v2_S6"
LAYOUTS = ("L00", "L01", "L02", "L03", "L04", "L05")

#: Asset id prefix -> (display name, why it is there). The "why" is the same
#: sentence the README obstacle table carries.
KINDS = {
    "rack": ("Storage rack",
             "Crosses the straight start–goal line by design, forcing an\n"
             "S-shaped route. Its flat 2.4 m face is also the cleanest\n"
             "LiDAR calibration target."),
    "column": ("Structural column",
               "Off-route building column: a precisely known square return\n"
               "for sector calibration, plus a wall-clearance hazard."),
    "crate": ("Crate stack",
              "Stacked pallet load off the route. Tests whether clearance is\n"
              "kept while converging on the goal."),
    "cart": ("Powered transfer cart",
             "The dynamic hazard. Crosses the aisle at constant speed, so free\n"
             "space in its lane is a function of time — a timing problem,\n"
             "not only a geometric one."),
}

SCAN_PLANE = 1.14
CRUISE = 1.0


def asset_rows() -> list[dict]:
    with (RESULTS / "asset_library.csv").open(encoding="utf8") as handle:
        return list(csv.DictReader(handle))


def kind_of(obstacle_id: str) -> str:
    for key in ("rack", "column", "crate", "cart"):
        if obstacle_id.startswith(key) or key in obstacle_id:
            return key
    return "crate"


def catalogue() -> None:
    rows = asset_rows()
    counts = {k: Counter() for k in KINDS}
    spec: dict[str, dict] = {}
    for row in rows:
        kind = kind_of(row["obstacle_id"])
        counts[kind][row["layout_id"]] += 1
        spec.setdefault(kind, row)

    # Real render crops, taken from the side capture so the heights are visible.
    source = RENDERS / "V-SIDE_L00_altitude_poster.png"
    crop = Image.open(source).convert("RGB") if source.is_file() else None
    # (left, top, right, bottom) in the 1920x1080 poster, picked to frame each
    # body. Recorded here so the crops are reproducible rather than eyeballed
    # once and lost.
    BOXES = {"rack": (430, 230, 700, 680), "column": (1180, 110, 1340, 500),
             "crate": (1300, 440, 1500, 660), "cart": (760, 410, 960, 670)}

    figure, axes = plt.subplots(1, 4, figsize=(13.6, 4.6))
    for axis, (kind, (name, why)) in zip(axes, KINDS.items()):
        row = spec.get(kind)
        if crop is not None and kind in BOXES:
            axis.imshow(crop.crop(BOXES[kind]))
        axis.set_xticks([])
        axis.set_yticks([])
        for spine in axis.spines.values():
            spine.set_color(S.GRID)
        moving = row and int(row["moving"])
        axis.set_title(name, loc="left", fontsize=11, fontweight="bold",
                       color=S.CART if moving else S.INK)
        height = float(row["height_m"]) if row else 0.0
        lines = [
            f"{float(row['size_x_m']):.2f} × {float(row['size_y_m']):.2f} × "
            f"{height:.2f} m" if row else "—",
            ("moving: ±1.9 m at 0.45 m/s, period 16.89 s" if moving
             else "static"),
            f"height vs 1.14 m scan plane: +{height - SCAN_PLANE:.2f} m",
            "count per layout L00–L05: "
            + "/".join(str(counts[kind].get(l, 0)) for l in LAYOUTS),
        ]
        axis.text(0.0, -0.06, "\n".join(lines), transform=axis.transAxes,
                  fontsize=7.6, va="top", color=S.INK_SOFT, linespacing=1.6)
        axis.text(0.0, -0.40, why, transform=axis.transAxes, fontsize=7.2,
                  va="top", color=S.INK_MUTED, linespacing=1.5)
        S.provenance(axis, "WEBOTS CAPTURE (crop)", loc="lower right")

    figure.suptitle("Obstacle catalogue — four types, each placed for a reason",
                    x=0.012, ha="left", fontsize=12.5, fontweight="bold")
    figure.tight_layout(rect=(0, 0.30, 1, 0.94))
    S.save(figure, FIGURES / "obstacle_catalogue")
    print("ok   obstacle_catalogue")


def side_elevation() -> None:
    """x vs z: obstacle heights, the scan plane, and the flown altitude."""
    from dns.scenario import Scenario
    scenario = Scenario.load(ROOT / "config" / "scenarios" / "L00_C.json")
    track = RESULTS / "runs" / "L00_C_ppo_seed7_v01" / "trajectory.csv"
    with track.open(encoding="utf8") as handle:
        rows = list(csv.DictReader(handle))

    figure, axis = plt.subplots(figsize=(8.4, 3.4))
    for obstacle in scenario.obstacles:
        moving = obstacle.motion.type != "static" and obstacle.motion.amplitude_m
        half = obstacle.size_xy[0] / 2.0
        axis.add_patch(Rectangle(
            (obstacle.xy[0] - half, 0.0), 2 * half, obstacle.height_m,
            facecolor=S.CART if moving else S.STATIC_OBSTACLE, alpha=0.85,
            edgecolor=S.STATIC_EDGE, linewidth=0.7, zorder=4))
        axis.text(obstacle.xy[0], obstacle.height_m + 0.07,
                  f"{obstacle.height_m:.1f}", ha="center", fontsize=7,
                  color=S.INK_SOFT, zorder=6)

    axis.axhline(SCAN_PLANE, color=S.GOAL, linewidth=1.6,
                 linestyle=(0, (5, 2)), zorder=7)
    axis.annotate(f"LiDAR scan plane  {SCAN_PLANE:.2f} m",
                  (0.15, SCAN_PLANE + 0.06), fontsize=8, color=S.GOAL,
                  fontweight="bold", zorder=8)
    axis.plot([float(r["x_m"]) for r in rows],
              [float(r["altitude_m"]) for r in rows],
              color=S.CONTROLLER_COLOUR["ppo_seed7"], linewidth=2.0, zorder=9,
              label="flown altitude")
    axis.axhline(CRUISE, color=S.INK_MUTED, linewidth=0.9,
                 linestyle=(0, (1, 2)), zorder=3)
    axis.annotate("cruise 1.00 m", (9.1, CRUISE - 0.16), fontsize=7.6,
                  color=S.INK_SOFT, ha="right")
    axis.axhline(2.5, color=S.WALL, linewidth=0.9, linestyle=(0, (4, 3)))
    axis.annotate("wall top 2.50 m", (9.85, 2.56), fontsize=7.6,
                  color=S.WALL, ha="right")

    altitudes = np.array([float(r["altitude_m"]) for r in rows])
    worst_mm = float(np.abs(altitudes - CRUISE).max()) * 1000.0
    axis.annotate(f"altitude held to ±{worst_mm:.2f} mm over "
                  f"{len(rows)} control steps",
                  (0.015, 0.80), xycoords="axes fraction", fontsize=7.8,
                  va="top", color=S.INK_SOFT)
    axis.set_xlim(0, 10)
    axis.set_ylim(0, 2.9)
    axis.set_xlabel("x [m]")
    axis.set_ylabel("z [m]")
    axis.set_title("Every obstacle rises above the scan plane, so nothing is "
                   "solid but unsensed",
                   loc="left", fontsize=10.5, fontweight="bold", pad=6)
    axis.legend(loc="upper right", fontsize=7.6)
    S.grid(axis, "y")
    S.provenance(axis, "LOGGED-DATA RENDER", loc="lower right")
    figure.tight_layout()
    S.save(figure, FIGURES / "side_elevation")
    print("ok   side_elevation")


def main() -> int:
    S.apply()
    FIGURES.mkdir(parents=True, exist_ok=True)
    for function in (catalogue, side_elevation):
        try:
            function()
        except Exception as error:                               # noqa: BLE001
            print(f"FAIL {function.__name__}: {type(error).__name__}: {error}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
