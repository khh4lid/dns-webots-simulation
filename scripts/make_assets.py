"""Generate the README visual assets from verified project data.

Every asset here is derived from a file produced by a real run -- the scenario
configuration, a Webots capture, a measurement JSON, or the experiment CSVs.
Nothing is drawn by hand and nothing is illustrative. Each output is recorded in
``assets/ASSET_PROVENANCE.json`` with the source it came from.

Run::

    python scripts/make_assets.py

Requires ``matplotlib`` and ``Pillow``; the video thumbnail additionally needs
``imageio-ffmpeg``. A missing input is reported and skipped rather than faked.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

RESULTS = PROJECT_ROOT / "results"
ASSETS = PROJECT_ROOT / "assets"

# --- palette -------------------------------------------------------------
# Validated with the data-viz palette validator (light surface #fcfcfb):
# the three method hues pass the lightness band, chroma floor, all-pairs CVD
# separation (worst 9.2 deutan) and the normal-vision floor (worst 24.0).
# Aqua sits below 3:1 against the surface, so every chart that uses it also
# carries a legend and direct labels -- never colour alone.
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"

METHOD = {"ppo": "#2a78d6", "direct": "#eb6834", "reactive": "#1baf7a"}

# Scene colours deliberately mirror the simulator's own materials so the map
# and the screenshot can be read against each other.
C_RACK = "#545a63"
C_COLUMN = "#8c8e93"
C_CRATE = "#756044"
C_CART = "#eb7321"
C_START = "#1baf7a"
C_GOAL = "#2a78d6"

PROVENANCE: list[dict] = []


def record(path: Path, kind: str, sources: list[str], note: str) -> None:
    PROVENANCE.append({
        "asset": str(path.relative_to(PROJECT_ROOT)).replace("\\", "/"),
        "kind": kind,
        "generated_by": "scripts/make_assets.py",
        "sources": sources,
        "note": note,
    })
    print(f"  + {path.relative_to(PROJECT_ROOT)}")


def style_axes(ax, *, grid: bool = True) -> None:
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=MUTED, labelsize=8, length=3, width=0.8)
    for lbl in ax.get_xticklabels() + ax.get_yticklabels():
        lbl.set_color(INK_2)
    if grid:
        ax.grid(True, color=GRID, linewidth=0.6, alpha=0.9)
        ax.set_axisbelow(True)


def new_figure(w: float, h: float):
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(w, h), dpi=150, facecolor=SURFACE)
    return fig


def save(fig, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, facecolor=SURFACE, bbox_inches="tight", pad_inches=0.18)
    import matplotlib.pyplot as plt
    plt.close(fig)


# --------------------------------------------------------------------------
# scenario helpers
# --------------------------------------------------------------------------
def load_scenario(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf8"))


def footprint(obs: dict) -> np.ndarray:
    """Return the obstacle footprint corners in the DNS frame."""
    cx, cy = obs["xy"]
    sx, sy = obs.get("size_xy", (0.6, 0.6))
    yaw = float(obs.get("yaw_rad", 0.0))
    hx, hy = sx / 2.0, sy / 2.0
    corners = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]])
    rot = np.array([[math.cos(yaw), -math.sin(yaw)],
                    [math.sin(yaw), math.cos(yaw)]])
    return corners @ rot.T + np.array([cx, cy])


def obstacle_colour(obs: dict) -> str:
    oid = obs["id"]
    if oid.startswith("rack"):
        return C_RACK
    if oid.startswith("column"):
        return C_COLUMN
    if oid.startswith("crate"):
        return C_CRATE
    return C_CART


# --------------------------------------------------------------------------
# A. annotated hero screenshot
# --------------------------------------------------------------------------
class Projector:
    """Pinhole projection matching the Webots ``Camera`` node.

    Verified against the capture: the start pad projects to pixel (563, 302)
    and the rendered pad centroid measures (562, 302).
    """

    def __init__(self, eye, target, width: int, height: int, hfov: float):
        self.eye = np.asarray(eye, float)
        self.w, self.h = width, height
        fwd = np.asarray(target, float) - self.eye
        self.fwd = fwd / np.linalg.norm(fwd)
        right = np.cross(self.fwd, np.array([0.0, 0.0, 1.0]))
        self.right = right / np.linalg.norm(right)
        self.up = np.cross(self.right, self.fwd)
        self.f = (width / 2.0) / math.tan(hfov / 2.0)

    def __call__(self, point) -> tuple[float, float]:
        d = np.asarray(point, float) - self.eye
        z = float(d @ self.fwd)
        return (self.w / 2.0 + self.f * float(d @ self.right) / z,
                self.h / 2.0 - self.f * float(d @ self.up) / z)


def annotate_hero(scenario: dict, shots: dict) -> Path | None:
    import matplotlib.pyplot as plt
    from PIL import Image

    src = RESULTS / "screenshots" / "hero.png"
    if not src.is_file():
        print("  MISSING results/screenshots/hero.png -- run client_screenshot.py")
        return None
    view = next((v for v in shots["views"] if v["view"] == "hero"), None)
    if view is None:
        print("  MISSING 'hero' view in screenshots.json")
        return None

    img = np.asarray(Image.open(src).convert("RGB"))
    h, w = img.shape[:2]
    cam = scenario.get("camera", {})
    proj = Projector(view["eye_webots"], view["target_webots"],
                     int(cam.get("width", w)), int(cam.get("height", h)), 1.35)

    def dns_to_webots(xy):
        return np.array([xy[0], -xy[1]], float)

    # Label boxes are placed at absolute pixel positions chosen against the
    # projected anchors (printed by this projector) so that none of them
    # overlaps another or an important part of the render. The empty
    # background margins left, right and top carry most of them.
    labels = []
    sx, sy = scenario["drone"]["start_xy"]
    gx, gy = scenario["goal"]["xy"]
    labels.append((f"Start  ({sx:.2f}, {sy:.2f}) m", dns_to_webots((sx, sy)),
                   0.02, C_START, (588, 108)))
    labels.append((f"Goal  ({gx:.2f}, {gy:.2f}) m\n"
                   f"tolerance {scenario['goal']['tolerance_m']:.2f} m",
                   dns_to_webots((gx, gy)), 0.02, C_GOAL, (628, 683)))
    labels.append((f"Drone — {scenario['drone']['cruise_altitude_m']:.1f} m "
                   f"altitude,\n{scenario['drone']['collision_radius_m']:.2f} m "
                   "collision radius",
                   dns_to_webots((sx, sy)), 1.05, INK, (260, 112)))

    text = {"rack_a": "Storage rack A\n0.8 × 2.4 m, h 2.0 m",
            "rack_b": "Storage rack B\n0.8 × 2.4 m, h 2.0 m",
            "column_1": "Column\n0.4 × 0.4 m",
            "column_2": "Column\n0.4 × 0.4 m",
            "crate_stack": "Crate stack\n0.7 × 0.7 m",
            "cart": "Transfer cart — moving\n0.9 × 0.55 m @ 0.45 m/s"}
    places = {"column_1": (168, 243), "rack_a": (1002, 214),
              "cart": (1078, 338), "column_2": (1132, 486),
              "rack_b": (352, 512), "crate_stack": (300, 648)}
    for obs in scenario["obstacles"]:
        oid = obs["id"]
        if oid not in text:
            continue
        labels.append((text[oid], dns_to_webots(obs["xy"]),
                       obs["height_m"] * 0.65, obstacle_colour(obs), places[oid]))

    fig = plt.figure(figsize=(w / 150, h / 150), dpi=150, facecolor=SURFACE)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(img)
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.axis("off")

    for caption, xy_w, z, colour, (lx, ly) in labels:
        u, v = proj([xy_w[0], xy_w[1], z])
        if not (0 <= u <= w and 0 <= v <= h):
            continue
        ax.annotate(
            caption, xy=(u, v), xytext=(lx, ly),
            fontsize=8.5, color=INK, linespacing=1.35,
            ha="center", va="center",
            bbox=dict(boxstyle="round,pad=0.42", facecolor="#ffffff",
                      edgecolor=colour, linewidth=1.3, alpha=0.95),
            arrowprops=dict(arrowstyle="-", color=colour, linewidth=1.3,
                            shrinkA=2, shrinkB=3, alpha=0.95))
        ax.plot([u], [v], marker="o", markersize=5, color=colour,
                markeredgecolor="#ffffff", markeredgewidth=1.2, zorder=5)

    out = ASSETS / "screenshots" / "hero_annotated.png"
    save(fig, out)
    record(out, "annotated Webots capture",
           ["results/screenshots/hero.png", "results/screenshots/screenshots.json",
            "config/scenario_warehouse.json"],
           "Genuine Webots R2025a render from the drone's scene Camera node. "
           "Labels are placed by projecting the configured world coordinates "
           "through the recorded camera pose; no scene element was redrawn.")
    return out


# --------------------------------------------------------------------------
# B. scaled top-down technical map
# --------------------------------------------------------------------------
def environment_map(scenario: dict) -> Path:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle, Circle

    lo, hi = scenario["arena"]["low_xy"], scenario["arena"]["high_xy"]
    fig = new_figure(10.0, 6.6)
    ax = fig.add_subplot(111)
    style_axes(ax)
    ax.set_aspect("equal")

    wall = scenario["arena"]["wall_thickness_m"]
    ax.add_patch(Rectangle((lo[0] - wall, lo[1] - wall),
                           (hi[0] - lo[0]) + 2 * wall, (hi[1] - lo[1]) + 2 * wall,
                           facecolor="none", edgecolor=AXIS, linewidth=3.0))

    radius = scenario["drone"]["collision_radius_m"]
    for obs in scenario["obstacles"]:
        poly = footprint(obs)
        colour = obstacle_colour(obs)
        moving = "motion" in obs
        ax.add_patch(Polygon(poly, closed=True, facecolor=colour,
                             edgecolor=colour, linewidth=1.0,
                             alpha=0.55 if moving else 0.85,
                             linestyle="--" if moving else "-"))
        cx, cy = obs["xy"]
        sx_, sy_ = obs.get("size_xy", (0.6, 0.6))
        spec = f"{sx_:.2f} × {sy_:.2f} m, h {obs['height_m']:.1f} m"
        # A 0.4 m column is far too small to hold text at this scale, so only
        # the large footprints get an inside label; everything else is called
        # out beside the shape.
        if moving:
            # Sits inside its own swept band, so the callout goes above the
            # band where nothing else is drawn.
            amp_ = obs["motion"]["amplitude_m"]
            ax.text(cx, cy - amp_ - sy_ / 2 - 0.18,
                    f"{obs['id'].replace('_', ' ')}\n{spec}",
                    ha="center", va="bottom", fontsize=6.8, color=INK_2, zorder=6)
        elif min(sx_, sy_) >= 0.75:
            ax.text(cx, cy, obs["id"].replace("_", " "), ha="center",
                    va="center", fontsize=7.5, color="#ffffff",
                    fontweight="bold", zorder=6)
            ax.text(cx, cy - sy_ / 2 - 0.16, spec, ha="center", va="bottom",
                    fontsize=6.8, color=INK_2, zorder=6)
        else:
            ax.text(cx + sx_ / 2 + 0.18, cy,
                    f"{obs['id'].replace('_', ' ')}\n{spec}",
                    ha="left", va="center", fontsize=6.8, color=INK_2, zorder=6)

        if moving:
            amp = obs["motion"]["amplitude_m"]
            ax.add_patch(Rectangle((cx - sx_ / 2, cy - amp - sy_ / 2),
                                   sx_, 2 * amp + sy_, facecolor=colour,
                                   alpha=0.12, edgecolor=colour,
                                   linestyle=":", linewidth=1.0, zorder=1))
            ax.annotate("", xy=(cx, cy + amp), xytext=(cx, cy - amp),
                        arrowprops=dict(arrowstyle="<->", color=colour,
                                        linewidth=1.4))
            ax.text(cx - sx_ / 2 - 0.22, cy + amp,
                    f"sweep ±{amp:.2f} m\nat {obs['motion']['speed_mps']:.2f} m/s",
                    fontsize=7, color=INK_2, ha="right", va="center")

    sx, sy = scenario["drone"]["start_xy"]
    gx, gy = scenario["goal"]["xy"]
    tol = scenario["goal"]["tolerance_m"]
    ax.add_patch(Circle((gx, gy), tol, facecolor=C_GOAL, alpha=0.16,
                        edgecolor=C_GOAL, linestyle="--", linewidth=1.2))
    ax.plot([sx], [sy], marker="o", markersize=11, color=C_START,
            markeredgecolor="#ffffff", markeredgewidth=1.6, zorder=7)
    ax.plot([gx], [gy], marker="*", markersize=19, color=C_GOAL,
            markeredgecolor="#ffffff", markeredgewidth=1.2, zorder=7)
    ax.text(sx - 0.30, sy, f"START\n({sx:.2f}, {sy:.2f})", ha="right",
            va="center", fontsize=8, color=INK, fontweight="bold")
    ax.text(gx, gy - tol - 0.22, f"GOAL  ({gx:.2f}, {gy:.2f})\ntolerance {tol:.2f} m",
            ha="center", va="bottom", fontsize=8, color=INK, fontweight="bold")

    ax.annotate("", xy=(hi[0], hi[1] + 0.62), xytext=(lo[0], hi[1] + 0.62),
                arrowprops=dict(arrowstyle="<->", color=INK_2, linewidth=1.1))
    ax.text((lo[0] + hi[0]) / 2, hi[1] + 0.74, f"{hi[0] - lo[0]:.1f} m",
            ha="center", va="bottom", fontsize=8.5, color=INK)
    ax.annotate("", xy=(lo[0] - 0.62, hi[1]), xytext=(lo[0] - 0.62, lo[1]),
                arrowprops=dict(arrowstyle="<->", color=INK_2, linewidth=1.1))
    ax.text(lo[0] - 0.74, (lo[1] + hi[1]) / 2, f"{hi[1] - lo[1]:.1f} m",
            ha="right", va="center", fontsize=8.5, color=INK, rotation=90)

    ax.set_xlim(lo[0] - 1.25, hi[0] + 0.55)
    ax.set_ylim(hi[1] + 1.15, lo[1] - 1.15)      # +y points right => y down
    ax.set_xticks(np.arange(0, hi[0] + 0.1, 1.0))
    ax.set_yticks(np.arange(lo[1], hi[1] + 0.1, 1.0))
    ax.set_xlabel("DNS x -- forward (m)", fontsize=9, color=INK_2)
    ax.set_ylabel("DNS y -- right (m)", fontsize=9, color=INK_2)
    ax.set_title(
        f"Warehouse aisle, scaled plan  ·  {scenario['name']}  ·  "
        f"drone collision radius {radius:.2f} m",
        fontsize=10.5, color=INK, pad=12, loc="left")

    out = ASSETS / "figures" / "environment_map.png"
    save(fig, out)
    record(out, "generated technical drawing", ["config/scenario_warehouse.json"],
           "Scaled plan drawn directly from the scenario JSON that generates the "
           "Webots world, so the map cannot drift from the simulated geometry.")
    return out


# --------------------------------------------------------------------------
# C. moving-obstacle validation
# --------------------------------------------------------------------------
def moving_obstacle_figure() -> Path | None:
    import matplotlib.pyplot as plt
    src = RESULTS / "moving_obstacle_warehouse.json"
    if not src.is_file():
        print(f"  MISSING {src.relative_to(PROJECT_ROOT)}")
        return None
    data = json.loads(src.read_text(encoding="utf8"))
    steps = data["steps"]
    t = np.array([s["t"] for s in steps])
    expected = np.array([s["expected_xy"][1] for s in steps])
    actual = np.array([s["actual_xy"][1] for s in steps])
    sector_err = np.array([s["max_sector_error_m"] for s in steps])
    replay = next((c["detail"] for c in data["checks"]
                   if c["check"] == "obstacle_replays_exactly"), "")

    motion = data["obstacle"]["motion"]
    fig = new_figure(10.0, 5.4)
    gs = fig.add_gridspec(2, 1, height_ratios=[1.35, 1.0], hspace=0.42)

    ax = fig.add_subplot(gs[0])
    style_axes(ax)
    ax.plot(t, expected, color=MUTED, linewidth=4.0, alpha=0.55,
            label="Commanded path  y(t) from the motion model", zorder=2)
    ax.plot(t, actual, color=METHOD["ppo"], linewidth=1.6,
            label="Supervisor-reported cart position", zorder=3)
    ax.axhline(0.0, color=AXIS, linewidth=0.8, linestyle=":")
    ax.set_ylabel("cart y, DNS frame (m)", fontsize=9, color=INK_2)
    ax.set_title(
        f"Moving obstacle — path, determinism and sensed agreement  ·  "
        f"{data['passed']}/{data['total']} checks passed over {len(t)} steps",
        fontsize=10.5, color=INK, pad=10, loc="left")
    leg = ax.legend(frameon=False, fontsize=8.5, loc="lower right")
    for text in leg.get_texts():
        text.set_color(INK_2)
    ax.text(0.012, 0.04,
            f"A = {motion['amplitude_m']:.2f} m   T = {motion['period_s']:.2f} s   "
            f"configured {motion['speed_mps']:.2f} m/s   measured median "
            f"{data['measured_median_speed_mps']:.4f} m/s",
            transform=ax.transAxes, fontsize=8, color=INK_2)
    # The cart is kinematically driven FROM this equation, so the two curves
    # agreeing to 0.000e+00 m is exact by construction and is not an
    # independent measurement. Say so on the figure rather than implying
    # physical fidelity. Placed as a caption under the axes so it never
    # crosses the plotted path.
    fig.text(0.012, -0.055,
             "The two curves coincide at 0.000e+00 m by construction — the cart is "
             "kinematically driven from this equation, so the top panel verifies "
             "that the implementation applies\nthe model exactly; it is not "
             "evidence of rigid-body fidelity. The independent facts are the "
             f"measured median speed and replay determinism ({replay.split(': ')[-1]}).",
             fontsize=7.8, color=INK_2, va="top", ha="left", linespacing=1.6)

    ax2 = fig.add_subplot(gs[1])
    style_axes(ax2)
    ax2.plot(t, sector_err, color=METHOD["direct"], linewidth=1.5, zorder=3)
    ax2.axhline(float(sector_err.mean()), color=INK_2, linewidth=1.0,
                linestyle="--", zorder=2)
    ax2.set_xlabel("episode time (s)", fontsize=9, color=INK_2)
    ax2.set_ylabel("worst sector\n|LiDAR − analytic| (m)", fontsize=9, color=INK_2)
    ax2.set_ylim(0, float(sector_err.max()) * 1.35)
    ax2.text(0.985, float(sector_err.mean()) / (float(sector_err.max()) * 1.35) + 0.06,
             f"mean {sector_err.mean():.4f} m   peak {sector_err.max():.4f} m",
             transform=ax2.transAxes, va="bottom", ha="right",
             fontsize=8, color=INK_2)
    ax2.set_title(
        "Independent check — measured scan against an analytic ray-cast of the "
        "same moving scene, all 12 sectors, occlusion included",
        fontsize=9, color=INK_2, pad=8, loc="left")

    out = ASSETS / "figures" / "moving_obstacle_validation.png"
    save(fig, out)
    record(out, "measurement plot", ["results/moving_obstacle_warehouse.json"],
           "Top: supervisor-reported cart position against the closed-form path "
           "it is driven from (exact by construction; labelled as such on the "
           "figure). Bottom: the genuinely independent comparison of the measured "
           "LiDAR scan against an analytic ray-cast during the same episode.")
    return out


# --------------------------------------------------------------------------
# D. LiDAR sector validation
# --------------------------------------------------------------------------
def lidar_figure() -> Path | None:
    import matplotlib.pyplot as plt
    src = RESULTS / "calibration_warehouse.json"
    if not src.is_file():
        print(f"  MISSING {src.relative_to(PROJECT_ROOT)}")
        return None
    data = json.loads(src.read_text(encoding="utf8"))
    lid = data["lidar"]
    check = lid["sector_check"]
    meas = np.array(check["lidar_ranges_m"], float)
    truth = np.array(check["geometric_ranges_m"], float)
    sectors = np.arange(len(meas))

    fig = new_figure(10.0, 4.2)
    ax = fig.add_subplot(111)
    style_axes(ax)
    width = 0.38
    ax.bar(sectors - width / 2, truth, width, color=MUTED, alpha=0.65,
           label="Analytic ray-cast (simulator ground truth)")
    ax.bar(sectors + width / 2, meas, width, color=METHOD["ppo"],
           label="Webots LiDAR, 720 beams binned into 12 sectors")
    for s, (m, g) in enumerate(zip(meas, truth)):
        ax.text(s, max(m, g) + 0.10, f"{abs(m - g):.2f}", ha="center",
                va="bottom", fontsize=6.8, color=INK_2)

    ax.set_xticks(sectors)
    ax.set_xticklabels([f"{s}\n{s * 30}°" for s in sectors], fontsize=7.5)
    ax.set_xlabel("PPO sector (sector 0 forward, 3 right, 6 back, 9 left)",
                  fontsize=9, color=INK_2)
    ax.set_ylabel("range to nearest surface (m)", fontsize=9, color=INK_2)
    ax.set_title(
        f"LiDAR sector agreement with an independent ray-cast  ·  "
        f"angular fit RMS {lid['rms_error_m']:.4f} m  ·  "
        f"max sector difference {check['max_abs_diff_m']:.3f} m",
        fontsize=10.5, color=INK, pad=10, loc="left")
    leg = ax.legend(frameon=False, fontsize=8.5, loc="upper right")
    for text in leg.get_texts():
        text.set_color(INK_2)
    ax.text(0.012, 0.92,
            f"measured beam offset {lid['offset_deg']:.1f}°   "
            f"direction {int(lid['direction']):+d}   "
            f"{lid['finite_beams']} of {lid['resolution']} beams finite   "
            "numbers above the bars are per-sector |difference| in m",
            transform=ax.transAxes, fontsize=7.8, color=INK_2, va="top")

    out = ASSETS / "figures" / "lidar_validation.png"
    save(fig, out)
    record(out, "measurement plot", ["results/calibration_warehouse.json"],
           "Sector ranges read from the Webots Lidar device compared against an "
           "analytic ray-cast of the same scene from the same pose.")
    return out


# --------------------------------------------------------------------------
# E. success rate with confidence intervals
# --------------------------------------------------------------------------
def success_rate_figure() -> Path | None:
    import matplotlib.pyplot as plt
    src = RESULTS / "experiments_warehouse" / "experiment_summary.csv"
    if not src.is_file():
        print(f"  MISSING {src.relative_to(PROJECT_ROOT)}")
        return None
    rows = list(csv.DictReader(src.open(encoding="utf8")))

    modes = [("B_static", "Mode B — static (cart parked)"),
             ("C_dynamic", "Mode C — dynamic (cart sweeping)")]
    order = ["ppo_seed7", "ppo_seed19", "ppo_seed43", "reactive", "direct"]
    nice = {"ppo_seed7": "PPO  seed 7", "ppo_seed19": "PPO  seed 19",
            "ppo_seed43": "PPO  seed 43", "reactive": "reactive rule (scripted)",
            "direct": "goal-direct (scripted)"}

    fig = new_figure(10.0, 4.6)
    axes = fig.subplots(1, 2, sharex=True)
    for ax, (mode, title) in zip(axes, modes):
        style_axes(ax)
        sub = {r["controller"]: r for r in rows if r["mode"] == mode}
        ys = np.arange(len(order))
        for i, name in enumerate(order):
            r = sub.get(name)
            if r is None:
                continue
            rate = float(r["success_rate"])
            lo = float(r["success_rate_ci95_low"])
            hi = float(r["success_rate_ci95_high"])
            colour = METHOD["ppo"] if name.startswith("ppo") else METHOD[name]
            ax.plot([lo, hi], [i, i], color=colour, linewidth=2.4, alpha=0.40,
                    solid_capstyle="round", zorder=2)
            ax.plot([rate], [i], marker="o", markersize=9, color=colour,
                    markeredgecolor="#ffffff", markeredgewidth=1.4, zorder=3)
            ax.text(hi + 0.035, i, f"{int(r['goal'])}/{int(r['episodes'])}",
                    va="center", ha="left", fontsize=8.5, color=INK)
        ax.set_yticks(ys)
        ax.set_yticklabels([nice[n] for n in order], fontsize=8.5)
        ax.invert_yaxis()
        ax.set_xlim(-0.04, 1.30)
        ax.set_xticks([0, 0.25, 0.5, 0.75, 1.0])
        ax.set_xticklabels(["0%", "25%", "50%", "75%", "100%"])
        ax.set_title(title, fontsize=9.5, color=INK, pad=8, loc="left")
        ax.set_xlabel("episodes reaching the goal", fontsize=9, color=INK_2)

    fig.suptitle(
        "Goal-reaching rate with 95% Wilson intervals  ·  90 episodes total  ·  "
        "one fixed obstacle layout",
        fontsize=10.5, color=INK, x=0.012, ha="left", y=1.02)
    fig.text(0.012, -0.07,
             "Dots are point estimates; bars are 95% confidence intervals. The "
             "static cells hold only 3 episodes each, so their intervals span most "
             "of the range and separate nothing. The two scripted baselines were "
             "not tuned for this scene and are a sanity floor, not a method "
             "comparison.",
             fontsize=8, color=INK_2, ha="left", va="top", wrap=True)

    out = ASSETS / "figures" / "success_rate.png"
    save(fig, out)
    record(out, "experiment result",
           ["results/experiments_warehouse/experiment_summary.csv"],
           "Point estimates and Wilson 95% intervals computed by "
           "tests/client_experiments.py over 90 recorded episodes.")
    return out


# --------------------------------------------------------------------------
# F. trajectories
# --------------------------------------------------------------------------
def trajectories_figure(scenario: dict) -> Path | None:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Circle
    src = RESULTS / "experiments_warehouse" / "traces.json"
    if not src.is_file():
        print(f"  MISSING {src.relative_to(PROJECT_ROOT)}")
        return None
    traces = json.loads(src.read_text(encoding="utf8"))
    summary_path = RESULTS / "experiments_warehouse" / "experiment_results.csv"
    first = {}
    if summary_path.is_file():
        for r in csv.DictReader(summary_path.open(encoding="utf8")):
            first.setdefault(f"{r['mode']}/{r['controller']}", r["outcome"])

    lo, hi = scenario["arena"]["low_xy"], scenario["arena"]["high_xy"]
    series = [("ppo_seed7", METHOD["ppo"], "-", "PPO seed 7"),
              ("ppo_seed19", METHOD["ppo"], "--", "PPO seed 19"),
              ("ppo_seed43", METHOD["ppo"], ":", "PPO seed 43"),
              ("direct", METHOD["direct"], "-", "goal-direct (scripted)"),
              ("reactive", METHOD["reactive"], "-", "reactive rule (scripted)")]
    marks = {"goal": ("*", "reached goal"), "collision": ("X", "collision"),
             "timeout": ("s", "timeout")}

    # Mode B is a different scenario file: the cart is parked against the wall
    # rather than absent, so the obstacle count and the LiDAR scene stay
    # identical between the two modes. Draw each panel from its own config.
    static_path = PROJECT_ROOT / "config" / "scenario_warehouse_static.json"
    static = load_scenario(static_path) if static_path.is_file() else scenario

    fig = new_figure(10.0, 7.2)
    axes = fig.subplots(2, 1)
    for ax, mode, scene, title in zip(
            axes, ["B_static", "C_dynamic"], [static, scenario],
            ["Mode B — static (cart parked against the wall)",
             "Mode C — dynamic (cart sweeping the aisle)"]):
        style_axes(ax)
        ax.set_aspect("equal")
        for obs in scene["obstacles"]:
            sweeps = float(obs.get("motion", {}).get("amplitude_m", 0.0)) > 0.0
            ax.add_patch(Polygon(footprint(obs), closed=True,
                                 facecolor=obstacle_colour(obs),
                                 edgecolor=obstacle_colour(obs),
                                 alpha=0.50 if sweeps else 0.80,
                                 linestyle="--" if sweeps else "-", zorder=2))
        gx, gy = scenario["goal"]["xy"]
        ax.add_patch(Circle((gx, gy), scenario["goal"]["tolerance_m"],
                            facecolor=C_GOAL, alpha=0.14, edgecolor=C_GOAL,
                            linestyle="--", zorder=2))
        sx, sy = scenario["drone"]["start_xy"]
        ax.plot([sx], [sy], marker="o", markersize=8, color=C_START,
                markeredgecolor="#ffffff", markeredgewidth=1.3, zorder=6)

        for name, colour, dash, label in series:
            trace = traces.get(f"{mode}/{name}")
            if not trace:
                continue
            xy = np.array([s["xy"] for s in trace])
            ax.plot(xy[:, 0], xy[:, 1], color=colour, linewidth=1.7,
                    linestyle=dash, label=label, zorder=4, alpha=0.95)
            outcome = first.get(f"{mode}/{name}", "")
            marker, _ = marks.get(outcome, ("o", outcome))
            ax.plot([xy[-1, 0]], [xy[-1, 1]], marker=marker, markersize=10,
                    color=colour, markeredgecolor="#ffffff", markeredgewidth=1.2,
                    zorder=7)

        ax.set_xlim(lo[0] - 0.3, hi[0] + 0.3)
        ax.set_ylim(hi[1] + 0.3, lo[1] - 0.3)
        ax.set_title(title, fontsize=9.5, color=INK, pad=8, loc="left")
        ax.set_ylabel("DNS y — right (m)", fontsize=9, color=INK_2)
    axes[-1].set_xlabel("DNS x — forward (m)", fontsize=9, color=INK_2)

    handles, labels = axes[0].get_legend_handles_labels()
    leg = fig.legend(handles, labels, frameon=False, fontsize=8.5, ncol=5,
                     loc="lower center", bbox_to_anchor=(0.5, -0.045))
    for text in leg.get_texts():
        text.set_color(INK_2)
    fig.suptitle(
        "First recorded episode of each controller  ·  star = goal, "
        "X = collision, square = timeout",
        fontsize=10.5, color=INK, x=0.012, ha="left", y=0.98)

    out = ASSETS / "figures" / "trajectories.png"
    save(fig, out)
    record(out, "experiment result",
           ["results/experiments_warehouse/traces.json",
            "results/experiments_warehouse/experiment_results.csv",
            "config/scenario_warehouse.json"],
           "Flown paths as recorded by the simulator for the first episode of "
           "each mode/controller cell, including the episodes that failed.")
    return out


# --------------------------------------------------------------------------
# G. video thumbnail
# --------------------------------------------------------------------------
def video_thumbnail() -> Path | None:
    mp4 = RESULTS / "demo" / "DNS_Webots_Research_Demo.mp4"
    if not mp4.is_file():
        print(f"  MISSING {mp4.relative_to(PROJECT_ROOT)}")
        return None
    try:
        import imageio_ffmpeg
        exe = imageio_ffmpeg.get_ffmpeg_exe()
    except Exception as exc:                                      # noqa: BLE001
        print(f"  imageio-ffmpeg unavailable ({exc}); skipping thumbnail")
        return None

    out = ASSETS / "demo-thumbnail.jpg"
    out.parent.mkdir(parents=True, exist_ok=True)
    # 22 s in: mid-run, drone between the two racks rather than on the pad.
    proc = subprocess.run(
        [exe, "-y", "-loglevel", "error", "-ss", "22.0", "-i", str(mp4),
         "-frames:v", "1", "-q:v", "3", str(out)],
        capture_output=True, text=True)
    if proc.returncode != 0 or not out.is_file():
        print(f"  thumbnail extraction failed: {proc.stderr[:300]}")
        return None
    record(out, "frame extracted from the demonstration video",
           ["results/demo/DNS_Webots_Research_Demo.mp4"],
           "Single frame at t = 22.0 s of the recorded Webots run; not re-rendered.")
    return out


# --------------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=PROJECT_ROOT / "config" / "scenario_warehouse.json")
    args = parser.parse_args(argv)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams.update({
            "font.family": "sans-serif",
            "font.sans-serif": ["Segoe UI", "DejaVu Sans", "Arial"],
            "axes.titlecolor": INK,
            "figure.facecolor": SURFACE,
            "savefig.facecolor": SURFACE,
        })
    except ImportError:
        print("matplotlib is required: pip install -r requirements.txt")
        return 1

    scenario = load_scenario(args.config)
    shots_path = RESULTS / "screenshots" / "screenshots.json"
    shots = json.loads(shots_path.read_text(encoding="utf8")) if shots_path.is_file() else {"views": []}

    print(f"building assets from {args.config.name}\n")
    annotate_hero(scenario, shots)
    environment_map(scenario)
    moving_obstacle_figure()
    lidar_figure()
    success_rate_figure()
    trajectories_figure(scenario)
    video_thumbnail()

    manifest = ASSETS / "ASSET_PROVENANCE.json"
    manifest.write_text(json.dumps({
        "scenario": scenario["name"],
        "config": str(args.config.relative_to(PROJECT_ROOT)).replace("\\", "/"),
        "webots_version": "R2025a",
        "statement": "Every asset listed here is derived from a file produced by "
                     "a real run of this project. No asset is an artist's "
                     "impression and none was sourced from a third party.",
        "assets": PROVENANCE,
    }, indent=2), encoding="utf8")
    print(f"\n{len(PROVENANCE)} assets written; manifest at "
          f"{manifest.relative_to(PROJECT_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
