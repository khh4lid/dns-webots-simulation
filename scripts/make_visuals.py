"""Stage B visuals that are computed from logs rather than captured in Webots.

Produces, all labelled LOGGED-DATA RENDER or SCHEMATIC:

* ``F4``  verification panel (observation dt, reset, LiDAR vs ray caster,
  LiDAR-cart consistency over a crossing, box-vs-circle geometry)
* ``mirror_overlay``  L00 vs L02 trajectories reflected onto one another --
  the visual proof that L02 was actually flown mirrored
* ``mirror_phases``  phase-by-phase outcome comparison, L00 vs L02
* ``scorecard``  six layout thumbnails with the pooled PPO Mode C result
* ``shift_ladder.gif``  L00 -> L05 maps, corridor width and IoU on each frame
* ADR images: racks crossing the straight line, circle-vs-box false collision,
  the timing ladder, the kinematic cart in space-time, the reactive
  repulsion zone

Run::

    python scripts/make_visuals.py           # everything
    python scripts/make_visuals.py F4        # one item
"""
from __future__ import annotations

import csv
import json
import math
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import dns_style as S                                            # noqa: E402
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.lines import Line2D                              # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle, Wedge  # noqa: E402

from dns.feasibility import describe                             # noqa: E402
from dns.scenario import Obstacle, Scenario                      # noqa: E402

SCENARIOS = ROOT / "config" / "scenarios"
RESULTS = ROOT / "results" / "v2_S6"
RUNS = RESULTS / "runs"
FIGURES = ROOT / "assets" / "figures"
GIFS = ROOT / "assets" / "gifs"
DECISIONS = ROOT / "assets" / "decisions"
LAYOUTS = ("L00", "L01", "L02", "L03", "L04", "L05")


def load_csv(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open(encoding="utf8") as handle:
        return list(csv.DictReader(handle))


def trajectory(run_id: str):
    rows = load_csv(RUNS / run_id / "trajectory.csv")
    if not rows:
        return None
    return np.array([[float(r["x_m"]), float(r["y_m"])] for r in rows])


def footprint(obstacle, t: float = 0.0) -> np.ndarray:
    placed = obstacle.placed_at(t)
    half = np.asarray(placed.half_size, dtype=float)
    local = np.array([[-half[0], -half[1]], [half[0], -half[1]],
                      [half[0], half[1]], [-half[0], half[1]]])
    c, s = math.cos(placed.yaw_rad), math.sin(placed.yaw_rad)
    return local @ np.array([[c, s], [-s, c]]) + np.asarray(placed.centre)


def sketch_layout(axis, scenario: Scenario, *, cart=True, alpha=1.0) -> None:
    low = np.asarray(scenario.low_xy, dtype=float)
    high = np.asarray(scenario.high_xy, dtype=float)
    axis.add_patch(Rectangle((low[0], low[1]), high[0] - low[0],
                             high[1] - low[1], facecolor="none",
                             edgecolor=S.WALL, linewidth=1.2, zorder=3))
    for obstacle in scenario.obstacles:
        moving = obstacle.motion.type != "static" and obstacle.motion.amplitude_m
        if moving and not cart:
            continue
        axis.add_patch(Polygon(footprint(obstacle), closed=True, alpha=alpha,
                               facecolor=S.CART if moving else S.STATIC_OBSTACLE,
                               edgecolor=S.STATIC_EDGE, linewidth=0.5, zorder=5))
    axis.add_patch(Circle(scenario.start_xy, 0.14, facecolor=S.START,
                          edgecolor="white", linewidth=0.7, zorder=10))
    axis.add_patch(Circle(scenario.goal_xy, scenario.goal_tolerance_m,
                          facecolor=S.GOAL, alpha=0.18, edgecolor=S.GOAL,
                          linewidth=0.8, zorder=9))
    axis.set_xlim(low[0] - 0.2, high[0] + 0.2)
    axis.set_ylim(low[1] - 0.2, high[1] + 0.2)
    axis.set_aspect("equal", adjustable="box")


# ---------------------------------------------------------------------------
# F4 - verification panel
# ---------------------------------------------------------------------------
def figure_f4() -> None:
    figure, axes = plt.subplots(2, 2, figsize=(7.4, 5.4))

    # (a) observation interval, measured from every logged trajectory
    axis = axes[0, 0]
    deltas = []
    for folder in sorted(RUNS.glob("L0*_v00"))[:60]:
        rows = load_csv(folder / "trajectory.csv")
        times = [float(r["t_s"]) for r in rows]
        deltas.extend(np.diff(times).tolist())
    deltas = np.asarray(deltas)
    if deltas.size:
        error_ns = (deltas - 0.2) * 1e9
        axis.hist(error_ns, bins=41, color=S.CONTROLLER_COLOUR["ppo_pooled"],
                  edgecolor="white", linewidth=0.5)
        axis.set_xlabel("observation interval error [ns]  (target 0.200000 s)")
        axis.set_ylabel("control steps")
        axis.annotate(f"n = {deltas.size} steps\nmax |error| = "
                      f"{np.abs(error_ns).max():.3g} ns",
                      (0.03, 0.95), xycoords="axes fraction", fontsize=7,
                      va="top", color=S.INK_SOFT)
    axis.set_title("(a) Observation interval", loc="left", fontsize=8.6)
    S.grid(axis, "y")

    # (b) LiDAR vs analytic ray caster, all sectors
    axis = axes[0, 1]
    consistency = RESULTS / "lidar_cart_consistency.json"
    if consistency.is_file():
        report = json.loads(consistency.read_text(encoding="utf8"))
        maes = [r["all_sectors"]["mae_m"] for r in report["runs"]]
        carts = [r["cart_sector_only"]["mae_m"] for r in report["runs"]
                 if "cart_sector_only" in r]
        positions = np.arange(len(maes))
        axis.bar(positions - 0.2, maes, width=0.38, color="#9aa0a6",
                 edgecolor="white", linewidth=0.6, label="all sectors")
        axis.bar(positions + 0.2, carts, width=0.38, color=S.CART,
                 edgecolor="white", linewidth=0.6, label="cart sector")
        axis.set_xticks(positions)
        axis.set_xticklabels([r["layout"] for r in report["runs"]],
                             rotation=90, fontsize=5.6)
        axis.set_ylabel("mean |error| [m]")
        pooled = report["cart_sector_pooled"]
        axis.annotate(f"cart sector pooled\nn = {pooled['comparisons']}, "
                      f"MAE = {pooled['mae_m']:.4f} m",
                      (0.97, 0.95), xycoords="axes fraction", fontsize=7,
                      ha="right", va="top", color=S.INK_SOFT)
        axis.legend(loc="upper left", fontsize=6.4)
    axis.set_title("(b) LiDAR vs analytic ray caster", loc="left", fontsize=8.6)
    S.grid(axis, "y")

    # (c) LiDAR-cart consistency through a crossing
    axis = axes[1, 0]
    rows = load_csv(RESULTS / "lidar_cart_consistency.csv")
    pick = [r for r in rows if r["run_id"].startswith("L05_C_ppo_seed7")]
    if pick:
        times = [float(r["t_s"]) for r in pick]
        axis.plot(times, [float(r["measured_m"]) for r in pick], color=S.CART,
                  linewidth=1.6, label="measured sector range")
        axis.plot(times, [float(r["raycast_m"]) for r in pick], color=S.INK,
                  linewidth=1.0, linestyle=(0, (3, 2)), label="analytic ray caster")
        axis.set_xlabel("simulated time [s]")
        axis.set_ylabel("range to cart [m]")
        axis.legend(loc="upper right", fontsize=6.6)
    axis.set_title("(c) Cart tracked through a crossing", loc="left", fontsize=8.6)
    S.grid(axis, "both")

    # (d) box vs circle: the v1 false-collision defect
    axis = axes[1, 1]
    rack = Obstacle(id="rack", xy=(0.0, 0.0), shape="box", size_xy=(0.8, 2.4),
                    height_m=2.0)
    axis.add_patch(Polygon(footprint(rack), closed=True,
                           facecolor=S.STATIC_OBSTACLE, edgecolor=S.STATIC_EDGE,
                           linewidth=0.8, zorder=5))
    bounding = rack.bounding_radius
    axis.add_patch(Circle((0, 0), bounding, facecolor=S.OUTCOME_COLOUR["collision_static"],
                          alpha=0.16, edgecolor=S.OUTCOME_COLOUR["collision_static"],
                          linewidth=1.0, linestyle=(0, (4, 2)), zorder=4))
    probe = np.array([0.9, 1.1])
    true_gap = float(rack.placed_at(0.0).surface_distance(probe[None, :])[0])
    axis.plot(*probe, marker="o", markersize=6, color=S.GOAL, zorder=8)
    axis.annotate(f"drone here:\ntrue gap {true_gap:.2f} m\n"
                  f"circle model says COLLISION",
                  probe, textcoords="offset points", xytext=(8, -4),
                  fontsize=6.8, color=S.INK_SOFT)
    axis.annotate(f"bounding circle r = {bounding:.3f} m",
                  (0, -bounding), textcoords="offset points", xytext=(0, -12),
                  fontsize=6.6, ha="center",
                  color=S.OUTCOME_COLOUR["collision_static"])
    axis.set_xlim(-1.8, 2.2)
    axis.set_ylim(-1.8, 1.8)
    axis.set_aspect("equal")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title("(d) Exact box vs bounding circle", loc="left", fontsize=8.6)

    for axis in axes.ravel():
        S.provenance(axis, "LOGGED-DATA RENDER", loc="lower right")
    axes[1, 1].texts[-1].set_text("SCHEMATIC")
    figure.suptitle("F4  Verification panel — we check the simulator instead of trusting it",
                    x=0.02, ha="left", fontsize=10.5, fontweight="bold")
    figure.tight_layout(rect=(0, 0, 1, 0.95))
    S.save(figure, FIGURES / "F4_verification_panel")
    print("ok   F4")


# ---------------------------------------------------------------------------
# mirror evidence
# ---------------------------------------------------------------------------
def figure_mirror_overlay() -> None:
    base = Scenario.load(SCENARIOS / "L00_C.json")
    mirror = Scenario.load(SCENARIOS / "L02_C.json")
    figure, axes = plt.subplots(3, 1, figsize=(7.2, 7.6))

    for axis, (scenario, label) in zip(axes[:2],
                                       ((base, "L00 — baseline"),
                                        (mirror, "L02 — mirrored"))):
        sketch_layout(axis, scenario)
        name = "L00" if scenario is base else "L02"
        for controller in ("ppo_seed7", "ppo_seed19", "ppo_seed43"):
            points = trajectory(f"{name}_C_{controller}_v00")
            if points is not None:
                axis.plot(points[:, 0], points[:, 1],
                          color=S.CONTROLLER_COLOUR[controller], linewidth=1.5,
                          zorder=11, label=controller.replace("ppo_", ""))
        axis.set_title(label, loc="left", fontsize=8.8)
        axis.set_ylabel("y [m]")
        axis.set_yticks([-3, 0, 3])
        S.provenance(axis, "LOGGED-DATA RENDER", loc="upper right")
    axes[0].legend(loc="upper left", bbox_to_anchor=(1.005, 1.0),
                   fontsize=6.8, title="checkpoint", title_fontsize=6.8)

    # Third panel: L02 reflected back onto L00. If the flight really was
    # mirrored, the reflected L02 track lies on the L00 track.
    axis = axes[2]
    sketch_layout(axis, base, cart=False, alpha=0.35)
    for controller in ("ppo_seed7", "ppo_seed19", "ppo_seed43"):
        a = trajectory(f"L00_C_{controller}_v00")
        b = trajectory(f"L02_C_{controller}_v00")
        if a is None or b is None:
            continue
        axis.plot(a[:, 0], a[:, 1], color=S.CONTROLLER_COLOUR[controller],
                  linewidth=1.5, zorder=11)
        axis.plot(b[:, 0], -b[:, 1], color=S.CONTROLLER_COLOUR[controller],
                  linewidth=1.2, linestyle=(0, (3, 2)), alpha=0.9, zorder=12)
    axis.set_title("L00 (solid) with L02 reflected about y = 0 (dashed)",
                   loc="left", fontsize=8.8)
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_yticks([-3, 0, 3])
    axis.annotate("the two sets do NOT coincide: the same checkpoint flies a\n"
                  "different route on the mirrored layout, yet the pooled\n"
                  "success rate is identical (24/45 on both)",
                  (0.02, 0.04), xycoords="axes fraction", fontsize=6.8,
                  color=S.INK_SOFT, va="bottom")
    S.provenance(axis, "LOGGED-DATA RENDER", loc="upper right")

    figure.suptitle("Mirror test — L02 is L00 reflected about y = 0",
                    x=0.02, ha="left", fontsize=10.2, fontweight="bold")
    figure.tight_layout(rect=(0, 0.055, 0.86, 0.955), h_pad=2.2)
    S.save(figure, FIGURES / "E15_mirror_overlay")
    print("ok   mirror_overlay")


def figure_mirror_phases() -> None:
    rows = load_csv(RESULTS / "results_v2.csv")
    if not rows:
        return
    controllers = ("ppo_seed7", "ppo_seed19", "ppo_seed43")
    figure, axes = plt.subplots(3, 1, figsize=(7.0, 4.2), sharex=True)
    order = ["goal", "collision_static", "collision_cart", "collision_wall",
             "timeout"]
    index = {name: i for i, name in enumerate(order)}

    for axis, controller in zip(axes, controllers):
        for layout, offset, marker in (("L00", -0.18, "o"), ("L02", 0.18, "s")):
            cell = sorted([r for r in rows if r["layout_id"] == layout
                           and r["mode"] == "C_dynamic"
                           and r["controller"] == controller],
                          key=lambda r: r["variant"])
            xs = np.arange(len(cell)) + offset
            ys = [index.get(r["outcome"], len(order)) for r in cell]
            axis.plot(xs, ys, marker=marker, linestyle="none", markersize=5,
                      color=S.CONTROLLER_COLOUR[controller],
                      alpha=1.0 if layout == "L00" else 0.5,
                      markeredgecolor="white", markeredgewidth=0.6,
                      label=layout, zorder=5)
        same = sum(1 for a, b in zip(
            sorted([r for r in rows if r["layout_id"] == "L00"
                    and r["mode"] == "C_dynamic" and r["controller"] == controller],
                   key=lambda r: r["variant"]),
            sorted([r for r in rows if r["layout_id"] == "L02"
                    and r["mode"] == "C_dynamic" and r["controller"] == controller],
                   key=lambda r: r["variant"]))
            if a["outcome"] == b["outcome"])
        axis.set_yticks(range(len(order)))
        axis.set_yticklabels([o.replace("_", " ") for o in order], fontsize=6.2)
        axis.set_ylabel(controller.replace("ppo_", ""), fontsize=7.4)
        axis.annotate(f"same outcome on {same}/15 cart phases",
                      (0.99, 0.08), xycoords="axes fraction", fontsize=6.6,
                      ha="right", color=S.INK_SOFT)
        S.grid(axis, "y")
    axes[0].legend(loc="upper left", ncol=2, fontsize=6.6)
    axes[-1].set_xlabel("cart-phase variant (v00 … v14) — identical phases on both layouts")
    axes[-1].set_xticks(range(15))
    axes[-1].set_xticklabels([f"{i:02d}" for i in range(15)], fontsize=6)
    figure.text(0.02, 0.005, "LOGGED-DATA RENDER", fontsize=6.0,
                color=S.INK_MUTED)
    figure.suptitle("Mirror test, phase by phase — identical cart phases, "
                    "L00 vs L02", x=0.02, ha="left", fontsize=9.8,
                    fontweight="bold")
    figure.tight_layout(rect=(0, 0.015, 1, 0.94))
    S.save(figure, FIGURES / "E16_mirror_phases")
    print("ok   mirror_phases")


# ---------------------------------------------------------------------------
# scorecard
# ---------------------------------------------------------------------------
def figure_scorecard() -> None:
    pooled = {(r["layout_id"], r["mode"], r["group"]): r
              for r in load_csv(RESULTS / "pooled_rates.csv")}
    figure, axes = plt.subplots(1, 6, figsize=(10.6, 2.5))
    for axis, layout in zip(axes, LAYOUTS):
        path = SCENARIOS / f"{layout}_C.json"
        if not path.is_file():
            axis.axis("off")
            continue
        scenario = Scenario.load(path)
        sketch_layout(axis, scenario)
        axis.set_xticks([])
        axis.set_yticks([])
        for spine in axis.spines.values():
            spine.set_visible(False)
        entry = pooled.get((layout, "C_dynamic", "ppo_pooled"))
        meta = json.loads(path.read_text(encoding="utf8"))["provenance"]
        if entry:
            rate = float(entry["rate"])
            colour = (S.OUTCOME_COLOUR["goal"] if rate >= 0.5
                      else S.CART if rate > 0 else S.OUTCOME_COLOUR["collision_static"])
            axis.set_title(f"{layout} · {meta['layout_title']}", fontsize=7.6,
                           loc="center", pad=3)
            note = " (excluded)" if layout == "L04" else ""
            axis.annotate(f"{entry['k']}/{entry['n']} = {rate:.0%}{note}",
                          (0.5, -0.10), xycoords="axes fraction", fontsize=8.2,
                          ha="center", va="top", color=colour, fontweight="bold")
            axis.annotate(f"[{float(entry['wilson_low']):.2f}, "
                          f"{float(entry['wilson_high']):.2f}]",
                          (0.5, -0.26), xycoords="axes fraction", fontsize=6.4,
                          ha="center", va="top", color=S.INK_SOFT)
    figure.text(0.012, 0.02, "LOGGED-DATA RENDER · pooled PPO success, Mode C, "
                             "n = 45 distinct cart phases per layout · "
                             "Wilson 95% in brackets",
                fontsize=6.4, color=S.INK_MUTED)
    figure.suptitle("Results scorecard — PPO pooled success across the shift ladder",
                    x=0.012, ha="left", fontsize=10.0, fontweight="bold")
    figure.tight_layout(rect=(0, 0.06, 1, 0.92))
    S.save(figure, FIGURES / "scorecard")
    print("ok   scorecard")


# ---------------------------------------------------------------------------
# shift-ladder GIF
# ---------------------------------------------------------------------------
def gif_shift_ladder() -> None:
    import imageio_ffmpeg                                        # noqa: F401
    table = {r["layout_id"]: r for r in load_csv(RESULTS / "scenario_table.csv")
             if r["mode"] == "C"}
    frames = []
    for layout in LAYOUTS:
        path = SCENARIOS / f"{layout}_C.json"
        if not path.is_file():
            continue
        scenario = Scenario.load(path)
        meta = json.loads(path.read_text(encoding="utf8"))["provenance"]
        facts, astar_path, _grid = describe(scenario)
        figure, axis = plt.subplots(figsize=(6.4, 4.0))
        sketch_layout(axis, scenario)
        if astar_path:
            points = np.asarray(astar_path)
            axis.plot(points[:, 0], points[:, 1], color=S.ASTAR, linewidth=1.4,
                      linestyle=(0, (4, 1.6)), zorder=8)
        descriptor = table.get(layout, {})
        axis.set_title(f"{layout} — {meta['layout_title']}", loc="left",
                       fontsize=11, fontweight="bold")
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.annotate(
            f"corridor {facts['min_corridor_width_enroute_m']:.2f} m     "
            f"IoU vs L00 {float(descriptor.get('iou_to_L00', 0)):.3f}     "
            f"A* {facts['astar_length_m']:.2f} m",
            (0.5, -0.20), xycoords="axes fraction", fontsize=8.6, ha="center",
            va="top", color=S.INK)
        S.scale_bar(axis, 1.0, origin=(0.3, -2.75))
        S.provenance(axis, "LOGGED-DATA RENDER", loc="upper right")
        figure.tight_layout(rect=(0, 0.06, 1, 1))
        figure.canvas.draw()
        buffer = np.asarray(figure.canvas.buffer_rgba())[:, :, :3].copy()
        frames.extend([buffer] * 2)          # ~1 s per layout at 2 fps
        plt.close(figure)

    GIFS.mkdir(parents=True, exist_ok=True)
    write_gif(frames, GIFS / "shift_ladder.gif", fps=2)
    print(f"ok   shift_ladder.gif ({len(frames)} frames)")


def write_gif(frames: list[np.ndarray], path: Path, fps: int = 10) -> None:
    """Encode a GIF through imageio-ffmpeg's binary with a generated palette.

    Webots' bundled ffmpeg does not run in this environment, so the binary that
    ships with imageio-ffmpeg is used. A per-clip palette (palettegen /
    paletteuse) keeps a 256-colour GIF of a figure readable instead of dithered
    into mud.
    """
    import subprocess
    import imageio_ffmpeg

    if not frames:
        return
    height, width = frames[0].shape[:2]
    # Even dimensions, required by most encoders.
    width -= width % 2
    height -= height % 2
    raw = b"".join(f[:height, :width].tobytes() for f in frames)
    exe = imageio_ffmpeg.get_ffmpeg_exe()
    path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        exe, "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{width}x{height}", "-r", str(fps), "-i", "pipe:0",
        "-filter_complex",
        "[0:v]scale=900:-1:flags=lanczos,split[a][b];"
        "[a]palettegen=max_colors=192[p];[b][p]paletteuse=dither=bayer:bayer_scale=3",
        "-loop", "0", str(path),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE,
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.PIPE)
    _out, error = process.communicate(raw)
    if process.returncode != 0:
        raise RuntimeError(f"ffmpeg failed: {error.decode()[:300]}")
    size = path.stat().st_size / 1e6
    if size > 8.0:
        print(f"  !! {path.name} is {size:.1f} MB, above the 8 MB limit")


# ---------------------------------------------------------------------------
# ADR decision images
# ---------------------------------------------------------------------------
def figure_adr_line_block() -> None:
    """ADR-005: the racks cross the straight start-goal line on purpose."""
    scenario = Scenario.load(SCENARIOS / "L00_C.json")
    figure, axis = plt.subplots(figsize=(6.6, 3.0))
    sketch_layout(axis, scenario, cart=False)
    start = np.asarray(scenario.start_xy)
    goal = np.asarray(scenario.goal_xy)
    axis.plot([start[0], goal[0]], [start[1], goal[1]],
              color=S.OUTCOME_COLOUR["collision_static"], linewidth=1.6,
              linestyle=(0, (5, 2)), zorder=12)
    for obstacle in scenario.obstacles:
        if not obstacle.id.startswith("rack"):
            continue
        corners = footprint(obstacle)
        depth = (corners[:, 1].max() if obstacle.xy[1] < 0
                 else -corners[:, 1].min())
        x = obstacle.xy[0]
        axis.annotate(f"crosses y = 0 by {depth:.2f} m",
                      (x, 0.0), textcoords="offset points",
                      xytext=(0, 26 if obstacle.xy[1] < 0 else -34),
                      fontsize=7.2, ha="center",
                      color=S.OUTCOME_COLOUR["collision_static"],
                      arrowprops=dict(arrowstyle="->", linewidth=0.9,
                                      color=S.OUTCOME_COLOUR["collision_static"]))
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title("ADR-005 — the racks deliberately block the straight line",
                   loc="left", fontsize=9.2, fontweight="bold")
    axis.annotate("An earlier layout left the line open; the goal-direct\n"
                  "baseline then scored 3/3 and the static condition\n"
                  "measured nothing.",
                  (0.015, 0.05), xycoords="axes fraction", fontsize=6.8,
                  color=S.INK_SOFT, va="bottom")
    S.scale_bar(axis, 1.0, origin=(0.3, -2.75))
    S.provenance(axis, "LOGGED-DATA RENDER", loc="upper right")
    figure.tight_layout()
    S.save(figure, DECISIONS / "ADR-005_line_block", vector=False)
    print("ok   ADR-005")


def figure_adr_timing() -> None:
    """ADR-008: the three clocks and how they nest."""
    figure, axis = plt.subplots(figsize=(6.8, 2.4))
    rows = [("physics step", 0.008, S.STATIC_OBSTACLE, "8 ms — Webots basicTimeStep"),
            ("autopilot inner loop", 0.008, S.CONTROLLER_COLOUR["ppo_seed43"],
             "125 Hz — attitude and thrust"),
            ("policy command", 0.200, S.CONTROLLER_COLOUR["ppo_pooled"],
             "5 Hz — one velocity setpoint per 25 physics steps")]
    for index, (label, period, colour, note) in enumerate(rows):
        y = len(rows) - index - 1
        count = int(round(0.2 / period))
        for k in range(count):
            axis.add_patch(Rectangle((k * period, y - 0.3), period * 0.86, 0.6,
                                     facecolor=colour, edgecolor="white",
                                     linewidth=0.5))
        axis.annotate(note, (0.205, y), fontsize=7.2, va="center",
                      color=S.INK_SOFT)
        axis.annotate(label, (-0.004, y), fontsize=7.6, va="center", ha="right",
                      color=S.INK, fontweight="bold")
    axis.set_xlim(-0.075, 0.42)
    axis.set_ylim(-0.75, len(rows) - 0.25)
    axis.set_yticks([])
    axis.set_xticks([0, 0.05, 0.1, 0.15, 0.2])
    axis.set_xlabel("simulated time [s]")
    for spine in ("left", "right", "top"):
        axis.spines[spine].set_visible(False)
    axis.set_title("ADR-008 — one policy command per 25 physics steps, exactly",
                   loc="left", fontsize=9.2, fontweight="bold")
    S.provenance(axis, "SCHEMATIC", loc="lower right")
    figure.tight_layout()
    S.save(figure, DECISIONS / "ADR-008_timing_ladder", vector=False)
    print("ok   ADR-008")


def figure_adr_cart_spacetime() -> None:
    """ADR-006: the kinematic cart is an exact function of time."""
    scenario = Scenario.load(SCENARIOS / "L00_C.json")
    cart = next(o for o in scenario.obstacles if o.motion.type == "linear")
    period = 4.0 * cart.motion.amplitude_m / cart.motion.speed_mps
    times = np.linspace(0.0, 2 * period, 600)
    ys = np.array([cart.centre_at(t)[1] for t in times])

    figure, axis = plt.subplots(figsize=(6.6, 2.8))
    axis.plot(times, ys, color=S.CART, linewidth=2.0, zorder=5)
    axis.axhline(0.0, color=S.GRID, linewidth=0.8)
    axis.annotate(f"period {period:.4f} s  ·  speed {cart.motion.speed_mps} m/s  ·  "
                  f"amplitude ±{cart.motion.amplitude_m} m",
                  (0.015, 0.94), xycoords="axes fraction", fontsize=7.4,
                  va="top", color=S.INK_SOFT)
    axis.annotate("t = 0 pose is the −1.9 m extreme,\n"
                  "not the sweep centre (v1 defect, now a regression test)",
                  (0.0, ys[0]), textcoords="offset points", xytext=(16, 10),
                  fontsize=6.8, color=S.INK_SOFT,
                  arrowprops=dict(arrowstyle="->", linewidth=0.9, color=S.INK_SOFT))
    axis.set_xlabel("simulated time [s]")
    axis.set_ylabel("cart y [m]")
    axis.set_title("ADR-006 — the cart's pose is an exact function of time",
                   loc="left", fontsize=9.2, fontweight="bold")
    S.grid(axis, "both")
    S.provenance(axis, "LOGGED-DATA RENDER", loc="lower right")
    figure.tight_layout()
    S.save(figure, DECISIONS / "ADR-006_cart_spacetime", vector=False)
    print("ok   ADR-006")


def figure_adr_reactive() -> None:
    """C1 / ADR context: the repulsion zone around the goal."""
    report_path = RESULTS / "reactive_gnron_analysis.json"
    if not report_path.is_file():
        return
    report = json.loads(report_path.read_text(encoding="utf8"))
    entry = report["layouts"]["L00"]
    crate = entry["surface_vs_centre_to_goal"]["crate_stack"]
    scenario = Scenario.load(SCENARIOS / "L00_B.json")

    figure, axis = plt.subplots(figsize=(6.2, 3.4))
    sketch_layout(axis, scenario, cart=False)
    goal = np.asarray(scenario.goal_xy)
    axis.add_patch(Circle(goal, report["repulsion_radius_m"], facecolor=S.CART,
                          alpha=0.14, edgecolor=S.CART, linewidth=1.1,
                          linestyle=(0, (4, 2)), zorder=6))
    axis.annotate(f"gap_rule repulsion radius {report['repulsion_radius_m']} m",
                  (goal[0], goal[1] + report["repulsion_radius_m"]),
                  textcoords="offset points", xytext=(-6, 6), fontsize=7.0,
                  ha="right", color=S.CART)
    stop = entry.get("reactive_mode_b", {}).get("final", {})
    if stop:
        axis.plot(stop["x_m"], stop["y_m"], marker="X", markersize=9,
                  color=S.CONTROLLER_COLOUR["reactive"], zorder=12)
        axis.annotate(f"timeout here, {stop['remaining_m']:.2f} m short",
                      (stop["x_m"], stop["y_m"]), textcoords="offset points",
                      xytext=(6, -16), fontsize=7.0,
                      color=S.CONTROLLER_COLOUR["reactive"])
    points = trajectory("L00_B_reactive_v00")
    if points is not None:
        axis.plot(points[:, 0], points[:, 1],
                  color=S.CONTROLLER_COLOUR["reactive"], linewidth=1.4, zorder=11)
    axis.annotate(f"crate surface → goal {crate['surface_to_goal_m']:.4f} m\n"
                  f"(v1 quoted the CENTRE, {crate['centre_to_goal_m']:.2f} m)",
                  (0.015, 0.06), xycoords="axes fraction", fontsize=7.0,
                  color=S.INK_SOFT, va="bottom")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title("C1 — what actually stops the reactive controller",
                   loc="left", fontsize=9.2, fontweight="bold")
    S.provenance(axis, "LOGGED-DATA RENDER", loc="upper right")
    figure.tight_layout()
    S.save(figure, DECISIONS / "ADR-C1_reactive_zone", vector=False)
    print("ok   C1 reactive")


ITEMS = {
    "F4": figure_f4,
    "mirror_overlay": figure_mirror_overlay,
    "mirror_phases": figure_mirror_phases,
    "scorecard": figure_scorecard,
    "shift_ladder": gif_shift_ladder,
    "ADR005": figure_adr_line_block,
    "ADR008": figure_adr_timing,
    "ADR006": figure_adr_cart_spacetime,
    "C1": figure_adr_reactive,
}


def main() -> int:
    S.apply()
    for directory in (FIGURES, GIFS, DECISIONS):
        directory.mkdir(parents=True, exist_ok=True)
    for name in (sys.argv[1:] or list(ITEMS)):
        function = ITEMS.get(name)
        if function is None:
            print(f"unknown item {name}")
            continue
        try:
            function()
        except Exception as error:                               # noqa: BLE001
            print(f"FAIL {name}: {type(error).__name__}: {error}")
            import traceback
            traceback.print_exc()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
