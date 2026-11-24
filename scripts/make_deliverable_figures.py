"""The five figures for DNS_SIMULATION_RESEARCH_DELIVERABLE.

Every panel is drawn from a recorded file or from the validated scenario
configuration. A figure whose input is missing is reported PENDING and skipped
rather than approximated.

    results/screenshots/*.png                  -> fig1 environment overview
    config/scenario_warehouse.json             -> fig2 technical map
    (schematic)                                -> fig3 architecture
    results/moving_obstacle_warehouse.json     -> fig4 moving-obstacle validation
    results/lidar_attitude_warehouse.json      -> fig5 sensor validation
    results/calibration_warehouse.json         -> fig5 sensor validation

Run::

    python scripts/make_deliverable_figures.py
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                      # noqa: E402
from matplotlib.image import imread                                  # noqa: E402
from matplotlib.patches import (                                     # noqa: E402
    Circle, FancyArrowPatch, FancyBboxPatch, Rectangle)
from matplotlib.transforms import Affine2D                           # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.scenario import Scenario                                    # noqa: E402

RESULTS = PROJECT_ROOT / "results"

BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, BASELINE, SURFACE = "#e1e0d9", "#c3c2b7", "#fcfcfb"
GOOD, CRITICAL = "#0ca30c", "#d03b3b"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE, "font.family": "sans-serif",
    "font.sans-serif": ["Segoe UI", "DejaVu Sans", "sans-serif"],
    "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "semibold",
    "axes.labelsize": 9, "axes.labelcolor": INK2, "axes.edgecolor": BASELINE,
    "axes.linewidth": 0.8, "axes.grid": True, "grid.color": GRID,
    "grid.linewidth": 0.6, "xtick.color": MUTED, "ytick.color": MUTED,
    "xtick.labelsize": 8, "ytick.labelsize": 8, "legend.frameon": False,
    "legend.fontsize": 8, "lines.linewidth": 2.0,
})

written: list[str] = []
pending: list[str] = []


def save(fig, stem: str, out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{stem}.png"
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    written.append(stem)
    print(f"  wrote {path.name}")


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf8")) if path.is_file() else None


def draw_obstacle(ax, obstacle, centre=None, *, face, edge, alpha=1.0,
                  hatch=None, zorder=4, lw=0.9):
    """Draw an obstacle's true footprint: a rectangle for a box, else a disc."""
    cx, cy = centre if centre is not None else obstacle.xy
    if obstacle.shape == "box":
        w, h = obstacle.size_xy
        patch = Rectangle((cx - w / 2, cy - h / 2), w, h, facecolor=face,
                          edgecolor=edge, linewidth=lw, alpha=alpha,
                          hatch=hatch, zorder=zorder)
        if abs(obstacle.yaw_rad) > 1e-9:
            patch.set_transform(
                Affine2D().rotate_around(cx, cy, obstacle.yaw_rad) + ax.transData)
        ax.add_patch(patch)
    else:
        ax.add_patch(Circle((cx, cy), obstacle.radius_m, facecolor=face,
                            edgecolor=edge, linewidth=lw, alpha=alpha,
                            hatch=hatch, zorder=zorder))


# ----------------------------------------------------------------- figure 1
def figure_overview(out: Path) -> None:
    shots = RESULTS / "screenshots"
    wanted = [("overview_3d", "(a) 3-D perspective"),
              ("topdown", "(b) top-down")]
    have = [(n, t) for n, t in wanted if (shots / f"{n}.png").is_file()]
    if not have:
        pending.append("fig1: no screenshots in results/screenshots")
        print("  PENDING fig1: no screenshots")
        return
    fig, axes = plt.subplots(len(have), 1, figsize=(8.6, 4.9 * len(have)))
    axes = np.atleast_1d(axes)
    for ax, (name, title) in zip(axes, have):
        ax.imshow(imread(shots / f"{name}.png"))
        ax.set_xticks([])
        ax.set_yticks([])
        ax.grid(False)
        for spine in ax.spines.values():
            spine.set_edgecolor(BASELINE)
        ax.set_title(title, loc="left", color=INK, fontsize=9.5)
    fig.suptitle(
        "Figure 1 — the DNS warehouse environment, rendered by Webots R2025a\n"
        "Green pad = start, blue pad = goal (drawn at the 0.4 m success "
        "tolerance), dark grey = storage racks, light grey = structural\n"
        "columns, brown = crate stack, orange = transfer cart, floor grid = "
        "1 m. In (a) the goal pad is hidden behind the crate stack;\n"
        "(b) shows the full layout. Frames come from the drone's Camera "
        "sensor, not the GUI viewport (see section 8).",
        x=0.005, ha="left", color=INK, fontsize=9.5, fontweight="semibold")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    save(fig, "fig1_environment_overview", out)


# ----------------------------------------------------------------- figure 2
def figure_map(scenario: Scenario, out: Path) -> None:
    # Specs live in a side panel keyed by number. Printing them on the map
    # itself made the callouts collide with each other and with the
    # footprints they describe, which is worse than an extra lookup.
    fig, (ax, panel) = plt.subplots(
        1, 2, figsize=(13.2, 6.4), gridspec_kw={"width_ratios": [2.45, 1.0]})
    panel.axis("off")
    panel.grid(False)
    low = np.asarray(scenario.low_xy, dtype=float)
    high = np.asarray(scenario.high_xy, dtype=float)
    size = high - low
    radius = scenario.collision_radius_m

    ax.add_patch(Rectangle(low, *size, facecolor="#f2f1ed", edgecolor=BASELINE,
                           linewidth=1.6, zorder=1))

    rows = []
    for number, obstacle in enumerate(scenario.obstacles, start=1):
        moving = obstacle.motion.type != "static" and obstacle.motion.amplitude_m
        colour = ORANGE if moving else "#aeb2b8"
        if moving:
            axis = 0 if obstacle.motion.axis == "x" else 1
            lo = np.array(obstacle.xy, dtype=float)
            hi = lo.copy()
            lo[axis] -= obstacle.motion.amplitude_m
            hi[axis] += obstacle.motion.amplitude_m
            ax.plot([lo[0], hi[0]], [lo[1], hi[1]], color=ORANGE,
                    linewidth=1.5, linestyle=(0, (5, 3)), zorder=3)
            for end in (lo, hi):
                draw_obstacle(ax, obstacle, end, face="none", edge=ORANGE,
                              alpha=0.85, zorder=3, lw=0.9)
            ax.annotate(
                f"sweep +/-{obstacle.motion.amplitude_m:g} m at "
                f"{obstacle.motion.speed_mps:g} m/s\n"
                f"period {4 * obstacle.motion.amplitude_m / obstacle.motion.speed_mps:.1f} s",
                xy=(lo[0], lo[1]), xytext=(14, -26), textcoords="offset points",
                color=ORANGE, fontsize=7.8, fontweight="semibold")
        # inflated no-go footprint
        draw_obstacle(ax, obstacle, face="none", edge=MUTED, alpha=0.9,
                      zorder=3, lw=0.6)
        draw_obstacle(ax, obstacle, face=colour, edge=MUTED, zorder=4)
        size_text = (f"{obstacle.size_xy[0]:g} x {obstacle.size_xy[1]:g} m"
                     if obstacle.shape == "box" else f"r = {obstacle.radius_m:g} m")
        ax.annotate(str(number), xy=obstacle.xy, ha="center", va="center",
                    fontsize=8.2, color=INK, fontweight="semibold", zorder=7,
                    bbox=dict(boxstyle="circle,pad=0.26", facecolor=SURFACE,
                              edgecolor=INK2, alpha=0.95, linewidth=0.8))
        motion_text = (f"{obstacle.motion.type}, +/-{obstacle.motion.amplitude_m:g} m "
                       f"along {obstacle.motion.axis} at "
                       f"{obstacle.motion.speed_mps:g} m/s"
                       if moving else "static")
        rows.append((number, obstacle, size_text, motion_text))

    ax.plot(*scenario.start_xy, marker="o", markersize=11, color=GOOD, zorder=8)
    ax.annotate(f"START ({scenario.start_xy[0]:g}, {scenario.start_xy[1]:g}) m",
                xy=scenario.start_xy, xytext=(0, 24), textcoords="offset points",
                ha="center", color=GOOD, fontsize=8.6, fontweight="semibold")
    ax.add_patch(Circle(scenario.goal_xy, scenario.goal_tolerance_m,
                        facecolor="#dceaff", edgecolor=BLUE, linewidth=1.5,
                        linestyle=(0, (4, 2)), zorder=6))
    ax.plot(*scenario.goal_xy, marker="*", markersize=16, color=BLUE, zorder=8)
    ax.annotate(f"GOAL ({scenario.goal_xy[0]:g}, {scenario.goal_xy[1]:g}) m\n"
                f"tolerance {scenario.goal_tolerance_m:g} m",
                xy=scenario.goal_xy, xytext=(0, 26), textcoords="offset points",
                ha="center", color=BLUE, fontsize=8.6, fontweight="semibold")

    y_dim = high[1] + 0.75
    ax.annotate("", xy=(low[0], y_dim), xytext=(high[0], y_dim),
                arrowprops=dict(arrowstyle="<->", color=INK2, linewidth=1.1))
    ax.text((low[0] + high[0]) / 2, y_dim + 0.12, f"{size[0]:g} m",
            ha="center", va="bottom", color=INK, fontsize=9.5,
            fontweight="semibold")
    x_dim = high[0] + 0.75
    ax.annotate("", xy=(x_dim, low[1]), xytext=(x_dim, high[1]),
                arrowprops=dict(arrowstyle="<->", color=INK2, linewidth=1.1))
    ax.text(x_dim + 0.14, 0, f"{size[1]:g} m", ha="left", va="center",
            color=INK, fontsize=9.5, fontweight="semibold", rotation=90)

    ax.annotate("", xy=(1.55, -2.62), xytext=(0.3, -2.62),
                arrowprops=dict(arrowstyle="->", color=INK, linewidth=1.5))
    ax.text(1.65, -2.62, "+x forward", va="center", fontsize=8.2, color=INK)
    ax.annotate("", xy=(0.3, -1.45), xytext=(0.3, -2.62),
                arrowprops=dict(arrowstyle="->", color=INK, linewidth=1.5))
    ax.text(0.4, -1.38, "+y right", fontsize=8.2, color=INK)
    ax.plot(0.3, -2.62, marker="o", markersize=4.5, color=INK)
    ax.text(0.3, -2.95, "origin (0, 0)", fontsize=7.6, color=MUTED, ha="center")

    ax.set_xlim(low[0] - 1.1, high[0] + 1.5)
    ax.set_ylim(high[1] + 1.35, low[1] - 1.35)
    ax.set_aspect("equal")
    ax.set_xlabel("x, forward [m]")
    ax.set_ylabel("y, right [m]")
    ax.set_title(
        f"Figure 2 — scaled technical map, scenario {scenario.name} "
        f"({size[0]:g} x {size[1]:g} m)\n"
        f"walls {scenario.wall_height_m:g} m · altitude "
        f"{scenario.cruise_altitude_m:g} m · drone collision radius "
        f"{radius:g} m · thin outlines are the true footprints,\n"
        f"drawn from the same configuration file the simulator loads",
        loc="left", color=INK, fontsize=9.5)

    # ---- side panel
    y = 0.985
    panel.text(0.0, y, "Obstacles", fontsize=10, color=INK,
               fontweight="semibold", va="top", transform=panel.transAxes)
    y -= 0.055
    for number, obstacle, size_text, motion_text in rows:
        panel.text(0.0, y, f"{number}", fontsize=8.4, color=INK,
                   fontweight="semibold", va="top", transform=panel.transAxes)
        panel.text(0.055, y, obstacle.label or obstacle.id, fontsize=8.4,
                   color=INK, fontweight="semibold", va="top",
                   transform=panel.transAxes)
        y -= 0.036
        panel.text(0.055, y,
                   f"id {obstacle.id} · {obstacle.shape} · {size_text} · "
                   f"h {obstacle.height_m:g} m",
                   fontsize=7.4, color=INK2, va="top",
                   transform=panel.transAxes)
        y -= 0.032
        panel.text(0.055, y,
                   f"centre ({obstacle.xy[0]:g}, {obstacle.xy[1]:g}) m · "
                   f"{motion_text}", fontsize=7.4, color=INK2, va="top",
                   transform=panel.transAxes)
        y -= 0.032
        if obstacle.purpose:
            wrapped = obstacle.purpose
            if len(wrapped) > 150:
                wrapped = wrapped[:147].rsplit(" ", 1)[0] + "..."
            import textwrap as _tw
            for line in _tw.wrap(wrapped, 54):
                panel.text(0.055, y, line, fontsize=6.9, color=MUTED,
                           va="top", transform=panel.transAxes, style="italic")
                y -= 0.027
        y -= 0.016

    panel.text(0.0, y, "Start / goal", fontsize=9.4, color=INK,
               fontweight="semibold", va="top", transform=panel.transAxes)
    y -= 0.042
    for label, point, extra in (
            ("start", scenario.start_xy, f"altitude {scenario.cruise_altitude_m:g} m"),
            ("goal", scenario.goal_xy,
             f"success within {scenario.goal_tolerance_m:g} m")):
        panel.text(0.055, y, f"{label}: ({point[0]:g}, {point[1]:g}) m · {extra}",
                   fontsize=7.6, color=INK2, va="top",
                   transform=panel.transAxes)
        y -= 0.033

    save(fig, "fig2_technical_map", out)


# ----------------------------------------------------------------- figure 3
def figure_architecture(out: Path) -> None:
    fig, ax = plt.subplots(figsize=(10.4, 5.8))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 58)
    ax.axis("off")
    ax.grid(False)

    def box(x, y, w, h, title, body, face, edge, owner):
        ax.add_patch(FancyBboxPatch((x, y), w, h,
                                    boxstyle="round,pad=0.6,rounding_size=1.4",
                                    facecolor=face, edgecolor=edge,
                                    linewidth=1.3, zorder=3))
        ax.text(x + w / 2, y + h - 3.0, title, ha="center", va="top",
                fontsize=9, fontweight="semibold", color=INK, zorder=4)
        ax.text(x + w / 2, y + h / 2 - 2.6, body, ha="center", va="center",
                fontsize=7.2, color=INK2, zorder=4)
        ax.text(x + w / 2, y + 1.7, owner, ha="center", va="center",
                fontsize=6.8, color=edge, fontweight="semibold", zorder=4)

    def arrow(x1, y1, x2, y2, label="", dx=0.0, dy=1.7):
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>",
                                     mutation_scale=13, color=INK2,
                                     linewidth=1.5, zorder=2, shrinkA=2,
                                     shrinkB=2))
        if label:
            ax.text((x1 + x2) / 2 + dx, (y1 + y2) / 2 + dy, label,
                    ha="center", fontsize=7.1, color=INK2,
                    fontweight="semibold", zorder=5)

    box(2, 36, 27, 18, "Webots environment", "warehouse world, racks,\n"
        "columns, crate, moving cart\n8 ms physics step",
        "#e8eef8", BLUE, "SIMULATION (this work)")
    box(36.5, 36, 27, 18, "Sensor interface", "360 deg LiDAR, 720 beams\n"
        "gimballed mount, 5 m cap\nGPS velocity, IMU attitude",
        "#e8eef8", BLUE, "SIMULATION (this work)")
    box(71, 36, 27, 18, "Sensor processing", "beams -> 12 sectors\n"
        "navigation.pack_scan\nranges + validity",
        "#e9f7f1", AQUA, "SENSOR TEAMMATE")
    box(71, 7, 27, 18, "PPO policy", "52-32-32-2 tanh MLP\n"
        "observation builder\naction a in [-1, +1]",
        "#fdeee7", ORANGE, "SPENCER (DRL)")
    box(36.5, 7, 27, 18, "Velocity interface", "v = a x 1.2 m/s per axis\n"
        "held 0.2 s, 5 Hz\nreset / step contract",
        "#e8eef8", BLUE, "SIMULATION (this work)")
    box(2, 7, 27, 18, "Drone dynamics", "0.55 kg quadcopter\n"
        "cascaded autopilot 125 Hz\nrotor thrust -> motion",
        "#e8eef8", BLUE, "SIMULATION (this work)")

    arrow(29, 45, 36.5, 45, "measured state")
    arrow(63.5, 45, 71, 45, "raw beams [m]")
    arrow(84.5, 36, 84.5, 25, "12 sectors")
    arrow(71, 16, 63.5, 16, "a_x, a_y")
    arrow(36.5, 16, 29, 16, "v_x, v_y [m/s]")
    arrow(15.5, 25, 15.5, 36, "rotor thrust")

    ax.add_patch(FancyBboxPatch((2, 0.3), 96, 4.6,
                                boxstyle="round,pad=0.3,rounding_size=1.0",
                                facecolor="#f3f2ef", edgecolor=BASELINE,
                                linewidth=1.0, zorder=1))
    ax.text(50, 2.6, "Ground-truth monitor and experiment logger  —  exact "
                     "obstacle footprints, swept collision test, clearance, "
                     "episode outcome, CSV/JSON records   "
                     "(evaluation only; never enters the observation)",
            ha="center", va="center", fontsize=7.4, color=INK2)

    ax.set_title("Figure 3 — DNS closed-loop architecture, with the ownership "
                 "boundary between the simulation environment and the\n"
                 "teammates' components marked on each block",
                 loc="left", color=INK, fontsize=10.5)
    save(fig, "fig3_architecture", out)


# ----------------------------------------------------------------- figure 4
def figure_moving_obstacle(out: Path) -> None:
    data = load_json(RESULTS / "moving_obstacle_warehouse.json")
    if not data:
        pending.append("fig4: no results/moving_obstacle_warehouse.json")
        print("  PENDING fig4: no moving-obstacle record")
        return
    steps = data["steps"]
    motion = data["obstacle"]["motion"]
    t = np.array([r["t"] for r in steps])
    commanded = np.array([r["expected_xy"][1] for r in steps])
    measured = np.array([r["actual_xy"][1] for r in steps])
    error = np.array([r["error_m"] for r in steps])

    fig, axes = plt.subplots(3, 1, figsize=(8.4, 7.4), sharex=True,
                             gridspec_kw={"height_ratios": [2.0, 1.3, 1.3]})

    ax = axes[0]
    ax.plot(t, commanded, color=BLUE, linewidth=3.2, alpha=0.45,
            label="commanded by the trajectory model")
    ax.plot(t, measured, color=ORANGE, linewidth=1.5,
            label="measured in the simulator (supervisor)")
    ax.axhline(motion["amplitude_m"], color=MUTED, linewidth=0.9,
               linestyle=(0, (3, 3)))
    ax.axhline(-motion["amplitude_m"], color=MUTED, linewidth=0.9,
               linestyle=(0, (3, 3)))
    ax.text(t[-1], motion["amplitude_m"], " declared envelope", fontsize=7.4,
            color=MUTED, va="bottom", ha="right")
    ax.set_ylabel("cart position, y [m]")
    ax.legend(loc="lower right", ncol=1)
    ax.set_title("(a) commanded vs measured trajectory — the two curves are "
                 "indistinguishable", loc="left", color=INK, fontsize=9.5)

    ax = axes[1]
    speed = np.abs(np.diff(measured)) / np.diff(t)
    ax.plot(t[1:], speed, color=AQUA, linewidth=1.4)
    ax.axhline(motion["speed_mps"], color=INK2, linewidth=1.0,
               linestyle=(0, (4, 2)))
    ax.text(t[-1], motion["speed_mps"], f" configured {motion['speed_mps']:g} m/s ",
            fontsize=7.4, color=INK2, va="bottom", ha="right")
    ax.set_ylabel("|dy/dt| [m/s]")
    ax.set_ylim(0, motion["speed_mps"] * 1.6)
    ax.set_title("(b) measured travel speed — the dips are the two turning "
                 "points, where the triangle wave reverses", loc="left",
                 color=INK, fontsize=9.5)

    ax = axes[2]
    ax.plot(t, error * 1000.0, color=CRITICAL, linewidth=1.5)
    ax.set_ylabel("tracking error [mm]")
    ax.set_xlabel("simulation time since reset [s]")
    peak = float(error.max()) * 1000.0
    ax.set_ylim(-0.05, max(0.12, peak * 1.6))
    ax.set_title(f"(c) |measured - commanded|, peak {peak:.4f} mm: the "
                 "supervisor reproduces the analytic path exactly",
                 loc="left", color=INK, fontsize=9.5)

    fig.suptitle("Figure 4 — validation of the scripted moving obstacle "
                 "against its own trajectory model\n"
                 f"{data['obstacle'].get('label') or data['obstacle']['id']}, "
                 f"{motion['type']} motion along {motion['axis']}, "
                 f"amplitude +/-{motion['amplitude_m']:g} m, period "
                 f"{motion['period_s']:.2f} s; "
                 f"{len(steps)} control steps",
                 x=0.005, ha="left", color=INK, fontsize=10,
                 fontweight="semibold")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    save(fig, "fig4_moving_obstacle_validation", out)


# ----------------------------------------------------------------- figure 5
def figure_sensor(out: Path) -> None:
    attitude = load_json(RESULTS / "lidar_attitude_warehouse.json")
    mover = load_json(RESULTS / "moving_obstacle_warehouse.json")
    if not attitude or not mover:
        pending.append("fig5: missing lidar_attitude_warehouse/moving_obstacle")
        print("  PENDING fig5")
        return

    fig, axes = plt.subplots(1, 2, figsize=(10.6, 4.4))

    ax = axes[0]
    colours = {"body_fixed": ORANGE, "stabilised": BLUE}
    for mount in ("body_fixed", "stabilised"):
        rows = [s for s in attitude["steps"] if s["mount"] == mount]
        if not rows:
            continue
        tilt = np.array([r["tilt_deg"] for r in rows])
        rms = np.array([r["paired_rms_m"] for r in rows], dtype=float)
        order = np.argsort(tilt)
        ax.plot(tilt[order], rms[order], color=colours[mount], linewidth=1.0,
                alpha=0.45, zorder=2)
        ax.scatter(tilt, rms, s=32, color=colours[mount], alpha=0.9,
                   edgecolor=SURFACE, linewidth=1.0, zorder=4,
                   label=f"{mount} mount")
    ax.set_xlabel("airframe tilt [deg]")
    ax.set_ylabel("sector range error vs analytic truth, RMS [m]")
    ax.legend(loc="upper left")
    ax.set_title("(a) attitude ablation: one factor, the lidar mount",
                 loc="left", color=INK, fontsize=9.5)

    ax = axes[1]
    errors = np.array([r["max_sector_error_m"] for r in mover["steps"]])
    ax.hist(errors, bins=24, color=AQUA, edgecolor=SURFACE, linewidth=0.8,
            zorder=3)
    ax.axvline(float(errors.mean()), color=INK2, linewidth=1.3,
               linestyle=(0, (4, 2)), zorder=4)
    ax.text(float(errors.mean()), ax.get_ylim()[1] * 0.92,
            f" mean {errors.mean():.3f} m", fontsize=8, color=INK2, va="top")
    ax.set_xlabel("worst per-sector |measured - analytic| in a step [m]")
    ax.set_ylabel("control steps")
    ax.set_title("(b) sensor vs independent analytic ray-cast,\n"
                 "all 12 sectors, moving scene, occlusion included",
                 loc="left", color=INK, fontsize=9.5)

    fig.suptitle(
        "Figure 5 — LiDAR validation. (a) holding the scan plane level is what "
        "keeps range error flat as the airframe tilts;\n"
        "(b) across a dynamic episode the measured scan matches a wholly "
        "independent analytic ray-cast of the same scene.",
        x=0.005, ha="left", color=INK, fontsize=9.5, fontweight="semibold")
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    save(fig, "fig5_sensor_validation", out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=PROJECT_ROOT / "config" / "scenario_warehouse.json")
    parser.add_argument("--out", type=Path,
                        default=PROJECT_ROOT / "evidence" / "deliverable_figures")
    args = parser.parse_args()

    scenario = Scenario.load(args.config)
    print(f"scenario: {scenario.name}")
    figure_overview(args.out)
    figure_map(scenario, args.out)
    figure_architecture(args.out)
    figure_moving_obstacle(args.out)
    figure_sensor(args.out)
    print(f"\n{len(written)} figures written to {args.out}")
    for item in pending:
        print(f"  PENDING {item}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
