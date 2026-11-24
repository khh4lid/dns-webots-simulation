"""The five figures for the final research package, all from recorded data.

Inputs are the files a real run produced; a figure whose input is missing is
reported PENDING and skipped, never invented.

    config/scenario_research_demo.json      -> fig1 environment layout
    (schematic, no data needed)             -> fig2 system architecture
    results/two_frames_webots_*_lidar.json  -> fig3 lidar validation
    results/lidar_attitude.json             -> fig3 lidar validation
    results/experiments/traces.json         -> fig4 trajectories
    results/experiments/experiment_summary.csv -> fig5 results

Run::

    python scripts/make_package_figures.py --out evidence/package_figures
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                      # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Rectangle  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.scenario import Scenario                                    # noqa: E402

RESULTS = PROJECT_ROOT / "results"

# Validated categorical palette (dataviz references/palette.md); slots 1-3 pass
# the all-pairs CVD and normal-vision floors, so the three PPO checkpoints take
# them and the scripted baselines wear neutral ink.
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, BASELINE, SURFACE = "#e1e0d9", "#c3c2b7", "#fcfcfb"
GOOD, CRITICAL = "#0ca30c", "#d03b3b"

SERIES = {"ppo_seed7": BLUE, "ppo_seed19": ORANGE, "ppo_seed43": AQUA,
          "reactive": MUTED, "direct": INK2}

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
    print(f"  wrote {path}")


def miss(label: str, path: Path) -> None:
    pending.append(f"{label}: missing {path}")
    print(f"  PENDING {label}: no {path}")


# ----------------------------------------------------------------- figure 1
def figure_layout(scenario: Scenario, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(9.0, 6.2))
    low = np.asarray(scenario.low_xy, dtype=float)
    high = np.asarray(scenario.high_xy, dtype=float)
    size = high - low

    ax.add_patch(Rectangle(low, *size, facecolor="#f2f1ed",
                           edgecolor=BASELINE, linewidth=1.6, zorder=1))

    for obstacle in scenario.obstacles:
        moving = obstacle.motion.type != "static" and obstacle.motion.amplitude_m
        colour = ORANGE if moving else "#b9bcc2"
        if moving:
            axis = 0 if obstacle.motion.axis == "x" else 1
            lo = np.array(obstacle.xy, dtype=float)
            hi = lo.copy()
            lo[axis] -= obstacle.motion.amplitude_m
            hi[axis] += obstacle.motion.amplitude_m
            ax.plot([lo[0], hi[0]], [lo[1], hi[1]], color=ORANGE,
                    linewidth=1.4, linestyle=(0, (5, 3)), zorder=3)
            for end in (lo, hi):
                ax.add_patch(Circle(end, obstacle.radius_m, facecolor="none",
                                    edgecolor=ORANGE, linewidth=1.0,
                                    linestyle=(0, (2, 2)), alpha=0.75, zorder=3))
            # Anchor on the low end: the high end sits next to obs_b's
            # annotation and the two labels overlap there.
            ax.annotate(f"sweep +/-{obstacle.motion.amplitude_m:g} m "
                        f"at {obstacle.motion.speed_mps:g} m/s",
                        xy=(lo[0], lo[1]), xytext=(16, 10),
                        textcoords="offset points", color=ORANGE, fontsize=8,
                        fontweight="semibold")
        ax.add_patch(Circle(obstacle.xy, obstacle.radius_m, facecolor=colour,
                            edgecolor=MUTED, linewidth=0.9, zorder=4))
        ax.add_patch(Circle(obstacle.xy,
                            obstacle.radius_m + scenario.collision_radius_m,
                            facecolor="none", edgecolor=MUTED, linewidth=0.7,
                            linestyle=(0, (2, 2)), zorder=4))
        ax.annotate(f"{obstacle.id}\n({obstacle.xy[0]:g}, {obstacle.xy[1]:g}) m\n"
                    f"r = {obstacle.radius_m:g} m",
                    xy=obstacle.xy, xytext=(0, -obstacle.radius_m * 34 - 26),
                    textcoords="offset points", ha="center", fontsize=7.5,
                    color=INK2, zorder=6)

    ax.plot(*scenario.start_xy, marker="o", markersize=11, color=GOOD, zorder=7)
    ax.annotate(f"START\n({scenario.start_xy[0]:g}, {scenario.start_xy[1]:g}) m",
                xy=scenario.start_xy, xytext=(-4, 26),
                textcoords="offset points", ha="center", color=GOOD,
                fontsize=8.5, fontweight="semibold")
    ax.add_patch(Circle(scenario.goal_xy, scenario.goal_tolerance_m,
                        facecolor="none", edgecolor=CRITICAL, linewidth=1.4,
                        linestyle=(0, (4, 2)), zorder=6))
    ax.plot(*scenario.goal_xy, marker="*", markersize=16, color=CRITICAL,
            zorder=7)
    ax.annotate(f"GOAL\n({scenario.goal_xy[0]:g}, {scenario.goal_xy[1]:g}) m\n"
                f"tolerance {scenario.goal_tolerance_m:g} m",
                xy=scenario.goal_xy, xytext=(0, 30),
                textcoords="offset points", ha="center", color=CRITICAL,
                fontsize=8.5, fontweight="semibold")

    # dimension annotations
    y_dim = high[1] + 0.62
    ax.annotate("", xy=(low[0], y_dim), xytext=(high[0], y_dim),
                arrowprops=dict(arrowstyle="<->", color=INK2, linewidth=1.1))
    ax.text((low[0] + high[0]) / 2, y_dim + 0.1, f"{size[0]:g} m",
            ha="center", va="bottom", color=INK, fontsize=9,
            fontweight="semibold")
    x_dim = high[0] + 0.62
    ax.annotate("", xy=(x_dim, low[1]), xytext=(x_dim, high[1]),
                arrowprops=dict(arrowstyle="<->", color=INK2, linewidth=1.1))
    ax.text(x_dim + 0.12, 0, f"{size[1]:g} m", ha="left", va="center",
            color=INK, fontsize=9, fontweight="semibold", rotation=90)

    # coordinate axes
    ax.annotate("", xy=(1.4, -2.55), xytext=(0.35, -2.55),
                arrowprops=dict(arrowstyle="->", color=INK, linewidth=1.4))
    ax.text(1.5, -2.55, "+x forward", va="center", fontsize=8, color=INK)
    ax.annotate("", xy=(0.35, -1.5), xytext=(0.35, -2.55),
                arrowprops=dict(arrowstyle="->", color=INK, linewidth=1.4))
    ax.text(0.42, -1.45, "+y right", fontsize=8, color=INK)
    ax.plot(0.35, -2.55, marker="o", markersize=4, color=INK)
    ax.text(0.35, -2.9, "origin (0, 0)", fontsize=7.5, color=MUTED,
            ha="center")

    ax.set_xlim(low[0] - 1.0, high[0] + 1.3)
    ax.set_ylim(high[1] + 1.2, low[1] - 1.2)      # +y right -> downwards
    ax.set_aspect("equal")
    ax.set_xlabel("x, forward [m]")
    ax.set_ylabel("y, right [m]")
    ax.set_title(
        f"Figure 1 — DNS navigation arena, {size[0]:g} x {size[1]:g} m "
        f"(scenario: {scenario.name})\n"
        f"walls {scenario.wall_height_m:g} m · flight altitude "
        f"{scenario.cruise_altitude_m:g} m · drone collision radius "
        f"{scenario.collision_radius_m:g} m · dashed rings = inflated no-go region",
        loc="left", color=INK)
    save(fig, "fig1_environment_layout", out)


# ----------------------------------------------------------------- figure 2
def figure_architecture(out: Path) -> None:
    fig, ax = plt.subplots(figsize=(10.0, 5.4))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 54)
    ax.axis("off")

    def box(x, y, w, h, title, body, face, edge):
        ax.add_patch(FancyBboxPatch((x, y), w, h,
                                    boxstyle="round,pad=0.6,rounding_size=1.4",
                                    facecolor=face, edgecolor=edge,
                                    linewidth=1.3, zorder=3))
        ax.text(x + w / 2, y + h - 3.4, title, ha="center", va="top",
                fontsize=9, fontweight="semibold", color=INK, zorder=4)
        ax.text(x + w / 2, y + h / 2 - 2.4, body, ha="center", va="center",
                fontsize=7.4, color=INK2, zorder=4)

    def arrow(x1, y1, x2, y2, label="", colour=INK2, offset=(0, 1.6)):
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2),
                                     arrowstyle="-|>", mutation_scale=13,
                                     color=colour, linewidth=1.5, zorder=2,
                                     shrinkA=2, shrinkB=2))
        if label:
            ax.text((x1 + x2) / 2 + offset[0], (y1 + y2) / 2 + offset[1],
                    label, ha="center", fontsize=7.2, color=colour,
                    fontweight="semibold", zorder=5)

    box(2, 33, 26, 17, "Webots physics  (R2025a)",
        "rigid-body quadcopter, 0.55 kg\nrotor thrust via Propeller nodes\n"
        "8 ms timestep (125 Hz)", "#e8eef8", BLUE)
    box(36, 33, 26, 17, "Sensing",
        "360° LiDAR, 720 beams, 5 m cap\ngimbal-stabilised mount\n"
        "GPS velocity, IMU attitude", "#e8eef8", BLUE)
    box(70, 33, 28, 17, "Observation builder",
        "navigation.observation\n(Spencer, unmodified)\n"
        "12 sectors → 52 features", "#fdeee7", ORANGE)
    box(70, 6, 28, 17, "PPO policy",
        "52 - 32 - 32 - 2 tanh MLP\ntrained in planar.py\n"
        "action a in [-1, +1] per axis", "#fdeee7", ORANGE)
    box(36, 6, 26, 17, "Velocity command",
        "v = a × 1.2 m/s per axis\nheld for 0.2 s\n5 Hz outer loop",
        "#e9f7f1", AQUA)
    box(2, 6, 26, 17, "Flight controller",
        "cascaded autopilot at 125 Hz\nvelocity → tilt → torque\n"
        "→ four rotor thrusts", "#e9f7f1", AQUA)

    arrow(28, 41.5, 36, 41.5, "state")
    arrow(62, 41.5, 70, 41.5, "ranges, validity")
    arrow(84, 33, 84, 23, "52 features")
    arrow(70, 14.5, 62, 14.5, "a_x, a_y")
    arrow(36, 14.5, 28, 14.5, "v_x, v_y  [m/s]")
    arrow(15, 23, 15, 33, "rotor thrust")

    ax.text(50, 28.2, "measured state and sensing, never privileged ground truth",
            ha="center", fontsize=7.6, color=MUTED, style="italic")

    # side channel: supervisor ground truth used only for scoring
    ax.add_patch(FancyBboxPatch((2, 0.2), 96, 4.2,
                                boxstyle="round,pad=0.3,rounding_size=1.0",
                                facecolor="#f3f2ef", edgecolor=BASELINE,
                                linewidth=1.0, zorder=1))
    ax.text(50, 2.3, "Supervisor ground truth  →  collision test, clearance, "
                     "obstacle poses, episode logging   "
                     "(evaluation only — never entered into the observation)",
            ha="center", va="center", fontsize=7.6, color=INK2)

    ax.set_title("Figure 2 — DNS closed-loop architecture and the "
                 "simulator/policy boundary", loc="left", color=INK,
                 fontsize=10.5)
    save(fig, "fig2_system_architecture", out)


# ----------------------------------------------------------------- figure 3
def figure_lidar(out: Path) -> None:
    packet = None
    for candidate in sorted(RESULTS.glob("two_frames_webots_*_lidar.json")):
        packet = json.loads(candidate.read_text(encoding="utf8"))
        break
    attitude_path = RESULTS / "lidar_attitude.json"
    if packet is None or not attitude_path.is_file():
        miss("fig3 lidar validation", attitude_path)
        return
    attitude = json.loads(attitude_path.read_text(encoding="utf8"))

    fig = plt.figure(figsize=(11.0, 5.2))
    ax = fig.add_subplot(1, 2, 1, projection="polar")
    frame = packet["frames"][0]
    ranges = np.array(frame["ranges"], dtype=float)
    valid = np.array(frame["valid"], dtype=bool)
    cap = 5.0
    n = len(ranges)
    width = 2 * math.pi / n
    centres = np.arange(n) * width
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_facecolor(SURFACE)
    ax.bar(centres[valid], ranges[valid], width=width * 0.94, color=BLUE,
           alpha=0.8, edgecolor=SURFACE, linewidth=1.4, zorder=3)
    for i, (centre, r) in enumerate(zip(centres, ranges)):
        ax.text(centre, cap * 1.16, f"{i}\n{r:.2f}", ha="center", va="center",
                fontsize=6.8, color=INK2)
    for name, sector in (("FWD", 0), ("RIGHT", 3), ("BACK", 6), ("LEFT", 9)):
        ax.text(centres[sector], cap * 1.44, name, ha="center", va="center",
                fontsize=8, color=INK, fontweight="semibold")
    ax.set_ylim(0, cap)
    ax.set_yticks([1, 2, 3, 4, 5])
    ax.set_yticklabels(["1", "2", "3", "4", "5 m"], fontsize=6.5)
    ax.set_rlabel_position(255)
    ax.set_xticks(centres)
    ax.set_xticklabels([])
    ax.grid(color=GRID, linewidth=0.6)
    ax.set_title(f"(a) measured 12-sector scan · {packet.get('lidar_beams')} beams\n"
                 "0° forward, clockwise positive", loc="left",
                 color=INK, fontsize=9, pad=18)

    ax2 = fig.add_subplot(1, 2, 2)
    colours = {"body_fixed": ORANGE, "stabilised": BLUE}
    for mount in ("body_fixed", "stabilised"):
        rows = [s for s in attitude["steps"] if s["mount"] == mount]
        if not rows:
            continue
        tilt = np.array([r["tilt_deg"] for r in rows])
        rms = np.array([r["paired_rms_m"] for r in rows], dtype=float)
        order = np.argsort(tilt)
        ax2.plot(tilt[order], rms[order], color=colours[mount], linewidth=1.1,
                 alpha=0.5, zorder=2)
        ax2.scatter(tilt, rms, s=32, color=colours[mount], alpha=0.9,
                    edgecolor=SURFACE, linewidth=1.0, zorder=4, label=mount)
    ax2.set_xlabel("airframe tilt [deg]")
    ax2.set_ylabel("sector range error vs ground truth, RMS [m]")
    ax2.legend(loc="upper left")
    ax2.set_title("(b) mount validation: error vs attitude\n"
                  "identical commanded manoeuvres", loc="left", color=INK,
                  fontsize=9)
    fig.suptitle("Figure 3 — LiDAR configuration and sensor validation",
                 x=0.005, ha="left", color=INK, fontsize=10.5,
                 fontweight="semibold")
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    save(fig, "fig3_lidar_validation", out)


# ----------------------------------------------------------------- figure 4
def figure_trajectories(scenario: Scenario, out: Path) -> None:
    path = RESULTS / "experiments" / "traces.json"
    if not path.is_file():
        miss("fig4 trajectories", path)
        return
    traces = json.loads(path.read_text(encoding="utf8"))
    modes = [("B_static", "Mode B — static (crossing obstacle parked)"),
             ("C_dynamic", "Mode C — dynamic (obstacle crossing the route)")]
    show = ["ppo_seed7", "ppo_seed19", "ppo_seed43", "reactive"]

    fig, axes = plt.subplots(2, 1, figsize=(9.0, 8.4), sharex=True)
    low = np.asarray(scenario.low_xy, dtype=float)
    high = np.asarray(scenario.high_xy, dtype=float)

    for ax, (mode, title) in zip(axes, modes):
        ax.add_patch(Rectangle(low, *(high - low), facecolor="#f5f4f1",
                               edgecolor=BASELINE, linewidth=1.3, zorder=1))
        for obstacle in scenario.obstacles:
            moving = (obstacle.motion.type != "static"
                      and obstacle.motion.amplitude_m)
            if moving and mode == "C_dynamic":
                axis = 0 if obstacle.motion.axis == "x" else 1
                lo, hi = np.array(obstacle.xy), np.array(obstacle.xy)
                lo = lo.astype(float).copy(); hi = hi.astype(float).copy()
                lo[axis] -= obstacle.motion.amplitude_m
                hi[axis] += obstacle.motion.amplitude_m
                ax.plot([lo[0], hi[0]], [lo[1], hi[1]], color=ORANGE,
                        linewidth=1.3, linestyle=(0, (5, 3)), zorder=2)
                ax.add_patch(Circle(obstacle.xy, obstacle.radius_m,
                                    facecolor=ORANGE, alpha=0.25,
                                    edgecolor=ORANGE, linewidth=1.0, zorder=2))
            elif moving:
                parked = (5.1, -2.45)
                ax.add_patch(Circle(parked, obstacle.radius_m,
                                    facecolor="#dfe1e5", edgecolor=MUTED,
                                    linewidth=0.8, zorder=2))
            else:
                ax.add_patch(Circle(obstacle.xy, obstacle.radius_m,
                                    facecolor="#cfd2d7", edgecolor=MUTED,
                                    linewidth=0.8, zorder=2))
        ax.plot(*scenario.start_xy, marker="o", markersize=8, color=GOOD,
                zorder=8)
        ax.add_patch(Circle(scenario.goal_xy, scenario.goal_tolerance_m,
                            facecolor="none", edgecolor=CRITICAL,
                            linewidth=1.2, linestyle=(0, (4, 2)), zorder=6))
        ax.plot(*scenario.goal_xy, marker="*", markersize=13, color=CRITICAL,
                zorder=8)

        for name in show:
            trace = traces.get(f"{mode}/{name}")
            if not trace:
                continue
            xy = np.array([p["xy"] for p in trace])
            colour = SERIES.get(name, BLUE)
            ax.plot(xy[:, 0], xy[:, 1], color=colour, zorder=6,
                    linewidth=2.0 if name.startswith("ppo") else 1.3,
                    linestyle="-" if name.startswith("ppo") else (0, (4, 2)))
            reached = (float(np.hypot(*(xy[-1] - np.asarray(scenario.goal_xy))))
                       <= scenario.goal_tolerance_m)
            ax.plot(xy[-1, 0], xy[-1, 1],
                    marker="o" if reached else "X",
                    markersize=7 if reached else 11,
                    color=colour if reached else CRITICAL,
                    markeredgecolor=SURFACE, markeredgewidth=1.4, zorder=9)
            ax.annotate(name, xy=(xy[-1, 0], xy[-1, 1]), xytext=(7, 7),
                        textcoords="offset points", fontsize=7.5,
                        color=colour, fontweight="semibold", zorder=9)

        ax.set_xlim(low[0] - 0.4, high[0] + 0.9)
        ax.set_ylim(high[1] + 0.4, low[1] - 0.4)
        ax.set_aspect("equal")
        ax.set_ylabel("y, right [m]")
        ax.set_title(title, loc="left", color=INK, fontsize=9.5)
    axes[-1].set_xlabel("x, forward [m]")
    axes[0].text(0.005, 1.22, "Figure 4 — recorded drone trajectories "
                 "(episode 0 of each condition)\n"
                 "circle = goal reached, X = collision; dashed orange = "
                 "crossing obstacle sweep",
                 transform=axes[0].transAxes, fontsize=10.5, color=INK,
                 fontweight="semibold", va="top")
    save(fig, "fig4_trajectories", out)


# ----------------------------------------------------------------- figure 5
def figure_results(out: Path) -> None:
    path = RESULTS / "experiments" / "experiment_summary.csv"
    if not path.is_file():
        miss("fig5 results", path)
        return
    with path.open(encoding="utf8") as handle:
        rows = list(csv.DictReader(handle))

    order = ["ppo_seed7", "ppo_seed19", "ppo_seed43", "reactive", "direct"]
    modes = [("B_static", "Mode B — static"), ("C_dynamic", "Mode C — dynamic")]
    fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.6), sharey=True)

    for ax, (mode, title) in zip(axes, modes):
        data = {r["controller"]: r for r in rows if r["mode"] == mode}
        names = [n for n in order if n in data]
        values = [float(data[n]["success_rate"]) * 100 for n in names]
        lows = [float(data[n]["success_rate_ci95_low"]) * 100 for n in names]
        highs = [float(data[n]["success_rate_ci95_high"]) * 100 for n in names]
        episodes = [int(data[n]["episodes"]) for n in names]
        colours = [SERIES.get(n, BLUE) for n in names]
        x = np.arange(len(names))
        bars = ax.bar(x, values, 0.62, color=colours, edgecolor=SURFACE,
                      linewidth=1.5, zorder=3)
        ax.errorbar(x, values,
                    yerr=[np.array(values) - np.array(lows),
                          np.array(highs) - np.array(values)],
                    fmt="none", ecolor=INK2, elinewidth=1.2, capsize=5,
                    zorder=5)
        for bar, value, n in zip(bars, values, episodes):
            # A zero-height bar has no fill to sit on, so the sample-size label
            # has to switch to dark ink or it vanishes into the surface.
            inside = value > 12
            ax.text(bar.get_x() + bar.get_width() / 2, 3 if inside else -7,
                    f"n={n}", ha="center", fontsize=7.5,
                    color=SURFACE if inside else INK2,
                    fontweight="semibold", zorder=6)
            ax.text(bar.get_x() + bar.get_width() / 2, value + 7,
                    f"{value:.0f}%", ha="center", fontsize=8.5, color=INK,
                    fontweight="semibold", zorder=6)
        ax.set_xticks(x)
        ax.set_xticklabels(names, rotation=18, ha="right", fontsize=8)
        ax.set_ylim(-12, 118)
        ax.grid(axis="x", visible=False)
        ax.set_title(title, loc="left", color=INK, fontsize=9.5)
    axes[0].set_ylabel("goal-reaching rate [%]")
    fig.suptitle("Figure 5 — navigation success by controller and condition\n"
                 "bars are successes / episodes in that cell; whiskers are "
                 "Wilson 95% intervals. Mode B repeats one fixed layout "
                 "(determinism check); Mode C varies the crossing phase.",
                 x=0.005, ha="left", color=INK, fontsize=10,
                 fontweight="semibold")
    fig.tight_layout(rect=(0, 0, 1, 0.84))
    save(fig, "fig5_results", out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=PROJECT_ROOT / "config" / "scenario_research_demo.json")
    parser.add_argument("--out", type=Path,
                        default=PROJECT_ROOT / "evidence" / "package_figures")
    args = parser.parse_args()

    scenario = Scenario.load(args.config)
    print(f"scenario: {scenario.name}")
    figure_layout(scenario, args.out)
    figure_architecture(args.out)
    figure_lidar(args.out)
    figure_trajectories(scenario, args.out)
    figure_results(args.out)

    print(f"\n{len(written)} figures written")
    for item in pending:
        print(f"  PENDING {item}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
