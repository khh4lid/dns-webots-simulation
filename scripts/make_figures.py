"""Build the research figures from recorded simulator output.

Every figure here is drawn from a results file produced by a real Webots run.
Nothing is synthesised, and a figure whose input file is missing is reported as
PENDING and skipped rather than invented.

Run::

    cd webots_dns
    python scripts/make_figures.py

Inputs -> outputs (all under evidence/figures/):

    results/policy_rollout/traces.json   -> fig04_trajectories.(png|svg)
                                            fig05_velocity_tracking.(png|svg)
                                            fig06_clearance.(png|svg)
    results/two_frames_webots_*_lidar.json -> fig03_lidar_sectors.(png|svg)
    results/lidar_attitude.json          -> fig11_mount_comparison.(png|svg)
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                     # noqa: E402
from matplotlib.patches import Circle, Rectangle                    # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.scenario import DEFAULT_SCENARIO_PATH, Scenario             # noqa: E402

RESULTS = PROJECT_ROOT / "results"
FIGURES = PROJECT_ROOT / "evidence" / "figures"

# ---------------------------------------------------------------- house style
# Validated categorical palette (see dataviz references/palette.md). Slots 1-3
# clear the all-pairs CVD and normal-vision floors in both modes, so the three
# PPO checkpoints take those; the scripted baselines are reference lines and
# wear neutral ink so they cannot be mistaken for a subject series.
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, BASELINE, SURFACE = "#e1e0d9", "#c3c2b7", "#fcfcfb"
GOOD, CRITICAL, WARNING = "#0ca30c", "#d03b3b", "#fab219"

SERIES = {
    "ppo_seed7": BLUE,
    "ppo_seed19": ORANGE,
    "ppo_seed43": AQUA,
    "reactive": MUTED,
    "direct": INK2,
}

plt.rcParams.update({
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "font.family": "sans-serif",
    "font.sans-serif": ["Segoe UI", "DejaVu Sans", "sans-serif"],
    "font.size": 9,
    "axes.titlesize": 10,
    "axes.titleweight": "semibold",
    "axes.labelsize": 9,
    "axes.labelcolor": INK2,
    "axes.edgecolor": BASELINE,
    "axes.linewidth": 0.8,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.6,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "legend.frameon": False,
    "legend.fontsize": 8,
    "lines.linewidth": 2.0,
})

written: list[str] = []
pending: list[str] = []


def save(fig, stem: str) -> None:
    FIGURES.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "svg"):
        path = FIGURES / f"{stem}.{ext}"
        fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    written.append(stem)
    print(f"  wrote {FIGURES / (stem + '.png')} (+ .svg)")


def load(path: Path, label: str):
    if not path.is_file():
        pending.append(f"{label}: missing {path.relative_to(PROJECT_ROOT)}")
        print(f"  PENDING {label}: no {path.relative_to(PROJECT_ROOT)}")
        return None
    return json.loads(path.read_text(encoding="utf8"))


def draw_arena(ax, scenario: Scenario, *, legend_labels: bool = False) -> None:
    """Walls, obstacles, start and goal, in DNS coordinates."""
    low, high = np.asarray(scenario.low_xy), np.asarray(scenario.high_xy)
    ax.add_patch(Rectangle(low, *(high - low), facecolor="none",
                           edgecolor=BASELINE, linewidth=1.4, zorder=1))
    for obstacle in scenario.obstacles:
        ax.add_patch(Circle(obstacle.xy, obstacle.radius_m, facecolor="#dcdbd4",
                            edgecolor=MUTED, linewidth=0.8, zorder=2))
        # inflated by the drone radius: the real no-go region
        ax.add_patch(Circle(obstacle.xy,
                            obstacle.radius_m + scenario.collision_radius_m,
                            facecolor="none", edgecolor=MUTED, linewidth=0.6,
                            linestyle=(0, (2, 2)), zorder=2))
    ax.plot(*scenario.start_xy, marker="o", markersize=7, color=GOOD, zorder=5,
            label="start" if legend_labels else None)
    ax.add_patch(Circle(scenario.goal_xy, scenario.goal_tolerance_m,
                        facecolor="none", edgecolor=GOOD, linewidth=1.2,
                        linestyle=(0, (4, 2)), zorder=4))
    ax.plot(*scenario.goal_xy, marker="*", markersize=13, color=GOOD, zorder=5,
            label="goal" if legend_labels else None)
    ax.set_xlim(low[0] - 0.4, high[0] + 0.4)
    ax.set_ylim(high[1] + 0.4, low[1] - 0.4)        # +y is right -> y down
    ax.set_aspect("equal")


# --------------------------------------------------------------- figure 4
def figure_trajectories(scenario: Scenario) -> None:
    traces = load(RESULTS / "policy_rollout" / "traces.json", "fig04 trajectories")
    if not traces:
        return
    order = [k for k in SERIES if k in traces] + [k for k in traces if k not in SERIES]
    fig, axes = plt.subplots(len(order), 1, figsize=(7.6, 2.0 * len(order)),
                             sharex=True)
    axes = np.atleast_1d(axes)
    for ax, name in zip(axes, order):
        episode = traces[name][0]                   # first episode of each
        xy = np.array([p["xy"] for p in episode])
        colour = SERIES.get(name, BLUE)
        draw_arena(ax, scenario)
        ax.plot(xy[:, 0], xy[:, 1], color=colour, zorder=6, solid_capstyle="round")
        ax.plot(xy[-1, 0], xy[-1, 1], marker="o", markersize=6,
                markerfacecolor=colour, markeredgecolor=SURFACE,
                markeredgewidth=1.5, zorder=7)
        reached = (np.hypot(*(xy[-1] - np.asarray(scenario.goal_xy)))
                   <= scenario.goal_tolerance_m)
        verdict = "reached goal" if reached else "terminated early (collision)"
        ax.text(0.012, 0.9, name, transform=ax.transAxes, color=colour,
                fontsize=10, fontweight="semibold", va="top")
        ax.text(0.012, 0.1, f"{len(xy) - 1} steps · {verdict}",
                transform=ax.transAxes, color=INK2, fontsize=8, va="bottom")
        if not reached:
            ax.plot(xy[-1, 0], xy[-1, 1], marker="x", markersize=11,
                    color=CRITICAL, markeredgewidth=2.2, zorder=8)
        ax.set_ylabel("y (right) [m]")
    axes[-1].set_xlabel("x (forward) [m]")
    axes[0].set_title("Recorded drone trajectories, Webots physical flight with "
                      "real lidar\nsingle fixed 10 x 6 m layout, one episode each",
                      loc="left", color=INK)
    save(fig, "fig04_trajectories")


# --------------------------------------------------------------- figure 5
def figure_velocity_tracking() -> None:
    traces = load(RESULTS / "policy_rollout" / "traces.json",
                  "fig05 velocity tracking")
    if not traces:
        return
    name = "ppo_seed19" if "ppo_seed19" in traces else sorted(traces)[0]
    episode = traces[name][0]
    t = np.array([p["t"] for p in episode])
    measured = np.array([p["v"] for p in episode])
    commanded = np.array([p["cmd"] for p in episode])

    fig, axes = plt.subplots(2, 1, figsize=(7.6, 4.6), sharex=True)
    for ax, axis, label in zip(axes, (0, 1), ("vx (forward)", "vy (right)")):
        # The command for the interval starting at t is held until the next
        # sample, so a step plot is the honest rendering of a 5 Hz setpoint.
        ax.step(t, commanded[:, axis], where="post", color=BLUE,
                label="commanded (PPO action x 1.2)")
        ax.plot(t, measured[:, axis], color=ORANGE, label="measured (GPS)")
        ax.axhline(0.0, color=BASELINE, linewidth=0.8, zorder=0)
        ax.set_ylabel(f"{label} [m/s]")
    axes[0].legend(loc="upper right", ncol=2)
    error = measured - commanded
    rmse = np.sqrt((error ** 2).mean(axis=0))
    axes[-1].set_xlabel("simulation time since reset [s]")
    axes[0].set_title(
        f"Velocity tracking, {name}: commanded setpoint vs measured response\n"
        f"per-axis RMSE {rmse[0]:.3f} / {rmse[1]:.3f} m/s "
        f"(5 Hz command, 125 Hz inner loop)", loc="left", color=INK)
    save(fig, "fig05_velocity_tracking")


# --------------------------------------------------------------- figure 6
def figure_clearance(scenario: Scenario) -> None:
    traces = load(RESULTS / "policy_rollout" / "traces.json", "fig06 clearance")
    if not traces:
        return
    fig, ax = plt.subplots(figsize=(7.6, 3.8))
    for name, colour in SERIES.items():
        if name not in traces:
            continue
        episode = traces[name][0]
        t = np.array([p["t"] for p in episode[1:]])
        nearest = np.array([p["nearest"] for p in episode[1:]], dtype=float)
        ax.plot(t, nearest, color=colour,
                linewidth=2.0 if name.startswith("ppo") else 1.2,
                linestyle="-" if name.startswith("ppo") else (0, (4, 2)),
                zorder=5 if name.startswith("ppo") else 3)
        ax.annotate(name, xy=(t[-1], nearest[-1]), xytext=(4, 0),
                    textcoords="offset points", color=colour, fontsize=8,
                    va="center", fontweight="semibold")
    margin = scenario.collision_radius_m
    ax.axhline(margin, color=CRITICAL, linewidth=1.2, linestyle=(0, (3, 2)))
    ax.text(0.01, margin, f" collision radius {margin} m", color=CRITICAL,
            fontsize=8, va="bottom", transform=ax.get_yaxis_transform())
    ax.set_xlabel("simulation time since reset [s]")
    ax.set_ylabel("nearest lidar return [m]")
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.set_title("Measured obstacle clearance over an episode\n"
                 "minimum over the 12 observed sectors; lidar measurement, "
                 "not supervisor ground truth", loc="left", color=INK)
    save(fig, "fig06_clearance")


# --------------------------------------------------------------- figure 3
def figure_lidar_sectors() -> None:
    packet = None
    for candidate in sorted(RESULTS.glob("two_frames_webots_*_lidar.json")):
        packet = json.loads(candidate.read_text(encoding="utf8"))
        break
    if packet is None:
        pending.append("fig03 lidar sectors: no two_frames_webots_*_lidar.json")
        print("  PENDING fig03 lidar sectors: no lidar handoff packet")
        return

    frame = packet["frames"][0]
    ranges = np.array(frame["ranges"], dtype=float)
    valid = np.array(frame["valid"], dtype=bool)
    cap = 5.0
    n = len(ranges)
    width = 2 * math.pi / n
    centres = np.arange(n) * width

    fig = plt.figure(figsize=(6.6, 7.4))
    # Leave real space above the dial: the sector-0 callout sits outside the
    # axes at r = 1.4 * cap and will collide with a tight-layout title.
    fig.subplots_adjust(top=0.80, bottom=0.04)
    ax = fig.add_subplot(111, projection="polar")
    # DNS convention: 0 deg is forward (drawn up) and angles grow clockwise
    # towards the drone's right, so the picture matches the sector numbering.
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_facecolor(SURFACE)

    ax.bar(centres[valid], ranges[valid], width=width * 0.94, bottom=0.0,
           color=BLUE, alpha=0.78, edgecolor=SURFACE, linewidth=1.5, zorder=3)
    if (~valid).any():
        ax.bar(centres[~valid], np.full((~valid).sum(), cap), width=width * 0.94,
               color="#dcdbd4", edgecolor=SURFACE, linewidth=1.5, zorder=2)

    for i, (centre, r, ok) in enumerate(zip(centres, ranges, valid)):
        label = f"{i}\n{r:.2f} m" if ok else f"{i}\nunobserved"
        ax.text(centre, cap * 1.14, label, ha="center", va="center",
                fontsize=7.5, color=INK2)
    for name, sector in (("FORWARD", 0), ("RIGHT", 3), ("BACK", 6), ("LEFT", 9)):
        ax.text(centres[sector], cap * 1.40, name, ha="center", va="center",
                fontsize=8.5, color=INK, fontweight="semibold")

    ax.set_ylim(0, cap)
    ax.set_yticks([1, 2, 3, 4, 5])
    ax.set_yticklabels(["1", "2", "3", "4", "5 m"], fontsize=7)
    # Park the radial ticks on an empty bearing so they clear the sector callouts.
    ax.set_rlabel_position(255)
    ax.set_xticks(centres)
    ax.set_xticklabels([])
    ax.grid(color=GRID, linewidth=0.6)
    mount = packet.get("lidar_mount", "?")
    beams = packet.get("lidar_beams", "?")
    fig.suptitle(
        "Measured 12-sector lidar scan at episode reset\n"
        f"{beams} beams binned into 30 deg sectors · mount: {mount} · "
        "range capped at 5 m\n0 deg forward, angles increase clockwise "
        "towards the drone's right",
        x=0.02, y=0.985, ha="left", va="top", color=INK, fontsize=10)
    save(fig, "fig03_lidar_sectors")


# -------------------------------------------------------------- figure 11
def figure_mount_comparison() -> None:
    data = load(RESULTS / "lidar_attitude.json", "fig11 mount comparison")
    if not data:
        return
    steps = data["steps"]
    modes = [m for m in ("body_fixed", "stabilised")
             if any(s["mount"] == m for s in steps)]
    if not modes:
        return

    fig, axes = plt.subplots(1, 2, figsize=(8.6, 3.9))
    colours = {"body_fixed": ORANGE, "stabilised": BLUE}

    ax = axes[0]
    for mode in modes:
        rows = [s for s in steps if s["mount"] == mode]
        tilt = np.array([r["tilt_deg"] for r in rows])
        rms = np.array([r["paired_rms_m"] for r in rows], dtype=float)
        ax.scatter(tilt, rms, s=34, color=colours[mode], alpha=0.85,
                   edgecolor=SURFACE, linewidth=1.0, zorder=4, label=mode)
        order = np.argsort(tilt)
        ax.plot(tilt[order], rms[order], color=colours[mode], linewidth=1.0,
                alpha=0.45, zorder=3)
    ax.set_xlabel("airframe tilt [deg]")
    ax.set_ylabel("sector range error vs ground truth, RMS [m]")
    ax.legend(loc="upper left")
    ax.set_title("Range accuracy vs attitude", loc="left", color=INK)

    # With the corrected arena (2.5 m walls, 0.14 m mount) neither mount loses a
    # surface outright, so a violation-count panel would read all-zero and imply
    # the mounts are equivalent. What still separates them is accuracy.
    ax = axes[1]
    width = 0.36
    metrics = [("median_paired_rms_m", "median"), ("worst_paired_rms_m", "worst case")]
    x = np.arange(len(metrics))
    for i, mode in enumerate(modes):
        values = [data["by_mount"][mode][key] for key, _ in metrics]
        offset = (i - (len(modes) - 1) / 2) * (width + 0.02)
        bars = ax.bar(x + offset, values, width, color=colours[mode],
                      edgecolor=SURFACE, linewidth=1.5, label=mode, zorder=3)
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, value + 0.004,
                    f"{value:.3f}", ha="center", fontsize=8, color=INK2,
                    fontweight="semibold")
    ax.set_xticks(x)
    ax.set_xticklabels([label for _, label in metrics])
    ax.set_ylabel("sector range error vs ground truth, RMS [m]")
    ax.set_ylim(0, max(data["by_mount"][m][k]
                       for m in modes for k, _ in metrics) * 1.30)
    ax.legend(loc="upper left")
    ax.grid(axis="x", visible=False)
    strict = {m: data["by_mount"][m]["strict_steps_with_too_short"] for m in modes}
    ax.set_title("Error magnitude over 29 control steps", loc="left", color=INK)
    ax.text(0.5, -0.30,
            "self-occlusion / floor-strike steps:  "
            + ",  ".join(f"{m} {strict[m]}" for m in modes),
            transform=ax.transAxes, ha="center", fontsize=8, color=INK2)

    fig.suptitle(
        "Body-fixed vs gimbal-stabilised lidar mount, identical commanded "
        "manoeuvres\n"
        "corrected arena (2.5 m walls, 0.14 m mount): neither mount now loses a "
        "wall outright — before the fix the\nbody-fixed plane reported "
        "\"clear to 5 m\" over a real wall in 23 of 29 steps (mean 3.4 of 12 "
        "sectors)",
        x=0.005, ha="left", color=INK, fontsize=10, fontweight="semibold")
    fig.tight_layout(rect=(0, 0.02, 1, 0.88))
    save(fig, "fig11_mount_comparison")


def main() -> int:
    scenario = Scenario.load(DEFAULT_SCENARIO_PATH)
    print(f"scenario: {scenario.name}")
    figure_lidar_sectors()
    figure_trajectories(scenario)
    figure_velocity_tracking()
    figure_clearance(scenario)
    figure_mount_comparison()

    print(f"\n{len(written)} figures written to "
          f"{FIGURES.relative_to(PROJECT_ROOT)}/")
    if pending:
        print(f"{len(pending)} PENDING (input not available, nothing invented):")
        for item in pending:
            print(f"  - {item}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
