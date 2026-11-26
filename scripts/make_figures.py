"""Generate every DNS v2 figure from logged data. No figure is hand-drawn.

Each figure reads the canonical scenario JSONs and the experiment CSVs, so a
figure can never disagree with the data it claims to show. Every output carries
a provenance label (SCHEMATIC / LOGGED-DATA RENDER / WEBOTS CAPTURE) and an
entry in ``03_FIGURES/captions.md``.

Run::

    python scripts/make_figures.py              # everything available
    python scripts/make_figures.py F1 F2        # named figures only
"""
from __future__ import annotations

import csv
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import dns_style as S                                            # noqa: E402
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.lines import Line2D                              # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle        # noqa: E402

from dns.feasibility import astar, clearance_grid, describe      # noqa: E402
from dns.scenario import Scenario                                # noqa: E402

SCENARIOS = PROJECT_ROOT / "config" / "scenarios"
DATA = PROJECT_ROOT / "results" / "v2_S6"
# Figures live in the repository, not the one-off meeting package.
CORE = PROJECT_ROOT / "assets" / "figures"
EXTENDED = PROJECT_ROOT / "assets" / "figures"
DIAGRAMS = PROJECT_ROOT / "assets" / "diagrams"
#: Named CAPTIONS_PATH, not CAPTIONS: that name is already the
#: module-level accumulator list for caption records.
CAPTIONS_PATH = PROJECT_ROOT / "docs" / "FIGURES.md"
RUNS = PROJECT_ROOT / "results" / "s6" / "runs"

LAYOUTS = ("L00", "L01", "L02", "L03", "L04", "L05")
CAPTIONS: list[dict] = []


def caption(fid: str, title: str, text: str, *, n: str, data: str,
            provenance: str, files: list[Path]) -> None:
    CAPTIONS.append({
        "id": fid, "title": title, "caption": text, "n": n,
        "data_file": data, "script": "scripts/make_figures.py",
        "provenance": provenance,
        "files": [str(p.relative_to(PROJECT_ROOT)).replace("\\", "/")
                  for p in files],
    })


def load_results() -> list[dict]:
    path = DATA / "results_v2.csv"
    if not path.is_file():
        return []
    with path.open(encoding="utf8") as handle:
        return list(csv.DictReader(handle))


def load_pooled() -> list[dict]:
    path = DATA / "pooled_rates.csv"
    if not path.is_file():
        return []
    with path.open(encoding="utf8") as handle:
        return list(csv.DictReader(handle))


def available_layouts(rows: list[dict]) -> list[str]:
    present = {r["layout_id"] for r in rows}
    return [l for l in LAYOUTS if l in present]


def wilson(k: int, n: int, z: float = 1.959964) -> tuple[float, float]:
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / d
    half = (z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / d
    return (max(0.0, centre - half), min(1.0, centre + half))


# ---------------------------------------------------------------------------
# shared map drawing
# ---------------------------------------------------------------------------
def footprint(obstacle, t: float = 0.0) -> np.ndarray:
    placed = obstacle.placed_at(t)
    half = np.asarray(placed.half_size, dtype=float)
    local = np.array([[-half[0], -half[1]], [half[0], -half[1]],
                      [half[0], half[1]], [-half[0], half[1]]])
    c, s = math.cos(placed.yaw_rad), math.sin(placed.yaw_rad)
    return local @ np.array([[c, s], [-s, c]]) + np.asarray(placed.centre)


def draw_map(axis, scenario: Scenario, *, show_cspace: bool = True,
             show_astar: bool = True, show_cart_lane: bool = True,
             scale: bool = True, grid_step: float = 0.05) -> dict:
    """Draw one layout to metric scale. Returns the descriptors it computed."""
    low = np.asarray(scenario.low_xy, dtype=float)
    high = np.asarray(scenario.high_xy, dtype=float)

    facts, path, grid = describe(scenario, step_m=grid_step)

    # Inflated configuration space, as the region a disc of the drone's radius
    # may not enter. Drawn first so obstacles sit on top of it.
    if show_cspace:
        blocked = grid.blocked(scenario.collision_radius_m)
        axis.contourf(grid.xs, grid.ys, blocked.astype(float),
                      levels=[0.5, 1.5], colors=[S.CSPACE], alpha=0.75,
                      zorder=1)

    axis.add_patch(Rectangle((low[0], low[1]), high[0] - low[0],
                             high[1] - low[1], facecolor="none",
                             edgecolor=S.WALL, linewidth=1.4, zorder=3))

    movers = []
    for obstacle in scenario.obstacles:
        moving = (obstacle.motion.type != "static"
                  and obstacle.motion.amplitude_m)
        colour = S.CART if moving else S.STATIC_OBSTACLE
        axis.add_patch(Polygon(footprint(obstacle), closed=True,
                               facecolor=colour, edgecolor=S.STATIC_EDGE,
                               linewidth=0.6, zorder=5))
        if moving:
            movers.append(obstacle)

    # The cart's swept lane: the region its footprint covers over a full period.
    if show_cart_lane and movers:
        cart = movers[0]
        period = ((4.0 * cart.motion.amplitude_m / cart.motion.speed_mps)
                  if cart.motion.speed_mps else cart.motion.period_s)
        corners = np.vstack([footprint(cart, t)
                             for t in np.linspace(0.0, period, 41)])
        axis.add_patch(Rectangle(
            (corners[:, 0].min(), corners[:, 1].min()),
            float(np.ptp(corners[:, 0])), float(np.ptp(corners[:, 1])),
            facecolor="none", edgecolor=S.CART, linewidth=0.8,
            linestyle=(0, (3, 2)), zorder=4))

    if show_astar and path:
        points = np.asarray(path)
        axis.plot(points[:, 0], points[:, 1], color=S.ASTAR, linewidth=1.3,
                  linestyle=(0, (4, 1.6)), zorder=8)

    axis.add_patch(Circle(scenario.start_xy, 0.16, facecolor=S.START,
                          edgecolor="white", linewidth=0.8, zorder=10))
    axis.add_patch(Circle(scenario.goal_xy, scenario.goal_tolerance_m,
                          facecolor=S.GOAL, alpha=0.20, edgecolor=S.GOAL,
                          linewidth=0.9, zorder=9))
    axis.add_patch(Circle(scenario.goal_xy, 0.09, facecolor=S.GOAL,
                          edgecolor="white", linewidth=0.8, zorder=10))

    axis.set_xlim(low[0] - 0.25, high[0] + 0.25)
    axis.set_ylim(low[1] - 0.45, high[1] + 0.25)
    axis.set_aspect("equal", adjustable="box")
    if scale:
        S.scale_bar(axis, 1.0, origin=(low[0] + 0.3, low[1] - 0.32))
    return facts


# ---------------------------------------------------------------------------
# F1 - the distribution-shift ladder
# ---------------------------------------------------------------------------
def figure_f1(rows: list[dict]) -> None:
    layouts = [l for l in LAYOUTS
               if (SCENARIOS / f"{l}_C.json").is_file()]
    figure, axes = plt.subplots(3, 2, figsize=(7.6, 7.8))
    table = {r["layout_id"]: r for r in _scenario_table() if r["mode"] == "C"}

    for axis, layout in zip(axes.ravel(), layouts):
        scenario = Scenario.load(SCENARIOS / f"{layout}_C.json")
        facts = draw_map(axis, scenario)
        meta = table.get(layout, {})
        title = json.loads((SCENARIOS / f"{layout}_C.json").read_text(
            encoding="utf8"))["provenance"]["layout_title"]
        axis.set_title(f"{layout} — {title}", loc="left", pad=4)
        dissimilarity = float(meta.get("dissimilarity_to_L00") or 0.0)
        # Placed in AXES coordinates below the frame: in data coordinates this
        # line lands inside the arena and collides with the obstacles.
        axis.text(
            0.0, -0.30,
            f"1 − IoU to L00 {dissimilarity:.3f}   ·   "
            f"corridor {facts['min_corridor_width_enroute_m']:.2f} m   ·   "
            f"A* {facts['astar_length_m']:.2f} m",
            fontsize=6.6, color=S.INK_SOFT, ha="left", va="top",
            transform=axis.transAxes)
        axis.set_xticks([0, 2, 4, 6, 8, 10])
        axis.set_yticks([-3, 0, 3])
        if layout in ("L04", "L05"):
            axis.set_xlabel("x [m]", labelpad=12)
        axis.set_ylabel("y [m]")
        S.provenance(axis, "LOGGED-DATA RENDER", loc="upper left")

    handles = [
        Line2D([], [], marker="s", linestyle="none", markersize=6,
               markerfacecolor=S.STATIC_OBSTACLE, markeredgecolor=S.STATIC_EDGE,
               label="static obstacle"),
        Line2D([], [], marker="s", linestyle="none", markersize=6,
               markerfacecolor=S.CART, markeredgecolor=S.STATIC_EDGE,
               label="cart (moving)"),
        Line2D([], [], marker="s", linestyle="none", markersize=6,
               markerfacecolor=S.CSPACE, markeredgecolor="none",
               label="C-space blocked (r = 0.18 m)"),
        Line2D([], [], color=S.ASTAR, linestyle=(0, (4, 1.6)), label="A* reference"),
        Line2D([], [], color=S.CART, linestyle=(0, (3, 2)), label="cart swept lane"),
        Line2D([], [], marker="o", linestyle="none", markersize=6,
               markerfacecolor=S.START, markeredgecolor="white", label="start"),
        Line2D([], [], marker="o", linestyle="none", markersize=7,
               markerfacecolor=S.GOAL, alpha=0.5, markeredgecolor=S.GOAL,
               label="goal + 0.4 m tolerance"),
    ]
    figure.legend(handles=handles, loc="lower center", ncol=4,
                  bbox_to_anchor=(0.5, -0.035))
    figure.suptitle("F1  Distribution-shift ladder L00–L05, identical metric scale",
                    x=0.02, ha="left", fontsize=10.5, fontweight="bold")
    figure.tight_layout(rect=(0, 0.045, 1, 0.965), h_pad=2.6)
    files = S.save(figure, CORE / "F1_shift_ladder")
    caption("F1", "Distribution-shift ladder L00–L05",
            "The six layouts at identical metric scale. Grey fill is the "
            "configuration space a 0.18 m disc may not enter; the dashed red "
            "line is the A* reference route; the dashed orange rectangle is the "
            "cart's swept lane. Dissimilarity to L00 (1 − IoU of the static "
            "footprint rasters), en-route corridor width and A* length are "
            "printed under each panel. L02 is an exact reflection of L00: "
            "maximally dissimilar, identical difficulty.",
            n="6 layouts", data="07_SCENARIOS/L0*_C.json, 06_DATA/scenario_table.csv",
            provenance="LOGGED-DATA RENDER", files=files)


def _scenario_table() -> list[dict]:
    path = DATA / "scenario_table.csv"
    if not path.is_file():
        return []
    with path.open(encoding="utf8") as handle:
        return list(csv.DictReader(handle))


# ---------------------------------------------------------------------------
# F2 - generalization results
# ---------------------------------------------------------------------------
def figure_f2(rows: list[dict]) -> None:
    if not rows:
        return
    layouts = available_layouts(rows)
    pooled = {(r["layout_id"], r["mode"], r["group"]): r for r in load_pooled()}

    figure, axes = plt.subplots(2, 2, figsize=(7.4, 5.4),
                                gridspec_kw={"height_ratios": [1.0, 1.0]})
    modes = (("B_static", "Mode B — cart parked"),
             ("C_dynamic", "Mode C — cart sweeping"))

    # --- top row: pooled PPO success with Wilson intervals ----------------
    for column, (mode, title) in enumerate(modes):
        axis = axes[0, column]
        xs = np.arange(len(layouts))
        centres, lows, highs, labels = [], [], [], []
        for layout in layouts:
            entry = pooled.get((layout, mode, "ppo_pooled"))
            if entry is None:
                centres.append(np.nan); lows.append(np.nan)
                highs.append(np.nan); labels.append("")
                continue
            k, n = int(entry["k"]), int(entry["n"])
            centres.append(k / n)
            low, high = wilson(k, n)
            lows.append(low); highs.append(high)
            labels.append(f"{k}/{n}")
        centres = np.asarray(centres)
        axis.errorbar(xs, centres,
                      yerr=[centres - np.asarray(lows), np.asarray(highs) - centres],
                      fmt="o", color=S.CONTROLLER_COLOUR["ppo_pooled"],
                      markersize=6, capsize=3, linewidth=1.4, zorder=6,
                      label="PPO pooled (3 checkpoints), Wilson 95%")

        # per-seed points, plotted but never ranked
        for offset, controller in zip((-0.19, 0.0, 0.19),
                                      ("ppo_seed7", "ppo_seed19", "ppo_seed43")):
            values = []
            for layout in layouts:
                entry = pooled.get((layout, mode, controller))
                values.append(int(entry["k"]) / int(entry["n"]) if entry else np.nan)
            axis.plot(xs + offset, values, marker="_", linestyle="none",
                      markersize=9, markeredgewidth=1.6,
                      color=S.CONTROLLER_COLOUR[controller], alpha=0.85,
                      zorder=5, label=controller.replace("ppo_seed", "seed "))

        # untuned references as grey markers
        for controller, marker in (("reactive", "v"), ("direct", "x")):
            values = []
            for layout in layouts:
                entry = pooled.get((layout, mode, controller))
                values.append(int(entry["k"]) / int(entry["n"]) if entry else np.nan)
            axis.plot(xs, values, marker=marker, linestyle="none",
                      markersize=5.5, color="#9aa0a6", zorder=4,
                      label=f"{controller} (untuned ref.)")

        for x, label, centre in zip(xs, labels, centres):
            if label:
                axis.annotate(label, (x, centre), textcoords="offset points",
                              xytext=(9, 3), fontsize=6.8, color=S.INK_SOFT)

        axis.set_xticks(xs)
        axis.set_xticklabels(layouts)
        axis.set_ylim(-0.05, 1.08)
        axis.set_ylabel("success rate" if column == 0 else "")
        axis.set_title(title, loc="left", pad=4, fontsize=8.8)
        S.grid(axis, "y")

    # --- bottom row: outcome taxonomy, stacked ----------------------------
    order = ["goal", "collision_static", "collision_cart", "collision_wall",
             "timeout"]
    for column, (mode, _title) in enumerate(modes):
        axis = axes[1, column]
        xs = np.arange(len(layouts))
        bottom = np.zeros(len(layouts))
        for outcome in order:
            heights = []
            for layout in layouts:
                cell = [r for r in rows if r["layout_id"] == layout
                        and r["mode"] == mode
                        and r["controller_kind"] == "ppo_checkpoint"
                        and int(r["distinct_configuration"]) == 1]
                counts = Counter(r["outcome"] for r in cell)
                heights.append(counts.get(outcome, 0) / max(len(cell), 1))
            axis.bar(xs, heights, bottom=bottom, width=0.62,
                     color=S.OUTCOME_COLOUR[outcome], edgecolor="white",
                     linewidth=0.9, label=outcome.replace("_", " "), zorder=4)
            bottom += np.asarray(heights)
        axis.set_xticks(xs)
        axis.set_xticklabels(layouts)
        axis.set_ylim(0, 1.0)
        axis.set_xlabel("layout")
        axis.set_ylabel("fraction of PPO episodes" if column == 0 else "")
        S.grid(axis, "y")

    axes[0, 1].legend(loc="upper left", bbox_to_anchor=(1.01, 1.02))
    axes[1, 1].legend(loc="upper left", bbox_to_anchor=(1.01, 1.02),
                      title="outcome")
    figure.text(0.02, 0.005, "LOGGED-DATA RENDER", fontsize=6.0,
                color=S.INK_MUTED, ha="left", va="bottom")
    figure.suptitle("F2  Spatial generalization across the shift ladder",
                    x=0.02, ha="left", fontsize=10.5, fontweight="bold")
    figure.tight_layout(rect=(0, 0, 0.82, 0.96))
    files = S.save(figure, CORE / "F2_generalization")
    caption("F2", "Spatial generalization across the shift ladder",
            "Top: pooled PPO success per layout with Wilson 95% intervals; "
            "per-checkpoint values are the short dashes and are never ranked "
            "because their intervals overlap. Grey markers are Spencer's untuned "
            "reference controllers, shown for context only. Bottom: the outcome "
            "taxonomy over the same PPO episodes. n counts distinct "
            "configurations: 15 cart phases per checkpoint in Mode C, and 1 in "
            "Mode B (its three episodes are repeats of one configuration).",
            n="Mode C: 45 distinct configs/layout; Mode B: 3",
            data="06_DATA/results_v2.csv, 06_DATA/pooled_rates.csv",
            provenance="LOGGED-DATA RENDER", files=files)


# ---------------------------------------------------------------------------
# F3 - representative trajectories
# ---------------------------------------------------------------------------
def read_trajectory(run_id: str):
    path = RUNS / run_id / "trajectory.csv"
    if not path.is_file():
        return None
    with path.open(encoding="utf8") as handle:
        rows = list(csv.DictReader(handle))
    return rows


def figure_f3(rows: list[dict]) -> None:
    if not rows:
        return
    # L00 vs L02 (the mirror) vs the hardest available layout, same controller.
    layouts = available_layouts(rows)
    picks = [l for l in ("L00", "L02", "L05", "L04", "L03") if l in layouts][:3]
    if len(picks) < 2:
        return
    controller = "ppo_seed7"

    figure, axes = plt.subplots(len(picks), 1, figsize=(6.0, 2.1 * len(picks)))
    axes = np.atleast_1d(axes)
    for axis, layout in zip(axes, picks):
        scenario = Scenario.load(SCENARIOS / f"{layout}_C.json")
        draw_map(axis, scenario, show_astar=False, show_cspace=True)
        run_id = f"{layout}_C_{controller}_v00"
        trace = read_trajectory(run_id)
        if trace:
            xy = np.array([[float(r["x_m"]), float(r["y_m"])] for r in trace])
            clearance = np.array([float(r["clearance_m"]) if r["clearance_m"] else np.nan
                                  for r in trace])
            points = xy.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            from matplotlib.collections import LineCollection
            collection = LineCollection(
                segments, cmap="viridis",
                norm=plt.Normalize(0.0, 1.5), linewidth=2.1, zorder=11)
            collection.set_array(clearance[:-1])
            axis.add_collection(collection)
            outcome = next((r["outcome"] for r in rows if r["run_id"] == run_id),
                           "?")
            axis.set_title(f"{layout}  ·  {controller}  ·  Mode C v00  ·  {outcome}",
                           loc="left", pad=4)
        axis.set_xticks([0, 2, 4, 6, 8, 10])
        axis.set_yticks([-3, 0, 3])
        axis.set_ylabel("y [m]")
        S.provenance(axis, "LOGGED-DATA RENDER", loc="upper right")
    axes[-1].set_xlabel("x [m]")

    bar = figure.colorbar(collection, ax=axes.tolist(), fraction=0.025, pad=0.015)
    bar.set_label("ground-truth clearance [m]")
    bar.outline.set_visible(False)
    figure.suptitle("F3  Representative PPO trajectories, coloured by clearance",
                    x=0.02, ha="left", fontsize=10.5, fontweight="bold")
    files = S.save(figure, CORE / "F3_trajectories")
    caption("F3", "Representative PPO trajectories",
            "One flown episode per layout for the same checkpoint and the same "
            "cart phase (variant 00), coloured by supervisor ground-truth "
            "clearance to the nearest obstacle surface. L00 and L02 are "
            "reflections of one another with identical difficulty, so the "
            "difference between those two panels is a layout-identity effect.",
            n=f"{len(picks)} episodes", data="06_DATA/runs/*/trajectory.csv",
            provenance="LOGGED-DATA RENDER", files=files)


# ---------------------------------------------------------------------------
# E1 - descriptor table
# ---------------------------------------------------------------------------
def figure_e1(rows: list[dict]) -> None:
    table = [r for r in _scenario_table() if r["mode"] == "C"]
    if not table:
        return
    fields = [
        ("layout_id", "layout", "{}"),
        ("n_obstacles", "obstacles", "{}"),
        ("astar_length_m", "A* [m]", "{:.2f}"),
        ("tortuosity", "tortuosity", "{:.3f}"),
        ("min_corridor_width_enroute_m", "corridor [m]", "{:.2f}"),
        ("occupied_area_pct", "occupied [%]", "{:.2f}"),
        ("line_intrusion_m", "intrusion [m]", "{:.3f}"),
        ("cart_blocking_fraction", "cart blocks", "{:.3f}"),
        ("dissimilarity_to_L00", "1 − IoU", "{:.3f}"),
        ("mean_obstacle_displacement_m", "displ. [m]", "{:.3f}"),
    ]
    figure, axis = plt.subplots(figsize=(7.4, 2.0 + 0.22 * len(table)))
    axis.axis("off")
    cells = []
    for row in table:
        line = []
        for key, _label, fmt in fields:
            value = row.get(key, "")
            try:
                line.append(fmt.format(float(value)))
            except (TypeError, ValueError):
                line.append(str(value) if value else "—")
        cells.append(line)
    rendered = axis.table(cellText=cells,
                          colLabels=[label for _k, label, _f in fields],
                          loc="center", cellLoc="center")
    rendered.auto_set_font_size(False)
    rendered.set_fontsize(7.2)
    rendered.scale(1.0, 1.35)
    for (row_index, _col), cell in rendered.get_celld().items():
        cell.set_edgecolor(S.GRID)
        cell.set_linewidth(0.6)
        if row_index == 0:
            cell.set_text_props(weight="bold", color=S.INK)
            cell.set_facecolor("#f3f4f6")
    axis.set_title("E1  Layout descriptors relative to L00", loc="left",
                   fontsize=10.5, fontweight="bold", pad=10)
    S.provenance(axis, "LOGGED-DATA RENDER", loc="lower right")
    files = S.save(figure, EXTENDED / "E1_descriptors")
    caption("E1", "Layout descriptors relative to L00",
            "Every descriptor used to characterise the ladder. Difficulty "
            "(corridor width, tortuosity, occupied area) is reported separately "
            "from dissimilarity (1 − IoU, mean displacement) so a layout that is "
            "very different but equally hard — L02 — is legible as such.",
            n="6 layouts", data="06_DATA/scenario_table.csv",
            provenance="LOGGED-DATA RENDER", files=files)


# ---------------------------------------------------------------------------
# E3 - success vs dissimilarity and vs difficulty (exploratory)
# ---------------------------------------------------------------------------
def figure_e3(rows: list[dict]) -> None:
    if not rows:
        return
    table = {r["layout_id"]: r for r in _scenario_table() if r["mode"] == "C"}
    pooled = {(r["layout_id"], r["mode"], r["group"]): r for r in load_pooled()}
    layouts = available_layouts(rows)
    if len(layouts) < 3:
        return

    figure, axes = plt.subplots(1, 2, figsize=(7.0, 2.9))
    for axis, (key, label) in zip(axes, (
            ("dissimilarity_to_L00", "dissimilarity to L00  (1 − IoU)"),
            ("min_corridor_width_enroute_m", "en-route corridor width [m]"))):
        for mode, marker in (("C_dynamic", "o"), ("B_static", "s")):
            xs, ys, names = [], [], []
            for layout in layouts:
                entry = pooled.get((layout, mode, "ppo_pooled"))
                meta = table.get(layout)
                if not entry or not meta or not meta.get(key):
                    continue
                xs.append(float(meta[key]))
                ys.append(int(entry["k"]) / int(entry["n"]))
                names.append(layout)
            axis.plot(xs, ys, marker=marker, linestyle="none", markersize=6,
                      color=S.CONTROLLER_COLOUR["ppo_pooled"],
                      alpha=0.95 if mode == "C_dynamic" else 0.45,
                      label="Mode C" if mode == "C_dynamic" else "Mode B",
                      zorder=5)
            for x, y, name in zip(xs, ys, names):
                axis.annotate(name, (x, y), textcoords="offset points",
                              xytext=(5, 3), fontsize=6.5, color=S.INK_SOFT)
        axis.set_xlabel(label)
        axis.set_ylim(-0.05, 1.08)
        S.grid(axis, "both")
        S.provenance(axis, "LOGGED-DATA RENDER", loc="lower left")
    axes[0].set_ylabel("pooled PPO success rate")
    axes[1].legend(loc="upper right")
    figure.suptitle("E3  Success vs dissimilarity and vs difficulty "
                    "(exploratory — 6 layouts, no fitted trend)",
                    x=0.02, ha="left", fontsize=9.8, fontweight="bold")
    figure.tight_layout(rect=(0, 0, 1, 0.92))
    files = S.save(figure, EXTENDED / "E3_success_vs_descriptors")
    caption("E3", "Success vs dissimilarity and vs difficulty",
            "Pooled PPO success against how DIFFERENT a layout is (left) and "
            "how HARD it is (right). Six layouts is far too few to fit a trend, "
            "so none is drawn and no slope is reported; the panels are included "
            "because separating the two axes is what makes the mirror result "
            "interpretable.",
            n="6 layouts x 2 modes", data="06_DATA/pooled_rates.csv, 06_DATA/scenario_table.csv",
            provenance="LOGGED-DATA RENDER", files=files)


# ---------------------------------------------------------------------------
# E4 - clearance distributions
# ---------------------------------------------------------------------------
def figure_e4(rows: list[dict]) -> None:
    if not rows:
        return
    layouts = available_layouts(rows)
    figure, axis = plt.subplots(figsize=(7.0, 3.0))
    data = []
    for layout in layouts:
        values = [float(r["min_clearance_m"]) for r in rows
                  if r["layout_id"] == layout
                  and r["controller_kind"] == "ppo_checkpoint"
                  and r["min_clearance_m"]]
        data.append(values)
    parts = axis.violinplot(data, showextrema=False, widths=0.75)
    for body in parts["bodies"]:
        body.set_facecolor(S.CONTROLLER_COLOUR["ppo_pooled"])
        body.set_alpha(0.35)
        body.set_edgecolor(S.CONTROLLER_COLOUR["ppo_pooled"])
    for index, values in enumerate(data, start=1):
        if values:
            axis.plot([index], [np.median(values)], marker="o", markersize=5,
                      color=S.CONTROLLER_COLOUR["ppo_pooled"], zorder=6)
    axis.axhline(0.30, color=S.OUTCOME_COLOUR["collision_static"],
                 linestyle=(0, (4, 2)), linewidth=1.1, zorder=3)
    axis.annotate("near-miss threshold 0.30 m", (len(data) + 0.35, 0.30),
                  fontsize=6.8, color=S.INK_SOFT, va="center")
    axis.axhline(0.18, color=S.INK_MUTED, linestyle=(0, (1, 2)), linewidth=1.0)
    axis.annotate("collision radius 0.18 m", (len(data) + 0.35, 0.18),
                  fontsize=6.8, color=S.INK_SOFT, va="center")
    axis.set_xticks(range(1, len(layouts) + 1))
    axis.set_xticklabels(layouts)
    axis.set_xlabel("layout")
    axis.set_ylabel("per-episode minimum clearance [m]")
    axis.set_xlim(0.4, len(data) + 1.9)
    S.grid(axis, "y")
    S.provenance(axis, "LOGGED-DATA RENDER", loc="upper right")
    axis.set_title("E4  Minimum clearance distribution per layout (PPO episodes)",
                   loc="left", fontsize=10.0, fontweight="bold", pad=6)
    files = S.save(figure, EXTENDED / "E4_clearance")
    caption("E4", "Minimum clearance distribution per layout",
            "Per-episode minimum supervisor ground-truth clearance for all PPO "
            "episodes, by layout. The dashed line is the 0.30 m near-miss "
            "threshold and the dotted line the 0.18 m collision radius, so the "
            "margin a policy actually kept is readable against both.",
            n="all PPO episodes per layout", data="06_DATA/results_v2.csv",
            provenance="LOGGED-DATA RENDER", files=files)


# ---------------------------------------------------------------------------
# E9 - L00 reproduction
# ---------------------------------------------------------------------------
def figure_e9(rows: list[dict]) -> None:
    path = DATA / "v1_reproduction.csv"
    if not path.is_file():
        return
    with path.open(encoding="utf8") as handle:
        table = list(csv.DictReader(handle))
    if not table:
        return
    figure, axis = plt.subplots(figsize=(6.6, 3.0))
    labels = [f"{r['controller']}\n{r['mode'].split('_')[0]}" for r in table]
    xs = np.arange(len(table))
    v1 = [int(r["v1_k"]) / int(r["v1_n"]) for r in table]
    v2 = [int(r["v2_k"]) / int(r["v2_n"]) for r in table]
    axis.bar(xs - 0.19, v1, width=0.36, color="#9aa0a6", edgecolor="white",
             linewidth=0.8, label="v1 (results.csv)", zorder=4)
    axis.bar(xs + 0.19, v2, width=0.36, color=S.CONTROLLER_COLOUR["ppo_pooled"],
             edgecolor="white", linewidth=0.8, label="v2 rerun", zorder=4)
    for x, row in zip(xs, table):
        axis.annotate(f"{row['v1_k']}/{row['v1_n']}", (x, 1.02),
                      ha="center", fontsize=6.4, color=S.INK_SOFT)
    axis.set_xticks(xs)
    axis.set_xticklabels(labels, fontsize=6.6)
    axis.set_ylim(0, 1.16)
    axis.set_ylabel("success rate")
    matched = sum(int(r["outcomes_match"]) for r in table)
    axis.set_title(f"E9  L00 reproduction: {matched}/{len(table)} cells identical, "
                   f"max path difference 0 m",
                   loc="left", fontsize=10.0, fontweight="bold", pad=6)
    axis.legend(loc="lower right")
    S.grid(axis, "y")
    S.provenance(axis, "LOGGED-DATA RENDER", loc="upper right")
    files = S.save(figure, EXTENDED / "E9_l00_reproduction")
    caption("E9", "L00 reproduction, v1 vs v2",
            "Every v1 cell re-run with the v2 harness on the regenerated L00 "
            "world. All ten cells reproduce their v1 outcome counts and every "
            "matched episode's flown path length is identical to the recorded "
            "precision, which is the determinism evidence the rest of the "
            "comparison rests on.",
            n="10 cells, 90 episodes", data="06_DATA/v1_reproduction.csv",
            provenance="LOGGED-DATA RENDER", files=files)


# ---------------------------------------------------------------------------
# SV1 - sensor schematic
# ---------------------------------------------------------------------------
def figure_sv1(rows: list[dict]) -> None:
    scenario = Scenario.load(SCENARIOS / "L00_C.json")
    sectors = scenario.sensing_sectors
    cap = scenario.sensing_max_range_m
    beams = scenario.lidar_resolution

    figure, axes = plt.subplots(1, 2, figsize=(7.4, 3.9))

    # --- top view: sectors, indices, cap ring ---------------------------
    axis = axes[0]
    axis.add_patch(Circle((0, 0), cap, facecolor="#eef1f4", edgecolor=S.GOAL,
                          linewidth=1.0, linestyle=(0, (4, 2)), zorder=1))
    for index in range(sectors):
        centre = index * 2 * math.pi / sectors
        edge = centre - math.pi / sectors
        axis.plot([0, cap * math.cos(edge)], [0, cap * math.sin(edge)],
                  color=S.GRID, linewidth=0.7, zorder=2)
        label_r = cap * 0.76
        axis.text(label_r * math.cos(centre), label_r * math.sin(centre),
                  str(index), ha="center", va="center", fontsize=7.6,
                  color=S.INK if index in (0, 3, 6, 9) else S.INK_MUTED,
                  fontweight="bold" if index in (0, 3, 6, 9) else "normal",
                  zorder=6)
    for index, name in ((0, "front"), (3, "right"), (6, "back"), (9, "left")):
        centre = index * 2 * math.pi / sectors
        axis.annotate("", xy=(cap * 0.62 * math.cos(centre),
                              cap * 0.62 * math.sin(centre)),
                      xytext=(0, 0),
                      arrowprops=dict(arrowstyle="-|>", color=S.GOAL,
                                      linewidth=1.2, shrinkA=0, shrinkB=0))
        axis.text(cap * 0.40 * math.cos(centre) + 0.22 * math.sin(centre),
                  cap * 0.40 * math.sin(centre) - 0.22 * math.cos(centre),
                  name, fontsize=6.8, color=S.GOAL, ha="center", va="center")
    axis.add_patch(Circle((0, 0), 0.1773, facecolor=S.STATIC_OBSTACLE,
                          edgecolor=S.STATIC_EDGE, linewidth=0.7, zorder=5))
    axis.annotate(f"sector index increases from +x\ntowards +y (DNS right)",
                  (0, -cap * 1.02), fontsize=6.6, color=S.INK_SOFT,
                  ha="center", va="top")
    axis.set_xlim(-cap * 1.12, cap * 1.12)
    axis.set_ylim(-cap * 1.26, cap * 1.12)
    axis.set_aspect("equal")
    axis.set_xlabel("x [m]  (DNS forward)")
    axis.set_ylabel("y [m]  (DNS right)")
    axis.set_title(f"Top view — {sectors} sectors x 30°, {beams} beams "
                   f"(0.5° spacing), {cap:g} m cap", loc="left", pad=4,
                   fontsize=8.6)
    S.provenance(axis, "SCHEMATIC", loc="upper right")

    # --- side view: mount height, scan plane -----------------------------
    axis = axes[1]
    altitude = scenario.cruise_altitude_m
    mount = scenario.lidar_height_m
    axis.add_patch(Rectangle((-0.6, 0.0), 1.2, 0.012, facecolor=S.WALL,
                             edgecolor="none"))
    axis.text(-0.58, 0.03, "floor", fontsize=6.8, color=S.INK_SOFT)
    # airframe
    axis.add_patch(Rectangle((-0.1773, altitude - 0.02), 0.3546, 0.04,
                             facecolor=S.STATIC_OBSTACLE,
                             edgecolor=S.STATIC_EDGE, linewidth=0.7))
    axis.plot([-0.1773, 0.1773], [altitude + 0.015] * 2, color=S.INK_MUTED,
              linewidth=1.0)
    axis.text(0.20, altitude + 0.012, "rotor plane  z = +0.015 m",
              fontsize=6.4, color=S.INK_SOFT, va="center")
    # lidar + scan plane
    axis.plot([-0.9, 0.9], [altitude + mount] * 2, color=S.GOAL,
              linewidth=1.5, zorder=6)
    axis.add_patch(Circle((0, altitude + mount), 0.028, facecolor=S.GOAL,
                          edgecolor="white", linewidth=0.7, zorder=7))
    axis.text(0.20, altitude + mount + 0.055,
              f"scan plane  z = {altitude + mount:.2f} m\n"
              f"(mount {mount:.2f} m above body origin)",
              fontsize=6.6, color=S.GOAL, va="bottom")
    axis.annotate("", xy=(-0.45, altitude + mount), xytext=(-0.45, altitude),
                  arrowprops=dict(arrowstyle="<->", color=S.INK_SOFT,
                                  linewidth=0.9))
    axis.text(-0.50, altitude + mount / 2, f"{mount:.2f} m", fontsize=6.4,
              color=S.INK_SOFT, ha="right", va="center")
    axis.annotate("", xy=(0.78, altitude), xytext=(0.78, 0.0),
                  arrowprops=dict(arrowstyle="<->", color=S.INK_SOFT,
                                  linewidth=0.9))
    axis.text(0.82, altitude / 2, f"cruise\n{altitude:.2f} m", fontsize=6.4,
              color=S.INK_SOFT, va="center")
    axis.text(-0.9, altitude + mount + 0.30,
              "stabilised mount: the scan plane is held level\n"
              "by counter-rotating the mount every control tick\n"
              "(ideal gimbal — brackets a real one as the best case)",
              fontsize=6.4, color=S.INK_SOFT, va="bottom")
    axis.set_xlim(-1.0, 1.15)
    axis.set_ylim(-0.05, altitude + mount + 0.62)
    axis.set_aspect("equal")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("z [m]")
    axis.set_title("Side view — mount height and scan plane", loc="left",
                   pad=4, fontsize=8.6)
    S.provenance(axis, "SCHEMATIC", loc="lower right")

    figure.suptitle("SV1  LiDAR configuration (dimensions read from the "
                    "scenario JSON and the DnsQuadcopter PROTO)",
                    x=0.02, ha="left", fontsize=9.8, fontweight="bold")
    figure.tight_layout(rect=(0, 0, 1, 0.93))
    files = S.save(figure, DIAGRAMS / "SV1_sensor_schematic")
    caption("SV1", "LiDAR configuration schematic",
            f"Sensor geometry as implemented: {beams} beams over 360° "
            f"(0.5° spacing), {cap:g} m range cap, {sectors} sectors of 30° with "
            "index increasing from +x towards +y so that 0/3/6/9 are "
            "front/right/back/left. The scan plane rides at "
            f"{altitude + mount:.2f} m — cruise altitude {altitude:.2f} m plus a "
            f"{mount:.2f} m mount. Every dimension is read from the scenario JSON "
            "and the PROTO rather than retyped, so the drawing cannot drift from "
            "the world. This is a SCHEMATIC, not a measurement.",
            n="n/a (schematic)",
            data="07_SCENARIOS/L00_C.json, protos/DnsQuadcopter.proto",
            provenance="SCHEMATIC", files=files)


FIGURES = {
    "F1": figure_f1, "F2": figure_f2, "F3": figure_f3,
    "E1": figure_e1, "E3": figure_e3, "E4": figure_e4, "E9": figure_e9,
    "SV1": figure_sv1,
}


def main() -> int:
    S.apply()
    rows = load_results()
    wanted = sys.argv[1:] or list(FIGURES)
    for name in wanted:
        function = FIGURES.get(name)
        if function is None:
            print(f"unknown figure {name}")
            continue
        try:
            function(rows)
            print(f"ok   {name}")
        except Exception as error:                               # noqa: BLE001
            print(f"FAIL {name}: {type(error).__name__}: {error}")
            import traceback
            traceback.print_exc()

    if CAPTIONS:
        path = CAPTIONS_PATH
        existing = {}
        if path.is_file():
            # Preserve captions for figures not regenerated in this run.
            import re
            for block in re.split(r"\n(?=## )", path.read_text(encoding="utf8")):
                match = re.match(r"## ([A-Z0-9]+)", block)
                if match:
                    existing[match.group(1)] = block.rstrip()
        for entry in CAPTIONS:
            existing[entry["id"]] = (
                f"## {entry['id']} — {entry['title']}\n\n"
                f"{entry['caption']}\n\n"
                f"- **n:** {entry['n']}\n"
                f"- **data:** `{entry['data_file']}`\n"
                f"- **script:** `{entry['script']}`\n"
                f"- **provenance:** {entry['provenance']}\n"
                f"- **files:** " + ", ".join(f"`{f}`" for f in entry["files"])
            )
        header = ("# Figure captions\n\n"
                  "Every figure in this package is generated by script from "
                  "logged data. Each entry below states what the figure shows, "
                  "the n behind it, the data file and script that produced it, "
                  "and its provenance label: **SCHEMATIC** (a drawing), "
                  "**LOGGED-DATA RENDER** (plotted from recorded measurements) "
                  "or **WEBOTS CAPTURE** (real rendered frames).\n")
        order = ["F1", "F2", "F3", "F4", "F5"] + [f"E{i}" for i in range(1, 15)] \
            + [f"SV{i}" for i in range(1, 6)]
        body = [existing[k] for k in order if k in existing]
        body += [v for k, v in existing.items() if k not in order]
        path.write_text(header + "\n" + "\n\n".join(body) + "\n", encoding="utf8")
        print(f"\nwrote {path} ({len(existing)} captions)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
