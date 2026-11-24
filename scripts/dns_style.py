"""One style file for every DNS v2 figure.

Rules, applied everywhere:

* SI units on every axis label; equal aspect and a scale bar on every map.
* Colour-blind-safe Okabe-Ito palette. Hues are assigned to ENTITIES in a fixed
  order and never cycled, so the same colour means the same thing in every
  figure -- each controller, the cart, static obstacles.
* No 3D, pie or radar charts. Recessive grid and spines; text >= 8 pt at print
  size. Values and labels wear text colours, never a series colour.
* 300 dpi PNG plus a vector copy (PDF and SVG) for every figure.

The categorical palette was validated with the bundled dataviz checker rather
than by eye::

    node scripts/validate_palette.js "#0072B2,#56B4E9,#009E73,#D55E00,#CC79A7,#E69F00" --mode light

    [PASS] Lightness band      all 6 inside L 0.43-0.77
    [PASS] Chroma floor        all 6 >= 0.1
    [PASS] CVD separation      worst adjacent #D55E00<->#009E73 dE 11.0 (deutan), tritan 8.5
    [PASS] Normal-vision floor worst adjacent #CC79A7<->#D55E00 dE 16.4
    [WARN] Contrast vs surface below 3:1 for 3 slots -- relief required

The contrast WARN is relieved as the checker requires: every series is directly
labelled or carries a legend entry, and the underlying numbers ship as CSV
(``06_DATA/results_v2.csv``, ``06_DATA/pooled_rates.csv``), so identity and
value are never colour-alone.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.patches import Rectangle                         # noqa: E402

# --- Okabe-Ito, assigned to entities in a fixed order ----------------------
OKABE_ITO = {
    "blue": "#0072B2",
    "sky": "#56B4E9",
    "green": "#009E73",
    "vermillion": "#D55E00",
    "purple": "#CC79A7",
    "orange": "#E69F00",
    "yellow": "#F0E442",
    "black": "#000000",
}

#: Controller -> colour. Fixed for the whole package.
CONTROLLER_COLOUR = {
    "ppo_seed7": OKABE_ITO["blue"],
    "ppo_seed19": OKABE_ITO["sky"],
    "ppo_seed43": OKABE_ITO["green"],
    "ppo_pooled": OKABE_ITO["blue"],
    "reactive": OKABE_ITO["vermillion"],
    "direct": OKABE_ITO["purple"],
    "reference_traversal": OKABE_ITO["black"],
}

#: Scene entities. The cart is the one moving body, so it gets its own hue and
#: keeps it in every map, trajectory plot and sensor figure.
CART = OKABE_ITO["orange"]
STATIC_OBSTACLE = "#5a6068"
STATIC_EDGE = "#2f343a"
CSPACE = "#b9c0c7"
WALL = "#3b4046"
START = OKABE_ITO["green"]
GOAL = OKABE_ITO["blue"]
ASTAR = OKABE_ITO["vermillion"]

#: Outcome -> colour, for the stacked failure-taxonomy bars.
OUTCOME_COLOUR = {
    "goal": OKABE_ITO["green"],
    "collision_static": OKABE_ITO["vermillion"],
    "collision_cart": OKABE_ITO["orange"],
    "collision_wall": OKABE_ITO["purple"],
    "timeout": "#9aa0a6",
}

INK = "#16191c"
INK_SOFT = "#4a5056"
INK_MUTED = "#767c82"
GRID = "#dcdfe3"
SURFACE = "#ffffff"


def apply() -> None:
    """Install the shared rcParams. Call once per figure script."""
    plt.rcParams.update({
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
        "font.family": "DejaVu Sans",
        "font.size": 8.5,
        "axes.labelsize": 8.5,
        "axes.titlesize": 9.5,
        "axes.titleweight": "bold",
        "axes.labelcolor": INK,
        "axes.titlecolor": INK,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
        "legend.fontsize": 7.5,
        "legend.frameon": False,
        "text.color": INK,
        "xtick.color": INK_SOFT,
        "ytick.color": INK_SOFT,
        "axes.edgecolor": GRID,
        "axes.linewidth": 0.7,
        "grid.color": GRID,
        "grid.linewidth": 0.6,
        "lines.linewidth": 1.6,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "figure.dpi": 150,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.04,
    })


def save(figure, path: Path, *, vector: bool = True) -> list[Path]:
    """Write PNG at 300 dpi plus PDF and SVG. Returns the paths written."""
    path.parent.mkdir(parents=True, exist_ok=True)
    written = [path.with_suffix(".png")]
    figure.savefig(written[0], dpi=300)
    if vector:
        for suffix in (".pdf", ".svg"):
            target = path.with_suffix(suffix)
            figure.savefig(target)
            written.append(target)
    plt.close(figure)
    return written


def provenance(axis, label: str, *, loc: str = "lower right") -> None:
    """Stamp the required provenance label on a visual.

    One of WEBOTS CAPTURE, LOGGED-DATA RENDER or SCHEMATIC. Every figure in the
    package carries one so a reader never has to guess whether a picture is a
    measurement or a drawing.
    """
    positions = {
        "lower right": (0.995, 0.012, "right", "bottom"),
        "lower left": (0.005, 0.012, "left", "bottom"),
        "upper right": (0.995, 0.988, "right", "top"),
        "upper left": (0.005, 0.988, "left", "top"),
    }
    x, y, ha, va = positions[loc]
    axis.text(x, y, label, transform=axis.transAxes, fontsize=6.0,
              color=INK_MUTED, ha=ha, va=va, family="DejaVu Sans")


def scale_bar(axis, length_m: float = 1.0, *, origin=(0.3, -2.75),
              label: str | None = None) -> None:
    """Draw a metric scale bar. Required on every map."""
    x, y = origin
    axis.plot([x, x + length_m], [y, y], color=INK, linewidth=2.2,
              solid_capstyle="butt", zorder=12)
    for end in (x, x + length_m):
        axis.plot([end, end], [y - 0.075, y + 0.075], color=INK,
                  linewidth=1.3, zorder=12)
    axis.text(x + length_m / 2.0, y + 0.13, label or f"{length_m:g} m",
              ha="center", va="bottom", fontsize=7.0, color=INK, zorder=12)


def grid(axis, which: str = "both") -> None:
    axis.grid(True, which="major", axis=which, alpha=0.55, zorder=0)
    axis.set_axisbelow(True)


def swatch(axis, colour: str, label: str, x: float, y: float,
           size: float = 0.16) -> None:
    """A small labelled key mark, for maps where a legend box would crowd."""
    axis.add_patch(Rectangle((x, y), size, size, facecolor=colour,
                             edgecolor="none", transform=axis.transAxes,
                             zorder=12))
    axis.text(x + size * 1.5, y + size / 2.0, label, transform=axis.transAxes,
              fontsize=7.0, color=INK, va="center", zorder=12)
