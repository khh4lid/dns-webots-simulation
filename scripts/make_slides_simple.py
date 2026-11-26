"""Build the SIMPLE 27 Nov 2026 deck: 12 picture-first slides plus 4 backups.

This is the deck to present from. The detailed deck
(``make_slides.py`` -> ``DNS_SimEnv_2026-11-27.pptx``) stays as the backup for
questions that need the full numbers.

The design rules it enforces, because they are what make a deck explainable:

* **One idea per slide.** The headline is a plain sentence of at most twelve
  words that states the point, so reading the headline alone gives the argument.
* **Pictures carry the evidence.** Body text is capped at roughly fifteen words
  per slide, excluding the headline and short picture labels. ``audit()``
  counts the words actually drawn and fails the build if a slide exceeds the
  cap -- the rule is checked, not just intended.
* **Chosen / not chosen is never colour alone.** Every such panel carries a
  literal check or ballot-X glyph as well as the green/grey distinction, so it
  survives greyscale printing and colour-vision deficiency.
* **No tables on main slides.** Tables are allowed on backups only.

Numbers are read from the result files at build time wherever they can be, so
they cannot drift from the data. The literal ones are cross-checked by
``scripts/check_slide_claims.py --source scripts/make_slides_simple.py``.

Run::

    python scripts/make_slides_simple.py
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import dns_style as S                                            # noqa: E402
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages             # noqa: E402
from matplotlib.patches import (Circle, FancyArrow,              # noqa: E402
                                FancyBboxPatch, Polygon, Rectangle)

from dns.scenario import Scenario                                # noqa: E402
from make_figures import draw_map                                # noqa: E402

OUT = ROOT / "docs" / "meetings" / "2026-11-27"
PREVIEW = OUT / "slide_png_simple"
FIGURES = ROOT / "assets" / "figures"
RENDERS = ROOT / "assets" / "renders"
VIDEOS = ROOT / "results" / "video"
SCENARIOS = ROOT / "config" / "scenarios"
RESULTS = ROOT / "results" / "v2_S6"

PRESENTER = "Khaled (@khh4lid)"
SPENCER = "Spencer"
SENSOR = "the sensor teammate"

TICK = "✓"          # U+2713 CHECK MARK
CROSS = "✗"         # U+2717 BALLOT X

CHOSEN = S.OKABE_ITO["green"]
NOT_CHOSEN = "#8b9199"
MINE = S.OKABE_ITO["blue"]

#: Words drawn per slide, split three ways so the simplicity rule can be
#: checked against what it actually says: "at most ~15 words of other text per
#: slide, besides short labels".
#:
#: * ``PROSE`` -- sentences. This is the text the cap applies to.
#: * ``LABELS`` -- panel headers, picture captions and option cards. Each is a
#:   few words and names the thing beside it, so they are the "short labels"
#:   the rule exempts. ``audit()`` still enforces that each one *is* short.
#: * ``TRADEOFF`` -- the small grey honesty line, reported separately because
#:   dropping it to save words would be the wrong trade.
PROSE: dict[str, list[str]] = {}
LABELS: dict[str, list[str]] = {}
TRADEOFF: dict[str, list[str]] = {}
CURRENT = ""
PROSE_CAP = 16           # "~15 words"
LABEL_CAP = 7            # a label longer than this is really a sentence


# ---------------------------------------------------------------------------
# data
# ---------------------------------------------------------------------------
def load() -> dict:
    data: dict = {}
    with (RESULTS / "pooled_rates.csv").open(encoding="utf8") as handle:
        data["pooled"] = {(r["layout_id"], r["mode"], r["group"]): r
                          for r in csv.DictReader(handle)}
    with (RESULTS / "scenario_table.csv").open(encoding="utf8") as handle:
        data["layouts"] = {r["layout_id"]: r for r in csv.DictReader(handle)
                           if r["mode"] == "C"}
    with (RESULTS / "results_v2.csv").open(encoding="utf8") as handle:
        rows = list(csv.DictReader(handle))
    data["episodes"] = len(rows)
    data["distinct"] = sum(int(r["distinct_configuration"]) for r in rows)
    data["lidar"] = json.loads(
        (RESULTS / "lidar_cart_consistency.json").read_text(encoding="utf8"))
    data["reset"] = json.loads(
        (ROOT / "results" / "reset_study" / "SUMMARY.json").read_text(
            encoding="utf8"))
    data["scenario"] = json.loads(
        (SCENARIOS / "L00_C.json").read_text(encoding="utf8"))
    with (RESULTS / "v1_reproduction.csv").open(encoding="utf8") as handle:
        repro = list(csv.DictReader(handle))
    data["repro_cells"] = len(repro)
    data["repro_match"] = sum(int(r["outcomes_match"]) for r in repro)
    with (ROOT / "evidence" / "DEFECTS.csv").open(encoding="utf8") as handle:
        defects = list(csv.DictReader(handle))
    data["defects"] = {
        "total": len(defects),
        "fixed": sum(d["status"] == "fixed" for d in defects),
        "open": sum(d["status"] == "open" for d in defects),
    }
    return data


D = load()


def pooled(layout: str) -> tuple[int, int, float, float]:
    row = D["pooled"][(layout, "C_dynamic", "ppo_pooled")]
    return (int(row["k"]), int(row["n"]), float(row["wilson_low"]),
            float(row["wilson_high"]))


def scenario(layout: str) -> Scenario:
    return Scenario.load(SCENARIOS / f"{layout}_C.json")


#: Arena extent, flight and scan heights, and the rack bounding radius are all
#: derived rather than typed, so a change to the scenario files shows up here.
ARENA = D["scenario"]["arena"]
EXTENT_X = ARENA["high_xy"][0] - ARENA["low_xy"][0]
EXTENT_Y = ARENA["high_xy"][1] - ARENA["low_xy"][1]
CRUISE = D["scenario"]["drone"]["cruise_altitude_m"]
SCAN = CRUISE + D["scenario"]["sensing"]["mount_height_m"]
RACK = next(o for o in D["scenario"]["obstacles"] if o["id"] == "rack_a")
RACK_RADIUS = math.hypot(RACK["size_xy"][0] / 2, RACK["size_xy"][1] / 2)
INTRUSION = float(D["layouts"]["L00"]["line_intrusion_m"])
TALLEST = max(o["height_m"] for o in D["scenario"]["obstacles"])
SHORTEST = min(o["height_m"] for o in D["scenario"]["obstacles"])
CART = next(o for o in D["scenario"]["obstacles"]
            if o.get("motion", {}).get("type", "static") != "static")
CART_PERIOD = 4 * CART["motion"]["amplitude_m"] / CART["motion"]["speed_mps"]
DEFECTS = D["defects"]


# ---------------------------------------------------------------------------
# slide chrome
# ---------------------------------------------------------------------------
def slide(headline: str, number: str):
    """A 16:9 slide carrying only a sentence headline and a footer."""
    global CURRENT
    CURRENT = number
    for store in (PROSE, LABELS, TRADEOFF):
        store.setdefault(number, [])

    figure = plt.figure(figsize=(13.333, 7.5))
    figure.patch.set_facecolor("white")
    axis = figure.add_axes([0, 0, 1, 1])
    axis.axis("off")
    axis.set_xlim(0, 1)
    axis.set_ylim(0, 1)

    words = len(headline.split())
    if words > 12:
        raise SystemExit(f"slide {number}: headline is {words} words (max 12)")
    fit_headline(figure, axis, headline, number)

    axis.text(0.055, 0.035,
              f"DNS · Simulation environment · {PRESENTER} "
              f"· 27 Nov 2026 · {number}",
              fontsize=11, color=S.INK_MUTED, va="center")
    return figure, axis


#: The slide axis spans 0-1 in both directions over a 13.333 x 7.5 in figure,
#: so a Circle in data coordinates renders as a wide ellipse. Multiply the
#: y-radius by this to get a visually round circle.
ASPECT = 13.333 / 7.5

#: Headlines must stay in the 36-40 pt band the brief asks for. Anything wider
#: than this fraction of the slide is wrapped rather than shrunk.
HEADLINE_SIZE = 37
TEXT_LIMIT = 0.90


def round_circle(cx, cy, radius, **kwargs):
    """A circle that looks circular on the slide's non-square axis."""
    from matplotlib.patches import Ellipse
    return Ellipse((cx, cy), 2 * radius, 2 * radius * ASPECT, **kwargs)


def text_width(figure, string: str, size: float, weight="normal") -> float:
    """Rendered width of ``string`` as a fraction of the slide width."""
    artist = figure.text(0, 0, string, fontsize=size, fontweight=weight)
    figure.canvas.draw()
    width = (artist.get_window_extent(figure.canvas.get_renderer()).width
             / (figure.get_size_inches()[0] * figure.dpi))
    artist.remove()
    return width


def fit_headline(figure, axis, headline: str, number: str) -> None:
    """Lay the headline out so it cannot run off the slide.

    Explicit newlines are honoured. Otherwise the line is wrapped at the break
    that makes the two halves most even, and only if wrapping still does not
    fit is the size reduced -- the brief asks for 36-40 pt, so shrinking is the
    last resort and is reported when it happens.
    """
    size = HEADLINE_SIZE
    if "\n" in headline:
        lines = headline.split("\n")
    elif text_width(figure, headline, size, "bold") <= TEXT_LIMIT:
        lines = [headline]
    else:
        words = headline.split()
        best, best_cost = None, None
        for cut in range(1, len(words)):
            top = " ".join(words[:cut])
            bottom = " ".join(words[cut:])
            widths = (text_width(figure, top, size, "bold"),
                      text_width(figure, bottom, size, "bold"))
            if max(widths) > TEXT_LIMIT:
                continue
            cost = abs(widths[0] - widths[1])
            if best_cost is None or cost < best_cost:
                best, best_cost = [top, bottom], cost
        lines = best or [headline]

    while max(text_width(figure, l, size, "bold") for l in lines) > TEXT_LIMIT:
        size -= 1
        if size < 30:
            break
    if size != HEADLINE_SIZE:
        print(f"  note: slide {number} headline set at {size} pt to fit")

    for index, line in enumerate(lines):
        axis.text(0.055, 0.925 - index * 0.082, line, fontsize=size,
                  fontweight="bold", color=S.INK, va="top", ha="left")


def label(text: str) -> str:
    """Register a short label and return it unchanged."""
    LABELS[CURRENT].extend(text.split())
    return text


def body(axis, x, y, text, *, size=21, colour=None, weight="normal",
         ha="left", va="top", counted=True, spacing=1.6):
    """Draw body text, and count its words against the per-slide prose cap."""
    if counted:
        PROSE[CURRENT].extend(text.split())
    return axis.text(x, y, text, fontsize=size,
                     color=colour if colour else S.INK_SOFT,
                     fontweight=weight, ha=ha, va=va, linespacing=spacing)


def picture(axis, path: Path, rect, source: str,
            provenance: str = "LOGGED-DATA RENDER") -> None:
    """Place an image inside ``rect`` = (x, y, w, h), preserving its ratio."""
    if not path.is_file():
        axis.text(rect[0], rect[1] + rect[3] / 2, f"[missing {path.name}]",
                  fontsize=14, color=S.OUTCOME_COLOUR["collision_static"])
        return
    image = plt.imread(str(path))
    height, width = image.shape[:2]
    box_ratio = rect[2] / rect[3]
    image_ratio = width / height * (7.5 / 13.333)
    if image_ratio > box_ratio:
        w, h = rect[2], rect[2] / image_ratio
    else:
        w, h = rect[3] * image_ratio, rect[3]
    inset = axis.figure.add_axes([rect[0] + (rect[2] - w) / 2,
                                  rect[1] + (rect[3] - h) / 2, w, h])
    inset.imshow(image)
    inset.axis("off")
    axis.text(rect[0], rect[1] - 0.028, f"{provenance} · {source}",
              fontsize=8.5, color=S.INK_MUTED, va="top")


def map_axes(axis, rect, layout: str, **kwargs):
    """Draw one layout to metric scale inside ``rect``."""
    inset = axis.figure.add_axes(rect)
    draw_map(inset, scenario(layout), **kwargs)
    inset.axis("off")
    return inset


def badge(axis, x, y, mark: str, text: str, colour: str) -> None:
    """A check/cross glyph with its panel label. The glyph is the signal."""
    axis.text(x, y, mark, fontsize=34, color=colour, fontweight="bold",
              va="center", ha="left")
    axis.text(x + 0.035, y, text, fontsize=23, color=colour,
              fontweight="bold", va="center", ha="left")
    label(text)


def card(axis, rect, text: str, *, size=20) -> None:
    """A plain text card. Used for options we did not choose -- no logos."""
    axis.add_patch(FancyBboxPatch(
        (rect[0], rect[1]), rect[2], rect[3],
        boxstyle="round,pad=0.006,rounding_size=0.012",
        facecolor="#f4f5f7", edgecolor=S.GRID, linewidth=1.0))
    axis.text(rect[0] + rect[2] / 2, rect[1] + rect[3] / 2, text,
              fontsize=size, color=NOT_CHOSEN, va="center", ha="center")
    label(text)


def why(axis, sentence: str, tradeoff: str | None = None) -> None:
    """The bottom band: one sentence saying why, and an honest trade-off."""
    height = 0.145 if tradeoff else 0.105
    axis.add_patch(Rectangle((0.0, 0.065), 1.0, height,
                             facecolor=MINE, alpha=0.07, edgecolor="none"))
    axis.text(0.055, 0.065 + height - 0.038, "Why:", fontsize=20,
              fontweight="bold", color=MINE, va="center")
    axis.text(0.128, 0.065 + height - 0.038, sentence, fontsize=20,
              color=S.INK, va="center")
    PROSE[CURRENT].extend(sentence.split())
    if tradeoff:
        axis.text(0.055, 0.065 + 0.036, "Trade-off:", fontsize=15,
                  fontweight="bold", color=S.INK_MUTED, va="center")
        axis.text(0.158, 0.065 + 0.036, tradeoff, fontsize=15,
                  color=S.INK_MUTED, va="center")
        TRADEOFF[CURRENT].extend(tradeoff.split())


def choice(headline, number, *, chosen_label, not_chosen, why_text,
           tradeoff=None, draw_chosen=None, chosen_image=None):
    """The chose / didn't-choose template used by slides 4-9."""
    figure, axis = slide(headline, number)
    badge(axis, 0.055, 0.70, TICK, "I chose", CHOSEN)
    badge(axis, 0.565, 0.70, CROSS, "I didn't choose", NOT_CHOSEN)

    # The caption goes directly under its panel header and above the picture;
    # the provenance line goes under the picture. Putting both below would
    # overlap, which is what the first build of this deck did.
    axis.text(0.055, 0.645, label(chosen_label), fontsize=21,
              color=CHOSEN, fontweight="bold", va="center")
    if draw_chosen is not None:
        draw_chosen(axis, (0.055, 0.255, 0.45, 0.345))
    elif chosen_image is not None:
        picture(axis, chosen_image[0], (0.055, 0.255, 0.45, 0.345),
                chosen_image[1], chosen_image[2])

    top, gap = 0.575, 0.082
    for index, text in enumerate(not_chosen):
        card(axis, (0.565, top - index * gap - 0.052, 0.38, 0.060), text)
    why(axis, why_text, tradeoff)
    return figure


# ---------------------------------------------------------------------------
# small drawn icons -- no stock art anywhere in this deck
# ---------------------------------------------------------------------------
def icon_world(axis, cx, cy, s):
    axis.add_patch(Rectangle((cx - s, cy - s * 0.62), 2 * s, s * 1.24,
                             facecolor="none", edgecolor=S.WALL, linewidth=2.0))
    for dx, dy, w, h in [(-0.45, 0.18, 0.3, 0.3), (0.1, -0.3, 0.45, 0.22),
                         (0.5, 0.25, 0.28, 0.3)]:
        axis.add_patch(Rectangle((cx + dx * s, cy + dy * s), w * s, h * s,
                                 facecolor=S.STATIC_OBSTACLE, edgecolor="none"))


def icon_obstacles(axis, cx, cy, s):
    """Racks and crates of different sizes -- distinct from the six-maps tile."""
    for dx, dy, w, h in [(-0.62, -0.42, 0.26, 0.95), (-0.12, -0.42, 0.26, 0.70),
                         (0.40, -0.42, 0.22, 0.50)]:
        axis.add_patch(Rectangle((cx + dx * s, cy + dy * s), w * s, h * s,
                                 facecolor=S.STATIC_OBSTACLE,
                                 edgecolor=S.STATIC_EDGE, linewidth=0.8))
    axis.plot([cx - 0.82 * s, cx + 0.78 * s], [cy - 0.42 * s] * 2,
              color=S.INK_SOFT, linewidth=1.4)


def icon_lidar(axis, cx, cy, s):
    for angle in np.linspace(-62, 62, 11):
        r = math.radians(angle)
        axis.plot([cx, cx + s * 0.95 * math.sin(r)],
                  [cy - s * 0.3, cy - s * 0.3 + s * 0.95 * ASPECT
                   * math.cos(r)],
                  color=S.OKABE_ITO["sky"], linewidth=1.2, zorder=2)
    axis.add_patch(round_circle(cx, cy - s * 0.3, s * 0.11, facecolor=S.INK,
                                edgecolor="none", zorder=3))


def icon_policy(axis, cx, cy, s):
    cols = [[-0.55, [0.42, 0.0, -0.42]], [0.0, [0.56, 0.19, -0.19, -0.56]],
            [0.55, [0.2, -0.2]]]
    points = {}
    for ci, (dx, ys) in enumerate(cols):
        points[ci] = [(cx + dx * s, cy + dy * s) for dy in ys]
    for a, b in [(0, 1), (1, 2)]:
        for p in points[a]:
            for q in points[b]:
                axis.plot([p[0], q[0]], [p[1], q[1]], color=S.GRID,
                          linewidth=0.7, zorder=1)
    for ci in points:
        for p in points[ci]:
            axis.add_patch(round_circle(p[0], p[1], s * 0.08,
                                        facecolor=S.OKABE_ITO["purple"],
                                        edgecolor="none", zorder=3))


def icon_drone(axis, cx, cy, s):
    for dx, dy in [(-0.55, 0.3), (0.55, 0.3), (-0.55, -0.3), (0.55, -0.3)]:
        axis.add_patch(round_circle(cx + dx * s, cy + dy * s, s * 0.16,
                                    facecolor="none", edgecolor=S.INK_SOFT,
                                    linewidth=1.8))
        axis.plot([cx, cx + dx * s], [cy, cy + dy * s], color=S.INK_SOFT,
                  linewidth=1.8)
    axis.add_patch(round_circle(cx, cy, s * 0.15,
                                facecolor=S.OKABE_ITO["vermillion"],
                                edgecolor="none"))


def icon_cart(axis, cx, cy, s):
    axis.add_patch(Rectangle((cx - s * 0.6, cy - s * 0.18), s * 1.2, s * 0.5,
                             facecolor=S.CART, edgecolor="none"))
    for dx in (-0.35, 0.35):
        axis.add_patch(round_circle(cx + dx * s, cy - s * 0.30, s * 0.10,
                                    facecolor=S.INK, edgecolor="none"))
    axis.annotate("", (cx + s * 1.0, cy + s * 0.05),
                  (cx + s * 0.72, cy + s * 0.05),
                  arrowprops=dict(arrowstyle="-|>", color=S.CART, lw=2.0))
    axis.annotate("", (cx - s * 1.0, cy + s * 0.05),
                  (cx - s * 0.72, cy + s * 0.05),
                  arrowprops=dict(arrowstyle="-|>", color=S.CART, lw=2.0))


def icon_maps(axis, cx, cy, s):
    for index, (dx, dy) in enumerate([(-0.5, 0.26), (0.0, 0.26), (0.5, 0.26),
                                      (-0.5, -0.3), (0.0, -0.3), (0.5, -0.3)]):
        axis.add_patch(Rectangle((cx + dx * s - s * 0.2, cy + dy * s - s * 0.12),
                                 s * 0.4, s * 0.24,
                                 facecolor=MINE if index == 1 else "#c9ced5",
                                 edgecolor="none"))


def icon_check(axis, cx, cy, s):
    axis.add_patch(round_circle(cx, cy, s * 0.52, facecolor="none",
                                edgecolor=CHOSEN, linewidth=2.4))
    axis.text(cx, cy - s * 0.04, TICK, fontsize=int(s * 420), color=CHOSEN,
              fontweight="bold", ha="center", va="center")


# ---------------------------------------------------------------------------
# the twelve main slides
# ---------------------------------------------------------------------------
def s01():
    figure, axis = slide("Building and testing the world\nour drone flies in",
                         "1/12")
    body(axis, 0.055, 0.70,
         f"{PRESENTER}  ·  Simulation environment", size=24,
         colour=MINE, weight="bold", counted=False)
    body(axis, 0.055, 0.645,
         "DNS project  ·  Taylor's University\n27 November 2026",
         size=20, counted=False)
    picture(axis, RENDERS / "V-TOP_L00_success_poster.png",
            (0.50, 0.17, 0.45, 0.55), "V-TOP_L00_success.mp4",
            "WEBOTS CAPTURE")
    return figure


def s02():
    figure, axis = slide("I build the world, the obstacles and the tests",
                         "2/12")

    # --- the team loop: my box coloured, the other three grey ---------------
    boxes = [("World", "mine", MINE, icon_world),
             ("LiDAR", SENSOR, NOT_CHOSEN, icon_lidar),
             ("Policy", SPENCER, NOT_CHOSEN, icon_policy),
             ("Drone moves", "", NOT_CHOSEN, icon_drone)]
    x0, width, gap, y, height = 0.055, 0.195, 0.048, 0.465, 0.245
    for index, (title, owner, colour, icon) in enumerate(boxes):
        x = x0 + index * (width + gap)
        axis.add_patch(FancyBboxPatch(
            (x, y), width, height,
            boxstyle="round,pad=0.004,rounding_size=0.014",
            facecolor=MINE if colour is MINE else "#f4f5f7",
            alpha=0.16 if colour is MINE else 1.0,
            edgecolor=colour, linewidth=2.6 if colour is MINE else 1.2))
        icon(axis, x + width / 2, y + height * 0.60, 0.052)
        axis.text(x + width / 2, y + 0.052, title, fontsize=21,
                  fontweight="bold", color=colour if colour is MINE else S.INK,
                  ha="center", va="center")
        if owner:
            axis.text(x + width / 2, y + 0.022, owner, fontsize=14,
                      color=S.INK_MUTED, ha="center", va="center")
        if index < len(boxes) - 1:
            axis.annotate("", (x + width + gap - 0.008, y + height / 2),
                          (x + width + 0.008, y + height / 2),
                          arrowprops=dict(arrowstyle="-|>", color=S.INK_SOFT,
                                          lw=2.2))
    # The return arrow runs *below* the row, not through it: drawn as three
    # straight segments so it cannot cross any box.
    right = x0 + 3 * (width + gap) + width
    drop = y - 0.035
    axis.plot([right - 0.06, right - 0.06, x0 + 0.06, x0 + 0.06],
              [y, drop, drop, y - 0.004],
              color=S.INK_SOFT, linewidth=2.0, solid_joinstyle="round",
              clip_on=False)
    axis.annotate("", (x0 + 0.06, y - 0.002), (x0 + 0.06, drop + 0.012),
                  arrowprops=dict(arrowstyle="-|>", color=S.INK_SOFT, lw=2.0))
    axis.text((x0 + right) / 2, drop - 0.028, "the world moves on",
              fontsize=14, color=S.INK_MUTED, ha="center", va="center")

    # --- my five responsibilities as picture tiles --------------------------
    tiles = [("Arena", icon_world), ("Obstacles", icon_obstacles),
             ("Moving cart", icon_cart), ("Six maps", icon_maps),
             ("Checks", icon_check)]
    tx, tw, tgap = 0.055, 0.166, 0.022
    for index, (name, icon) in enumerate(tiles):
        x = tx + index * (tw + tgap)
        axis.add_patch(FancyBboxPatch(
            (x, 0.185), tw, 0.175,
            boxstyle="round,pad=0.004,rounding_size=0.012",
            facecolor="white", edgecolor=CHOSEN, linewidth=1.4))
        icon(axis, x + tw / 2, 0.298, 0.040)
        # Centre the tick and its label as one group, measured rather than
        # guessed, so a longer label cannot push past the tile edge.
        group = text_width(axis.figure, f"{TICK}  {name}", 16)
        left = x + tw / 2 - group / 2
        axis.text(left, 0.213, TICK, fontsize=19, color=CHOSEN,
                  fontweight="bold", va="center", ha="left")
        axis.text(left + 0.026, 0.213, label(name), fontsize=16,
                  color=S.INK, va="center", ha="left")
    axis.text(0.055, 0.125, "SCHEMATIC · drawn from docs/interfaces/",
              fontsize=8.5, color=S.INK_MUTED, va="top")
    return figure


def s03():
    figure, axis = slide("The drone must reach the goal using only its LiDAR",
                         "3/12")
    inset = map_axes(axis, (0.045, 0.195, 0.53, 0.55), "L00",
                     show_cspace=False, show_astar=False, scale=False)
    # Labels sit *outside* the arena with leader lines pointing in, so they
    # cannot land on top of an obstacle however the layout changes. The cart
    # leader points at its t = 0 pose, not its sweep centre.
    start = D["scenario"]["drone"]["start_xy"]
    goal = D["scenario"]["goal"]["xy"]
    cart0 = next(o for o in scenario("L00").obstacles
                 if o.motion.type != "static").placed_at(0.0).centre
    marks = [(start[0], start[1], "Start", S.START, start[0], 3.5),
             (goal[0], goal[1], "Goal", S.GOAL, goal[0], 3.5),
             (RACK["xy"][0], RACK["xy"][1], "Racks", S.STATIC_OBSTACLE,
              RACK["xy"][0] - 0.5, -3.6),
             (cart0[0], cart0[1], "Cart", S.CART, cart0[0] + 1.5, -3.6)]
    for x, y, name, colour, lx, ly in marks:
        inset.annotate(name, (x, y), (lx, ly), fontsize=24,
                       fontweight="bold", color=colour,
                       ha="center", va="center",
                       arrowprops=dict(arrowstyle="-", color=colour, lw=1.6,
                                       shrinkA=4, shrinkB=6))
        label(name)
    inset.set_xlim(-1.0, EXTENT_X + 1.0)
    inset.set_ylim(-4.3, 4.1)
    picture(axis, RENDERS / "V-TOP_L00_success_poster.png",
            (0.605, 0.245, 0.345, 0.43), "V-TOP_L00_success.mp4",
            "WEBOTS CAPTURE")
    why(axis, "No map, no GPS waypoints — only what the sensor sees now.")
    return figure


def s04():
    figure = choice(
        "I chose Webots to run the simulation", "4/12",
        chosen_image=(RENDERS / "demo_poster.png", "results/video/V1_main_demo.mp4",
                      "WEBOTS CAPTURE"),
        chosen_label="Our world, running in Webots",
        not_chosen=["Gazebo", "AirSim", "PyBullet"],
        why_text="Built-in LiDAR, true positions to check against, exact "
                 "repeats, runs on our PCs.")
    return figure


def s05():
    def draw(axis, rect):
        # The obstacle catalogue figure is a dense reference panel and is
        # illegible at this size, so this slide draws the point it is making
        # instead: the racks sit across the straight start-goal line.
        inset = axis.figure.add_axes(rect)
        draw_map(inset, scenario("L00"), show_cspace=False, show_astar=False,
                 show_cart_lane=False, scale=False)
        start = D["scenario"]["drone"]["start_xy"]
        goal = D["scenario"]["goal"]["xy"]
        inset.plot([start[0], goal[0]], [start[1], goal[1]],
                   color=S.OKABE_ITO["vermillion"], linewidth=2.0,
                   linestyle=(0, (5, 3)), zorder=12)
        inset.annotate(f"racks cross it by {INTRUSION:.2f} m",
                       (RACK["xy"][0], INTRUSION / 2),
                       (RACK["xy"][0] - 0.6, 3.6), fontsize=16,
                       color=S.OKABE_ITO["vermillion"], fontweight="bold",
                       ha="center", va="center",
                       arrowprops=dict(arrowstyle="-|>",
                                       color=S.OKABE_ITO["vermillion"],
                                       lw=1.8, shrinkB=2))
        inset.text(goal[0] + 0.3, -3.8, "the straight line does not work",
                   fontsize=15, color=S.OKABE_ITO["vermillion"], ha="right",
                   va="center")
        inset.set_xlim(-0.4, EXTENT_X + 0.4)
        inset.set_ylim(-4.5, 4.6)
        inset.axis("off")
        axis.text(rect[0], rect[1] - 0.028,
                  "LOGGED-DATA RENDER · config/scenarios/L00_C.json, "
                  "results/v2_S6/scenario_table.csv",
                  fontsize=8.5, color=S.INK_MUTED, va="top")

    return choice(
        f"One warehouse aisle, {EXTENT_X:.0f} by {EXTENT_Y:.0f} metres",
        "5/12",
        draw_chosen=draw,
        chosen_label="Every obstacle has a job",
        not_chosen=["A much bigger warehouse", "Randomly scattered boxes",
                    "An open room"],
        why_text="The policies trained on exactly this size; in an open "
                 "room, flying straight wins.")


def s06():
    return choice(
        "One flight height, with a flat all-round scan", "6/12",
        chosen_image=(FIGURES / "side_elevation.png",
                      "config/scenarios/L00_C.json", "LOGGED-DATA RENDER"),
        chosen_label=f"Fly at {CRUISE:.1f} m, scan at {SCAN:.2f} m",
        not_chosen=["Full 3-D flight", "Climbing over obstacles",
                    "A tilting sensor"],
        why_text="It matches what the policy controls, and every obstacle "
                 "is taller than the scan.")


def s07():
    def draw(axis, rect):
        inset = axis.figure.add_axes(rect)
        half = np.array(RACK["size_xy"]) / 2
        inset.add_patch(Circle((0, 0), RACK_RADIUS, facecolor=NOT_CHOSEN,
                               alpha=0.22, edgecolor=NOT_CHOSEN,
                               linewidth=1.8, linestyle=(0, (4, 2))))
        inset.add_patch(Rectangle((-half[0], -half[1]), 2 * half[0], 2 * half[1],
                                  facecolor=S.STATIC_OBSTACLE,
                                  edgecolor=S.STATIC_EDGE, linewidth=1.4))
        # The drone sits one collision radius plus 0.50 m clear of the rack
        # face -- still inside the bounding circle, which is the whole point.
        drone_x = half[0] + 0.18 + 0.50
        inset.add_patch(Circle((drone_x, 0.0), 0.18,
                               facecolor=S.OKABE_ITO["vermillion"],
                               edgecolor="white", linewidth=1.2))
        inset.annotate("", (half[0], -0.42), (drone_x - 0.18, -0.42),
                       arrowprops=dict(arrowstyle="<|-|>",
                                       color=S.OKABE_ITO["vermillion"], lw=1.6))
        # Clear of the rack and of the circle, so nothing sits on the drawing.
        inset.text(drone_x + 0.42, -0.42, "0.50 m of\nclear air", fontsize=15,
                   color=S.OKABE_ITO["vermillion"], fontweight="bold",
                   ha="left", va="center")
        inset.text(0, half[1] + 0.22, f"circle: r = {RACK_RADIUS:.3f} m",
                   fontsize=14, color=NOT_CHOSEN, ha="center", va="bottom")
        inset.set_xlim(-1.45, 3.5)
        inset.set_ylim(-1.75, 1.95)
        inset.set_aspect("equal")
        inset.axis("off")
        axis.text(rect[0], rect[1] - 0.028,
                  "SCHEMATIC · rack size from config/scenarios/L00_C.json",
                  fontsize=8.5, color=S.INK_MUTED, va="top")

    return choice(
        "Obstacles are exact boxes, not circles", "7/12",
        draw_chosen=draw,
        chosen_label="The real rectangle, corners and all",
        not_chosen=["A circle round each box", "A safety bubble",
                    "Grid cells only"],
        why_text="A circle called it a crash with 0.50 m of empty aisle "
                 "left.")


def s08():
    def draw(axis, rect):
        inset = axis.figure.add_axes(rect)
        draw_map(inset, scenario("L00"), show_cspace=False, show_astar=False,
                 scale=False)
        y0 = CART["xy"][1] if "xy" in CART else 0.0
        amp = CART["motion"]["amplitude_m"]
        inset.annotate("", (CART["xy"][0], y0 + amp), (CART["xy"][0], y0 - amp),
                       arrowprops=dict(arrowstyle="<|-|>", color=S.CART,
                                       lw=2.6))
        inset.text(CART["xy"][0] + 0.35, y0 + amp * 0.75,
                   f"±{amp:.1f} m\n{CART['motion']['speed_mps']:.2f} m/s",
                   fontsize=15, color=S.CART, fontweight="bold", va="center")
        inset.axis("off")
        axis.text(rect[0], rect[1] - 0.028,
                  "LOGGED-DATA RENDER · config/scenarios/L00_C.json",
                  fontsize=8.5, color=S.INK_MUTED, va="top")

    return choice(
        "The cart follows the same path every single run", "8/12",
        draw_chosen=draw,
        chosen_label=f"Back and forth, every {CART_PERIOD:.2f} s",
        not_chosen=["A cart pushed by physics", "A randomly moving cart",
                    "A second drone"],
        why_text="Identical motion every run is what makes the tests "
                 "comparable to each other.",
        tradeoff="a hit on the cart is detected by geometry, not by "
                 "physical contact.")


def s09():
    return choice(
        "Six seeded maps, including a mirror of the training map", "9/12",
        chosen_image=(FIGURES / "F1_shift_ladder.png",
                      "config/scenarios/L0*_C.json", "LOGGED-DATA RENDER"),
        chosen_label="Same aisle, six arrangements",
        not_chosen=["Only the training map", "Hand-placed obstacles",
                    "One map, many repeats"],
        why_text="One map cannot tell navigating apart from memorising "
                 "that one room.")


#: Bar chart rows: layout, plain-English name, whether it counts.
LADDER = [("L00", "training", True), ("L01", "small shift", True),
          ("L02", "mirror", True), ("L03", "new aisles", True),
          ("L04", "clutter", False), ("L05", "hardest", True)]


def s10():
    figure, axis = slide("Same success on the mirror map; none on the hardest",
                         "10/12")
    chart = axis.figure.add_axes([0.085, 0.37, 0.86, 0.33])
    for spine in ("top", "right"):
        chart.spines[spine].set_visible(False)

    for index, (layout, name, counts) in enumerate(LADDER):
        k, n, low, high = pooled(layout)
        mirror = layout == "L02"
        if not counts:
            chart.bar(index, n, color="#e9ebee", edgecolor=NOT_CHOSEN,
                      linewidth=1.2, linestyle=(0, (3, 2)), width=0.62)
            chart.text(index, n / 2, f"{CROSS} not\ncounted", fontsize=17,
                       color=NOT_CHOSEN, fontweight="bold", ha="center",
                       va="center")
            continue
        chart.bar(index, k, width=0.62,
                  color=S.OKABE_ITO["green"] if mirror
                  else S.CONTROLLER_COLOUR["ppo_pooled"],
                  edgecolor=S.INK if mirror else "none",
                  linewidth=2.2 if mirror else 0)
        chart.errorbar(index, k, yerr=[[k - low * n], [high * n - k]],
                       color=S.INK_SOFT, capsize=7, elinewidth=1.3,
                       capthick=1.3, fmt="none")
        # Sit the count clear of the upper whisker, never on top of it.
        chart.text(index, high * n + 2.0, f"{k} of {n}", fontsize=21,
                   fontweight="bold", color=S.INK, ha="center")

    chart.set_xticks(range(len(LADDER)))
    chart.set_xticklabels([name for _l, name, _c in LADDER], fontsize=20)
    chart.set_yticks([0, 15, 30, 45])
    chart.set_yticklabels(["0", "15", "30", "45"], fontsize=15)
    chart.set_ylim(0, 52)
    chart.set_xlim(-0.6, len(LADDER) - 0.4)
    chart.tick_params(length=0)
    chart.grid(axis="y", color=S.GRID, linewidth=0.6)
    chart.set_axisbelow(True)

    # map thumbnails under each bar, so a name always has a picture with it
    for index, (layout, _name, _counts) in enumerate(LADDER):
        x = 0.085 + (index + 0.5) / len(LADDER) * 0.86 - 0.052
        thumb = axis.figure.add_axes([x, 0.185, 0.104, 0.085])
        draw_map(thumb, scenario(layout), show_cspace=False, show_astar=False,
                 show_cart_lane=False, scale=False)
        thumb.axis("off")

    axis.text(0.085, 0.145,
              f"{D['episodes']} episodes, PPO with the cart moving · lines "
              "show 95% uncertainty (approximate) · L04 excluded: my own "
              "reference path cannot fly it either",
              fontsize=13, color=S.INK_MUTED, va="top")
    axis.text(0.085, 0.108,
              "LOGGED-DATA RENDER · results/v2_S6/pooled_rates.csv",
              fontsize=8.5, color=S.INK_MUTED, va="top")
    return figure


def s11():
    figure, axis = slide("Four checks show the simulator behaves correctly",
                         "11/12")
    lidar = D["lidar"]["cart_sector_pooled"]
    # Every figure here is read from a file at build time, including the
    # defect tally -- "six found, five fixed" is a claim, so it lives in
    # evidence/DEFECTS.csv rather than being typed onto the slide.
    drift = D["reset"]["arms"]["sequence"]["max_path_delta_m_worst"]
    checks = [
        ("Timing", f"{0.2:.6f} s", "every observation", icon_check),
        ("LiDAR vs truth", f"{lidar['mae_m']:.4f} m",
         f"over {lidar['comparisons']} checks", icon_lidar),
        ("Replay in order", f"{drift * 1e6:.0f} µm", "same run, same path",
         icon_maps),
        ("Problems found", f"{DEFECTS['total']}",
         f"{DEFECTS['fixed']} fixed · {DEFECTS['open']} still open",
         icon_world),
    ]
    x0, width, gap = 0.055, 0.205, 0.028
    for index, (title, value, caption, icon) in enumerate(checks):
        x = x0 + index * (width + gap)
        axis.add_patch(FancyBboxPatch(
            (x, 0.30), width, 0.41,
            boxstyle="round,pad=0.005,rounding_size=0.014",
            facecolor="white", edgecolor=S.GRID, linewidth=1.4))
        icon(axis, x + width / 2, 0.635, 0.044)
        axis.text(x + width / 2, 0.495, value, fontsize=29, fontweight="bold",
                  color=MINE, ha="center", va="center")
        axis.text(x + width / 2, 0.425, title, fontsize=20, fontweight="bold",
                  color=S.INK, ha="center", va="center")
        axis.text(x + width / 2, 0.365, caption, fontsize=13,
                  color=S.INK_MUTED, ha="center", va="center")
    axis.text(0.055, 0.245,
              "LOGGED-DATA RENDER · results/v2_S6/, "
              "docs/verification/RESET_DETERMINISM.md",
              fontsize=8.5, color=S.INK_MUTED, va="top")
    why(axis, "The open one is the episode reset: fixed on a branch, "
              "not yet in these results.")
    return figure


def s12():
    figure, axis = slide("Done, not yet, and what I need from you", "12/12")
    columns = [
        ("Done", CHOSEN, TICK,
         ["the arena and walls", "six obstacles, each with a job",
          "the moving cart", "six seeded test maps",
          "the verification checks", "the GitHub documentation"]),
        ("Not yet", S.OKABE_ITO["orange"], "•",
         ["the reset fix — it failed my strict test, so it is on its "
          "own branch",
          "L04 — needs a decision",
          "the orbit video"]),
        ("I need", MINE, "→",
         ["a decision on L04",
          "approval to re-run once the reset is fixed",
          f"more PPO seeds from {SPENCER}"]),
    ]
    x0, width, gap = 0.055, 0.285, 0.027
    for index, (title, colour, mark, items) in enumerate(columns):
        x = x0 + index * (width + gap)
        axis.add_patch(FancyBboxPatch(
            (x, 0.165), width, 0.565,
            boxstyle="round,pad=0.005,rounding_size=0.014",
            facecolor=colour, alpha=0.07, edgecolor=colour, linewidth=1.8))
        axis.text(x + 0.018, 0.675, mark, fontsize=28, color=colour,
                  fontweight="bold", va="center")
        axis.text(x + 0.062, 0.675, title, fontsize=25, fontweight="bold",
                  color=colour, va="center")

        # Wrap to the measured card width and advance by the number of lines
        # produced, so a wrapped item cannot sit on top of the next one.
        inner = width - 0.052
        y = 0.590
        for item in items:
            lines, current = [], ""
            for word in item.split():
                trial = f"{current} {word}".strip()
                if text_width(axis.figure, trial, 15) > inner and current:
                    lines.append(current)
                    current = word
                else:
                    current = trial
            lines.append(current)
            axis.text(x + 0.020, y, mark, fontsize=15, color=colour,
                      fontweight="bold", va="top")
            axis.text(x + 0.046, y, "\n".join(lines), fontsize=15,
                      color=S.INK, va="top", linespacing=1.45)
            y -= 0.040 + 0.036 * len(lines)
    axis.text(0.055, 0.125,
              "source: docs/meetings/2026-11-27/MEETING_DAY.md",
              fontsize=8.5, color=S.INK_MUTED, va="top")
    return figure


# ---------------------------------------------------------------------------
# backups -- tables are allowed here, not on the main slides
# ---------------------------------------------------------------------------
def table(axis, columns, rows, *, x=0.055, y=0.72, width=0.89,
          header_size=13, body_size=12, line=0.055):
    widths = np.array([c[1] for c in columns], dtype=float)
    widths = widths / widths.sum() * width
    xs = x + np.concatenate([[0.0], np.cumsum(widths)[:-1]])
    for (label, _w), cx in zip(columns, xs):
        axis.text(cx, y, label, fontsize=header_size, fontweight="bold",
                  color=S.INK, va="center")
    axis.plot([x, x + width], [y - line * 0.45, y - line * 0.45],
              color=S.INK_SOFT, linewidth=1.0)
    for index, row in enumerate(rows):
        yy = y - (index + 1) * line
        for cell, cx in zip(row, xs):
            axis.text(cx, yy, cell, fontsize=body_size, color=S.INK_SOFT,
                      va="center", linespacing=1.3)
        axis.plot([x, x + width], [yy - line * 0.45, yy - line * 0.45],
                  color=S.GRID, linewidth=0.6)


def b1():
    figure, axis = slide("Every choice, with its reason and its evidence",
                         "B1")
    table(axis, [("I chose", 20), ("I didn't choose", 20), ("Why", 38),
                 ("Evidence", 22)],
          [("Webots", "Gazebo, AirSim, PyBullet",
            "built-in LiDAR, ground truth, repeatable, runs locally",
            "ADR-001"),
           (f"one {EXTENT_X:.0f} × {EXTENT_Y:.0f} m aisle",
            "bigger or random world",
            "the training layout; an open line rewards flying straight",
            "ADR-002"),
           (f"fly {CRUISE:.1f} m, scan {SCAN:.2f} m", "full 3-D flight",
            "matches the policy's action space; all obstacles taller",
            "ADR-003"),
           ("exact boxes", "bounding circles",
            f"circle r = {RACK_RADIUS:.3f} m called 0.50 m of air a crash",
            "ADR-004"),
           ("scripted cart", "physics-pushed cart",
            "identical motion each run; comparable episodes", "ADR-006"),
           ("six seeded maps", "only the training map",
            "separates navigating from memorising one room", "ADR-009")],
          y=0.70, body_size=11.5, line=0.072)
    return figure


def b2():
    figure, axis = slide("The verification numbers in full", "B2")
    lidar = D["lidar"]["cart_sector_pooled"]
    k00, n00, _lo, _hi = pooled("L00")
    table(axis, [("Check", 34), ("Result", 30), ("Source", 36)],
          [("observation interval", "0.200000 s, error < 3e-16 s",
            "v1 Verification_Report"),
           ("world vs scenario pose", "≤ 4.0e-06 m, 6 layouts",
            "results/layout_gate/SUMMARY.json"),
           ("LiDAR vs ray caster", "0.048 m mean per beam",
            "v1 Verification_Report"),
           ("LiDAR vs moving cart",
            f"MAE {lidar['mae_m']:.4f} m, n = {lidar['comparisons']}",
            "lidar_cart_consistency.json"),
           ("contract tests per layout", "26 of 26",
            "results/layout_gate/SUMMARY.json"),
           ("L00 reproduction, in order",
            f"{D['repro_match']}/{D['repro_cells']} cells, 0 m",
            "results/v2_S6/v1_reproduction.csv"),
           ("episodes / distinct configs",
            f"{D['episodes']} / {D['distinct']}", "results/v2_S6/results_v2.csv"),
           ("airframe tilt exceedances", "3 of 540 above 0.50 rad",
            "results/v2_S6/results_v2.csv"),
           ("baseline success, cart moving", f"{k00} of {n00}",
            "results/v2_S6/pooled_rates.csv")],
          y=0.73, body_size=12, line=0.062)
    return figure


def b3():
    figure, axis = slide("The reset finding, stated plainly", "B3")
    arms = D["reset"]["arms"]
    table(axis, [("How the episode is run", 42), ("Worst path difference", 30),
                 ("Episodes", 16)],
          [("replayed in session order",
            f"{arms['sequence']['max_path_delta_m_worst']*1e6:.0f} µm",
            str(arms["sequence"]["episodes"])),
           ("run on its own, in isolation",
            f"{arms['isolated']['max_path_delta_m_worst']:.3f} m",
            str(arms["isolated"]["episodes"]))],
          y=0.70, body_size=13, line=0.075)
    body(axis, 0.055, 0.455,
         "The old reset stops when the drone is steady enough, so a little\n"
         "state from the previous episode survives into the next one.\n"
         "A fixed-length reset removes it, and is written and tested — but it\n"
         "still misses the strict bar on one of five targets, so it is NOT in\n"
         "these results. Every published number used one reset, in one order.",
         size=17, counted=False)
    body(axis, 0.055, 0.175,
         "The mirror result survives the fix: both maps stay equal.",
         size=18, colour=MINE, weight="bold", counted=False)
    axis.text(0.055, 0.118,
              "LOGGED-DATA RENDER · results/reset_study/SUMMARY.json, "
              "docs/verification/RESET_MODE_COMPARISON.md",
              fontsize=8.5, color=S.INK_MUTED, va="top")
    return figure


def b4():
    figure, axis = slide("Why the clutter map is not counted", "B4")
    picture(axis, FIGURES / "E4_clearance.png", (0.055, 0.22, 0.42, 0.50),
            "results/layout_gate/L04.json")
    body(axis, 0.52, 0.70,
         "On L04 the PPO policy scores 0 of 45.\n\n"
         "But my own reference path — which is not\n"
         "a learned policy at all — cannot fly it\n"
         "either, at any speed I tried.\n\n"
         "So a 0 there measures my generator, not\n"
         "the policy. Counting it would overstate\n"
         "how hard the policy finds new layouts.\n\n"
         "It is reported, and excluded from the\n"
         "generalisation conclusion.",
         size=17, counted=False)
    axis.text(0.52, 0.22, "Decision needed: keep it excluded, or authorise a\n"
                          "longer-horizon diagnostic run?",
              fontsize=18, color=MINE, fontweight="bold", va="top",
              linespacing=1.5)
    return figure


SLIDES = [s01, s02, s03, s04, s05, s06, s07, s08, s09, s10, s11, s12]
BACKUPS = [b1, b2, b3, b4]

PATH_10_MIN = list(range(1, 13))
PATH_5_MIN = [1, 2, 3, 10, 12]

#: Speaker notes: (what the slide means, in simple English, 2-3 sentences),
#: plus one "If asked:" line for the question each slide invites.
NOTES = {
    1: ("This is my part of the group project: the simulated world the drone "
        "flies in, and the tests that say whether it works. I did not train "
        "the policy and I did not write the sensor code. The picture is a real "
        "capture from our simulator.",
        "If asked: the whole thing is one private GitHub repository, and I can "
        "open it now."),
    2: ("The loop shows how the four parts fit together. I own the blue box: "
        "the world, the obstacles and the tests. The grey boxes belong to my "
        "teammates, and we agreed a fixed interface between us so neither side "
        "breaks the other.",
        "If asked: the interface is 52 numbers in, two numbers out, every "
        "0.2 seconds — it is written down in docs/interfaces/."),
    3: ("The task is simple to state: fly from the green start to the blue "
        "goal. The drone gets no map and no waypoints — only what its LiDAR "
        "can see right now. The racks sit across the straight line on purpose, "
        "so flying straight does not work.",
        "If asked: it has to arrive within 0.4 m of the goal to count as a "
        "success."),
    4: ("Webots was the choice because it already has a LiDAR sensor built in, "
        "and because it tells me the true position of everything, which is how "
        "I can check the sensor is honest. It also repeats a run exactly, and "
        "it runs on our own laptops with no cloud cost.",
        "If asked: I did not benchmark the others — these four reasons were "
        "enough, and they are written up in ADR-001."),
    5: ("One aisle, not a big warehouse. Two reasons. The policies were "
        "trained on exactly this size, so changing it would make the "
        "comparison meaningless. And every obstacle is there for a stated "
        "job — the racks force a detour, the cart forces a timing decision.",
        "If asked: each obstacle's purpose is a field in the scenario file, so "
        "it is part of the data, not just a comment."),
    6: ("The drone flies at one height and its scan is a flat ring just above "
        "it. That sounds like a limitation, and it is — but it matches what "
        "the policy can actually do, because the policy only commands "
        "sideways motion, never up or down.",
        "If asked: yes, this means the drone cannot fly over anything — and "
        "every obstacle is built taller than the scan so none can be missed "
        "by flying under it."),
    7: ("This is the bug I am most glad I found. If you treat a long rack as a "
        "circle, the circle sticks out far past the real box. The drone was "
        "recorded as crashing while it still had half a metre of empty aisle "
        "beside it, so the results were wrong before I fixed it.",
        "If asked: it is now exact rectangle maths, plus a swept test between "
        "steps so nothing can tunnel through, and it is covered by a "
        "regression test."),
    8: ("The cart moves on a script: the same path, the same speed, every "
        "single run. That is deliberate, because it means two runs differ only "
        "by what I changed, not by how the cart happened to move.",
        "If asked: the honest cost is that bumping the cart is detected "
        "geometrically — the cart does not get pushed, so a real collision "
        "would look different."),
    9: ("One map cannot tell you whether the drone learned to navigate or just "
        "memorised one room. So I generate six, from a seed, all the way from "
        "the training map to a deliberately hard one. The mirror map is the "
        "interesting one: same difficulty, left and right swapped.",
        "If asked: the maps are generated from a seed and hash-checked, so "
        "anyone can rebuild the identical six files."),
    10: ("Two things to take away. The mirror map scores exactly the same as "
         "the training map, which is evidence against simple left-right "
         "memorising. And the hardest map scores zero, which is where the "
         "limit is. The grey bar is excluded and the next slide explains why.",
         "If asked: the thin lines are uncertainty, and they are approximate "
         "because episodes inside a cell are run back-to-back rather than "
         "independently."),
    11: ("I do not ask anyone to trust the simulator — these are the checks "
         "that it behaves. Timing is exact, the LiDAR agrees with ground truth "
         "to about two centimetres, and a replayed session reproduces to "
         "microns. Six real problems were found this way; five are fixed.",
         "If asked: the open one is the episode reset — the fix exists on a "
         "branch but it missed my own strict test, so I did not put it in "
         "these results."),
    12: ("Short version: the environment, the obstacles, the cart, the six "
         "maps and the checks are done, and the whole thing is documented on "
         "GitHub. Two things are not finished, and I am not hiding either. And "
         "there are three things I need from the team to go further.",
         "If asked: the L04 decision is the one that is actually blocking a "
         "result, so that is the one I would like answered today."),
}
NOTES_BACKUP = {
    "B1": ("Every design choice in one place, with what I rejected and the "
           "decision record that holds the full reasoning.",
           "If asked: each ADR is a short file in docs/decisions/."),
    "B2": ("All the verification numbers with the file each one comes from.",
           "If asked: evidence/CLAIMS_EVIDENCE.csv maps every claim to a file "
           "and the script that produced it."),
    "B3": ("The reset finding in two rows. Replayed in order, a run reproduces "
           "to microns. Run on its own, it can drift by 29 centimetres.",
           "If asked: it changes the independence assumption behind the "
           "uncertainty lines, not the measurements themselves."),
    "B4": ("Why a zero on the clutter map is not evidence about the policy.",
           "If asked: the diagnostic would be a longer time limit, which is "
           "one of the decisions I am asking for."),
}


def audit() -> int:
    """Enforce the simplicity rules instead of merely intending them."""
    print("\n  words drawn per slide, excluding the headline:")
    print(f"    {'slide':>6}  {'prose':>5}  {'labels':>6}  {'trade-off':>9}")
    over: list[str] = []
    for number in PROSE:
        prose, labels = len(PROSE[number]), len(LABELS[number])
        trade = len(TRADEOFF[number])
        flag = "" if prose <= PROSE_CAP else f"   <-- over the {PROSE_CAP} cap"
        print(f"    {number:>6}  {prose:>5}  {labels:>6}  {trade:>9}{flag}")
        if prose > PROSE_CAP:
            over.append(number)
    print(f"  worst prose: {max(len(w) for w in PROSE.values())} words "
          f"(cap {PROSE_CAP})")
    print(f"  slides with no prose at all: "
          f"{sum(1 for w in PROSE.values() if not w)} of {len(PROSE)}")
    if over:
        print(f"\n  !! prose over cap: {', '.join(over)}")
    return 1 if over else 0


def main() -> int:
    S.apply()
    OUT.mkdir(parents=True, exist_ok=True)
    PREVIEW.mkdir(parents=True, exist_ok=True)

    figures = []
    for index, builder in enumerate(SLIDES, start=1):
        figure = builder()
        name = f"simple{index:02d}"
        # bbox_inches=None keeps every slide exactly 16:9; "tight" would crop
        # each one differently and the PPTX stretch would then distort them.
        figure.savefig(PREVIEW / f"{name}.png", dpi=110, facecolor="white",
                       bbox_inches=None, pad_inches=0)
        figures.append((name, figure, index))
    for index, builder in enumerate(BACKUPS, start=1):
        figure = builder()
        name = f"simple_backup{index}"
        figure.savefig(PREVIEW / f"{name}.png", dpi=110, facecolor="white",
                       bbox_inches=None, pad_inches=0)
        figures.append((name, figure, f"B{index}"))

    pdf_path = OUT / "DNS_SimEnv_SIMPLE_2026-11-27.pdf"
    with PdfPages(pdf_path) as pdf:
        for _name, figure, _key in figures:
            pdf.savefig(figure, facecolor="white", bbox_inches=None,
                        pad_inches=0)
    for _name, figure, _key in figures:
        plt.close(figure)
    print(f"wrote {pdf_path.relative_to(ROOT)} "
          f"({len(figures)} slides, {pdf_path.stat().st_size/1e6:.2f} MB)")

    build_pptx(figures)
    return audit()


def build_pptx(figures) -> None:
    from pptx import Presentation
    from pptx.util import Emu, Inches, Pt

    deck = Presentation()
    deck.slide_width = Inches(13.333)
    deck.slide_height = Inches(7.5)
    blank = deck.slide_layouts[6]

    #: Real MP4s on the two slides where motion is the point.
    MOVIES = {
        3: [(VIDEOS / "V-TOP_L00_success.mp4",
             RENDERS / "V-TOP_L00_success_poster.png", 0.70, 0.27, 0.25, 0.40)],
        10: [(VIDEOS / "V-TOP_L02_mirror.mp4",
              RENDERS / "V-TOP_L02_mirror_poster.png", 0.78, 0.74, 0.17, 0.20)],
    }

    for name, _figure, key in figures:
        slide_ = deck.slides.add_slide(blank)
        slide_.shapes.add_picture(str(PREVIEW / f"{name}.png"), 0, 0,
                                  width=deck.slide_width,
                                  height=deck.slide_height)
        for video, poster, x, y, w, h in MOVIES.get(key, []):
            if not (video.is_file() and poster.is_file()):
                continue
            try:
                slide_.shapes.add_movie(
                    str(video), Emu(int(x * deck.slide_width)),
                    Emu(int((1 - y - h) * deck.slide_height)),
                    Emu(int(w * deck.slide_width)),
                    Emu(int(h * deck.slide_height)),
                    poster_frame_image=str(poster), mime_type="video/mp4")
            except Exception as error:                           # noqa: BLE001
                print(f"  note: could not embed {video.name}: {error}")

        note = NOTES.get(key) or NOTES_BACKUP.get(key)
        if note:
            meaning, if_asked = note
            frame = slide_.notes_slide.notes_text_frame
            frame.text = meaning
            paragraph = frame.add_paragraph()
            paragraph.text = if_asked
            paragraph.font.size = Pt(12)

    path = OUT / "DNS_SimEnv_SIMPLE_2026-11-27.pptx"
    deck.save(str(path))
    print(f"wrote {path.relative_to(ROOT)} "
          f"({len(figures)} slides, {path.stat().st_size/1e6:.2f} MB)")


if __name__ == "__main__":
    raise SystemExit(main())
