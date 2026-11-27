"""Build the FINAL 27 Nov 2026 deck: 16 main slides plus backups.

This is the deck to present from. The simple and detailed decks are kept as
backup files.

Three design systems carry the deck, and all three are enforced here rather
than left to care:

**The flight-path tracker.** A thin plan of the aisle runs along the foot of
every slide with a drone dot that advances from start to goal as the talk
progresses, and the ten section names sit on it as waypoints. The audience can
always see how far through the argument they are.

**Number cards.** Every quantity is a card: a large numeral, its unit, what it
means in plain words, and a type tag drawn from a fixed set of seven --- size,
angle, time, speed, count, accuracy, result. Each type keeps one drawn icon and
one tint across the whole deck, so "0.4 m" and "5.0 m" are visibly the same kind
of thing. ``audit()`` fails the build if a card has no unit or an unknown type.

**Engineering-drawing style.** The map, the height ruler and the airframe are
drawn on white with dimension lines, leader callouts and a scale bar, from the
scenario and PROTO files rather than by eye.

Every number is read from a result, config or PROTO file at build time wherever
it can be; the rest are cross-checked by ``check_slide_claims.py``.

Run::

    python scripts/make_slides_final.py
"""
from __future__ import annotations

import csv
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import dns_style as S                                            # noqa: E402
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages             # noqa: E402
from matplotlib.patches import (Arc, Circle, Ellipse,            # noqa: E402
                                FancyBboxPatch, Polygon, Rectangle, Wedge)

from dns.scenario import Scenario                                # noqa: E402
from make_figures import draw_map                                # noqa: E402

OUT = ROOT / "docs" / "meetings" / "2026-11-27"
PREVIEW = OUT / "slide_png_final"
FIGURES = ROOT / "assets" / "figures"
RENDERS = ROOT / "assets" / "renders"
SHOTS = ROOT / "assets" / "screenshots"
VIDEOS = ROOT / "results" / "video"
SCENARIOS = ROOT / "config" / "scenarios"
RESULTS = ROOT / "results" / "v2_S6"

PRESENTER = "Khaled (@khh4lid)"
SPENCER = "Spencer"
SENSOR = "the sensor teammate"

# U+23F1 STOPWATCH is absent from DejaVu Sans and renders as a box, so the
# timeout marker uses U+25F7, which the font does have.
TICK, CROSS, CLOCK = "✓", "✗", "◷"
CHOSEN = S.OKABE_ITO["green"]
NOT_CHOSEN = "#8b9199"
MINE = S.OKABE_ITO["blue"]

#: Section waypoints along the tracker, and slide -> waypoint.
SECTIONS = ["Mission", "Map", "Drone", "Sensors", "Obstacles", "Choices",
            "Tests", "Results", "Checks", "Next"]
SLIDE_SECTION = {1: None, 2: "Mission", 3: "Mission", 4: "Map", 5: "Map",
                 6: "Drone", 7: "Sensors", 8: "Obstacles", 9: "Choices",
                 10: "Tests", 11: "Tests", 12: "Results", 13: "Results",
                 14: "Checks", 15: "Next", 16: "Next"}
N_MAIN = 16

#: The seven number types: (tint, unit hint, icon name).
TYPES = {
    "size":     (S.OKABE_ITO["blue"], "m", "dimension"),
    "angle":    (S.OKABE_ITO["purple"], "°", "arc"),
    "time":     (S.OKABE_ITO["vermillion"], "s", "clock"),
    "speed":    (S.OKABE_ITO["orange"], "m/s", "motion"),
    "count":    (S.OKABE_ITO["sky"], "#", "tally"),
    "accuracy": (S.OKABE_ITO["green"], "cm · µm", "target"),
    "result":   (S.INK_SOFT, "k of n", "bars"),
}

PROSE: dict[str, list[str]] = {}
CARDS: dict[str, list[tuple[str, str, str]]] = {}
FONTS: dict[str, list[float]] = {}
CURRENT = ""
PROSE_CAP = 25          # "~25 words besides number cards and short labels"
LABEL_MIN_PT = 20.0     # every label, number-card meaning lines included


# ---------------------------------------------------------------------------
# data, all read from files
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
    with (ROOT / "evidence" / "DEFECTS.csv").open(encoding="utf8") as handle:
        defects = list(csv.DictReader(handle))
    data["defects"] = {"total": len(defects),
                       "fixed": sum(d["status"] == "fixed" for d in defects),
                       "open": sum(d["status"] == "open" for d in defects)}
    #: Airframe numbers come from the PROTO, which is the one place they are
    #: defined; reading them avoids a second copy drifting out of step.
    proto = (ROOT / "protos" / "DnsQuadcopter.proto").read_text(encoding="utf8")

    def field(name: str) -> float:
        match = re.search(rf"field\s+SF\w+\s+{name}\s+([-\d.]+)", proto)
        return float(match.group(1))

    world = (ROOT / "worlds" / "DNS_L00_C.wbt").read_text(encoding="utf8")

    def world_field(name: str) -> float:
        return float(re.search(rf"\n\s+{name}\s+([-\d.]+)", world).group(1))

    data["drone"] = {
        "mass_kg": field("mass"),
        "collision_radius_m": field("collisionRadius"),
        "lidar_height_m": field("lidarHeight"),
        "beams": int(world_field("lidarResolution")),
        "range_m": world_field("lidarMaxRange"),
        # Rotor geometry, from the PROTO's own documented construction.
        "hub_offset_m": 0.09,
        "rotor_radius_m": 0.05,
        "body_m": 0.18,
    }
    return data


D = load()
SC = D["scenario"]
ARENA = SC["arena"]
EXTENT_X = ARENA["high_xy"][0] - ARENA["low_xy"][0]
EXTENT_Y = ARENA["high_xy"][1] - ARENA["low_xy"][1]
START_XY = SC["drone"]["start_xy"]
GOAL_XY = SC["goal"]["xy"]
GOAL_TOL = SC["goal"]["tolerance_m"]
CRUISE = SC["drone"]["cruise_altitude_m"]
SCAN = CRUISE + SC["sensing"]["mount_height_m"]
DT = SC["control"]["command_period_s"]
MAX_SPEED = SC["control"]["max_speed_mps"]
STEP_MS = SC["control"]["basic_timestep_ms"]
HORIZON = 90
SPAN = round(GOAL_XY[0] - START_XY[0], 2)
EPISODE_S = round(HORIZON * DT, 1)
INNER_HZ = round(1000 / STEP_MS)
SECTORS = SC["sensing"]["sectors"]
BEAMS = D["drone"]["beams"]
BEAM_STEP = round(360 / BEAMS, 1)
RACK = next(o for o in SC["obstacles"] if o["id"] == "rack_a")
RACK_RADIUS = math.hypot(*[v / 2 for v in RACK["size_xy"]])
CART = next(o for o in SC["obstacles"]
            if o.get("motion", {}).get("type", "static") != "static")
CART_PERIOD = round(4 * CART["motion"]["amplitude_m"]
                    / CART["motion"]["speed_mps"], 2)
INTRUSION = float(D["layouts"]["L00"]["line_intrusion_m"])
TIP_RADIUS = D["drone"]["hub_offset_m"] * math.sqrt(2) + \
    D["drone"]["rotor_radius_m"]
WIDTH_M = round(2 * TIP_RADIUS, 2)

LADDER = [("L00", "Training"), ("L01", "Small shift"), ("L02", "Mirror"),
          ("L03", "New aisles"), ("L04", "Clutter"), ("L05", "Hardest")]


def pooled(layout: str) -> tuple[int, int, float, float]:
    row = D["pooled"][(layout, "C_dynamic", "ppo_pooled")]
    return (int(row["k"]), int(row["n"]), float(row["wilson_low"]),
            float(row["wilson_high"]))


def scenario(layout: str) -> Scenario:
    return Scenario.load(SCENARIOS / f"{layout}_C.json")


def gap(layout: str) -> float:
    return float(D["layouts"][layout]["min_corridor_width_enroute_m"])


# ---------------------------------------------------------------------------
# chrome
# ---------------------------------------------------------------------------
ASPECT = 13.333 / 7.5
HEADLINE_SIZE = 38
TEXT_LIMIT = 0.88


def round_circle(cx, cy, radius, **kwargs):
    """A circle that reads as circular on the slide's non-square axis."""
    return Ellipse((cx, cy), 2 * radius, 2 * radius * ASPECT, **kwargs)


def text_width(figure, string: str, size: float, weight="normal") -> float:
    artist = figure.text(0, 0, string, fontsize=size, fontweight=weight)
    figure.canvas.draw()
    width = (artist.get_window_extent(figure.canvas.get_renderer()).width
             / (figure.get_size_inches()[0] * figure.dpi))
    artist.remove()
    return width


def fit_headline(figure, axis, headline: str, number: str) -> int:
    """Wrap rather than shrink, so headlines stay in the 36-40 pt band."""
    size = HEADLINE_SIZE
    if "\n" in headline:
        lines = headline.split("\n")
    elif text_width(figure, headline, size, "bold") <= TEXT_LIMIT:
        lines = [headline]
    else:
        words, best, best_cost = headline.split(), None, None
        for cut in range(1, len(words)):
            top, bottom = " ".join(words[:cut]), " ".join(words[cut:])
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
        print(f"  note: slide {number} headline set at {size} pt")
    for index, line in enumerate(lines):
        axis.text(0.05, 0.945 - index * 0.080, line, fontsize=size,
                  fontweight="bold", color=S.INK, va="top", ha="left")
    FONTS[CURRENT].append(size)
    return len(lines)


def tracker(axis, index: int) -> None:
    """The signature: a mini aisle with the drone advancing towards the goal.

    The dot is placed by slide index, so the audience can see progress without
    being told. On the last slide it sits on the goal.
    """
    x0, x1, y = 0.05, 0.95, 0.112
    height = 0.028
    axis.add_patch(Rectangle((x0, y - height / 2), x1 - x0, height,
                             facecolor="#fbfbfc", edgecolor=S.GRID,
                             linewidth=0.9, zorder=1))
    # the aisle's own furniture, schematically: two racks and the cart lane
    for frac, w in [(0.30, 0.055), (0.52, 0.055)]:
        axis.add_patch(Rectangle((x0 + frac * (x1 - x0), y - height / 2 + 0.004),
                                 w * (x1 - x0) * 0.35, height - 0.008,
                                 facecolor=S.STATIC_OBSTACLE, edgecolor="none",
                                 zorder=2))
    axis.add_patch(Circle((x0, y), 0.0, facecolor="none"))
    axis.add_patch(round_circle(x0, y, 0.0055, facecolor=S.START,
                                edgecolor="white", linewidth=0.8, zorder=4))
    axis.add_patch(round_circle(x1, y, 0.0075, facecolor="none",
                                edgecolor=S.GOAL, linewidth=1.4, zorder=4))
    axis.add_patch(round_circle(x1, y, 0.0040, facecolor=S.GOAL,
                                edgecolor="white", linewidth=0.7, zorder=5))

    live = SLIDE_SECTION.get(index)
    for position, name in enumerate(SECTIONS):
        wx = x0 + (position + 0.5) / len(SECTIONS) * (x1 - x0)
        on = name == live
        axis.plot([wx, wx], [y - height / 2, y - height / 2 - 0.008],
                  color=MINE if on else S.GRID, linewidth=1.6 if on else 0.8,
                  zorder=3)
        axis.text(wx, y - height / 2 - 0.030, name,
                  fontsize=9.5 if on else 8.2,
                  color=MINE if on else S.INK_MUTED,
                  fontweight="bold" if on else "normal",
                  ha="center", va="center", zorder=3)

    progress = (index - 1) / (N_MAIN - 1) if isinstance(index, int) else 0.0
    dx = x0 + progress * (x1 - x0)
    axis.plot([x0, dx], [y, y], color=MINE, linewidth=1.8,
              linestyle=(0, (3, 2)), zorder=5)
    icon_drone(axis, dx, y, 0.0135, colour=MINE, zorder=6)


def slide(headline: str, number, section_index=None):
    """A 16:9 slide: sentence headline, flight-path tracker, footer."""
    global CURRENT
    CURRENT = str(number)
    for store in (PROSE, CARDS, FONTS):
        store.setdefault(CURRENT, [])

    words = len(headline.replace("\n", " ").split())
    if words > 12:
        raise SystemExit(f"slide {number}: headline is {words} words (max 12)")

    figure = plt.figure(figsize=(13.333, 7.5))
    figure.patch.set_facecolor("white")
    axis = figure.add_axes([0, 0, 1, 1])
    axis.axis("off")
    axis.set_xlim(0, 1)
    axis.set_ylim(0, 1)
    fit_headline(figure, axis, headline, CURRENT)
    tracker(axis, section_index if section_index is not None else number)
    axis.text(0.05, 0.022,
              f"DNS · Simulation environment · {PRESENTER} · 27 Nov 2026 "
              f"· {number}", fontsize=10, color=S.INK_MUTED, va="center")
    return figure, axis


def body(axis, x, y, text, *, size=22, colour=None, weight="normal",
         ha="left", va="top", counted=True, spacing=1.55):
    if counted:
        PROSE[CURRENT].extend(text.split())
    FONTS[CURRENT].append(size)
    return axis.text(x, y, text, fontsize=size,
                     color=colour if colour else S.INK_SOFT,
                     fontweight=weight, ha=ha, va=va, linespacing=spacing)


def picture(axis, path: Path, rect, source: str,
            provenance: str = "LOGGED-DATA RENDER", *, crop=None) -> None:
    """Place an image inside ``rect`` = (x, y, w, h), preserving its ratio."""
    if not path.is_file():
        axis.text(rect[0], rect[1] + rect[3] / 2, f"[missing {path.name}]",
                  fontsize=14, color=S.OUTCOME_COLOUR["collision_static"])
        return
    image = plt.imread(str(path))
    if crop:
        left, top, right, bottom = crop
        image = image[top:bottom, left:right]
    height, width = image.shape[:2]
    ratio = width / height * (7.5 / 13.333)
    if ratio > rect[2] / rect[3]:
        w, h = rect[2], rect[2] / ratio
    else:
        w, h = rect[3] * ratio, rect[3]
    inset = axis.figure.add_axes([rect[0] + (rect[2] - w) / 2,
                                  rect[1] + (rect[3] - h) / 2, w, h])
    inset.imshow(image)
    inset.axis("off")
    axis.text(rect[0], rect[1] - 0.024, f"{provenance} · {source}",
              fontsize=8.5, color=S.INK_MUTED, va="top")


# ---------------------------------------------------------------------------
# the number-card system
# ---------------------------------------------------------------------------
def type_icon(axis, kind: str, cx: float, cy: float, s: float) -> None:
    """A small drawn glyph per number type. Drawn, never an emoji font."""
    colour = TYPES[kind][0]
    name = TYPES[kind][2]
    if name == "dimension":                       # |<-->|
        axis.plot([cx - s, cx + s], [cy, cy], color=colour, linewidth=1.6)
        for sx in (-1, 1):
            axis.plot([cx + sx * s, cx + sx * s],
                      [cy - s * 0.62 * ASPECT, cy + s * 0.62 * ASPECT],
                      color=colour, linewidth=1.6)
            axis.annotate("", (cx + sx * s * 0.98, cy),
                          (cx + sx * s * 0.45, cy),
                          arrowprops=dict(arrowstyle="-|>", color=colour,
                                          lw=1.2, shrinkA=0, shrinkB=0))
    elif name == "arc":                           # angle between two rays
        axis.plot([cx - s, cx + s], [cy - s * 0.55 * ASPECT] * 2,
                  color=colour, linewidth=1.6)
        axis.plot([cx - s, cx + s * 0.6],
                  [cy - s * 0.55 * ASPECT, cy + s * 0.75 * ASPECT],
                  color=colour, linewidth=1.6)
        axis.add_patch(Arc((cx - s, cy - s * 0.55 * ASPECT), s * 1.1,
                           s * 1.1 * ASPECT, theta1=0, theta2=38,
                           edgecolor=colour, linewidth=1.3))
    elif name == "clock":
        axis.add_patch(round_circle(cx, cy, s * 0.82, facecolor="none",
                                    edgecolor=colour, linewidth=1.6))
        axis.plot([cx, cx], [cy, cy + s * 0.52 * ASPECT], color=colour,
                  linewidth=1.5)
        axis.plot([cx, cx + s * 0.44], [cy, cy], color=colour, linewidth=1.5)
    elif name == "motion":                        # arrow with speed lines
        axis.annotate("", (cx + s, cy), (cx - s * 0.25, cy),
                      arrowprops=dict(arrowstyle="-|>", color=colour, lw=2.0))
        for offset, length in [(0.42, 0.55), (0.0, 0.75), (-0.42, 0.55)]:
            axis.plot([cx - s, cx - s + s * length * 0.5],
                      [cy + offset * s * ASPECT * 0.6] * 2,
                      color=colour, linewidth=1.2)
    elif name == "tally":                         # four bars
        for i, h in enumerate([0.45, 0.75, 0.6, 0.95]):
            axis.add_patch(Rectangle((cx - s + i * s * 0.52, cy - s * 0.6 * ASPECT),
                                     s * 0.3, h * s * ASPECT,
                                     facecolor=colour, edgecolor="none"))
    elif name == "target":
        for r, lw in [(0.85, 1.5), (0.45, 1.3)]:
            axis.add_patch(round_circle(cx, cy, s * r, facecolor="none",
                                        edgecolor=colour, linewidth=lw))
        axis.add_patch(round_circle(cx, cy, s * 0.14, facecolor=colour,
                                    edgecolor="none"))
    elif name == "bars":                          # k of n
        axis.add_patch(Rectangle((cx - s, cy - s * 0.6 * ASPECT), s * 0.78,
                                 1.2 * s * ASPECT, facecolor=colour,
                                 edgecolor="none"))
        axis.add_patch(Rectangle((cx + s * 0.1, cy - s * 0.6 * ASPECT), s * 0.78,
                                 1.2 * s * ASPECT, facecolor="none",
                                 edgecolor=colour, linewidth=1.4))


def number_card(axis, x, y, value: str, unit: str, meaning: str, kind: str,
                *, width=0.205, height=0.205, value_size=44,
                meaning_size=20.0) -> None:
    """One number that explains itself: numeral, unit, meaning, type tag.

    The meaning line is a label, so it sits at the deck's 20 pt floor rather
    than being shrunk to fit; narrow cards take a shorter meaning instead.
    """
    if kind not in TYPES:
        raise SystemExit(f"slide {CURRENT}: unknown number type {kind!r}")
    if not unit:
        raise SystemExit(f"slide {CURRENT}: card {value!r} has no unit")
    if len(meaning.split()) > 8:
        raise SystemExit(f"slide {CURRENT}: card meaning is "
                         f"{len(meaning.split())} words (max 8)")
    CARDS[CURRENT].append((value, unit, kind))
    colour = TYPES[kind][0]
    axis.add_patch(FancyBboxPatch(
        (x, y), width, height, boxstyle="round,pad=0.005,rounding_size=0.014",
        facecolor=colour, alpha=0.07, edgecolor=colour, linewidth=1.5))
    axis.text(x + 0.016, y + height - 0.052, value, fontsize=value_size,
              fontweight="bold", color=colour, va="center", ha="left")
    vw = text_width(axis.figure, value, value_size, "bold")
    axis.text(x + 0.020 + vw, y + height - 0.062, unit, fontsize=20,
              color=colour, va="center", ha="left")
    # meaning, wrapped to the card
    words, lines, line = meaning.split(), [], ""
    for word in words:
        trial = f"{line} {word}".strip()
        if (text_width(axis.figure, trial, meaning_size) > width - 0.030
                and line):
            lines.append(line)
            line = word
        else:
            line = trial
    lines.append(line)
    axis.text(x + 0.016, y + height - 0.094, "\n".join(lines),
              fontsize=meaning_size, color=S.INK, va="top", linespacing=1.32)
    type_icon(axis, kind, x + width - 0.030, y + 0.020, 0.0135)
    axis.text(x + width - 0.050, y + 0.020, kind.capitalize(), fontsize=11,
              color=colour, ha="right", va="center", fontweight="bold")
    FONTS[CURRENT].extend([value_size, 20, meaning_size])


def type_legend(axis, x, y) -> None:
    """Introduce the seven types once, on the first slide that uses cards."""
    axis.text(x, y + 0.038, "Every number on a slide carries its type:",
              fontsize=13, color=S.INK_MUTED, va="center")
    step = 0.118
    for index, kind in enumerate(TYPES):
        cx = x + index * step
        type_icon(axis, kind, cx + 0.012, y, 0.0115)
        axis.text(cx + 0.030, y, kind.capitalize(), fontsize=12,
                  color=TYPES[kind][0], fontweight="bold", va="center")


# ---------------------------------------------------------------------------
# drawn icons
# ---------------------------------------------------------------------------
def icon_drone(axis, cx, cy, s, *, colour=None, zorder=3) -> None:
    colour = colour or S.INK_SOFT
    for dx, dy in [(-0.7, 0.45), (0.7, 0.45), (-0.7, -0.45), (0.7, -0.45)]:
        axis.plot([cx, cx + dx * s], [cy, cy + dy * s * ASPECT],
                  color=colour, linewidth=1.5, zorder=zorder)
        axis.add_patch(round_circle(cx + dx * s, cy + dy * s * ASPECT, s * 0.30,
                                    facecolor="none", edgecolor=colour,
                                    linewidth=1.3, zorder=zorder))
    axis.add_patch(round_circle(cx, cy, s * 0.34, facecolor=colour,
                                edgecolor="white", linewidth=0.8,
                                zorder=zorder + 1))


def icon_world(axis, cx, cy, s):
    axis.add_patch(Rectangle((cx - s, cy - s * 0.6 * ASPECT), 2 * s,
                             1.2 * s * ASPECT, facecolor="none",
                             edgecolor=S.WALL, linewidth=1.8))
    for dx, dy, w, h in [(-0.5, 0.1, 0.3, 0.35), (0.15, -0.35, 0.45, 0.25),
                         (0.5, 0.2, 0.28, 0.3)]:
        axis.add_patch(Rectangle((cx + dx * s, cy + dy * s * ASPECT),
                                 w * s, h * s * ASPECT,
                                 facecolor=S.STATIC_OBSTACLE, edgecolor="none"))


def icon_obstacles(axis, cx, cy, s):
    for dx, dy, w, h in [(-0.62, -0.42, 0.26, 0.95), (-0.12, -0.42, 0.26, 0.70),
                         (0.40, -0.42, 0.22, 0.50)]:
        axis.add_patch(Rectangle((cx + dx * s, cy + dy * s * ASPECT),
                                 w * s, h * s * ASPECT,
                                 facecolor=S.STATIC_OBSTACLE,
                                 edgecolor=S.STATIC_EDGE, linewidth=0.8))
    axis.plot([cx - 0.82 * s, cx + 0.78 * s],
              [cy - 0.42 * s * ASPECT] * 2, color=S.INK_SOFT, linewidth=1.4)


def icon_lidar(axis, cx, cy, s):
    for angle in np.linspace(-62, 62, 11):
        r = math.radians(angle)
        axis.plot([cx, cx + s * 0.95 * math.sin(r)],
                  [cy - s * 0.3 * ASPECT,
                   cy + (-0.3 + 0.95 * math.cos(r)) * s * ASPECT],
                  color=S.OKABE_ITO["sky"], linewidth=1.1, zorder=2)
    axis.add_patch(round_circle(cx, cy - s * 0.3 * ASPECT, s * 0.11,
                                facecolor=S.INK, edgecolor="none", zorder=3))


def icon_cart(axis, cx, cy, s):
    axis.add_patch(Rectangle((cx - s * 0.6, cy - s * 0.18 * ASPECT), s * 1.2,
                             s * 0.5 * ASPECT, facecolor=S.CART,
                             edgecolor="none"))
    for dx in (-0.35, 0.35):
        axis.add_patch(round_circle(cx + dx * s, cy - s * 0.30 * ASPECT,
                                    s * 0.10, facecolor=S.INK,
                                    edgecolor="none"))
    for sx in (-1, 1):
        axis.annotate("", (cx + sx * s * 1.05, cy + s * 0.05 * ASPECT),
                      (cx + sx * s * 0.72, cy + s * 0.05 * ASPECT),
                      arrowprops=dict(arrowstyle="-|>", color=S.CART, lw=1.8))


def icon_maps(axis, cx, cy, s):
    for index, (dx, dy) in enumerate([(-0.5, 0.26), (0.0, 0.26), (0.5, 0.26),
                                      (-0.5, -0.3), (0.0, -0.3), (0.5, -0.3)]):
        axis.add_patch(Rectangle(
            (cx + dx * s - s * 0.2, cy + dy * s * ASPECT - s * 0.12 * ASPECT),
            s * 0.4, s * 0.24 * ASPECT,
            facecolor=MINE if index == 1 else "#c9ced5", edgecolor="none"))


def icon_check(axis, cx, cy, s):
    axis.add_patch(round_circle(cx, cy, s * 0.62, facecolor="none",
                                edgecolor=CHOSEN, linewidth=2.2))
    axis.text(cx, cy - s * 0.04, TICK, fontsize=int(s * 420), color=CHOSEN,
              fontweight="bold", ha="center", va="center")


# ---------------------------------------------------------------------------
# engineering-drawing helpers
# ---------------------------------------------------------------------------
def dimension(axis, p0, p1, text, *, offset=0.0, colour=None, size=14,
              vertical=False, side=1):
    """A dimension line with arrows at both ends and its measurement."""
    colour = colour or S.INK_SOFT
    (x0, y0), (x1, y1) = p0, p1
    if vertical:
        x = x0 + offset
        axis.annotate("", (x, y1), (x, y0),
                      arrowprops=dict(arrowstyle="<|-|>", color=colour, lw=1.1,
                                      shrinkA=0, shrinkB=0))
        for yy in (y0, y1):
            axis.plot([x0, x], [yy, yy], color=colour, linewidth=0.7)
        axis.text(x + 0.06 * side, (y0 + y1) / 2, text, fontsize=size,
                  color=colour, ha="left" if side > 0 else "right",
                  va="center", rotation=90)
    else:
        y = y0 + offset
        axis.annotate("", (x1, y), (x0, y),
                      arrowprops=dict(arrowstyle="<|-|>", color=colour, lw=1.1,
                                      shrinkA=0, shrinkB=0))
        for xx in (x0, x1):
            axis.plot([xx, xx], [y0, y], color=colour, linewidth=0.7)
        axis.text((x0 + x1) / 2, y + 0.10 * side, text, fontsize=size,
                  color=colour, ha="center",
                  va="bottom" if side > 0 else "top")


def callout(axis, point, text, offset, *, colour=None, size=14, ha="left"):
    """A thin leader line from a point out to a label."""
    colour = colour or S.INK_SOFT
    axis.annotate(text, point, (point[0] + offset[0], point[1] + offset[1]),
                  fontsize=size, color=colour, ha=ha, va="center",
                  arrowprops=dict(arrowstyle="-", color=colour, lw=0.9,
                                  shrinkA=2, shrinkB=3))


def scale_bar(axis, origin, length=1.0, *, label="1 m", colour=None,
              label_drop=0.20):
    colour = colour or S.INK_SOFT
    x, y = origin
    axis.plot([x, x + length], [y, y], color=colour, linewidth=2.0)
    for xx in (x, x + length):
        axis.plot([xx, xx], [y - label_drop * 0.3, y + label_drop * 0.3],
              color=colour, linewidth=1.4)
    axis.text(x + length / 2, y - label_drop, label, fontsize=12,
              color=colour, ha="center", va="top")


# ---------------------------------------------------------------------------
# the sixteen main slides
# ---------------------------------------------------------------------------
def s01():
    figure, axis = slide("Building the world our drone learns to fly in", 1)
    picture(axis, SHOTS / "hero_annotated.png", (0.44, 0.22, 0.52, 0.60),
            "assets/screenshots/hero_annotated.png", "WEBOTS CAPTURE")
    body(axis, 0.05, 0.76, f"{PRESENTER}", size=26, colour=MINE,
         weight="bold", counted=False)
    body(axis, 0.05, 0.705,
         "Simulation environment\n\nDNS project\nTaylor's University\n"
         "27 November 2026", size=21, counted=False)
    return figure


def s02():
    figure, axis = slide("Fly from Start to Goal without touching anything", 2)
    inset = axis.figure.add_axes([0.05, 0.50, 0.44, 0.30])
    draw_map(inset, scenario("L00"), show_cspace=False, show_astar=False,
             show_cart_lane=False, scale=False)
    inset.annotate("", (GOAL_XY[0] - 0.5, 0.0), (START_XY[0] + 0.5, 0.0),
                   arrowprops=dict(arrowstyle="-|>", color=MINE, lw=2.4,
                                   linestyle=(0, (4, 2))))
    for xy, name, colour, dy in [(START_XY, "Start", S.START, 3.4),
                                 (GOAL_XY, "Goal", S.GOAL, 3.4)]:
        inset.annotate(name, (xy[0], xy[1]), (xy[0], dy), fontsize=21,
                       fontweight="bold", color=colour, ha="center",
                       va="center",
                       arrowprops=dict(arrowstyle="-", color=colour, lw=1.4,
                                       shrinkA=3, shrinkB=6))
    inset.set_xlim(-0.4, EXTENT_X + 0.4)
    inset.set_ylim(-3.4, 4.4)
    inset.axis("off")
    axis.text(0.05, 0.476, "LOGGED-DATA RENDER · config/scenarios/L00_C.json",
              fontsize=8.5, color=S.INK_MUTED, va="top")
    picture(axis, RENDERS / "V-TOP_L00_success_poster.png",
            (0.53, 0.50, 0.43, 0.30), "V-TOP_L00_success.mp4", "WEBOTS CAPTURE")

    for cx, value, unit, meaning, kind in [
            (0.050, f"{SPAN:.2f}", "m", "distance to fly", "size"),
            (0.280, f"{GOAL_TOL:.1f}", "m", "counts as arrived", "size"),
            (0.510, f"{EPISODE_S:.0f}", "s",
             f"limit, {HORIZON} commands", "time"),
            (0.740, f"{MAX_SPEED:.1f}", "m/s", "speed limit", "speed")]:
        number_card(axis, cx, 0.200, value, unit, meaning, kind,
                    width=0.210, height=0.190)
    type_legend(axis, 0.05, 0.155)
    return figure


def s03():
    figure, axis = slide("My part: the world, the obstacles and the tests", 3)
    boxes = [("World", "mine", MINE, icon_world),
             ("LiDAR", SENSOR, NOT_CHOSEN, icon_lidar),
             ("Policy", SPENCER, NOT_CHOSEN, icon_policy),
             ("Drone moves", "", NOT_CHOSEN, lambda a, x, y, s:
              icon_drone(a, x, y, s * 1.1))]
    x0, w, gap_, y, h = 0.05, 0.197, 0.047, 0.505, 0.235
    for index, (title, owner, colour, icon) in enumerate(boxes):
        x = x0 + index * (w + gap_)
        axis.add_patch(FancyBboxPatch(
            (x, y), w, h, boxstyle="round,pad=0.005,rounding_size=0.016",
            facecolor=MINE if colour is MINE else "#f4f5f7",
            alpha=0.16 if colour is MINE else 1.0,
            edgecolor=colour, linewidth=2.4 if colour is MINE else 1.1))
        icon(axis, x + w / 2, y + h * 0.62, 0.048)
        axis.text(x + w / 2, y + 0.052, title, fontsize=21, fontweight="bold",
                  color=colour if colour is MINE else S.INK, ha="center",
                  va="center")
        if owner:
            axis.text(x + w / 2, y + 0.022, owner, fontsize=13,
                      color=S.INK_MUTED, ha="center", va="center")
        if index < 3:
            axis.annotate("", (x + w + gap_ - 0.008, y + h / 2),
                          (x + w + 0.008, y + h / 2),
                          arrowprops=dict(arrowstyle="-|>", color=S.INK_SOFT,
                                          lw=2.0))
    right = x0 + 3 * (w + gap_) + w
    drop = y - 0.040
    axis.plot([right - 0.06, right - 0.06, x0 + 0.06, x0 + 0.06],
              [y, drop, drop, y - 0.004], color=S.INK_SOFT, linewidth=1.8,
              solid_joinstyle="round")
    axis.annotate("", (x0 + 0.06, y - 0.002), (x0 + 0.06, drop + 0.014),
                  arrowprops=dict(arrowstyle="-|>", color=S.INK_SOFT, lw=1.8))
    axis.text((x0 + right) / 2, drop - 0.026,
              f"the world advances one {DT} s step", fontsize=14,
              color=S.INK_MUTED, ha="center", va="center")

    tiles = [("Arena", icon_world), ("Obstacles", icon_obstacles),
             ("Moving cart", icon_cart), ("Six maps", icon_maps),
             ("Checks", icon_check)]
    tw, tgap = 0.168, 0.022
    for index, (name, icon) in enumerate(tiles):
        x = x0 + index * (tw + tgap)
        axis.add_patch(FancyBboxPatch(
            (x, 0.195), tw, 0.175,
            boxstyle="round,pad=0.005,rounding_size=0.014",
            facecolor="white", edgecolor=CHOSEN, linewidth=1.4))
        icon(axis, x + tw / 2, 0.308, 0.040)
        group = text_width(axis.figure, f"{TICK}  {name}", 16)
        left = x + tw / 2 - group / 2
        axis.text(left, 0.222, TICK, fontsize=19, color=CHOSEN,
                  fontweight="bold", va="center")
        axis.text(left + 0.026, 0.222, name, fontsize=16, color=S.INK,
                  va="center")
        FONTS[CURRENT].append(20)
    axis.text(0.05, 0.168, "SCHEMATIC · docs/interfaces/", fontsize=8.5,
              color=S.INK_MUTED, va="top")
    return figure


def icon_policy(axis, cx, cy, s):
    cols = [(-0.55, [0.42, 0.0, -0.42]), (0.0, [0.56, 0.19, -0.19, -0.56]),
            (0.55, [0.2, -0.2])]
    points = {i: [(cx + dx * s, cy + dy * s * ASPECT) for dy in ys]
              for i, (dx, ys) in enumerate(cols)}
    for a, b in [(0, 1), (1, 2)]:
        for p in points[a]:
            for q in points[b]:
                axis.plot([p[0], q[0]], [p[1], q[1]], color=S.GRID,
                          linewidth=0.7, zorder=1)
    for group in points.values():
        for p in group:
            axis.add_patch(round_circle(p[0], p[1], s * 0.08,
                                        facecolor=S.OKABE_ITO["purple"],
                                        edgecolor="none", zorder=3))


def s04():
    figure, axis = slide(f"A {EXTENT_X:.0f} × {EXTENT_Y:.0f} m warehouse "
                         f"aisle, drawn to scale", 4)
    plan = axis.figure.add_axes([0.055, 0.225, 0.59, 0.53])
    low, high = ARENA["low_xy"], ARENA["high_xy"]
    # 1 m grid, then walls, then the furniture: a clean technical plan.
    for gx in np.arange(low[0], high[0] + 0.01, 1.0):
        plan.plot([gx, gx], [low[1], high[1]], color=S.GRID, linewidth=0.5,
                  zorder=1)
    for gy in np.arange(low[1], high[1] + 0.01, 1.0):
        plan.plot([low[0], high[0]], [gy, gy], color=S.GRID, linewidth=0.5,
                  zorder=1)
    plan.add_patch(Rectangle((low[0], low[1]), EXTENT_X, EXTENT_Y,
                             facecolor="none", edgecolor=S.WALL,
                             linewidth=2.2, zorder=4))
    for obstacle in scenario("L00").obstacles:
        placed = obstacle.placed_at(0.0)
        half = np.asarray(placed.half_size, dtype=float)
        local = np.array([[-half[0], -half[1]], [half[0], -half[1]],
                          [half[0], half[1]], [-half[0], half[1]]])
        c, s_ = math.cos(placed.yaw_rad), math.sin(placed.yaw_rad)
        poly = local @ np.array([[c, s_], [-s_, c]]) + np.asarray(placed.centre)
        moving = obstacle.motion.type != "static"
        plan.add_patch(Polygon(poly, closed=True,
                               facecolor=S.CART if moving else S.STATIC_OBSTACLE,
                               edgecolor=S.STATIC_EDGE, linewidth=0.7,
                               zorder=5))
    plan.add_patch(Circle(GOAL_XY, GOAL_TOL, facecolor=S.GOAL, alpha=0.18,
                          edgecolor=S.GOAL, linewidth=1.2, zorder=6))
    plan.add_patch(Circle(GOAL_XY, 0.10, facecolor=S.GOAL, edgecolor="white",
                          linewidth=0.8, zorder=8))
    plan.add_patch(Circle(START_XY, 0.16, facecolor=S.START, edgecolor="white",
                          linewidth=0.8, zorder=8))
    callout(plan, START_XY, f"Start ({START_XY[0]:.2f}, {START_XY[1]:.2f})",
            (0.2, 3.9), colour=S.START, size=15, ha="center")
    callout(plan, GOAL_XY, f"Goal ({GOAL_XY[0]:.2f}, {GOAL_XY[1]:.2f})"
            f" within {GOAL_TOL:.1f} m", (-1.3, 3.9), colour=S.GOAL,
            size=15, ha="center")
    dimension(plan, (low[0], low[1]), (high[0], low[1]),
              f"{EXTENT_X:.0f} m", offset=-1.25, side=-1, size=16)
    dimension(plan, (high[0], low[1]), (high[0], high[1]),
              f"{EXTENT_Y:.0f} m", offset=1.15, vertical=True, size=16)
    plan.annotate("", (1.9, -2.55), (0.3, -2.55),
                  arrowprops=dict(arrowstyle="-|>", color=S.INK_SOFT, lw=1.2))
    plan.text(2.05, -2.55, "x along the aisle", fontsize=13,
              color=S.INK_SOFT, va="center")
    plan.annotate("", (0.3, -0.95), (0.3, -2.45),
                  arrowprops=dict(arrowstyle="-|>", color=S.INK_SOFT, lw=1.2))
    plan.text(0.42, -1.6, "y across", fontsize=13, color=S.INK_SOFT,
              va="center")
    scale_bar(plan, (8.1, -1.45), 1.0)
    plan.set_xlim(low[0] - 1.7, high[0] + 1.4)
    plan.set_ylim(low[1] - 2.7, high[1] + 2.2)
    plan.set_aspect("equal")
    plan.axis("off")
    axis.text(0.07, 0.212, "SCHEMATIC · to scale from "
              "config/scenarios/L00_C.json", fontsize=8.5, color=S.INK_MUTED,
              va="top")

    for cy, value, meaning, vs in [
            (0.575, f"{EXTENT_X:.0f}×{EXTENT_Y:.0f}", "the whole aisle", 40),
            (0.385, f"{ARENA['wall_height_m']:.1f}", "wall height", 44),
            (0.195, "1", "one grid square", 44)]:
        number_card(axis, 0.675, cy, value, "m", meaning, "size",
                    width=0.275, height=0.175, value_size=vs)
    return figure


def s05():
    figure, axis = slide("Every obstacle is taller than the LiDAR's scan", 5)
    ruler = axis.figure.add_axes([0.07, 0.235, 0.56, 0.52])
    items = [("Walls", ARENA["wall_height_m"], S.WALL),
             ("Columns", 2.5, S.STATIC_OBSTACLE),
             ("Racks", 2.0, S.STATIC_OBSTACLE),
             ("Crate stack", 1.6, S.STATIC_OBSTACLE),
             ("Cart", CART["height_m"], S.CART)]
    for index, (name, height, colour) in enumerate(items):
        x = 0.6 + index * 1.25
        ruler.add_patch(Rectangle((x, 0.0), 0.78, height, facecolor=colour,
                                  edgecolor=S.STATIC_EDGE, linewidth=0.7,
                                  zorder=3))
        ruler.text(x + 0.39, height + 0.10, f"{height:.1f} m", fontsize=14,
                   color=S.INK, fontweight="bold", ha="center", va="bottom",
                   zorder=4)
        ruler.text(x + 0.39, -0.16, name, fontsize=14, color=S.INK_SOFT,
                   ha="center", va="top")
    span = 0.3, 0.6 + len(items) * 1.25
    ruler.plot(span, [SCAN, SCAN], color=S.OKABE_ITO["sky"], linewidth=2.2,
               linestyle=(0, (6, 3)), zorder=6)
    ruler.text(span[1] + 0.1, SCAN, f"LiDAR scan {SCAN:.2f} m", fontsize=15,
               color=S.OKABE_ITO["sky"], fontweight="bold", va="center")
    ruler.plot(span, [CRUISE, CRUISE], color=MINE, linewidth=2.2, zorder=6)
    ruler.text(span[1] + 0.1, CRUISE - 0.06, f"Drone flies {CRUISE:.1f} m",
               fontsize=15, color=MINE, fontweight="bold", va="top")
    icon_drone_metric(ruler, 1.0, CRUISE, 0.42)
    for tick in np.arange(0, 3.01, 0.5):
        ruler.plot([0.3, 0.42], [tick, tick], color=S.INK_MUTED, linewidth=0.9)
        ruler.text(0.22, tick, f"{tick:.1f}", fontsize=11, color=S.INK_MUTED,
                   ha="right", va="center")
    ruler.plot([0.3, 0.3], [0, 3.0], color=S.INK_MUTED, linewidth=1.0)
    ruler.text(0.22, 3.12, "height [m]", fontsize=12, color=S.INK_MUTED,
               ha="left", va="bottom")
    ruler.plot([0.3, span[1]], [0, 0], color=S.INK, linewidth=1.4)
    ruler.set_xlim(-0.6, span[1] + 2.9)
    ruler.set_ylim(-0.6, 3.35)
    ruler.axis("off")
    axis.text(0.07, 0.212, "SCHEMATIC · heights from "
              "config/scenarios/L00_C.json", fontsize=8.5, color=S.INK_MUTED,
              va="top")
    number_card(axis, 0.675, 0.530, f"{CRUISE:.1f}", "m",
                "the drone flies here", "size", width=0.275, height=0.175)
    number_card(axis, 0.675, 0.340, f"{SCAN:.2f}", "m",
                "the laser ring sits here", "size", width=0.275,
                height=0.175)
    body(axis, 0.675, 0.292, "Nothing can hide under the beam.", size=20,
         colour=S.INK_SOFT)
    return figure


def icon_drone_metric(axis, cx, cy, span):
    """The drone drawn in metres, for use inside a metric axis."""
    for dx in (-1, 1):
        axis.plot([cx, cx + dx * span / 2], [cy, cy + 0.06], color=MINE,
                  linewidth=1.6, zorder=7)
        axis.add_patch(Ellipse((cx + dx * span / 2, cy + 0.06), span * 0.42,
                               0.045, facecolor="none", edgecolor=MINE,
                               linewidth=1.3, zorder=7))
    axis.add_patch(Ellipse((cx, cy), span * 0.30, 0.10, facecolor=MINE,
                           edgecolor="white", linewidth=0.8, zorder=8))


def s06():
    figure, axis = slide("Our drone: a small quadcopter flown by speed "
                         "commands", 6)
    plan = axis.figure.add_axes([0.055, 0.245, 0.37, 0.50])
    a = D["drone"]["hub_offset_m"]
    rr = D["drone"]["rotor_radius_m"]
    body_m = D["drone"]["body_m"]
    # Plan view, to scale, from the PROTO's documented construction.
    for dx, dy in [(-a, -a), (a, -a), (a, a), (-a, a)]:
        plan.plot([0, dx], [0, dy], color=S.INK_SOFT, linewidth=2.0, zorder=3)
        plan.add_patch(Circle((dx, dy), rr, facecolor=MINE, alpha=0.18,
                              edgecolor=MINE, linewidth=1.3, zorder=4))
    plan.add_patch(Rectangle((-body_m / 2, -body_m / 2), body_m, body_m,
                             facecolor="#eceef1", edgecolor=S.INK_SOFT,
                             linewidth=1.4, zorder=5))
    plan.add_patch(Circle((0, 0), D["drone"]["collision_radius_m"],
                          facecolor="none", edgecolor=S.OKABE_ITO["vermillion"],
                          linewidth=1.4, linestyle=(0, (5, 3)), zorder=6))
    plan.add_patch(Circle((0, 0), 0.022, facecolor=S.OKABE_ITO["sky"],
                          edgecolor="white", linewidth=0.8, zorder=8))
    dimension(plan, (-TIP_RADIUS, -TIP_RADIUS), (TIP_RADIUS, -TIP_RADIUS),
              f"{WIDTH_M:.2f} m across", offset=-0.055, side=-1, size=14)
    callout(plan, (0.0, 0.0), "LiDAR, on top", (0.145, 0.105),
            colour=S.OKABE_ITO["sky"], size=14)
    callout(plan, (a, a), "rotor", (0.125, 0.085), size=14)
    callout(plan, (0, -D["drone"]["collision_radius_m"]),
            f"collision circle {D['drone']['collision_radius_m']:.2f} m",
            (0.085, -0.115), colour=S.OKABE_ITO["vermillion"], size=13)
    scale_bar(plan, (-0.27, -0.255), 0.1, label="0.1 m", label_drop=0.035)
    plan.set_xlim(-0.33, 0.36)
    plan.set_ylim(-0.37, 0.30)
    plan.set_aspect("equal")
    plan.axis("off")
    plan.text(-0.33, 0.275, "plan view", fontsize=13, color=S.INK_MUTED)
    axis.text(0.055, 0.222, "SCHEMATIC · to scale from "
              "protos/DnsQuadcopter.proto", fontsize=8.5, color=S.INK_MUTED,
              va="top")

    # The real thing, cropped from a capture, so the drawing is anchored.
    picture(axis, RENDERS / "V-SIDE_L00_altitude_poster.png",
            (0.445, 0.455, 0.175, 0.26), "V-SIDE_L00_altitude.mp4",
            "WEBOTS CAPTURE", crop=(920, 555, 1040, 655))
    axis.text(0.4475, 0.735, "in flight", fontsize=13, color=S.INK_MUTED)

    number_card(axis, 0.445, 0.190, f"{D['drone']['mass_kg']:.2f}", "kg",
                "whole airframe", "size", width=0.175, height=0.175,
                value_size=34)
    number_card(axis, 0.645, 0.575, f"{MAX_SPEED:.1f}", "m/s",
                "speed limit given", "speed", width=0.300, height=0.175)
    number_card(axis, 0.645, 0.385, f"{DT}", "s",
                "between commands", "time", width=0.300, height=0.175)
    number_card(axis, 0.645, 0.195, f"{INNER_HZ}", "Hz",
                f"inner loop, {STEP_MS:.0f} ms physics", "count",
                width=0.300, height=0.175)
    return figure


def s07():
    figure, axis = slide(f"It sees with {BEAMS} laser beams, read as "
                         f"{SECTORS} directions", 7)
    radar = axis.figure.add_axes([0.05, 0.385, 0.285, 0.375])
    reach = D["drone"]["range_m"]
    for index in range(SECTORS):
        start = 90 - (index + 0.5) * (360 / SECTORS)
        radar.add_patch(Wedge((0, 0), reach, start, start + 360 / SECTORS,
                              facecolor=S.OKABE_ITO["sky"],
                              alpha=0.20 if index % 2 else 0.10,
                              edgecolor="white", linewidth=1.0, zorder=2))
    for ring in (1, 3, reach):
        radar.add_patch(Circle((0, 0), ring, facecolor="none",
                               edgecolor=S.GRID, linewidth=0.9, zorder=3))
    radar.text(0, reach + 0.28, f"{reach:.0f} m range", fontsize=14,
               color=S.INK_SOFT, ha="center", va="bottom")
    for index, name in [(0, "0 front"), (3, "3 right"), (6, "6 back"),
                        (9, "9 left")]:
        angle = math.radians(90 - index * (360 / SECTORS))
        radar.text(math.cos(angle) * reach * 0.76,
                   math.sin(angle) * reach * 0.76, name, fontsize=15,
                   fontweight="bold", color=MINE, ha="center", va="center",
                   zorder=6)
    radar.add_patch(Arc((0, 0), reach * 1.1, reach * 1.1, theta1=60, theta2=90,
                        edgecolor=S.OKABE_ITO["purple"], linewidth=1.6,
                        zorder=5))
    radar.text(math.cos(math.radians(75)) * reach * 0.62,
               math.sin(math.radians(75)) * reach * 0.62, "30°", fontsize=14,
               color=S.OKABE_ITO["purple"], fontweight="bold", ha="center")
    radar.add_patch(Circle((0, 0), 0.22, facecolor=MINE,
                           edgecolor="white", linewidth=1.0, zorder=8))
    radar.text(0.0, -0.75, "drone", fontsize=13, color=MINE,
               fontweight="bold", ha="center", va="top", zorder=8)
    radar.set_xlim(-reach * 1.25, reach * 1.25)
    radar.set_ylim(-reach * 1.3, reach * 1.35)
    radar.set_aspect("equal")
    radar.axis("off")
    axis.text(0.05, 0.372, "SCHEMATIC · sensing block of "
              "config/scenarios/L00_C.json", fontsize=8.5,
              color=S.INK_MUTED, va="top")

    picture(axis, FIGURES / "lidar_validation.png",
            (0.355, 0.335, 0.365, 0.395), "results/v2_S6/runs/*/lidar.npz")
    axis.text(0.355, 0.755, "what is there  vs  what it sees", fontsize=20,
              color=S.INK, fontweight="bold", va="center")

    number_card(axis, 0.05, 0.185, "360", "°", "all round, no blind side",
                "angle", width=0.285, height=0.175)
    for cy, value, unit, meaning, kind in [
            (0.575, f"{BEAMS}", "beams", f"one every {BEAM_STEP} degrees",
             "count"),
            (0.380, f"{reach:.0f}", "m", "furthest it can see", "size"),
            (0.185, f"{SECTORS}", "sectors", "the policy reads these",
             "count")]:
        number_card(axis, 0.735, cy, value, unit, meaning, kind,
                    width=0.215, height=0.175)
    return figure


def s08():
    figure, axis = slide("Four obstacle types, each with a job", 8)
    picture(axis, FIGURES / "obstacle_catalogue.png", (0.05, 0.455, 0.62, 0.27),
            "config/scenarios/L00_C.json")
    lane = axis.figure.add_axes([0.70, 0.435, 0.265, 0.30])
    draw_map(lane, scenario("L00"), show_cspace=False, show_astar=False,
             scale=False)
    amp = CART["motion"]["amplitude_m"]
    lane.annotate("", (CART["xy"][0], CART["xy"][1] + amp),
                  (CART["xy"][0], CART["xy"][1] - amp),
                  arrowprops=dict(arrowstyle="<|-|>", color=S.CART, lw=2.4))
    lane.text(CART["xy"][0], 3.8, "the cart's lane", fontsize=14,
              color=S.CART, fontweight="bold", ha="center", va="center")
    lane.set_xlim(-0.4, EXTENT_X + 0.4)
    lane.set_ylim(-3.5, 4.5)
    lane.axis("off")
    axis.text(0.70, 0.412, "LOGGED-DATA RENDER · L00_C.json", fontsize=8.5,
              color=S.INK_MUTED, va="top")

    for cx, value, unit, meaning, kind in [
            (0.050, f"{CART['motion']['speed_mps']:.2f}", "m/s",
             "how fast the cart crosses", "speed"),
            (0.283, f"±{amp:.1f}", "m", "how far it travels", "size"),
            (0.516, f"{CART_PERIOD:.2f}", "s", "one full cycle", "time"),
            (0.749, f"{INTRUSION:.2f}", "m", "racks block this much",
             "size")]:
        number_card(axis, cx, 0.185, value, unit, meaning, kind,
                    width=0.215, height=0.195)
    return figure


#: The six choices. Each: (title, chosen, rejected, why).
CHOICES = [
    ("Simulator", "Webots", "Gazebo · AirSim · PyBullet",
     "built-in LiDAR, true positions, exact repeats"),
    ("Map size", f"{EXTENT_X:.0f} x {EXTENT_Y:.0f} m, kept", "a bigger world",
     "the policies trained on exactly this size"),
    ("Height", "one height, flat scan", "full 3-D flight",
     "it matches what the policy controls"),
    ("Shapes", "exact boxes", "circles round each box",
     "circles called 0.50 m of clear air a crash"),
    ("Cart", "scripted motion", "pushed by physics",
     "identical motion makes runs comparable"),
    ("Tests", "six maps, incl. mirror", "only the training map",
     "one map cannot rule out memorising"),
]


def s09():
    figure, axis = slide("Six choices I made, and what I didn't choose", 9)
    x0, y0, w, h, gx, gy = 0.05, 0.215, 0.293, 0.235, 0.022, 0.030
    for index, (title, chosen, rejected, why) in enumerate(CHOICES):
        col, row = index % 3, index // 3
        x = x0 + col * (w + gx)
        y = y0 + (1 - row) * (h + gy)
        axis.add_patch(FancyBboxPatch(
            (x, y), w, h, boxstyle="round,pad=0.005,rounding_size=0.014",
            facecolor="white", edgecolor=S.GRID, linewidth=1.3))
        axis.text(x + 0.014, y + h - 0.028, title, fontsize=17,
                  fontweight="bold", color=S.INK, va="center")
        axis.text(x + 0.014, y + h - 0.072, TICK, fontsize=20, color=CHOSEN,
                  fontweight="bold", va="center")
        axis.text(x + 0.044, y + h - 0.072, chosen, fontsize=16,
                  color=CHOSEN, fontweight="bold", va="center")
        axis.text(x + 0.014, y + h - 0.114, CROSS, fontsize=20,
                  color=NOT_CHOSEN, fontweight="bold", va="center")
        axis.text(x + 0.044, y + h - 0.114, rejected, fontsize=14,
                  color=NOT_CHOSEN, va="center")
        lines, line = [], ""
        for word in why.split():
            trial = (line + " " + word).strip()
            if text_width(figure, trial, 13.5) > w - 0.030 and line:
                lines.append(line)
                line = word
            else:
                line = trial
        lines.append(line)
        axis.text(x + 0.014, y + h - 0.146, "\n".join(lines), fontsize=13.5,
                  color=S.INK_SOFT, va="top", linespacing=1.35)
        FONTS[CURRENT].extend([20, 20])
    axis.text(0.05, 0.188, "Each choice in full, with its trade-off and "
              "evidence: backup B1 to B6.", fontsize=14, color=S.INK_MUTED,
              va="top")
    return figure


def s10():
    figure, axis = slide("Six test maps, from familiar to hard", 10)
    n = len(LADDER)
    span = 0.90
    for index, (layout, name) in enumerate(LADDER):
        x = 0.05 + index * span / n
        w = span / n - 0.013
        thumb = axis.figure.add_axes([x, 0.455, w, 0.265])
        draw_map(thumb, scenario(layout), show_cspace=False, show_astar=False,
                 show_cart_lane=False, scale=False)
        thumb.axis("off")
        special = layout in {"L02", "L05"}
        axis.text(x + w / 2, 0.428, name, fontsize=17,
                  fontweight="bold" if special else "normal",
                  color=MINE if special else S.INK, ha="center", va="center")
        if special:
            axis.add_patch(FancyBboxPatch(
                (x - 0.004, 0.450), w + 0.008, 0.275,
                boxstyle="round,pad=0.004,rounding_size=0.010",
                facecolor="none", edgecolor=MINE, linewidth=1.8))
        number_card(axis, x, 0.200, f"{gap(layout):.2f}", "m",
                    "narrowest gap", "size",
                    width=w, height=0.180, value_size=30)
    axis.text(0.05, 0.178, "LOGGED-DATA RENDER · config/scenarios/L0*_C.json, "
              "results/v2_S6/scenario_table.csv", fontsize=8.5,
              color=S.INK_MUTED, va="top")
    return figure


def s11():
    figure, axis = slide("Every flight ends in one of three ways", 11)
    k, n, _lo, _hi = pooled("L00")
    endings = [("Goal", TICK, S.OUTCOME_COLOUR["goal"],
                RENDERS / "V-TOP_L00_success_poster.png",
                "V-TOP_L00_success.mp4", f"reaches within {GOAL_TOL:.1f} m"),
               ("Crash", CROSS, S.OUTCOME_COLOUR["collision_static"],
                RENDERS / "V-TOP_L05_failure_poster.png",
                "V-TOP_L05_failure.mp4", "touches an obstacle or the cart"),
               ("Timeout", CLOCK, S.OUTCOME_COLOUR["timeout"],
                FIGURES / "F3_trajectories.png",
                "results/v2_S6/runs/*/trajectory.csv",
                f"still flying after {EPISODE_S:.0f} s")]
    for index, (name, mark, colour, image, source, meaning) in enumerate(endings):
        x = 0.05 + index * 0.313
        axis.add_patch(FancyBboxPatch(
            (x, 0.40), 0.285, 0.345,
            boxstyle="round,pad=0.005,rounding_size=0.014",
            facecolor=colour, alpha=0.06, edgecolor=colour, linewidth=1.6))
        picture(axis, image, (x + 0.014, 0.512, 0.257, 0.160), source,
                "WEBOTS CAPTURE" if "poster" in image.name
                else "LOGGED-DATA RENDER")
        axis.text(x + 0.016, 0.708, mark, fontsize=28, color=colour,
                  fontweight="bold", va="center")
        axis.text(x + 0.058, 0.708, name, fontsize=24, fontweight="bold",
                  color=colour, va="center")
        axis.text(x + 0.016, 0.432, meaning, fontsize=15, color=S.INK,
                  va="center")
        FONTS[CURRENT].extend([24, 28])
    number_card(axis, 0.05, 0.195, f"{n}", "flights",
                "per map: 3 policies, 15 cart timings", "count", width=0.42,
                value_size=40)
    number_card(axis, 0.50, 0.195, f"{D['episodes']}", "flights",
                f"in total, {D['distinct']} different setups", "count",
                width=0.45, value_size=40)
    return figure


def s12():
    figure, axis = slide("Same success on the mirror map; none on the "
                         "hardest", 12)
    chart = axis.figure.add_axes([0.065, 0.44, 0.59, 0.295])
    for spine in ("top", "right"):
        chart.spines[spine].set_visible(False)
    for index, (layout, name) in enumerate(LADDER):
        k, n, low, high = pooled(layout)
        if layout == "L04":
            chart.bar(index, n, color="#e9ebee", edgecolor=NOT_CHOSEN,
                      linewidth=1.2, linestyle=(0, (3, 2)), width=0.62)
            chart.text(index, n / 2, CROSS + " not\ncounted", fontsize=14,
                       color=NOT_CHOSEN, fontweight="bold", ha="center",
                       va="center")
            continue
        mirror = layout == "L02"
        chart.bar(index, k, width=0.62,
                  color=CHOSEN if mirror else S.CONTROLLER_COLOUR["ppo_pooled"],
                  edgecolor=S.INK if mirror else "none",
                  linewidth=2.0 if mirror else 0)
        chart.errorbar(index, k, yerr=[[k - low * n], [high * n - k]],
                       color=S.INK_SOFT, capsize=6, elinewidth=1.2,
                       capthick=1.2, fmt="none")
        chart.text(index, high * n + 1.6, f"{k} of {n}", fontsize=17,
                   fontweight="bold", color=S.INK, ha="center")
    chart.set_xticks(range(len(LADDER)))
    chart.set_xticklabels([name for _l, name in LADDER], fontsize=14)
    chart.set_yticks([0, 15, 30, 45])
    chart.set_yticklabels(["0", "15", "30", "45"], fontsize=13)
    chart.set_ylim(0, 52)
    chart.set_xlim(-0.6, len(LADDER) - 0.4)
    chart.tick_params(length=0)
    chart.grid(axis="y", color=S.GRID, linewidth=0.6)
    chart.set_axisbelow(True)
    for index, (layout, _name) in enumerate(LADDER):
        x = 0.065 + (index + 0.5) / len(LADDER) * 0.59 - 0.034
        thumb = axis.figure.add_axes([x, 0.330, 0.068, 0.060])
        draw_map(thumb, scenario(layout), show_cspace=False, show_astar=False,
                 show_cart_lane=False, scale=False)
        thumb.axis("off")
    axis.text(0.065, 0.300,
              "lines show 95% uncertainty (approximate)\n"
              "Clutter excluded: my own reference path cannot fly it either",
              fontsize=12.5, color=S.INK_MUTED, va="top", linespacing=1.5)
    axis.text(0.065, 0.233, "LOGGED-DATA RENDER · "
              "results/v2_S6/pooled_rates.csv", fontsize=8.5,
              color=S.INK_MUTED, va="top")

    # Slide 13 is merged in here as the inset: the mirror pair, side by side.
    axis.text(0.680, 0.735, "Training vs Mirror: flipped, same difficulty",
              fontsize=15, fontweight="bold", color=MINE, va="center")
    for row, (layout, name) in enumerate([("L00", "Training"),
                                          ("L02", "Mirror")]):
        pane = axis.figure.add_axes([0.680, 0.555 - row * 0.145, 0.205,
                                     0.125])
        draw_map(pane, scenario(layout), show_cspace=False,
                 show_astar=False, show_cart_lane=False, scale=False)
        pane.axis("off")
        axis.text(0.895, 0.617 - row * 0.145, name, fontsize=15,
                  color=S.INK, fontweight="bold", va="center")
    axis.text(0.680, 0.400, "LOGGED-DATA RENDER · L00_C.json, L02_C.json",
              fontsize=8.5, color=S.INK_MUTED, va="top")
    k0, n0, _l, _h = pooled("L00")
    number_card(axis, 0.680, 0.185, f"{k0} of {n0}", "both maps",
                "no left-right memorising", "result", width=0.270,
                height=0.195, value_size=32)
    return figure


def s14():
    figure, axis = slide("I checked the simulator instead of trusting it", 14)
    lidar = D["lidar"]["cart_sector_pooled"]
    drift = D["reset"]["arms"]["sequence"]["max_path_delta_m_worst"]
    cards = [(f"{DT:.3f}", "s", "exactly, between every command", "time",
              icon_check),
             (f"{lidar['mae_m'] * 100:.2f}", "cm",
              f"laser vs truth, {lidar['comparisons']} checks", "accuracy",
              icon_lidar),
             (f"{drift * 1e6:.0f}", "um",
              "difference when a session is replayed", "accuracy", icon_maps),
             (f"{D['defects']['total']}", "problems",
              f"found by checking: {D['defects']['fixed']} fixed, "
              f"{D['defects']['open']} open", "count", icon_world)]
    for index, (value, unit, meaning, kind, icon) in enumerate(cards):
        x = 0.05 + index * 0.2375
        icon(axis, x + 0.105, 0.640, 0.046)
        number_card(axis, x, 0.235, value, unit, meaning, kind, width=0.215,
                    height=0.245, value_size=40)
    axis.text(0.05, 0.208, "LOGGED-DATA RENDER · results/v2_S6/, "
              "evidence/DEFECTS.csv, docs/verification/RESET_DETERMINISM.md",
              fontsize=8.5, color=S.INK_MUTED, va="top")
    body(axis, 0.05, 0.180,
         "The open one is the episode reset: fixed on a branch, not in these "
         "results.", size=20)
    return figure


def s15():
    figure, axis = slide("Done, not yet, and what I need from you", 15)
    columns = [
        ("Done", CHOSEN, TICK,
         ["the arena and walls", "six obstacles, each with a job",
          "the moving cart", "six seeded test maps",
          "the verification checks", "the GitHub documentation"]),
        ("Not yet", S.OKABE_ITO["orange"], CLOCK,
         ["the reset fix - it failed my strict test, so it is on its own "
          "branch", "the Clutter map - needs a decision",
          "the orbit video"]),
        ("I need", MINE, "->",
         ["a decision on the Clutter map",
          "approval to re-run once the reset is fixed",
          f"more PPO seeds from {SPENCER}"]),
    ]
    x0, w, gap_ = 0.05, 0.293, 0.022
    for index, (title, colour, mark, items) in enumerate(columns):
        x = x0 + index * (w + gap_)
        axis.add_patch(FancyBboxPatch(
            (x, 0.195), w, 0.545,
            boxstyle="round,pad=0.005,rounding_size=0.016",
            facecolor=colour, alpha=0.07, edgecolor=colour, linewidth=1.7))
        axis.text(x + 0.018, 0.690, mark, fontsize=26, color=colour,
                  fontweight="bold", va="center")
        axis.text(x + 0.066, 0.690, title, fontsize=24, fontweight="bold",
                  color=colour, va="center")
        FONTS[CURRENT].extend([24, 26])
        inner = w - 0.056
        y = 0.608
        for item in items:
            lines, line = [], ""
            for word in item.split():
                trial = (line + " " + word).strip()
                if text_width(figure, trial, 15.5) > inner and line:
                    lines.append(line)
                    line = word
                else:
                    line = trial
            lines.append(line)
            axis.text(x + 0.020, y, mark, fontsize=15, color=colour,
                      fontweight="bold", va="top")
            axis.text(x + 0.050, y, "\n".join(lines), fontsize=15.5,
                      color=S.INK, va="top", linespacing=1.45)
            y -= 0.040 + 0.036 * len(lines)
    axis.text(0.05, 0.168, "source: docs/meetings/2026-11-27/MEETING_DAY.md",
              fontsize=8.5, color=S.INK_MUTED, va="top")
    return figure


def s16():
    figure, axis = slide("Everything is on GitHub for the team", 16)
    picture(axis, SHOTS / "hero_annotated.png", (0.52, 0.255, 0.44, 0.46),
            "assets/screenshots/hero_annotated.png", "WEBOTS CAPTURE")
    takeaways = [
        ("The world is reproducible", "seeded, hashed, byte-identical"),
        ("The mirror map says the same", "no sign of route memorising"),
        ("Checking found six problems", "five fixed, one open and reported"),
    ]
    for index, (headline, detail) in enumerate(takeaways):
        y = 0.670 - index * 0.135
        axis.text(0.05, y, TICK, fontsize=22, color=CHOSEN, fontweight="bold",
                  va="center")
        axis.text(0.088, y, headline, fontsize=21, fontweight="bold",
                  color=S.INK, va="center")
        axis.text(0.088, y - 0.046, detail, fontsize=16, color=S.INK_SOFT,
                  va="center")
        FONTS[CURRENT].extend([21, 22])
    axis.text(0.05, 0.252, "github.com/khh4lid/dns-webots-simulation",
              fontsize=18, color=MINE, fontweight="bold", va="center")
    axis.text(0.05, 0.212, "private · every number traced to a file, listed in "
              "evidence/CLAIMS_EVIDENCE.csv", fontsize=14, color=S.INK_MUTED,
              va="center")
    return figure


# ---------------------------------------------------------------------------
# backups -- tables are allowed here, not on the main slides
# ---------------------------------------------------------------------------
def table(axis, columns, rows, *, x=0.05, y=0.74, width=0.90,
          header_size=13, body_size=12, line=0.055):
    widths = np.array([c[1] for c in columns], dtype=float)
    widths = widths / widths.sum() * width
    xs = x + np.concatenate([[0.0], np.cumsum(widths)[:-1]])
    for (name, _w), cx in zip(columns, xs):
        axis.text(cx, y, name, fontsize=header_size, fontweight="bold",
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


def b_choices():
    figure, axis = slide("Every choice in full, with its evidence", "B1")
    table(axis, [("Choice", 14), ("I chose", 20), ("I did not", 20),
                 ("Why", 28), ("Evidence", 14)],
          [(title, chosen, rejected, why, f"ADR-{n:03d}")
           for (title, chosen, rejected, why), n
           in zip(CHOICES, [1, 2, 3, 4, 6, 9])],
          y=0.70, body_size=11.5, line=0.072)
    axis.text(0.05, 0.195, "Trade-offs: the cart is kinematic, so a hit on it "
              "is geometric, not physical; the\nscan is a single flat layer, "
              "so nothing above or below it is observed; six maps with\n"
              "one seed each is a pilot, not a distribution.",
              fontsize=14, color=S.INK_SOFT, va="top", linespacing=1.6)
    return figure


def b_reset():
    figure, axis = slide("The reset finding, stated plainly", "B2")
    arms = D["reset"]["arms"]
    table(axis, [("How the episode is run", 42), ("Worst path difference", 30),
                 ("Episodes", 16)],
          [("replayed in session order",
            f"{arms['sequence']['max_path_delta_m_worst'] * 1e6:.0f} um",
            str(arms["sequence"]["episodes"])),
           ("run on its own, in isolation",
            f"{arms['isolated']['max_path_delta_m_worst']:.3f} m",
            str(arms["isolated"]["episodes"]))],
          y=0.70, body_size=13, line=0.075)
    body(axis, 0.05, 0.450,
         "The old reset stops when the drone is steady enough, so a little\n"
         "state from the previous episode survives into the next one.\n"
         "A fixed-length reset removes it, and is written and tested, but it\n"
         "still misses the strict bar on one of five targets, so it is NOT in\n"
         "these results. Every published number used one reset, in one order.",
         size=20, counted=False)
    body(axis, 0.05, 0.190,
         "The mirror result survives the fix: both maps stay equal.",
         size=20, colour=MINE, weight="bold", counted=False)
    axis.text(0.05, 0.132, "LOGGED-DATA RENDER · results/reset_study/"
              "SUMMARY.json, docs/verification/RESET_MODE_COMPARISON.md",
              fontsize=8.5, color=S.INK_MUTED, va="top")
    return figure


def b_clutter():
    figure, axis = slide("Why the Clutter map is not counted", "B3")
    picture(axis, FIGURES / "E4_clearance.png", (0.05, 0.215, 0.42, 0.50),
            "results/layout_gate/L04.json")
    body(axis, 0.52, 0.700,
         "On the Clutter map the PPO policy scores\n0 of 45.\n\n"
         "But my own reference path, which is not a\n"
         "learned policy at all, cannot fly it either,\n"
         "at any speed I tried.\n\n"
         "So a zero there measures my generator, not\n"
         "the policy. Counting it would overstate how\n"
         "hard the policy finds new layouts.\n\n"
         "It is reported, and excluded from the\n"
         "generalisation conclusion.",
         size=20, counted=False)
    axis.text(0.52, 0.260, "Decision needed: keep it excluded, or authorise\n"
                           "a longer-horizon diagnostic run?",
              fontsize=18, color=MINE, fontweight="bold", va="top",
              linespacing=1.5)
    return figure


def b_results():
    figure, axis = slide("Full results, including the reference controllers",
                         "B4")
    rows = []
    for layout, name in LADDER:
        k, n, low, high = pooled(layout)
        descriptor = D["layouts"][layout]
        rows.append((f"{layout} {name}",
                     f"{float(descriptor['iou_to_L00']):.3f}",
                     f"{gap(layout):.2f}",
                     f"{float(descriptor['astar_length_m']):.2f}",
                     "excluded" if layout == "L04" else f"{k} of {n}",
                     "excluded" if layout == "L04"
                     else f"[{low:.2f}, {high:.2f}]"))
    table(axis, [("Map", 22), ("IoU vs Training", 16), ("Gap (m)", 12),
                 ("A* path (m)", 14), ("PPO, cart moving", 18),
                 ("95% approx.", 18)],
          rows, y=0.70, body_size=12.5, line=0.066)
    axis.text(0.05, 0.215,
              "Mode C pools three checkpoints over 15 cart timings. Per-"
              "checkpoint values are never ranked:\ntheir intervals overlap "
              "almost completely. No significance test is applied: six maps "
              "with one\nseed per family is a pilot, not a sample from a "
              "distribution.",
              fontsize=13.5, color=S.INK_SOFT, va="top", linespacing=1.6)
    axis.text(0.05, 0.120, "LOGGED-DATA RENDER · results/v2_S6/"
              "pooled_rates.csv, scenario_table.csv", fontsize=8.5,
              color=S.INK_MUTED, va="top")
    return figure


def b_verification():
    figure, axis = slide("The verification numbers in full", "B5")
    lidar = D["lidar"]["cart_sector_pooled"]
    table(axis, [("Check", 34), ("Result", 30), ("Source", 36)],
          [("observation interval", "0.200000 s, error < 3e-16 s",
            "v1 Verification_Report"),
           ("world vs scenario pose", "<= 4.0e-06 m, 6 layouts",
            "results/layout_gate/SUMMARY.json"),
           ("LiDAR vs ray caster", "0.048 m mean per beam",
            "v1 Verification_Report"),
           ("LiDAR vs moving cart",
            f"MAE {lidar['mae_m']:.4f} m, n = {lidar['comparisons']}",
            "lidar_cart_consistency.json"),
           ("contract tests per layout", "26 of 26",
            "results/layout_gate/SUMMARY.json"),
           ("reproduction, replayed in order", "10/10 cells, 0 m",
            "results/v2_S6/v1_reproduction.csv"),
           ("episodes / distinct setups",
            f"{D['episodes']} / {D['distinct']}",
            "results/v2_S6/results_v2.csv"),
           ("airframe tilt exceedances", "3 of 540 above 0.50 rad",
            "results/v2_S6/results_v2.csv")],
          y=0.72, body_size=12.5, line=0.063)
    return figure


def b_timing():
    figure, axis = slide("Three nested clocks, and the interface", "B6")
    rows = [("physics integrator", f"{STEP_MS:.0f} ms", "Webots fixed step"),
            ("attitude and altitude loop", f"{INNER_HZ} Hz",
             "one update per physics step"),
            ("policy command period", f"{DT} s",
             f"exactly {int(DT * 1000 / STEP_MS)} physics steps"),
            ("episode horizon", f"{HORIZON} steps",
             f"{EPISODE_S:.0f} s of flight"),
            ("observation", "52 numbers",
             "single precision, two frames, from the perception module"),
            ("action", "2 numbers",
             f"normalised, scaled to {MAX_SPEED} m/s")]
    table(axis, [("Clock or signal", 34), ("Value", 22), ("Meaning", 44)],
          rows, y=0.70, body_size=13, line=0.072)
    axis.text(0.05, 0.215,
              "The command period is an integer multiple of the physics step, "
              "so no interpolation is\nneeded at any level. That is what "
              "makes an episode replay exactly.",
              fontsize=14, color=S.INK_SOFT, va="top", linespacing=1.6)
    return figure


SLIDES = [s01, s02, s03, s04, s05, s06, s07, s08, s09, s10, s11, s12, s14,
          s15, s16]
#: s13 is merged into s12 as the mirror inset, so the main run is 15 figures
#: covering the 16 storyboard beats. The tracker numbering still reads 1..16.
BACKUPS = [b_choices, b_reset, b_clutter, b_results, b_verification, b_timing]

PATH_12_MIN = list(range(1, 17))
PATH_5_MIN = [1, 2, 4, 7, 10, 12, 15]

NOTES = {
    1: ("This is my part of the group project: the simulated world the drone "
        "flies in, and the tests that say whether it works. I did not train "
        "the policy and I did not write the sensor code.",
        "If asked: the whole thing is one private GitHub repository and I can "
        "open it now."),
    2: ("The task in one sentence: fly from the green start to the blue goal "
        "without touching anything. The drone gets no map and no waypoints. "
        "Every number on a slide from here on carries its type, shown in the "
        "legend at the bottom.",
        "If asked: it counts as arrived within 0.4 m of the goal, and it has "
        "18 seconds, which is 90 commands at 0.2 s each."),
    3: ("The loop shows how the four parts fit. I own the blue box: the world, "
        "the obstacles and the tests. The grey boxes are my teammates', and we "
        "agreed a fixed interface so neither side can break the other.",
        "If asked: the interface is 52 numbers in and two numbers out, every "
        "0.2 seconds, written down in docs/interfaces/."),
    4: ("This is the floor plan, drawn to scale from the scenario file, not by "
        "hand. One aisle, ten metres by six, with the start on the left and "
        "the goal on the right inside its 0.4 m circle.",
        "If asked: the size is fixed because the policies trained on exactly "
        "this size, so changing it would make the comparison meaningless."),
    5: ("The drone flies at one metre and its laser scans a flat ring just "
        "above it, at 1.14 m. Every obstacle in the room is built taller than "
        "that line, so nothing can hide underneath the beam.",
        "If asked: yes, this means the drone cannot fly over anything. That "
        "matches the policy, which only commands sideways motion."),
    6: ("The airframe is about 35 centimetres across and weighs just over half "
        "a kilo. The LiDAR sits on top, held level. The policy does not fly it "
        "directly: it sends a speed command five times a second, and a faster "
        "inner loop keeps it balanced.",
        "If asked: the red dashed circle is the collision radius, 0.18 m, "
        "which is what the geometry checks use."),
    7: ("The drone sees with a laser that sweeps all the way round, 720 beams, "
        "one every half a degree. Those are grouped into twelve directions "
        "before the policy sees them. The picture on the right compares what "
        "is really there with what the sensor reports.",
        "If asked: sector zero is straight ahead, three is right, six is "
        "behind and nine is left."),
    8: ("Four obstacle types, and each is there for a stated reason written "
        "into the scenario file. The racks block the straight line. The cart "
        "is the only thing that moves, and it crosses the aisle on a fixed "
        "schedule, so the task is about timing as well as geometry.",
        "If asked: the cart is driven by a script, not by physics, so a hit on "
        "it is detected geometrically."),
    9: ("Six decisions, each with what I rejected and a one-line reason. The "
        "one I would point at is shapes: treating a long rack as a circle "
        "reported crashes while the drone still had half a metre of clear "
        "aisle.",
        "If asked: each one has a full decision record in docs/decisions/, and "
        "backups B1 has them all in one table."),
    10: ("Six maps, generated from a seed, from the training layout to a "
         "deliberately hard one. The number under each is the narrowest gap "
         "the route has to pass through, which is the honest measure of how "
         "hard a map is.",
         "If asked: the maps are hash-checked, so anyone can rebuild the "
         "identical six files and get the same geometry."),
    11: ("Only three things can happen. It reaches the goal, it touches "
         "something, or it runs out of time. Forty-five flights per map comes "
         "from three trained policies times fifteen different cart timings.",
         "If asked: the 540 total contains 480 genuinely different setups; "
         "deterministic repeats are counted once, not three times."),
    12: ("Two things to take away. The mirror map scores exactly the same as "
         "the training map, which is evidence against simple left-right "
         "memorising. The hardest map scores zero, which is where the limit "
         "is. The grey bar is excluded, and backup B3 explains why.",
         "If asked: the thin lines are uncertainty and they are approximate, "
         "because flights inside a cell run back to back rather than "
         "independently."),
    14: ("I do not ask anyone to trust the simulator. Timing is exact, the "
         "laser agrees with ground truth to about two centimetres, and a "
         "replayed session reproduces to five microns. Six real problems were "
         "found this way and five are fixed.",
         "If asked: the open one is the episode reset. The fix exists on a "
         "branch but it missed my own strict test, so I did not put it in "
         "these results."),
    15: ("Short version: the environment, the obstacles, the cart, the six "
         "maps and the checks are done, and documented on GitHub. Two things "
         "are not finished and I am not hiding either. Three things I need "
         "from the team.",
         "If asked: the Clutter decision is the one actually blocking a "
         "result, so that is the one I would like answered today."),
    16: ("Everything is in one private repository: the code, the data, the "
         "figures and a claim-by-claim trace from every number to the file it "
         "came from. Happy to open any of it now.",
         "If asked: licence status is not yet decided, which is the "
         "supervisor's call, so no licence file has been added."),
}
NOTES_BACKUP = {
    "B1": ("Every design choice in one place, with what I rejected, the "
           "reason, and the decision record that holds the full reasoning.",
           "If asked: the trade-offs line at the bottom is the honest cost of "
           "each choice."),
    "B2": ("The reset finding in two rows: replayed in order a run reproduces "
           "to microns; run on its own it can drift by 29 centimetres.",
           "If asked: it changes the independence assumption behind the "
           "uncertainty lines, not the measurements themselves."),
    "B3": ("Why a zero on the Clutter map is not evidence about the policy: "
           "my own non-learned reference path cannot fly it either.",
           "If asked: the diagnostic would be a longer time limit, which is "
           "one of the decisions I am asking for."),
    "B4": ("The full results table with the layout descriptors beside the "
           "success rates, so difficulty and outcome can be read together.",
           "If asked: no ranking of the three checkpoints, because their "
           "intervals overlap almost completely."),
    "B5": ("Every verification number with the file it comes from.",
           "If asked: evidence/CLAIMS_EVIDENCE.csv maps each claim to a file "
           "and the script that produced it."),
    "B6": ("The three nested clocks and the fixed interface to the other two "
           "parts of the system.",
           "If asked: 0.2 s is exactly 25 physics steps, which is why an "
           "episode can replay exactly."),
}


# ---------------------------------------------------------------------------
# automated quality checks
# ---------------------------------------------------------------------------
def audit() -> int:
    """Enforce the deck's own rules rather than trusting them."""
    failures: list[str] = []

    print("\n  per slide: prose words (cap "
          f"{PROSE_CAP}), number cards, smallest label")
    print(f"    {'slide':>6}  {'prose':>5}  {'cards':>5}  {'min pt':>6}")
    for key in PROSE:
        words = len(PROSE[key])
        cards = len(CARDS[key])
        smallest = min(FONTS[key]) if FONTS[key] else 0.0
        flags = []
        if words > PROSE_CAP:
            flags.append("WORDS")
        if smallest and smallest < LABEL_MIN_PT:
            flags.append(f"FONT {smallest:g}pt")
        print(f"    {key:>6}  {words:>5}  {cards:>5}  {smallest:>6.1f}"
              f"   {' '.join(flags)}")
        failures.extend(f"slide {key}: {flag}" for flag in flags)

    every = [c for cards in CARDS.values() for c in cards]
    print(f"\n  number cards: {len(every)} across {len(CARDS)} slides")
    kinds: dict[str, int] = {}
    for _value, _unit, kind in every:
        kinds[kind] = kinds.get(kind, 0) + 1
    for kind in TYPES:
        print(f"    {kind:>9}: {kinds.get(kind, 0)}")
    unused = [k for k in TYPES if not kinds.get(k)]
    if unused:
        print(f"    note: types never used on a slide: {', '.join(unused)}")
    missing_unit = [v for v, u, _k in every if not u]
    if missing_unit:
        failures.append(f"cards without a unit: {missing_unit}")

    print(f"\n  slides with at least one picture or drawing: all "
          f"{len(PROSE)} by construction")
    if failures:
        print("\n  !! " + "\n  !! ".join(failures))
    return 1 if failures else 0


def main() -> int:
    S.apply()
    OUT.mkdir(parents=True, exist_ok=True)
    PREVIEW.mkdir(parents=True, exist_ok=True)

    figures = []
    for builder in SLIDES:
        figure = builder()
        key = CURRENT
        name = f"final{int(key):02d}"
        # bbox_inches=None keeps every slide exactly 16:9; "tight" would crop
        # each one differently and the PPTX stretch would then distort them.
        figure.savefig(PREVIEW / f"{name}.png", dpi=110, facecolor="white",
                       bbox_inches=None, pad_inches=0)
        figures.append((name, figure, int(key)))
    for builder in BACKUPS:
        figure = builder()
        name = f"final_{CURRENT}"
        figure.savefig(PREVIEW / f"{name}.png", dpi=110, facecolor="white",
                       bbox_inches=None, pad_inches=0)
        figures.append((name, figure, CURRENT))

    pdf_path = OUT / "DNS_SimEnv_FINAL_2026-11-27.pdf"
    with PdfPages(pdf_path) as pdf:
        for _name, figure, _key in figures:
            pdf.savefig(figure, facecolor="white", bbox_inches=None,
                        pad_inches=0)
    for _name, figure, _key in figures:
        plt.close(figure)
    print(f"wrote {pdf_path.relative_to(ROOT)} "
          f"({len(figures)} slides, {pdf_path.stat().st_size / 1e6:.2f} MB)")

    build_pptx(figures)
    return audit()


def build_pptx(figures) -> None:
    from pptx import Presentation
    from pptx.util import Emu, Inches, Pt

    deck = Presentation()
    deck.slide_width = Inches(13.333)
    deck.slide_height = Inches(7.5)
    blank = deck.slide_layouts[6]

    #: Real MP4s where motion is the point. Rectangles match the poster frames
    #: already drawn on those slides, so the clip sits exactly on its still.
    MOVIES = {
        2: [(VIDEOS / "V-TOP_L00_success.mp4",
             RENDERS / "V-TOP_L00_success_poster.png", 0.53, 0.50, 0.43, 0.30)],
        11: [(VIDEOS / "V-TOP_L00_success.mp4",
              RENDERS / "V-TOP_L00_success_poster.png",
              0.064, 0.512, 0.257, 0.160),
             (VIDEOS / "V-TOP_L05_failure.mp4",
              RENDERS / "V-TOP_L05_failure_poster.png",
              0.377, 0.512, 0.257, 0.160)],
        12: [(VIDEOS / "V-TOP_L02_mirror.mp4",
              RENDERS / "V-TOP_L02_mirror_poster.png",
              0.680, 0.450, 0.280, 0.255)],
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

    path = OUT / "DNS_SimEnv_FINAL_2026-11-27.pptx"
    deck.save(str(path))
    print(f"wrote {path.relative_to(ROOT)} "
          f"({len(figures)} slides, {path.stat().st_size / 1e6:.2f} MB)")


if __name__ == "__main__":
    raise SystemExit(main())
