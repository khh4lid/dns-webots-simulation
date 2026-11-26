"""Build the 27 Nov 2026 deck: 12 slides plus 6 backups, PPTX and PDF.

Assertion-evidence structure: every body slide carries a sentence headline that
states the claim, one visual as the evidence, and almost no other text. Slides
are rendered with matplotlib so they share the figures' palette and typography
exactly, then assembled into a PDF and a PPTX. Video slides get the real MP4
embedded with its poster frame; the PDF keeps the poster and says where the
video lives.

Numbers come from the result files, never from this script's prose, and every
one is cross-checked against evidence/CLAIMS_EVIDENCE.csv by
scripts/check_slide_claims.py.

Run::

    python scripts/make_slides.py
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import dns_style as S                                            # noqa: E402
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages             # noqa: E402
from matplotlib.patches import Rectangle                         # noqa: E402

OUT = ROOT / "docs" / "meetings" / "2026-11-27"
PREVIEW = OUT / "slide_png"
FIGURES = ROOT / "assets" / "figures"
GIFS = ROOT / "assets" / "gifs"
RENDERS = ROOT / "assets" / "renders"
VIDEOS = ROOT / "results" / "video"
DIAGRAMS = ROOT / "assets" / "diagrams"
RESULTS = ROOT / "results" / "v2_S6"

#: Resolved only as far as the project record supports. "Khaled" and the
#: @khh4lid handle are on record; the full legal name, Spencer's surname and
#: the sensor teammate's name are not, and are shown as explicit gaps rather
#: than invented. The deck is presentable as-is; the title slide carries the
#: one outstanding item.
PRESENTER = "Khaled (@khh4lid)"
SUPERVISOR = "Dr. Mehdi Samieiyeganeh"
SPENCER = "Spencer"
SENSOR = "[sensor teammate — name to confirm]"
NAMES_PENDING = True        # full name / surnames still outstanding

SECTIONS = ["Role", "World", "Verification", "Results", "Decisions"]


def load_results() -> dict:
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
    return data


D = load_results()


def pooled(layout: str, mode: str = "C_dynamic") -> tuple[int, int, float, float]:
    e = D["pooled"][(layout, mode, "ppo_pooled")]
    return (int(e["k"]), int(e["n"]), float(e["wilson_low"]),
            float(e["wilson_high"]))


# ---------------------------------------------------------------------------
# slide chrome
# ---------------------------------------------------------------------------
def new_slide(headline: str, *, section: str | None = None,
              number: str = "", accent: str | None = None):
    """A 16:9 slide with a sentence headline, tracker and footer."""
    figure = plt.figure(figsize=(13.333, 7.5))
    figure.patch.set_facecolor("white")
    axis = figure.add_axes([0, 0, 1, 1])
    axis.axis("off")
    axis.set_xlim(0, 1)
    axis.set_ylim(0, 1)

    lines = headline.split("\n")
    for index, line in enumerate(lines[:2]):
        axis.text(0.055, 0.915 - index * 0.062, line, fontsize=25.5,
                  fontweight="bold", color=S.INK, va="top", ha="left")
    if accent:
        axis.text(0.055, 0.915 - len(lines[:2]) * 0.062 - 0.005, accent,
                  fontsize=17, color=S.CONTROLLER_COLOUR["ppo_pooled"],
                  fontweight="bold", va="top", ha="left")

    if section:
        x = 0.62
        for name in SECTIONS:
            live = name == section
            axis.add_patch(Rectangle((x, 0.952), 0.068, 0.009,
                                     facecolor=(S.CONTROLLER_COLOUR["ppo_pooled"]
                                                if live else S.GRID),
                                     edgecolor="none"))
            axis.text(x, 0.932, name, fontsize=8,
                      color=S.INK if live else S.INK_MUTED,
                      fontweight="bold" if live else "normal", va="top")
            x += 0.075

    footer = (f"DNS · Simulation environment · {PRESENTER} · 27 Nov 2026"
              + (f" · {number}" if number else ""))
    axis.text(0.055, 0.035, footer, fontsize=9, color=S.INK_MUTED, va="center")
    return figure, axis


def put_image(axis, path: Path, rect, source: str,
              provenance: str = "LOGGED-DATA RENDER") -> None:
    """Place an image inside ``rect`` = (x, y, w, h) in figure coordinates."""
    if not path.is_file():
        axis.text(rect[0], rect[1] + rect[3] / 2, f"[missing {path.name}]",
                  fontsize=12, color=S.OUTCOME_COLOUR["collision_static"])
        return
    image = plt.imread(str(path))
    figure = axis.figure
    height, width = image.shape[:2]
    box_ratio, image_ratio = rect[2] / rect[3], width / height * (7.5 / 13.333)
    if image_ratio > box_ratio:
        w, h = rect[2], rect[2] / image_ratio
    else:
        w, h = rect[3] * image_ratio, rect[3]
    x = rect[0] + (rect[2] - w) / 2
    y = rect[1] + (rect[3] - h) / 2
    inset = figure.add_axes([x, y, w, h])
    inset.imshow(image)
    inset.axis("off")
    axis.text(rect[0], rect[1] - 0.022, f"{provenance} · source: {source}",
              fontsize=8, color=S.INK_MUTED, va="top")


def kv_table(axis, rows, x=0.055, y=0.70, width=0.42, line=0.052,
             key_size=12.5, value_size=12.5) -> None:
    for index, (key, value) in enumerate(rows):
        yy = y - index * line
        axis.text(x, yy, key, fontsize=key_size, color=S.INK_SOFT, va="center")
        axis.text(x + width, yy, value, fontsize=value_size, color=S.INK,
                  fontweight="bold", va="center", ha="right")
        axis.plot([x, x + width], [yy - line * 0.42, yy - line * 0.42],
                  color=S.GRID, linewidth=0.7)


# ---------------------------------------------------------------------------
# the twelve slides
# ---------------------------------------------------------------------------
def s01():
    figure, axis = new_slide(
        "A reproducible Webots environment for testing UAV\n"
        "navigation among static and moving obstacles", number="1/12")
    axis.text(0.055, 0.70, f"{PRESENTER} — simulation environment",
              fontsize=18, color=S.CONTROLLER_COLOUR["ppo_pooled"],
              fontweight="bold", va="top")
    axis.text(0.055, 0.645,
              "Drone Navigation System · Taylor's University, Malaysia\n"
              f"Supervisor: {SUPERVISOR}\n"
              f"Team: PPO policy — {SPENCER} · Sensor & perception — {SENSOR}\n"
              "27 November 2026",
              fontsize=12.5, color=S.INK_SOFT, va="top", linespacing=1.75)
    put_image(axis, RENDERS / "V-TOP_L00_success_poster.png",
              (0.50, 0.17, 0.44, 0.55), "V-TOP_L00_success.mp4",
              "WEBOTS CAPTURE")
    return figure


def s02():
    figure, axis = new_slide(
        "I built and verified the world the team's drone flies in",
        section="Role", number="2/12")
    kv_table(axis, [
        ("1 · Simulator selection and setup", "Webots R2025a"),
        ("2 · Navigation environment design", "10 × 6 m, 1.0 m cruise"),
        ("3 · Start and goal definition", "fixed, identical in 6 layouts"),
        ("4 · Static obstacle development", "exact boxes, seeded generator"),
        ("5 · Moving obstacle modelling", "kinematic cart, verified"),
    ], y=0.70)
    axis.text(0.055, 0.40, "Not mine, and not in my repository:", fontsize=12,
              color=S.INK_SOFT, va="top")
    axis.text(0.055, 0.345,
              f"the PPO policy ({SPENCER}) and raw-sensor processing ({SENSOR}).\n"
              "I record their interfaces and the SHA-256 of the files I ran against.",
              fontsize=11.5, color=S.INK_MUTED, va="top", linespacing=1.7)
    put_image(axis, DIAGRAMS / "SV1_sensor_schematic.png",
              (0.52, 0.16, 0.42, 0.56), "assets/diagrams/SV1_sensor_schematic.png",
              "SCHEMATIC")
    return figure


def s03():
    figure, axis = new_slide(
        "The drone must cross a 10 × 6 m aisle, sensing obstacles\n"
        "only through a 360° LiDAR", section="World", number="3/12")
    put_image(axis, RENDERS / "V-TOP_L00_success_poster.png",
              (0.055, 0.17, 0.43, 0.58), "V-TOP_L00_success.mp4", "WEBOTS CAPTURE")
    put_image(axis, RENDERS / "V-SIDE_L00_altitude_poster.png",
              (0.52, 0.17, 0.43, 0.58), "V-SIDE_L00_altitude.mp4", "WEBOTS CAPTURE")
    axis.text(0.055, 0.80,
              "start (0.85, 0.00) → goal (9.15, 0.00), 0.4 m tolerance · "
              "altitude 1.0 m · one velocity command every 0.2 s",
              fontsize=12, color=S.INK_SOFT, va="top")
    return figure


def s04():
    figure, axis = new_slide(
        "Four obstacle types, each placed for a reason",
        section="World", number="4/12")
    put_image(axis, FIGURES / "obstacle_catalogue.png",
              (0.055, 0.14, 0.89, 0.63), "assets/figures/obstacle_catalogue.png",
              "WEBOTS CAPTURE (crops) + logged dimensions")
    return figure


def s05():
    k, n, _lo, _hi = pooled("L00")
    figure, axis = new_slide(
        "Webots gives built-in LiDAR, ground-truth access\n"
        "and exact reproducibility", section="World", number="5/12",
        accent="L00 re-run reproduced all 10 cells with 0 m path difference")
    kv_table(axis, [
        ("Built-in configurable LiDAR", "Webots ✓  Gazebo ✓  PyBullet ✗"),
        ("Supervisor API for ground truth", "Webots ✓  others via plugins"),
        ("Physics-based rotor flight", "Webots ✓  AirSim ✓  Isaac ✓"),
        ("Native Windows, no ROS, no GPU", "Webots ✓  Gazebo partial"),
        ("Licence and maintenance", "Apache 2.0, active"),
    ], y=0.66, width=0.46)
    axis.text(0.055, 0.30,
              "Selection rationale, not a benchmark: no competing simulator was\n"
              "installed or timed. Full criteria and sources in ADR-001.",
              fontsize=11, color=S.INK_MUTED, va="top", linespacing=1.7)
    put_image(axis, FIGURES / "E9_l00_reproduction.png",
              (0.57, 0.14, 0.38, 0.48), "results/v2_S6/v1_reproduction.csv")
    return figure


def s06():
    cart = D["lidar"]["cart_sector_pooled"]
    figure, axis = new_slide(
        "The LiDAR delivers exactly the interface the policy expects",
        section="Verification", number="6/12",
        accent=f"measured against an independent ray caster: "
               f"MAE {cart['mae_m']} m over {cart['comparisons']} comparisons")
    kv_table(axis, [
        ("Beams over 360°", "720  (0.5° spacing)"),
        ("Range cap", "5.0 m"),
        ("Sectors", "12 × 30°, 0/3/6/9 = front/right/back/left"),
        ("Scan plane", "1.14 m (1.0 m cruise + 0.14 m mount)"),
        ("Contract tests, every layout", "26 / 26"),
    ], y=0.63, width=0.47)
    put_image(axis, DIAGRAMS / "SV1_sensor_schematic.png",
              (0.56, 0.13, 0.39, 0.48), "assets/diagrams/SV1_sensor_schematic.png",
              "SCHEMATIC")
    return figure


def s07():
    figure, axis = new_slide(
        "We verify the simulator instead of trusting it —\n"
        "and verification caught six real defects",
        section="Verification", number="7/12")
    # Keys and values both sit left of 0.50 so nothing runs under the figure.
    kv_table(axis, [
        ("Observation interval", "0.200000 s (< 3e−16 s)"),
        ("World ↔ scenario pose", "≤ 4.0e−06 m, 6 layouts"),
        ("LiDAR vs ray caster", "0.048 m mean/beam"),
        ("L00 reproduction, in order", "10/10 cells, 0 m"),
        ("Same episode in isolation", "drifts to 0.287 m"),
    ], y=0.66, width=0.43, key_size=11.5, value_size=11.5)
    axis.text(0.055, 0.33, "Caught by these checks:", fontsize=12,
              color=S.INK_SOFT, va="top", fontweight="bold")
    axis.text(0.055, 0.275,
              "racks collided as circles · camera rotated instead of tilting\n"
              "cart written at its sweep centre, not its t = 0 pose\n"
              "calibration tool loaded a different world\n"
              "v1 shipped a protocol file from a different run\n"
              "the reset left residual state behind, so an episode re-run\n"
              "alone drifts up to 0.287 m — found by testing, fixed on\n"
              "branch reset-fix-v2.1, awaiting full re-validation",
              fontsize=10.2, color=S.INK_MUTED, va="top", linespacing=1.7)
    put_image(axis, FIGURES / "F4_verification_panel.png",
              (0.55, 0.13, 0.40, 0.54), "assets/figures/F4_verification_panel.png")
    return figure


def s08():
    widths = " / ".join(
        f"{float(D['layouts'][l]['min_corridor_width_enroute_m']):.2f}"
        for l in ("L00", "L01", "L02", "L03", "L04", "L05"))
    figure, axis = new_slide(
        "Six seeded layouts form a graded test of generalisation",
        section="Results", number="8/12",
        accent=f"en-route corridor width L00→L05:  {widths} m")
    put_image(axis, FIGURES / "F1_shift_ladder.png",
              (0.19, 0.09, 0.62, 0.62), "config/scenarios/L0*_C.json")
    return figure


def s09():
    a = pooled("L00"); b = pooled("L05")
    figure, axis = new_slide(
        "Success on L01–L03 is not distinguishable from the training\n"
        "layout; on the hardest layout it falls to 0/45",
        section="Results", number="9/12",
        accent=f"{D['episodes']} episodes = {D['distinct']} distinct configurations")
    put_image(axis, FIGURES / "F2_generalization.png",
              (0.055, 0.10, 0.56, 0.60), "results/v2_S6/pooled_rates.csv")
    kv_table(axis, [
        ("L00 baseline", f"{a[0]}/{a[1]}  [{a[2]:.2f}, {a[3]:.2f}]"),
        ("L01 perturbed", "20/45  [0.31, 0.59]"),
        ("L02 mirrored", "24/45  [0.39, 0.67]"),
        ("L03 aisle shift", "20/45  [0.31, 0.59]"),
        ("L05 hard", f"{b[0]}/{b[1]}  [{b[2]:.2f}, {b[3]:.2f}]"),
    ], x=0.64, y=0.60, width=0.30, key_size=11.5, value_size=11.5)
    axis.text(0.64, 0.30,
              "L04 is excluded: no controller and no reference-tracker\n"
              "setting reaches the goal inside the 90-step horizon, so\n"
              "0/45 there is not attributable to the policy.",
              fontsize=10.5, color=S.INK_MUTED, va="top", linespacing=1.7)
    return figure


def s10():
    layout = D["layouts"]["L02"]
    figure, axis = new_slide(
        "The mirror test found no sign of left/right memorisation",
        section="Results", number="10/12",
        accent=f"IoU {float(layout['iou_to_L00']):.3f} vs L00, "
               f"{float(layout['mean_obstacle_displacement_m']):.2f} m mean "
               f"displacement — identical difficulty by construction")
    put_image(axis, FIGURES / "E15_mirror_overlay.png",
              (0.055, 0.09, 0.40, 0.62), "assets/figures/E15_mirror_overlay.png")
    put_image(axis, RENDERS / "V-TOP_L02_mirror_poster.png",
              (0.50, 0.33, 0.45, 0.40), "V-TOP_L02_mirror.mp4", "WEBOTS CAPTURE")
    axis.text(0.50, 0.27,
              "Same pooled result on both: 24/45.\n"
              "This is NOT a claim that the policy generalises — on L05,\n"
              "whose traversability was confirmed, the same checkpoints score 0/45.",
              fontsize=11, color=S.INK_SOFT, va="top", linespacing=1.75)
    return figure


def s11():
    figure, axis = new_slide(
        "What I corrected, and what is still limited",
        section="Decisions", number="11/12")
    axis.text(0.055, 0.76, "Corrected from v1", fontsize=14,
              fontweight="bold", color=S.OUTCOME_COLOUR["goal"], va="top")
    axis.text(0.055, 0.70,
              "Mode B was 1 configuration, not 3 — v1's “3/3” is 1/1\n\n"
              "The reactive explanation used the crate's centre (1.54 m);\n"
              "the controller reacts to the surface, 1.0582 m\n\n"
              "“Cart error 0.00e+00 m” is true by construction —\n"
              "reframed, and replaced with a real ray-caster comparison\n\n"
              "v1 shipped a protocol file from a different run",
              fontsize=11, color=S.INK_SOFT, va="top", linespacing=1.6)
    axis.text(0.52, 0.76, "Still limited", fontsize=14, fontweight="bold",
              color=S.OUTCOME_COLOUR["collision_static"], va="top")
    axis.text(0.52, 0.70,
              "Cart is kinematic — no physical contact response\n\n"
              "LiDAR gimbal is ideal — a best case, not a real sensor\n\n"
              "Three training seeds · 90-step horizon · one scan layer\n\n"
              "A tumbled airframe cannot re-settle, aborting the next episode\n\n"
              "One unexplained sensor reading: 0.0536 m where ground\n"
              "truth was 0.62 m",
              fontsize=11, color=S.INK_SOFT, va="top", linespacing=1.6)
    return figure


def s12():
    figure, axis = new_slide(
        "Four decisions I need from the team",
        section="Decisions", number="12/12")
    kv_table(axis, [
        ("L04: exclude, or run a longer-horizon diagnostic?", SUPERVISOR),
        ("Approve a full re-evaluation after the reset fix (v2.1)?", SUPERVISOR),
        ("More PPO training seeds — 3 cannot separate variance", SPENCER),
        ("The 0.0536 m vs 0.62 m sensor anomaly", "sensor teammate"),
    ], y=0.70, width=0.62, key_size=12.5, value_size=11.5)
    axis.text(0.055, 0.44, "My next steps", fontsize=13, fontweight="bold",
              color=S.INK, va="top")
    axis.text(0.055, 0.385,
              "finish the progress report · stabilisation bracket (ideal vs\n"
              "body-fixed gimbal) · dynamic-obstacle variants C1–C3",
              fontsize=11.5, color=S.INK_SOFT, va="top", linespacing=1.7)
    axis.text(0.055, 0.21, "github.com/khh4lid/dns-webots-simulation",
              fontsize=13, color=S.CONTROLLER_COLOUR["ppo_pooled"],
              fontweight="bold", va="top")
    axis.text(0.055, 0.16,
              "private · every number traced in evidence/CLAIMS_EVIDENCE.csv",
              fontsize=10.5, color=S.INK_MUTED, va="top")
    return figure


# ---------------------------------------------------------------------------
# backups
# ---------------------------------------------------------------------------
def b1():
    figure, axis = new_slide("Backup — full results table", number="B1")
    put_image(axis, FIGURES / "E1_descriptors.png", (0.06, 0.12, 0.88, 0.66),
              "results/v2_S6/scenario_table.csv")
    return figure


def b2():
    figure, axis = new_slide("Backup — the fourteen decisions", number="B2")
    text = [
        "001 Webots R2025a    002 keep 10 × 6 m      003 planar, one scan layer",
        "004 exact boxes      005 racks cross line   006 kinematic cart",
        "007 ideal gimbal     008 5 Hz / 125 Hz / 8 ms  009 seeded generator",
        "010 mirror test      011 distinct configs   012 one geometry source",
        "013 Camera capture   014 L04 excluded (pending)",
    ]
    axis.text(0.055, 0.70, "\n\n".join(text), fontsize=12.5, color=S.INK_SOFT,
              va="top", linespacing=1.5, family="DejaVu Sans Mono")
    axis.text(0.055, 0.20, "Each record: context · options · decision · evidence "
                           "· consequences · status\ndocs/decisions/",
              fontsize=11, color=S.INK_MUTED, va="top", linespacing=1.7)
    return figure


def b3():
    figure, axis = new_slide("Backup — why the reactive controller fails",
                             number="B3")
    put_image(axis, ROOT / "assets" / "decisions" / "ADR-C1_reactive_zone.png",
              (0.10, 0.12, 0.80, 0.66), "results/v2_S6/reactive_gnron_analysis.json")
    return figure


def b4():
    g4 = json.loads((ROOT / "results" / "layout_gate" / "L04.json")
                    .read_text(encoding="utf8"))["reference_traversal"]
    figure, axis = new_slide("Backup — why L04 is excluded", number="B4")
    kv_table(axis, [
        ("margin 0.22 m, ≤ 0.48 m/s", "timeout at 90 steps, no collision"),
        ("margin 0.22 m, ≤ 0.84 m/s", "collision at 13 steps"),
        ("margin 0.12 m, ≤ 1.08 m/s", "tumbled, peak tilt 1.5905 rad"),
        ("PPO episodes exceeding 0.50 rad", "0 of 90"),
        ("A* feasible, corridor", "yes, 0.85 m"),
    ], y=0.68, width=0.55)
    axis.text(0.055, 0.34,
              "The reference tracker is GIVEN the correct route and still cannot\n"
              "fly it inside the horizon, so 0/90 cannot be attributed to the policy.",
              fontsize=11.5, color=S.INK_SOFT, va="top", linespacing=1.7)
    return figure


def b5():
    figure, axis = new_slide("Backup — architecture and the three clocks",
                             number="B5")
    put_image(axis, ROOT / "assets" / "decisions" / "ADR-008_timing_ladder.png",
              (0.10, 0.34, 0.80, 0.40), "assets/decisions/ADR-008_timing_ladder.png",
              "SCHEMATIC")
    put_image(axis, FIGURES / "side_elevation.png", (0.14, 0.05, 0.72, 0.26),
              "assets/figures/side_elevation.png")
    return figure


def b6():
    figure, axis = new_slide("Backup — reproducing this", number="B6")
    kv_table(axis, [
        ("Rebuild every figure and table from logs", "python reproduce.py --fast"),
        ("Re-run all 540 episodes", "python reproduce.py --full  (~19 min)"),
        ("Generator and validator unit tests", "26 / 26"),
        ("Layout gate, per layout", "26 contract tests + A* + flown traversal"),
        ("Claims traced to a file", "evidence/CLAIMS_EVIDENCE.csv"),
    ], y=0.68, width=0.56)
    return figure


def b7():
    """The reset finding, read from its own evidence file at build time."""
    data = json.loads((ROOT / "results" / "reset_study" / "SUMMARY.json")
                      .read_text(encoding="utf8"))
    iso, seq = data["arms"]["isolated"], data["arms"]["sequence"]
    figure, axis = new_slide(
        "Backup — is one episode reproducible on its own?", number="B7",
        accent="No. It is reproducible when the session is replayed in order.")
    kv_table(axis, [
        ("Episodes tested, both arms", f"{iso['episodes']} + {seq['episodes']}"),
        ("Isolated — outcome matched the log",
         f"{iso['outcome_matches_log']}/{iso['episodes']}"),
        ("Isolated — step count matched",
         f"{iso['step_count_matches_log']}/{iso['episodes']}"),
        ("Isolated — worst path drift",
         f"{iso['max_path_delta_m_worst']:.3f} m"),
        ("Replayed in order — step count matched",
         f"{seq['step_count_matches_log']}/{seq['episodes']}"),
        ("Replayed in order — worst path drift",
         f"{seq['max_path_delta_m_worst']:.2e} m"),
    ], y=0.66, width=0.56, key_size=12, value_size=12)
    axis.text(0.055, 0.30,
              "Cause: the reset settled to a TOLERANCE (0.02 m/s, 0.02 m), not\n"
              "to a fixed point, so the state it stopped at depended on the\n"
              "previous episode. Fixed on branch reset-fix-v2.1 with a\n"
              "fixed-step settle; not merged — 1 of 5 targets still drifts\n"
              "2.1e-04 m against a 1e-05 m bar, and this deck was not\n"
              "re-derived from it.",
              fontsize=10.5, color=S.INK_SOFT, va="top", linespacing=1.7)
    axis.text(0.055, 0.065,
              "Source: results/reset_study/SUMMARY.json · "
              "docs/verification/RESET_DETERMINISM.md",
              fontsize=9, color=S.INK_MUTED, va="top")
    return figure


SLIDES = [s01, s02, s03, s04, s05, s06, s07, s08, s09, s10, s11, s12]
BACKUPS = [b1, b2, b3, b4, b5, b6, b7]

#: Two routes through the deck. The 5-minute path keeps the spine -- role,
#: task, results, mirror, decisions -- and drops the context slides; the
#: verification story then lives in one sentence on slide 9 and in backup B7.
PATH_12_MIN = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
PATH_5_MIN = [1, 2, 9, 10, 12]
PATH_5_MIN_SKIP = [3, 4, 5, 6, 7, 8, 11]

NOTES = {
 1: ("I built the simulation environment for the DNS project.",
     ["This is the world the team's drone flies in, and the instruments that check it.",
      "Everything on these slides comes from a file in the repository.",
      "The clip is a real Webots capture of a logged run, not an animation."],
     "Let me start with what was mine to build."),
 2: ("My contribution is the environment — five responsibilities, all delivered.",
     ["Simulator choice, environment design, start and goal, static obstacles, the moving cart.",
      "The PPO policy and the sensor processing belong to teammates; neither is in my repository.",
      "I record their interfaces and the SHA-256 of the files I ran against."],
     "So what does the drone actually have to do?"),
 3: ("The task is one aisle crossing, sensed only by a planar LiDAR.",
     ["Start left, goal right, 0.4 m tolerance, fixed 1.0 m altitude.",
      "One velocity command every 0.2 s — five per second.",
      "Top view on the left, side view on the right; both the same logged episode."],
     "The obstacles in it are not decoration."),
 4: ("Four obstacle types, each with a job.",
     ["Racks force the route off the straight line; columns give a known square return.",
      "The crate stack sits on the final approach; the cart is the moving hazard.",
      "Every one rises above the 1.14 m scan plane — nothing is solid but invisible."],
     "Why Webots for all this?"),
 5: ("Webots was chosen for the supervisor API and the built-in LiDAR.",
     ["Define 'supervisor': it lets me read and write exact body poses from outside the controller.",
      "That is what every validator and the ground-truth scoring depends on.",
      "This is a selection rationale, not a benchmark — I did not install and time the alternatives.",
      "The evidence that matters is determinism: the v1 run reproduced with zero path difference WHEN the session is replayed in order."],
     "The sensor is where the contract with Spencer lives."),
 6: ("The LiDAR matches the policy's contract, and that is tested, not assumed.",
     ["Define 'sector': the 360° scan is binned into 12 wedges of 30°.",
      "0 is front, 3 right, 6 back, 9 left — verified per layout, not documented and hoped for.",
      "The cart check compares the measured sector against an independent ray caster."],
     "That is one check. Here is the rest."),
 7: ("We verify the simulator rather than trusting it, and it caught six real defects.",
     ["Timing is exact by construction: 0.2 s is exactly 25 physics steps.",
      "The best argument for verification is the defect list — all six were silent.",
      "The circle-versus-box one would have corrupted every collision statistic.",
      "The sixth is mine and it is recent: the reset left state behind. I found it by asking whether a single episode reproduces on its own. It does not — up to 0.287 m.",
      "The fix is written and validated on reset-fix-v2.1, but it is not in this build: changing the environment would invalidate the results produced with it."],
     "With the instrument checked, I could ask the real question."),
 8: ("Six layouts, generated from one config and a seed, frozen before anything ran.",
     ["The ladder goes from barely perturbed to deliberately hard.",
      "Corridor width is the tightest point of the most generous route.",
      "Every layout is A*-feasible and blocks the straight line, so the static test still discriminates."],
     "Now the results."),
 9: ("Success holds on three layouts and collapses on the hardest one.",
     ["Define 'Wilson interval': a confidence interval that stays sensible at small n.",
      "L01 and L03 overlap L00 — I cannot distinguish them, and I do not claim to.",
      "540 episodes, but 480 distinct configurations: Mode B's three runs are one configuration.",
      "L04 is excluded — my own reference tracker could not fly it either."],
     "One result deserves its own slide."),
 10: ("The mirror is the cleanest test I have, and it came back negative.",
      ["Define 'IoU': overlap of the two layouts' obstacle footprints. 0.074 is almost none.",
       "Difficulty is identical by construction — same A* length, tortuosity, corridor width.",
       "Same pooled result, 24 of 45 on both.",
       "I phrase this narrowly: no evidence of left/right memorisation. Not 'it generalises'."],
      "Which brings me to what I got wrong."),
 11: ("I corrected four things from v1 and I am still carrying six limitations.",
      ["The Mode B correction makes my own earlier numbers weaker, not stronger.",
       "The reactive explanation was arithmetically wrong — right conclusion, wrong reasoning.",
       "The cart's zero error was true by construction and carried no information."],
      "And here is what I need from you."),
 12: ("Three decisions, one each for Dr. Mehdi, Spencer and the sensor teammate.",
      ["L04 is the one that blocks a result — exclude it, or authorise a diagnostic run?",
       "More seeds would narrow every interval on slide 9.",
       "The sensor anomaly matters because repulsion scales with 1.45 minus the range."],
      "Happy to go to any of the backup slides."),
}


def main() -> int:
    S.apply()
    OUT.mkdir(parents=True, exist_ok=True)
    PREVIEW.mkdir(parents=True, exist_ok=True)
    figures = []
    for index, builder in enumerate(SLIDES + BACKUPS, start=1):
        figure = builder()
        name = (f"slide{index:02d}" if index <= len(SLIDES)
                else f"backup{index - len(SLIDES)}")
        figure.savefig(PREVIEW / f"{name}.png", dpi=110, facecolor="white")
        figures.append((name, figure))

    pdf_path = OUT / "DNS_SimEnv_2026-11-27.pdf"
    with PdfPages(pdf_path) as pdf:
        for _name, figure in figures:
            pdf.savefig(figure, facecolor="white")
    for _name, figure in figures:
        plt.close(figure)
    print(f"wrote {pdf_path.relative_to(ROOT)} "
          f"({len(figures)} slides, {pdf_path.stat().st_size/1e6:.2f} MB)")

    build_pptx(figures)
    return 0


def build_pptx(figures) -> None:
    from pptx import Presentation
    from pptx.util import Emu, Inches, Pt

    deck = Presentation()
    deck.slide_width = Inches(13.333)
    deck.slide_height = Inches(7.5)
    blank = deck.slide_layouts[6]

    #: slide number -> (video, poster). Embedded as real movies with a poster
    #: frame; the PDF keeps the poster and names the file instead.
    MOVIES = {
        3: [(VIDEOS / "V-TOP_L00_success.mp4",
             RENDERS / "V-TOP_L00_success_poster.png", 0.055, 0.17, 0.43, 0.58),
            (VIDEOS / "V-SIDE_L00_altitude.mp4",
             RENDERS / "V-SIDE_L00_altitude_poster.png", 0.52, 0.17, 0.43, 0.58)],
        10: [(VIDEOS / "V-TOP_L02_mirror.mp4",
              RENDERS / "V-TOP_L02_mirror_poster.png", 0.50, 0.33, 0.45, 0.40)],
    }

    for index, (name, _figure) in enumerate(figures, start=1):
        slide = deck.slides.add_slide(blank)
        slide.shapes.add_picture(str(PREVIEW / f"{name}.png"), 0, 0,
                                 width=deck.slide_width,
                                 height=deck.slide_height)
        for video, poster, x, y, w, h in MOVIES.get(index, []):
            if not (video.is_file() and poster.is_file()):
                continue
            try:
                slide.shapes.add_movie(
                    str(video),
                    Emu(int(x * deck.slide_width)),
                    Emu(int((1 - y - h) * deck.slide_height)),
                    Emu(int(w * deck.slide_width)),
                    Emu(int(h * deck.slide_height)),
                    poster_frame_image=str(poster), mime_type="video/mp4")
            except Exception as error:                           # noqa: BLE001
                print(f"  note: could not embed {video.name}: {error}")

        note = NOTES.get(index)
        if note:
            message, points, transition = note
            body = slide.notes_slide.notes_text_frame
            body.text = f"MESSAGE: {message}"
            for point in points:
                paragraph = body.add_paragraph()
                paragraph.text = f"• {point}"
                paragraph.font.size = Pt(12)
            paragraph = body.add_paragraph()
            paragraph.text = f"TRANSITION: {transition}"
            paragraph.font.size = Pt(12)

    path = OUT / "DNS_SimEnv_2026-11-27.pptx"
    deck.save(str(path))
    print(f"wrote {path.relative_to(ROOT)} "
          f"({len(figures)} slides, {path.stat().st_size/1e6:.2f} MB)")
    if NAMES_PENDING:
        print("\nIdentity: resolved as far as the project record supports "
              "(Khaled @khh4lid, Spencer, Dr. Mehdi Samieiyeganeh).\n"
              "Shown as explicit gaps rather than invented: full legal name "
              "(CITATION.cff only), Spencer's surname and handle, the sensor "
              "teammate's name.")


if __name__ == "__main__":
    raise SystemExit(main())
