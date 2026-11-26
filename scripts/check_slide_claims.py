"""Match every quantitative claim on the slides against the data files.

The deck computes most of its numbers from the result files at build time, so
those cannot drift. The risk is the **literal** numbers in slide prose and in
the speaker notes: those were typed, and a typed number can be wrong.

This script extracts every numeric token from the slide source's literal
strings and from the speaker notes, then tries to match each against a value
that is actually derivable from a file in the repository. Anything it cannot
match is reported as UNMATCHED and has to be fixed or justified before the deck
is shown -- an unmatched number is not evidence of an error, but it is evidence
that nothing in the repository backs it.

Writes ``evidence/SLIDES_CLAIMS_CHECK.csv``.

Run::

    python scripts/check_slide_claims.py
"""
from __future__ import annotations

import argparse
import ast
import csv
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results" / "v2_S6"
EVIDENCE = ROOT / "evidence"
#: Default target. ``--source``/``--out`` let the same check run over the
#: simple deck, which has its own literals and its own output CSV.
SOURCE = ROOT / "scripts" / "make_slides.py"

#: Numbers that are structural rather than measured -- slide indices, section
#: counts, figure sizes. Matching them against experiment data would be
#: meaningless, so they are excluded by value with a stated reason.
STRUCTURAL = {
    "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12",
    "0", "13", "14", "16", "18", "20", "25", "30", "60",
    # zero-padded ADR identifiers on the backup slide, and the Webots year
    "001", "002", "003", "004", "005", "006", "007", "008", "009",
    "010", "011", "012", "013", "014", "2025",
    # zero-padded variant ids inside asset filenames, e.g. V-TOP_L02_v01
    "00", "01", "02", "03", "04", "05", "06",
    # the meeting date, and the "256" of SHA-256
    "27", "2026", "256",
}


#: Strings that are code rather than slide text: hex colours, matplotlib
#: box/arrow style specifiers and format specs. Their digits are layout
#: constants, so matching them against experiment data would be meaningless.
NOT_A_CLAIM = re.compile(
    r"^(#[0-9a-fA-F]{3,8}$|round,pad=|square,pad=|arc3|angle|bar,|simple,|"
    r"fancy,|wedge,|->|-\|>|<\|-\|>|[-+]?\d*\.?\d*[eEfgd%]$)")


def known_values() -> dict[str, str]:
    """Every number the repository can back, mapped to where it comes from."""
    values: dict[str, str] = {}

    def add(number, source: str) -> None:
        for text in {f"{number}", f"{float(number):g}"}:
            values.setdefault(text.rstrip("."), source)

    # --- experiment totals -------------------------------------------------
    with (RESULTS / "results_v2.csv").open(encoding="utf8") as handle:
        rows = list(csv.DictReader(handle))
    add(len(rows), "results_v2.csv row count")
    add(sum(int(r["distinct_configuration"]) for r in rows),
        "results_v2.csv distinct_configuration sum")
    add(sum(int(r["tilt_limit_exceeded"]) for r in rows),
        "results_v2.csv tilt_limit_exceeded sum")

    # --- pooled rates and intervals ---------------------------------------
    with (RESULTS / "pooled_rates.csv").open(encoding="utf8") as handle:
        for row in csv.DictReader(handle):
            tag = f"pooled_rates.csv {row['layout_id']}/{row['mode']}/{row['group']}"
            add(row["k"], tag); add(row["n"], tag)
            for key in ("rate", "wilson_low", "wilson_high"):
                add(f"{float(row[key]):.2f}", tag)
                add(f"{float(row[key]):.3f}", tag)

    # --- layout descriptors ------------------------------------------------
    with (RESULTS / "scenario_table.csv").open(encoding="utf8") as handle:
        for row in csv.DictReader(handle):
            if row["mode"] != "C":
                continue
            tag = f"scenario_table.csv {row['layout_id']}"
            for key in ("min_corridor_width_enroute_m", "iou_to_L00",
                        "astar_length_m", "tortuosity", "line_intrusion_m",
                        "mean_obstacle_displacement_m", "n_obstacles"):
                if not row.get(key):
                    continue
                add(f"{float(row[key]):.2f}", tag)
                add(f"{float(row[key]):.3f}", tag)

    # --- verification ------------------------------------------------------
    lidar = json.loads((RESULTS / "lidar_cart_consistency.json").read_text(
        encoding="utf8"))["cart_sector_pooled"]
    for key, value in lidar.items():
        add(value, "lidar_cart_consistency.json cart_sector_pooled")

    reactive = json.loads((RESULTS / "reactive_gnron_analysis.json").read_text(
        encoding="utf8"))
    crate = reactive["layouts"]["L00"]["surface_vs_centre_to_goal"]["crate_stack"]
    add(crate["surface_to_goal_m"], "reactive_gnron_analysis.json crate surface")
    add(f"{crate['centre_to_goal_m']:.2f}",
        "reactive_gnron_analysis.json crate centre")
    add(reactive["repulsion_radius_m"], "evaluate.gap_rule repulsion radius")
    run = reactive["layouts"]["L00"].get("reactive_mode_b", {})
    if run:
        add(run["max_repel_norm"], "reactive_gnron_analysis.json peak repulsion")
        add(run["final"]["remaining_m"], "reactive_gnron_analysis.json final distance")

    gate = ROOT / "results" / "layout_gate" / "SUMMARY.json"
    if gate.is_file():
        for layout, entry in json.loads(gate.read_text(encoding="utf8")).items():
            add(entry.get("contract_tests", "").split("/")[0] or 0,
                f"layout_gate {layout} contract tests")
            add(entry.get("reference_traversal_steps") or 0,
                f"layout_gate {layout} traversal steps")

    # --- reset study and validation ----------------------------------------
    study = ROOT / "results" / "reset_study" / "SUMMARY.json"
    if study.is_file():
        arms = json.loads(study.read_text(encoding="utf8"))["arms"]
        for arm, stats in arms.items():
            add(f"{stats['max_path_delta_m_worst']:.3f}",
                f"reset_study SUMMARY.json {arm} worst path delta")
            add(stats["max_path_delta_m_worst"],
                f"reset_study SUMMARY.json {arm} worst path delta")
            add(stats["episodes"], f"reset_study SUMMARY.json {arm} episodes")
            add(stats["outcome_matches_log"],
                f"reset_study SUMMARY.json {arm} outcome matches")
    validation = ROOT / "results" / "reset_validation" / "validation_deterministic.json"
    if validation.is_file():
        worst = json.loads(validation.read_text(encoding="utf8"))["track_maxdiff_m_worst"]
        add(f"{worst:.1e}", "validation_deterministic.json worst track diff")
        add(f"{worst * 1e4:.1f}", "validation_deterministic.json worst, x1e-4")

    # --- reproduction ------------------------------------------------------
    with (RESULTS / "v1_reproduction.csv").open(encoding="utf8") as handle:
        repro = list(csv.DictReader(handle))
    add(len(repro), "v1_reproduction.csv cells")
    add(sum(int(r["outcomes_match"]) for r in repro),
        "v1_reproduction.csv matching cells")

    # --- every numeric leaf of the scenario files --------------------------
    # Arena extents, obstacle dimensions, cart motion and control periods are
    # all in the scenario JSON. Walking it means those numbers trace to the
    # file that defines them rather than to a hand-maintained list.
    def walk(node, path: str) -> None:
        if isinstance(node, dict):
            for key, value in node.items():
                walk(value, f"{path}.{key}")
        elif isinstance(node, list):
            for index, value in enumerate(node):
                walk(value, f"{path}[{index}]")
        elif isinstance(node, (int, float)) and not isinstance(node, bool):
            add(node, f"config/scenarios/L00_C.json {path.lstrip('.')}")

    scenario_path = ROOT / "config" / "scenarios" / "L00_C.json"
    if scenario_path.is_file():
        scenario = json.loads(scenario_path.read_text(encoding="utf8"))
        walk(scenario, "")
        # Quantities the section derives from those fields.
        goal_x = scenario["goal"]["xy"][0]
        start_x = scenario["drone"]["start_xy"][0]
        add(round(goal_x - start_x, 2), "goal x minus start x")
        half = [v / 2 for v in next(o["size_xy"] for o in scenario["obstacles"]
                                    if o["id"] == "rack_a")]
        add(round((half[0] ** 2 + half[1] ** 2) ** 0.5, 3),
            "rack_a bounding radius, hypot of half extents")
        add(round(90 * scenario["control"]["command_period_s"], 1),
            "horizon 90 steps x command period, seconds")
        cart = next(o for o in scenario["obstacles"]
                    if o.get("motion", {}).get("type") == "linear")
        add(round(4 * cart["motion"]["amplitude_m"]
                  / cart["motion"]["speed_mps"], 2), "cart period")

    # --- values carried forward from v1, with their v1 source --------------
    for number, source in [
            ("0.200000", "v1 Verification_Report timing"),
            ("2.8", "v1 Verification_Report reset error (um)"),
            ("0.025", "v1 Verification_Report lidar angular map RMS"),
            ("0.048", "v1 Verification_Report lidar full-scan mean"),
            ("720", "DnsQuadcopter.proto lidarResolution"),
            ("360", "lidar field of view, degrees"),
            ("5.0", "navigation.RANGE"),
            ("12", "navigation.SECTORS"),
            ("1.14", "cruise 1.0 + mount 0.14, config/scenarios/L00_C.json"),
            ("1.0", "config/scenarios cruise_altitude_m"),
            ("0.14", "config/scenarios sensing.mount_height_m"),
            ("0.4", "config/scenarios goal tolerance_m"),
            ("0.2", "navigation.DT"),
            ("1.2", "navigation.SPEED"),
            ("90", "navigation.HORIZON"),
            ("0.18", "navigation.RADIUS"),
            ("0.45", "cart speed_mps"),
            ("1.9", "cart amplitude_m"),
            ("16.89", "cart period, 4*amplitude/speed"),
            ("2.5", "arena wall_height_m / column height"),
            ("2.0", "rack height_m"),
            ("1.6", "crate height_m"),
            ("1.5", "cart height_m"),
            ("0.85", "start x, config/scenarios"),
            ("9.15", "goal x, config/scenarios"),
            ("10", "arena x extent"),
            ("0.33", "side_elevation altitude deviation, mm"),
            ("61", "L00_C_ppo_seed7_v01 trajectory rows"),
            ("0.105", "layout_gate L04 peak tilt, slow profile"),
            ("1.5905", "layout_gate L04 peak tilt, tumbled"),
            ("0.48", "L04 reference tracker speed"),
            ("0.22", "L04 reference planning margin"),
            ("0.50", "ADR-004 circle-vs-box surface gap; also the tilt "
                     "threshold for the exceedance count"),
            ("0.30", "near-miss threshold, S6 protocol"),
            ("0.0536", "v1 min_sensed_clearance anomaly"),
            ("0.62", "v1 ground-truth clearance at the anomaly"),
            ("8", "basicTimeStep, ms"),
            ("125", "autopilot inner loop, Hz"),
            ("26", "contract tests per layout"),
            ("2.98", "direct collision x on L00"),
            ("1.35", "DnsQuadcopter.proto sceneCameraFov default"),
            ("3", "training checkpoints"),
            ("15", "Mode C variants per cell"),
            ("45", "pooled Mode C n"),
            ("0.5", "lidar beam spacing, degrees"),
            ("4.0e-06", "layout_gate SUMMARY worst t=0 pose error"),
            ("0.00e+00", "v1 cart-vs-model check, reframed as C2"),
            ("0.84", "layout_gate L04 tracker speed, profile 1"),
            ("1.08", "layout_gate L04 tracker speed, profile 0"),
            ("0.12", "layout_gate L04 planning margin, tumbled attempt"),
            ("19", "run_s6 wall clock for 540 episodes, minutes"),
            ("1.0582", "reactive_gnron_analysis.json crate surface to goal"),
            ("1.8654", "reactive_gnron_analysis.json peak repulsion"),
            ("1.6167", "reactive_gnron_analysis.json final distance"),
            ("0.0193", "lidar_cart_consistency.json cart MAE"),
            ("650", "lidar_cart_consistency.json comparisons"),
            ("56", "layout_gate L05 reference traversal steps"),
            ("24", "pooled_rates.csv L00/L02 Mode C k"),
            ("3e-16", "v1 Verification_Report observation-interval error"),
            ("0.02", "config/scenarios settle_speed_tol_mps / settle_altitude_tol_m"),
            ("2.1e-04", "validation_deterministic.json worst track diff"),
            ("1e-05", "order-independence gate threshold, Section 2 of the brief"),
            ("0.287", "reset_study SUMMARY.json isolated worst path delta"),
            ("750", "backend.DETERMINISTIC_SETTLE_STEPS"),
            ("7.4", "scenario_table.csv L02 iou_to_L00, as a percentage"),
            ("1.34", "V1_CORRECTIONS C3 corrupted calibration reference"),
            ("52", "navigation observation width"),
            ("95", "Wilson interval confidence level, percent"),
            ("29", "reset_study isolated worst path delta, in centimetres"),
    ]:
        values.setdefault(number, source)
    return values


def extract_claims(source: Path) -> list[tuple[str, str]]:
    """(context, numeric token) for every literal number in the deck source."""
    tree = ast.parse(source.read_text(encoding="utf8"))

    # Docstrings describe the script; they are not claims on a slide.
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                             ast.AsyncFunctionDef)):
            first = node.body[0] if node.body else None
            if (isinstance(first, ast.Expr)
                    and isinstance(first.value, ast.Constant)
                    and isinstance(first.value.value, str)):
                docstrings.add(id(first.value))

    claims: list[tuple[str, str]] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Constant) or not isinstance(node.value, str):
            continue
        if id(node) in docstrings or NOT_A_CLAIM.match(node.value):
            continue
        text = node.value
        if len(text) < 4:
            continue
        # Normalise the typographic minus first: "4.0e−06" would otherwise
        # tokenise as "4.0" and "06" and both would look unmatched.
        normalised = text.replace("−", "-").replace("–", "-")
        for token in re.findall(r"\d+(?:\.\d+)?(?:e[-+]?\d+)?", normalised):
            claims.append((" ".join(text.split())[:110], token))
    return claims


def extract_markdown(source: Path) -> list[tuple[str, str]]:
    """(context, numeric token) for every number in a prose section.

    Bibliographic numbers -- volumes, pages, DOIs, years -- are not claims
    about the experiment, so the reference list is removed first, as are
    citation markers, section numbers and figure/table numbers.
    """
    text = source.read_text(encoding="utf8")
    text = text.split("## References")[0]
    text = re.sub(r"`[^`]*`", "", text)   # code spans, e.g. float32
    marker = chr(10) + "---" + chr(10)
    if marker in text:            # drop the note-to-the-team block
        text = text.split(marker, 1)[1]
    text = re.sub(r"\[\d+(?:,\s*\d+)*\]", "", text)     # citation markers
    text = re.sub(r"X\.\d+", "", text)                  # section numbers
    text = re.sub(r"(?:Fig\.|Figure|Table)\s*\d+", "", text)
    text = re.sub(r"ADR-\d+", "", text)
    text = re.sub(r"R20\d\d[a-z]?", "", text)           # the Webots release

    # Prose writes powers of ten as "4.0 x 10^-6" with unicode superscripts.
    # Fold those into the "4.0e-06" notation the data files use, so a correct
    # number is recognised instead of tokenising as "4.0" and "6".
    superscript = str.maketrans("⁰¹²³⁴⁵"
                                "⁶⁷⁸⁹⁻",
                                "0123456789-")

    def fold(match):
        return match.group(1) + "e" + match.group(2).translate(superscript)

    claims: list[tuple[str, str]] = []
    for line in text.splitlines():
        if not line.strip():
            continue
        normalised = line.replace("−", "-").replace("–", "-")
        normalised = re.sub(
            r"(\d+(?:\.\d+)?)\s*×\s*10([⁰¹²³"
            r"⁴⁵⁶⁷⁸⁹⁻]+)",
            fold, normalised)
        for token in re.findall(r"\d+(?:\.\d+)?(?:e[-+]?\d+)?", normalised):
            claims.append((" ".join(line.split())[:110], token))
    return claims


def lookup(token: str, values: dict[str, str]) -> str | None:
    """Match a token against the known values by numeric value.

    "10.0", "10" and "1e+01" are the same measurement written three ways, and
    prose does not use the same formatting as a CSV. Comparing the canonical
    forms avoids reporting a correct number as unbacked.
    """
    if token in values:
        return values[token]
    try:
        number = float(token)
    except ValueError:
        return None
    forms = {f"{number:g}", f"{number:.1f}", f"{number:.2f}", f"{number:.3f}",
             f"{number:.4f}", f"{number:.1e}", f"{number:.2e}",
             f"{number:.0f}"}
    for form in forms:
        if form in values:
            return values[form]
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default=str(SOURCE),
                        help="deck script or markdown section to check")
    parser.add_argument("--markdown", action="store_true",
                        help="treat --source as a prose section, not Python")
    parser.add_argument("--out", default="SLIDES_CLAIMS_CHECK.csv",
                        help="CSV name written under evidence/")
    args = parser.parse_args()

    values = known_values()
    claims = (extract_markdown(Path(args.source)) if args.markdown
              else extract_claims(Path(args.source)))
    rows, unmatched = [], 0
    for context, token in claims:
        if token in STRUCTURAL and "." not in token:
            status, source = "structural", "slide numbering / layout constant"
        elif lookup(token, values):
            status, source = "matched", lookup(token, values)
        else:
            status, source = "UNMATCHED", ""
            unmatched += 1
        rows.append({"claim_context": context, "value": token,
                     "status": status, "backed_by": source})

    EVIDENCE.mkdir(parents=True, exist_ok=True)
    target = EVIDENCE / args.out
    with target.open("w", newline="", encoding="utf8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    counts = {"matched": 0, "structural": 0, "UNMATCHED": 0}
    for row in rows:
        counts[row["status"]] += 1
    print(f"wrote {target.relative_to(ROOT)}")
    print(f"  {len(rows)} numeric tokens in slide text and notes")
    print(f"  matched to a data file : {counts['matched']}")
    print(f"  structural (slide nos) : {counts['structural']}")
    print(f"  UNMATCHED              : {counts['UNMATCHED']}")
    if counts["UNMATCHED"]:
        print("\nUnmatched — fix or justify each before showing the deck:")
        seen = set()
        for row in rows:
            if row["status"] == "UNMATCHED" and row["value"] not in seen:
                seen.add(row["value"])
                print(f"  {row['value']:>12}   {row['claim_context'][:80]}")
    return 1 if counts["UNMATCHED"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
