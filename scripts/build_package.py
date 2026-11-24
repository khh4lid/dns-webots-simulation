"""Assemble DNS_FINAL_RESEARCH_PACKAGE from verified artefacts.

Only files that exist are copied; anything missing is reported so the package
never contains a silent gap. A manifest records, for every artefact, which run
produced it and whether it is measured evidence, a schematic, or source.

Run::

    python scripts/build_package.py
    python scripts/build_package.py --zip
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = PROJECT_ROOT.parent
sys.path.insert(0, str(PROJECT_ROOT))

RESULTS = PROJECT_ROOT / "results"
EVIDENCE = PROJECT_ROOT / "evidence"

copied: list[dict] = []
missing: list[str] = []


def put(source: Path, destination: Path, *, kind: str, produced_by: str,
        note: str = "") -> None:
    if not source.is_file():
        missing.append(f"{destination.name}  <- {source}")
        print(f"  MISSING {source}")
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    digest = hashlib.sha256(destination.read_bytes()).hexdigest()
    copied.append({
        "file": str(destination.relative_to(PACKAGE)),
        "bytes": destination.stat().st_size,
        "sha256": digest[:16],
        "kind": kind,
        "produced_by": produced_by,
        "note": note,
        "source": str(source.relative_to(WORKSPACE)),
    })
    print(f"  + {destination.relative_to(PACKAGE)}")


def put_tree(source: Path, destination: Path, pattern: str, *, kind: str,
             produced_by: str, note: str = "") -> None:
    if not source.is_dir():
        missing.append(f"{destination.name}/  <- {source}")
        print(f"  MISSING dir {source}")
        return
    for path in sorted(source.glob(pattern)):
        if path.is_file():
            put(path, destination / path.name, kind=kind,
                produced_by=produced_by, note=note)


def main() -> int:
    global PACKAGE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path,
                        default=WORKSPACE / "DNS_FINAL_RESEARCH_PACKAGE")
    parser.add_argument("--zip", action="store_true")
    args = parser.parse_args()
    PACKAGE = args.out

    if PACKAGE.exists():
        shutil.rmtree(PACKAGE)
    PACKAGE.mkdir(parents=True)
    print(f"building {PACKAGE}\n")

    # ---------------------------------------------- 01 simulation (runnable)
    sim = PACKAGE / "01_WEBOTS_SIMULATION"
    print("01_WEBOTS_SIMULATION")
    # The layout mirrors the development workspace on purpose. A Webots world
    # resolves EXTERNPROTO relative to itself ("../protos/..."), and every
    # script derives its project root from its own location, so flattening the
    # tree would break both. worlds/ and protos/ as siblings is also the
    # conventional Webots project shape.
    put(PROJECT_ROOT / "worlds" / "DNS_Research_World.wbt",
        sim / "worlds" / "DNS_Research_World.wbt", kind="source",
        produced_by="python -m dns.worldgen --config "
                    "config/scenario_research_demo.json",
        note="generated from the scenario config; do not hand-edit")
    put(PROJECT_ROOT / "protos" / "DnsQuadcopter.proto",
        sim / "protos" / "DnsQuadcopter.proto", kind="source",
        produced_by="hand-written PROTO",
        note="quadcopter, gimballed lidar mount, optional demo camera")
    put(PROJECT_ROOT / "config" / "scenario_research_demo.json",
        sim / "config" / "scenario_research_demo.json", kind="source",
        produced_by="hand-written", note="MODE C, dynamic")
    put(PROJECT_ROOT / "config" / "scenario_demo_static.json",
        sim / "config" / "scenario_demo_static.json", kind="source",
        produced_by="hand-written", note="MODE B, crossing obstacle parked")
    # The unit tests load this as the reference scenario, so the package is
    # not self-testing without it.
    put(PROJECT_ROOT / "config" / "scenario_default.json",
        sim / "config" / "scenario_default.json", kind="source",
        produced_by="hand-written",
        note="MODE A / baseline arena used by the unit tests")
    # A copy at the documented path in the delivery spec, for quick reading.
    put(PROJECT_ROOT / "config" / "scenario_research_demo.json",
        sim / "scenario_config.json", kind="source",
        produced_by="hand-written",
        note="same file as config/scenario_research_demo.json, placed here "
             "for convenience")

    for module in ("__init__.py", "backend.py", "autopilot.py", "sensing.py",
                   "scenario.py", "worldgen.py", "launcher.py",
                   "spencer_link.py"):
        put(PROJECT_ROOT / "dns" / module, sim / "dns" / module, kind="source",
            produced_by="DNS simulation package")
    for script in ("with_webots.py", "verify_mp4.py", "make_package_figures.py",
                   "make_report.py", "show_handoff.py", "build_package.py"):
        put(PROJECT_ROOT / "scripts" / script, sim / "scripts" / script,
            kind="source", produced_by="DNS tooling")
    for test in ("client_validation.py", "client_experiments.py",
                 "client_record_demo.py", "client_moving_obstacle.py",
                 "client_lidar_attitude.py", "client_calibrate.py",
                 "unit_tests.py"):
        put(PROJECT_ROOT / "tests" / test, sim / "tests" / test,
            kind="source", produced_by="DNS test suite")

    # ---------------------------------------------------------- 02 video
    print("\n02_DEMO_VIDEO")
    put(RESULTS / "demo" / "DNS_Research_Demonstration.mp4",
        PACKAGE / "02_DEMO_VIDEO" / "DNS_Research_Demonstration.mp4",
        kind="recorded evidence",
        produced_by="tests/client_record_demo.py on DNS_Research_World.wbt",
        note="frames rendered by a Webots Camera node during real episodes; "
             "playback 1:1 with simulation time")
    put(RESULTS / "demo" / "recording_report.json",
        PACKAGE / "02_DEMO_VIDEO" / "recording_report.json",
        kind="recorded evidence",
        produced_by="tests/client_record_demo.py",
        note="segment timeline and the real outcome of every recorded episode")

    # --------------------------------------------------------- 03 report
    print("\n03_RESEARCH_REPORT")
    put(EVIDENCE / "DNS_Research_Report.pdf",
        PACKAGE / "03_RESEARCH_REPORT" / "DNS_Research_Report.pdf",
        kind="document", produced_by="scripts/make_report.py",
        note="every number read from the recorded result files")

    # ------------------------------------------- 04 figures/tables/results
    print("\n04_FIGURES_TABLES_RESULTS")
    figures = PACKAGE / "04_FIGURES_TABLES_RESULTS" / "figures"
    put_tree(EVIDENCE / "package_figures", figures, "fig*.png",
             kind="figure from recorded data",
             produced_by="scripts/make_package_figures.py",
             note="fig2 is a schematic; the rest are drawn from result files")
    tables = PACKAGE / "04_FIGURES_TABLES_RESULTS" / "tables"
    put_tree(EVIDENCE / "tables", tables, "table*.csv",
             kind="table", produced_by="scripts/make_report.py")
    put(RESULTS / "experiments" / "experiment_results.csv",
        PACKAGE / "04_FIGURES_TABLES_RESULTS" / "experiment_results.csv",
        kind="recorded evidence",
        produced_by="tests/client_experiments.py",
        note="one row per episode, 90 episodes")
    put(RESULTS / "experiments" / "experiment_summary.csv",
        PACKAGE / "04_FIGURES_TABLES_RESULTS" / "experiment_summary.csv",
        kind="recorded evidence", produced_by="tests/client_experiments.py")
    put(RESULTS / "experiments" / "protocol.json",
        PACKAGE / "04_FIGURES_TABLES_RESULTS" / "experiment_protocol.json",
        kind="recorded evidence", produced_by="tests/client_experiments.py",
        note="seeds, denominators, stated limitations")

    # ------------------------------------------------------ 05 verification
    print("\n05_VERIFICATION_EVIDENCE")
    verify = PACKAGE / "05_VERIFICATION_EVIDENCE"
    put(RESULTS / "validation_physical_lidar.json",
        verify / "validation_physical_lidar.json",
        kind="recorded evidence",
        produced_by="tests/client_validation.py --dynamics physical "
                    "--sensing lidar", note="22 interface and physics checks")
    put(RESULTS / "two_frames_webots_physical_lidar.json",
        verify / "two_frames_handoff.json", kind="recorded evidence",
        produced_by="tests/client_validation.py",
        note="accepted by Spencer's unmodified handoff.py verifier")
    put(RESULTS / "moving_obstacle.json", verify / "moving_obstacle.json",
        kind="recorded evidence", produced_by="tests/client_moving_obstacle.py",
        note="12 checks on the crossing obstacle")
    put(RESULTS / "lidar_attitude.json", verify / "lidar_attitude.json",
        kind="recorded evidence", produced_by="tests/client_lidar_attitude.py",
        note="body-fixed vs gimballed mount, 29 steps each")
    put(RESULTS / "calibration.json", verify / "lidar_calibration.json",
        kind="recorded evidence", produced_by="tests/client_calibrate.py",
        note="measured index-to-bearing mapping")
    put(RESULTS / "demo" / "mp4_verification.json",
        verify / "mp4_verification.json", kind="recorded evidence",
        produced_by="scripts/verify_mp4.py",
        note="MP4 container parsed; frame count and per-frame sizes")
    put(PROJECT_ROOT / "docs" / "SENSOR_ATTITUDE.md",
        verify / "SENSOR_ATTITUDE.md", kind="document",
        produced_by="hand-written from measurements",
        note="the four sensor defects, how each was measured and fixed")

    print("\nroot")
    put(EVIDENCE / "START_HERE.md", PACKAGE / "START_HERE.md",
        kind="document", produced_by="hand-written",
        note="read this first")

    manifest = {
        "package": PACKAGE.name,
        "webots_version": "R2025a",
        "python_version": sys.version.split()[0],
        "source_workspace": str(PROJECT_ROOT),
        "artefact_kinds": {
            "recorded evidence": "produced by a real Webots run; numbers in "
                                 "the report trace back to these files",
            "figure from recorded data": "plotted from the result files in "
                                         "this package",
            "schematic": "an explanatory diagram, not measured data",
            "document": "written prose",
            "source": "code or configuration needed to run the simulation",
        },
        "files": copied,
        "missing": missing,
    }
    (PACKAGE / "05_VERIFICATION_EVIDENCE" / "evidence_manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf8")
    print(f"\n  + 05_VERIFICATION_EVIDENCE/evidence_manifest.json")

    print(f"\n{len(copied)} files copied, {len(missing)} missing")
    for item in missing:
        print(f"  ! {item}")

    if args.zip:
        archive = shutil.make_archive(str(PACKAGE), "zip",
                                      root_dir=PACKAGE.parent,
                                      base_dir=PACKAGE.name)
        size = Path(archive).stat().st_size
        print(f"\nzip: {archive}  ({size / 1e6:.1f} MB)")
    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())
