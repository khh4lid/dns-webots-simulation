"""Assemble DNS_SIMULATION_RESEARCH_DELIVERABLE and verify it runs.

Only files that exist are copied; anything missing is reported rather than
silently omitted. The layout mirrors the development tree inside
01_SIMULATION/ because a Webots world resolves EXTERNPROTO relative to itself
and every script derives its root from its own location -- flattening the tree
would break both.

Run::

    python scripts/build_deliverable.py --zip
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
RESULTS = PROJECT_ROOT / "results"
EVIDENCE = PROJECT_ROOT / "evidence"

copied: list[dict] = []
missing: list[str] = []
PACKAGE = WORKSPACE / "DNS_SIMULATION_RESEARCH_DELIVERABLE"

RUN_BAT = """@echo off
REM Launch the DNS warehouse world in Webots.
REM Set WEBOTS_HOME first if Webots is not on the default path.
setlocal
if "%WEBOTS_HOME%"=="" set WEBOTS_HOME=D:\\Program files\\Webots
start "" "%WEBOTS_HOME%\\msys64\\mingw64\\bin\\webots.exe" "%~dp0worlds\\DNS_Warehouse.wbt"
echo.
echo The drone uses an ^<extern^> controller, so it will sit still until a
echo Python client attaches. From this folder, run for example:
echo.
echo   python scripts\\with_webots.py --world worlds\\DNS_Warehouse.wbt ^
echo       tests\\client_validation.py -- --config config\\scenario_warehouse.json ^
echo       --dynamics physical --sensing lidar
echo.
endlocal
"""


def put(source: Path, destination: Path, *, kind: str, produced_by: str,
        note: str = "") -> None:
    if not source.is_file():
        missing.append(f"{destination.name}  <- {source}")
        print(f"  MISSING {source}")
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    copied.append({
        "file": str(destination.relative_to(PACKAGE)).replace("\\", "/"),
        "bytes": destination.stat().st_size,
        "sha256": hashlib.sha256(destination.read_bytes()).hexdigest()[:16],
        "kind": kind, "produced_by": produced_by, "note": note,
        "source": str(source.relative_to(WORKSPACE)).replace("\\", "/"),
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
    parser.add_argument("--out", type=Path, default=PACKAGE)
    parser.add_argument("--zip", action="store_true")
    args = parser.parse_args()
    PACKAGE = args.out

    if PACKAGE.exists():
        shutil.rmtree(PACKAGE)
    PACKAGE.mkdir(parents=True)
    print(f"building {PACKAGE}\n")

    put(EVIDENCE / "README_deliverable.md", PACKAGE / "README.md",
        kind="document", produced_by="hand-written", note="read this first")

    # ------------------------------------------------------- 01 simulation
    sim = PACKAGE / "01_SIMULATION"
    print("01_SIMULATION")
    put(PROJECT_ROOT / "worlds" / "DNS_Warehouse.wbt",
        sim / "worlds" / "DNS_Warehouse.wbt", kind="source",
        produced_by="python -m dns.worldgen --config config/scenario_warehouse.json",
        note="generated from the scenario file; do not hand-edit")
    put(PROJECT_ROOT / "worlds" / "dns_arena.wbt",
        sim / "worlds" / "DNS_Baseline_10x6.wbt", kind="source",
        produced_by="python -m dns.worldgen --config config/scenario_default.json",
        note="cylinder baseline kept for PPO compatibility comparison")
    put(PROJECT_ROOT / "protos" / "DnsQuadcopter.proto",
        sim / "protos" / "DnsQuadcopter.proto", kind="source",
        produced_by="hand-written PROTO",
        note="quadcopter, gimballed lidar mount, offscreen demo camera")
    for name in ("scenario_warehouse.json", "scenario_warehouse_static.json",
                 "scenario_default.json"):
        put(PROJECT_ROOT / "config" / name, sim / "config" / name,
            kind="source", produced_by="hand-written")
    # The drone's controller field is "<extern>", so there is no in-project
    # Webots controllers/ directory to ship: the control code runs as a client
    # process and lives in dns/ + tests/. Shipping a second copy under
    # controllers/ only duplicated eight files that nothing imports.
    for script in ("with_webots.py", "verify_mp4.py", "inspect_results.py",
                   "make_deliverable_figures.py", "make_deliverable_report.py",
                   "make_verification.py", "build_deliverable.py",
                   "show_handoff.py"):
        put(PROJECT_ROOT / "scripts" / script, sim / "scripts" / script,
            kind="source", produced_by="DNS tooling")
    for test in ("client_validation.py", "client_experiments.py",
                 "client_record_demo.py", "client_moving_obstacle.py",
                 "client_lidar_attitude.py", "client_calibrate.py",
                 "client_screenshot.py", "unit_tests.py"):
        put(PROJECT_ROOT / "tests" / test, sim / "tests" / test,
            kind="source", produced_by="DNS test suite")
    # dns/ must be importable from the project root as well as controllers/
    for module in ("__init__.py", "backend.py", "autopilot.py", "sensing.py",
                   "scenario.py", "worldgen.py", "launcher.py",
                   "spencer_link.py"):
        put(PROJECT_ROOT / "dns" / module, sim / "dns" / module,
            kind="source", produced_by="DNS simulation package",
            note="import path used by scripts/ and tests/")
    (sim / "run.bat").write_text(RUN_BAT, encoding="utf8")
    print("  + 01_SIMULATION/run.bat")

    # ---------------------------------------------------- 02 demonstration
    print("\n02_DEMONSTRATION")
    put(RESULTS / "demo" / "DNS_Webots_Research_Demo.mp4",
        PACKAGE / "02_DEMONSTRATION" / "DNS_Webots_Research_Demo.mp4",
        kind="recorded evidence",
        produced_by="tests/client_record_demo.py on DNS_Warehouse.wbt",
        note="frames rendered by the in-scene Camera during real episodes; "
             "playback 1:1 with simulation time")
    put(RESULTS / "demo" / "recording_report.json",
        PACKAGE / "02_DEMONSTRATION" / "recording_report.json",
        kind="recorded evidence", produced_by="tests/client_record_demo.py",
        note="segment timeline and the true outcome of every recorded episode")

    # --------------------------------------------------------- 03 research
    print("\n03_RESEARCH")
    research = PACKAGE / "03_RESEARCH"
    put(EVIDENCE / "DNS_Simulation_Research_Report.pdf",
        research / "DNS_Simulation_Research_Report.pdf", kind="document",
        produced_by="scripts/make_deliverable_report.py",
        note="every number read from the recorded result files")
    put_tree(EVIDENCE / "deliverable_figures", research / "figures", "fig*.png",
             kind="figure from recorded data",
             produced_by="scripts/make_deliverable_figures.py",
             note="fig3 is a schematic; the rest are drawn from result files "
                  "or from real Webots renders")
    put_tree(EVIDENCE / "deliverable_tables", research / "tables", "table*.csv",
             kind="table", produced_by="scripts/make_deliverable_report.py")
    put(RESULTS / "experiments_warehouse" / "experiment_results.csv",
        research / "results.csv", kind="recorded evidence",
        produced_by="tests/client_experiments.py",
        note="one row per episode, 90 episodes")
    put(RESULTS / "experiments_warehouse" / "protocol.json",
        research / "experiment_protocol.json", kind="recorded evidence",
        produced_by="tests/client_experiments.py",
        note="seeds, denominators, stated limitations")

    # ----------------------------------------------------- 04 team handoff
    print("\n04_TEAM_HANDOFF")
    handoff = PACKAGE / "04_TEAM_HANDOFF"
    put(EVIDENCE / "PPO_Interface.md", handoff / "PPO_Interface.md",
        kind="document", produced_by="hand-written")
    put(EVIDENCE / "Sensor_Interface.md", handoff / "Sensor_Interface.md",
        kind="document", produced_by="hand-written")
    put(RESULTS / "two_frames_webots_physical_lidar.json",
        handoff / "two_frames.json", kind="recorded evidence",
        produced_by="tests/client_validation.py on DNS_Warehouse.wbt",
        note="two frames 0.2 s apart; accepted by Spencer's unmodified "
             "handoff.py verifier")
    put(RESULTS / "calibration_warehouse.json",
        handoff / "lidar_calibration.json", kind="recorded evidence",
        produced_by="tests/client_calibrate.py",
        note="measured index-to-bearing mapping")

    # ----------------------------------------------------- 05 verification
    print("\n05_VERIFICATION")
    verify = PACKAGE / "05_VERIFICATION"
    put(RESULTS / "validation_physical_lidar.json",
        verify / "validation_physical_lidar.json", kind="recorded evidence",
        produced_by="tests/client_validation.py", note="22 checks")
    put(RESULTS / "moving_obstacle_warehouse.json",
        verify / "moving_obstacle.json", kind="recorded evidence",
        produced_by="tests/client_moving_obstacle.py", note="13 checks")
    put(RESULTS / "lidar_attitude_warehouse.json",
        verify / "lidar_attitude_ablation.json", kind="recorded evidence",
        produced_by="tests/client_lidar_attitude.py",
        note="body-fixed vs gimballed mount, 29 steps each")
    put(RESULTS / "demo" / "mp4_verification.json",
        verify / "mp4_verification.json", kind="recorded evidence",
        produced_by="scripts/verify_mp4.py",
        note="MP4 container parsed; frame count and per-frame sizes")
    put(PROJECT_ROOT / "docs" / "SENSOR_ATTITUDE.md",
        verify / "SENSOR_ATTITUDE.md", kind="document",
        produced_by="hand-written from measurements",
        note="the sensor defects found, how each was measured and fixed")

    manifest = {
        "package": PACKAGE.name,
        "webots_version": "R2025a",
        "python_version": sys.version.split()[0],
        "source_workspace": str(PROJECT_ROOT),
        "scope": "Simulation environment development. PPO training and LiDAR "
                 "signal processing are teammates' contributions and are not "
                 "claimed here.",
        "artefact_kinds": {
            "recorded evidence": "produced by a real Webots run; the report's "
                                 "numbers trace back to these files",
            "figure from recorded data": "plotted from the result files, or a "
                                         "real Webots render",
            "schematic": "explanatory diagram, not measured data",
            "document": "written prose",
            "source": "code or configuration needed to run the simulation",
        },
        "files": copied,
        "missing": missing,
    }
    (verify / "evidence_manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf8")
    print("  + 05_VERIFICATION/evidence_manifest.json")

    print(f"\n{len(copied)} files copied, {len(missing)} missing")
    for item in missing:
        print(f"  ! {item}")

    if args.zip:
        archive = shutil.make_archive(str(PACKAGE), "zip",
                                      root_dir=PACKAGE.parent,
                                      base_dir=PACKAGE.name)
        print(f"\nzip: {archive}  "
              f"({Path(archive).stat().st_size / 1e6:.1f} MB)")
    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())
