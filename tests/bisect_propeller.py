"""Bisect the gap between the working Propeller sample and the DNS airframe.

Starts from a replica of the official sample (known to fly) and applies the
differences one at a time, running each variant headless and recording whether
it still produces thrust. The first variant that stops climbing identifies the
responsible field.

Run directly (it launches Webots itself)::

    python tests/bisect_propeller.py
"""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.launcher import find_free_port, webots_binary, webots_ports_in_use  # noqa: E402

WORLDS = PROJECT_ROOT / "worlds"
RESULTS = PROJECT_ROOT / "results"
BASE = WORLDS / "_probe_official.wbt"

#: (label, description, substitutions applied cumulatively, velocity sign)
VARIANTS = [
    ("00_baseline", "official sample replica", []),
    ("00b_neg_t1_neg_w", "NEGATIVE thrustConstant driven at NEGATIVE velocity "
                         "(what every counter-rotating rotor does)",
     [("thrustConstants 0.00163465 0.001", "thrustConstants -0.00163465 0.001")], -1.0),
    ("00c_restore", "restore positive thrustConstant and velocity",
     [("thrustConstants -0.00163465 0.001", "thrustConstants 0.00163465 0.001")]),
    ("01_no_accel", "remove 'acceleration 50' from the motor",
     [("        acceleration 50\n", "")]),
    ("02_maxvel600", "maxVelocity 400 -> 600, maxTorque 15 -> 30",
     [("maxVelocity 400", "maxVelocity 600"), ("maxTorque 15", "maxTorque 30")]),
    ("03_thrust_t2_zero", "thrustConstants 0.00163465 0.001 -> 0.00163465 0",
     [("thrustConstants 0.00163465 0.001", "thrustConstants 0.00163465 0")]),
    ("04_explicit_mass", "robot physics -> density -1 / explicit mass",
     [("  physics Physics {\n    damping Damping {\n    }\n  }",
       "  physics Physics {\n    density -1\n    mass 1.338\n  }")]),
    ("05_cylinder_bound", "boundingObject Group -> single Cylinder",
     [("  boundingObject Group {\n    children [\n      USE BODY\n      USE MAIN_AXLE_LARGE\n      USE MAIN_AXLE_THIN\n    ]\n  }",
       "  boundingObject Cylinder {\n    height 0.2\n    radius 0.18\n  }")]),
    ("06_timestep8", "WorldInfo basicTimeStep 8 (controller still steps 64 ms)",
     [('  title "Propeller probe"', '  title "Propeller probe"\n  basicTimeStep 8')]),
]


def run_variant(label: str, world: Path, step_ms: int, timeout: float = 40.0) -> str:
    out = RESULTS / f"bisect_{label}.txt"
    if out.exists():
        out.unlink()
    port = find_free_port(1240)
    args = [str(webots_binary()), "--batch", "--mode=fast", "--no-rendering",
            "--minimize", f"--port={port}", str(world)]
    process = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.time() + timeout
    try:
        while time.time() < deadline:
            if out.exists():
                time.sleep(0.6)       # let the write finish
                break
            time.sleep(0.5)
    finally:
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
    if not out.exists():
        return "NO OUTPUT"
    text = out.read_text(encoding="utf8")
    for line in text.splitlines():
        if line.startswith("final z"):
            return line
    return text.strip().splitlines()[-1] if text.strip() else "EMPTY"


def main() -> int:
    RESULTS.mkdir(parents=True, exist_ok=True)
    base_text = BASE.read_text(encoding="utf8")
    print(f"other Webots instances on ports: {sorted(webots_ports_in_use())}\n")

    text = base_text
    for entry in VARIANTS:
        label, description, substitutions = entry[0], entry[1], entry[2]
        sign = entry[3] if len(entry) > 3 else 1.0
        for old, new in substitutions:
            if old not in text:
                print(f"!! variant {label}: pattern not found, aborting bisect\n   {old[:60]!r}")
                return 2
            text = text.replace(old, new)
        world = WORLDS / f"_bisect_{label}.wbt"
        world.write_text(
            text.replace('controller "probe_official"',
                         'controller "probe_official"\n  controllerArgs [\n'
                         f'    "{label}"\n    "{sign:+.0f}"\n  ]'),
            encoding="utf8")
        verdict = run_variant(label, world, 64)
        flag = "OK  " if "CLIMBED" in verdict else "FAIL"
        print(f"[{flag}] {label:20s} {description}")
        print(f"        {verdict}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
