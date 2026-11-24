"""Second bisect: close the gap between the working single-rotor probe and the
failing four-rotor probe.

Varies the controller step, whether the velocity command is re-sent every tick,
and whether the Propeller has a ``fastHelix`` defined.
"""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.launcher import find_free_port, webots_binary  # noqa: E402

WORLDS = PROJECT_ROOT / "worlds"
RESULTS = PROJECT_ROOT / "results"
BASE = WORLDS / "_probe_propeller.wbt"

FAST_HELIX_BLOCK = """      fastHelix Solid {
        translation %s
        name "fast%s"
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 0.9 0.2 0.2
              transparency 0.7
            }
            geometry Cylinder {
              height 0.004
              radius 0.05
            }
          }
        ]
      }
"""

#: label -> (step_ms, resend, add_fast_helix)
CASES = [
    ("quad_a_step8_nosend", 8, 0, False),
    ("quad_b_step8_resend", 8, 1, False),
    ("quad_c_step32_resend", 32, 1, False),
    ("quad_d_step8_fasthelix", 8, 0, True),
    ("quad_e_step32_fasthelix_resend", 32, 1, True),
]


def add_fast_helix(text: str) -> str:
    """Insert a fastHelix next to each slowHelix."""
    out = []
    for line in text.splitlines(keepends=True):
        stripped = line.strip()
        if stripped.startswith("slowHelix DEF HELIX"):
            index = stripped.split("HELIX")[1].split()[0]
            # the helix translation repeats the centerOfThrust of that rotor
            position = {"0": "0.09 0.09 0.015", "1": "0.09 -0.09 0.015",
                        "2": "-0.09 0.09 0.015", "3": "-0.09 -0.09 0.015"}[index]
            out.append(FAST_HELIX_BLOCK % (position, index))
        out.append(line)
    return "".join(out)


def run(label: str, world: Path, args: list[str], timeout: float = 45.0) -> str:
    out = RESULTS / f"{label}.txt"
    if out.exists():
        out.unlink()
    port = find_free_port(1250)
    process = subprocess.Popen(
        [str(webots_binary()), "--batch", "--mode=fast", "--no-rendering",
         "--minimize", f"--port={port}", str(world)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.time() + timeout
    try:
        while time.time() < deadline:
            if out.exists():
                time.sleep(0.6)
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
    for line in out.read_text(encoding="utf8").splitlines():
        if line.startswith("final z"):
            return line
    return "EMPTY"


def main() -> int:
    RESULTS.mkdir(parents=True, exist_ok=True)
    base_text = BASE.read_text(encoding="utf8")
    for label, step, resend, fast in CASES:
        text = add_fast_helix(base_text) if fast else base_text
        text = text.replace(
            '  controller "probe_propeller"',
            '  controller "probe_propeller"\n  controllerArgs [\n'
            f'    "{label}"\n    "{step}"\n    "{resend}"\n  ]')
        world = WORLDS / f"_bisect_{label}.wbt"
        world.write_text(text, encoding="utf8")
        verdict = run(label, world, [])
        flag = "OK  " if "CLIMBED" in verdict else "FAIL"
        print(f"[{flag}] {label:32s} step={step:<3} resend={resend} fastHelix={fast}")
        print(f"        {verdict}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
