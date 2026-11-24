"""Isolate the four-rotor case inside the known-good shell.

The single-rotor probes fly but tumble, because an off-axis thrust with no
attitude control is unstable. A balanced quad has zero net torque, so any
altitude change is unambiguously thrust. This varies rotor count, thrust
constants and commanded speed one at a time.
"""
from __future__ import annotations

import copy
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.launcher import find_free_port, webots_binary  # noqa: E402

WORLDS = PROJECT_ROOT / "worlds"
RESULTS = PROJECT_ROOT / "results"
A = 0.09

BASE = {
    "rotors": 1,
    "kt": 0.00163465,
    "t2": 0.001,
    "speed": 100.0,
    "bounding": "group",
    "physics": "damping",
    "start_z": -0.01,
}

STEPS = [
    ("Q0_single_official", {}),
    ("Q1_quad_official_kt", {"rotors": 4}),
    ("Q2_quad_t2_zero", {"t2": 0.0}),
    ("Q3_quad_dns_kt_speed", {"kt": 0.00026, "speed": 200.0}),
    ("Q4_quad_dns_shell", {"bounding": "cylinder", "physics": "mass"}),
    # Airborne spawn sweep. Every case below is clear of the ground, so the
    # free-fall comparison is valid and the ground normal force cannot mask it.
    ("Q5a_air_z010", {"start_z": 0.10}),
    ("Q5b_air_z020", {"start_z": 0.20}),
    ("Q5c_air_z050", {"start_z": 0.50}),
    ("Q5d_air_z100", {"start_z": 1.00}),
]

ROTOR_LAYOUT = [(+A, +A, True), (+A, -A, False), (-A, +A, False), (-A, -A, True)]


def rotor_block(index: int, x: float, y: float, ccw: bool, cfg: dict) -> str:
    t1 = cfg["kt"] if ccw else -cfg["kt"]
    pos = f"{x:g} {y:g} 0.015"
    return f"""    Propeller {{
      shaftAxis 0 0 1
      centerOfThrust {pos}
      thrustConstants {t1:g} {cfg["t2"]:g}
      torqueConstants 1e-07 0
      device RotationalMotor {{
        name "m{index}"
        maxVelocity 600
        maxTorque 30
      }}
      slowHelix Solid {{
        translation {pos}
        name "helix{index}"
        children [
          Shape {{
            appearance PBRAppearance {{
              baseColor 0 0 0
            }}
            geometry Cylinder {{
              height 0.004
              radius 0.05
            }}
          }}
        ]
      }}
    }}
"""


def world_text(cfg: dict, label: str) -> str:
    rotors = "".join(rotor_block(i, x, y, ccw, cfg)
                     for i, (x, y, ccw) in enumerate(ROTOR_LAYOUT[:cfg["rotors"]]))
    bounding = ("""  boundingObject Group {
    children [
      USE BODY
    ]
  }""" if cfg["bounding"] == "group" else """  boundingObject Cylinder {
    height 0.07
    radius 0.18
  }""")
    physics = ("""  physics Physics {
    damping Damping {
    }
  }""" if cfg["physics"] == "damping" else """  physics Physics {
    density -1
    mass 0.49
  }""")
    return f"""#VRML_SIM R2025a utf8

# quad bisect: {label}
WorldInfo {{
  title "quad bisect {label}"
}}
Viewpoint {{
  orientation -0.286 0.493 0.821 1.23
  position -0.76 -1.35 1.14
}}
Background {{
  skyColor [
    0.2 0.2 0.3
  ]
}}
DirectionalLight {{
  direction 0 0 -1
  intensity 2
}}
DEF PLATFORM Solid {{
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.8 0.8 0.8
      }}
      geometry Plane {{
      }}
    }}
  ]
  boundingObject Pose {{
    children [
      Plane {{
      }}
    ]
  }}
  locked TRUE
}}
DEF HELI Robot {{
  translation 0 0 {cfg["start_z"]:g}
  children [
    GPS {{
    }}
    InertialUnit {{
    }}
    DEF BODY Pose {{
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 0.03 0.04 0.94
          }}
          geometry Box {{
            size 0.18 0.18 0.06
          }}
        }}
      ]
    }}
{rotors}  ]
  name "heli"
{bounding}
{physics}
  controller "probe_multi"
  controllerArgs [
    "{label}"
    "{cfg["speed"]:g}"
  ]
  supervisor TRUE
}}
"""


def run(label: str, world: Path, timeout: float = 45.0) -> str:
    out = RESULTS / f"qbisect_{label}.txt"
    if out.exists():
        out.unlink()
    port = find_free_port(1280)
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
        if line.startswith("verdict"):
            return line
    return "EMPTY"


def main() -> int:
    RESULTS.mkdir(parents=True, exist_ok=True)
    cfg = copy.deepcopy(BASE)
    for label, changes in STEPS:
        cfg.update(changes)
        world = WORLDS / f"_quad_{label}.wbt"
        world.write_text(world_text(cfg, label), encoding="utf8")
        verdict = run(label, world)
        flag = "OK  " if "THRUST" in verdict else "FAIL"
        print(f"[{flag}] {label:24s} {changes}")
        print(f"        {verdict}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
