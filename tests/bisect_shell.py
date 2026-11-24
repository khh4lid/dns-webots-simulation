"""Cumulative bisect from the working Propeller sample to the DNS airframe.

Both ends are generated from one parameterised template, so each step changes
exactly one thing and nothing drifts. The first step that stops producing
thrust names the responsible field.
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

WORKING = {
    "start_z": -0.01,
    "body": "helicopter",
    "bounding": "group",
    "physics": "damping",
    "helix_physics": False,
    "helix_named": False,
    "fast_helix": True,
    "fast_threshold": None,
    "motor_accel": 50,
    "motor_maxvel": 400,
    "motor_maxtorque": 15,
    "cot": (0.0122777, 0.0, 0.19),
    "helix_pos": (0.01, 0.0, 0.2),
}

STEPS = [
    ("S0_working", {}),
    ("S1_helix_phys", {"helix_physics": True, "helix_named": True}),
    ("S2_no_fasthelix", {"fast_helix": False, "fast_threshold": 50}),
    ("S3_motor_fields", {"motor_accel": None, "motor_maxvel": 600, "motor_maxtorque": 30}),
    ("S4_rotor_position", {"cot": (0.09, 0.09, 0.015), "helix_pos": (0.09, 0.09, 0.015)}),
    ("S5_cyl_bounding", {"bounding": "cylinder"}),
    ("S6_explicit_mass", {"physics": "mass"}),
    ("S7_box_body", {"body": "box"}),
    # Spawn height sweep: S7 starts resting on the platform and flies, so if the
    # break is sharp rather than gradual it is about ground contact at t=0.
    ("S8a_z0", {"start_z": 0.0}),
    ("S8b_z005", {"start_z": 0.05}),
    ("S8c_z02", {"start_z": 0.2}),
    ("S8d_z10", {"start_z": 1.0}),
    ("S8e_back_to_ground", {"start_z": -0.01}),
]


def world_text(cfg: dict, label: str) -> str:
    cot = " ".join(f"{v:g}" for v in cfg["cot"])
    hpos = " ".join(f"{v:g}" for v in cfg["helix_pos"])

    body = """    DEF BODY Pose {
      translation 0 0 0.1
      rotation -0.5773509358554485 -0.5773509358554485 -0.5773489358556708 -2.094397
      children [
        Shape {
          appearance PBRAppearance {
            baseColor 0.03 0.04 0.94
          }
          geometry Capsule {
            height 0.1
            radius 0.05
            subdivision 24
          }
        }
      ]
    }""" if cfg["body"] == "helicopter" else """    DEF BODY Pose {
      children [
        Shape {
          appearance PBRAppearance {
            baseColor 0.03 0.04 0.94
          }
          geometry Box {
            size 0.18 0.18 0.06
          }
        }
      ]
    }"""

    bounding = """  boundingObject Group {
    children [
      USE BODY
    ]
  }""" if cfg["bounding"] == "group" else """  boundingObject Cylinder {
    height 0.07
    radius 0.18
  }"""

    physics = """  physics Physics {
    damping Damping {
    }
  }""" if cfg["physics"] == "damping" else """  physics Physics {
    density -1
    mass 0.49
  }"""

    helix_extra = """
        boundingObject Cylinder {
          height 0.004
          radius 0.05
        }
        physics Physics {
          density -1
          mass 0.015
        }""" if cfg["helix_physics"] else ""
    helix_name = '\n        name "helix0"' if cfg["helix_named"] else ""

    fast = f"""
      fastHelix Solid {{
        translation {hpos}
        children [
          Shape {{
            appearance PBRAppearance {{
              baseColor 0.9 0.2 0.2
              transparency 0.6
            }}
            geometry Cylinder {{
              height 0.003
              radius 0.05
            }}
          }}
        ]
      }}""" if cfg["fast_helix"] else ""
    threshold = f"\n      fastHelixThreshold {cfg['fast_threshold']}" if cfg["fast_threshold"] else ""
    accel = f"\n        acceleration {cfg['motor_accel']}" if cfg["motor_accel"] else ""

    return f"""#VRML_SIM R2025a utf8

# bisect step: {label}
WorldInfo {{
  title "bisect {label}"
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
{body}
    DEF ROTOR Propeller {{
      shaftAxis 0 0 1
      centerOfThrust {cot}
      thrustConstants 0.00163465 0.001
      torqueConstants 1e-07 1e-07{threshold}
      device RotationalMotor {{
        name "motor"{accel}
        maxVelocity {cfg["motor_maxvel"]}
        maxTorque {cfg["motor_maxtorque"]}
      }}{fast}
      slowHelix Solid {{
        translation {hpos}{helix_name}
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
        ]{helix_extra}
      }}
    }}
  ]
  name "heli"
{bounding}
{physics}
  controller "probe_official"
  controllerArgs [
    "{label}"
    "+1"
  ]
  supervisor TRUE
}}
"""


def run(label: str, world: Path, timeout: float = 45.0) -> str:
    out = RESULTS / f"bisect_{label}.txt"
    if out.exists():
        out.unlink()
    port = find_free_port(1270)
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
    lines = out.read_text(encoding="utf8").splitlines()
    trace = [ln for ln in lines if ln.startswith("  ")]
    final = next((ln for ln in lines if ln.startswith("final z")), "EMPTY")
    first = trace[0].split()[1] if trace else "?"
    last = trace[-1].split()[1] if trace else "?"
    return f"{final}   [z {first} -> {last}]"


def main() -> int:
    RESULTS.mkdir(parents=True, exist_ok=True)
    cfg = copy.deepcopy(WORKING)
    for label, changes in STEPS:
        cfg.update(changes)
        world = WORLDS / f"_shell_{label}.wbt"
        world.write_text(world_text(cfg, label), encoding="utf8")
        verdict = run(label, world)
        # the robot starts at start_z; climbing clearly above it means thrust
        climbed = "CLIMBED" in verdict
        print(f"[{'OK  ' if climbed else 'FAIL'}] {label:20s} {changes}")
        print(f"        {verdict}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
