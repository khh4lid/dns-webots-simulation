"""Generate ``worlds/dns_arena.wbt`` from a scenario JSON file.

The world is written rather than hand-maintained so that arena size, start,
goal and obstacles always agree with ``config/*.json``. Only Webots base nodes
and the local ``DnsQuadcopter`` PROTO are used, so the world opens offline.

Run::

    python -m dns.worldgen                                  # default scenario
    python -m dns.worldgen --config config/other.json --out worlds/other.wbt
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from .scenario import DEFAULT_SCENARIO_PATH, Scenario, to_webots

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WORLD_PATH = PROJECT_ROOT / "worlds" / "dns_arena.wbt"

#: Markers sit flat on the floor so they stay below the drone's horizontal
#: lidar plane and cannot be mistaken for obstacles by the sensor pipeline.
MARKER_THICKNESS = 0.006


def _fmt(*values: float) -> str:
    return " ".join(f"{v:.6g}" for v in values)


def look_at_rotation(position, target, up=(0.0, 0.0, 1.0)) -> tuple[float, float, float, float]:
    """Webots ``Viewpoint.orientation`` that aims ``position`` at ``target``.

    A Webots viewpoint is an OpenGL camera: it looks down its own **-z** axis
    with +y up. So the camera basis is

        z_cam = -forward,   x_cam = normalize(up x z_cam),   y_cam = z_cam x x_cam

    and the orientation field is that basis as an axis-angle rotation.

    Hand-written viewpoints are easy to get wrong and the error is invisible in
    a headless run: the stored default aimed its axis along +z, which spins the
    camera about the vertical instead of tilting it down, so every rendered
    frame was empty sky. Computing it from a look-at target removes the whole
    class of mistake.
    """
    position = np.asarray(position, dtype=float)
    target = np.asarray(target, dtype=float)
    forward = target - position
    norm = float(np.linalg.norm(forward))
    if norm < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    forward /= norm

    z_cam = -forward
    up_vec = np.asarray(up, dtype=float)
    if abs(float(np.dot(up_vec, z_cam))) > 0.999:   # looking straight up/down
        up_vec = np.array([0.0, 1.0, 0.0])
    x_cam = np.cross(up_vec, z_cam)
    x_cam /= np.linalg.norm(x_cam)
    y_cam = np.cross(z_cam, x_cam)

    R = np.column_stack((x_cam, y_cam, z_cam))
    angle = math.acos(max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0)))
    if angle < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    sin_a = math.sin(angle)
    if abs(sin_a) < 1e-9:                            # angle ~ pi
        diagonal = np.clip((np.diag(R) + 1.0) / 2.0, 0.0, None)
        axis = np.sqrt(diagonal)
        axis = axis / float(np.linalg.norm(axis))
        return (float(axis[0]), float(axis[1]), float(axis[2]), angle)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    axis /= (2.0 * sin_a)
    axis /= float(np.linalg.norm(axis))
    return (float(axis[0]), float(axis[1]), float(axis[2]), float(angle))


def _solid_box(name: str, centre_w, size, colour, *, collide: bool,
               yaw: float = 0.0, roughness: float = 0.85,
               metalness: float = 0.0) -> str:
    bounding = f"""
  boundingObject Box {{
    size {_fmt(*size)}
  }}""" if collide else ""
    rotation = f"\n  rotation 0 0 1 {yaw:.6g}" if abs(yaw) > 1e-12 else ""
    return f"""
DEF {name} Solid {{
  translation {_fmt(*centre_w)}{rotation}
  name "{name}"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor {_fmt(*colour)}
        roughness {roughness:.3g}
        metalness {metalness:.3g}
      }}
      geometry Box {{
        size {_fmt(*size)}
      }}
    }}
  ]{bounding}
}}
"""


def _solid_cylinder(name: str, centre_w, radius: float, height: float, colour,
                    *, collide: bool, transparency: float = 0.0) -> str:
    bounding = f"""
  boundingObject Cylinder {{
    height {height:.6g}
    radius {radius:.6g}
  }}""" if collide else ""
    return f"""
DEF {name} Solid {{
  translation {_fmt(*centre_w)}
  name "{name}"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor {_fmt(*colour)}
        transparency {transparency:.3g}
        roughness 0.7
        metalness 0
      }}
      geometry Cylinder {{
        height {height:.6g}
        radius {radius:.6g}
      }}
    }}
  ]{bounding}
}}
"""


def build(scenario: Scenario, proto_relpath: str = "../protos/DnsQuadcopter.proto") -> str:
    low = np.asarray(scenario.low_xy, dtype=float)
    high = np.asarray(scenario.high_xy, dtype=float)

    # Arena footprint in Webots coordinates. The y flip maps an interval onto
    # itself reversed, so take the min/max again after transforming.
    corners_w = np.stack([to_webots(low), to_webots(high)])
    low_w, high_w = corners_w.min(axis=0), corners_w.max(axis=0)
    size_w = high_w - low_w
    centre_w = (low_w + high_w) / 2.0

    th = scenario.wall_thickness_m
    wh = scenario.wall_height_m

    parts: list[str] = []

    # floor: top surface exactly at z = 0
    parts.append(_solid_box(
        "DNS_FLOOR",
        (centre_w[0], centre_w[1], -0.01),
        (size_w[0], size_w[1], 0.02),
        (0.62, 0.63, 0.65),
        collide=True, roughness=0.95,
    ))

    # 1 m measurement grid, painted flat on the floor. Purely visual: no
    # collider, and at z = 0.0015 it sits far below the 1.14 m scan plane, so
    # it cannot return a lidar range. It gives the viewer a scale reference in
    # the video and in screenshots.
    if scenario.floor_grid:
        line = 0.012
        grid_colour = (0.52, 0.53, 0.56)
        for i in range(int(round(size_w[0])) + 1):
            x = low_w[0] + i
            parts.append(_solid_box(
                f"DNS_GRID_X{i}", (x, centre_w[1], 0.0015),
                (line, size_w[1], 0.003), grid_colour, collide=False))
        for j in range(int(round(size_w[1])) + 1):
            y = low_w[1] + j
            parts.append(_solid_box(
                f"DNS_GRID_Y{j}", (centre_w[0], y, 0.0015),
                (size_w[0], line, 0.003), grid_colour, collide=False))

    # walls: inner faces exactly on the arena bounds
    wall_colour = (0.45, 0.48, 0.52)
    parts.append(_solid_box(
        "DNS_WALL_XLOW", (low_w[0] - th / 2, centre_w[1], wh / 2),
        (th, size_w[1] + 2 * th, wh), wall_colour, collide=True))
    parts.append(_solid_box(
        "DNS_WALL_XHIGH", (high_w[0] + th / 2, centre_w[1], wh / 2),
        (th, size_w[1] + 2 * th, wh), wall_colour, collide=True))
    parts.append(_solid_box(
        "DNS_WALL_YLOW", (centre_w[0], low_w[1] - th / 2, wh / 2),
        (size_w[0], th, wh), wall_colour, collide=True))
    parts.append(_solid_box(
        "DNS_WALL_YHIGH", (centre_w[0], high_w[1] + th / 2, wh / 2),
        (size_w[0], th, wh), wall_colour, collide=True))

    # Start and goal pads. Flat and non-colliding by design: anything tall
    # enough to enter the scan plane would be reported as an obstacle, and a
    # collider would let the drone "crash" into its own goal marker. Each pad
    # is a filled disc inside a contrasting ring so it stays legible from the
    # overhead camera as well as from a low angle.
    start_w = to_webots(scenario.start_xy)
    goal_w = to_webots(scenario.goal_xy)
    start_radius = scenario.collision_radius_m * 2.2
    parts.append(_solid_cylinder(
        "DNS_START_RING", (start_w[0], start_w[1], MARKER_THICKNESS / 2),
        start_radius * 1.28, MARKER_THICKNESS, (0.07, 0.33, 0.15),
        collide=False))
    parts.append(_solid_cylinder(
        "DNS_START_MARKER", (start_w[0], start_w[1], MARKER_THICKNESS * 1.5),
        start_radius, MARKER_THICKNESS, (0.16, 0.66, 0.31), collide=False))
    # The goal pad is drawn at exactly the success tolerance, so what a viewer
    # sees is the real termination condition rather than a decoration.
    parts.append(_solid_cylinder(
        "DNS_GOAL_RING", (goal_w[0], goal_w[1], MARKER_THICKNESS / 2),
        scenario.goal_tolerance_m * 1.22, MARKER_THICKNESS,
        (0.06, 0.22, 0.45), collide=False))
    parts.append(_solid_cylinder(
        "DNS_GOAL_MARKER", (goal_w[0], goal_w[1], MARKER_THICKNESS * 1.5),
        scenario.goal_tolerance_m, MARKER_THICKNESS,
        (0.16, 0.47, 0.86), collide=False))

    # obstacles. The Webots solid must match the footprint the collision test
    # and the analytic ray-cast use, or the picture and the physics disagree.
    for obstacle in scenario.obstacles:
        # Write a moving obstacle at its t = 0 pose, not at its motion centre.
        # The backend repositions obstacles on reset, so a mismatch here is
        # invisible in a scripted run -- but it makes the saved world disagree
        # with the model for anyone who opens the .wbt directly, and it made
        # the calibration reference disagree with the lidar by 1.34 m.
        c_w = to_webots(obstacle.centre_at(0.0))
        name = f"DNS_OBSTACLE_{obstacle.id.upper()}"
        if obstacle.shape == "box":
            # DNS +y is Webots -y, so a yaw about +z changes sign too.
            parts.append(_solid_box(
                name, (c_w[0], c_w[1], obstacle.height_m / 2),
                (obstacle.size_xy[0], obstacle.size_xy[1], obstacle.height_m),
                obstacle.colour, collide=True, yaw=-obstacle.yaw_rad))
        else:
            parts.append(_solid_cylinder(
                name, (c_w[0], c_w[1], obstacle.height_m / 2),
                obstacle.radius_m, obstacle.height_m, obstacle.colour,
                collide=True))

    obstacle_block = "".join(parts)
    # Aim the camera at the arena centre rather than trusting a stored
    # orientation. The viewpoint position is in Webots coordinates; the target
    # is the arena centre lifted to roughly the drone's cruise height so the
    # flight volume, not the floor, sits in the middle of the frame.
    target_w = (centre_w[0], centre_w[1], scenario.cruise_altitude_m)
    vp_pos = _fmt(*scenario.viewpoint_position)
    vp_ori = _fmt(*look_at_rotation(scenario.viewpoint_position, target_w))
    follow_block = ('\n  follow "DnsDrone"\n  followSmoothness 0.3'
                    if scenario.viewpoint_follow else "")
    camera_block = (f"""
  sceneCamera TRUE
  sceneCameraWidth {scenario.camera_width}
  sceneCameraHeight {scenario.camera_height}"""
                    if scenario.camera_enabled else "")
    drone_z = scenario.cruise_altitude_m
    source = scenario.source_path.name if scenario.source_path else "<generated>"

    return f"""#VRML_SIM R2025a utf8

EXTERNPROTO "{proto_relpath}"

# ---------------------------------------------------------------------------
# Drone Navigation System - Phase 1 navigation arena
#
# GENERATED FILE. Edit config/{source} and re-run:
#     python -m dns.worldgen
#
# Scenario : {scenario.name}
# Arena    : {_fmt(*scenario.size_xy)} m, DNS bounds {list(scenario.low_xy)} -> {list(scenario.high_xy)}
# Start    : DNS {list(scenario.start_xy)} at {drone_z} m altitude
# Goal     : DNS {list(scenario.goal_xy)}, tolerance {scenario.goal_tolerance_m} m
# Frames   : DNS +x forward / +y right;  Webots ENU +x east / +y north / +z up
#            x_dns = x_webots,  y_dns = -y_webots   (see dns/scenario.py)
#
# The drone controller is "<extern>": start Webots first, then run the Python
# client (see README.md). Only base nodes and the local PROTO are referenced,
# so this world loads without network access.
# ---------------------------------------------------------------------------

WorldInfo {{
  info [
    "Drone Navigation System - Phase 1 navigation arena"
    "Scenario: {scenario.name}"
    "Owner: Khaled (Simulation Environment)"
  ]
  title "DNS Navigation Arena"
  basicTimeStep {scenario.basic_timestep_ms}
  coordinateSystem "ENU"
  defaultDamping Damping {{
    linear 0.5
    angular 0.5
  }}
}}
Viewpoint {{
  orientation {vp_ori}
  position {vp_pos}
  near 0.2{follow_block}
}}
Background {{
  skyColor [
    0.21 0.24 0.29
  ]
}}
# Key light casts the shadows that make depth and obstacle height readable;
# the two fills lift the shadowed faces so geometry stays legible on video
# without washing the scene out.
DirectionalLight {{
  ambientIntensity 0.55
  direction -0.45 0.55 -1
  intensity 2.1
  castShadows TRUE
}}
DirectionalLight {{
  ambientIntensity 0.35
  direction 0.6 -0.35 -0.8
  intensity 1.0
}}
DirectionalLight {{
  ambientIntensity 0.25
  direction 0 0 -1
  intensity 0.55
}}
{obstacle_block}
DEF DNS_DRONE DnsQuadcopter {{
  translation {_fmt(start_w[0], start_w[1], drone_z)}
  rotation 0 0 1 {scenario.fixed_yaw_rad:.6g}
  name "DnsDrone"
  controller "<extern>"
  supervisor TRUE
  collisionRadius {scenario.collision_radius_m:.6g}
  lidarMaxRange {scenario.sensing_max_range_m:.6g}
  lidarResolution {scenario.lidar_resolution}
  lidarHeight {scenario.lidar_height_m:.6g}{camera_block}
}}
"""


def write(scenario: Scenario, out: Path, proto_relpath: str = "../protos/DnsQuadcopter.proto") -> Path:
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build(scenario, proto_relpath), encoding="utf8")
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_SCENARIO_PATH)
    parser.add_argument("--out", type=Path, default=DEFAULT_WORLD_PATH)
    args = parser.parse_args(argv)

    scenario = Scenario.load(args.config)
    path = write(scenario, args.out)
    print(f"wrote {path}")
    print(f"  scenario      : {scenario.name}")
    print(f"  arena (DNS)   : {list(scenario.low_xy)} -> {list(scenario.high_xy)}")
    print(f"  start / goal  : {list(scenario.start_xy)} -> {list(scenario.goal_xy)}")
    print(f"  obstacles     : {len(scenario.obstacles)}")
    print(f"  basicTimeStep : {scenario.basic_timestep_ms} ms "
          f"({scenario.steps_per_command} steps per {scenario.command_period_s} s command)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
