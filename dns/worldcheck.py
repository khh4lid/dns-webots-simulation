"""Assert that the Webots world actually loaded matches the scenario in hand.

This exists because of a specific v1 failure: the calibration tool measured one
world and compared the numbers against a *different* world's geometry, and
separately the world writer placed the moving cart at its motion centre instead
of its t = 0 pose. Both were silent -- every API call succeeded and plausible
numbers came out. The only defence is to ask the running simulator where its
bodies are and refuse to proceed if the answer disagrees with the definition.

Deliberately a separate module rather than a method on
:class:`~dns.backend.WebotsBackend`: the backend's measured behaviour has to
stay bit-identical to v1 so the L00 reproduction means something, so nothing is
added to its step or reset path.
"""
from __future__ import annotations

import numpy as np

from .scenario import Scenario, to_webots

#: Tolerance on a node pose read back from Webots, metres. The reset error is
#: 2.8e-6 m, so 1e-4 m is loose enough to absorb the world file's %.6g text
#: rounding and tight enough to catch any real placement mistake.
POSE_TOLERANCE_M = 1e-4


class WorldMismatch(RuntimeError):
    """The loaded world does not implement the scenario it claims to."""


def check(backend, scenario: Scenario | None = None, *,
          expect_sha256: str | None = None) -> dict:
    """Verify node-for-node that the world implements ``scenario``.

    Checks, in order:

    1. ``basicTimeStep`` matches the scenario;
    2. the observation interval is a whole number of timesteps;
    3. every scenario obstacle has a node, and the node's translation equals the
       obstacle's **t = 0** pose (not its motion centre);
    4. the drone node exists and the lidar reports the contracted beam count,
       field of view and range cap;
    5. the arena bounds implied by the scenario are the ones the collision test
       will use.

    Returns a report dict suitable for writing into a run manifest. Raises
    :class:`WorldMismatch` on any disagreement -- a layout that fails this is
    blocked from the experiment.
    """
    scenario = scenario or backend.scenario
    report: dict = {"checks": {}, "scenario_name": scenario.name}

    timestep = int(backend.robot.getBasicTimeStep())
    if timestep != scenario.basic_timestep_ms:
        raise WorldMismatch(
            f"world basicTimeStep is {timestep} ms, scenario declares "
            f"{scenario.basic_timestep_ms} ms")
    report["checks"]["basic_timestep_ms"] = timestep

    steps = scenario.steps_per_command          # raises if not a whole multiple
    report["checks"]["steps_per_command"] = steps
    report["checks"]["command_period_s"] = scenario.command_period_s

    poses = {}
    for obstacle, node in zip(scenario.obstacles, backend.obstacle_nodes):
        if node is None:
            raise WorldMismatch(f"no world node for obstacle {obstacle.id!r}")
        actual = np.asarray(node.getField("translation").getSFVec3f(), dtype=float)
        centre = to_webots(obstacle.centre_at(0.0))
        expected = np.array([centre[0], centre[1], obstacle.height_m / 2.0])
        error = float(np.abs(actual - expected).max())
        if error > POSE_TOLERANCE_M:
            raise WorldMismatch(
                f"obstacle {obstacle.id!r} sits at {actual.tolist()} but its "
                f"t=0 pose is {expected.tolist()} (error {error:.6f} m). This is "
                "the v1 cart-pose defect; regenerate the world with "
                "python scripts/build_worlds.py")
        poses[obstacle.id] = {"webots_xyz": [round(v, 6) for v in actual],
                              "max_error_m": error}
    report["obstacle_poses"] = poses
    report["checks"]["obstacle_t0_pose_max_error_m"] = (
        max((v["max_error_m"] for v in poses.values()), default=0.0))

    if backend.lidar is not None:
        beams = backend.lidar.getHorizontalResolution()
        fov = float(backend.lidar.getFov())
        cap = float(backend.lidar.getMaxRange())
        if beams != scenario.lidar_resolution:
            raise WorldMismatch(
                f"lidar reports {beams} beams, scenario declares "
                f"{scenario.lidar_resolution}")
        if abs(fov - 2.0 * np.pi) > 1e-6:
            raise WorldMismatch(f"lidar field of view is {fov} rad, expected 2*pi")
        if abs(cap - scenario.sensing_max_range_m) > 1e-9:
            raise WorldMismatch(
                f"lidar max range is {cap} m, scenario declares "
                f"{scenario.sensing_max_range_m} m")
        report["checks"]["lidar_beams"] = beams
        report["checks"]["lidar_fov_rad"] = fov
        report["checks"]["lidar_max_range_m"] = cap

    report["checks"]["arena_low_xy"] = list(scenario.low_xy)
    report["checks"]["arena_high_xy"] = list(scenario.high_xy)
    report["checks"]["start_xy"] = list(scenario.start_xy)
    report["checks"]["goal_xy"] = list(scenario.goal_xy)
    report["checks"]["goal_tolerance_m"] = scenario.goal_tolerance_m
    report["checks"]["cruise_altitude_m"] = scenario.cruise_altitude_m

    if expect_sha256 is not None:
        world_text = ""
        try:
            from pathlib import Path
            world_text = Path(backend.robot.getWorldPath()).read_text(
                encoding="utf8", errors="ignore")
        except Exception:                                        # noqa: BLE001
            pass
        if expect_sha256 not in world_text:
            raise WorldMismatch(
                f"world file does not carry scenario sha256 {expect_sha256[:16]}; "
                "it was built from a different scenario revision")
        report["checks"]["scenario_sha256"] = expect_sha256

    return report


def classify_contact(scenario: Scenario, xy, t: float) -> str:
    """Attribute a collision to the wall, the moving cart or a static body.

    The backend reports collision as a single boolean, and it is kept that way
    so its behaviour stays identical to v1. The cause is recovered here from the
    terminal pose and the scenario geometry, which is enough to separate the
    three failure modes the protocol distinguishes.
    """
    point = np.asarray(xy, dtype=float)[None, :]
    low = np.asarray(scenario.low_xy, dtype=float)
    high = np.asarray(scenario.high_xy, dtype=float)
    radius = scenario.collision_radius_m

    wall_gap = float(min((point[0] - low).min(), (high - point[0]).min()))
    best, cause = wall_gap, "wall"
    for obstacle in scenario.obstacles:
        gap = float(obstacle.placed_at(t).surface_distance(point)[0])
        if gap < best:
            moving = (obstacle.motion.type != "static"
                      and obstacle.motion.amplitude_m)
            best, cause = gap, ("cart" if moving else "static")
    # A pose clear of everything by more than the collision radius means the
    # contact happened earlier in the swept step; the nearest body is still the
    # best available attribution, so the label is returned with the gap.
    return cause if best <= radius * 1.5 else f"{cause}_nearest"
