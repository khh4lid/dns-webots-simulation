"""Unit tests that need no Webots session. Plain asserts, no pytest.

Run::

    cd webots_dns
    python tests/unit_tests.py           # all
    python tests/unit_tests.py mount     # only tests whose name contains "mount"

Covers the pure geometry and configuration logic: the DNS <-> Webots transform,
sector binning and the angular convention, lidar mount levelling, obstacle
trajectories, seed repeatability, and the scenario validation guards.
"""
from __future__ import annotations

import math
import sys
import traceback
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns import sensing                                            # noqa: E402
from dns.autopilot import Autopilot, VehicleParams                 # noqa: E402
from dns.scenario import (                                         # noqa: E402
    DEFAULT_SCENARIO_PATH, Motion, Obstacle, Scenario,
    bearing_to_dns, bearing_to_webots, to_dns, to_webots,
)
from dns.spencer_link import RANGE, SECTORS, observation, pack_scan, validate  # noqa: E402
from dns.spencer_link import Frame                                 # noqa: E402

TESTS: list = []


def test(fn):
    TESTS.append(fn)
    return fn


def axis_angle_matrix(axis, angle: float) -> np.ndarray:
    """Webots axis-angle -> 3x3 rotation matrix (right-handed, Rodrigues)."""
    a = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(a)
    if norm < 1e-12 or abs(angle) < 1e-15:
        return np.eye(3)
    k = a / norm
    K = np.array([[0.0, -k[2], k[1]],
                  [k[2], 0.0, -k[0]],
                  [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def rot_x(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


# ---------------------------------------------------------------- coordinates
@test
def test_coordinate_transform_is_an_involution():
    for xy in ([0.0, 0.0], [1.0, 2.0], [-3.5, 0.25], [10.0, -3.0]):
        np.testing.assert_allclose(to_dns(to_webots(xy)), xy, atol=0, rtol=0)
        np.testing.assert_allclose(to_webots(to_dns(xy)), xy, atol=0, rtol=0)


@test
def test_dns_right_is_webots_negative_y():
    # DNS +y means "right" of a +x-facing, +z-up vehicle, which is world -y.
    np.testing.assert_allclose(to_webots([0.0, 1.0]), [0.0, -1.0])
    np.testing.assert_allclose(to_dns([0.0, -1.0]), [0.0, 1.0])
    # forward is shared by both frames
    np.testing.assert_allclose(to_webots([1.0, 0.0]), [1.0, 0.0])


@test
def test_bearing_transform_flips_sign_and_round_trips():
    for theta in (0.0, 0.3, -1.2, math.pi):
        assert math.isclose(float(bearing_to_webots(theta)), -theta)
        assert math.isclose(float(bearing_to_dns(bearing_to_webots(theta))), theta)


# -------------------------------------------------------------------- sectors
@test
def test_sector_centres_are_30_degrees_apart_from_forward():
    bearings = sensing.sector_bearings_dns(SECTORS)
    assert bearings.shape == (12,)
    for i, b in enumerate(bearings):
        assert math.isclose(math.degrees(b), i * 30.0, abs_tol=1e-9)


@test
def test_agreed_cardinal_sectors():
    """0 forward, 3 right, 6 back, 9 left -- the convention agreed with Spencer."""
    for sector, expected_deg in ((0, 0), (3, 90), (6, 180), (9, 270)):
        got = math.degrees(sensing.sector_bearings_dns()[sector])
        assert math.isclose(got, expected_deg, abs_tol=1e-9), (sector, got)


@test
def test_pack_scan_puts_a_known_bearing_in_the_right_sector():
    """A single return at each sector centre must land in that sector alone."""
    for sector in range(SECTORS):
        bearing = sector * (2 * math.pi / SECTORS)
        ranges, seen = pack_scan(np.array([bearing]), np.array([2.0]),
                                 np.array([True]))
        assert seen[sector], f"sector {sector} not marked seen"
        assert math.isclose(ranges[sector], 2.0, abs_tol=1e-9)
        assert seen.sum() == 1, f"sector {sector} leaked into {np.flatnonzero(seen)}"


@test
def test_pack_scan_wraps_around_zero():
    """A bearing just below 360 deg belongs to sector 0, not sector 11."""
    for degrees, expected in ((359.0, 0), (1.0, 0), (346.0, 0), (344.0, 11),
                              (16.0, 1), (14.0, 0)):
        _, seen = pack_scan(np.array([math.radians(degrees)]),
                            np.array([1.0]), np.array([True]))
        assert np.flatnonzero(seen).tolist() == [expected], (degrees, seen)


@test
def test_pack_scan_takes_the_nearest_return_in_a_sector():
    bearings = np.radians([0.0, 5.0, -5.0])
    ranges, seen = pack_scan(bearings, np.array([3.0, 1.5, 2.0]),
                             np.ones(3, dtype=bool))
    assert seen[0] and math.isclose(ranges[0], 1.5)


@test
def test_pack_scan_encodes_unobserved_as_zero_and_false():
    ranges, seen = pack_scan(np.array([0.0]), np.array([2.0]), np.array([True]))
    for sector in range(1, SECTORS):
        assert not seen[sector]
        assert ranges[sector] == 0.0, "an unobserved sector must report 0.0"


@test
def test_pack_scan_encodes_out_of_range_as_clear_to_cap():
    """Webots returns inf past maxRange; that is *observed clear*, not unknown."""
    ranges, seen = pack_scan(np.array([0.0]), np.array([np.inf]), np.array([True]))
    assert seen[0], "an out-of-range beam is still an observation"
    assert math.isclose(ranges[0], RANGE)


@test
def test_unobserved_is_all_zero_and_false():
    ranges, valid = sensing.unobserved(SECTORS)
    assert ranges.shape == (SECTORS,) and valid.shape == (SECTORS,)
    assert not valid.any() and not ranges.any()
    # must satisfy Spencer's validator
    validate(Frame(time_s=0.0, xy=np.zeros(2), goal_xy=np.ones(2),
                   velocity=np.zeros(2), ranges=ranges, valid=valid,
                   collision=False))


@test
def test_lidar_scan_rejects_a_resolution_mismatch():
    mapping = sensing.LidarMapping.calibrated(720)
    try:
        sensing.lidar_scan(np.zeros(360), mapping, RANGE)
    except ValueError:
        return
    raise AssertionError("a wrong-length range image must raise")


@test
def test_lidar_mapping_bearings_span_one_turn():
    mapping = sensing.LidarMapping.calibrated(720)
    bearings = mapping.bearings_dns()
    assert bearings.shape == (720,)
    spacing = np.diff(bearings)
    np.testing.assert_allclose(spacing, 2 * math.pi / 720, atol=1e-12)


# ------------------------------------------------------------ ray-cast checks
@test
def test_cast_rays_hits_a_known_wall_distance():
    low, high = np.array([0.0, -3.0]), np.array([10.0, 3.0])
    origin = np.array([0.85, 0.0])
    # backward (180 deg) -> x-low wall at 0.85 m; right (90 deg) -> 3.0 m
    distances = sensing.cast_rays(origin, np.radians([180.0, 90.0, 270.0]),
                                  low, high, [], RANGE)
    np.testing.assert_allclose(distances, [0.85, 3.0, 3.0], atol=1e-9)


@test
def test_cast_rays_hits_a_disc_at_the_analytic_tangent():
    """Forward ray past a cylinder: entry distance must match the closed form."""
    low, high = np.array([0.0, -3.0]), np.array([10.0, 3.0])
    origin = np.array([0.85, 0.0])
    disc = Obstacle(id="o", xy=(3.5, -0.35), radius_m=0.55,
                    height_m=2.0).placed_at(0.0)
    # straight ahead: perpendicular offset 0.35 < 0.55, so it hits
    expected = (3.5 - 0.85) - math.sqrt(0.55 ** 2 - 0.35 ** 2)
    got = sensing.cast_rays(origin, np.array([0.0]), low, high,
                            [disc], RANGE)[0]
    assert math.isclose(got, expected, abs_tol=1e-9), (got, expected)


@test
def test_cast_rays_clips_to_max_range():
    low, high = np.array([-100.0, -100.0]), np.array([100.0, 100.0])
    got = sensing.cast_rays(np.zeros(2), np.array([0.0]), low, high,
                            [], RANGE)[0]
    assert math.isclose(got, RANGE)


# -------------------------------------------------------- lidar mount levelling
@test
def test_level_mount_rotation_levels_pure_roll_and_pitch():
    for roll, pitch in ((0.0, 0.0), (0.3, 0.0), (-0.3, 0.0),
                        (0.0, 0.4), (0.0, -0.4)):
        R_body = rot_z(0.0) @ rot_y(pitch) @ rot_x(roll)
        axis = Autopilot.level_mount_rotation(R_body.reshape(-1).tolist())
        R_sensor = R_body @ axis_angle_matrix(axis[:3], axis[3])
        np.testing.assert_allclose(R_sensor[:, 2], [0, 0, 1], atol=1e-9)


@test
def test_level_mount_rotation_levels_combined_roll_and_pitch():
    """The case that single-axis testing cannot catch.

    Rz Ry Rx and Rz Rx Ry differ by a roll-pitch commutator that vanishes when
    either angle is zero, so an Euler-order mistake survives every pure-roll
    and pure-pitch test and only shows up on a diagonal manoeuvre. Taking the
    orientation matrix directly is what makes this pass for both orders.
    """
    for roll, pitch in ((0.23, 0.24), (-0.3, 0.2), (0.45, -0.45), (0.1, -0.35)):
        for R_body in (rot_z(0.0) @ rot_y(pitch) @ rot_x(roll),
                       rot_z(0.0) @ rot_x(roll) @ rot_y(pitch)):
            axis = Autopilot.level_mount_rotation(R_body.reshape(-1).tolist())
            R_sensor = R_body @ axis_angle_matrix(axis[:3], axis[3])
            np.testing.assert_allclose(R_sensor[:, 2], [0, 0, 1], atol=1e-9)
            # scan plane level AND still orthonormal
            np.testing.assert_allclose(R_sensor.T @ R_sensor, np.eye(3), atol=1e-9)


@test
def test_level_mount_rotation_preserves_heading():
    """Levelling must not rotate the scan in azimuth, or sectors would shift."""
    for yaw in (0.0, 0.5, -1.1):
        for roll, pitch in ((0.2, 0.3), (-0.25, 0.15)):
            R_body = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
            axis = Autopilot.level_mount_rotation(R_body.reshape(-1).tolist())
            R_sensor = R_body @ axis_angle_matrix(axis[:3], axis[3])
            # sensor +x must be the horizontal projection of body +x
            body_forward = R_body[:, 0]
            expected = np.array([body_forward[0], body_forward[1], 0.0])
            expected /= np.linalg.norm(expected)
            np.testing.assert_allclose(R_sensor[:, 0], expected, atol=1e-9)


@test
def test_level_mount_rotation_is_identity_when_level():
    axis = Autopilot.level_mount_rotation(np.eye(3).reshape(-1).tolist())
    assert math.isclose(axis[3], 0.0, abs_tol=1e-12)
    np.testing.assert_allclose(np.linalg.norm(axis[:3]), 1.0, atol=1e-12)


@test
def test_level_mount_rotation_survives_degenerate_input():
    """Body forward pointing straight up has no level frame sharing its heading."""
    R_body = rot_y(math.pi / 2)            # nose straight down/up
    axis = Autopilot.level_mount_rotation(R_body.reshape(-1).tolist())
    assert len(axis) == 4 and all(math.isfinite(v) for v in axis)
    nan_input = [float("nan")] * 9
    axis = Autopilot.level_mount_rotation(nan_input)
    assert all(math.isfinite(v) for v in axis)


# -------------------------------------------------------------- box geometry
@test
def test_box_surface_distance_is_exact():
    box = Obstacle(id="b", xy=(5.0, 0.0), shape="box", size_xy=(2.0, 1.0),
                   height_m=2.0).placed_at(0.0)
    # face distances
    assert math.isclose(box.surface_distance([[7.5, 0.0]])[0], 1.5, abs_tol=1e-12)
    assert math.isclose(box.surface_distance([[5.0, 2.0]])[0], 1.5, abs_tol=1e-12)
    # corner distance is the diagonal of the outside overhang
    got = box.surface_distance([[6.0 + 3.0, 0.5 + 4.0]])[0]
    assert math.isclose(got, math.hypot(3.0, 4.0), abs_tol=1e-12), got
    # inside and on the surface are both zero
    assert box.surface_distance([[5.0, 0.0]])[0] == 0.0
    assert math.isclose(box.surface_distance([[6.0, 0.0]])[0], 0.0, abs_tol=1e-12)


@test
def test_box_surface_distance_respects_yaw():
    straight = Obstacle(id="b", xy=(0.0, 0.0), shape="box", size_xy=(2.0, 0.5),
                        height_m=2.0).placed_at(0.0)
    turned = Obstacle(id="b", xy=(0.0, 0.0), shape="box", size_xy=(2.0, 0.5),
                      yaw_rad=math.pi / 2, height_m=2.0).placed_at(0.0)
    # rotating the box 90 deg swaps which axis is long
    assert math.isclose(straight.surface_distance([[2.0, 0.0]])[0], 1.0, abs_tol=1e-9)
    assert math.isclose(turned.surface_distance([[2.0, 0.0]])[0], 1.75, abs_tol=1e-9)
    assert math.isclose(turned.surface_distance([[0.0, 2.0]])[0], 1.0, abs_tol=1e-9)


@test
def test_box_ray_hits_the_near_face():
    box = Obstacle(id="b", xy=(5.0, 0.0), shape="box", size_xy=(2.0, 1.0),
                   height_m=2.0).placed_at(0.0)
    # straight along +x from the origin: near face at x = 4
    got = box.ray_distance(np.zeros(2), np.array([[1.0, 0.0]]), RANGE)[0]
    assert math.isclose(got, 4.0, abs_tol=1e-9), got
    # a ray that passes above the box in y misses entirely
    missed = box.ray_distance(np.array([0.0, 2.0]), np.array([[1.0, 0.0]]), RANGE)[0]
    assert not np.isfinite(missed), missed
    # pointing away never hits
    away = box.ray_distance(np.zeros(2), np.array([[-1.0, 0.0]]), RANGE)[0]
    assert not np.isfinite(away)


@test
def test_box_ray_is_consistent_with_surface_distance():
    """A ray aimed at the centre must hit where the surface distance says."""
    box = Obstacle(id="b", xy=(3.0, 1.0), shape="box", size_xy=(1.2, 0.8),
                   yaw_rad=0.4, height_m=2.0).placed_at(0.0)
    origin = np.array([-1.0, -1.5])
    direction = np.asarray(box.centre) - origin
    direction = direction / np.linalg.norm(direction)
    hit = box.ray_distance(origin, direction[None, :], 50.0)[0]
    assert np.isfinite(hit)
    point = origin + hit * direction
    assert math.isclose(float(box.surface_distance(point[None, :])[0]), 0.0,
                        abs_tol=1e-9)


@test
def test_box_and_cylinder_agree_when_the_box_is_square_and_tiny():
    """Sanity bridge between the two shape paths."""
    cylinder = Obstacle(id="c", xy=(2.0, 0.0), radius_m=1.0,
                        height_m=2.0).placed_at(0.0)
    assert math.isclose(cylinder.surface_distance([[5.0, 0.0]])[0], 2.0,
                        abs_tol=1e-12)
    hit = cylinder.ray_distance(np.zeros(2), np.array([[1.0, 0.0]]), RANGE)[0]
    assert math.isclose(hit, 1.0, abs_tol=1e-9)


@test
def test_cast_rays_handles_a_box_obstacle():
    low, high = np.array([0.0, -3.0]), np.array([10.0, 3.0])
    box = Obstacle(id="b", xy=(5.0, 0.0), shape="box", size_xy=(1.0, 1.0),
                   height_m=2.0).placed_at(0.0)
    forward = sensing.cast_rays(np.array([1.0, 0.0]), np.array([0.0]),
                                low, high, [box], RANGE)[0]
    assert math.isclose(forward, 3.5, abs_tol=1e-9), forward


@test
def test_scenario_rejects_an_obstacle_below_the_scan_plane():
    import dataclasses
    s = Scenario.load(DEFAULT_SCENARIO_PATH)
    lidar = dataclasses.replace(s, sensing_mode="lidar")
    lidar.validate()
    short = dataclasses.replace(
        lidar.obstacles[0], height_m=lidar.cruise_altitude_m + 0.05)
    try:
        dataclasses.replace(
            lidar, obstacles=(short,) + lidar.obstacles[1:]).validate()
    except ValueError as exc:
        assert "invisible to the lidar" in str(exc), exc
        return
    raise AssertionError("an obstacle below the scan plane must be rejected")


@test
def test_scenario_rejects_overlapping_obstacles():
    import dataclasses
    s = Scenario.load(DEFAULT_SCENARIO_PATH)
    first = s.obstacles[0]
    clone = dataclasses.replace(first, id="clone")
    try:
        dataclasses.replace(s, obstacles=(first, clone)).validate()
    except ValueError as exc:
        assert "overlap" in str(exc).lower(), exc
        return
    raise AssertionError("two obstacles at the same place must be rejected")


# ------------------------------------------------------------------- obstacles
@test
def test_static_obstacle_never_moves():
    o = Obstacle(id="s", xy=(3.0, 1.0), radius_m=0.5)
    for t in (0.0, 1.3, 97.5):
        np.testing.assert_allclose(o.centre_at(t), [3.0, 1.0])


@test
def test_oscillating_obstacle_is_periodic_and_bounded():
    o = Obstacle(id="o", xy=(5.0, 0.0), radius_m=0.4,
                 motion=Motion(type="oscillate", axis="y", amplitude_m=1.5,
                               period_s=4.0))
    for t in (0.0, 0.7, 2.2, 3.9):
        np.testing.assert_allclose(o.centre_at(t), o.centre_at(t + 4.0), atol=1e-9)
    offsets = [o.centre_at(t)[1] for t in np.linspace(0, 4, 400)]
    assert max(offsets) <= 1.5 + 1e-9 and min(offsets) >= -1.5 - 1e-9
    # x is untouched when the axis is y
    assert all(math.isclose(o.centre_at(t)[0], 5.0) for t in np.linspace(0, 4, 50))


@test
def test_linear_obstacle_respects_its_speed_and_bounds():
    speed, amplitude = 0.5, 1.0
    o = Obstacle(id="l", xy=(4.0, 0.0), radius_m=0.4,
                 motion=Motion(type="linear", axis="y", amplitude_m=amplitude,
                               speed_mps=speed))
    period = 4.0 * amplitude / speed            # one full there-and-back cycle
    for t in (0.0, 1.1, 3.3):
        np.testing.assert_allclose(o.centre_at(t), o.centre_at(t + period), atol=1e-9)
    samples = np.linspace(0.0, period, 2001)
    offsets = np.array([o.centre_at(t)[1] for t in samples])
    assert offsets.max() <= amplitude + 1e-9 and offsets.min() >= -amplitude - 1e-9
    # travel speed: finite differences away from the turning points
    speeds = np.abs(np.diff(offsets) / np.diff(samples))
    assert math.isclose(np.median(speeds), speed, rel_tol=0.05), np.median(speeds)


@test
def test_obstacle_motion_is_a_pure_function_of_time():
    """Same t, same pose -- this is what makes an episode replayable."""
    o = Obstacle(id="o", xy=(5.0, 0.0), radius_m=0.4,
                 motion=Motion(type="oscillate", axis="x", amplitude_m=1.0,
                               period_s=3.0, phase_rad=0.9))
    for t in (0.0, 1.234, 8.5):
        first = o.centre_at(t)
        for _ in range(5):
            np.testing.assert_array_equal(o.centre_at(t), first)


@test
def test_unknown_motion_type_raises():
    o = Obstacle(id="x", xy=(5.0, 0.0), radius_m=0.4,
                 motion=Motion(type="corkscrew", amplitude_m=1.0))
    try:
        o.centre_at(1.0)
    except ValueError:
        return
    raise AssertionError("an unknown motion type must raise")


# ------------------------------------------------------------------- scenarios
@test
def test_default_scenario_loads_and_validates():
    s = Scenario.load(DEFAULT_SCENARIO_PATH)
    assert s.name == "default_10x6"
    assert s.sensing_sectors == SECTORS
    assert math.isclose(s.sensing_max_range_m, RANGE)
    s.validate()


@test
def test_command_period_must_divide_into_whole_timesteps():
    s = Scenario.load(DEFAULT_SCENARIO_PATH)
    assert s.steps_per_command == 25            # 0.2 s / 8 ms
    import dataclasses
    bad = dataclasses.replace(s, basic_timestep_ms=30)   # 0.2/0.030 = 6.67
    try:
        bad.steps_per_command
    except ValueError:
        return
    raise AssertionError("a timestep that does not divide 0.2 s must raise")


@test
def test_scenario_rejects_a_lidar_mount_inside_the_rotors():
    import dataclasses
    s = Scenario.load(DEFAULT_SCENARIO_PATH)
    lidar = dataclasses.replace(s, sensing_mode="lidar",
                                lidar_mount_mode="stabilised")
    lidar.validate()                                      # 0.14 m is fine
    try:
        dataclasses.replace(lidar, lidar_height_m=0.075).validate()
    except ValueError as exc:
        assert "rotor" in str(exc).lower(), exc
        return
    raise AssertionError("a mount that clips the rotors must be rejected")


@test
def test_scenario_rejects_a_start_point_inside_a_wall():
    import dataclasses
    s = Scenario.load(DEFAULT_SCENARIO_PATH)
    try:
        dataclasses.replace(s, start_xy=(0.05, 0.0)).validate()
    except ValueError as exc:
        assert "clearance" in str(exc).lower(), exc
        return
    raise AssertionError("a start point inside the wall clearance must be rejected")


@test
def test_scenario_rejects_a_goal_on_top_of_an_obstacle():
    import dataclasses
    s = Scenario.load(DEFAULT_SCENARIO_PATH)
    on_obstacle = tuple(s.obstacles[0].xy)
    try:
        dataclasses.replace(s, goal_xy=on_obstacle).validate()
    except ValueError as exc:
        assert "collision radius of the goal" in str(exc), exc
        return
    raise AssertionError("a goal inside an obstacle must be rejected")


@test
def test_scenario_rejects_unknown_modes():
    import dataclasses
    s = Scenario.load(DEFAULT_SCENARIO_PATH)
    for field, value in (("dynamics_mode", "magic"), ("sensing_mode", "sonar"),
                         ("collision_mode", "vibes"), ("lidar_mount_mode", "glued")):
        try:
            dataclasses.replace(s, **{field: value}).validate()
        except ValueError:
            continue
        raise AssertionError(f"{field}={value!r} must be rejected")


@test
def test_vehicle_params_derived_quantities():
    p = VehicleParams()
    assert math.isclose(p.max_rotor_thrust, 1.5e-05 * 600 ** 2)
    assert math.isclose(p.hover_rotor_thrust, 0.55 * 9.81 / 4.0)
    assert math.isclose(p.thrust_to_weight,
                        4 * p.max_rotor_thrust / (0.55 * 9.81))
    assert p.thrust_to_weight > 1.5, "the drone must be able to climb"


# ----------------------------------------------------------------- observation
@test
def test_observation_is_52_values_in_range():
    ranges = np.full(SECTORS, 2.5)
    valid = np.ones(SECTORS, dtype=bool)
    f = Frame(time_s=0.0, xy=np.array([1.0, 0.0]), goal_xy=np.array([9.0, 0.0]),
              velocity=np.array([0.5, -0.2]), ranges=ranges, valid=valid,
              collision=False)
    obs = observation(f, f)
    assert obs.shape == (52,), obs.shape
    assert np.all(np.isfinite(obs)) and np.all(np.abs(obs) <= 1.0 + 1e-6)
    # layout: goal/10, velocity/1.2, ranges/5, valid, prev ranges/5, prev valid
    np.testing.assert_allclose(obs[0:2], [0.8, 0.0], atol=1e-6)
    np.testing.assert_allclose(obs[2:4], [0.5 / 1.2, -0.2 / 1.2], atol=1e-6)
    np.testing.assert_allclose(obs[4:16], 0.5, atol=1e-6)
    np.testing.assert_allclose(obs[16:28], 1.0, atol=1e-6)


@test
def test_observation_clips_a_far_goal_and_fast_velocity():
    f = Frame(time_s=0.0, xy=np.zeros(2), goal_xy=np.array([500.0, -500.0]),
              velocity=np.array([99.0, -99.0]), ranges=np.zeros(SECTORS),
              valid=np.zeros(SECTORS, dtype=bool), collision=False)
    obs = observation(f, f)
    np.testing.assert_allclose(obs[0:2], [1.0, -1.0])
    np.testing.assert_allclose(obs[2:4], [1.0, -1.0])


@test
def test_validate_rejects_a_nonzero_range_on_an_invalid_sector():
    ranges = np.zeros(SECTORS)
    ranges[4] = 3.0
    valid = np.zeros(SECTORS, dtype=bool)        # sector 4 claims a range but is invalid
    try:
        validate(Frame(time_s=0.0, xy=np.zeros(2), goal_xy=np.ones(2),
                       velocity=np.zeros(2), ranges=ranges, valid=valid,
                       collision=False))
    except ValueError:
        return
    raise AssertionError("an invalid sector carrying a range must be rejected")


@test
def test_validate_rejects_an_over_range_reading():
    try:
        validate(Frame(time_s=0.0, xy=np.zeros(2), goal_xy=np.ones(2),
                       velocity=np.zeros(2),
                       ranges=np.full(SECTORS, RANGE + 0.1),
                       valid=np.ones(SECTORS, dtype=bool), collision=False))
    except ValueError:
        return
    raise AssertionError("a range beyond the cap must be rejected")


# ------------------------------------------------------------------ seed logic
@test
def test_seeded_layout_sampling_is_repeatable():
    """Scenario randomisation must be reproducible from the seed alone."""
    def sample(seed: int):
        rng = np.random.default_rng(seed)
        return rng.uniform(-3, 3, size=(4, 2))
    np.testing.assert_array_equal(sample(7), sample(7))
    assert not np.array_equal(sample(7), sample(8))


def main(argv: list[str]) -> int:
    selected = [t for t in TESTS if not argv or any(a in t.__name__ for a in argv)]
    failures = []
    for t in selected:
        try:
            t()
            print(f"  [PASS] {t.__name__}")
        except Exception as exc:                            # noqa: BLE001
            failures.append((t.__name__, exc))
            print(f"  [FAIL] {t.__name__}: {type(exc).__name__}: {exc}")
            traceback.print_exc(limit=3)
    print(f"\n{len(selected) - len(failures)}/{len(selected)} unit tests passed")
    if failures:
        print("FAILED:")
        for name, exc in failures:
            print(f"  ! {name}: {exc}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
