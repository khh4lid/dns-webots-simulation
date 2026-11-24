"""Measure the simulator's sign conventions instead of assuming them.

Three things cannot be read off the documentation with certainty once a device
is mounted on a particular airframe:

1. the sign of the yaw reaction torque for a given rotor mix,
2. which way a positive tilt target actually moves the drone,
3. how a lidar horizontal index maps onto a bearing.

Each is measured here against known ground truth and written to
``results/calibration.json``. The autopilot and the sensing module then use the
measured values. Run via::

    python scripts/with_webots.py tests/client_calibrate.py
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from controller import Supervisor  # noqa: E402

from dns import sensing  # noqa: E402
from dns.autopilot import Autopilot, Gains, VehicleParams  # noqa: E402
from dns.scenario import (  # noqa: E402
    DEFAULT_SCENARIO_PATH, Scenario, to_dns, to_webots,
)

RESULTS = PROJECT_ROOT / "results"


def settle(robot, autopilot, scenario, timestep, seconds, command=(0.0, 0.0)):
    """Hold a velocity command for ``seconds`` of simulation time."""
    command = np.asarray(command, dtype=float)
    end = robot.getTime() + seconds
    while robot.getTime() < end:
        autopilot.update(command, scenario.cruise_altitude_m)
        if robot.step(timestep) == -1:
            raise RuntimeError("simulation ended during settle")
    return autopilot.state


def place(robot, drone, scenario, xy_dns, altitude):
    """Reposition the drone and clear its momentum."""
    w = to_webots(xy_dns)
    drone.getField("translation").setSFVec3f([float(w[0]), float(w[1]), float(altitude)])
    drone.getField("rotation").setSFRotation([0.0, 0.0, 1.0, float(scenario.fixed_yaw_rad)])
    drone.resetPhysics()


def main() -> int:
    # The scenario must describe the world Webots actually loaded. This used
    # to be hardcoded to the default arena, which silently compared one
    # world's lidar against another world's geometry and reported a 1.48 m
    # "calibration error" that was entirely an artefact of the mismatch.
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_SCENARIO_PATH)
    parser.add_argument("--out", type=Path, default=RESULTS / "calibration.json")
    args = parser.parse_args()
    scenario = Scenario.load(args.config)
    print(f"scenario: {scenario.name}  ({args.config.name})")

    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    drone = robot.getFromDef("DNS_DRONE")
    params = VehicleParams.from_proto(drone)
    report: dict = {"webots_basic_timestep_ms": timestep, "vehicle": {
        "mass_kg": params.mass, "inertia": list(params.inertia),
        "arm_offset_m": params.arm_offset,
        "hover_rotor_thrust_N": params.hover_rotor_thrust,
        "max_rotor_thrust_N": params.max_rotor_thrust,
        "thrust_to_weight": params.thrust_to_weight,
        "yaw_moment_per_newton_m": params.yaw_moment_per_newton}}

    print(f"vehicle: m={params.mass} kg  hover/rotor={params.hover_rotor_thrust:.4f} N  "
          f"max/rotor={params.max_rotor_thrust:.3f} N  TWR={params.thrust_to_weight:.2f}")

    autopilot = Autopilot(robot, params, Gains(), timestep)
    robot.step(timestep)      # populate sensors

    # ---------------------------------------------------------------- 1. yaw
    # Drive a pure yaw differential: rotors 0 and 3 (CCW) heavy, 1 and 2 (CW)
    # light. In the mixing convention of autopilot.py this is a positive tau_z
    # command. Measure which way the body actually rotates.
    print("\n[1] yaw reaction torque sign")
    place(robot, drone, scenario, (5.0, 0.0), scenario.cruise_altitude_m)
    autopilot.reset()
    settle(robot, autopilot, scenario, timestep, 1.5)
    hover = params.hover_rotor_thrust
    delta = 0.25 * hover
    autopilot.set_rotor_thrusts([hover + delta, hover - delta, hover - delta, hover + delta])
    yaw_start = robot.getFromDef("DNS_DRONE")
    start_rpy = np.asarray(autopilot.imu.getRollPitchYaw(), dtype=float)
    burst_end = robot.getTime() + 0.5
    while robot.getTime() < burst_end:
        autopilot.set_rotor_thrusts([hover + delta, hover - delta, hover - delta, hover + delta])
        if robot.step(timestep) == -1:
            raise RuntimeError("simulation ended")
    end_rpy = np.asarray(autopilot.imu.getRollPitchYaw(), dtype=float)
    yaw_rate = float(autopilot.gyro.getValues()[2])
    yaw_change = float(end_rpy[2] - start_rpy[2])
    # A positive tau_z command that produces a positive yaw rate means the
    # assumed mixing sign is already correct.
    yaw_mix_sign = 1.0 if yaw_rate > 0 else -1.0
    print(f"    rotors(0,3) heavy -> yaw rate {yaw_rate:+.4f} rad/s, "
          f"yaw change {yaw_change:+.4f} rad over 0.5 s")
    print(f"    => yaw_mix_sign = {yaw_mix_sign:+.0f}")
    report["yaw"] = {"measured_yaw_rate_rad_s": yaw_rate,
                     "measured_yaw_change_rad": yaw_change,
                     "yaw_mix_sign": yaw_mix_sign,
                     "note": "rotors 0 and 3 (CCW) heavy = positive tau_z in the autopilot mix"}
    _ = yaw_start

    # Rebuild the autopilot with the measured sign for the remaining tests.
    gains = Gains(yaw_mix_sign=yaw_mix_sign)
    autopilot = Autopilot(robot, params, gains, timestep)

    # ------------------------------------------------------- 2. axis response
    print("\n[2] commanded axis -> measured motion")
    axis_report = {}
    for label, command in (("+x_forward", (0.6, 0.0)), ("+y_right", (0.0, 0.6))):
        place(robot, drone, scenario, (5.0, 0.0), scenario.cruise_altitude_m)
        autopilot.reset()
        settle(robot, autopilot, scenario, timestep, 1.5)
        before = to_dns(np.asarray(drone.getField("translation").getSFVec3f())[:2])
        settle(robot, autopilot, scenario, timestep, 1.5, command)
        state = autopilot.state
        after = to_dns(np.asarray(drone.getField("translation").getSFVec3f())[:2])
        moved = after - before
        measured = np.array([state.velocity_w[0], -state.velocity_w[1]])
        ok = bool(np.dot(moved, np.asarray(command)) > 0
                  and np.dot(measured, np.asarray(command)) > 0)
        print(f"    cmd {label:11s} {list(command)} -> moved {np.round(moved, 3).tolist()} m, "
              f"v_meas {np.round(measured, 3).tolist()} m/s  {'OK' if ok else 'WRONG SIGN'}")
        axis_report[label] = {"command_mps": list(command),
                              "displacement_dns_m": moved.tolist(),
                              "measured_velocity_dns_mps": measured.tolist(),
                              "direction_correct": ok}
    report["axes"] = axis_report

    # ------------------------------------------------------------- 3. lidar
    print("\n[3] lidar index -> bearing mapping")
    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)
    # An asymmetric pose makes the wall distances distinct on every side, so the
    # fit has a unique solution.
    probe_xy = (1.6, 1.1)
    place(robot, drone, scenario, probe_xy, scenario.cruise_altitude_m)
    autopilot.reset()
    settle(robot, autopilot, scenario, timestep, 2.5)
    actual_xy = to_dns(np.asarray(drone.getField("translation").getSFVec3f())[:2])
    robot.step(timestep)
    scan = np.asarray(lidar.getRangeImage(), dtype=float)
    low = np.asarray(scenario.low_xy)
    high = np.asarray(scenario.high_xy)
    placed = scenario.placed_at(0.0)
    mapping = sensing.fit_lidar_mapping(scan, actual_xy, low, high, placed,
                                        scenario.sensing_max_range_m)
    print(f"    drone at DNS {np.round(actual_xy, 3).tolist()}, {len(scan)} beams, "
          f"{int(np.isfinite(scan).sum())} finite")
    print(f"    direction={mapping.direction:+.0f}  offset={math.degrees(mapping.offset):+.2f} deg  "
          f"rms={mapping.rms_error_m:.4f} m")
    report["lidar"] = mapping.to_dict()
    report["lidar"]["probe_position_dns"] = actual_xy.tolist()
    report["lidar"]["finite_beams"] = int(np.isfinite(scan).sum())

    # cross-check: sector scan from the real lidar vs the analytic ray-cast
    measured_ranges, measured_valid = sensing.lidar_scan(
        scan, mapping, scenario.sensing_max_range_m)
    truth_ranges, _ = sensing.geometric_scan(actual_xy, low, high, placed,
                                             scenario.sensing_max_range_m)
    residual = measured_ranges - truth_ranges
    print("    sector | lidar  | geometric | diff")
    for i in range(len(measured_ranges)):
        print(f"      {i:2d}   | {measured_ranges[i]:6.3f} | {truth_ranges[i]:9.3f} "
              f"| {residual[i]:+.3f}  valid={bool(measured_valid[i])}")
    print(f"    max |diff| = {np.abs(residual).max():.4f} m")
    report["lidar"]["sector_check"] = {
        "lidar_ranges_m": measured_ranges.tolist(),
        "geometric_ranges_m": truth_ranges.tolist(),
        "max_abs_diff_m": float(np.abs(residual).max())}

    RESULTS.mkdir(parents=True, exist_ok=True)
    report["scenario"] = scenario.name
    report["config"] = str(args.config)
    out = args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2), encoding="utf8")
    print(f"\nwrote {out}")

    ok = (all(v["direction_correct"] for v in axis_report.values())
          and mapping.rms_error_m < 0.15)
    print("PASS" if ok else "FAIL: see the entries above")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
