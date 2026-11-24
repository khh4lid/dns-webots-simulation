"""Flight test: hover hold, then commanded vs measured velocity on both axes.

Run via::

    python scripts/with_webots.py tests/client_flight.py
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from controller import Supervisor  # noqa: E402

from dns.autopilot import Autopilot, Gains, VehicleParams  # noqa: E402
from dns.scenario import DEFAULT_SCENARIO_PATH, Scenario, to_dns, to_webots  # noqa: E402

RESULTS = PROJECT_ROOT / "results"


#: Steps to discard after a supervisor teleport. The Webots GPS derives its
#: speed vector by differencing successive positions, so the step that spans a
#: jump reports a speed of (jump / timestep) -- hundreds of m/s. Feeding that to
#: the controller saturates every loop at once. Stepping a couple of times with
#: the rotors idle flushes the stale difference before control resumes.
TELEPORT_FLUSH_STEPS = 2


def place(robot, drone, autopilot, scenario, xy_dns, altitude, timestep):
    w = to_webots(xy_dns)
    drone.getField("translation").setSFVec3f([float(w[0]), float(w[1]), float(altitude)])
    drone.getField("rotation").setSFRotation([0.0, 0.0, 1.0, float(scenario.fixed_yaw_rad)])
    drone.resetPhysics()
    drone.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    for _ in range(TELEPORT_FLUSH_STEPS):
        autopilot.idle()
        if robot.step(timestep) == -1:
            raise RuntimeError("simulation ended during reposition")
    autopilot.reset()


def fly(robot, autopilot, scenario, timestep, seconds, command=(0.0, 0.0)):
    """Hold a command and return the per-step trace."""
    command = np.asarray(command, dtype=float)
    trace = []
    end = robot.getTime() + seconds
    while robot.getTime() < end:
        state = autopilot.update(command, scenario.cruise_altitude_m)
        if robot.step(timestep) == -1:
            raise RuntimeError("simulation ended")
        trace.append({
            "t": robot.getTime(),
            "z": float(state.position_w[2]),
            "v_dns": [float(state.velocity_w[0]), float(-state.velocity_w[1])],
            "rpy": state.rpy.tolist(),
            "thrust_N": float(state.thrust_total),
            "saturated": bool(state.saturated),
        })
    return trace


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_SCENARIO_PATH)
    parser.add_argument("--out", type=Path, default=RESULTS / "flight_test.json")
    parser.add_argument("--home-xy", type=float, nargs=2, default=None,
                        help="DNS position to fly each test from")
    args = parser.parse_args()

    scenario = Scenario.load(args.config)
    home = tuple(args.home_xy) if args.home_xy else scenario.start_xy
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    drone = robot.getFromDef("DNS_DRONE")
    params = VehicleParams.from_proto(drone)
    autopilot = Autopilot(robot, params, Gains(), timestep)
    robot.step(timestep)

    report: dict = {}
    failures: list[str] = []

    # ------------------------------------------------------------ hover
    print("[1] hover hold at 1.0 m from a standing start")
    place(robot, drone, autopilot, scenario, home, scenario.cruise_altitude_m, timestep)
    trace = fly(robot, autopilot, scenario, timestep, 5.0)
    settled = trace[len(trace) // 2:]
    z = np.array([s["z"] for s in settled])
    speed = np.linalg.norm([[s["v_dns"][0], s["v_dns"][1]] for s in settled], axis=1)
    rpy = np.array([s["rpy"] for s in settled])
    print(f"    altitude  mean {z.mean():.4f} m   error {z.mean() - scenario.cruise_altitude_m:+.4f} m"
          f"   spread {z.max() - z.min():.4f} m")
    print(f"    horizontal speed mean {speed.mean():.4f} m/s  max {speed.max():.4f} m/s")
    print(f"    yaw drift {np.degrees(rpy[:, 2]).max():+.4f} deg   "
          f"max |roll| {np.degrees(np.abs(rpy[:, 0])).max():.3f} deg   "
          f"max |pitch| {np.degrees(np.abs(rpy[:, 1])).max():.3f} deg")
    report["hover"] = {"altitude_mean_m": float(z.mean()),
                       "altitude_error_m": float(z.mean() - scenario.cruise_altitude_m),
                       "altitude_spread_m": float(z.max() - z.min()),
                       "horizontal_speed_mean_mps": float(speed.mean()),
                       "max_abs_yaw_deg": float(np.degrees(np.abs(rpy[:, 2])).max())}
    if abs(z.mean() - scenario.cruise_altitude_m) > 0.05:
        failures.append("hover altitude error above 5 cm")
    if speed.mean() > 0.05:
        failures.append("hover horizontal drift above 5 cm/s")
    if np.degrees(np.abs(rpy[:, 2])).max() > 2.0:
        failures.append("yaw drifted more than 2 deg")

    # -------------------------------------------------- velocity tracking
    print("\n[2] velocity tracking (command held 3.0 s, measured over the last 0.8 s)")
    print("    command [vx, vy]        measured            error       tilt(roll,pitch) deg")
    tracking = []
    for command in ([0.6, 0.0], [-0.6, 0.0], [0.0, 0.6], [0.0, -0.6],
                    [1.2, 0.0], [0.85, 0.85]):
        place(robot, drone, autopilot, scenario, home, scenario.cruise_altitude_m, timestep)
        settle = fly(robot, autopilot, scenario, timestep, 1.5)   # settle at hover
        trace = fly(robot, autopilot, scenario, timestep, 3.0, command)
        tail = trace[-int(0.8 / (timestep / 1000.0)):]
        measured = np.mean([s["v_dns"] for s in tail], axis=0)
        tilt = np.degrees(np.mean([s["rpy"][:2] for s in tail], axis=0))
        error = measured - np.asarray(command)
        saturated = any(s["saturated"] for s in tail)
        where = to_dns(np.asarray(drone.getField("translation").getSFVec3f())[:2])
        print(f"    {str(command):22s} [{measured[0]:+.3f}, {measured[1]:+.3f}]   "
              f"[{error[0]:+.3f}, {error[1]:+.3f}]   ({tilt[0]:+.2f}, {tilt[1]:+.2f})"
              f"   z {settle[-1]['z']:.2f}->{trace[-1]['z']:.2f}"
              f"  end {np.round(where, 2).tolist()}{'  SAT' if saturated else ''}")
        tracking.append({"command_mps": command, "measured_mps": measured.tolist(),
                         "error_mps": error.tolist(), "mean_tilt_deg": tilt.tolist(),
                         "saturated": saturated,
                         "z_after_settle_m": settle[-1]["z"],
                         "z_at_end_m": trace[-1]["z"],
                         "end_xy_dns": where.tolist()})
        if np.linalg.norm(error) > 0.12:
            failures.append(f"velocity error {np.linalg.norm(error):.3f} m/s for command {command}")
    report["velocity_tracking"] = tracking

    # ------------------------------------------------- step response time
    print("\n[3] step response: time to reach 90% of a 1.0 m/s forward command")
    place(robot, drone, autopilot, scenario, home, scenario.cruise_altitude_m, timestep)
    fly(robot, autopilot, scenario, timestep, 1.5)
    trace = fly(robot, autopilot, scenario, timestep, 2.5, (1.0, 0.0))
    t0 = trace[0]["t"]
    rise = next((s["t"] - t0 for s in trace if s["v_dns"][0] >= 0.9), None)
    after_one_step = next((s["v_dns"][0] for s in trace if s["t"] - t0 >= 0.2), None)
    print(f"    reaches 0.90 m/s after {rise:.3f} s" if rise else "    never reached 0.90 m/s")
    print(f"    velocity after one 0.2 s control step: {after_one_step:+.3f} m/s")
    report["step_response"] = {"rise_time_to_90pct_s": rise,
                               "velocity_after_one_command_period_mps": after_one_step}

    RESULTS.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2), encoding="utf8")
    print(f"\nwrote {args.out}")

    if failures:
        print("\nFAIL:")
        for item in failures:
            print(f"  ! {item}")
        return 1
    print("\nPASS: hover stable, both axes track their commanded velocity")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())




