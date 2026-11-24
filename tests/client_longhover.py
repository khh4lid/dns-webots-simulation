"""Does the autopilot hold altitude indefinitely, with no resets or commands?

The velocity-tracking failures appear after a roughly fixed amount of
simulation time rather than after a fixed number of tests, so this isolates
whether the drift is caused by elapsed time alone.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from controller import Supervisor  # noqa: E402

from dns.autopilot import Autopilot, Gains, VehicleParams  # noqa: E402
from dns.scenario import DEFAULT_SCENARIO_PATH, Scenario  # noqa: E402


def main() -> int:
    config = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_SCENARIO_PATH
    scenario = Scenario.load(config)
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    drone = robot.getFromDef("DNS_DRONE")
    params = VehicleParams.from_proto(drone)
    autopilot = Autopilot(robot, params, Gains(), timestep)
    robot.step(timestep)

    drone.getField("translation").setSFVec3f([6.0, 0.0, scenario.cruise_altitude_m])
    drone.getField("rotation").setSFRotation([0.0, 0.0, 1.0, 0.0])
    drone.resetPhysics()
    autopilot.reset()
    robot.step(timestep)

    print("     t[s]     z[m]     vz[m/s]   roll      pitch     yaw      T[N]   sat")
    start = robot.getTime()
    next_report = 0.0
    worst = 0.0
    while robot.getTime() - start < 40.0:
        state = autopilot.update(np.zeros(2), scenario.cruise_altitude_m)
        if robot.step(timestep) == -1:
            return 1
        elapsed = robot.getTime() - start
        worst = max(worst, abs(state.position_w[2] - scenario.cruise_altitude_m))
        if elapsed >= next_report:
            next_report += 2.0
            r, p, y = np.degrees(state.rpy)
            print(f"   {elapsed:6.2f}  {state.position_w[2]:8.4f}  {state.velocity_w[2]:+8.4f}  "
                  f"{r:+8.4f}  {p:+8.4f}  {y:+8.4f}  {state.thrust_total:6.2f}  "
                  f"{'SAT' if state.saturated else ''}")

    print(f"\nworst altitude error over 40 s: {worst:.4f} m")
    return 0 if worst < 0.05 else 1


if __name__ == "__main__":
    raise SystemExit(main())
