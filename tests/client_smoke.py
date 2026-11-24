"""Smoke test: attach to the running arena and report what Webots actually has.

Run via::

    python scripts/with_webots.py tests/client_smoke.py

This verifies the world parsed, the PROTO instantiated, every device exists and
the supervisor can see the scene tree. It does not fly the drone.
"""
from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from controller import Supervisor  # noqa: E402  (env is prepared by with_webots.py)

EXPECTED_DEVICES = ("gps", "gyro", "imu", "lidar", "m0", "m1", "m2", "m3")
EXPECTED_DEFS = ("DNS_DRONE", "DNS_FLOOR", "DNS_WALL_XLOW", "DNS_WALL_XHIGH",
                 "DNS_WALL_YLOW", "DNS_WALL_YHIGH", "DNS_START_MARKER",
                 "DNS_GOAL_MARKER", "DNS_OBSTACLE_OBS_A", "DNS_OBSTACLE_OBS_B")


def main() -> int:
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    print(f"connected: name={robot.getName()!r} basicTimeStep={timestep} ms")

    failures: list[str] = []

    print(f"\ndevices ({robot.getNumberOfDevices()} reported):")
    present = {robot.getDeviceByIndex(i).getName() for i in range(robot.getNumberOfDevices())}
    for name in sorted(present):
        print(f"  - {name}")
    for name in EXPECTED_DEVICES:
        if name not in present:
            failures.append(f"missing device {name!r}")

    print("\nscene tree DEF nodes:")
    for name in EXPECTED_DEFS:
        node = robot.getFromDef(name)
        if node is None:
            failures.append(f"missing DEF {name!r}")
            print(f"  - {name:24s} MISSING")
        else:
            field = node.getField("translation")
            where = field.getSFVec3f() if field else None
            print(f"  - {name:24s} type={node.getTypeName():16s} translation={where}")

    # PROTO parameters are readable through the DEF node, so the Python side can
    # take its vehicle constants from the same source the simulator used.
    drone = robot.getFromDef("DNS_DRONE")
    if drone is not None:
        print("\nDnsQuadcopter PROTO parameters:")
        for name in ("mass", "armOffset", "rotorRadius", "collisionRadius",
                     "thrustConstant", "torqueConstant", "motorMaxVelocity",
                     "lidarMaxRange", "lidarResolution"):
            field = drone.getField(name)
            if field is None:
                failures.append(f"PROTO parameter {name!r} not exposed")
                print(f"  - {name:18s} MISSING")
                continue
            getter = field.getSFFloat if name != "lidarResolution" else field.getSFInt32
            print(f"  - {name:18s} = {getter()}")

    # one simulation step proves physics and the extern link are both live
    if robot.step(timestep) == -1:
        failures.append("robot.step returned -1 on the first step")
    print(f"\nafter one step: getTime()={robot.getTime():.6f} s")

    if failures:
        print("\nFAIL:")
        for item in failures:
            print(f"  ! {item}")
        return 1
    print("\nPASS: world, PROTO, devices and supervisor access all present")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
