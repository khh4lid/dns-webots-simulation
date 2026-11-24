"""Isolate rotor thrust from the controller.

Stage 1 drives all four motors open-loop at a fixed speed that should give a
thrust-to-weight well above 1 and watches the altitude. Stage 2 runs the
autopilot and dumps its internals. Whichever stage fails locates the fault.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from controller import Supervisor  # noqa: E402

from dns.autopilot import GRAVITY, Autopilot, Gains, VehicleParams  # noqa: E402
from dns.scenario import load_default  # noqa: E402


def main() -> int:
    scenario = load_default()
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    drone = robot.getFromDef("DNS_DRONE")
    params = VehicleParams.from_proto(drone)
    weight = params.mass * GRAVITY
    print(f"mass={params.mass} kg  weight={weight:.3f} N  k_t={params.thrust_constant}")

    gps = robot.getDevice("gps")
    gps.enable(timestep)
    imu = robot.getDevice("imu")
    imu.enable(timestep)
    motors = [robot.getDevice(n) for n in ("m0", "m1", "m2", "m3")]
    for motor in motors:
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
    robot.step(timestep)

    # ---------------------------------------------------- stage 1: open loop
    speed = 400.0
    per_rotor = params.thrust_constant * speed ** 2
    print(f"\n[stage 1] open loop |w|={speed} rad/s -> {per_rotor:.3f} N/rotor, "
          f"{4 * per_rotor:.3f} N total (TWR {4 * per_rotor / weight:.2f})")
    drone.getField("translation").setSFVec3f([5.0, 0.0, 1.0])
    drone.getField("rotation").setSFRotation([0.0, 0.0, 1.0, 0.0])
    drone.resetPhysics()
    robot.step(timestep)

    for spin, motor in zip((+1, -1, -1, +1), motors):
        motor.setVelocity(spin * speed)

    start = robot.getTime()
    print("     t[s]   z[m]    vz[m/s]  roll     pitch    yaw      motor vel")
    while robot.getTime() - start < 6.0:
        # Re-issue the command every tick: the autopilot does, and that is the
        # only difference between this stage and the one that flies.
        for spin, motor in zip((+1, -1, -1, +1), motors):
            motor.setVelocity(spin * speed)
        if robot.step(timestep) == -1:
            return 1
        elapsed = robot.getTime() - start
        if abs(elapsed * 2 - round(elapsed * 2)) < 1e-6:
            pos = gps.getValues()
            vel = gps.getSpeedVector()
            rpy = imu.getRollPitchYaw()
            vels = [f"{m.getVelocity():+6.1f}" for m in motors]
            print(f"    {elapsed:5.2f}  {pos[2]:6.3f}  {vel[2]:+7.3f}  "
                  f"{rpy[0]:+7.4f} {rpy[1]:+7.4f} {rpy[2]:+7.4f}  {' '.join(vels)}")

    climbed = gps.getValues()[2] > 1.05
    print(f"    open-loop result: {'CLIMBED (propellers OK)' if climbed else 'DID NOT CLIMB'}")

    # --------------------------------------------------- stage 2: autopilot
    print("\n[stage 2] autopilot hover")
    autopilot = Autopilot(robot, params, Gains(), timestep)
    drone.getField("translation").setSFVec3f([5.0, 0.0, 1.0])
    drone.getField("rotation").setSFRotation([0.0, 0.0, 1.0, 0.0])
    drone.resetPhysics()
    autopilot.reset()
    robot.step(timestep)

    start = robot.getTime()
    print("     t[s]   z[m]    vz[m/s]  T_tot[N]  thrusts[N]                  sat")
    while robot.getTime() - start < 8.0:
        state = autopilot.update(np.zeros(2), scenario.cruise_altitude_m)
        if robot.step(timestep) == -1:
            return 1
        elapsed = robot.getTime() - start
        if abs(elapsed * 2 - round(elapsed * 2)) < 1e-6:
            thrusts = " ".join(f"{t:5.2f}" for t in state.rotor_thrusts)
            print(f"    {elapsed:5.2f}  {state.position_w[2]:6.3f}  {state.velocity_w[2]:+7.3f}  "
                  f"{state.thrust_total:8.3f}  [{thrusts}]  {state.saturated}")

    held = abs(autopilot.state.position_w[2] - scenario.cruise_altitude_m) < 0.1
    print(f"    autopilot result: {'HOLDS ALTITUDE' if held else 'DOES NOT HOLD ALTITUDE'}")
    return 0 if (climbed and held) else 1


if __name__ == "__main__":
    raise SystemExit(main())
