"""Measure the rotor spin-up curve without disturbing the motor state.

``resetPhysics()`` clears motor state as well as body velocity, so holding the
drone that way restarts the spin-up every tick. ``Node.setVelocity`` zeroes the
body velocity only, which keeps the drone still while the rotors spin up.

After the hold, the drone is released and sampled over a very short window, so
``|v_z|`` stays near zero and drag is negligible::

    T = m * (a_z + g)      with |v_z| ~ 0
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from controller import Supervisor  # noqa: E402

from dns.autopilot import GRAVITY, ROTOR_SPIN, VehicleParams  # noqa: E402

RESULTS = PROJECT_ROOT / "results"
HOLD_Z = 50.0
RELEASE_S = 0.08       # short enough that |v_z| stays below ~0.8 m/s


def main() -> int:
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    drone = robot.getFromDef("DNS_DRONE")
    params = VehicleParams.from_proto(drone)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    motors = [robot.getDevice(n) for n in ("m0", "m1", "m2", "m3")]
    for motor in motors:
        motor.setPosition(float("inf"))
    robot.step(timestep)

    command_total = 10.8                    # N, twice hover
    per_rotor = command_total / 4.0
    speed = float(np.sqrt(per_rotor / params.thrust_constant))
    print(f"mass {params.mass} kg, weight {params.mass * GRAVITY:.3f} N")
    print(f"command {command_total:.1f} N total = {speed:.1f} rad/s per rotor\n")

    def drive():
        for motor, spin in zip(motors, ROTOR_SPIN):
            motor.setVelocity(spin * speed)

    rows = []
    print("   hold[s]    a_z[m/s^2]   v_end[m/s]   thrust[N]   ratio")
    for hold in (0.0, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0):
        # full reset once, so every point starts from stopped rotors
        drone.getField("translation").setSFVec3f([5.0, 0.0, HOLD_Z])
        drone.getField("rotation").setSFRotation([0.0, 0.0, 1.0, 0.0])
        drone.resetPhysics()
        for motor in motors:
            motor.setVelocity(0.0)
        robot.step(timestep)

        drive()
        start = robot.getTime()
        while robot.getTime() - start < hold:
            drive()
            # hold still without touching motor state
            drone.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            drone.getField("translation").setSFVec3f([5.0, 0.0, HOLD_Z])
            if robot.step(timestep) == -1:
                return 1

        drone.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot.step(timestep)
        v0 = gps.getSpeedVector()[2]
        t0 = robot.getTime()
        while robot.getTime() - t0 < RELEASE_S:
            drive()
            if robot.step(timestep) == -1:
                return 1
        v1 = gps.getSpeedVector()[2]
        dt = robot.getTime() - t0
        accel = (v1 - v0) / dt
        thrust = params.mass * (accel + GRAVITY)
        ratio = thrust / command_total
        print(f"   {hold:6.2f}     {accel:+9.3f}    {v1:+8.3f}   {thrust:8.3f}   {ratio:6.3f}")
        rows.append({"hold_s": hold, "a_z": accel, "v_end": v1,
                     "thrust_N": thrust, "ratio": ratio})

    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / "spinup.json").write_text(json.dumps(
        {"mass_kg": params.mass, "command_total_N": command_total,
         "commanded_omega_rad_s": speed, "release_window_s": RELEASE_S,
         "samples": rows}, indent=2), encoding="utf8")
    print(f"\nwrote {RESULTS / 'spinup.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
