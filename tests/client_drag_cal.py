"""Separate aerodynamic drag from rotor thrust.

A falling body in this world obeys ``a_z = T/m - g - c*v_z``, so thrust and the
damping coefficient ``c`` are not identifiable from one drop. Dropping with the
rotors stopped fixes ``c`` from the terminal velocity (``c = g / |v_term|``),
after which a second drop with the rotors running gives the real thrust.
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
DROP_Z = 400.0          # plenty of room to reach terminal velocity
FALL_S = 12.0


def drop(robot, drone, gps, motors, params, per_rotor, timestep):
    speed = float(np.sqrt(max(per_rotor, 0.0) / params.thrust_constant))
    drone.getField("translation").setSFVec3f([5.0, 0.0, DROP_Z])
    drone.getField("rotation").setSFRotation([0.0, 0.0, 1.0, 0.0])
    drone.resetPhysics()
    robot.step(timestep)

    trace = []
    start = robot.getTime()
    while robot.getTime() - start < FALL_S:
        for motor, spin in zip(motors, ROTOR_SPIN):
            motor.setVelocity(spin * speed)
        if robot.step(timestep) == -1:
            raise RuntimeError("simulation ended")
        trace.append((robot.getTime() - start, gps.getValues()[2], gps.getSpeedVector()[2]))
    # terminal velocity: mean over the last second
    tail = [v for t, z, v in trace if t > FALL_S - 1.0]
    return speed, float(np.mean(tail)), trace


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

    print(f"mass {params.mass} kg, weight {params.mass * GRAVITY:.3f} N")
    print(f"hover needs {params.hover_rotor_thrust:.3f} N/rotor "
          f"({np.sqrt(params.hover_rotor_thrust / params.thrust_constant):.1f} rad/s)\n")

    # ---- 1. rotors stopped -> damping coefficient
    _, v_free, trace_free = drop(robot, drone, gps, motors, params, 0.0, timestep)
    damping = GRAVITY / abs(v_free)
    print(f"[1] rotors stopped: terminal v_z = {v_free:+.4f} m/s")
    print(f"    => linear damping c = g/|v| = {damping:.4f} 1/s")
    print(f"    (world declares defaultDamping linear 0.5)\n")

    # ---- 2. rotors running -> thrust from the new terminal velocity
    rows = []
    print("[2] thrust from terminal velocity, T = m * (g + c * v_term)")
    print("    cmd/rotor  cmd total   omega    v_term     measured N   ratio")
    for per_rotor in (0.7, 1.35, 2.7, 5.4):
        speed, v_term, _ = drop(robot, drone, gps, motors, params, per_rotor, timestep)
        thrust = params.mass * (GRAVITY + damping * v_term)
        commanded = 4.0 * per_rotor
        ratio = thrust / commanded
        print(f"     {per_rotor:6.3f}   {commanded:8.3f}  {speed:6.1f}  {v_term:+8.3f}  "
              f"{thrust:10.3f}   {ratio:6.3f}")
        rows.append({"commanded_per_rotor_N": per_rotor, "commanded_total_N": commanded,
                     "omega_rad_s": speed, "v_terminal_mps": v_term,
                     "measured_total_N": thrust, "ratio": ratio})

    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / "drag_calibration.json").write_text(json.dumps({
        "mass_kg": params.mass, "thrust_constant": params.thrust_constant,
        "drop_altitude_m": DROP_Z, "fall_s": FALL_S,
        "terminal_velocity_rotors_off_mps": v_free,
        "measured_linear_damping_per_s": damping,
        "samples": rows}, indent=2), encoding="utf8")

    mean_ratio = float(np.mean([r["ratio"] for r in rows]))
    print(f"\nmean measured/commanded thrust = {mean_ratio:.4f}")
    print(f"wrote {RESULTS / 'drag_calibration.json'}")
    _ = trace_free
    return 0 if abs(mean_ratio - 1.0) < 0.15 else 1


if __name__ == "__main__":
    raise SystemExit(main())
