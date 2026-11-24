"""Measure actual rotor thrust in free air, and the rotor spin-up transient.

Two effects confound a naive drop test:

* the world applies ``defaultDamping linear 0.5``, so a fast-falling body feels
  several newtons of drag. The vertical equation is therefore
  ``a_z = T/m - g - c*v_z`` and thrust must be recovered as
  ``T = m * (a_z + g + c * v_z)``;
* the rotor takes time to reach the commanded speed, so a measurement taken
  immediately after the command reads low.

The drone is held at altitude with zero velocity while the rotors spin up, then
released and sampled over a short window where ``|v_z|`` is still small. Sweeping
the hold time exposes the spin-up curve; sweeping the command gives the thrust
curve.
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
from dns.scenario import load_default  # noqa: E402

RESULTS = PROJECT_ROOT / "results"
HOLD_ALTITUDE = 40.0
#: WorldInfo defaultDamping.linear in worlds/dns_arena.wbt.
LINEAR_DAMPING = 0.5
MEASURE_S = 0.15


class Rig:
    def __init__(self, robot, drone, params):
        self.robot = robot
        self.drone = drone
        self.params = params
        self.timestep = int(robot.getBasicTimeStep())
        self.gps = robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.motors = [robot.getDevice(n) for n in ("m0", "m1", "m2", "m3")]
        for motor in self.motors:
            motor.setPosition(float("inf"))

    def drive(self, per_rotor: float) -> float:
        speed = float(np.sqrt(max(per_rotor, 0.0) / self.params.thrust_constant))
        for motor, spin in zip(self.motors, ROTOR_SPIN):
            motor.setVelocity(spin * speed)
        return speed

    def pin(self):
        self.drone.getField("translation").setSFVec3f([5.0, 0.0, HOLD_ALTITUDE])
        self.drone.getField("rotation").setSFRotation([0.0, 0.0, 1.0, 0.0])
        self.drone.resetPhysics()

    def measure(self, per_rotor: float, hold_s: float) -> dict:
        """Hold while the rotors spin up, release, then sample."""
        self.pin()
        self.robot.step(self.timestep)
        speed = self.drive(per_rotor)

        start = self.robot.getTime()
        while self.robot.getTime() - start < hold_s:
            self.pin()                      # keeps velocity at zero
            self.drive(per_rotor)
            if self.robot.step(self.timestep) == -1:
                raise RuntimeError("simulation ended")

        v0 = self.gps.getSpeedVector()[2]
        t0 = self.robot.getTime()
        speeds = []
        while self.robot.getTime() - t0 < MEASURE_S:
            self.drive(per_rotor)
            if self.robot.step(self.timestep) == -1:
                raise RuntimeError("simulation ended")
            speeds.append(self.gps.getSpeedVector()[2])
        v1 = speeds[-1]
        dt = self.robot.getTime() - t0

        accel = (v1 - v0) / dt
        v_mean = float(np.mean([v0] + speeds))
        thrust = self.params.mass * (accel + GRAVITY + LINEAR_DAMPING * v_mean)
        return {"commanded_per_rotor_N": per_rotor, "commanded_total_N": 4 * per_rotor,
                "hold_s": hold_s, "omega_rad_s": speed, "a_z": accel,
                "v_mean": v_mean, "measured_total_N": thrust,
                "ratio": thrust / (4 * per_rotor) if per_rotor else float("nan")}


def main() -> int:
    scenario = load_default()
    robot = Supervisor()
    drone = robot.getFromDef("DNS_DRONE")
    params = VehicleParams.from_proto(drone)
    rig = Rig(robot, drone, params)
    robot.step(rig.timestep)

    print(f"mass {params.mass} kg, weight {params.mass * GRAVITY:.3f} N, "
          f"hover {params.hover_rotor_thrust:.3f} N/rotor "
          f"({np.sqrt(params.hover_rotor_thrust / params.thrust_constant):.1f} rad/s)")
    print(f"drag model: a_z = T/m - g - {LINEAR_DAMPING} * v_z\n")

    report: dict = {"mass_kg": params.mass, "linear_damping": LINEAR_DAMPING,
                    "measure_s": MEASURE_S}

    # ---- spin-up transient, measured continuously with no re-pinning
    # resetPhysics() also clears the motor state, so holding the drone in place
    # every tick would keep restarting the spin-up. The drone is released once
    # and thrust is recovered from the drag-corrected acceleration.
    print("[1] spin-up: free release, command 2.70 N/rotor = 10.8 N total")
    print("      t[s]      z[m]     v_z      a_z     thrust[N]   ratio")
    rig.pin()
    robot.step(rig.timestep)
    rig.drive(2.70)
    spin_rows = []
    previous_v = rig.gps.getSpeedVector()[2]
    previous_t = robot.getTime()
    start = robot.getTime()
    next_report = 0.0
    while robot.getTime() - start < 4.0:
        rig.drive(2.70)
        if robot.step(rig.timestep) == -1:
            return 1
        now, v = robot.getTime(), rig.gps.getSpeedVector()[2]
        dt = now - previous_t
        if dt <= 0:
            continue
        accel = (v - previous_v) / dt
        thrust = params.mass * (accel + GRAVITY + LINEAR_DAMPING * v)
        previous_v, previous_t = v, now
        elapsed = now - start
        if elapsed >= next_report:
            next_report += 0.25
            row = {"t": elapsed, "z": rig.gps.getValues()[2], "v_z": v,
                   "a_z": accel, "thrust_N": thrust, "ratio": thrust / 10.8}
            spin_rows.append(row)
            print(f"    {elapsed:6.2f}  {row['z']:8.2f}  {v:+7.3f}  {accel:+7.3f}  "
                  f"{thrust:8.3f}  {row['ratio']:7.3f}")
    report["spin_up"] = spin_rows

    settled_hold = 1.0
    print(f"\n[2] thrust curve at hold={settled_hold}s")
    print("   cmd/rotor  cmd total  omega      a_z      v_mean   thrust[N]   ratio")
    curve_rows = []
    for per_rotor in (0.0, 0.7, 1.35, 2.0, 2.7, 4.0, 5.4):
        row = rig.measure(per_rotor, settled_hold)
        curve_rows.append(row)
        print(f"    {per_rotor:6.3f}   {4 * per_rotor:8.3f}  {row['omega_rad_s']:6.1f}  "
              f"{row['a_z']:+8.3f}  {row['v_mean']:+7.3f}  "
              f"{row['measured_total_N']:8.3f}  {row['ratio']:7.3f}")
    report["thrust_curve"] = curve_rows

    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / "thrust_curve.json").write_text(json.dumps(report, indent=2), encoding="utf8")
    print(f"\nwrote {RESULTS / 'thrust_curve.json'}")

    useful = [r for r in curve_rows if r["commanded_total_N"] > 0]
    mean_ratio = float(np.mean([r["ratio"] for r in useful]))
    print(f"mean measured/commanded = {mean_ratio:.4f}")
    _ = scenario
    return 0 if abs(mean_ratio - 1.0) < 0.1 else 1


if __name__ == "__main__":
    raise SystemExit(main())
