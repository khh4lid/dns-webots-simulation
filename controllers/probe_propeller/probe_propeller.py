"""Internal controller for the 4-rotor propeller probes.

controllerArgs: <label> <step_ms> <resend 0|1>

Spins every rotor at a speed whose nominal thrust far exceeds the vehicle
weight and records the altitude, so the Propeller node can be checked in
isolation from the DNS PROTO and autopilot.
"""
import os
import sys

from controller import Supervisor

LABEL = sys.argv[1] if len(sys.argv) > 1 else "probe_propeller"
STEP_MS = int(sys.argv[2]) if len(sys.argv) > 2 else 0          # 0 = basicTimeStep
RESEND = bool(int(sys.argv[3])) if len(sys.argv) > 3 else False
#: Bitmask of which rotors to spin. 0b1001 = m0 and m3 only (both CCW, positive
#: thrustConstant) which is a torque-free pair, so any altitude change isolates
#: the thrust of the positive-constant rotors from the negative-constant ones.
MASK = int(sys.argv[4], 0) if len(sys.argv) > 4 else 0b1111
#: Whether to command velocity 0 before spinning up. The official sample never
#: does this; it goes straight from setPosition(inf) to the running velocity.
ZERO_FIRST = bool(int(sys.argv[5])) if len(sys.argv) > 5 else True

OUT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..",
                                   "results", f"{LABEL}.txt"))

KT = 0.00026
MASS = 0.49 + 4 * 0.015
SPINS = (+1, -1, -1, +1)
SPEED = 200.0


def main():
    robot = Supervisor()
    basic = int(robot.getBasicTimeStep())
    step = STEP_MS or basic
    gps = robot.getDevice("gps")
    gps.enable(step)
    imu = robot.getDevice("imu")
    imu.enable(step)
    motors = [robot.getDevice(f"m{i}") for i in range(4)]
    for motor in motors:
        motor.setPosition(float("inf"))
        if not ZERO_FIRST:
            continue
        motor.setVelocity(0.0)

    per_rotor = KT * SPEED ** 2
    lines = [
        f"label={LABEL}  basicTimeStep={basic}  controller step={step}  resend={RESEND}",
        f"speed={SPEED} rad/s -> {per_rotor:.3f} N/rotor, {4 * per_rotor:.3f} N total "
        f"(weight {MASS * 9.81:.3f} N, TWR {4 * per_rotor / (MASS * 9.81):.2f})",
        "",
        "   t[s]    z[m]     vz[m/s]",
    ]
    def drive():
        for index, (motor, spin) in enumerate(zip(motors, SPINS)):
            motor.setVelocity(spin * SPEED if (MASK >> index) & 1 else 0.0)

    lines.insert(2, f"rotor mask={MASK:04b} (m3 m2 m1 m0 reading right to left)")
    robot.step(step)
    drive()

    start = robot.getTime()
    while robot.getTime() - start < 1.5:
        if RESEND:
            drive()
        if robot.step(step) == -1:
            break
        elapsed = robot.getTime() - start
        if abs(elapsed * 10 - round(elapsed * 10)) < (step / 1000.0) * 0.6:
            lines.append(f"  {elapsed:5.2f}  {gps.getValues()[2]:7.3f}  "
                         f"{gps.getSpeedVector()[2]:+8.3f}")

    final_z = gps.getValues()[2]
    lines.append("")
    lines.append(f"final z = {final_z:.4f} (start 1.0) -> "
                 f"{'CLIMBED (thrust present)' if final_z > 1.05 else 'FELL (no thrust)'}")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf8") as handle:
        handle.write("\n".join(lines) + "\n")


main()
