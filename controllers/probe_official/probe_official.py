"""Replica of the official axial_rotor sample controller, with file logging.

An optional controllerArg names the output file, so the bisect harness can run
many world variants against this one controller.
"""
import os
import sys

from controller import Supervisor

_LABEL = sys.argv[1] if len(sys.argv) > 1 else None
#: Second arg flips the commanded motor velocity, so a variant can pair a
#: negative thrustConstant with a negative angular velocity -- the combination
#: every counter-rotating rotor on a quadcopter relies on.
_SIGN = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
_NAME = f"bisect_{_LABEL}.txt" if _LABEL else "probe_official.txt"
OUT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "results", _NAME))

MAIN_HELIX_VELOCITY = 100.0
TARGET_ALTITUDE = 2.0


def main():
    robot = Supervisor()
    time_step = 64
    gps = robot.getDevice("gps")
    gps.enable(time_step)
    motor = robot.getDevice("motor")
    motor.setPosition(float("inf"))
    motor.setVelocity(_SIGN * (MAIN_HELIX_VELOCITY + 1))

    lines = [f"basicTimeStep={robot.getBasicTimeStep()}  controller step={time_step}  "
             f"velocity sign={_SIGN:+.0f}",
             "", "   t[s]     z[m]"]

    robot.step(time_step)
    rest_z = gps.getValues()[2]
    peak_z = rest_z
    start = robot.getTime()
    while robot.step(time_step) != -1 and robot.getTime() - start < 6.0:
        altitude = gps.getValues()[2]
        peak_z = max(peak_z, altitude)
        ratio = 1.0 - altitude / TARGET_ALTITUDE
        motor.setVelocity(_SIGN * (MAIN_HELIX_VELOCITY + ratio))
        elapsed = robot.getTime() - start
        if abs(elapsed * 2 - round(elapsed * 2)) < 0.033:
            lines.append(f"  {elapsed:5.2f}  {altitude:8.4f}")

    final = gps.getValues()[2]
    lines.append("")
    lines.append(f"final z = {final:.4f}  peak {peak_z:.4f}  initial {rest_z:.4f} -> "
                 f"{'CLIMBED (thrust present)' if peak_z > rest_z + 0.05 else 'FELL (no thrust)'}")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf8") as handle:
        handle.write("\n".join(lines) + "\n")


main()
