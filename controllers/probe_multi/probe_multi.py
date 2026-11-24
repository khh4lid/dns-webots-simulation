"""Drive every rotor present at a fixed speed and report the vertical response.

controllerArgs: <label> <speed_rad_s>

Measures the net vertical acceleration over the first flight phase and
compares it with free fall, so "thrust present" is decided by physics rather
than by whether an unstabilised airframe happened to stay upright.
"""
import os
import sys

from controller import Supervisor

LABEL = sys.argv[1] if len(sys.argv) > 1 else "probe_multi"
SPEED = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0
OUT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..",
                                   "results", f"qbisect_{LABEL}.txt"))

SPINS = (+1, -1, -1, +1)


def main():
    robot = Supervisor()
    step = int(robot.getBasicTimeStep())
    gps = robot.getDevice("gps")
    gps.enable(step)

    motors = []
    for name in ("motor", "m0", "m1", "m2", "m3"):
        device = robot.getDevice(name)
        if device is not None:
            motors.append(device)
    for motor in motors:
        motor.setPosition(float("inf"))

    for index, motor in enumerate(motors):
        motor.setVelocity(SPINS[index % 4] * SPEED)

    lines = [f"label={LABEL}  rotors={len(motors)}  speed={SPEED}  step={step} ms",
             "", "   t[s]     z[m]      vz[m/s]"]

    robot.step(step)
    start_time = robot.getTime()
    start_vz = gps.getSpeedVector()[2]
    samples = []
    while robot.getTime() - start_time < 3.0:
        for index, motor in enumerate(motors):
            motor.setVelocity(SPINS[index % 4] * SPEED)
        if robot.step(step) == -1:
            break
        elapsed = robot.getTime() - start_time
        samples.append((elapsed, gps.getValues()[2], gps.getSpeedVector()[2]))
        if abs(elapsed * 4 - round(elapsed * 4)) < (step / 1000.0) * 0.6:
            lines.append(f"  {elapsed:5.2f}  {samples[-1][1]:8.3f}  {samples[-1][2]:+9.3f}")

    # Net vertical acceleration over the first 0.25 s. Only meaningful when the
    # airframe starts clear of the ground -- a resting body reads ~0 because of
    # the normal force, which would look like thrust.
    early = [s for s in samples if s[0] <= 0.25]
    accel = (early[-1][2] - start_vz) / early[-1][0] if len(early) >= 2 else float("nan")
    start_z = samples[0][1] if samples else float("nan")
    peak_z = max((s[1] for s in samples), default=float("nan"))
    lines.append("")
    lines.append(f"start z = {start_z:.4f}   peak z = {peak_z:.4f}   "
                 f"gain = {peak_z - start_z:+.4f} m")
    lines.append(f"net vertical acceleration over first {early[-1][0]:.3f} s = "
                 f"{accel:+.3f} m/s^2   (free fall = -9.81)")
    # Climbing well above the spawn height is unambiguous; so is an early
    # acceleration clearly above free fall.
    thrust = (peak_z - start_z) > 0.25 or accel > -9.0
    lines.append(f"verdict: {'THRUST PRESENT' if thrust else 'NO THRUST (free fall)'}")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf8") as handle:
        handle.write("\n".join(lines) + "\n")


main()
