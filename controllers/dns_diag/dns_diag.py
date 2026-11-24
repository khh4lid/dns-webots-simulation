"""Diagnostic internal controller.

Webots routes its own parser warnings to the GUI console, which is invisible in
headless batch runs. This controller is launched *by* Webots, so if it writes
its report the world and the PROTO both instantiated correctly. Used only to
bisect world-loading problems; the real runtime path is the ``<extern>``
controller in ``dns/backend.py``.
"""
import os
import traceback

from controller import Supervisor

OUT = os.path.join(os.path.dirname(__file__), "..", "..", "results", "diag.txt")


def main():
    lines = []
    try:
        robot = Supervisor()
        lines.append(f"name={robot.getName()!r}")
        lines.append(f"basicTimeStep={robot.getBasicTimeStep()}")
        lines.append(f"supervisor={robot.getSupervisor()}")
        count = robot.getNumberOfDevices()
        lines.append(f"devices={count}")
        for i in range(count):
            device = robot.getDeviceByIndex(i)
            lines.append(f"  device[{i}] {device.getName()!r} nodeType={device.getNodeType()}")
        for name in ("DNS_DRONE", "DNS_FLOOR", "DNS_OBSTACLE_OBS_A", "DNS_GOAL_MARKER"):
            node = robot.getFromDef(name)
            lines.append(f"  DEF {name} -> {None if node is None else node.getTypeName()}")
        drone = robot.getFromDef("DNS_DRONE")
        if drone is not None:
            for param in ("mass", "armOffset", "thrustConstant", "torqueConstant",
                          "collisionRadius", "lidarResolution"):
                field = drone.getField(param)
                lines.append(f"  param {param} -> {None if field is None else 'present'}")
        robot.step(int(robot.getBasicTimeStep()))
        lines.append(f"time_after_one_step={robot.getTime()}")
        lines.append("DIAG_OK")
    except Exception:
        lines.append("DIAG_EXCEPTION")
        lines.append(traceback.format_exc())

    os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
    with open(os.path.abspath(OUT), "w", encoding="utf8") as handle:
        handle.write("\n".join(lines) + "\n")


main()
