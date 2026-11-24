"""Can the lidar mount be re-oriented at run time? Find out before relying on it.

``DnsQuadcopter`` is a template PROTO (``# template language: javascript``).
Webots regenerates such a node when a field used by the template engine is
written, and regeneration would invalidate the ``Lidar`` device handle the
backend holds. ``lidarMountRotation`` is linked with a plain ``IS`` and is
deliberately not referenced from any ``%< >%`` block, so it should update in
place -- but "should" is not evidence.

Two candidate write paths are probed:

1. the exposed PROTO field  ``drone.getField("lidarMountRotation")``
2. the internal DEF node    ``drone.getFromProtoDef("DNS_LIDAR_MOUNT")``

For each: write a known tilt, step, and check that the device still answers and
that the range image changed in the direction a tilted scan plane predicts.
The drone is parked level on the ground-effect-free hover point throughout, so
any change in the scan comes from the mount, not from the airframe.

Run via::

    python scripts/with_webots.py --timeout 300 tests/client_gimbal_probe.py
"""
from __future__ import annotations

import dataclasses
import json
import math
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.scenario import DEFAULT_SCENARIO_PATH, Scenario  # noqa: E402

RESULTS = PROJECT_ROOT / "results"
#: Pitch applied to the mount for the probe [rad]. Large enough that a 0.85 m
#: wall 0.125 m below the beam is cleared decisively.
PROBE_PITCH = 0.30


def summarise(image) -> dict:
    a = np.asarray(image, dtype=float)
    finite = np.isfinite(a)
    return {"beams": int(a.size), "finite": int(finite.sum()),
            "min_m": float(a[finite].min()) if finite.any() else None,
            "mean_m": float(a[finite].mean()) if finite.any() else None}


def main() -> int:
    scenario = dataclasses.replace(Scenario.load(DEFAULT_SCENARIO_PATH),
                                   dynamics_mode="physical", sensing_mode="lidar")
    scenario.validate()

    from dns.backend import WebotsBackend  # noqa: E402

    backend = WebotsBackend(scenario)
    drone, lidar = backend.drone, backend.lidar
    backend.reset(seed=3, scene="static")

    report: dict = {"probe_pitch_rad": PROBE_PITCH,
                    "probe_pitch_deg": math.degrees(PROBE_PITCH)}

    def hold_and_read(label: str) -> dict:
        """Hover in place for one command period, then read the scan."""
        for _ in range(backend.steps_per_command):
            backend.autopilot.update(np.zeros(2), scenario.cruise_altitude_m)
            backend._step_once()
        image = lidar.getRangeImage()
        if image is None:
            return {"label": label, "device_alive": False}
        out = summarise(image)
        out.update(label=label, device_alive=True)
        roll, pitch, _ = backend.autopilot.state.rpy
        out["airframe_roll_deg"] = math.degrees(roll)
        out["airframe_pitch_deg"] = math.degrees(pitch)
        return out

    baseline = hold_and_read("baseline_identity_mount")
    print(f"[baseline] {baseline}")
    report["baseline"] = baseline

    # ---------------------------------------------------------------- path 1
    print("\n[path 1] drone.getField('lidarMountRotation')")
    field = drone.getField("lidarMountRotation")
    path1: dict = {"field_found": field is not None}
    if field is not None:
        try:
            field.setSFRotation([0.0, 1.0, 0.0, PROBE_PITCH])
            path1["write_raised"] = None
            after = hold_and_read("after_field_write")
            path1.update(after)
            readback = list(field.getSFRotation())
            path1["readback"] = readback
            path1["readback_matches"] = bool(
                abs(readback[3] - PROBE_PITCH) < 1e-6)
            # restore
            field.setSFRotation([0.0, 1.0, 0.0, 0.0])
            path1["restored"] = hold_and_read("after_field_restore")
        except Exception as exc:                        # noqa: BLE001
            path1["write_raised"] = f"{type(exc).__name__}: {exc}"
    print(json.dumps(path1, indent=2))
    report["path1_proto_field"] = path1

    # ---------------------------------------------------------------- path 2
    print("\n[path 2] drone.getFromProtoDef('DNS_LIDAR_MOUNT')")
    path2: dict = {}
    try:
        mount = drone.getFromProtoDef("DNS_LIDAR_MOUNT")
        path2["node_found"] = mount is not None
        if mount is not None:
            rot = mount.getField("rotation")
            path2["rotation_field_found"] = rot is not None
            if rot is not None:
                rot.setSFRotation([0.0, 1.0, 0.0, PROBE_PITCH])
                after = hold_and_read("after_protodef_write")
                path2.update(after)
                path2["readback"] = list(rot.getSFRotation())
                rot.setSFRotation([0.0, 1.0, 0.0, 0.0])
                path2["restored"] = hold_and_read("after_protodef_restore")
    except Exception as exc:                            # noqa: BLE001
        path2["raised"] = f"{type(exc).__name__}: {exc}"
    print(json.dumps(path2, indent=2))
    report["path2_proto_def"] = path2

    # ---------------------------------------------------------------- verdict
    def tilted(entry: dict) -> bool:
        """A tilted scan plane over 1.2 m walls must lose beams."""
        return (entry.get("device_alive")
                and baseline.get("finite") is not None
                and entry.get("finite") is not None
                and entry["finite"] < baseline["finite"])

    works1 = bool(path1.get("device_alive") and path1.get("readback_matches")
                  and tilted(path1))
    works2 = bool(path2.get("device_alive") and tilted(path2))
    report["verdict"] = {
        "proto_field_write_works": works1,
        "proto_def_write_works": works2,
        "recommended": ("proto_field" if works1 else
                        "proto_def" if works2 else "neither"),
    }
    print("\n--- verdict")
    print(json.dumps(report["verdict"], indent=2))

    RESULTS.mkdir(parents=True, exist_ok=True)
    out = RESULTS / "gimbal_probe.json"
    out.write_text(json.dumps(report, indent=2), encoding="utf8")
    print(f"\nwrote {out}")
    return 0 if (works1 or works2) else 1


if __name__ == "__main__":
    raise SystemExit(main())
