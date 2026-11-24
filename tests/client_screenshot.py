"""Capture still frames of the environment, rendered by Webots.

Uses the drone's offscreen ``Camera`` rather than the GUI viewport, which
renders nothing when Webots is launched non-interactively here (see
docs/SENSOR_ATTITUDE.md and results/render_probe/). The backend world-locks
the camera, so each shot is taken from a fixed studio viewpoint while the drone
holds its start pose.

Run::

    python scripts/with_webots.py --boot-wait 12 --timeout 600 \\
        --world worlds/DNS_Warehouse.wbt tests/client_screenshot.py \\
        -- --config config/scenario_warehouse.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.scenario import Scenario, to_webots                         # noqa: E402

RESULTS = PROJECT_ROOT / "results"


def grab(camera) -> np.ndarray | None:
    raw = camera.getImage()
    if not raw:
        return None
    width, height = camera.getWidth(), camera.getHeight()
    data = np.frombuffer(raw, dtype=np.uint8)
    if data.size < width * height * 4:
        return None
    return data[:width * height * 4].reshape(height, width, 4)[:, :, 2::-1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=PROJECT_ROOT / "config" / "scenario_warehouse.json")
    parser.add_argument("--out", type=Path, default=RESULTS / "screenshots")
    parser.add_argument("--settle-steps", type=int, default=90,
                        help="physics steps to let the renderer and the "
                             "airframe settle before each capture")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    scenario = Scenario.load(args.config)
    from dns.backend import WebotsBackend                            # noqa: E402

    backend = WebotsBackend(scenario)
    camera = backend.robot.getDevice("scene_camera")
    if camera is None:
        print("FAIL: this world has no scene_camera; set camera.enabled")
        return 1

    centre = to_webots(((scenario.low_xy[0] + scenario.high_xy[0]) / 2.0,
                        (scenario.low_xy[1] + scenario.high_xy[1]) / 2.0))
    look_at = np.array([centre[0], centre[1], 0.9])

    # Eye points sit above the near wall line (y >= -3.6 at height > 5 m) or
    # inside the room. A camera pulled far back at a shallow angle is hidden
    # behind the 2.5 m wall: from (5, -11.5, 8.5) the wall occludes
    # 2.5 / tan(33 deg) = 3.9 m of the 6 m depth, which swallows the start pad.
    views = {
        "overview_3d": (np.array([-2.2, -4.6, 9.2]), np.array([6.3, 0.1, 0.7])),
        "topdown": (np.array([5.0, 0.0, 11.0]), np.array([5.0, 0.0, 0.0])),
        "aisle_low": (np.array([0.35, -2.1, 2.4]), np.array([7.5, 0.3, 0.9])),
        "corner": (np.array([11.5, -4.0, 6.0]), np.array([3.5, 0.3, 0.7])),
    }

    backend.attach_scene_camera(*views["overview_3d"])
    backend.reset(seed=1, scene="moving")

    written = []
    for name, (eye, target) in views.items():
        backend._camera_eye_w = eye
        backend._camera_target_w = target
        for _ in range(args.settle_steps):
            backend.autopilot.update(np.zeros(2), scenario.cruise_altitude_m)
            backend._move_obstacles(backend.episode_time)
            backend._step_once()
        frame = grab(camera)
        if frame is None:
            print(f"  {name}: camera returned nothing")
            continue
        unique = int(len(np.unique(frame.reshape(-1, 3), axis=0)))
        path = args.out / f"{name}.png"
        try:
            from PIL import Image
            Image.fromarray(frame).save(path)
        except Exception as exc:                                  # noqa: BLE001
            print(f"  {name}: save failed {exc}")
            continue
        written.append({"view": name, "file": str(path),
                        "eye_webots": eye.tolist(),
                        "target_webots": target.tolist(),
                        "unique_colours": unique})
        print(f"  {name}: {path.name}  unique colours {unique:,}")

    (args.out / "screenshots.json").write_text(
        json.dumps({"scenario": scenario.name, "look_at": look_at.tolist(),
                    "views": written}, indent=2), encoding="utf8")
    print(f"\nwrote {len(written)} screenshots to {args.out}")
    return 0 if written else 1


if __name__ == "__main__":
    raise SystemExit(main())
