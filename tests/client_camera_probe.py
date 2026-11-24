"""Does an offscreen Webots Camera render, when the GUI viewport does not?

``exportImage`` from the main 3D view returned a perfectly uniform fill of the
background colour (1 unique colour, std 0.0) at three different drone poses, so
the window renderer draws no geometry in this automated session. A ``Camera``
node is a different path -- it renders to an offscreen texture -- so it is worth
testing independently before declaring video capture impossible.

Verdict is on pixels: a working camera gives many unique colours and images that
*change* as the drone flies.

Run::

    python scripts/with_webots.py --boot-wait 12 --timeout 300 \\
        tests/client_camera_probe.py
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.scenario import DEFAULT_SCENARIO_PATH, Scenario             # noqa: E402
from dns.spencer_link import DT                                      # noqa: E402

RESULTS = PROJECT_ROOT / "results" / "camera_probe"


def frame_array(camera) -> np.ndarray | None:
    """Camera image as an (h, w, 3) uint8 array, or None if nothing rendered."""
    raw = camera.getImage()
    if not raw:
        return None
    height, width = camera.getHeight(), camera.getWidth()
    data = np.frombuffer(raw, dtype=np.uint8)
    if data.size < width * height * 4:
        return None
    # Webots delivers BGRA.
    return data[:width * height * 4].reshape(height, width, 4)[:, :, 2::-1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=RESULTS)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    scenario = Scenario.load(DEFAULT_SCENARIO_PATH)
    from dns.backend import WebotsBackend                            # noqa: E402

    backend = WebotsBackend(scenario)
    camera = backend.robot.getDevice("scene_camera")
    if camera is None:
        print("no 'scene_camera' device: regenerate the world with "
              "sceneCamera TRUE in the DnsQuadcopter PROTO")
        return 1
    camera.enable(backend.timestep)
    print(f"camera {camera.getWidth()}x{camera.getHeight()} enabled")

    backend.reset(seed=1, scene="static")
    for _ in range(3):
        backend.robot.step(backend.timestep)

    shots = []
    arrays = []
    for index, command in enumerate((np.zeros(2),
                                     np.array([1.0, 0.0]),
                                     np.array([0.0, 1.0]))):
        for _ in range(10):
            backend.advance(command, DT)
        array = frame_array(camera)
        if array is None:
            shots.append({"index": index, "rendered": False})
            print(f"  shot{index}: camera returned no image")
            continue
        arrays.append(array)
        flat = array.reshape(-1, 3)
        unique = int(len(np.unique(flat, axis=0)))
        info = {
            "index": index, "rendered": True,
            "command": command.tolist(),
            "drone_xy": backend.position_dns().tolist(),
            "unique_colours": unique,
            "std_rgb": [round(float(v), 2) for v in flat.std(axis=0)],
            "mean_rgb": [round(float(v), 2) for v in flat.mean(axis=0)],
        }
        shots.append(info)
        print(f"  shot{index}: unique={unique:,} std={info['std_rgb']} "
              f"mean={info['mean_rgb']}")
        try:
            from PIL import Image
            Image.fromarray(array).save(args.out / f"cam{index}.png")
        except Exception as exc:                                  # noqa: BLE001
            print(f"    (png save failed: {exc})")

    rendered = [s for s in shots if s.get("rendered")]
    varied = len(rendered) >= 2 and any(
        not np.array_equal(arrays[0], other) for other in arrays[1:])
    rich = all(s["unique_colours"] > 50 for s in rendered) if rendered else False

    report = {"shots": shots, "frames_differ": bool(varied),
              "content_rich": bool(rich)}
    report["conclusion"] = (
        "CAMERA WORKS: offscreen rendering produces varied scene content"
        if (varied and rich) else
        "CAMERA BLANK: offscreen rendering is also unavailable here")
    (args.out / "camera_probe.json").write_text(json.dumps(report, indent=2),
                                                encoding="utf8")
    print("\n" + report["conclusion"])
    return 0 if (varied and rich) else 1


if __name__ == "__main__":
    raise SystemExit(main())
