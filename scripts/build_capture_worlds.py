"""Build capture-only copies of the experiment worlds.

A capture world differs from its experiment world in **exactly one field**:
``sceneCameraFov`` on the drone PROTO, so a presentation view can use a narrow
field of view from a high eye and approximate an orthographic plan.

Nothing else changes. In particular no helper geometry is added: the Webots
LiDAR is a rendering sensor, so a visible marker would appear in the range
image and could change both the observation and the collision outcome. A
``Camera`` is a sensor, not geometry, and carries no mass or collider.

That this is true is not asserted, it is **measured**: every capture records
its per-control-step poses and compares them against the logged trajectory,
and the resulting `max_path_delta_m` is written into the video sidecar.

Run::

    python scripts/build_capture_worlds.py
"""
from __future__ import annotations

import re
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORLDS = ROOT / "worlds"
# Capture worlds live beside the experiment worlds, not in a
# sub-folder: the PROTO is referenced as "../protos/..." and a
# deeper directory silently breaks that path, which shows up only as
# "robot not in the list of <extern> controllers".
OUT = WORLDS

#: view -> field of view [rad]. Must match tests/client_capture.py::VIEW_FOV.
#: plan: 2*atan(5.6/30). At 16:9 an 11.2 m wide frame is 6.3 m tall, which
#: holds the 6 m arena with ~5% margin top and bottom.
VIEWS = {"plan": 0.3704, "side": 0.95}

#: (layout, mode) pairs that presentation footage is shot from.
TARGETS = [("L00", "C"), ("L02", "C"), ("L05", "C")]


def build(source: Path, view: str, fov: float) -> Path:
    text = source.read_text(encoding="utf8")
    if "sceneCamera TRUE" not in text:
        raise SystemExit(f"{source.name} has no scene camera; regenerate it")
    if "sceneCameraFov" in text:
        text = re.sub(r"sceneCameraFov\s+[-0-9.e]+",
                      f"sceneCameraFov {fov:.6g}", text)
    else:
        text = text.replace("  sceneCamera TRUE",
                            f"  sceneCamera TRUE\n  sceneCameraFov {fov:.6g}")
    header = (
        "# CAPTURE COPY -- presentation footage only.\n"
        f"# Identical to {source.name} except sceneCameraFov = {fov:.6g} rad.\n"
        "# No helper geometry is added: the lidar is a rendering sensor, so a\n"
        "# visible marker would enter the range image and could change the\n"
        "# observation and the collision outcome. Never used for experiments.\n")
    text = text.replace("#VRML_SIM R2025a utf8\n",
                        "#VRML_SIM R2025a utf8\n\n" + header)
    target = OUT / f"CAPTURE_{source.stem}_{view}.wbt"
    target.write_text(text, encoding="utf8")
    return target


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    written = []
    for layout, mode in TARGETS:
        source = WORLDS / f"DNS_{layout}_{mode}.wbt"
        if not source.is_file():
            print(f"skip {source.name}: absent")
            continue
        for view, fov in VIEWS.items():
            written.append(build(source, view, fov))
            print(f"wrote worlds/{written[-1].name}  fov={fov:.4f} rad")

    # Prove the only difference is the one field.
    for target in written:
        source = WORLDS / (target.stem.rsplit("_", 1)[0]
                           .removeprefix("CAPTURE_") + ".wbt")
        def meaningful(text: str) -> list[str]:
            return [l for l in text.splitlines()
                    if l.strip() and not l.lstrip().startswith("#")
                    and "sceneCameraFov" not in l]
        a = meaningful(source.read_text(encoding="utf8"))
        b = meaningful(target.read_text(encoding="utf8"))
        if a != b:
            raise SystemExit(f"{target.name} differs from {source.name} by more "
                             "than the camera field -- refusing to continue")
    print(f"\n{len(written)} capture worlds; each differs from its experiment "
          "world by the sceneCameraFov field alone (verified line by line)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
