"""Is Webots rendering at all, and does real-time mode fix movie capture?

The first movie probe produced a structurally valid MP4 -- 148 frames, 25 fps,
5.92 s -- whose frames averaged 47 bytes. That is a blank picture, so one of two
things is true and they need different fixes:

A. **Nothing is being rendered.** Then ``Supervisor.exportImage`` also yields a
   uniform image, and a movie can never work from the 3D view.
B. **Rendering works but the simulation outruns it.** The controller steps as
   fast as the machine allows, so the recorder samples a handful of wall-clock
   instants and the interesting motion falls between frames. Then
   ``--mode=realtime`` fixes it.

This probe distinguishes them: it exports PNGs from three clearly different
drone poses and measures whether the *files differ*. Identical-size, tiny PNGs
mean (A); visibly different PNGs mean (B).

Run::

    python scripts/with_webots.py --gui --boot-wait 12 --timeout 300 \\
        --webots-arg=--mode=realtime tests/client_render_probe.py
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.scenario import DEFAULT_SCENARIO_PATH, Scenario             # noqa: E402
from dns.spencer_link import DT                                      # noqa: E402

RESULTS = PROJECT_ROOT / "results" / "render_probe"


def digest(path: Path) -> dict:
    if not path.is_file():
        return {"exists": False}
    raw = path.read_bytes()
    return {"exists": True, "size_bytes": len(raw),
            "sha256": hashlib.sha256(raw).hexdigest()[:16]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=RESULTS)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    scenario = Scenario.load(DEFAULT_SCENARIO_PATH)
    from dns.backend import WebotsBackend                            # noqa: E402

    backend = WebotsBackend(scenario)
    robot = backend.robot
    backend.reset(seed=1, scene="static")

    shots = []
    for index, command in enumerate((np.zeros(2),
                                     np.array([1.0, 0.0]),
                                     np.array([0.0, 1.0]))):
        for _ in range(10):                      # 2 s of motion between shots
            backend.advance(command, DT)
        path = args.out / f"shot{index}.png"
        if path.exists():
            path.unlink()
        robot.exportImage(str(path), 100)
        # exportImage is asynchronous: the write lands on a later step.
        for _ in range(10):
            robot.step(backend.timestep)
        info = digest(path)
        info.update(index=index, command=command.tolist(),
                    drone_xy=backend.position_dns().tolist())
        shots.append(info)
        print(f"  shot{index}: cmd {command.tolist()} at "
              f"{np.round(backend.position_dns(), 3).tolist()} -> {info}")

    present = [s for s in shots if s.get("exists")]
    hashes = {s["sha256"] for s in present}
    sizes = [s["size_bytes"] for s in present]
    report = {
        "shots": shots,
        "all_written": len(present) == len(shots),
        "distinct_images": len(hashes),
        "size_range": [min(sizes), max(sizes)] if sizes else None,
    }
    if not present:
        report["conclusion"] = ("A: exportImage wrote nothing -- the 3D view is "
                                "not available to this process")
    elif len(hashes) <= 1:
        report["conclusion"] = ("A: every export is byte-identical -- the view "
                                "is not rendering scene content")
    elif max(sizes) < 20_000:
        report["conclusion"] = ("A: images differ but are tiny -- the view is "
                                "nearly blank")
    else:
        report["conclusion"] = ("B: rendering works and the images differ, so "
                                "the movie needs real-time pacing, not a "
                                "rendering fix")
    (args.out / "render_probe.json").write_text(json.dumps(report, indent=2),
                                                encoding="utf8")
    print("\n" + json.dumps(report, indent=2))
    return 0 if report["conclusion"].startswith("B") else 1


if __name__ == "__main__":
    raise SystemExit(main())
