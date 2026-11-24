"""Probe: can Webots actually record an MP4 from this setup?

The movie API is a mandatory deliverable, and it has two failure modes that do
NOT raise an exception:

* ``--no-rendering`` / ``--minimize`` (what our headless harness normally uses)
  means there are no rendered frames to capture, so the encoder produces an
  empty or missing file while every call "succeeds";
* the encoder can fail asynchronously, which is reported by ``movieFailed()``
  long after ``movieStartRecording()`` returned.

So this probe asserts on the *file*, not on the API returning cleanly.

Run WITH THE GUI (rendering must be on)::

    python scripts/with_webots.py --gui --timeout 300 tests/client_movie_probe.py
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.launcher import find_webots_home                            # noqa: E402
from dns.scenario import DEFAULT_SCENARIO_PATH, Scenario             # noqa: E402
from dns.spencer_link import DT                                      # noqa: E402

RESULTS = PROJECT_ROOT / "results"


def ffprobe(path: Path) -> dict:
    """Decode the file with Webots' own ffmpeg. Missing ffprobe is not fatal."""
    home = find_webots_home()
    exe = home / "msys64" / "mingw64" / "bin" / "ffprobe.exe"
    if not exe.is_file():
        return {"available": False}
    try:
        out = subprocess.run(
            [str(exe), "-v", "error", "-print_format", "json",
             "-show_format", "-show_streams", str(path)],
            capture_output=True, text=True, timeout=60)
        if out.returncode != 0:
            return {"available": True, "ok": False, "stderr": out.stderr[:400]}
        info = json.loads(out.stdout)
        video = next((s for s in info.get("streams", [])
                      if s.get("codec_type") == "video"), None)
        return {
            "available": True, "ok": video is not None,
            "codec": video and video.get("codec_name"),
            "width": video and video.get("width"),
            "height": video and video.get("height"),
            "nb_frames": video and video.get("nb_frames"),
            "duration_s": float(info.get("format", {}).get("duration", 0.0) or 0.0),
            "size_bytes": int(info.get("format", {}).get("size", 0) or 0),
        }
    except Exception as exc:                                  # noqa: BLE001
        return {"available": True, "ok": False, "error": repr(exc)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--quality", type=int, default=95)
    parser.add_argument("--acceleration", type=int, default=1)
    parser.add_argument("--out", type=Path,
                        default=RESULTS / "movie_probe" / "probe.mp4")
    args = parser.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    if args.out.exists():
        args.out.unlink()

    scenario = Scenario.load(DEFAULT_SCENARIO_PATH)
    from dns.backend import WebotsBackend                     # noqa: E402

    backend = WebotsBackend(scenario)
    robot = backend.robot
    print(f"target : {args.out}")
    print(f"size   : {args.width}x{args.height}  quality {args.quality}  "
          f"acceleration {args.acceleration}")

    backend.reset(seed=1, scene="static")
    robot.movieStartRecording(str(args.out), args.width, args.height,
                              0, args.quality, args.acceleration, False)
    print("movieStartRecording called")

    steps = int(args.seconds / DT)
    for i in range(steps):
        # a visible, simple motion so successive frames genuinely differ
        command = np.array([0.8, 0.0]) if i < steps // 2 else np.array([0.0, 0.8])
        backend.advance(command, DT)
    print(f"advanced {steps} control steps ({steps * DT:.1f} s sim time)")

    robot.movieStopRecording()
    print("movieStopRecording called; waiting for the encoder")

    deadline = time.time() + 120.0
    ready = failed = False
    while time.time() < deadline:
        ready = bool(robot.movieIsReady())
        failed = bool(robot.movieFailed())
        if failed or (ready and args.out.is_file() and args.out.stat().st_size > 0):
            break
        robot.step(backend.timestep)
    print(f"movieIsReady={ready}  movieFailed={failed}")

    exists = args.out.is_file()
    size = args.out.stat().st_size if exists else 0
    probe = ffprobe(args.out) if exists and size else {"available": None}

    report = {
        "path": str(args.out), "exists": exists, "size_bytes": size,
        "movie_is_ready": ready, "movie_failed": failed,
        "requested": {"width": args.width, "height": args.height,
                      "quality": args.quality,
                      "acceleration": args.acceleration,
                      "sim_seconds": steps * DT},
        "ffprobe": probe,
    }
    verdict = bool(exists and size > 10_000 and not failed
                   and probe.get("ok") is not False)
    report["verdict"] = "USABLE" if verdict else "NOT USABLE"

    (RESULTS / "movie_probe" / "probe_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf8")
    print("\n" + json.dumps(report, indent=2))
    return 0 if verdict else 1


if __name__ == "__main__":
    raise SystemExit(main())
