"""Record the DNS demonstration video from genuinely rendered Webots frames.

Why not ``Supervisor.movieStartRecording``
------------------------------------------
It was tried first and it fails silently here. It produced a structurally valid
MP4 -- 148 frames, 25 fps -- whose every frame was a uniform fill of the
background colour. ``Supervisor.exportImage`` confirmed the cause: the main 3D
viewport renders **no geometry** in this automated session (1 unique colour,
std 0.0, at three different drone poses, and unchanged after the viewpoint was
corrected). The GUI render path is unavailable when Webots is launched
non-interactively.

A ``Camera`` node renders to an offscreen texture instead, and that path *does*
work here (520-564 unique colours, frames changing with the drone). So every
frame in this video is a real Webots render of the real simulation, captured
through the camera sensor rather than through the window.

The camera is world-locked by the backend: it is a child of the drone, so the
backend writes its mount pose with the inverse of the airframe pose each tick,
holding a fixed studio viewpoint while the drone flies.

Encoding uses the ffmpeg shipped by ``imageio-ffmpeg``; the ffmpeg bundled with
Webots does not run in this environment (exit 53, no output).

Timing is honest: frames are captured every ``--every`` physics steps and the
video is encoded at the matching rate, so playback is 1:1 with simulation time.
No frame is ever duplicated or stretched.

Run::

    python scripts/with_webots.py --boot-wait 12 --timeout 2400 \\
        --world worlds/DNS_Research_World.wbt tests/client_record_demo.py \\
        -- --config config/scenario_research_demo.json
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.scenario import Scenario, to_webots                        # noqa: E402
from dns.spencer_link import DT, Navigation, find_ppo_dir           # noqa: E402

RESULTS = PROJECT_ROOT / "results"


class Encoder:
    """Pipe raw RGB frames straight into ffmpeg."""

    def __init__(self, path: Path, width: int, height: int, fps: float):
        import imageio_ffmpeg
        self.path = path
        self.width, self.height, self.fps = width, height, fps
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists():
            path.unlink()
        self.process = subprocess.Popen(
            [imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error",
             "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", f"{width}x{height}", "-r", f"{fps:g}", "-i", "-",
             "-c:v", "libx264", "-preset", "medium", "-crf", "20",
             "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE)
        self.frames = 0

    def add(self, rgb: np.ndarray) -> None:
        self.process.stdin.write(rgb.astype(np.uint8).tobytes())
        self.frames += 1

    def close(self) -> str:
        self.process.stdin.close()
        err = self.process.stderr.read().decode("utf8", "replace")
        self.process.wait(timeout=180)
        return err


def _font(size: int):
    """A readable TrueType face, falling back to PIL's bitmap font."""
    from PIL import ImageFont
    for candidate in (r"C:\Windows\Fonts\segoeui.ttf",
                      r"C:\Windows\Fonts\arial.ttf",
                      "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(candidate, size)
        except OSError:
            continue
    return ImageFont.load_default()


class Captioner:
    """Burn a title bar and a per-segment caption into each frame.

    Done here rather than with an ffmpeg ``drawtext`` filter so the result does
    not depend on a fontconfig setup, and so the text is part of the recorded
    frame rather than a post-process over footage.
    """

    def __init__(self, width: int, height: int):
        self.width, self.height = width, height
        self.title_font = _font(max(18, width // 46))
        self.caption_font = _font(max(15, width // 62))
        self.small_font = _font(max(13, width // 78))
        self.title = "Drone Navigation System  ·  Webots R2025a"
        self.caption = ""
        self.detail = ""

    def apply(self, rgb: np.ndarray) -> np.ndarray:
        from PIL import Image, ImageDraw
        image = Image.fromarray(rgb)
        draw = ImageDraw.Draw(image, "RGBA")
        bar = max(34, self.height // 18)
        draw.rectangle([0, 0, self.width, bar], fill=(11, 11, 11, 170))
        draw.text((14, bar // 2), self.title, font=self.title_font,
                  fill=(255, 255, 255, 235), anchor="lm")
        if self.caption:
            foot = max(52, self.height // 11)
            draw.rectangle([0, self.height - foot, self.width, self.height],
                           fill=(11, 11, 11, 170))
            draw.text((14, self.height - foot + foot * 0.34), self.caption,
                      font=self.caption_font, fill=(255, 255, 255, 240),
                      anchor="lm")
            if self.detail:
                draw.text((14, self.height - foot + foot * 0.72), self.detail,
                          font=self.small_font, fill=(195, 194, 183, 235),
                          anchor="lm")
        return np.asarray(image)


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
    parser.add_argument("--static-config", type=Path,
                        default=PROJECT_ROOT / "config" / "scenario_warehouse_static.json",
                        help="same world, transfer cart parked; used for "
                             "the static-navigation segment")
    parser.add_argument("--out", type=Path,
                        default=RESULTS / "demo" / "DNS_Research_Demonstration.mp4")
    parser.add_argument("--every", type=int, default=5,
                        help="capture one frame every N physics steps "
                             "(5 x 8 ms = 40 ms = 25 fps, real time)")
    parser.add_argument("--policy", default="seed19")
    args = parser.parse_args()

    scenario = Scenario.load(args.config)
    fps = 1000.0 / (scenario.basic_timestep_ms * args.every)

    from dns.backend import WebotsBackend                           # noqa: E402

    backend = WebotsBackend(scenario)
    camera = backend.robot.getDevice("scene_camera")
    if camera is None:
        print("FAIL: world has no scene_camera; regenerate with camera.enabled")
        return 1

    centre_w = to_webots(((scenario.low_xy[0] + scenario.high_xy[0]) / 2.0,
                          (scenario.low_xy[1] + scenario.high_xy[1]) / 2.0))
    look_at = np.array([centre_w[0], centre_w[1], 1.0])

    # Studio viewpoints, Webots world coordinates. y_webots = -y_dns.
    # Each eye sits above the near wall line or inside the room: pulled back
    # at a shallow angle, the 2.5 m wall hides 2.5 / tan(elevation) metres of
    # the 6 m floor, which swallows the start pad entirely.
    OVERVIEW = (np.array([-2.2, -4.6, 9.2]), np.array([6.3, 0.1, 0.7]))
    TOPDOWN = (np.array([5.0, 0.0, 11.0]), np.array([5.0, 0.0, 0.0]))
    LOW_ANGLE = (np.array([0.35, -2.1, 2.4]), np.array([7.5, 0.3, 0.9]))

    backend.attach_scene_camera(*OVERVIEW)
    encoder = Encoder(args.out, camera.getWidth(), camera.getHeight(), fps)
    print(f"recording {camera.getWidth()}x{camera.getHeight()} @ {fps:g} fps "
          f"-> {args.out}")

    timeline: list[dict] = []
    tick = {"n": 0, "recording": False}
    captioner = Captioner(camera.getWidth(), camera.getHeight())

    def hook(_backend) -> None:
        """Grab a frame every ``--every`` physics steps, from inside advance."""
        if not tick["recording"]:
            return
        tick["n"] += 1
        if tick["n"] % args.every:
            return
        frame = grab(camera)
        if frame is not None:
            encoder.add(captioner.apply(frame))

    backend.step_hook = hook

    def capture_hold(seconds: float, label: str, caption: str = "",
                     detail: str = "") -> None:
        """Hold a hover and record, for establishing shots."""
        captioner.caption, captioner.detail = caption, detail
        steps = int(seconds * 1000.0 / scenario.basic_timestep_ms)
        start = encoder.frames
        tick["recording"] = True
        for _ in range(steps):
            backend.autopilot.update(np.zeros(2), scenario.cruise_altitude_m)
            backend._move_obstacles(backend.episode_time)
            backend._step_once()
        tick["recording"] = False
        timeline.append({"segment": label, "frames": encoder.frames - start,
                         "sim_seconds": seconds})
        print(f"  [{label}] {encoder.frames - start} frames")

    def capture_orbit(seconds: float, label: str, caption: str = "",
                      detail: str = "") -> None:
        """Slow arc around the arena so the whole scene is readable."""
        captioner.caption, captioner.detail = caption, detail
        steps = int(seconds * 1000.0 / scenario.basic_timestep_ms)
        start = encoder.frames
        # A crane move kept high and close rather than a wide low orbit: at
        # radius 5.5 and height 9.5 the elevation stays near 60 deg, so the
        # 2.5 m wall never hides more than about 1.4 m of floor.
        radius, height = 5.5, 9.5
        backend._camera_target_w = np.array([centre_w[0], centre_w[1], 0.7])
        tick["recording"] = True
        for i in range(steps):
            phase = -0.72 * math.pi + 0.62 * math.pi * (i / max(1, steps - 1))
            backend._camera_eye_w = np.array([
                centre_w[0] + radius * math.cos(phase),
                centre_w[1] + radius * math.sin(phase), height])
            backend.autopilot.update(np.zeros(2), scenario.cruise_altitude_m)
            backend._move_obstacles(backend.episode_time)
            backend._step_once()
        tick["recording"] = False
        timeline.append({"segment": label, "frames": encoder.frames - start,
                         "sim_seconds": seconds})
        print(f"  [{label}] {encoder.frames - start} frames")

    def capture_episode(policy, label: str, eye, caption: str = "",
                        detail: str = "") -> dict:
        """Record one real closed-loop episode, whatever its outcome.

        The episode is driven entirely through Spencer's ``Navigation.step``;
        frames come out of the step hook, so the simulation clock advances
        exactly once per control period.
        """
        captioner.caption, captioner.detail = caption, detail
        backend._camera_eye_w = np.asarray(eye[0], dtype=float)
        backend._camera_target_w = np.asarray(eye[1], dtype=float)
        env = Navigation(backend)
        obs, _ = env.reset(seed=200_000_000, scene="moving")
        start = encoder.frames
        tick["recording"] = True
        while True:
            action = np.asarray(policy(obs), dtype=float)
            obs, reward, ended, cut, info = env.step(action)
            if ended or cut:
                break
        tick["recording"] = False
        record = {"segment": label, "frames": encoder.frames - start,
                  "outcome": info["outcome"], "steps": env.steps,
                  "sim_seconds": env.steps * DT,
                  "final_xy": env.frame.xy.tolist(),
                  "remaining_m": info["remaining_m"]}
        timeline.append(record)
        print(f"  [{label}] {record['frames']} frames, "
              f"outcome={info['outcome']}, {env.steps} steps")
        return record

    # ---- load the policy ------------------------------------------------
    find_ppo_dir()
    from ppo import PPO                                              # noqa: E402
    model_path = find_ppo_dir() / "evidence" / args.policy / "model.npz"
    if not model_path.is_file():
        print(f"FAIL: no checkpoint at {model_path}")
        return 1
    policy = PPO.load(model_path).predict
    print(f"policy : {model_path}")

    # ---- the programme --------------------------------------------------
    # MODE B and MODE C share one world. Swapping the scenario swaps only the
    # crossing obstacle's motion (parked vs sweeping), so lighting, geometry
    # and camera are identical and the two modes are directly comparable.
    dynamic_scenario = scenario
    static_scenario = Scenario.load(args.static_config)
    static_scenario.validate()

    sensing_line = (f"LiDAR 360 deg, {scenario.lidar_resolution} beams, 5 m cap, "
                    f"12 sectors  ·  PPO {args.policy} at 5 Hz, max 1.2 m/s/axis")

    backend.scenario = static_scenario
    backend.reset(seed=200_000_000, scene="static")
    capture_orbit(12.0, "1_environment_overview",
                  "Warehouse inspection aisle: 10.0 x 6.0 m, walls 2.5 m",
                  "Green pad = start (0.85, 0.0) m   Blue pad = goal "
                  "(9.15, 0.0) m, tolerance 0.4 m   Dark grey = storage racks "
                  "  Light grey = columns   Orange = transfer cart   "
                  "Floor grid = 1 m")
    capture_hold(6.0, "2_drone_start_and_goal",
                 "Quadcopter on the start pad, holding 1.0 m altitude",
                 "0.55 kg airframe, thrust/weight 4.0, cascaded autopilot at "
                 "125 Hz  ·  " + sensing_line)
    episode_a = capture_episode(
        policy, "3_static_navigation_overview", OVERVIEW,
        "MODE B - static navigation: racks only, transfer cart parked",
        "Every command comes from the trained PPO policy reading real LiDAR")
    episode_b = capture_episode(
        policy, "4_static_navigation_topdown", TOPDOWN,
        "MODE B - same run, top-down view",
        f"Outcome: {episode_a['outcome'].upper()} in {episode_a['steps']} "
        f"control steps ({episode_a['sim_seconds']:.1f} s)")

    backend.scenario = dynamic_scenario
    backend.reset(seed=200_000_000, scene="moving")
    capture_hold(4.0, "5_crossing_obstacle_intro",
                 "MODE C - the transfer cart now crosses the aisle",
                 "Constant 0.45 m/s, sweeping y = -1.9 to +1.9 m at x = 5.1 m, "
                 "period 16.9 s")
    episode_c = capture_episode(
        policy, "6_dynamic_navigation_overview", OVERVIEW,
        "MODE C - dynamic navigation, same policy and same start",
        "Only one factor changed: the cart moves")
    episode_d = capture_episode(
        policy, "7_dynamic_navigation_lowangle", LOW_ANGLE,
        "MODE C - same run, low angle",
        f"Outcome: {episode_c['outcome'].upper()} after "
        f"{episode_c['steps']} control steps")
    backend._camera_eye_w, backend._camera_target_w = OVERVIEW
    capture_hold(5.0, "8_final_state",
                 f"Recorded result: static {episode_a['outcome'].upper()}, "
                 f"dynamic {episode_c['outcome'].upper()}",
                 "Unsuccessful runs are shown, not hidden. Single fixed layout "
                 "- a demonstration, not a success-rate estimate.")

    stderr = encoder.close()
    duration = encoder.frames / fps
    report = {
        "path": str(args.out),
        "width": camera.getWidth(), "height": camera.getHeight(),
        "fps": fps, "frames": encoder.frames,
        "duration_s": duration,
        "playback": "1:1 with simulation time; no frames duplicated or stretched",
        "frame_source": "Webots Camera node (offscreen render), NOT the GUI "
                        "viewport, which renders nothing in this session",
        "policy": str(model_path),
        "scenarios": {"static_segment": static_scenario.name,
                      "dynamic_segment": dynamic_scenario.name},
        "timeline": timeline,
        "episode_outcomes": {
            "static": [episode_a["outcome"], episode_b["outcome"]],
            "dynamic": [episode_c["outcome"], episode_d["outcome"]],
        },
        "ffmpeg_stderr": stderr[:2000],
    }
    out_json = args.out.parent / "recording_report.json"
    out_json.write_text(json.dumps(report, indent=2), encoding="utf8")

    print(f"\nencoded {encoder.frames} frames = {duration:.1f} s at {fps:g} fps")
    print(f"outcomes: {report['episode_outcomes']}")
    if stderr.strip():
        print(f"ffmpeg said: {stderr[:500]}")
    print(f"wrote {args.out}")
    print(f"wrote {out_json}")
    return 0 if args.out.is_file() and args.out.stat().st_size > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
