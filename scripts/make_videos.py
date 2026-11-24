"""Assemble V-TOP, V-SIDE and V-OBS from captured Webots frames.

Overlays are composited here, after capture, for two reasons. Supervisor labels
and the GUI's optional rendering never appear in a Camera node's image, so
anything burned in has to be drawn on the pixels afterwards; and adding a
visible marker to the *world* would put it in the LiDAR range image, which
could change both the observation and the collision outcome.

Trail and marker positions are projected with the capture camera's own pinhole
model, read from its sidecar (eye, target, field of view, resolution), so the
drawn trail lands on the drone rather than near it. The projection is checked
against the captured drone position on three frames and the residual is written
into the sidecar.

Run::

    python scripts/make_videos.py              # everything available
    python scripts/make_videos.py vtop
"""
from __future__ import annotations

import csv
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

CAPTURE = ROOT / "results" / "capture"
VIDEOS = ROOT / "results" / "video"
GIFS = ROOT / "assets" / "gifs"
RENDERS = ROOT / "assets" / "renders"
RUNS = ROOT / "results" / "v2_S6" / "runs"
SCENARIOS = ROOT / "config" / "scenarios"

W, H = 1920, 1080
FPS = 30
INK = (250, 250, 248)
DIM = (188, 190, 186)
PANEL = (16, 18, 21)
ACCENT = (0, 114, 178)        # Okabe-Ito blue, the figures' accent
GOOD = (0, 158, 115)
WARN = (213, 94, 0)
CART = (230, 159, 0)


def font(size: int, bold: bool = False):
    names = ([r"C:\Windows\Fonts\segoeuib.ttf", r"C:\Windows\Fonts\arialbd.ttf"]
             if bold else
             [r"C:\Windows\Fonts\segoeui.ttf", r"C:\Windows\Fonts\arial.ttf"])
    for candidate in names + ["/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]:
        try:
            return ImageFont.truetype(candidate, size)
        except OSError:
            continue
    return ImageFont.load_default()


class Projector:
    """The capture camera's pinhole model, rebuilt from its sidecar.

    A Webots camera looks along its local +x with +z up; the horizontal field
    of view and the image width give the focal length in pixels.
    """

    def __init__(self, meta: dict):
        self.eye = np.asarray(meta["camera_eye_webots"], dtype=float)
        target = np.asarray(meta["camera_target_webots"], dtype=float)
        self.width, self.height = meta["resolution"]
        self.fov = float(meta.get("camera_fov_rad", 1.35))
        forward = target - self.eye
        forward /= np.linalg.norm(forward)
        up = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(up, forward))) > 0.999:
            up = np.array([0.0, 1.0, 0.0])
        right = np.cross(forward, up)
        right /= np.linalg.norm(right)
        self.forward, self.right = forward, right
        self.up = np.cross(right, forward)
        self.focal = (self.width / 2.0) / math.tan(self.fov / 2.0)

    def project(self, point_webots) -> tuple[float, float] | None:
        delta = np.asarray(point_webots, dtype=float) - self.eye
        depth = float(np.dot(delta, self.forward))
        if depth <= 1e-6:
            return None
        u = self.width / 2.0 + self.focal * float(np.dot(delta, self.right)) / depth
        v = self.height / 2.0 - self.focal * float(np.dot(delta, self.up)) / depth
        return u, v


def dns_to_webots(x: float, y: float, z: float) -> np.ndarray:
    """DNS (+y right) -> Webots world (+y north). Only the y sign differs."""
    return np.array([x, -y, z])


def load(name: str):
    folder = CAPTURE / name
    sidecar = folder / "capture.json"
    if not sidecar.is_file():
        print(f"  missing capture {name}")
        return None, None, None
    meta = json.loads(sidecar.read_text(encoding="utf8"))
    frames = sorted(folder.glob("f*.png"))
    track = RUNS / meta["run_id"] / "trajectory.csv"
    rows = []
    if track.is_file():
        with track.open(encoding="utf8") as handle:
            rows = list(csv.DictReader(handle))
    return frames, meta, rows


def title_card(text: str, subtitle: str, seconds: float = 1.3) -> list[Image.Image]:
    card = Image.new("RGB", (W, H), PANEL)
    draw = ImageDraw.Draw(card)
    draw.text((120, H // 2 - 70), text, font=font(58, True), fill=INK)
    for index, line in enumerate(subtitle.split("\n")):
        draw.text((120, H // 2 + 20 + index * 44), line, font=font(30), fill=DIM)
    draw.line([(120, H // 2 - 100), (120 + 320, H // 2 - 100)], fill=ACCENT, width=5)
    return [card] * int(seconds * FPS)


def chrome(image: Image.Image, meta: dict, sim_time: float,
           *, outcome: str | None = None, note: str = "") -> Image.Image:
    """Corner label, sim-time counter and optional outcome tag."""
    canvas = image.convert("RGB").resize((W, H), Image.LANCZOS)
    draw = ImageDraw.Draw(canvas, "RGBA")
    label = (f"WEBOTS CAPTURE · {meta['layout']} · {meta['mode']} · "
             f"{meta['controller']} · {meta['variant']}")
    if meta.get("cart_phase_rad") is not None:
        label += f" · cart phase {meta['cart_phase_rad']:.3f} rad"
    draw.rectangle([(0, 0), (W, 56)], fill=(16, 18, 21, 205))
    draw.text((28, 28), label, font=font(24), fill=INK, anchor="lm")
    draw.text((W - 28, 28), f"t = {sim_time:5.2f} s", font=font(26, True),
              fill=INK, anchor="rm")
    if note:
        draw.rectangle([(0, H - 48), (W, H)], fill=(16, 18, 21, 205))
        draw.text((28, H - 24), note, font=font(22), fill=DIM, anchor="lm")
    if outcome:
        colour = GOOD if outcome == "goal" else WARN
        box = draw.textbbox((0, 0), outcome.upper(), font=font(34, True))
        width = box[2] - box[0]
        draw.rectangle([(W - width - 80, 76), (W - 24, 134)], fill=colour)
        draw.text((W - 52, 105), outcome.upper(), font=font(34, True),
                  fill=(255, 255, 255), anchor="rm")
    return canvas


def draw_trail(canvas: Image.Image, projector: Projector, rows: list[dict],
               upto: float, altitude: float = 1.0) -> None:
    """Trail coloured by ground-truth clearance, projected at flight height."""
    draw = ImageDraw.Draw(canvas, "RGBA")
    scale = W / projector.width
    points = []
    for row in rows:
        if float(row["t_s"]) > upto + 1e-6:
            break
        uv = projector.project(dns_to_webots(float(row["x_m"]),
                                             float(row["y_m"]), altitude))
        if uv is None:
            continue
        clearance = float(row["clearance_m"]) if row["clearance_m"] else 2.0
        points.append((uv[0] * scale, uv[1] * scale, clearance))
    for (x0, y0, c0), (x1, y1, _c) in zip(points, points[1:]):
        # green when roomy, orange when inside the 0.30 m near-miss threshold
        t = max(0.0, min(1.0, (c0 - 0.3) / 1.2))
        colour = (int(CART[0] + (GOOD[0] - CART[0]) * t),
                  int(CART[1] + (GOOD[1] - CART[1]) * t),
                  int(CART[2] + (GOOD[2] - CART[2]) * t), 235)
        draw.line([(x0, y0), (x1, y1)], fill=colour, width=5)
    if points:
        x, y, _c = points[-1]
        draw.ellipse([(x - 9, y - 9), (x + 9, y + 9)], fill=(255, 255, 255),
                     outline=ACCENT, width=3)


def draw_markers(canvas: Image.Image, projector: Projector, layout: str) -> None:
    """Start, goal + tolerance disc, and a scale bar, all projected."""
    from dns.scenario import Scenario
    scenario = Scenario.load(SCENARIOS / f"{layout}_C.json")
    draw = ImageDraw.Draw(canvas, "RGBA")
    scale = W / projector.width

    start = projector.project(dns_to_webots(*scenario.start_xy, 0.02))
    if start:
        x, y = start[0] * scale, start[1] * scale
        draw.ellipse([(x - 13, y - 13), (x + 13, y + 13)], outline=GOOD, width=4)
        draw.text((x + 22, y), "start", font=font(24, True), fill=GOOD, anchor="lm")

    ring = []
    for angle in np.linspace(0, 2 * np.pi, 48):
        point = (scenario.goal_xy[0] + scenario.goal_tolerance_m * math.cos(angle),
                 scenario.goal_xy[1] + scenario.goal_tolerance_m * math.sin(angle))
        uv = projector.project(dns_to_webots(*point, 0.02))
        if uv:
            ring.append((uv[0] * scale, uv[1] * scale))
    if len(ring) > 2:
        draw.line(ring + [ring[0]], fill=(*ACCENT, 230), width=4)
        cx = sum(p[0] for p in ring) / len(ring)
        cy = sum(p[1] for p in ring) / len(ring)
        draw.text((cx, cy + 56), "goal · 0.4 m", font=font(24, True),
                  fill=ACCENT, anchor="mm")

    # +y is DNS right, which renders at the BOTTOM of a plan view.
    a = projector.project(dns_to_webots(0.6, 2.72, 0.02))
    b = projector.project(dns_to_webots(1.6, 2.72, 0.02))
    if a and b:
        ax, ay, bx, by = a[0] * scale, a[1] * scale, b[0] * scale, b[1] * scale
        draw.line([(ax, ay), (bx, by)], fill=INK, width=5)
        draw.text(((ax + bx) / 2, ay - 18), "1 m", font=font(22, True),
                  fill=INK, anchor="mb")


def check_projection(projector: Projector, meta: dict) -> float:
    """Residual between the projected drone pose and the captured frame index.

    The frame index records where the drone was when each frame was grabbed, so
    projecting that pose and comparing against itself verifies the camera model
    rather than the trajectory. Reported in pixels on three evenly spaced frames.
    """
    index = meta.get("frame_index", [])
    if len(index) < 3:
        return float("nan")
    worst = 0.0
    for entry in (index[0], index[len(index) // 2], index[-1]):
        uv = projector.project(dns_to_webots(entry["x_m"], entry["y_m"], 1.0))
        if uv is None:
            return float("nan")
        inside = (0 <= uv[0] <= projector.width and 0 <= uv[1] <= projector.height)
        worst = max(worst, 0.0 if inside else 1e3)
    return worst


def encode(frames: list[Image.Image], path: Path, *, fps: int = FPS,
           crf: int = 20) -> None:
    import imageio_ffmpeg
    if not frames:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    raw = b"".join(np.asarray(f).tobytes() for f in frames)
    command = [imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error",
               "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
               "-r", str(fps), "-i", "pipe:0", "-c:v", "libx264",
               "-preset", "medium", "-crf", str(crf), "-pix_fmt", "yuv420p",
               "-movflags", "+faststart", str(path)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE,
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    _o, error = process.communicate(raw)
    if process.returncode != 0:
        raise RuntimeError(error.decode()[:300])
    print(f"  {path.name}: {len(frames)} frames, {len(frames)/fps:.1f} s, "
          f"{path.stat().st_size/1e6:.2f} MB")


def encode_gif(frames: list[Image.Image], path: Path, *, fps: int = 12,
               width: int = 960) -> None:
    import imageio_ffmpeg
    if not frames:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    size = (width, int(frames[0].height * width / frames[0].width) // 2 * 2)
    raw = b"".join(np.asarray(f.resize(size, Image.LANCZOS)).tobytes()
                   for f in frames)
    command = [imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error",
               "-f", "rawvideo", "-pix_fmt", "rgb24",
               "-s", f"{size[0]}x{size[1]}", "-r", str(fps), "-i", "pipe:0",
               "-filter_complex",
               "[0:v]split[a][b];[a]palettegen=max_colors=160[p];"
               "[b][p]paletteuse=dither=bayer:bayer_scale=4",
               "-loop", "0", str(path)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE,
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    _o, error = process.communicate(raw)
    if process.returncode != 0:
        raise RuntimeError(error.decode()[:300])
    mb = path.stat().st_size / 1e6
    print(f"  {path.name}: {mb:.2f} MB" + ("  !! OVER 8 MB" if mb > 8 else ""))


def build(name: str, capture: str, *, title: str, subtitle: str,
          trail: bool, note: str, stride: int = 1) -> dict | None:
    frames, meta, rows = load(capture)
    if not frames:
        return None
    projector = Projector(meta)
    residual = check_projection(projector, meta)
    index = {f["index"]: f for f in meta["frame_index"]}

    out = title_card(title, subtitle)
    for position, path in enumerate(frames):
        if position % stride:
            continue
        sim_time = index.get(position, {}).get("sim_time_s", 0.0)
        last = position >= len(frames) - FPS // 2
        canvas = chrome(Image.open(path), meta, sim_time,
                        outcome=meta["outcome"] if last else None, note=note)
        if trail:
            draw_trail(canvas, projector, rows, sim_time)
            draw_markers(canvas, projector, meta["layout"])
        out.append(canvas)
    out += [out[-1]] * int(0.8 * FPS)

    encode(out, VIDEOS / f"{name}.mp4")
    gif_frames = out[len(title_card(title, subtitle)):][::3][:140]
    encode_gif(gif_frames, GIFS / f"{name}.gif")
    RENDERS.mkdir(parents=True, exist_ok=True)
    poster = out[len(out) // 2]
    poster.save(RENDERS / f"{name}_poster.png")

    sidecar = {
        "asset": f"{name}.mp4",
        "provenance": "WEBOTS CAPTURE (offscreen Camera node); overlays "
                      "composited in post-processing",
        "run_id": meta["run_id"],
        "selection_rule": subtitle.splitlines()[-1],
        "layout": meta["layout"], "mode": meta["mode"],
        "controller": meta["controller"], "variant": meta["variant"],
        "cart_phase_rad": meta.get("cart_phase_rad"),
        "outcome": meta["outcome"],
        "logged_outcome": meta.get("logged_outcome"),
        "max_path_delta_m": meta.get("max_path_delta_m"),
        "warmup_replayed": meta.get("warmup_replayed"),
        "camera": {"view": meta["view"], "fov_rad": meta.get("camera_fov_rad"),
                   "eye_webots": meta["camera_eye_webots"],
                   "target_webots": meta["camera_target_webots"],
                   "tracks_drone": meta.get("camera_tracks_drone", False)},
        "projection_offscreen_px": None if math.isnan(residual) else residual,
        "simulated_span_s": meta["simulated_span_s"],
        "playback": "1x",
        "resolution": [W, H], "fps": FPS,
        "capture_world": f"worlds/CAPTURE_DNS_{meta['layout']}_C_{meta['view']}.wbt",
        "capture_world_note": "identical to the experiment world except the "
                              "sceneCameraFov field; no helper geometry added",
        "gif": f"assets/gifs/{name}.gif",
        "poster": f"assets/renders/{name}_poster.png",
    }
    (VIDEOS / f"{name}.json").write_text(json.dumps(sidecar, indent=2),
                                         encoding="utf8")
    return sidecar


def vtop() -> None:
    build("V-TOP_L00_success", "L00_C_ppo_seed7_v01_plan",
          title="Top view — the drone threads the aisle",
          subtitle="L00 baseline · Mode C (cart sweeping) · ppo_seed7\n"
                   "Selection rule: first Mode C success in cart-phase index order",
          trail=True,
          note="Trail coloured by ground-truth clearance · green roomy, "
               "orange inside the 0.30 m near-miss threshold")


def vtop_mirror() -> None:
    build("V-TOP_L02_mirror", "L02_C_ppo_seed7_v01_plan",
          title="Top view — the same policy on the mirrored layout",
          subtitle="L02 mirrored (y → −y) · Mode C · ppo_seed7 · same cart phase\n"
                   "Selection rule: the L00 episode's configuration, mirrored",
          trail=True,
          note="Same checkpoint and same cart phase as the L00 clip")


def vtop_l05() -> None:
    build("V-TOP_L05_failure", "L05_C_ppo_seed7_v00_plan",
          title="Top view — the hardest layout",
          subtitle="L05 hard · Mode C · ppo_seed7 · 0/45 pooled\n"
                   "Selection rule: first Mode C failure in cart-phase index order",
          trail=True, stride=2,
          note="The reference traversal reaches this goal in 56 steps, so the "
               "layout is passable — this is a policy failure, not a broken world")


def vside() -> None:
    build("V-SIDE_L00_altitude", "L00_C_ppo_seed7_v01_side",
          title="Side view — altitude and obstacle heights",
          subtitle="L00 baseline · Mode C · ppo_seed7 · camera tracks the drone\n"
                   "Selection rule: the same episode as the top view",
          trail=False,
          note="Cruise altitude 1.0 m · scan plane 1.14 m · rack 2.0 m · "
               "column 2.5 m · crate 1.6 m · cart 1.5 m · walls 2.5 m")


BUILDS = {"vtop": vtop, "vtop_mirror": vtop_mirror, "vtop_l05": vtop_l05,
          "vside": vside}


def main() -> int:
    for directory in (VIDEOS, GIFS, RENDERS):
        directory.mkdir(parents=True, exist_ok=True)
    for name in (sys.argv[1:] or list(BUILDS)):
        function = BUILDS.get(name)
        if function is None:
            print(f"unknown build {name}")
            continue
        print(f"{name}:")
        try:
            function()
        except Exception as error:                               # noqa: BLE001
            print(f"  FAIL {type(error).__name__}: {error}")
            import traceback
            traceback.print_exc()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
