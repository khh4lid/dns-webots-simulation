"""Assemble GIFs, the demo MP4 and its poster from captured Webots frames.

Overlays are drawn here, in post-processing, rather than in the simulator:
Supervisor labels and the GUI's optional rendering do not appear in a Camera
node's image, so anything burned into the frame has to be composited onto the
captured pixels afterwards.

Inputs are the PNG sequences written by ``tests/client_capture.py``, each with
a ``capture.json`` sidecar recording the run id, the cart phase, the outcome and
whether that outcome matched the logged episode. Every output is therefore
traceable to a row of ``results/v2_S6/results_v2.csv``.

Run::

    python scripts/make_clips.py                 # everything available
    python scripts/make_clips.py hero mirror
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

CAPTURE = ROOT / "results" / "capture"
GIFS = ROOT / "assets" / "gifs"
RENDERS = ROOT / "assets" / "renders"
VIDEOS = ROOT / "results" / "video"
RUNS = ROOT / "results" / "v2_S6" / "runs"

INK = (248, 248, 246)
DIM = (196, 198, 194)
BAR = (18, 20, 23)


def font(size: int):
    for candidate in (r"C:\Windows\Fonts\segoeui.ttf",
                      r"C:\Windows\Fonts\arial.ttf",
                      "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(candidate, size)
        except OSError:
            continue
    return ImageFont.load_default()


def load_capture(name: str) -> tuple[list[Path], dict] | None:
    folder = CAPTURE / name
    sidecar = folder / "capture.json"
    if not sidecar.is_file():
        print(f"  missing capture: {name}")
        return None
    frames = sorted(folder.glob("f*.png"))
    return frames, json.loads(sidecar.read_text(encoding="utf8"))


def overlay(image: Image.Image, meta: dict, frame_meta: dict,
            *, title: str | None = None) -> Image.Image:
    """Burn a compact caption bar onto a captured frame."""
    width, height = image.size
    bar = max(44, height // 11)
    canvas = Image.new("RGB", (width, height + bar), BAR)
    canvas.paste(image, (0, 0))
    draw = ImageDraw.Draw(canvas)
    big, small = font(int(bar * 0.40)), font(int(bar * 0.30))

    left = title or (f"{meta['layout']} · {meta['mode']} · {meta['controller']} "
                     f"· {meta['variant']}")
    draw.text((14, height + bar * 0.30), left, font=big, fill=INK, anchor="lm")
    right = (f"t = {frame_meta.get('sim_time_s', 0):5.2f} s     "
             f"outcome: {meta['outcome']}")
    draw.text((width - 14, height + bar * 0.30), right, font=big, fill=INK,
              anchor="rm")
    draw.text((14, height + bar * 0.72), "WEBOTS CAPTURE (offscreen Camera node)",
              font=small, fill=DIM, anchor="lm")
    return canvas


def encode_gif(frames: list[Image.Image], path: Path, fps: int = 12,
               width: int = 900) -> None:
    import imageio_ffmpeg
    if not frames:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    scaled = []
    for frame in frames:
        ratio = width / frame.width
        size = (width - width % 2, int(frame.height * ratio) // 2 * 2)
        scaled.append(np.asarray(frame.resize(size, Image.LANCZOS)))
    height = scaled[0].shape[0]
    raw = b"".join(f.tobytes() for f in scaled)
    command = [
        imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{width - width % 2}x{height}", "-r", str(fps), "-i", "pipe:0",
        "-filter_complex",
        "[0:v]split[a][b];[a]palettegen=max_colors=160[p];"
        "[b][p]paletteuse=dither=bayer:bayer_scale=4",
        "-loop", "0", str(path),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE,
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    _out, error = process.communicate(raw)
    if process.returncode != 0:
        raise RuntimeError(error.decode()[:300])
    size_mb = path.stat().st_size / 1e6
    flag = "  !! OVER 8 MB" if size_mb > 8.0 else ""
    print(f"  {path.name}: {len(frames)} frames, {size_mb:.2f} MB{flag}")


def encode_mp4(frames: list[Image.Image], path: Path, fps: int = 30) -> None:
    import imageio_ffmpeg
    if not frames:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    width, height = frames[0].size
    width -= width % 2
    height -= height % 2
    raw = b"".join(np.asarray(f.crop((0, 0, width, height))).tobytes()
                   for f in frames)
    command = [
        imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
        "-r", str(fps), "-i", "pipe:0",
        "-c:v", "libx264", "-preset", "medium", "-crf", "23",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE,
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    _out, error = process.communicate(raw)
    if process.returncode != 0:
        raise RuntimeError(error.decode()[:300])
    print(f"  {path.name}: {len(frames)} frames, "
          f"{path.stat().st_size / 1e6:.2f} MB, {len(frames) / fps:.1f} s")


def frames_of(name: str, *, title: str | None = None,
              stride: int = 1) -> tuple[list[Image.Image], dict]:
    loaded = load_capture(name)
    if loaded is None:
        return [], {}
    paths, meta = loaded
    index = {f["index"]: f for f in meta.get("frame_index", [])}
    images = []
    for position, path in enumerate(paths):
        if position % stride:
            continue
        image = Image.open(path).convert("RGB")
        images.append(overlay(image, meta, index.get(position, {}), title=title))
    return images, meta


# ---------------------------------------------------------------------------
def clip_hero() -> None:
    """PPO reaching the goal on L00 with the cart sweeping."""
    images, meta = frames_of("L00_C_ppo_seed7_v01_overview", stride=2)
    if not images:
        return
    encode_gif(images, GIFS / "hero_l00_success.gif", fps=12, width=900)
    sidecar = {
        "asset": "assets/gifs/hero_l00_success.gif",
        "provenance": "WEBOTS CAPTURE (offscreen Camera node)",
        "run_id": meta["run_id"], "outcome": meta["outcome"],
        "logged_outcome": meta.get("logged_outcome"),
        "matches_logged_outcome": meta.get("matches_logged_outcome"),
        "logged_termination_step": meta.get("logged_termination_step"),
        "control_steps": meta["control_steps"],
        "simulated_span_s": meta["simulated_span_s"],
        "cart_phase_rad": meta["cart_phase_rad"],
        "note": meta.get("capture_note"),
        "source_row": "results/v2_S6/results_v2.csv",
    }
    (GIFS / "hero_l00_success.json").write_text(json.dumps(sidecar, indent=2),
                                                encoding="utf8")
    # Poster: the mid-flight frame, used as the clickable still for the MP4.
    RENDERS.mkdir(parents=True, exist_ok=True)
    images[len(images) // 2].save(RENDERS / "demo_poster.png")
    print("  demo_poster.png")


def clip_mirror() -> None:
    """L00 and L02 side by side, same checkpoint, same cart phase."""
    left, meta_a = frames_of("L00_C_ppo_seed7_v00_overview",
                             title="L00 — baseline")
    right, meta_b = frames_of("L02_C_ppo_seed7_v00_overview",
                              title="L02 — mirrored (y → −y)")
    if not left or not right:
        return
    count = max(len(left), len(right))
    pairs = []
    for index in range(count):
        a = left[min(index, len(left) - 1)]
        b = right[min(index, len(right) - 1)]
        canvas = Image.new("RGB", (a.width + b.width + 8, max(a.height, b.height)),
                           BAR)
        canvas.paste(a, (0, 0))
        canvas.paste(b, (a.width + 8, 0))
        pairs.append(canvas)
    encode_gif(pairs[::2], GIFS / "mirror_l00_vs_l02.gif", fps=12, width=1000)
    (GIFS / "mirror_l00_vs_l02.json").write_text(json.dumps({
        "asset": "assets/gifs/mirror_l00_vs_l02.gif",
        "provenance": "WEBOTS CAPTURE (offscreen Camera node)",
        "left": {"run_id": meta_a["run_id"], "outcome": meta_a["outcome"],
                 "matches_logged_outcome": meta_a.get("matches_logged_outcome")},
        "right": {"run_id": meta_b["run_id"], "outcome": meta_b["outcome"],
                  "matches_logged_outcome": meta_b.get("matches_logged_outcome")},
        "cart_phase_rad": meta_a["cart_phase_rad"],
        "note": "Same checkpoint, same cart phase, mirrored geometry. Both "
                "episodes end in a cart collision, at different times and "
                "places, because the route is reflected.",
    }, indent=2), encoding="utf8")


def clip_lidar() -> None:
    """Camera frames beside a synchronised render of the 12 sectors."""
    import dns_style as S
    import matplotlib.pyplot as plt
    S.apply()

    loaded = load_capture("L00_C_ppo_seed7_v00_topdown")
    if loaded is None:
        return
    paths, meta = loaded
    archive = RUNS / meta["run_id"] / "lidar.npz"
    if not archive.is_file():
        print("  no lidar.npz for this run")
        return
    data = np.load(archive, allow_pickle=False)
    times = data["t_s"]
    index = {f["index"]: f for f in meta["frame_index"]}

    composed = []
    for position, path in enumerate(paths[::2]):
        real = position * 2
        sim_time = index.get(real, {}).get("sim_time_s", 0.0)
        step = int(np.argmin(np.abs(times - sim_time)))
        ranges = data["sector_ranges_m"][step]
        valid = data["sector_valid"][step]

        figure, axis = plt.subplots(figsize=(4.4, 4.4), subplot_kw={"polar": True})
        angles = np.arange(12) * 2 * np.pi / 12
        colours = [S.CART if (valid[i] and ranges[i] < 2.0) else
                   S.CONTROLLER_COLOUR["ppo_pooled"] for i in range(12)]
        axis.bar(angles, np.where(valid, ranges, 0.0), width=2 * np.pi / 12 * 0.92,
                 color=colours, edgecolor="white", linewidth=0.8, zorder=4)
        axis.set_theta_zero_location("N")
        axis.set_theta_direction(-1)
        axis.set_ylim(0, 5.0)
        axis.set_yticks([1, 2, 3, 4, 5])
        axis.set_yticklabels(["1", "2", "3", "4", "5 m"], fontsize=6)
        axis.set_xticks(angles)
        axis.set_xticklabels([f"{i}" + (" front" if i == 0 else " right" if i == 3
                                        else " back" if i == 6 else " left" if i == 9
                                        else "") for i in range(12)], fontsize=6.5)
        axis.set_title(f"12 sectors · t = {sim_time:5.2f} s", fontsize=9,
                       fontweight="bold")
        axis.text(0.5, -0.12, "LOGGED-DATA RENDER", transform=axis.transAxes,
                  fontsize=6, color=S.INK_MUTED, ha="center")
        figure.tight_layout()
        figure.canvas.draw()
        plot = Image.fromarray(
            np.asarray(figure.canvas.buffer_rgba())[:, :, :3].copy())
        plt.close(figure)

        shot = overlay(Image.open(path).convert("RGB"), meta,
                       index.get(real, {}), title="L00 · top-down capture")
        plot = plot.resize((int(plot.width * shot.height / plot.height),
                            shot.height), Image.LANCZOS)
        canvas = Image.new("RGB", (shot.width + plot.width + 8, shot.height),
                           (255, 255, 255))
        canvas.paste(shot, (0, 0))
        canvas.paste(plot, (shot.width + 8, 0))
        composed.append(canvas)

    encode_gif(composed, GIFS / "lidar_sectors.gif", fps=10, width=1000)
    (GIFS / "lidar_sectors.json").write_text(json.dumps({
        "asset": "assets/gifs/lidar_sectors.gif",
        "provenance": "left WEBOTS CAPTURE, right LOGGED-DATA RENDER",
        "run_id": meta["run_id"],
        "note": "The 12 packed sectors the policy actually receives, drawn from "
                "results/v2_S6/runs/<run_id>/lidar.npz and synchronised to the "
                "captured frames by simulated time. Sector 0 is front, 3 right, "
                "6 back, 9 left; bars turn orange inside 2 m.",
    }, indent=2), encoding="utf8")


def clip_demo() -> None:
    """The main demonstration video: L00 success, then the mirrored layout."""
    segments = [
        ("L00_C_ppo_seed7_v01_overview", "L00 baseline — PPO reaches the goal"),
        ("L00_C_ppo_seed7_v00_topdown", "L00 top-down — the cart crossing"),
        ("L02_C_ppo_seed7_v00_overview", "L02 mirrored — same policy, "
                                         "reflected layout"),
    ]
    frames: list[Image.Image] = []
    manifest = []
    for name, title in segments:
        images, meta = frames_of(name, title=title)
        if not images:
            continue
        target = images[0].size
        frames.extend(i.resize(target, Image.LANCZOS) if i.size != target else i
                      for i in images)
        manifest.append({"segment": title, "run_id": meta["run_id"],
                         "frames": len(images), "outcome": meta["outcome"],
                         "matches_logged_outcome": meta.get("matches_logged_outcome")})
    if not frames:
        return
    size = frames[0].size
    frames = [f.resize(size, Image.LANCZOS) if f.size != size else f for f in frames]
    encode_mp4(frames, VIDEOS / "V1_main_demo.mp4", fps=30)
    (VIDEOS / "V1_main_demo.json").write_text(json.dumps({
        "asset": "V1_main_demo.mp4",
        "provenance": "WEBOTS CAPTURE (offscreen Camera node)",
        "distribution": "GitHub Release asset for v2.0 (not committed to git)",
        "fps": 30, "segments": manifest,
        "poster": "assets/renders/demo_poster.png",
    }, indent=2), encoding="utf8")


CLIPS = {"hero": clip_hero, "mirror": clip_mirror, "lidar": clip_lidar,
         "demo": clip_demo}


def main() -> int:
    for directory in (GIFS, RENDERS, VIDEOS):
        directory.mkdir(parents=True, exist_ok=True)
    for name in (sys.argv[1:] or list(CLIPS)):
        function = CLIPS.get(name)
        if function is None:
            print(f"unknown clip {name}")
            continue
        print(f"{name}:")
        try:
            function()
        except Exception as error:                               # noqa: BLE001
            print(f"  FAIL {type(error).__name__}: {error}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
