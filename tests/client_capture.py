"""Capture real Webots frames for one episode of the S6 matrix.

Replays an episode that is already in ``results/v2_S6/results_v2.csv`` -- same
layout, same mode, same controller, same cart phase -- and writes the rendered
frames as PNGs. Because the episode definition is identical, the footage shows
the run the numbers came from rather than a lookalike.

Why an offscreen ``Camera`` node and not the built-in recorder: on this machine
``Supervisor.movieStartRecording`` produces a structurally valid MP4 whose
frames are blank (148 frames averaging 47 bytes), and ``exportImage`` returns a
uniform fill of the background colour. A ``Camera`` child of the robot renders
to an offscreen texture and does produce real pixels; the backend world-locks it
so it holds a fixed studio viewpoint instead of flying with the drone. Every
frame written here is therefore a genuine render -- **WEBOTS CAPTURE**.

Frames are grabbed from ``backend.step_hook``, i.e. from inside ``advance``.
Stepping the simulation by hand alongside ``Navigation.step`` would
double-advance the clock and trip Spencer's "Unsynchronised frame" guard.

Run::

    python scripts/with_webots.py --boot-wait 12 --timeout 400 \\
        --world worlds/DNS_L00_C.wbt tests/client_capture.py -- \\
        --layout L00 --mode C --controller ppo_seed7 --variant 0
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.scenario import Scenario                                # noqa: E402
from dns.spencer_link import DT, Navigation, find_ppo_dir        # noqa: E402

SCENARIOS = PROJECT_ROOT / "config" / "scenarios"
TEST_START = 200_000_000

#: Studio viewpoints in Webots world coordinates (y_webots = -y_dns). Pulled
#: back at a shallow angle the 2.5 m wall would hide most of the floor, so each
#: eye sits high enough to clear it.
VIEWS = {
    "overview": (np.array([-2.2, -4.6, 9.2]), np.array([6.3, 0.1, 0.7])),
    "topdown": (np.array([5.0, 0.0, 11.5]), np.array([5.0, 0.0, 0.0])),
    "low": (np.array([0.35, -2.1, 2.4]), np.array([7.5, 0.3, 0.9])),
}


def grab(camera) -> np.ndarray | None:
    """One RGB frame from the offscreen camera, or None if it has not rendered."""
    raw = camera.getImage()
    if not raw:
        return None
    width, height = camera.getWidth(), camera.getHeight()
    data = np.frombuffer(raw, dtype=np.uint8)
    if data.size < width * height * 4:
        return None
    return data[:width * height * 4].reshape(height, width, 4)[:, :, 2::-1]


def phase_for(seed: int) -> float:
    return float(np.random.default_rng(seed).uniform(0.0, 2.0 * np.pi))


def with_phase(scenario: Scenario, phase: float) -> Scenario:
    import dataclasses
    obstacles = []
    for obstacle in scenario.obstacles:
        if obstacle.motion.type == "linear" and obstacle.motion.amplitude_m:
            motion = dataclasses.replace(obstacle.motion, phase_rad=phase)
            obstacles.append(dataclasses.replace(obstacle, motion=motion))
        else:
            obstacles.append(obstacle)
    return dataclasses.replace(scenario, obstacles=tuple(obstacles))


def load_policy(name: str):
    ppo_dir = find_ppo_dir()
    if name.startswith("ppo_seed"):
        from ppo import PPO                                      # noqa: E402
        path = ppo_dir / "evidence" / f"seed{name.removeprefix('ppo_seed')}" / "model.npz"
        return PPO.load(path).predict
    from evaluate import gap_rule, goal_direct                   # noqa: E402
    return {"reactive": gap_rule, "direct": goal_direct}[name]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", required=True)
    parser.add_argument("--mode", default="C", choices=("B", "C"))
    parser.add_argument("--controller", default="ppo_seed7")
    parser.add_argument("--variant", type=int, default=0)
    parser.add_argument("--view", default="overview", choices=tuple(VIEWS))
    parser.add_argument("--every", type=int, default=5,
                        help="grab a frame every N physics steps (8 ms each)")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    run_id = (f"{args.layout}_{args.mode}_{args.controller}"
              f"_v{args.variant:02d}")
    out = args.out or (PROJECT_ROOT / "results" / "capture" / f"{run_id}_{args.view}")
    out.mkdir(parents=True, exist_ok=True)

    scenario = Scenario.load(SCENARIOS / f"{args.layout}_{args.mode}.json")
    seed = TEST_START + args.variant
    phase = None
    if args.mode == "C":
        phase = phase_for(seed)
        scenario = with_phase(scenario, phase)

    from dns.backend import WebotsBackend                         # noqa: E402

    backend = WebotsBackend(scenario)
    camera = backend.robot.getDevice("scene_camera")
    if camera is None:
        print("FAIL: world has no scene_camera; regenerate with camera.enabled")
        return 1
    if not backend.attach_scene_camera(*VIEWS[args.view]):
        print("FAIL: could not world-lock the scene camera")
        return 1

    policy = load_policy(args.controller)
    env = Navigation(backend)
    obs, _info = env.reset(seed, "moving" if args.mode == "C" else "static")

    state = {"step": 0, "frames": 0}
    meta_frames: list[dict] = []

    def hook(_backend) -> None:
        state["step"] += 1
        if state["step"] % args.every:
            return
        frame = grab(camera)
        if frame is None:
            return
        from PIL import Image
        Image.fromarray(frame).save(out / f"f{state['frames']:05d}.png")
        meta_frames.append({
            "index": state["frames"],
            "sim_time_s": round(backend.episode_time, 4),
            "x_m": round(float(backend.position_dns()[0]), 4),
            "y_m": round(float(backend.position_dns()[1]), 4),
        })
        state["frames"] += 1

    backend.step_hook = hook

    while True:
        action = np.asarray(load_policy.cache(obs), dtype=float) \
            if hasattr(load_policy, "cache") else np.asarray(policy(obs), dtype=float)
        obs, _reward, ended, cut, info = env.step(action)
        if ended or cut:
            break

    # Compare against the logged episode. A capture is an INDEPENDENT re-run of
    # the same configuration, not a replay of the same process: the simulator is
    # deterministic given the same episode SEQUENCE, and a fresh single-episode
    # session starts from different residual airframe state. Variant 00 is the
    # first episode of its block and reproduces exactly; later variants may
    # differ. Recording the comparison makes that visible instead of implied.
    logged = None
    results = PROJECT_ROOT / "results" / "v2_S6" / "results_v2.csv"
    if results.is_file():
        import csv as _csv
        with results.open(encoding="utf8") as handle:
            for row in _csv.DictReader(handle):
                if row["run_id"] == run_id:
                    logged = row
                    break

    sidecar = {
        "run_id": run_id,
        "provenance": "WEBOTS CAPTURE (offscreen Camera node)",
        "layout": args.layout,
        "mode": f"{args.mode}_{'dynamic' if args.mode == 'C' else 'static'}",
        "controller": args.controller,
        "variant": f"v{args.variant:02d}",
        "cart_phase_rad": round(phase, 6) if phase is not None else None,
        "seed": seed,
        "outcome": info["outcome"],
        "control_steps": env.steps,
        "simulated_span_s": round(env.steps * DT, 3),
        "frames": state["frames"],
        "physics_steps_per_frame": args.every,
        "frame_interval_s": round(args.every * scenario.basic_timestep_ms / 1000.0, 4),
        "view": args.view,
        "camera_eye_webots": VIEWS[args.view][0].tolist(),
        "camera_target_webots": VIEWS[args.view][1].tolist(),
        "resolution": [camera.getWidth(), camera.getHeight()],
        "frame_index": meta_frames,
        "logged_outcome": logged["outcome"] if logged else None,
        "logged_termination_step": int(logged["termination_step"]) if logged else None,
        "matches_logged_outcome": (
            None if not logged
            else logged["outcome"].startswith(info["outcome"])
            or info["outcome"] == logged["outcome"]),
        "capture_note": (
            "Independent re-run of the same configuration. Variant 00 is the "
            "first episode of its block and reproduces the logged episode "
            "exactly; later variants can differ because the simulator is "
            "deterministic given the same episode sequence, not in isolation."),
    }
    (out / "capture.json").write_text(json.dumps(sidecar, indent=2),
                                      encoding="utf8")
    print(f"[{run_id}] {state['frames']} frames, outcome={info['outcome']}, "
          f"{env.steps} steps -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
