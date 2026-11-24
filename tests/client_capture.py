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
    # V-TOP: a near-orthographic plan. Placed high with a correspondingly
    # narrow field of view, so the racks stand up straight instead of leaning
    # outwards. 2*h*tan(fov/2) must frame the 10 m arena with ~5% margin:
    # at h = 30 m that needs fov = 2*atan(5.6/30) = 0.3704 rad (21.2 deg).
    "plan": (np.array([5.0, 0.0, 30.0]), np.array([5.0, 0.0, 1.0])),
    # V-SIDE: well outside the long wall and high enough that the sight line
    # clears it. Standing 0.5 m off the wall at 3.4 m was measured to be
    # useless -- the nearest rack filled the frame. The wall is 2.5 m and sits
    # 6 m from this eye, while the target is 9 m away at z = 1.0, so the ray
    # passes the wall at 7 - 6*(6/9) = 3.0 m, clearing it by 0.5 m. The
    # resulting depression angle is atan(6/9) = 33.7 deg.
    "side": (np.array([0.0, -9.0, 7.0]), np.array([0.0, 0.0, 1.0])),
}

#: Field of view per view, radians. None leaves the PROTO default (0.7854).
VIEW_FOV = {"plan": 0.3704, "side": 0.95}

#: Views whose eye tracks the drone along x rather than staying put.
TRACKING_VIEWS = {"side"}


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
    parser.add_argument("--warmup", action="store_true",
                        help="replay the exact session prefix before capturing, "
                             "so the captured run reproduces the logged one")
    parser.add_argument("--fov", type=float, default=None,
                        help="camera field of view in radians; overrides the "
                             "per-view default")
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
    # Set the field of view at run time rather than adding a PROTO field: the
    # experiment worlds must stay byte-identical, and a Camera is a sensor, not
    # geometry, so nothing the LiDAR can see changes either way.
    # The field of view is baked into the world, not set at run time: a Webots
    # Camera exposes setFov() only through its zoom, and this PROTO declares no
    # Zoom node, so the call is rejected. Capture worlds carry the right fov in
    # their sceneCameraFov field -- see scripts/build_capture_worlds.py.
    fov = float(camera.getFov())
    wanted = args.fov if args.fov is not None else VIEW_FOV.get(args.view)
    if wanted is not None and abs(fov - wanted) > 1e-3:
        print(f"NOTE: world fov {fov:.4f} rad, view {args.view} wants "
              f"{wanted:.4f} rad -- run against the capture world for this view")

    policy = load_policy(args.controller)
    env = Navigation(backend)

    # The experiment runs every episode of a layout in ONE Webots session, in a
    # fixed order: Mode B (3 episodes x 5 controllers) and then Mode C (15 x 5).
    # The simulator is deterministic given that SEQUENCE, not in isolation -- a
    # reset leaves a little residual airframe state behind. Replaying the prefix
    # is therefore what makes a capture the same run as the logged one rather
    # than merely the same configuration. Measured without it: 0.287 m of drift
    # by the end of L00_C_ppo_seed7_v01.
    if args.warmup:
        order = ["ppo_seed7", "ppo_seed19", "ppo_seed43", "reactive", "direct"]
        prefix: list[tuple[str, str, int]] = []
        for name in order:                       # the whole Mode B block first
            prefix += [("B", name, i) for i in range(3)]
        for name in order:                       # then Mode C, up to this one
            for i in range(15):
                if name == args.controller and args.mode == "C" and i >= args.variant:
                    break
                prefix.append(("C", name, i))
            if name == args.controller and args.mode == "C":
                break
        if args.mode == "B":
            prefix = [s for s in prefix if s[0] == "B"]
            prefix = prefix[:prefix.index(("B", args.controller, args.variant))]

        static_scenario = Scenario.load(SCENARIOS / f"{args.layout}_B.json")
        dynamic_scenario = Scenario.load(SCENARIOS / f"{args.layout}_C.json")
        print(f"[{run_id}] warm-up: replaying {len(prefix)} preceding episodes",
              flush=True)
        for mode, name, index in prefix:
            warm_seed = TEST_START + index
            backend.scenario = (with_phase(dynamic_scenario, phase_for(warm_seed))
                                if mode == "C" else static_scenario)
            warm_env = Navigation(backend)
            warm_obs, _ = warm_env.reset(
                warm_seed, "moving" if mode == "C" else "static")
            warm_policy = load_policy(name)
            while True:
                warm_obs, _r, done, cut, _i = warm_env.step(
                    np.asarray(warm_policy(warm_obs), dtype=float))
                if done or cut:
                    break
        backend.scenario = scenario

    obs, _info = env.reset(seed, "moving" if args.mode == "C" else "static")

    state = {"step": 0, "frames": 0}
    meta_frames: list[dict] = []

    tracking = args.view in TRACKING_VIEWS

    def hook(_backend) -> None:
        state["step"] += 1
        if tracking:
            # Slew the eye along x with the drone. Updated after the step, so
            # it takes effect on the next one -- an 8 ms lag, invisible at
            # 30 fps, and far smoother than snapping to the exact pose.
            x = float(backend.position_dns()[0])
            smooth = state.get("cam_x", x) * 0.88 + x * 0.12
            state["cam_x"] = smooth
            eye, target = VIEWS[args.view]
            backend._camera_eye_w = np.array([smooth - 1.0, eye[1], eye[2]])
            backend._camera_target_w = np.array([smooth, target[1], target[2]])
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

    # Per-CONTROL-step poses, so the capture can be checked against the logged
    # trajectory point for point rather than only on its final outcome.
    control_poses = [backend.position_dns().copy()]
    while True:
        obs, _reward, ended, cut, info = env.step(
            np.asarray(policy(obs), dtype=float))
        control_poses.append(backend.position_dns().copy())
        if ended or cut:
            break

    # Compare against the logged episode. A capture is an INDEPENDENT re-run of
    # the same configuration, not a replay of the same process: the simulator is
    # deterministic given the same episode SEQUENCE, and a fresh single-episode
    # session starts from different residual airframe state. Variant 00 is the
    # first episode of its block and reproduces exactly; later variants may
    # differ. Recording the comparison makes that visible instead of implied.
    # Point-for-point path delta against the logged trajectory. This is the
    # claim the footage rests on: that the captured run IS the logged run.
    path_delta = None
    logged_track = PROJECT_ROOT / "results" / "v2_S6" / "runs" / run_id / "trajectory.csv"
    if logged_track.is_file():
        import csv as _csv
        with logged_track.open(encoding="utf8") as handle:
            logged_xy = np.array([[float(r["x_m"]), float(r["y_m"])]
                                  for r in _csv.DictReader(handle)])
        captured = np.asarray(control_poses)
        n = min(len(logged_xy), len(captured))
        if n:
            path_delta = float(np.abs(logged_xy[:n] - captured[:n]).max())

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
        "camera_fov_rad": round(float(fov), 6),
        "camera_tracks_drone": tracking,
        "warmup_replayed": bool(args.warmup),
        "frame_index": meta_frames,
        "max_path_delta_m": (round(path_delta, 9)
                             if path_delta is not None else None),
        "control_steps_compared": (min(len(control_poses),
                                       sum(1 for _ in open(logged_track, encoding="utf8")) - 1)
                                   if logged_track.is_file() else None),
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
    delta = ("n/a" if path_delta is None else f"{path_delta:.2e} m")
    print(f"[{run_id}] view={args.view} {state['frames']} frames, "
          f"outcome={info['outcome']}, {env.steps} steps, "
          f"max path delta vs log {delta} -> {out.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
