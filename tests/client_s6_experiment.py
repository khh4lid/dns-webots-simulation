"""The S6 spatial-generalization experiment, for one layout.

Runs the pre-registered matrix on a single layout: 5 controllers x
(Mode B: 3 episodes + Mode C: 15 episodes) = 90 episodes. Everything the
protocol in ``10_EVIDENCE/S6_GENERALIZATION_PROTOCOL.md`` specifies is measured
here and nothing else is decided here -- the protocol was committed first.

Both modes run in one Webots session against the layout's ``_C`` world: Mode B
and Mode C share an identical static obstacle set by construction and the cart
node exists in both, so pointing the backend at the ``_B`` scenario parks the
cart on reset. That keeps B and C bit-comparable on the same loaded geometry.

Run::

    python scripts/with_webots.py --boot-wait 12 --timeout 1200 \\
        --world worlds/DNS_L03_C.wbt tests/client_s6_experiment.py -- --layout L03
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns import worldcheck                                       # noqa: E402
from dns.feasibility import astar, clearance_grid, path_length   # noqa: E402
from dns.scenario import Scenario                                # noqa: E402
from dns.spencer_link import DT, Navigation, find_ppo_dir        # noqa: E402

SCENARIOS = PROJECT_ROOT / "config" / "scenarios"

#: v1's seed base, reused so Mode C's 15 cart phases are identical to v1's.
TEST_START = 200_000_000
#: Protocol default: a near miss is clearance below this, and it is above the
#: 0.18 m collision radius so a near miss is never also a collision.
NEAR_MISS_M = 0.30
#: Full 720-beam range images are kept only for variant 00 of each
#: (layout, mode, controller) cell. Storing them for all 540 episodes would be
#: roughly 280 MB of float64. The cap is declared here and in the manifest so
#: the subsetting is never mistaken for full coverage.
LIDAR_ARCHIVE_VARIANTS = (0,)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_controllers(seeds=(7, 19, 43)) -> dict:
    """Spencer's five controllers, unmodified, with their file fingerprints."""
    ppo_dir = find_ppo_dir()
    from evaluate import gap_rule, goal_direct                   # noqa: E402
    from ppo import PPO                                          # noqa: E402

    controllers = {}
    for seed in seeds:
        path = ppo_dir / "evidence" / f"seed{seed}" / "model.npz"
        if not path.is_file():
            raise SystemExit(f"missing checkpoint {path}")
        controllers[f"ppo_seed{seed}"] = {
            "policy": PPO.load(path).predict, "kind": "ppo_checkpoint",
            "source": str(path), "sha256": sha256_file(path),
            "determinism": "tanh(actor(obs)); no sampling, no RNG"}
    evaluate_sha = sha256_file(ppo_dir / "evaluate.py")
    controllers["reactive"] = {
        "policy": gap_rule, "kind": "untuned_reference",
        "source": "evaluate.gap_rule", "sha256": evaluate_sha,
        "determinism": "deterministic function of the observation"}
    controllers["direct"] = {
        "policy": goal_direct, "kind": "untuned_reference",
        "source": "evaluate.goal_direct", "sha256": evaluate_sha,
        "determinism": "deterministic function of the observation"}
    return controllers


def phase_for(seed: int) -> float:
    """Deterministic crossing phase in [0, 2*pi). v1's rule, unchanged."""
    return float(np.random.default_rng(seed).uniform(0.0, 2.0 * np.pi))


def with_phase(scenario: Scenario, phase: float) -> Scenario:
    obstacles = []
    for obstacle in scenario.obstacles:
        if obstacle.motion.type == "linear" and obstacle.motion.amplitude_m:
            motion = dataclasses.replace(obstacle.motion, phase_rad=phase)
            obstacles.append(dataclasses.replace(obstacle, motion=motion))
        else:
            obstacles.append(obstacle)
    return dataclasses.replace(scenario, obstacles=tuple(obstacles))


def reference_length(scenario: Scenario) -> float | None:
    """A* path length for this layout, used as the path-efficiency numerator."""
    grid = clearance_grid(scenario, 0.0, 0.05)
    path = astar(grid, scenario.start_xy, scenario.goal_xy,
                 scenario.collision_radius_m + 0.02)
    return path_length(path) if path else None


def run_episode(backend, policy, seed: int, scene: str, *,
                run_id: str, archive_lidar: bool) -> tuple[dict, dict]:
    """One episode. Returns (metrics row, per-run artefacts)."""
    scenario = backend.scenario
    env = Navigation(backend)
    obs, _info = env.reset(seed, scene)

    trajectory = []
    clearances = []
    lidar_frames = []
    events = []

    def sample(step: int, commanded) -> None:
        t = backend.episode_time
        frame = env.frame
        gap = scenario.clearance_at(frame.xy, t)
        clearances.append(gap)
        record = backend.record
        trajectory.append({
            "step": step, "t_s": round(float(frame.time_s), 6),
            "x_m": round(float(frame.xy[0]), 5),
            "y_m": round(float(frame.xy[1]), 5),
            "altitude_m": round(float(record.altitude_m), 5),
            "yaw_rad": round(float(record.yaw_rad), 6),
            "roll_rad": round(float(record.roll_rad), 6),
            "pitch_rad": round(float(record.pitch_rad), 6),
            "vx_mps": round(float(frame.velocity[0]), 5),
            "vy_mps": round(float(frame.velocity[1]), 5),
            "cmd_vx_mps": round(float(commanded[0]), 5),
            "cmd_vy_mps": round(float(commanded[1]), 5),
            "clearance_m": round(float(gap), 5) if np.isfinite(gap) else "",
            "min_sector_m": round(float(np.min(frame.ranges[frame.valid])), 5)
                            if np.any(frame.valid) else "",
            "remaining_m": round(float(np.linalg.norm(
                np.asarray(scenario.goal_xy) - frame.xy)), 5),
        })
        if archive_lidar:
            lidar_frames.append({
                "t_s": float(frame.time_s),
                "xy": frame.xy.astype(float),
                "ranges": frame.ranges.astype(float),
                "valid": frame.valid.astype(bool),
                "obstacles": scenario.obstacle_centres(t).astype(float),
            })

    sample(0, np.zeros(2))
    flown = 0.0
    peak_roll = peak_pitch = 0.0
    while True:
        action = np.asarray(policy(obs), dtype=float)
        commanded = action * scenario.max_speed_mps
        obs, _reward, ended, cut, info = env.step(action)
        flown += info["step_m"]
        peak_roll = max(peak_roll, abs(float(backend.record.roll_rad)))
        peak_pitch = max(peak_pitch, abs(float(backend.record.pitch_rad)))
        sample(env.steps, commanded)
        if ended or cut:
            break

    t_end = backend.episode_time
    clearance = np.asarray([c for c in clearances if np.isfinite(c)], dtype=float)
    # Near misses are contiguous runs below the threshold, so a single long
    # grazing pass counts once rather than once per control step.
    below = clearance < NEAR_MISS_M
    near_miss_count = int(np.sum(below & ~np.concatenate(([False], below[:-1]))))
    near_miss_duration = float(np.sum(below) * DT)

    outcome = info["outcome"]
    if outcome == "collision":
        cause = worldcheck.classify_contact(scenario, env.frame.xy, t_end)
        outcome = f"collision_{cause.replace('_nearest', '')}"
        events.append({"step": env.steps, "t_s": round(t_end, 4),
                       "event": "collision", "attributed_to": cause,
                       "x_m": round(float(env.frame.xy[0]), 4),
                       "y_m": round(float(env.frame.xy[1]), 4)})

    reference = backend._s6_reference_length
    row = {
        "run_id": run_id,
        "outcome": outcome,
        "goal_reached": int(info["outcome"] == "goal"),
        "termination_step": env.steps,
        "time_to_goal_s": (round(env.steps * DT, 4)
                           if info["outcome"] == "goal" else ""),
        "sim_duration_s": round(env.steps * DT, 4),
        "path_length_m": round(flown, 4),
        "path_efficiency": (round(reference / flown, 4)
                            if (reference and flown > 0
                                and info["outcome"] == "goal") else ""),
        "astar_reference_m": round(reference, 4) if reference else "",
        "final_goal_distance_m": round(info["remaining_m"], 4),
        "min_clearance_m": round(float(clearance.min()), 4) if clearance.size else "",
        "mean_clearance_m": round(float(clearance.mean()), 4) if clearance.size else "",
        "near_miss_count": near_miss_count,
        "near_miss_duration_s": round(near_miss_duration, 4),
        "near_miss_threshold_m": NEAR_MISS_M,
        "max_abs_roll_rad": round(peak_roll, 4),
        "max_abs_pitch_rad": round(peak_pitch, 4),
        "tilt_limit_exceeded": int(max(peak_roll, peak_pitch) > 0.50),
        "min_sensed_clearance_m": round(min(
            (float(r["min_sector_m"]) for r in trajectory
             if r["min_sector_m"] != ""), default=float("nan")), 4),
    }
    artefacts = {"trajectory": trajectory, "events": events,
                 "lidar": lidar_frames}
    return row, artefacts


def write_run_folder(root: Path, run_id: str, manifest: dict,
                     artefacts: dict, scenario: Scenario) -> None:
    folder = root / run_id
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "manifest.json").write_text(json.dumps(manifest, indent=2),
                                          encoding="utf8")
    trajectory = artefacts["trajectory"]
    with (folder / "trajectory.csv").open("w", newline="", encoding="utf8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(trajectory[0]))
        writer.writeheader()
        writer.writerows(trajectory)

    with (folder / "obstacles.csv").open("w", newline="", encoding="utf8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["obstacle_id", "shape", "x_m", "y_m", "size_x_m",
                         "size_y_m", "height_m", "yaw_rad", "motion_type",
                         "amplitude_m", "speed_mps", "t0_x_m", "t0_y_m"])
        for obstacle in scenario.obstacles:
            centre = obstacle.centre_at(0.0)
            writer.writerow([obstacle.id, obstacle.shape,
                             round(obstacle.xy[0], 5), round(obstacle.xy[1], 5),
                             obstacle.size_xy[0], obstacle.size_xy[1],
                             obstacle.height_m, round(obstacle.yaw_rad, 6),
                             obstacle.motion.type, obstacle.motion.amplitude_m,
                             obstacle.motion.speed_mps,
                             round(float(centre[0]), 5), round(float(centre[1]), 5)])

    with (folder / "events.csv").open("w", newline="", encoding="utf8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["step", "t_s", "event", "attributed_to", "x_m", "y_m"])
        for event in artefacts["events"]:
            writer.writerow([event["step"], event["t_s"], event["event"],
                             event["attributed_to"], event["x_m"], event["y_m"]])

    if artefacts["lidar"]:
        frames = artefacts["lidar"]
        np.savez_compressed(
            folder / "lidar.npz",
            t_s=np.asarray([f["t_s"] for f in frames]),
            pose_xy=np.stack([f["xy"] for f in frames]),
            sector_ranges_m=np.stack([f["ranges"] for f in frames]),
            sector_valid=np.stack([f["valid"] for f in frames]),
            obstacle_centres_xy=np.stack([f["obstacles"] for f in frames]),
            sector_count=np.asarray(12),
            note=np.asarray("sector_ranges_m are the 12 packed sector minima "
                            "fed to navigation.observation; obstacle_centres_xy "
                            "are supervisor ground truth at the same instants"),
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", required=True)
    parser.add_argument("--reset-mode", default="settle_tolerance",
                        choices=("settle_tolerance", "deterministic"),
                        help="settle_tolerance reproduces the published "
                             "v1/v2 numbers; deterministic makes each "
                             "episode independently reproducible")
    parser.add_argument("--static-episodes", type=int, default=3)
    parser.add_argument("--dynamic-episodes", type=int, default=15)
    parser.add_argument("--out", type=Path,
                        default=PROJECT_ROOT / "results" / "s6")
    args = parser.parse_args()

    runs_root = args.out / "runs"
    runs_root.mkdir(parents=True, exist_ok=True)

    dynamic_path = SCENARIOS / f"{args.layout}_C.json"
    static_path = SCENARIOS / f"{args.layout}_B.json"
    dynamic = Scenario.load(dynamic_path)
    static = Scenario.load(static_path)
    digests = {
        "C": json.loads(dynamic_path.read_text(encoding="utf8"))["provenance"]["scenario_sha256"],
        "B": json.loads(static_path.read_text(encoding="utf8"))["provenance"]["scenario_sha256"],
    }
    controllers = load_controllers()

    from dns.backend import WebotsBackend                         # noqa: E402

    backend = WebotsBackend(dynamic)
    backend.reset_mode = args.reset_mode
    # Blocking check: refuse to produce results against a world that does not
    # implement the scenario. This is the v1 defect class made fatal.
    world_report = worldcheck.check(backend, dynamic, expect_sha256=digests["C"])
    print(f"[{args.layout}] world check OK, max t=0 pose error "
          f"{world_report['checks']['obstacle_t0_pose_max_error_m']:.2e} m",
          flush=True)

    references = {"C": reference_length(dynamic), "B": reference_length(static)}
    rows = []
    started = time.time()

    for mode, scenario_base, episodes in (
            ("B", static, args.static_episodes),
            ("C", dynamic, args.dynamic_episodes)):
        print(f"\n[{args.layout}] MODE {mode}: {episodes} episodes x "
              f"{len(controllers)} controllers", flush=True)
        for name, entry in controllers.items():
            for index in range(episodes):
                seed = TEST_START + index
                if mode == "C":
                    phase = phase_for(seed)
                    backend.scenario = with_phase(scenario_base, phase)
                else:
                    phase = None
                    backend.scenario = scenario_base
                backend._s6_reference_length = references[mode]

                run_id = f"{args.layout}_{mode}_{name}_v{index:02d}"
                archive = index in LIDAR_ARCHIVE_VARIANTS
                row, artefacts = run_episode(
                    backend, entry["policy"], seed,
                    "moving" if mode == "C" else "static",
                    run_id=run_id, archive_lidar=archive)

                row.update({
                    "layout_id": args.layout,
                    "layout_title": json.loads(
                        dynamic_path.read_text(encoding="utf8"))["provenance"]["layout_title"],
                    "mode": f"{mode}_{'dynamic' if mode == 'C' else 'static'}",
                    "controller": name,
                    "controller_kind": entry["kind"],
                    "controller_sha256": entry["sha256"],
                    "variant": f"v{index:02d}",
                    "variant_definition": (
                        f"cart initial phase = {phase:.6f} rad"
                        if phase is not None else
                        "identical configuration (repeat; seed unused by reset)"),
                    "distinct_configuration": 1 if mode == "C" else (1 if index == 0 else 0),
                    "seed": seed,
                    "obstacle_phase_rad": round(phase, 6) if phase is not None else "",
                    "scenario_name": backend.scenario.name,
                    "scenario_sha256": digests[mode],
                    "n_obstacles": len(backend.scenario.obstacles),
                })
                rows.append(row)

                write_run_folder(runs_root, run_id, {
                    "run_id": run_id,
                    "layout_id": args.layout,
                    "mode": row["mode"],
                    "mode_meaning": ("cart sweeping" if mode == "C"
                                     else "cart parked at arena edge"),
                    "variant": row["variant"],
                    "variant_definition": row["variant_definition"],
                    "distinct_configuration": bool(row["distinct_configuration"]),
                    "controller": name,
                    "controller_kind": entry["kind"],
                    "controller_source": entry["source"],
                    "controller_sha256": entry["sha256"],
                    "controller_determinism": entry["determinism"],
                    "scenario_sha256": digests[mode],
                    "scenario_file": f"07_SCENARIOS/{args.layout}_{mode}.json",
                    "snapshot": "webots_dns_BASELINE_SNAPSHOT_2026-11-24",
                    "webots_version": "R2025a",
                    "reset_mode": args.reset_mode,
                    "basic_timestep_ms": backend.scenario.basic_timestep_ms,
                    "observation_interval_s": backend.scenario.command_period_s,
                    "steps_per_command": backend.scenario.steps_per_command,
                    "horizon_steps": 90,
                    "seed": seed,
                    "outcome": row["outcome"],
                    "termination_step": row["termination_step"],
                    "lidar_archived": archive,
                    "lidar_archive_policy":
                        "full sector log kept for variant 00 of each cell only",
                    "world_check": world_report["checks"],
                }, artefacts, backend.scenario)

            group = [r for r in rows if r["controller"] == name
                     and r["mode"].startswith(mode)]
            goals = sum(r["goal_reached"] for r in group)
            print(f"  {name:<12} goal {goals}/{len(group)}", flush=True)

    columns = [
        "run_id", "layout_id", "layout_title", "mode", "controller",
        "controller_kind", "variant", "variant_definition",
        "distinct_configuration", "seed", "obstacle_phase_rad",
        "scenario_name", "scenario_sha256", "controller_sha256", "n_obstacles",
        "outcome", "goal_reached", "termination_step", "time_to_goal_s",
        "sim_duration_s", "path_length_m", "astar_reference_m",
        "path_efficiency", "final_goal_distance_m", "min_clearance_m",
        "mean_clearance_m", "near_miss_count", "near_miss_duration_s",
        "near_miss_threshold_m", "min_sensed_clearance_m",
        "max_abs_roll_rad", "max_abs_pitch_rad", "tilt_limit_exceeded",
    ]
    csv_path = args.out / f"results_{args.layout}.csv"
    with csv_path.open("w", newline="", encoding="utf8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)

    wall = time.time() - started
    (args.out / f"run_meta_{args.layout}.json").write_text(json.dumps({
        "layout": args.layout,
        "episodes": len(rows),
        "wall_clock_s": round(wall, 2),
        "wall_clock_per_episode_s": round(wall / max(len(rows), 1), 3),
        "world": Path(backend.robot.getWorldPath()).name,
        "scenario_sha256": digests,
        "controllers": {k: {"kind": v["kind"], "source": v["source"],
                            "sha256": v["sha256"]}
                        for k, v in controllers.items()},
        "protocol": "docs/verification/S6_GENERALIZATION_PROTOCOL.md",
        "reset_mode": args.reset_mode,
        "near_miss_threshold_m": NEAR_MISS_M,
        "lidar_archive_variants": list(LIDAR_ARCHIVE_VARIANTS),
    }, indent=2), encoding="utf8")

    print(f"\n[{args.layout}] {len(rows)} episodes in {wall:.1f} s "
          f"({wall / max(len(rows), 1):.2f} s/episode)", flush=True)
    print(f"[{args.layout}] wrote {csv_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
