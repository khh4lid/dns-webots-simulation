"""The DNS experiment campaign: static vs dynamic navigation, five controllers.

Design
------
Two conditions in one world, differing in exactly one factor:

* **MODE B, static** -- the crossing obstacle is parked at the arena edge.
  The layout is then fixed and physical mode is near-deterministic, so repeats
  measure run-to-run variation, **not** scenario variation. Few episodes are
  run and the result is reported as deterministic.
* **MODE C, dynamic** -- the same obstacle sweeps the route. Here each seed
  selects a different **initial phase** of the crossing, which genuinely
  changes when the obstacle arrives relative to the drone. Those episodes are
  distinct scenario instances, so a rate over them means something.

This phase randomisation is the honest way to get sample variety out of a fixed
layout. It is *not* a substitute for randomising obstacle positions, and no
claim of spatial generalisation is made from it.

Controllers are all Spencer's: three trained PPO checkpoints plus his two
scripted baselines, so "PPO" never silently means a hand-written rule.

Run::

    python scripts/with_webots.py --boot-wait 12 --timeout 3000 \\
        --world worlds/DNS_Research_World.wbt tests/client_experiments.py
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.scenario import Motion, Scenario                            # noqa: E402
from dns.spencer_link import DT, Navigation, find_ppo_dir            # noqa: E402

RESULTS = PROJECT_ROOT / "results" / "experiments"
TEST_START = 200_000_000


def load_controllers(seeds) -> dict:
    ppo_dir = find_ppo_dir()
    from evaluate import gap_rule, goal_direct                       # noqa: E402
    from ppo import PPO                                              # noqa: E402

    controllers = {
        "ppo_seed7": None, "ppo_seed19": None, "ppo_seed43": None,
    }
    controllers = {}
    for seed in seeds:
        path = ppo_dir / "evidence" / f"seed{seed}" / "model.npz"
        if path.is_file():
            controllers[f"ppo_seed{seed}"] = {
                "policy": PPO.load(path).predict,
                "kind": "ppo_checkpoint", "source": str(path)}
    controllers["reactive"] = {"policy": gap_rule, "kind": "scripted",
                               "source": "evaluate.gap_rule"}
    controllers["direct"] = {"policy": goal_direct, "kind": "scripted",
                             "source": "evaluate.goal_direct"}
    return controllers


def phase_for(seed: int) -> float:
    """Deterministic crossing phase in [0, 2*pi) from an episode seed."""
    return float(np.random.default_rng(seed).uniform(0.0, 2.0 * np.pi))


def with_phase(scenario: Scenario, phase: float) -> Scenario:
    """Copy the scenario with the moving obstacle's phase replaced."""
    obstacles = []
    for obstacle in scenario.obstacles:
        if obstacle.motion.type == "linear" and obstacle.motion.amplitude_m:
            motion = dataclasses.replace(obstacle.motion, phase_rad=phase)
            obstacles.append(dataclasses.replace(obstacle, motion=motion))
        else:
            obstacles.append(obstacle)
    return dataclasses.replace(scenario, obstacles=tuple(obstacles))


def run_episode(backend, policy, seed: int, scene: str,
                trace: list | None = None) -> dict:
    """One episode. Returns every field the results table needs.

    When ``trace`` is supplied it is filled with the per-step record the
    trajectory and time-series figures are drawn from, so the figures and the
    results table come from the same run rather than from separate executions.
    """
    env = Navigation(backend)
    obs, _ = env.reset(seed, scene)
    if trace is not None:
        trace.append({"t": env.frame.time_s, "xy": env.frame.xy.tolist(),
                      "v": env.frame.velocity.tolist(), "cmd": [0.0, 0.0],
                      "nearest": None,
                      "movers": backend.scenario.obstacle_centres(
                          backend.episode_time).tolist()})
    covered = env.frame.ranges[env.frame.valid]
    nearest_sensed = float(covered.min()) if covered.size else float("nan")

    path_m = total_reward = 0.0
    tracking_sq = []
    nearest_truth = float("inf")
    start_xy = env.frame.xy.copy()

    while True:
        action = np.asarray(policy(obs), dtype=float)
        commanded = action * backend.scenario.max_speed_mps
        obs, reward, ended, cut, info = env.step(action)
        total_reward += reward
        path_m += info["step_m"]
        if np.isfinite(info["nearest_m"]) and info["nearest_m"] > 0:
            nearest_sensed = min(nearest_sensed, info["nearest_m"])
        tracking_sq.append(np.sum((env.frame.velocity - commanded) ** 2))
        if trace is not None:
            trace.append({"t": env.frame.time_s, "xy": env.frame.xy.tolist(),
                          "v": env.frame.velocity.tolist(),
                          "cmd": commanded.tolist(),
                          "nearest": info["nearest_m"],
                          "movers": backend.scenario.obstacle_centres(
                              backend.episode_time).tolist()})

        # Supervisor ground-truth clearance, independent of the sensor. Uses
        # the exact footprint, so a rectangular rack reports the distance to
        # its face rather than to a circle drawn around it.
        gap = backend.scenario.clearance_at(env.frame.xy, backend.episode_time)
        if np.isfinite(gap):
            nearest_truth = min(nearest_truth, gap)
        if ended or cut:
            break

    return {
        "scenario": backend.scenario.name,
        "seed": seed,
        "scene_label": scene,
        "start_x_m": round(float(start_xy[0]), 4),
        "start_y_m": round(float(start_xy[1]), 4),
        "goal_x_m": float(backend.scenario.goal_xy[0]),
        "goal_y_m": float(backend.scenario.goal_xy[1]),
        "n_static_obstacles": sum(1 for o in backend.scenario.obstacles
                                  if o.motion.type == "static"
                                  or not o.motion.amplitude_m),
        "n_moving_obstacles": sum(1 for o in backend.scenario.obstacles
                                  if o.motion.type != "static"
                                  and o.motion.amplitude_m),
        "obstacle_speed_mps": max(
            [o.motion.speed_mps for o in backend.scenario.obstacles
             if o.motion.type != "static" and o.motion.amplitude_m] or [0.0]),
        "outcome": info["outcome"],
        "goal_reached": int(info["outcome"] == "goal"),
        "collision": int(info["outcome"] == "collision"),
        "timeout": int(info["outcome"] == "timeout"),
        "control_steps": env.steps,
        "sim_duration_s": round(env.steps * DT, 4),
        "path_length_m": round(path_m, 4),
        "final_goal_distance_m": round(info["remaining_m"], 4),
        "min_sensed_clearance_m": round(nearest_sensed, 4),
        "min_truth_clearance_m": (round(nearest_truth, 4)
                                  if np.isfinite(nearest_truth) else ""),
        "velocity_tracking_rmse_mps": round(
            float(np.sqrt(np.mean(tracking_sq))), 4) if tracking_sq else "",
        "return_sum": round(total_reward, 4),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dynamic-config", type=Path,
                        default=PROJECT_ROOT / "config" / "scenario_research_demo.json")
    parser.add_argument("--static-config", type=Path,
                        default=PROJECT_ROOT / "config" / "scenario_demo_static.json")
    parser.add_argument("--static-episodes", type=int, default=3)
    parser.add_argument("--dynamic-episodes", type=int, default=15)
    parser.add_argument("--seeds", type=int, nargs="+", default=(7, 19, 43))
    parser.add_argument("--out", type=Path, default=RESULTS)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    dynamic = Scenario.load(args.dynamic_config)
    static = Scenario.load(args.static_config)
    controllers = load_controllers(args.seeds)
    print("controllers: " + ", ".join(controllers))

    from dns.backend import WebotsBackend                            # noqa: E402

    backend = WebotsBackend(static)
    rows: list[dict] = []
    traces: dict[str, list] = {}
    started = time.time()

    print(f"\n[MODE B] static, {args.static_episodes} episodes each "
          "(fixed layout -- repeats measure determinism only)")
    backend.scenario = static
    for name, entry in controllers.items():
        for i in range(args.static_episodes):
            trace = [] if i == 0 else None
            row = run_episode(backend, entry["policy"], TEST_START + i,
                              "static", trace)
            if trace is not None:
                traces[f"B_static/{name}"] = trace
            row.update(experiment_id=f"B-{name}-{i:02d}", mode="B_static",
                       controller=name, controller_kind=entry["kind"],
                       obstacle_phase_rad="")
            rows.append(row)
        goals = sum(r["goal_reached"] for r in rows if r["controller"] == name
                    and r["mode"] == "B_static")
        print(f"  {name:<12} goal {goals}/{args.static_episodes}")

    print(f"\n[MODE C] dynamic, {args.dynamic_episodes} episodes each "
          "(each seed = a different crossing phase)")
    for name, entry in controllers.items():
        for i in range(args.dynamic_episodes):
            seed = TEST_START + i
            phase = phase_for(seed)
            backend.scenario = with_phase(dynamic, phase)
            trace = [] if i == 0 else None
            row = run_episode(backend, entry["policy"], seed, "moving", trace)
            if trace is not None:
                traces[f"C_dynamic/{name}"] = trace
            row.update(experiment_id=f"C-{name}-{i:02d}", mode="C_dynamic",
                       controller=name, controller_kind=entry["kind"],
                       obstacle_phase_rad=round(phase, 6))
            rows.append(row)
        group = [r for r in rows if r["controller"] == name
                 and r["mode"] == "C_dynamic"]
        print(f"  {name:<12} goal {sum(r['goal_reached'] for r in group)}"
              f"/{len(group)}  collision "
              f"{sum(r['collision'] for r in group)}  timeout "
              f"{sum(r['timeout'] for r in group)}")

    wall_s = time.time() - started

    # ---------------------------------------------------------------- output
    columns = ["experiment_id", "mode", "scenario", "controller",
               "controller_kind", "seed", "obstacle_phase_rad", "scene_label",
               "start_x_m", "start_y_m", "goal_x_m", "goal_y_m",
               "n_static_obstacles", "n_moving_obstacles", "obstacle_speed_mps",
               "outcome", "goal_reached", "collision", "timeout",
               "control_steps", "sim_duration_s", "path_length_m",
               "final_goal_distance_m", "min_sensed_clearance_m",
               "min_truth_clearance_m", "velocity_tracking_rmse_mps",
               "return_sum"]
    csv_path = args.out / "experiment_results.csv"
    with csv_path.open("w", newline="", encoding="utf8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)

    summary = []
    for mode in ("B_static", "C_dynamic"):
        for name in controllers:
            group = [r for r in rows if r["mode"] == mode
                     and r["controller"] == name]
            if not group:
                continue
            goals = [r for r in group if r["goal_reached"]]
            n = len(group)
            successes = len(goals)
            # Wilson 95% interval: sane at these sample sizes, unlike normal
            p = successes / n
            z = 1.959964
            denominator = 1 + z * z / n
            centre = (p + z * z / (2 * n)) / denominator
            half = (z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
                    / denominator)
            summary.append({
                "mode": mode, "controller": name,
                "kind": controllers[name]["kind"], "episodes": n,
                "goal": successes,
                "collision": sum(r["collision"] for r in group),
                "timeout": sum(r["timeout"] for r in group),
                "success_rate": round(p, 4),
                "success_rate_ci95_low": round(max(0.0, centre - half), 4),
                "success_rate_ci95_high": round(min(1.0, centre + half), 4),
                "mean_path_m": round(float(np.mean(
                    [r["path_length_m"] for r in group])), 4),
                "mean_completion_s": (round(float(np.mean(
                    [r["sim_duration_s"] for r in goals])), 4) if goals else ""),
                "mean_min_truth_clearance_m": round(float(np.mean(
                    [r["min_truth_clearance_m"] for r in group
                     if r["min_truth_clearance_m"] != ""])), 4),
                "mean_velocity_rmse_mps": round(float(np.mean(
                    [r["velocity_tracking_rmse_mps"] for r in group
                     if r["velocity_tracking_rmse_mps"] != ""])), 4),
            })
    summary_path = args.out / "experiment_summary.csv"
    with summary_path.open("w", newline="", encoding="utf8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)

    (args.out / "protocol.json").write_text(json.dumps({
        "webots_version": "R2025a",
        "world": "worlds/DNS_Research_World.wbt",
        "configs": {"static": str(args.static_config),
                    "dynamic": str(args.dynamic_config)},
        "modes": {
            "B_static": "crossing obstacle parked at the arena edge; fixed "
                        "layout, physical mode near-deterministic, so repeats "
                        "measure run-to-run variation only",
            "C_dynamic": "crossing obstacle sweeping; each seed selects a "
                         "different initial phase, so episodes are distinct "
                         "scenario instances",
        },
        "episodes_per_controller": {"B_static": args.static_episodes,
                                    "C_dynamic": args.dynamic_episodes},
        "seeds": [TEST_START + i for i in range(max(args.static_episodes,
                                                    args.dynamic_episodes))],
        "phase_rule": "numpy default_rng(seed).uniform(0, 2*pi)",
        "horizon_steps": 90, "command_period_s": DT,
        "max_speed_mps_per_axis": dynamic.max_speed_mps,
        "goal_tolerance_m": dynamic.goal_tolerance_m,
        "collision_rule": dynamic.collision_mode,
        "sensing": {"mode": dynamic.sensing_mode,
                    "mount": dynamic.lidar_mount_mode,
                    "beams": dynamic.lidar_resolution,
                    "max_range_m": dynamic.sensing_max_range_m},
        "observation_builder": "navigation.observation (Spencer, unmodified)",
        "controllers": {k: {"kind": v["kind"], "source": v["source"]}
                        for k, v in controllers.items()},
        "denominator_note": "every rate is successes / episodes in that "
                            "mode+controller cell; no episode was excluded",
        "limitation": "ONE fixed obstacle layout throughout. Mode C varies the "
                      "crossing phase only. No spatial-generalisation claim.",
        "wall_clock_s": round(wall_s, 2),
        "total_episodes": len(rows),
    }, indent=2), encoding="utf8")

    (args.out / "traces.json").write_text(json.dumps(traces), encoding="utf8")

    print(f"\n{len(rows)} episodes in {wall_s:.1f} s wall clock")
    print(f"wrote {csv_path}")
    print(f"wrote {summary_path}")
    print(f"wrote {args.out / 'protocol.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
