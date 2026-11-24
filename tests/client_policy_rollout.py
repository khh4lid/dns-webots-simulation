"""M6: fly Spencer's trained PPO checkpoints in Webots, end to end.

This is the first test where nothing is hand-chosen: every velocity command
comes from a trained policy reading a 52-feature observation built by Spencer's
own ``navigation.observation`` from real Webots lidar returns and a measured
GPS velocity.

Chain under test::

    Webots lidar + GPS  ->  dns.backend.WebotsBackend  ->  navigation.Frame
      ->  navigation.observation (52 features)  ->  PPO.predict  ->  [a_x, a_y]
      ->  x1.2  ->  [vx, vy] m/s  ->  Autopilot (125 Hz)  ->  rotor thrust
      ->  Webots physics  ->  measured motion

Controllers compared, all of them Spencer's code:

* ``seed7`` / ``seed19`` / ``seed43`` -- trained PPO checkpoints
* ``direct``   -- ``evaluate.goal_direct``, head straight at the goal
* ``reactive`` -- ``evaluate.gap_rule``, goal attraction plus sector repulsion

**Scope limit, stated up front:** every episode runs the *same* fixed
``default_10x6`` layout, so repeated episodes differ only through flight-control
noise, not through map variation. These numbers are a pilot that shows the
closed loop works in a physically simulated environment. They are **not** a
success-rate estimate and **not** a generalisation result -- that needs the
randomised layouts of scenario S6.

Run via::

    python scripts/with_webots.py --timeout 1800 tests/client_policy_rollout.py -- --episodes 3
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

from dns.scenario import DEFAULT_SCENARIO_PATH, Scenario  # noqa: E402
from dns.spencer_link import DT, Navigation, find_ppo_dir  # noqa: E402

RESULTS = PROJECT_ROOT / "results"
#: Matches evaluate.TEST_START so map ids never overlap Spencer's training maps.
TEST_START = 200_000_000


def load_controllers(seeds: tuple[int, ...]) -> dict:
    """Spencer's two scripted baselines plus every available PPO checkpoint."""
    ppo_dir = find_ppo_dir()
    from evaluate import gap_rule, goal_direct      # noqa: E402  (his module)
    from ppo import PPO                             # noqa: E402

    controllers: dict[str, dict] = {
        "direct": {"policy": goal_direct, "kind": "scripted", "source": "evaluate.goal_direct"},
        "reactive": {"policy": gap_rule, "kind": "scripted", "source": "evaluate.gap_rule"},
    }
    for seed in seeds:
        path = ppo_dir / "evidence" / f"seed{seed}" / "model.npz"
        if not path.is_file():
            print(f"  ! no checkpoint at {path}, skipping")
            continue
        model = PPO.load(path)
        controllers[f"ppo_seed{seed}"] = {
            "policy": model.predict, "kind": "ppo_checkpoint",
            "source": str(path),
            "config": json.loads((path.parent / "config.json").read_text(encoding="utf8"))
            if (path.parent / "config.json").is_file() else None,
        }
    return controllers


def rollout(env: Navigation, policy, seed: int, scene: str) -> tuple[dict, list]:
    """One episode. Mirrors evaluate.episode but reuses the single backend.

    Webots allows exactly one Robot per controller process, so Spencer's
    ``evaluate.episode`` (which constructs a backend per call) cannot be used
    directly. The metric names are kept identical so the two sets of numbers
    stay comparable.
    """
    obs, _ = env.reset(seed, scene)
    trace = [dict(t=env.frame.time_s, xy=env.frame.xy.tolist(),
                  v=env.frame.velocity.tolist(), cmd=[0.0, 0.0],
                  nearest=None)]
    path_m = total_reward = 0.0
    covered = env.frame.ranges[env.frame.valid]
    nearest = float(covered.min()) if covered.size else 0.0

    while True:
        action = np.asarray(policy(obs), dtype=float)
        obs, reward, ended, cut, info = env.step(action)
        total_reward += reward
        path_m += info["step_m"]
        nearest = min(nearest, info["nearest_m"])
        trace.append(dict(t=env.frame.time_s, xy=env.frame.xy.tolist(),
                          v=env.frame.velocity.tolist(),
                          cmd=(action * 1.2).tolist(),
                          nearest=info["nearest_m"]))
        if ended or cut:
            break

    metrics = dict(map_seed=seed, scene=scene, outcome=info["outcome"],
                   steps=env.steps, duration_s=env.steps * DT, path_m=path_m,
                   return_sum=total_reward, min_sensed_range_m=nearest,
                   remaining_m=info["remaining_m"])
    return metrics, trace


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_SCENARIO_PATH)
    parser.add_argument("--episodes", type=int, default=3,
                        help="episodes per controller on the fixed layout")
    parser.add_argument("--seeds", type=int, nargs="+", default=(7, 19, 43))
    parser.add_argument("--sensing", choices=("lidar", "geometric", "none"),
                        default="lidar")
    parser.add_argument("--out", type=Path, default=RESULTS / "policy_rollout")
    args = parser.parse_args()

    scenario = dataclasses.replace(Scenario.load(args.config),
                                   dynamics_mode="physical",
                                   sensing_mode=args.sensing)
    scenario.validate()
    scene = ("moving" if any(o.motion.type != "static" for o in scenario.obstacles)
             else "static")

    print(f"scenario   : {scenario.name}   scene label: {scene}")
    print(f"dynamics   : {scenario.dynamics_mode}")
    print(f"sensing    : {scenario.sensing_mode} "
          f"(mount {scenario.lidar_mount_mode}, {scenario.lidar_resolution} beams)")
    print(f"start/goal : {list(scenario.start_xy)} -> {list(scenario.goal_xy)} "
          f"(tolerance {scenario.goal_tolerance_m} m)")
    print("controllers:")
    controllers = load_controllers(tuple(args.seeds))
    for name, entry in controllers.items():
        print(f"  {name:<14} {entry['kind']:<15} {entry['source']}")
    print()

    from dns.backend import WebotsBackend  # noqa: E402

    backend = WebotsBackend(scenario)
    env = Navigation(backend)

    rows: list[dict] = []
    traces: dict[str, list] = {}
    started = time.time()
    for name, entry in controllers.items():
        for i in range(args.episodes):
            seed = TEST_START + i
            metrics, trace = rollout(env, entry["policy"], seed, scene)
            metrics["controller"] = name
            metrics["kind"] = entry["kind"]
            metrics["episode"] = i
            rows.append(metrics)
            traces.setdefault(name, []).append(trace)
            print(f"  {name:<14} ep{i}  {metrics['outcome']:<9} "
                  f"steps {metrics['steps']:>3}  "
                  f"path {metrics['path_m']:>5.2f} m  "
                  f"remaining {metrics['remaining_m']:>5.2f} m  "
                  f"min sensed {metrics['min_sensed_range_m']:>5.2f} m  "
                  f"return {metrics['return_sum']:>7.2f}")
    wall_s = time.time() - started

    # ---------------------------------------------------------------- summary
    summary = []
    for name in controllers:
        group = [r for r in rows if r["controller"] == name]
        goals = [r for r in group if r["outcome"] == "goal"]
        summary.append(dict(
            controller=name, kind=controllers[name]["kind"], episodes=len(group),
            goal=sum(r["outcome"] == "goal" for r in group),
            collision=sum(r["outcome"] == "collision" for r in group),
            timeout=sum(r["outcome"] == "timeout" for r in group),
            mean_return=float(np.mean([r["return_sum"] for r in group])),
            mean_path_m=float(np.mean([r["path_m"] for r in group])),
            mean_min_sensed_m=float(np.mean([r["min_sensed_range_m"] for r in group])),
            mean_final_remaining_m=float(np.mean([r["remaining_m"] for r in group])),
            mean_goal_time_s=float(np.mean([r["duration_s"] for r in goals]))
            if goals else None,
        ))

    args.out.mkdir(parents=True, exist_ok=True)
    with (args.out / "episodes.csv").open("w", newline="", encoding="utf8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with (args.out / "summary.csv").open("w", newline="", encoding="utf8") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)
    (args.out / "traces.json").write_text(json.dumps(traces), encoding="utf8")
    (args.out / "protocol.json").write_text(json.dumps({
        "scenario": scenario.name,
        "config": str(args.config),
        "layout": "SINGLE FIXED LAYOUT -- repeated episodes vary only by "
                  "flight-control noise. Not a success-rate estimate and not a "
                  "generalisation test; that requires randomised layouts (S6).",
        "dynamics": scenario.dynamics_mode,
        "sensing": scenario.sensing_mode,
        "lidar_mount": scenario.lidar_mount_mode,
        "lidar_beams": scenario.lidar_resolution,
        "collision": scenario.collision_mode,
        "scene_label": scene,
        "episodes_per_controller": args.episodes,
        "map_seeds": [TEST_START + i for i in range(args.episodes)],
        "horizon_steps": 90,
        "command_period_s": DT,
        "max_speed_mps_per_axis": scenario.max_speed_mps,
        "goal_tolerance_m": scenario.goal_tolerance_m,
        "webots_version": "R2025a",
        "observation_builder": "navigation.observation (Spencer, unmodified)",
        "controllers": {k: {"kind": v["kind"], "source": v["source"]}
                        for k, v in controllers.items()},
        "wall_clock_s": wall_s,
    }, indent=2), encoding="utf8")

    print(f"\n--- summary ({args.episodes} episodes each, ONE fixed layout)")
    print(f"{'controller':<14} {'kind':<15} {'goal':>5} {'coll':>5} {'t/o':>4} "
          f"{'mean path':>10} {'mean ret':>9} {'min sensed':>11}")
    for s in summary:
        print(f"{s['controller']:<14} {s['kind']:<15} "
              f"{s['goal']:>5} {s['collision']:>5} {s['timeout']:>4} "
              f"{s['mean_path_m']:>9.2f}m {s['mean_return']:>9.2f} "
              f"{s['mean_min_sensed_m']:>10.2f}m")
    print(f"\nwall clock {wall_s:.1f} s for {len(rows)} episodes")
    print(f"wrote {args.out}/episodes.csv, summary.csv, traces.json, protocol.json")

    ppo_rows = [r for r in rows if r["kind"] == "ppo_checkpoint"]
    if not ppo_rows:
        print("\nNo PPO checkpoint ran: milestone M6 BLOCKED")
        return 1
    print(f"\nM6 closed loop exercised: {len(ppo_rows)} PPO episodes, "
          f"{sum(r['outcome'] == 'goal' for r in ppo_rows)} reached goal, "
          f"{sum(r['outcome'] == 'collision' for r in ppo_rows)} collided, "
          f"{sum(r['outcome'] == 'timeout' for r in ppo_rows)} timed out")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
