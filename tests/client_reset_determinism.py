"""Is a single episode reproducible on its own, or only in sequence?

The L00 reproduction showed all ten cells reproducing with a maximum path
difference of 0 m -- but that re-ran the **whole 90-episode session** in the
same order. A separate observation while capturing footage suggested something
weaker: replaying one episode in a fresh session drifted 0.287 m from the log,
and only replaying the session prefix first brought it back to 5e-06 m.

Those two facts are consistent with the simulator being deterministic given the
episode SEQUENCE but not given an episode in isolation. This client tests that
directly, because the difference matters: if an episode's outcome depends on
the state left by its predecessor, the 15 cart-phase variants in a cell are not
independent draws, and that is a threat to validity worth stating.

Procedure, for one target variant per invocation:

* ``--isolated``  reset and fly the target as the FIRST episode after the world
  loads, so nothing precedes it;
* ``--sequence``  replay the exact session prefix first, then fly the target.

Both log the post-settle initial state (the thing a reset is supposed to make
identical) and the full trajectory, so the comparison is against the logged run
and against each other.

Run via scripts/run_reset_study.py, which drives both arms for several variants.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.scenario import Scenario                                # noqa: E402
from dns.spencer_link import DT, Navigation, find_ppo_dir        # noqa: E402

SCENARIOS = PROJECT_ROOT / "config" / "scenarios"
RUNS = PROJECT_ROOT / "results" / "v2_S6" / "runs"
OUT = PROJECT_ROOT / "results" / "reset_study"
TEST_START = 200_000_000
ORDER = ["ppo_seed7", "ppo_seed19", "ppo_seed43", "reactive", "direct"]


def phase_for(seed: int) -> float:
    return float(np.random.default_rng(seed).uniform(0.0, 2.0 * np.pi))


def with_phase(scenario: Scenario, phase: float) -> Scenario:
    import dataclasses
    obstacles = []
    for obstacle in scenario.obstacles:
        if obstacle.motion.type == "linear" and obstacle.motion.amplitude_m:
            obstacles.append(dataclasses.replace(
                obstacle, motion=dataclasses.replace(obstacle.motion,
                                                     phase_rad=phase)))
        else:
            obstacles.append(obstacle)
    return dataclasses.replace(scenario, obstacles=tuple(obstacles))


def load_policy(name: str):
    ppo_dir = find_ppo_dir()
    if name.startswith("ppo_seed"):
        from ppo import PPO                                      # noqa: E402
        return PPO.load(ppo_dir / "evidence"
                        / f"seed{name.removeprefix('ppo_seed')}"
                        / "model.npz").predict
    from evaluate import gap_rule, goal_direct                   # noqa: E402
    return {"reactive": gap_rule, "direct": goal_direct}[name]


def fly(backend, policy, seed: int, scene: str) -> dict:
    """One episode. Returns the post-settle start state and the trajectory."""
    env = Navigation(backend)
    obs, _info = env.reset(seed, scene)
    record = backend.record
    start = {
        "x_m": float(env.frame.xy[0]), "y_m": float(env.frame.xy[1]),
        "altitude_m": float(record.altitude_m),
        "roll_rad": float(record.roll_rad), "pitch_rad": float(record.pitch_rad),
        "vx_mps": float(env.frame.velocity[0]),
        "vy_mps": float(env.frame.velocity[1]),
    }
    track = [[float(env.frame.xy[0]), float(env.frame.xy[1])]]
    while True:
        obs, _r, ended, cut, info = env.step(np.asarray(policy(obs), dtype=float))
        track.append([float(env.frame.xy[0]), float(env.frame.xy[1])])
        if ended or cut:
            break
    return {"start_state": start, "track": track, "outcome": info["outcome"],
            "steps": env.steps,
            "final_distance_m": round(info["remaining_m"], 6)}


def logged_track(run_id: str):
    path = RUNS / run_id / "trajectory.csv"
    if not path.is_file():
        return None
    with path.open(encoding="utf8") as handle:
        rows = list(csv.DictReader(handle))
    return np.array([[float(r["x_m"]), float(r["y_m"])] for r in rows])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", default="L00")
    parser.add_argument("--controller", default="ppo_seed7")
    parser.add_argument("--variant", type=int, required=True)
    parser.add_argument("--arm", required=True, choices=("isolated", "sequence"))
    args = parser.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    static = Scenario.load(SCENARIOS / f"{args.layout}_B.json")
    dynamic = Scenario.load(SCENARIOS / f"{args.layout}_C.json")
    seed = TEST_START + args.variant
    target = with_phase(dynamic, phase_for(seed))

    from dns.backend import WebotsBackend                         # noqa: E402

    backend = WebotsBackend(target)

    prefix_episodes = 0
    if args.arm == "sequence":
        # The experiment's own order: the whole Mode B block, then Mode C up to
        # the target.
        prefix: list[tuple[str, str, int]] = []
        for name in ORDER:
            prefix += [("B", name, i) for i in range(3)]
        for name in ORDER:
            for i in range(15):
                if name == args.controller and i >= args.variant:
                    break
                prefix.append(("C", name, i))
            if name == args.controller:
                break
        for mode, name, index in prefix:
            warm_seed = TEST_START + index
            backend.scenario = (with_phase(dynamic, phase_for(warm_seed))
                                if mode == "C" else static)
            fly(backend, load_policy(name), warm_seed,
                "moving" if mode == "C" else "static")
        prefix_episodes = len(prefix)

    backend.scenario = target
    result = fly(backend, load_policy(args.controller), seed, "moving")

    run_id = f"{args.layout}_C_{args.controller}_v{args.variant:02d}"
    reference = logged_track(run_id)
    track = np.asarray(result["track"])
    delta = None
    if reference is not None:
        n = min(len(reference), len(track))
        delta = float(np.abs(reference[:n] - track[:n]).max())

    report = {
        "run_id": run_id, "arm": args.arm, "variant": args.variant,
        "prefix_episodes_replayed": prefix_episodes,
        "outcome": result["outcome"], "steps": result["steps"],
        "final_distance_m": result["final_distance_m"],
        "start_state": result["start_state"],
        "max_path_delta_vs_log_m": delta,
        "steps_vs_log": None,
        "track": [[round(v, 6) for v in p] for p in result["track"]],
    }
    if reference is not None:
        report["steps_vs_log"] = len(track) - len(reference)

    (OUT / f"{run_id}_{args.arm}.json").write_text(
        json.dumps(report, indent=2), encoding="utf8")
    print(f"[{run_id}] arm={args.arm:<8} prefix={prefix_episodes:>2} "
          f"outcome={result['outcome']:<10} steps={result['steps']:>3} "
          f"delta_vs_log="
          f"{'n/a' if delta is None else f'{delta:.3e} m'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
