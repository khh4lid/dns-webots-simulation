"""Extended reset validation: does an episode depend on what preceded it?

The decisive test is not "isolated versus sequential" but **the same episode
after different predecessors**. If a reset truly erases history, running target
T after distractor A must give exactly the same result as running T after
distractor B, and as running T first.

That can all be done inside one Webots session, which makes a large battery
affordable: for each target, the client runs

    T  ->  A  ->  T  ->  B  ->  T

and compares the three T runs against each other, component by component.
Different targets use different distractors, so several preceding orders are
covered. A second session repeats the battery with the other reset mode, so the
two are compared under identical conditions.

Everything compared is a state the reset is supposed to restore: position,
orientation, linear and angular velocity, the autopilot integrator, the packed
sensor vector, the obstacle phase, and the episode clock.

Run::

    python scripts/with_webots.py --boot-wait 12 --timeout 1800 \\
        --world worlds/DNS_L00_C.wbt tests/client_reset_validation.py -- \\
        --reset-mode deterministic
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
OUT = PROJECT_ROOT / "results" / "reset_validation"
TEST_START = 200_000_000

#: (controller, variant) targets. Five distinct episodes across two
#: controllers, spanning both outcomes and a range of cart phases.
TARGETS = [
    ("ppo_seed7", 0), ("ppo_seed7", 1), ("ppo_seed7", 5),
    ("ppo_seed19", 2), ("ppo_seed19", 7),
]
#: Distractors run between repeats, so each target is preceded by two
#: different episode orders as well as by nothing.
DISTRACTORS = [("ppo_seed43", 3), ("reactive", 9), ("direct", 4),
               ("ppo_seed43", 11), ("reactive", 6)]


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


def capture_state(backend, env) -> dict:
    """Everything a reset is supposed to restore, read after reset."""
    drone = backend.drone
    pose = list(drone.getField("translation").getSFVec3f())
    rotation = list(drone.getField("rotation").getSFRotation())
    try:
        velocity = list(drone.getVelocity())          # linear xyz + angular xyz
    except Exception:                                 # noqa: BLE001
        velocity = [float("nan")] * 6
    record = backend.record
    frame = env.frame
    movers = [o for o in backend.scenario.obstacles
              if o.motion.type != "static" and o.motion.amplitude_m]
    return {
        "translation": pose,
        "rotation": rotation,
        "velocity_linear": velocity[:3],
        "velocity_angular": velocity[3:],
        "roll_rad": float(record.roll_rad),
        "pitch_rad": float(record.pitch_rad),
        "yaw_rad": float(record.yaw_rad),
        "measured_velocity_dns": [float(v) for v in frame.velocity],
        "autopilot_integral": [float(v) for v in backend.autopilot._integral],
        "sector_ranges": [float(v) for v in frame.ranges],
        "sector_valid": [bool(v) for v in frame.valid],
        "episode_time_s": float(frame.time_s),
        "sim_time_s": float(backend.robot.getTime()),
        "cart_y_m": (float(movers[0].centre_at(0.0)[1]) if movers else None),
        "cart_phase_rad": (float(movers[0].motion.phase_rad) if movers else None),
    }


def fly(backend, name: str, variant: int, dynamic: Scenario) -> dict:
    seed = TEST_START + variant
    backend.scenario = with_phase(dynamic, phase_for(seed))
    env = Navigation(backend)
    obs, _info = env.reset(seed, "moving")
    state = capture_state(backend, env)
    track = [[float(env.frame.xy[0]), float(env.frame.xy[1])]]
    while True:
        obs, _r, ended, cut, info = env.step(
            np.asarray(load_policy(name)(obs), dtype=float))
        track.append([float(env.frame.xy[0]), float(env.frame.xy[1])])
        if ended or cut:
            break
    return {"controller": name, "variant": variant, "outcome": info["outcome"],
            "steps": env.steps, "state": state, "track": track,
            "final_distance_m": round(info["remaining_m"], 9)}


def compare(a: dict, b: dict) -> dict:
    """Component-wise difference between two runs of the same episode."""
    sa, sb = a["state"], b["state"]
    out = {"outcome_same": a["outcome"] == b["outcome"],
           "steps_same": a["steps"] == b["steps"],
           "steps_a": a["steps"], "steps_b": b["steps"]}
    for key in ("translation", "rotation", "velocity_linear",
                "velocity_angular", "measured_velocity_dns",
                "autopilot_integral", "sector_ranges"):
        va, vb = np.asarray(sa[key], dtype=float), np.asarray(sb[key], dtype=float)
        out[f"{key}_maxdiff"] = float(np.abs(va - vb).max())
    for key in ("roll_rad", "pitch_rad", "yaw_rad", "episode_time_s",
                "cart_y_m", "cart_phase_rad"):
        if sa[key] is None or sb[key] is None:
            continue
        out[f"{key}_diff"] = abs(float(sa[key]) - float(sb[key]))
    out["sector_valid_same"] = sa["sector_valid"] == sb["sector_valid"]
    ta, tb = np.asarray(a["track"]), np.asarray(b["track"])
    n = min(len(ta), len(tb))
    out["track_maxdiff_m"] = float(np.abs(ta[:n] - tb[:n]).max())
    out["track_len_a"], out["track_len_b"] = len(ta), len(tb)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reset-mode", default="deterministic",
                        choices=("settle_tolerance", "deterministic"))
    parser.add_argument("--layout", default="L00")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    dynamic = Scenario.load(SCENARIOS / f"{args.layout}_C.json")
    from dns.backend import WebotsBackend                         # noqa: E402
    backend = WebotsBackend(dynamic)
    backend.reset_mode = args.reset_mode
    print(f"reset_mode = {args.reset_mode}", flush=True)

    results = []
    for index, (name, variant) in enumerate(TARGETS):
        d1 = DISTRACTORS[index % len(DISTRACTORS)]
        d2 = DISTRACTORS[(index + 1) % len(DISTRACTORS)]

        first = fly(backend, name, variant, dynamic)          # nothing before
        fly(backend, *d1, dynamic)                            # predecessor A
        second = fly(backend, name, variant, dynamic)
        fly(backend, *d2, dynamic)                            # predecessor B
        third = fly(backend, name, variant, dynamic)

        entry = {
            "target": f"{name}_v{variant:02d}",
            "predecessor_a": f"{d1[0]}_v{d1[1]:02d}",
            "predecessor_b": f"{d2[0]}_v{d2[1]:02d}",
            "outcomes": [first["outcome"], second["outcome"], third["outcome"]],
            "steps": [first["steps"], second["steps"], third["steps"]],
            "first_vs_after_A": compare(first, second),
            "first_vs_after_B": compare(first, third),
            "after_A_vs_after_B": compare(second, third),
        }
        results.append(entry)
        worst = max(entry[k]["track_maxdiff_m"] for k in
                    ("first_vs_after_A", "first_vs_after_B", "after_A_vs_after_B"))
        agree = (len(set(entry["outcomes"])) == 1
                 and len(set(entry["steps"])) == 1)
        print(f"  {entry['target']:<18} outcomes={entry['outcomes']} "
              f"steps={entry['steps']} "
              f"{'AGREE' if agree else 'DIFFER'} worst track diff {worst:.3e} m",
              flush=True)

    deltas = [results[i][k]["track_maxdiff_m"] for i in range(len(results))
              for k in ("first_vs_after_A", "first_vs_after_B",
                        "after_A_vs_after_B")]
    summary = {
        "reset_mode": args.reset_mode,
        "layout": args.layout,
        "targets": len(TARGETS),
        "controllers": sorted({t[0] for t in TARGETS}),
        "comparisons": len(deltas),
        "all_outcomes_agree": all(len(set(r["outcomes"])) == 1 for r in results),
        "all_step_counts_agree": all(len(set(r["steps"])) == 1 for r in results),
        "track_maxdiff_m_worst": max(deltas),
        "track_maxdiff_m_median": float(np.median(deltas)),
        "results": results,
    }
    path = OUT / f"validation_{args.reset_mode}.json"
    path.write_text(json.dumps(summary, indent=2), encoding="utf8")
    print(f"\n{args.reset_mode}: {len(TARGETS)} targets x 3 runs, "
          f"{len(deltas)} pairwise comparisons")
    print(f"  outcomes agree    : {summary['all_outcomes_agree']}")
    print(f"  step counts agree : {summary['all_step_counts_agree']}")
    print(f"  worst track diff  : {summary['track_maxdiff_m_worst']:.3e} m")
    print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
