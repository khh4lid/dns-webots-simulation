"""Reproduce this repository's figures, tables and (optionally) its experiments.

``--fast``  rebuilds every scenario, descriptor table, analysis and figure from
            the logs already in the repository. No Webots, a few minutes.
``--full``  additionally re-runs all 540 episodes in Webots. ~19 minutes plus
            boot time, and it needs Spencer's DroneNavPPO tree beside this one.

Each step prints PASS or FAIL and the whole run returns non-zero if any step
failed, so this is usable as a check rather than only as a convenience.

Run::

    python reproduce.py --fast
    python reproduce.py --full
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent

#: (label, argv, expected wall-clock note). Order matters: scenarios before the
#: descriptors that read them, results before the figures that plot them.
FAST = [
    ("regenerate the six layouts (byte-identical)",
     [sys.executable, "-m", "dns.generator", "--all"], "~1 min"),
    ("rebuild the Webots worlds from the scenarios",
     [sys.executable, "scripts/build_worlds.py"], "~5 s"),
    ("layout descriptors and the asset library",
     [sys.executable, "scripts/describe_layouts.py"], "~30 s"),
    ("generator and validator unit tests",
     [sys.executable, "tests/test_generator.py"], "~2 min"),
    ("v1 distinctness, L00 reproduction, pooled rates",
     [sys.executable, "scripts/analyse_v1.py"], "~20 s"),
    ("reactive controller analysis (C1)",
     [sys.executable, "scripts/analyse_reactive.py"], "~10 s"),
    ("LiDAR-cart consistency (C2)",
     [sys.executable, "scripts/analyse_lidar_cart.py"], "~2 min"),
    ("claims table",
     [sys.executable, "scripts/make_claims.py"], "~5 s"),
    ("core and extended figures",
     [sys.executable, "scripts/make_figures.py"], "~2 min"),
    ("computed visuals, ADR images and the shift-ladder GIF",
     [sys.executable, "scripts/make_visuals.py"], "~1 min"),
    ("decision records",
     [sys.executable, "scripts/make_adrs.py"], "~2 s"),
    ("team documents",
     [sys.executable, "scripts/make_team_docs.py"], "~2 s"),
    ("changelog, citation, folder READMEs",
     [sys.executable, "scripts/make_support_docs.py"], "~2 s"),
    ("security scan",
     [sys.executable, "scripts/security_scan.py"], "~2 min"),
]

FULL = [
    ("blocking layout gates (contracts + reference traversal)",
     [sys.executable, "scripts/run_gates.py"], "~2 min"),
    ("the 540-episode S6 experiment",
     [sys.executable, "scripts/run_s6.py"], "~19 min"),
]


def run(label: str, argv: list[str], note: str) -> tuple[bool, float]:
    started = time.time()
    print(f"\n>>> {label}  ({note})", flush=True)
    done = subprocess.run(argv, cwd=str(ROOT))
    elapsed = time.time() - started
    ok = done.returncode == 0
    print(f"    {'PASS' if ok else 'FAIL'}  {elapsed:.1f} s", flush=True)
    return ok, elapsed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--fast", action="store_true",
                       help="rebuild everything from the logs; no Webots")
    group.add_argument("--full", action="store_true",
                       help="also re-run the simulations in Webots")
    args = parser.parse_args()

    steps = (FULL + FAST) if args.full else FAST
    results = []
    total = time.time()
    for label, argv, note in steps:
        results.append((label, *run(label, argv, note)))

    print("\n" + "=" * 70)
    failed = [r for r in results if not r[1]]
    for label, ok, elapsed in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {elapsed:6.1f} s  {label}")
    print(f"\n{len(results) - len(failed)}/{len(results)} steps passed in "
          f"{time.time() - total:.0f} s")
    if failed:
        print("\nFailed steps:")
        for label, _ok, _elapsed in failed:
            print(f"  - {label}")
        print("\nIf a Webots step failed, check WEBOTS_HOME and that Spencer's "
              "DroneNavPPO tree is beside this repository (or DNS_PPO_DIR is set).")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
