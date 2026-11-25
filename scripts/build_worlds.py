"""Write one Webots world per layout x mode from the canonical scenario JSONs.

The worlds are generated, never hand-edited, so the geometry in the ``.wbt``
cannot drift from the scenario that defines it. Each world's header carries the
scenario id and its SHA-256, and ``dns.worldcheck`` re-verifies at run time that
the loaded nodes actually sit where the scenario says -- the pair of checks that
closes the v1 defect where the cart was written at its motion centre instead of
its t = 0 pose.

Worlds are written into ``webots_dns/worlds/`` (where the relative PROTO path
resolves and the harness can launch them) and copied into the delivered package.

Run::

    python scripts/build_worlds.py
"""
from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dns import worldgen                                        # noqa: E402
from dns.scenario import Scenario, to_webots                    # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = PROJECT_ROOT / "config" / "scenarios"
WORLDS = PROJECT_ROOT / "worlds"
PACKAGE_WORLDS = SCENARIOS / "worlds"

LAYOUTS = ("L00", "L01", "L02", "L03", "L04", "L05")
MODES = ("C", "B")


def annotate(text: str, scenario_id: str, digest: str, source: str) -> str:
    """Stamp the scenario identity into the world header and WorldInfo.

    A world that cannot say which scenario produced it is unattributable, and
    v1 shipped a protocol file describing a different run than its own results
    CSV for exactly that reason.
    """
    marker = '    "Owner: Khaled (Simulation Environment)"'
    stamped = text.replace(marker, marker + f"""
    "Scenario id: {scenario_id}"
    "Scenario sha256: {digest}"
    "Scenario source: {source}\"""")
    return stamped.replace(
        "# GENERATED FILE. Edit config/",
        f"# Scenario id    : {scenario_id}\n"
        f"# Scenario sha256: {digest}\n"
        "# GENERATED FILE. Edit 07_SCENARIOS/")


def main() -> int:
    WORLDS.mkdir(parents=True, exist_ok=True)
    PACKAGE_WORLDS.mkdir(parents=True, exist_ok=True)
    written = []
    for layout in LAYOUTS:
        for mode in MODES:
            source = SCENARIOS / f"{layout}_{mode}.json"
            if not source.is_file():
                print(f"SKIP {source.name}: not generated")
                continue
            raw = json.loads(source.read_text(encoding="utf8"))
            digest = raw["provenance"]["scenario_sha256"]
            scenario = Scenario.load(source)

            name = f"DNS_{layout}_{mode}.wbt"
            text = annotate(worldgen.build(scenario), f"{layout}_{mode}",
                            digest, source.name)
            (WORLDS / name).write_text(text, encoding="utf8")
            shutil.copy2(WORLDS / name, PACKAGE_WORLDS / name)

            # Immediate read-back: every obstacle's t = 0 pose must appear in the
            # world text. This is the cart-pose regression test from v1, applied
            # to every body in every layout rather than to one world by hand.
            for obstacle in scenario.obstacles:
                centre = to_webots(obstacle.centre_at(0.0))
                token = f"DNS_OBSTACLE_{obstacle.id.upper()}"
                if token not in text:
                    raise SystemExit(f"{name}: missing node {token}")
                block = text.split(f"DEF {token} ")[1][:220]
                expected = " ".join(f"{v:.6g}" for v in
                                    (centre[0], centre[1], obstacle.height_m / 2))
                if expected not in block:
                    raise SystemExit(
                        f"{name}: {obstacle.id} t=0 pose {expected} not found in "
                        f"the written world (cart-pose regression)")
            written.append((name, digest, len(scenario.obstacles)))
            print(f"wrote worlds/{name}  sha256={digest[:16]}  "
                  f"obstacles={len(scenario.obstacles)}  t=0 poses OK")

    index = {"generated_worlds": [
        {"world": f"worlds/{name}", "scenario_sha256": digest,
         "n_obstacles": count} for name, digest, count in written]}
    (PACKAGE_WORLDS / "INDEX.json").write_text(json.dumps(index, indent=2),
                                               encoding="utf8")
    print(f"\n{len(written)} worlds written to {WORLDS} and copied to {PACKAGE_WORLDS}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
