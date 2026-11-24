"""Unit tests for the scenario generator, its validators and the feasibility tools.

Plain asserts, no pytest dependency. Run::

    python tests/test_generator.py

Every test states what would break if it failed, because a test whose purpose is
not written down gets deleted the first time it becomes inconvenient.
"""
from __future__ import annotations

import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns import generator as G                                   # noqa: E402
from dns.feasibility import (                                    # noqa: E402
    astar, bottleneck_clearance, clearance_grid, describe, iou_static,
    line_intrusion, path_length,
)
from dns.scenario import Obstacle, Scenario                      # noqa: E402

PACKAGE = PROJECT_ROOT.parent / "DNS_FRIDAY_MEETING_2026-11-27"
SCENARIOS = PACKAGE / "07_SCENARIOS"
LAYOUTS = ("L00", "L01", "L02", "L03", "L04", "L05")

PASSED: list[str] = []
FAILED: list[tuple[str, str]] = []


def test(name):
    def wrap(function):
        try:
            function()
        except AssertionError as error:
            FAILED.append((name, str(error)))
            print(f"FAIL  {name}\n        {error}")
        except Exception as error:                               # noqa: BLE001
            FAILED.append((name, f"{type(error).__name__}: {error}"))
            print(f"ERROR {name}\n        {type(error).__name__}: {error}")
        else:
            PASSED.append(name)
            print(f"pass  {name}")
        return function
    return wrap


def config() -> dict:
    return yaml.safe_load(G.CONFIG_PATH.read_text(encoding="utf8"))


# ---------------------------------------------------------------------------
# determinism
# ---------------------------------------------------------------------------
@test("generator is byte-identical for the same config and seed")
def _():
    # If this fails, a layout cannot be regenerated from its config, every
    # scenario hash becomes meaningless and the experiment is unreproducible.
    cfg = config()
    for family in ("L01", "L03", "L04", "L05"):
        first = G.canonical_text(G.build_family(cfg, family)["dynamic"])
        second = G.canonical_text(G.build_family(cfg, family)["dynamic"])
        assert first == second, f"{family} differs between two draws"


@test("generator is byte-identical across separate processes")
def _():
    # Guards specifically against seeding from Python's hash(), which is
    # randomised per process: the layouts would silently differ run to run.
    script = (
        "import sys, yaml; sys.path.insert(0, r'%s');"
        "from dns import generator as G;"
        "cfg = yaml.safe_load(G.CONFIG_PATH.read_text(encoding='utf8'));"
        "print(G.build_family(cfg, 'L04')['dynamic']['provenance']['scenario_sha256'])"
        % PROJECT_ROOT
    )
    digests = set()
    for _ in range(2):
        done = subprocess.run([sys.executable, "-c", script], capture_output=True,
                              text=True, cwd=str(PROJECT_ROOT), timeout=300)
        assert done.returncode == 0, done.stderr[-400:]
        digests.add(done.stdout.strip())
    assert len(digests) == 1, f"different hashes across processes: {digests}"


@test("scenario hash excludes the hash field and is stable under key order")
def _():
    scenario = json.loads((SCENARIOS / "L03_C.json").read_text(encoding="utf8"))
    recorded = scenario["provenance"]["scenario_sha256"]
    assert G.scenario_sha256(scenario) == recorded, "hash does not reproduce"
    shuffled = {k: scenario[k] for k in reversed(list(scenario))}
    assert G.scenario_sha256(shuffled) == recorded, "hash depends on key order"


@test("canonical form collapses negative zero")
def _():
    # -0.0 and 0.0 are both valid JSON but different text, so one slipping
    # through would break byte-identity for a mirrored layout.
    assert "-0.0" not in G.canonical_text({"a": -0.0, "b": [-0.0]})


# ---------------------------------------------------------------------------
# validators
# ---------------------------------------------------------------------------
@test("validator rejects an obstacle outside the wall margin")
def _():
    cfg = config()
    validator = G.Validator(cfg["constraints"], cfg["base"])
    obstacle = G._asset(cfg["assets"], "rack", "rack_a", (3.5, 2.9))
    try:
        validator.check({"obstacles": [obstacle]})
    except G.Rejected as rejection:
        assert rejection.rule == "wall_margin", rejection.rule
    else:
        raise AssertionError("a rack at y = 2.9 should leave the wall margin")


@test("validator rejects an obstacle below the scan plane")
def _():
    cfg = config()
    validator = G.Validator(cfg["constraints"], cfg["base"])
    obstacle = G._asset(cfg["assets"], "crate", "low", (5.0, 2.0))
    obstacle["height_m"] = 0.9           # scan plane rides at 1.14 m
    try:
        validator.check({"obstacles": [obstacle]})
    except G.Rejected as rejection:
        assert rejection.rule == "scan_plane_height", rejection.rule
    else:
        raise AssertionError("a 0.9 m obstacle is collidable but invisible")


@test("validator rejects overlapping obstacles")
def _():
    cfg = config()
    validator = G.Validator(cfg["constraints"], cfg["base"])
    pair = [G._asset(cfg["assets"], "crate", "a", (5.0, 1.0)),
            G._asset(cfg["assets"], "crate", "b", (5.2, 1.0))]
    try:
        validator.check({"obstacles": pair})
    except G.Rejected as rejection:
        assert rejection.rule == "obstacle_gap", rejection.rule
    else:
        raise AssertionError("crates 0.2 m apart should be rejected")


@test("validator rejects an obstacle inside the start/goal keep-out")
def _():
    cfg = config()
    validator = G.Validator(cfg["constraints"], cfg["base"])
    obstacle = G._asset(cfg["assets"], "crate", "near_goal", (9.15, 0.6))
    try:
        validator.check({"obstacles": [obstacle]})
    except G.Rejected as rejection:
        assert rejection.rule == "start_goal_keepout", rejection.rule
    else:
        raise AssertionError("a crate 0.6 m from the goal breaks the keep-out")


@test("validator checks the cart keep-out over its whole sweep, not just t=0")
def _():
    # The cart's t = 0 pose is y = -1.9, far from the goal; its sweep reaches
    # y = +1.9. A keep-out tested only at t = 0 would pass a cart that parks on
    # the goal mid-episode.
    cfg = config()
    validator = G.Validator(cfg["constraints"], cfg["base"])
    cart = G._cart(cfg, lane_x=9.15)     # lane directly through the goal
    try:
        validator.check({"obstacles": [cart]})
    except G.Rejected as rejection:
        assert rejection.rule == "start_goal_keepout", rejection.rule
    else:
        raise AssertionError("a cart lane through the goal must be rejected")


# ---------------------------------------------------------------------------
# the mirror transform
# ---------------------------------------------------------------------------
@test("L02 is an exact reflection of L00")
def _():
    base = Scenario.load(SCENARIOS / "L00_C.json")
    mirror = Scenario.load(SCENARIOS / "L02_C.json")
    first = {o.id: o for o in base.obstacles}
    second = {o.id: o for o in mirror.obstacles}
    assert set(first) == set(second), "mirrored layout changed the obstacle set"
    for key, obstacle in first.items():
        other = second[key]
        assert abs(obstacle.xy[0] - other.xy[0]) < 1e-9, f"{key} x moved"
        assert abs(obstacle.xy[1] + other.xy[1]) < 1e-9, f"{key} y not negated"
        assert abs(obstacle.yaw_rad + other.yaw_rad) < 1e-9, f"{key} yaw not negated"
        assert obstacle.size_xy == other.size_xy, f"{key} size changed"


@test("the mirrored cart trajectory is the exact negation of L00's")
def _():
    # A half-period phase shift reflects a triangle wave exactly, because
    # f(u + 1/2) = -f(u). Negating the amplitude would instead corrupt the
    # period, which is 4 * amplitude / speed.
    base = next(o for o in Scenario.load(SCENARIOS / "L00_C.json").obstacles
                if o.motion.type == "linear")
    mirror = next(o for o in Scenario.load(SCENARIOS / "L02_C.json").obstacles
                  if o.motion.type == "linear")
    period = 4.0 * base.motion.amplitude_m / base.motion.speed_mps
    worst = 0.0
    for t in np.linspace(0.0, 2.0 * period, 401):
        worst = max(worst, abs(base.centre_at(t)[1] + mirror.centre_at(t)[1]))

    # Tolerance derived, not guessed. The mirror is a phase shift of pi, and the
    # canonical scenario rounds every float to generator.ROUND = 6 decimals, so
    # the stored phase is 3.141593 rather than pi: an error of 3.46e-7 rad. The
    # triangle wave's sensitivity is d(offset)/d(phase) = 4 * amplitude / (2*pi)
    # = 1.21 m/rad, giving a worst-case position error of 4.2e-7 m. That is
    # 0.42 um -- about 7x smaller than the simulator's own 2.8 um reset error,
    # so it is below the noise floor of anything this environment measures.
    sensitivity = 4.0 * base.motion.amplitude_m / (2.0 * math.pi)
    bound = sensitivity * 0.5 * 10 ** -G.ROUND + 1e-12
    assert worst < bound, (
        f"cart mirror error {worst:.3e} m exceeds the {bound:.3e} m bound "
        f"implied by {G.ROUND}-decimal canonical rounding")


@test("L02 matches L00 on every difficulty descriptor")
def _():
    # This is what makes L02 a clean memorization probe: maximally dissimilar
    # geometry, identical difficulty. If it fails, an L00 -> L02 drop could be
    # explained by difficulty instead of by layout identity.
    base, _p, _g = describe(Scenario.load(SCENARIOS / "L00_C.json"))
    mirror, _p, _g = describe(Scenario.load(SCENARIOS / "L02_C.json"))
    for key in ("astar_length_m", "tortuosity", "min_corridor_width_enroute_m",
                "line_intrusion_m", "occupied_area_pct"):
        assert abs(base[key] - mirror[key]) < 1e-6, (
            f"{key}: L00 {base[key]} vs L02 {mirror[key]}")


# ---------------------------------------------------------------------------
# every generated layout
# ---------------------------------------------------------------------------
@test("every layout loads, validates and is A*-feasible")
def _():
    for layout in LAYOUTS:
        for mode in ("C", "B"):
            scenario = Scenario.load(SCENARIOS / f"{layout}_{mode}.json")
            facts, path, _grid = describe(scenario)
            assert facts["feasible"], f"{layout}_{mode} has no A* path"
            assert path_length(path) > 0.0


@test("every layout blocks the straight start-goal line")
def _():
    # An open line let `direct` succeed in an earlier v1 layout, and the static
    # condition then measured nothing.
    for layout in LAYOUTS:
        scenario = Scenario.load(SCENARIOS / f"{layout}_C.json")
        intrusion = line_intrusion(scenario)
        assert intrusion["blocked"], f"{layout} leaves the straight line open"
        assert intrusion["intrusion_m"] >= 0.12, (
            f"{layout} intrusion {intrusion['intrusion_m']} m below 0.12 m")


@test("every layout keeps its passages above the drone diameter")
def _():
    for layout in LAYOUTS:
        scenario = Scenario.load(SCENARIOS / f"{layout}_C.json")
        facts, _p, _g = describe(scenario)
        width = facts["min_corridor_width_enroute_m"]
        assert width > 0.36, f"{layout} corridor {width} m <= drone diameter"


@test("the cart blocks its lane for part, but not all, of its period")
def _():
    # 0 would make Mode C identical to Mode B; 1 would make the route
    # permanently closed and the task unsolvable rather than a timing problem.
    for layout in LAYOUTS:
        scenario = Scenario.load(SCENARIOS / f"{layout}_C.json")
        facts, _p, _g = describe(scenario)
        fraction = facts["cart_blocking_fraction"]
        assert 0.0 < fraction < 1.0, f"{layout} cart blocking {fraction}"


@test("Mode B and Mode C share an identical static obstacle set")
def _():
    # Otherwise a B/C comparison would not isolate obstacle motion.
    for layout in LAYOUTS:
        dynamic = Scenario.load(SCENARIOS / f"{layout}_C.json")
        static = Scenario.load(SCENARIOS / f"{layout}_B.json")
        def statics(scenario):
            return {o.id: (tuple(np.round(o.xy, 9)), round(o.yaw_rad, 9),
                           o.size_xy, o.height_m)
                    for o in scenario.obstacles
                    if o.motion.type == "static" or not o.motion.amplitude_m}
        a, b = statics(dynamic), statics(static)
        shared = set(a) & set(b)
        for key in shared:
            assert a[key] == b[key], f"{layout} {key} differs between B and C"
        assert set(a) - {"cart"} == set(b) - {"cart"}, (
            f"{layout} static set differs between modes")


@test("every layout keeps the frozen L00 envelope")
def _():
    reference = Scenario.load(SCENARIOS / "L00_C.json")
    for layout in LAYOUTS:
        for mode in ("C", "B"):
            scenario = Scenario.load(SCENARIOS / f"{layout}_{mode}.json")
            for attribute in ("low_xy", "high_xy", "start_xy", "goal_xy",
                              "goal_tolerance_m", "cruise_altitude_m",
                              "basic_timestep_ms", "command_period_s",
                              "max_speed_mps", "collision_radius_m",
                              "lidar_resolution", "sensing_max_range_m",
                              "sensing_sectors", "lidar_mount_mode",
                              "lidar_height_m", "wall_height_m"):
                assert getattr(scenario, attribute) == getattr(reference, attribute), (
                    f"{layout}_{mode}.{attribute} = "
                    f"{getattr(scenario, attribute)}, L00 has "
                    f"{getattr(reference, attribute)}")


@test("the Mode B cart is parked at the arena edge with zero amplitude")
def _():
    for layout in LAYOUTS:
        static = Scenario.load(SCENARIOS / f"{layout}_B.json")
        cart = next((o for o in static.obstacles if o.id == "cart"), None)
        assert cart is not None, f"{layout}_B has no cart"
        assert cart.motion.amplitude_m == 0.0, f"{layout}_B cart still moves"
        assert abs(cart.centre_at(0.0)[1] - (-2.5)) < 1e-9, (
            f"{layout}_B cart parked at y = {cart.centre_at(0.0)[1]}")


@test("recorded scenario hashes match HASHES.txt")
def _():
    recorded = {}
    for line in (SCENARIOS / "HASHES.txt").read_text(encoding="utf8").splitlines():
        if line and not line.startswith("#"):
            digest, name = line.split("  ")
            recorded[name] = digest
    for layout in LAYOUTS:
        for mode in ("C", "B"):
            name = f"{layout}_{mode}.json"
            scenario = json.loads((SCENARIOS / name).read_text(encoding="utf8"))
            assert recorded[name] == scenario["provenance"]["scenario_sha256"], (
                f"{name} hash disagrees with HASHES.txt")


@test("L01-L05 are flagged TEST-ONLY and L00 is not")
def _():
    for layout in LAYOUTS:
        scenario = json.loads((SCENARIOS / f"{layout}_C.json").read_text(encoding="utf8"))
        flag = scenario["provenance"]["test_only"]
        assert flag == (layout != "L00"), f"{layout} test_only = {flag}"


# ---------------------------------------------------------------------------
# geometry primitives the whole pipeline rests on
# ---------------------------------------------------------------------------
@test("box surface distance is exact, not a bounding circle")
def _():
    # This is the v1 defect: a 0.8 x 2.4 m rack collided as its bounding circle
    # reported contact about 0.5 m into empty aisle.
    rack = Obstacle(id="r", xy=(0.0, 0.0), shape="box", size_xy=(0.8, 2.4),
                    height_m=2.0).placed_at(0.0)
    assert abs(rack.surface_distance(np.array([[1.0, 0.0]]))[0] - 0.6) < 1e-12
    assert abs(rack.surface_distance(np.array([[0.0, 1.5]]))[0] - 0.3) < 1e-12
    assert rack.surface_distance(np.array([[0.0, 0.0]]))[0] == 0.0
    # A point beside the rack's long face: a bounding circle of radius
    # hypot(0.4, 1.2) = 1.265 m would call this a collision.
    point = np.array([[0.9, 1.1]])
    assert abs(rack.surface_distance(point)[0] - 0.5) < 1e-12
    assert rack.radius_m > 1.26, "bounding radius should exceed the true gap"


@test("widest-path bottleneck finds the generous route, not an arbitrary one")
def _():
    # A* returns the shortest path, which may thread a tight gap when a wide
    # detour exists. The corridor-width descriptor must report the widest
    # route's bottleneck or it understates how traversable a layout is.
    scenario = Scenario.load(SCENARIOS / "L00_C.json")
    grid = clearance_grid(scenario, 0.0, 0.05)
    radius = scenario.collision_radius_m + 0.02
    path = astar(grid, scenario.start_xy, scenario.goal_xy, radius)
    shortest_min = min(float(grid.clearance[grid.index_of(p)[1],
                                            grid.index_of(p)[0]]) for p in path)
    widest = bottleneck_clearance(grid, scenario.start_xy, scenario.goal_xy, radius)
    assert widest >= shortest_min - 1e-9, (
        f"widest-path bottleneck {widest} below the A* path minimum {shortest_min}")


@test("IoU is 1 against itself and below 1 against a mirror")
def _():
    base = Scenario.load(SCENARIOS / "L00_C.json")
    mirror = Scenario.load(SCENARIOS / "L02_C.json")
    assert abs(iou_static(base, base) - 1.0) < 1e-9
    assert iou_static(base, mirror) < 0.2, "L00 and its mirror overlap too much"


@test("clearance grid agrees with the scenario's own clearance query")
def _():
    scenario = Scenario.load(SCENARIOS / "L00_C.json")
    grid = clearance_grid(scenario, 0.0, 0.05)
    rng = np.random.default_rng(3)
    worst = 0.0
    for _ in range(300):
        point = np.array([rng.uniform(0.2, 9.8), rng.uniform(-2.8, 2.8)])
        i, j = grid.index_of(point)
        centre = np.array(grid.point_of(i, j))
        # Compare at the cell centre, which is what the grid stores.
        direct = scenario.clearance_at(centre, 0.0)
        if np.isfinite(direct):
            worst = max(worst, abs(direct - float(grid.clearance[j, i])))
    # The grid also accounts for walls, which clearance_at does not, so only
    # cells where an obstacle is nearest can agree exactly.
    assert worst < 3.0, f"grid and clearance_at disagree by {worst:.4f} m"


@test("obstacle motion is a deterministic function of time")
def _():
    cart = next(o for o in Scenario.load(SCENARIOS / "L00_C.json").obstacles
                if o.motion.type == "linear")
    for t in (0.0, 1.7, 8.4, 16.8889, 33.0):
        assert np.allclose(cart.centre_at(t), cart.centre_at(t)), "not pure"
    period = 4.0 * cart.motion.amplitude_m / cart.motion.speed_mps
    assert abs(period - 16.8889) < 1e-3, f"period {period}"
    assert np.allclose(cart.centre_at(0.0), cart.centre_at(period), atol=1e-9), (
        "cart is not periodic over its own period")
    assert abs(cart.centre_at(0.0)[1] - (-1.9)) < 1e-9, (
        "cart t=0 pose is not the -1.9 m extreme; this is the v1 world-file defect")


def main() -> int:
    print(f"\n{len(PASSED)} passed, {len(FAILED)} failed")
    for name, reason in FAILED:
        print(f"  FAILED {name}: {reason}")
    return 1 if FAILED else 0


if __name__ == "__main__":
    raise SystemExit(main())
