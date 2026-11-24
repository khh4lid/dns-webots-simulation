"""S6 scenario generator: ``generator_config.yaml`` + seed -> canonical scenario JSON.

The point of this module is that a layout is *data*, not a hand-edited world
file. One config and one seed reproduce a layout byte for byte, its SHA-256 is
recorded, and every consumer -- the world writer, the analytic ray caster, the
feasibility checker, the figures -- reads the same JSON. That closes the v1
defect class where the calibration tool measured one world and compared it with
another world's geometry.

Pipeline::

    generator_config.yaml + family + seed
        -> draw candidate obstacle set           (family-specific sampler)
        -> hard validators                       (reject and redraw on failure)
        -> feasibility + descriptors             (A*, bottleneck width, IoU)
        -> family acceptance targets             (reject and redraw on failure)
        -> canonical JSON + SHA-256
        -> worlds/DNS_<id>.wbt via dns.worldgen

Only obstacle PLACEMENT is ever sampled. Arena bounds, start, goal, tolerance,
cruise altitude, basicTimeStep, command period, speed cap, sensing and collision
mode are copied verbatim from L00 and are frozen by the Section 3 contracts, so
layout is the single independent variable across L00-L05.

Run::

    python -m dns.generator --all          # write L00..L05 + hashes + table
    python -m dns.generator --family L03    # one family
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .feasibility import describe
from .scenario import SCENARIO_SCHEMA, Scenario

PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = PROJECT_ROOT.parent
PACKAGE = WORKSPACE / "DNS_FRIDAY_MEETING_2026-11-27"
CONFIG_PATH = PACKAGE / "07_SCENARIOS" / "generator_config.yaml"
SCENARIO_DIR = PACKAGE / "07_SCENARIOS"

#: Decimal places every float in a canonical scenario is rounded to. Six is
#: well past the simulator's own resolution (the reset error is 2.8e-6 m) and it
#: removes the float-repr drift that would otherwise break byte-identity
#: between a freshly sampled layout and a round-tripped one.
ROUND = 6


# ---------------------------------------------------------------------------
# canonical form and hashing
# ---------------------------------------------------------------------------
def _round_floats(value):
    """Recursively round every float so the JSON text is stable."""
    if isinstance(value, dict):
        return {k: _round_floats(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_round_floats(v) for v in value]
    if isinstance(value, bool):
        return value
    if isinstance(value, (float, np.floating)):
        rounded = round(float(value), ROUND)
        # Collapse -0.0 to 0.0: both are valid JSON but they are different text.
        return 0.0 if rounded == 0.0 else rounded
    if isinstance(value, (int, np.integer)):
        return int(value)
    return value


def canonical_text(scenario: dict) -> str:
    """The exact bytes written to disk for a scenario. Deterministic."""
    return json.dumps(_round_floats(scenario), indent=2, sort_keys=True,
                      ensure_ascii=True) + "\n"


def scenario_sha256(scenario: dict) -> str:
    """Hash of the scenario with its own hash field removed.

    Every run asserts the loaded world's scenario hash against this value, so a
    world file that has drifted from its definition fails loudly instead of
    quietly producing results attributed to the wrong geometry.
    """
    stripped = copy.deepcopy(scenario)
    stripped.get("provenance", {}).pop("scenario_sha256", None)
    payload = json.dumps(_round_floats(stripped), sort_keys=True,
                         separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(payload.encode("utf8")).hexdigest()


def config_sha256(path: Path = CONFIG_PATH) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


# ---------------------------------------------------------------------------
# validators
# ---------------------------------------------------------------------------
class Rejected(Exception):
    """A candidate layout violated a hard constraint. Carries the rule name."""

    def __init__(self, rule: str, detail: str = ""):
        super().__init__(f"{rule}: {detail}" if detail else rule)
        self.rule = rule


@dataclass
class Validator:
    """The hard constraints every layout must satisfy before it is accepted.

    Each rule raises :class:`Rejected` with its own name so the generator can
    report *which* constraint was binding, and how often. A family whose
    rejection count is dominated by one rule is a badly specified family, and
    the counts are written to 06_DATA/generator_report.json for exactly that
    reason.
    """
    constraints: dict
    base: dict

    def check(self, scenario: dict) -> None:
        obstacles = scenario["obstacles"]
        low = np.asarray(self.base["arena"]["low_xy"], dtype=float)
        high = np.asarray(self.base["arena"]["high_xy"], dtype=float)
        margin = self.constraints["wall_margin_m"]
        start = np.asarray(self.base["drone"]["start_xy"], dtype=float)
        goal = np.asarray(self.base["goal"]["xy"], dtype=float)
        keepout = self.constraints["start_goal_keepout_m"]
        scan_plane = (self.base["drone"]["cruise_altitude_m"]
                      + self.base["sensing"]["mount_height_m"])
        scan_margin = self.constraints["scan_plane_margin_m"]

        placed = [_placed(o) for o in obstacles]

        # 1. inside the arena, with wall margin on every obstacle surface
        for obstacle, body in zip(obstacles, placed):
            corners = _corners(body)
            if (corners[:, 0].min() < low[0] + margin
                    or corners[:, 0].max() > high[0] - margin
                    or corners[:, 1].min() < low[1] + margin
                    or corners[:, 1].max() > high[1] - margin):
                raise Rejected("wall_margin", f"{obstacle['id']} leaves the "
                                              f"{margin} m wall margin")

        # 2. every obstacle must rise above the scan plane, or it would be
        #    solid to the collision test and invisible to the lidar
        for obstacle in obstacles:
            if obstacle["height_m"] <= scan_plane + scan_margin:
                raise Rejected("scan_plane_height",
                               f"{obstacle['id']} is {obstacle['height_m']} m, "
                               f"scan plane at {scan_plane:.3f} m")

        # 3. no two static obstacles closer than the minimum gap, evaluated
        #    over the moving bodies' whole sweep
        gap_min = self.constraints["min_obstacle_gap_m"]
        for i, (obs_a, body_a) in enumerate(zip(obstacles, placed)):
            for obs_b, body_b in list(zip(obstacles, placed))[i + 1:]:
                moving = _is_moving(obs_a) or _is_moving(obs_b)
                worst = _pair_gap(obs_a, obs_b)
                limit = self.constraints["cart_lane_clear_m"] if moving else gap_min
                if worst < limit:
                    raise Rejected(
                        "obstacle_gap" if not moving else "cart_lane_clear",
                        f"{obs_a['id']}/{obs_b['id']} gap {worst:.3f} m < {limit} m")

        # 4. start and goal keep-out, over the whole sweep of any mover
        for obstacle in obstacles:
            for label, point in (("start", start), ("goal", goal)):
                gap = _point_gap(obstacle, point)
                if gap < keepout:
                    raise Rejected("start_goal_keepout",
                                   f"{obstacle['id']} is {gap:.3f} m from "
                                   f"{label} (< {keepout} m)")


def _is_moving(obstacle: dict) -> bool:
    motion = obstacle.get("motion") or {}
    return motion.get("type", "static") != "static" and motion.get("amplitude_m", 0.0)


def _placed(obstacle: dict, t: float = 0.0):
    """A :class:`~dns.scenario.PlacedObstacle` for a raw scenario dict entry."""
    from .scenario import Obstacle
    return Obstacle.parse(obstacle).placed_at(t)


def _corners(body) -> np.ndarray:
    half = np.asarray(body.half_size, dtype=float)
    if body.shape == "cylinder":
        r = body.radius_m
        half = np.array([r, r])
    local = np.array([[-half[0], -half[1]], [half[0], -half[1]],
                      [half[0], half[1]], [-half[0], half[1]]])
    c, s = math.cos(body.yaw_rad), math.sin(body.yaw_rad)
    return local @ np.array([[c, s], [-s, c]]) + np.asarray(body.centre)


def _sweep_times(obstacle: dict, count: int = 49) -> np.ndarray:
    motion = obstacle.get("motion") or {}
    amplitude = motion.get("amplitude_m", 0.0)
    if motion.get("type", "static") == "static" or not amplitude:
        return np.zeros(1)
    speed = motion.get("speed_mps", 0.0)
    period = (4.0 * amplitude / speed) if speed else motion.get("period_s", 4.0)
    return np.linspace(0.0, period, count)


def _pair_gap(obs_a: dict, obs_b: dict) -> float:
    """Worst surface-to-surface gap between two obstacles over their motion."""
    worst = math.inf
    times = np.unique(np.concatenate((_sweep_times(obs_a), _sweep_times(obs_b))))
    for t in times:
        body_a, body_b = _placed(obs_a, float(t)), _placed(obs_b, float(t))
        # Surface-to-surface distance between two convex footprints, measured as
        # the minimum over each body's sampled boundary against the other body.
        gap = min(float(body_b.surface_distance(_boundary(body_a)).min()),
                  float(body_a.surface_distance(_boundary(body_b)).min()))
        worst = min(worst, gap)
    return worst


def _boundary(body, per_edge: int = 12) -> np.ndarray:
    """Points sampled around a footprint's outline."""
    corners = _corners(body)
    segments = []
    for i in range(len(corners)):
        a, b = corners[i], corners[(i + 1) % len(corners)]
        fractions = np.linspace(0.0, 1.0, per_edge, endpoint=False)[:, None]
        segments.append(a[None, :] + fractions * (b - a)[None, :])
    return np.vstack(segments)


def _point_gap(obstacle: dict, point: np.ndarray) -> float:
    """Worst distance from a point to an obstacle's surface over its motion."""
    worst = math.inf
    for t in _sweep_times(obstacle):
        body = _placed(obstacle, float(t))
        worst = min(worst, float(body.surface_distance(point[None, :])[0]))
    return worst


# ---------------------------------------------------------------------------
# asset helpers
# ---------------------------------------------------------------------------
def _asset(assets: dict, kind: str, oid: str, xy, yaw: float = 0.0,
           motion: dict | None = None) -> dict:
    spec = assets[kind]
    obstacle = {
        "id": oid,
        "label": spec["label"],
        "purpose": " ".join(spec["purpose"].split()),
        "shape": spec["shape"],
        "xy": [float(xy[0]), float(xy[1])],
        "size_xy": [float(spec["size_xy"][0]), float(spec["size_xy"][1])],
        "height_m": float(spec["height_m"]),
        "colour": [float(c) for c in spec["colour"]],
        "asset": kind,
    }
    if abs(yaw) > 1e-12:
        obstacle["yaw_rad"] = float(yaw)
    if motion:
        obstacle["motion"] = dict(motion)
    return obstacle


def _cart(config: dict, lane_x: float, phase_rad: float = 0.0,
          base_y: float = 0.0) -> dict:
    motion = config["cart_motion"]
    return _asset(config["assets"], "cart", "cart", (lane_x, base_y),
                  motion={"type": motion["type"], "axis": motion["axis"],
                          "amplitude_m": float(motion["amplitude_m"]),
                          "speed_mps": float(motion["speed_mps"]),
                          "phase_rad": float(phase_rad)})


def _parked_cart(config: dict, lane_x: float) -> dict:
    """The Mode B cart: parked at the arena edge, amplitude zero.

    Reproduces L00's ``scenario_warehouse_static.json`` exactly -- same lane x,
    same park y, same ``oscillate`` type with zero amplitude -- so Mode B means
    the same thing on every layout as it did in v1.
    """
    motion = config["cart_motion"]
    return _asset(config["assets"], "cart", "cart",
                  (lane_x, float(motion["park_y_m"])),
                  motion={"type": "oscillate", "axis": motion["axis"],
                          "amplitude_m": 0.0, "period_s": 4.0, "phase_rad": 0.0})


# ---------------------------------------------------------------------------
# family samplers
# ---------------------------------------------------------------------------
def _l00_obstacles(config: dict) -> list[dict]:
    """L00's obstacle set, read from the frozen training config."""
    source = PROJECT_ROOT / config["families"]["L00"]["source"]
    raw = json.loads(source.read_text(encoding="utf-8-sig"))
    assets = config["assets"]
    kind_of = {"rack_a": "rack", "rack_b": "rack", "column_1": "column",
               "column_2": "column", "crate_stack": "crate", "cart": "cart"}
    obstacles = []
    for entry in raw["obstacles"]:
        kind = kind_of[entry["id"]]
        obstacle = _asset(assets, kind, entry["id"], entry["xy"],
                          float(entry.get("yaw_rad", 0.0)),
                          entry.get("motion"))
        # L00 is the reference: keep ITS geometry, not the asset library's, so
        # that any divergence between the two is visible rather than silently
        # overwritten. (They agree; this asserts it.)
        for key in ("size_xy", "height_m"):
            if obstacle[key] != entry.get(key, obstacle[key]):
                raise ValueError(
                    f"asset library {kind}.{key} = {obstacle[key]} disagrees "
                    f"with L00 {entry['id']}.{key} = {entry[key]}")
        obstacles.append(obstacle)
    return obstacles


def _stable_seed(*parts) -> list[int]:
    """A reproducible RNG seed sequence from arbitrary labels.

    ``hash()`` is NOT usable here: Python randomises string hashing per process
    unless PYTHONHASHSEED is pinned, so a seed derived from it would produce a
    different layout on every run and destroy the byte-identity guarantee this
    whole module exists to provide. SHA-256 is stable across processes,
    interpreter versions and machines.
    """
    digest = hashlib.sha256("|".join(str(p) for p in parts).encode("utf8"))
    return [int.from_bytes(digest.digest()[i:i + 4], "big") for i in range(0, 16, 4)]


def _sample_free_point(rng, config: dict, existing: list[dict], *,
                       kind: str, x_range, y_range, yaw: float = 0.0,
                       keepout_extra: float = 0.0,
                       attempts: int = 600) -> tuple[float, float]:
    """Draw a point whose asset clears the walls, start, goal and everything placed.

    Rejecting locally rather than letting the top-level validator throw away the
    whole draw is what keeps the acceptance rate workable: with five extra
    crates, a single bad placement would otherwise discard four good ones.

    The wall test is applied here, at the candidate's own yaw, because a square
    crate rotated 45 deg reaches 0.35*sqrt(2) = 0.495 m from its centre rather
    than 0.35 m. Sampling against the un-rotated half-size sent roughly half of
    all L04 draws to the top-level `wall_margin` rule, each one discarding the
    whole layout.
    """
    constraints = config["constraints"]
    base = config["base"]
    low = np.asarray(base["arena"]["low_xy"], dtype=float)
    high = np.asarray(base["arena"]["high_xy"], dtype=float)
    margin = constraints["wall_margin_m"]
    start = np.asarray(base["drone"]["start_xy"], dtype=float)
    goal = np.asarray(base["goal"]["xy"], dtype=float)
    keepout = constraints["start_goal_keepout_m"] + keepout_extra

    # Largest distance from the footprint centre to any corner at this yaw.
    probe = _asset(config["assets"], kind, "_probe", (0.0, 0.0), yaw)
    reach = np.abs(_corners(_placed(probe))).max(axis=0)
    x_lo = max(x_range[0], low[0] + margin + reach[0])
    x_hi = min(x_range[1], high[0] - margin - reach[0])
    y_lo = max(y_range[0], low[1] + margin + reach[1])
    y_hi = min(y_range[1], high[1] - margin - reach[1])
    if x_lo >= x_hi or y_lo >= y_hi:
        raise Rejected("free_point", f"{kind} cannot fit the requested range")

    for _ in range(attempts):
        x = float(rng.uniform(x_lo, x_hi))
        y = float(rng.uniform(y_lo, y_hi))
        candidate = _asset(config["assets"], kind, "_probe", (x, y), yaw)
        if min(_point_gap(candidate, start), _point_gap(candidate, goal)) < keepout:
            continue
        limits = [constraints["cart_lane_clear_m"] if _is_moving(other)
                  else constraints["min_obstacle_gap_m"] for other in existing]
        if any(_pair_gap(candidate, other) < limit
               for other, limit in zip(existing, limits)):
            continue
        return x, y
    raise Rejected("free_point", f"no free {kind} placement in range")


def _family_l01(rng, config: dict) -> list[dict]:
    rules = config["families"]["L01"]
    jitter = rules["position_jitter_m"]
    yaw_jitter = math.radians(rules["yaw_jitter_deg"])
    obstacles = []
    for obstacle in _l00_obstacles(config):
        moved = dict(obstacle)
        if _is_moving(obstacle):
            # The cart's amplitude, speed, period and sweep centre are
            # contract-frozen (Section 2 forbids changing its dynamics), so only
            # its LANE POSITION x is jittered. Recorded in DECISIONS.md.
            moved["xy"] = [obstacle["xy"][0] + float(rng.uniform(-jitter, jitter)),
                           obstacle["xy"][1]]
        else:
            moved["xy"] = [obstacle["xy"][0] + float(rng.uniform(-jitter, jitter)),
                           obstacle["xy"][1] + float(rng.uniform(-jitter, jitter))]
            yaw = obstacle.get("yaw_rad", 0.0) + float(
                rng.uniform(-yaw_jitter, yaw_jitter))
            if abs(yaw) > 1e-12:
                moved["yaw_rad"] = yaw
        obstacles.append(moved)
    return obstacles


def _family_l02(config: dict) -> list[dict]:
    """L00 reflected across y = 0. Deterministic -- no seed, no rejection.

    The cart is mirrored by shifting its crossing phase by pi, not by negating
    its amplitude: the triangle-wave offset satisfies f(u + 1/2) = -f(u)
    exactly, so a half-period shift is an exact reflection of the whole
    trajectory, whereas a negative amplitude would corrupt the period formula
    (period = 4 * amplitude / speed).
    """
    obstacles = []
    for obstacle in _l00_obstacles(config):
        flipped = dict(obstacle)
        flipped["xy"] = [obstacle["xy"][0], -obstacle["xy"][1]]
        if "yaw_rad" in obstacle:
            flipped["yaw_rad"] = -obstacle["yaw_rad"]
        if _is_moving(obstacle):
            motion = dict(obstacle["motion"])
            motion["phase_rad"] = float(
                (motion.get("phase_rad", 0.0) + math.pi) % (2.0 * math.pi))
            flipped["motion"] = motion
        obstacles.append(flipped)
    return obstacles


def _aisle_racks(rng, config: dict, rules: dict) -> list[dict]:
    """Two racks on alternated sides, each still crossing the centre line."""
    assets = config["assets"]
    half_y = assets["rack"]["size_xy"][1] / 2.0
    low, high = rules["intrusion_range_m"]
    depth_a = float(rng.uniform(low, high))
    depth_b = float(rng.uniform(low, high))
    x_a = float(rng.uniform(*rules["rack_a_x_range_m"]))
    x_b = float(rng.uniform(*rules["rack_b_x_range_m"]))
    if rules.get("swap_sides", True):
        # rack_a moves to the right half (centre y > 0) and rack_b to the left,
        # the opposite of L00, so the S-bend runs the other way.
        y_a = half_y - depth_a
        y_b = depth_b - half_y
    else:
        y_a = depth_a - half_y
        y_b = half_y - depth_b
    return [_asset(assets, "rack", "rack_a", (x_a, y_a)),
            _asset(assets, "rack", "rack_b", (x_b, y_b))]


def _family_l03(rng, config: dict) -> list[dict]:
    rules = config["families"]["L03"]
    obstacles = _aisle_racks(rng, config, rules)
    obstacles.append(_cart(config, config["cart_motion"]["preferred_lane_x_m"]))
    # Columns and the crate stack are re-drawn in whatever margin the racks left.
    for oid, x_range, y_range in (
            ("column_1", (1.4, 2.8), (1.6, 2.6)),
            ("column_2", (7.2, 8.6), (-2.6, -1.6)),
            ("crate_stack", (7.0, 8.3), (0.6, 1.8))):
        kind = "column" if oid.startswith("column") else "crate"
        yaw = (float(rng.uniform(-math.pi / 4, math.pi / 4))
               if kind == "crate" else 0.0)
        x, y = _sample_free_point(rng, config, obstacles, kind=kind, yaw=yaw,
                                  x_range=x_range, y_range=y_range)
        obstacles.append(_asset(config["assets"], kind, oid, (x, y), yaw))
    return obstacles


def _family_clutter(rng, config: dict, rules: dict) -> list[dict]:
    """L03's aisle plus N extra crate stacks drawn by rejection sampling."""
    obstacles = _family_l03(rng, config)
    assets = config["assets"]
    yaw_low, yaw_high = rules["crate_yaw_range_deg"]
    exclude = rules.get("exclude_radius_from_goal_m", 0.0)
    for index in range(int(rules["extra_crates"])):
        yaw = math.radians(float(rng.uniform(yaw_low, yaw_high)))
        x, y = _sample_free_point(
            rng, config, obstacles, kind="crate", yaw=yaw,
            x_range=(1.6, 8.6), y_range=(-2.5, 2.5),
            keepout_extra=max(0.0, exclude - config["constraints"]["start_goal_keepout_m"]))
        obstacles.append(_asset(assets, "crate", f"crate_extra_{index + 1}",
                                (x, y), yaw))
    return obstacles


def _family_l05(rng, config: dict) -> list[dict]:
    """The hard rung: a deliberately built narrow gate, plus clutter and an occluder.

    The gate is CONSTRUCTED rather than sampled. Two constraints pull against
    each other: ``min_obstacle_gap_m`` (0.50 m) forbids any two static bodies
    from sitting closer than 0.50 m, which is also the floor of the narrow
    passage the family is supposed to contain. A narrow gate is therefore only
    reachable within a thin band right at the gap limit, and uniform rejection
    sampling finds it so rarely that 16 of 20 draws died on `free_point`
    instead. Placing the gate explicitly and then VERIFYING the measured
    widest-path bottleneck against `target_corridor_width_m` keeps the
    difficulty claim empirical while making the search terminate.

    Construction, on rack_b's wide side at rack_b's own station x:

        rack_b  [..., y_b]  <- gap w ->  crate  [0.7]  <- gap g ->  column [0.4]  -> wall

    `w` is sampled from the acceptance band. The column sits flush against the
    far wall margin so the slot it leaves to the wall (0.22 m) is below the
    0.36 m drone diameter and is therefore closed, which is what forces the
    route through the gate rather than around the picket.
    """
    rules = config["families"]["L05"]
    constraints = config["constraints"]
    assets = config["assets"]
    low_band, high_band = rules["target_corridor_width_m"]

    obstacles = _aisle_racks(rng, config, config["families"]["L03"])
    rack_b = obstacles[1]
    # rack_b is on the left after the side swap, so its wide free side is +y.
    rack_b_top = rack_b["xy"][1] + assets["rack"]["size_xy"][1] / 2.0
    wall_limit = (config["base"]["arena"]["high_xy"][1]
                  - constraints["wall_margin_m"])

    gate = float(rng.uniform(low_band, high_band))
    crate_half = assets["crate"]["size_xy"][1] / 2.0
    column_half = assets["column"]["size_xy"][1] / 2.0
    crate_y = rack_b_top + gate + crate_half
    column_y = wall_limit - column_half
    picket_gap = (column_y - column_half) - (crate_y + crate_half)
    if picket_gap < constraints["min_obstacle_gap_m"]:
        raise Rejected("gate_geometry",
                       f"picket gap {picket_gap:.3f} m below the "
                       f"{constraints['min_obstacle_gap_m']} m minimum")
    obstacles.append(_asset(assets, "crate", "gate_crate",
                            (rack_b["xy"][0], crate_y)))
    obstacles.append(_asset(assets, "column", "gate_column",
                            (rack_b["xy"][0], column_y)))

    obstacles.append(_cart(config, config["cart_motion"]["preferred_lane_x_m"]))

    # An occluder on the final approach: a crate inside an annulus around the
    # goal, so the goal is not visible in a straight line from mid-aisle.
    inner, outer = rules["occluder_radius_from_goal_m"]
    goal = np.asarray(config["base"]["goal"]["xy"], dtype=float)
    for _ in range(400):
        radius = float(rng.uniform(inner, outer))
        angle = float(rng.uniform(math.pi / 2, 3 * math.pi / 2))   # approach side
        point = goal + radius * np.array([math.cos(angle), math.sin(angle)])
        yaw = math.radians(float(rng.uniform(*rules["crate_yaw_range_deg"])))
        candidate = _asset(assets, "crate", "crate_occluder",
                           (float(point[0]), float(point[1])), yaw)
        corners = _corners(_placed(candidate))
        if (corners[:, 0].min() < constraints["wall_margin_m"]
                or corners[:, 0].max() > 10.0 - constraints["wall_margin_m"]
                or abs(corners[:, 1]).max() > wall_limit):
            continue
        if _point_gap(candidate, goal) < constraints["start_goal_keepout_m"]:
            continue
        if any(_pair_gap(candidate, other)
               < (constraints["cart_lane_clear_m"] if _is_moving(other)
                  else constraints["min_obstacle_gap_m"])
               for other in obstacles):
            continue
        obstacles.append(candidate)
        break
    else:
        raise Rejected("occluder", "no valid occluder placement")

    # A little extra clutter so L05 is not merely L03-with-a-gate.
    for index in range(int(rules["extra_crates"])):
        yaw = math.radians(float(rng.uniform(*rules["crate_yaw_range_deg"])))
        try:
            x, y = _sample_free_point(rng, config, obstacles, kind="crate",
                                      yaw=yaw, x_range=(1.6, 8.6),
                                      y_range=(-2.5, 2.5), attempts=200)
        except Rejected:
            # The gate picket plus the occluder already consume most of the free
            # area. Stopping early with fewer crates is preferable to discarding
            # a layout whose defining feature -- the gate -- is already correct;
            # the realised count is recorded in the scenario table.
            break
        obstacles.append(_asset(assets, "crate", f"crate_extra_{index + 1}",
                                (x, y), yaw))
    return obstacles


# ---------------------------------------------------------------------------
# assembly
# ---------------------------------------------------------------------------
def _assemble(config: dict, family: str, obstacles: list[dict], *,
              mode: str, seed: int | None, attempts: int,
              rejections: dict) -> dict:
    """Wrap a validated obstacle set in the frozen L00 envelope."""
    base = config["base"]
    rules = config["families"][family]
    suffix = {"C": "", "B": "_static"}[mode]
    scenario = {
        "schema": SCENARIO_SCHEMA,
        "name": f"{family.lower()}{suffix or '_dynamic'}",
        "description": " ".join(rules["description"].split()),
        "arena": dict(base["arena"]),
        "drone": dict(base["drone"]),
        "goal": dict(base["goal"]),
        "obstacles": obstacles,
        "control": dict(base["control"]),
        "dynamics": dict(base["dynamics"]),
        "sensing": dict(base["sensing"]),
        "collision": dict(base["collision"]),
        "camera": dict(base["camera"]),
        "viewpoint": dict(base["viewpoint"]),
        "provenance": {
            "layout_id": family,
            "layout_title": rules["title"],
            "mode": mode,
            "mode_meaning": ("C_dynamic: the cart sweeps the aisle"
                             if mode == "C" else
                             "B_static: the cart is parked at the arena edge, "
                             "amplitude 0"),
            "family_kind": rules["kind"],
            "test_only": bool(rules.get("test_only", True)),
            "generator": "dns.generator",
            "generator_config": "07_SCENARIOS/generator_config.yaml",
            "generator_config_sha256": config_sha256(),
            "seed": seed,
            "draw_attempts": attempts,
            "rejections_by_rule": dict(sorted(rejections.items())),
            "scenario_sha256": "",
        },
    }
    scenario["provenance"]["scenario_sha256"] = scenario_sha256(scenario)
    return scenario


def _validated_draw(config: dict, family: str) -> tuple[list[dict], int, dict, int | None]:
    """Draw and validate until a candidate passes every hard constraint.

    Returns the accepted obstacle set, the number of draws it took, the
    per-rule rejection counts and the seed used.
    """
    rules = config["families"][family]
    kind = rules["kind"]
    validator = Validator(config["constraints"], config["base"])
    rejections: dict[str, int] = {}

    if kind == "reference":
        obstacles = _l00_obstacles(config)
        validator.check({"obstacles": obstacles})
        return obstacles, 1, rejections, None
    if kind == "mirror":
        obstacles = _family_l02(config)
        validator.check({"obstacles": obstacles})
        return obstacles, 1, rejections, None

    if kind == "clutter":
        def sampler(rng, cfg):
            return _family_clutter(rng, cfg, cfg["families"][family])
    else:
        sampler = {"jitter": _family_l01, "aisle_shift": _family_l03,
                   "hard": _family_l05}.get(kind)
        if sampler is None:
            raise ValueError(f"unknown family kind {kind!r}")

    seed = int(rules["seed"])
    target = rules.get("target_corridor_width_m")
    max_attempts = int(config["constraints"]["max_attempts"])
    # The draw index is folded into the RNG seed so every attempt is an
    # independent, reproducible draw: re-running the generator replays exactly
    # the same sequence of candidates and accepts the same one.
    for attempt in range(1, max_attempts + 1):
        rng = np.random.default_rng(_stable_seed(seed, family, attempt))
        try:
            obstacles = sampler(rng, config)
            validator.check({"obstacles": obstacles})
            if target is not None:
                probe = _assemble(config, family, obstacles, mode="C", seed=seed,
                                  attempts=attempt, rejections=rejections)
                scenario = _load_dict(probe)
                facts, path, _grid = describe(
                    scenario, step_m=config["constraints"]["grid_m"],
                    inflate_margin_m=config["constraints"]["inflate_margin_m"])
                if path is None:
                    raise Rejected("astar_infeasible", "no path for the drone disc")
                width = facts["min_corridor_width_m"]
                if not (target[0] <= width <= target[1]):
                    raise Rejected("corridor_width_target",
                                   f"bottleneck {width:.3f} m outside "
                                   f"[{target[0]}, {target[1]}] m")
        except Rejected as rejection:
            rejections[rejection.rule] = rejections.get(rejection.rule, 0) + 1
            continue
        return obstacles, attempt, rejections, seed

    raise RuntimeError(
        f"family {family} exhausted {max_attempts} draws without a valid "
        f"layout; binding rules: {dict(sorted(rejections.items()))}")


def _load_dict(scenario: dict) -> Scenario:
    """Load a scenario dict through the real loader, via a temp canonical file.

    Going through :meth:`Scenario.load` rather than constructing a
    :class:`Scenario` directly means the generator is held to exactly the same
    schema and validation as any hand-written config -- including the v1
    clearance and visibility checks.
    """
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False,
                                     encoding="utf8") as handle:
        handle.write(canonical_text(scenario))
        temp = Path(handle.name)
    try:
        return Scenario.load(temp)
    finally:
        temp.unlink(missing_ok=True)


def build_family(config: dict, family: str) -> dict:
    """Both modes of one layout, plus its descriptors.

    Mode B and Mode C share the *same* static obstacle set by construction --
    they differ only in whether the cart sweeps or is parked -- so a Mode B/C
    comparison on any layout isolates obstacle motion exactly as it did in v1.
    """
    obstacles, attempts, rejections, seed = _validated_draw(config, family)
    lane_x = next((o["xy"][0] for o in obstacles if _is_moving(o)),
                  config["cart_motion"]["preferred_lane_x_m"])
    statics = [o for o in obstacles if not _is_moving(o)]

    dynamic = _assemble(config, family, obstacles, mode="C", seed=seed,
                        attempts=attempts, rejections=rejections)
    static = _assemble(config, family, statics + [_parked_cart(config, lane_x)],
                       mode="B", seed=seed, attempts=attempts,
                       rejections=rejections)
    return {"family": family, "dynamic": dynamic, "static": static,
            "attempts": attempts, "rejections": rejections, "seed": seed}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG_PATH)
    parser.add_argument("--family", action="append", default=None)
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--out", type=Path, default=SCENARIO_DIR)
    args = parser.parse_args(argv)

    config = yaml.safe_load(args.config.read_text(encoding="utf8"))
    families = args.family or (list(config["families"]) if args.all else ["L00"])
    args.out.mkdir(parents=True, exist_ok=True)

    hashes: list[str] = []
    report: dict = {"generator_config_sha256": config_sha256(args.config),
                    "families": {}}
    for family in families:
        built = build_family(config, family)
        for mode, key in (("C", "dynamic"), ("B", "static")):
            scenario = built[key]
            name = f"{family}_{mode}.json"
            path = args.out / name
            path.write_text(canonical_text(scenario), encoding="utf8")
            digest = scenario["provenance"]["scenario_sha256"]
            hashes.append(f"{digest}  {name}")
            print(f"wrote {path.name}  sha256={digest[:16]}  "
                  f"obstacles={len(scenario['obstacles'])}")
        report["families"][family] = {
            "seed": built["seed"], "draw_attempts": built["attempts"],
            "rejections_by_rule": built["rejections"],
            "n_obstacles": len(built["dynamic"]["obstacles"]),
        }

    (args.out / "HASHES.txt").write_text(
        "# SHA-256 of each canonical scenario JSON, hash field excluded.\n"
        "# Regenerate with: python -m dns.generator --all\n"
        + "\n".join(sorted(hashes)) + "\n", encoding="utf8")
    (PACKAGE / "06_DATA" / "generator_report.json").parent.mkdir(
        parents=True, exist_ok=True)
    (PACKAGE / "06_DATA" / "generator_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf8")
    print(f"\nwrote {args.out / 'HASHES.txt'} ({len(hashes)} scenarios)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
