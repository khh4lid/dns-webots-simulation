"""Layout feasibility and difficulty descriptors.

Everything here is an **environment test tool**, not a navigation method. Its
job is to answer, before any controller is allowed near a layout:

* is there a path at all for a disc of the drone's radius (A* on the inflated
  occupancy grid)?
* how tight is the tightest gap that path has to pass through (widest-path
  bottleneck, which is the principled form of "minimum corridor width")?
* how different is this layout from L00, reported separately from how hard it is?

The clearance field is computed from :meth:`dns.scenario.PlacedObstacle.
surface_distance` and the arena bounds -- the same code the collision test and
the analytic ray caster use -- so a layout cannot be declared feasible by one
geometry model and collided by another. That mismatch is exactly the v1 defect
where racks were drawn as rectangles and collided as circles.

No SciPy: the clearance field is evaluated exactly on the grid rather than
approximated by a chamfer distance transform, which is both more accurate and
one fewer dependency.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

import numpy as np

from .scenario import Scenario

#: Neighbour offsets and their step costs for an 8-connected grid.
_NEIGHBOURS = tuple(
    (dx, dy, math.hypot(dx, dy))
    for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)
)


@dataclass(frozen=True)
class Grid:
    """A clearance / occupancy raster over the arena, in the DNS frame.

    ``clearance[j, i]`` is the exact distance from the centre of cell
    ``(i, j)`` to the nearest obstacle surface or arena wall, in metres.
    Row index ``j`` runs over y, column index ``i`` over x.
    """
    low_xy: tuple[float, float]
    step_m: float
    clearance: np.ndarray            # (ny, nx) metres
    xs: np.ndarray                   # (nx,) cell-centre x
    ys: np.ndarray                   # (ny,) cell-centre y

    def blocked(self, radius_m: float) -> np.ndarray:
        """Configuration-space occupancy for a disc of ``radius_m``."""
        return self.clearance <= radius_m

    def index_of(self, point_xy) -> tuple[int, int]:
        """Nearest cell ``(i, j)`` to a DNS point."""
        i = int(np.clip(np.searchsorted(self.xs, point_xy[0]) , 0, len(self.xs) - 1))
        j = int(np.clip(np.searchsorted(self.ys, point_xy[1]), 0, len(self.ys) - 1))
        # searchsorted returns an insertion point; pick the closer neighbour.
        if i > 0 and abs(self.xs[i - 1] - point_xy[0]) < abs(self.xs[i] - point_xy[0]):
            i -= 1
        if j > 0 and abs(self.ys[j - 1] - point_xy[1]) < abs(self.ys[j] - point_xy[1]):
            j -= 1
        return i, j

    def point_of(self, i: int, j: int) -> tuple[float, float]:
        return float(self.xs[i]), float(self.ys[j])


def clearance_grid(scenario: Scenario, t: float = 0.0, step_m: float = 0.05,
                   include_moving: bool = False) -> Grid:
    """Exact distance-to-nearest-surface raster over the arena.

    ``include_moving`` decides whether scripted-moving obstacles contribute.
    Feasibility and the dissimilarity measure use the **static** set only: a
    layout whose only blockage is a cart that will move away is not infeasible,
    and including a time-dependent body would make the descriptor depend on an
    arbitrary choice of ``t``.
    """
    low = np.asarray(scenario.low_xy, dtype=float)
    high = np.asarray(scenario.high_xy, dtype=float)
    xs = np.arange(low[0] + step_m / 2.0, high[0], step_m)
    ys = np.arange(low[1] + step_m / 2.0, high[1], step_m)
    gx, gy = np.meshgrid(xs, ys)
    points = np.column_stack((gx.ravel(), gy.ravel()))

    # Walls first: distance to the nearest arena boundary.
    wall = np.minimum.reduce([
        points[:, 0] - low[0], high[0] - points[:, 0],
        points[:, 1] - low[1], high[1] - points[:, 1],
    ])
    clearance = wall

    for obstacle in scenario.obstacles:
        moving = obstacle.motion.type != "static" and obstacle.motion.amplitude_m
        if moving and not include_moving:
            continue
        placed = obstacle.placed_at(t)
        clearance = np.minimum(clearance, placed.surface_distance(points))

    return Grid(low_xy=(float(low[0]), float(low[1])), step_m=step_m,
                clearance=clearance.reshape(gy.shape), xs=xs, ys=ys)


# ---------------------------------------------------------------------------
# path search
# ---------------------------------------------------------------------------
def astar(grid: Grid, start_xy, goal_xy, radius_m: float) -> list[tuple[float, float]] | None:
    """Shortest 8-connected path for a disc of ``radius_m``, or ``None``.

    The start and goal cells are forced open: both are guaranteed clear of
    obstacles by the scenario validator, but a cell centre within ``radius_m``
    of a *wall* can still read blocked at a coarse grid step, and refusing to
    plan from the contractual start point would be an artefact of the raster.
    """
    blocked = grid.blocked(radius_m)
    ny, nx = blocked.shape
    si, sj = grid.index_of(start_xy)
    gi, gj = grid.index_of(goal_xy)
    blocked[sj, si] = False
    blocked[gj, gi] = False

    step = grid.step_m

    def heuristic(i: int, j: int) -> float:
        dx, dy = abs(i - gi), abs(j - gj)
        return step * (max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy))

    best = {(si, sj): 0.0}
    came: dict[tuple[int, int], tuple[int, int]] = {}
    queue = [(heuristic(si, sj), 0.0, si, sj)]
    while queue:
        _priority, cost, i, j = heapq.heappop(queue)
        if (i, j) == (gi, gj):
            path = [(i, j)]
            while path[-1] in came:
                path.append(came[path[-1]])
            return [grid.point_of(*cell) for cell in reversed(path)]
        if cost > best.get((i, j), math.inf):
            continue
        for dx, dy, weight in _NEIGHBOURS:
            ni, nj = i + dx, j + dy
            if not (0 <= ni < nx and 0 <= nj < ny) or blocked[nj, ni]:
                continue
            # Forbid diagonal moves that cut a corner between two blocked cells.
            if dx and dy and (blocked[j, ni] and blocked[nj, i]):
                continue
            new_cost = cost + weight * step
            if new_cost < best.get((ni, nj), math.inf) - 1e-12:
                best[(ni, nj)] = new_cost
                came[(ni, nj)] = (i, j)
                heapq.heappush(queue, (new_cost + heuristic(ni, nj), new_cost, ni, nj))
    return None


def bottleneck_clearance(grid: Grid, start_xy, goal_xy, radius_m: float,
                         exclude_radius_m: float = 0.0) -> float:
    """Largest value ``c`` such that some start->goal path never drops below it.

    This is the widest-path (maximum-capacity-path) problem, solved with a
    max-first Dijkstra. It is the correct formalisation of "minimum corridor
    width": the tightest point of the *most generous* route, rather than of
    whichever route A* happened to return. The physical free width of that
    bottleneck is ``2 * c``.

    ``exclude_radius_m`` lifts cells near the start and the goal out of the
    minimum. Both endpoints are fixed by contract at 0.85 m from an arena wall,
    so with no exclusion the bottleneck of an uncluttered layout is the
    *endpoint* rather than any obstacle, and L00, L01 and L02 all report the
    same 1.65 m ceiling. Excluding a small neighbourhood gives the en-route
    pinch, which is the quantity that actually varies with the layout.
    """
    clearance = grid.clearance
    if exclude_radius_m > 0.0:
        gx, gy = np.meshgrid(grid.xs, grid.ys)
        near = ((np.hypot(gx - start_xy[0], gy - start_xy[1]) <= exclude_radius_m)
                | (np.hypot(gx - goal_xy[0], gy - goal_xy[1]) <= exclude_radius_m))
        clearance = np.where(near, np.inf, clearance)
    ny, nx = clearance.shape
    si, sj = grid.index_of(start_xy)
    gi, gj = grid.index_of(goal_xy)

    # Negated because heapq is a min-heap; `capacity` is the best-known minimum
    # clearance along any path reaching a cell.
    capacity = np.full((ny, nx), -np.inf)
    start_capacity = max(float(clearance[sj, si]), radius_m)
    capacity[sj, si] = start_capacity
    queue = [(-start_capacity, si, sj)]
    while queue:
        negative, i, j = heapq.heappop(queue)
        value = -negative
        if value < capacity[j, i] - 1e-12:
            continue
        if (i, j) == (gi, gj):
            return float(value)
        for dx, dy, _weight in _NEIGHBOURS:
            ni, nj = i + dx, j + dy
            if not (0 <= ni < nx and 0 <= nj < ny):
                continue
            # The goal cell, like A*'s, is forced reachable at its own clearance.
            candidate = min(value, float(clearance[nj, ni]))
            if (ni, nj) == (gi, gj):
                candidate = min(value, max(float(clearance[nj, ni]), radius_m))
            if candidate > capacity[nj, ni] + 1e-12:
                capacity[nj, ni] = candidate
                heapq.heappush(queue, (-candidate, ni, nj))
    return float("-inf")


def path_length(path) -> float:
    points = np.asarray(path, dtype=float)
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def min_clearance_along(grid: Grid, path) -> float:
    return float(min(grid.clearance[grid.index_of(p)[1], grid.index_of(p)[0]]
                     for p in path))


# ---------------------------------------------------------------------------
# descriptors
# ---------------------------------------------------------------------------
def segment_clearance(scenario: Scenario, a, b, samples: int = 801,
                      t: float = 0.0) -> np.ndarray:
    """Clearance to the nearest *static* surface along a straight segment."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    points = a[None, :] + np.linspace(0.0, 1.0, samples)[:, None] * (b - a)[None, :]
    low = np.asarray(scenario.low_xy, dtype=float)
    high = np.asarray(scenario.high_xy, dtype=float)
    clearance = np.minimum.reduce([
        points[:, 0] - low[0], high[0] - points[:, 0],
        points[:, 1] - low[1], high[1] - points[:, 1]])
    for obstacle in scenario.obstacles:
        if obstacle.motion.type != "static" and obstacle.motion.amplitude_m:
            continue
        clearance = np.minimum(
            clearance, obstacle.placed_at(t).surface_distance(points))
    return clearance


def line_intrusion(scenario: Scenario) -> dict:
    """How, and how deeply, the straight start-goal line is blocked.

    Three numbers, because they answer different questions:

    ``intrusion_m``
        Geometric depth to which a static obstacle's footprint crosses the
        y = 0 centre line, measured on the far side from the obstacle's own
        centre. This is the quantity v1 reported as 0.20 m for L00's racks.
    ``cspace_penetration_m``
        How far the obstacle set, inflated by the drone radius, penetrates the
        segment. Positive means a point drone following the straight line would
        be in collision. Generalises to any geometry.
    ``blocked``
        Whether the inflated set covers the segment at all.
    """
    start = np.asarray(scenario.start_xy, dtype=float)
    goal = np.asarray(scenario.goal_xy, dtype=float)
    radius = scenario.collision_radius_m

    clearance = segment_clearance(scenario, start, goal)
    penetration = float(max(0.0, radius - clearance.min()))

    # Geometric crossing depth of the centre line, per static obstacle.
    deepest = 0.0
    for obstacle in scenario.obstacles:
        if obstacle.motion.type != "static" and obstacle.motion.amplitude_m:
            continue
        placed = obstacle.placed_at(0.0)
        # Does the footprint span any part of the segment in x?
        half = np.asarray(placed.half_size, dtype=float)
        corners = np.array([[-half[0], -half[1]], [half[0], -half[1]],
                            [half[0], half[1]], [-half[0], half[1]]])
        c, s = math.cos(placed.yaw_rad), math.sin(placed.yaw_rad)
        world = corners @ np.array([[c, s], [-s, c]]) + np.asarray(placed.centre)
        if not (world[:, 0].min() <= goal[0] and world[:, 0].max() >= start[0]):
            continue
        if world[:, 1].min() > 0.0 or world[:, 1].max() < 0.0:
            continue                       # does not reach the centre line
        centre_y = placed.centre[1]
        depth = (world[:, 1].max() - 0.0) if centre_y < 0.0 else (0.0 - world[:, 1].min())
        deepest = max(deepest, float(depth))

    return {"blocked": bool(penetration > 0.0),
            "intrusion_m": round(deepest, 4),
            "cspace_penetration_m": round(penetration, 4),
            "min_segment_clearance_m": round(float(clearance.min()), 4)}


def occupied_area_fraction(scenario: Scenario, t: float = 0.0,
                           include_moving: bool = False) -> float:
    """Fraction of the arena floor covered by obstacle footprints."""
    total = float(np.prod(scenario.size_xy))
    area = 0.0
    for obstacle in scenario.obstacles:
        moving = obstacle.motion.type != "static" and obstacle.motion.amplitude_m
        if moving and not include_moving:
            continue
        if obstacle.shape == "cylinder":
            area += math.pi * obstacle.radius_m ** 2
        else:
            area += obstacle.size_xy[0] * obstacle.size_xy[1]
    return area / total


def cart_interaction(scenario: Scenario, path, samples: int = 241) -> dict:
    """How the moving cart interacts with the reference A* path.

    Reported because a dynamic hazard that never reaches the route adds nothing:
    if ``blocking_fraction`` were 0 the layout's Mode C would be identical to
    its Mode B, and if it were 1 the route would be permanently closed and the
    task unsolvable rather than a timing problem.
    """
    movers = [o for o in scenario.obstacles
              if o.motion.type != "static" and o.motion.amplitude_m]
    if not movers or not path:
        return {"cart_present": False, "blocking_fraction": 0.0,
                "crossings": 0, "period_s": 0.0}
    cart = movers[0]
    period = ((4.0 * cart.motion.amplitude_m / cart.motion.speed_mps)
              if cart.motion.speed_mps else cart.motion.period_s)
    waypoints = np.asarray(path, dtype=float)
    radius = scenario.collision_radius_m

    flags = []
    for t in np.linspace(0.0, period, samples, endpoint=False):
        placed = cart.placed_at(t)
        flags.append(bool(placed.surface_distance(waypoints).min() <= radius))
    flags = np.asarray(flags)
    # A "crossing" is one closed->open->closed cycle, i.e. a rising edge.
    rising = int(np.sum((~np.roll(flags, 1)) & flags))
    return {"cart_present": True,
            "blocking_fraction": round(float(flags.mean()), 4),
            "crossings": rising,
            "period_s": round(float(period), 4)}


def iou_static(a: Scenario, b: Scenario, step_m: float = 0.05) -> float:
    """Intersection-over-union of the two layouts' static footprint rasters.

    Computed on the raw footprints (not the inflated C-space) so the measure
    describes the scene rather than the drone. ``1 - IoU`` is reported as the
    dissimilarity to L00.
    """
    grid_a = clearance_grid(a, 0.0, step_m)
    grid_b = clearance_grid(b, 0.0, step_m)
    # clearance == 0 exactly inside a footprint; walls give 0 only on the rim,
    # so compare interiors with a strict test against the obstacle set alone.
    occ_a = _footprint_mask(a, grid_a)
    occ_b = _footprint_mask(b, grid_b)
    union = np.logical_or(occ_a, occ_b).sum()
    if union == 0:
        return 0.0
    return float(np.logical_and(occ_a, occ_b).sum() / union)


def _footprint_mask(scenario: Scenario, grid: Grid) -> np.ndarray:
    gx, gy = np.meshgrid(grid.xs, grid.ys)
    points = np.column_stack((gx.ravel(), gy.ravel()))
    mask = np.zeros(len(points), dtype=bool)
    for obstacle in scenario.obstacles:
        if obstacle.motion.type != "static" and obstacle.motion.amplitude_m:
            continue
        mask |= obstacle.placed_at(0.0).surface_distance(points) <= 0.0
    return mask.reshape(gx.shape)


def mean_displacement(a: Scenario, b: Scenario) -> float:
    """Mean distance between same-id static obstacle centres, metres.

    ``nan`` when the two layouts share no static obstacle ids, which is the
    honest answer for a layout whose assets were re-drawn rather than moved.
    """
    first = {o.id: np.asarray(o.xy, dtype=float) for o in a.obstacles
             if o.motion.type == "static" or not o.motion.amplitude_m}
    second = {o.id: np.asarray(o.xy, dtype=float) for o in b.obstacles
              if o.motion.type == "static" or not o.motion.amplitude_m}
    shared = sorted(set(first) & set(second))
    if not shared:
        return float("nan")
    return float(np.mean([np.linalg.norm(first[k] - second[k]) for k in shared]))


def describe(scenario: Scenario, reference: Scenario | None = None,
             step_m: float = 0.05, inflate_margin_m: float = 0.02) -> dict:
    """Every per-layout descriptor the report and figures need.

    ``feasible`` is the gate: a layout with ``feasible=False`` must never reach
    the experiment, because a policy's failure there would be unattributable.
    """
    radius = scenario.collision_radius_m + inflate_margin_m
    grid = clearance_grid(scenario, 0.0, step_m)
    path = astar(grid, scenario.start_xy, scenario.goal_xy, radius)
    straight = float(np.linalg.norm(
        np.asarray(scenario.goal_xy) - np.asarray(scenario.start_xy)))

    intrusion = line_intrusion(scenario)
    result = {
        "layout": scenario.name,
        "n_obstacles": len(scenario.obstacles),
        "n_static": sum(1 for o in scenario.obstacles
                        if o.motion.type == "static" or not o.motion.amplitude_m),
        "n_moving": sum(1 for o in scenario.obstacles
                        if o.motion.type != "static" and o.motion.amplitude_m),
        "feasible": path is not None,
        "astar_length_m": round(path_length(path), 4) if path else None,
        "straight_line_m": round(straight, 4),
        "tortuosity": round(path_length(path) / straight, 4) if path else None,
        "min_corridor_width_m": None,
        "min_path_clearance_m": round(min_clearance_along(grid, path), 4) if path else None,
        "occupied_area_pct": round(100.0 * occupied_area_fraction(scenario), 3),
        "start_goal_line_of_sight": not intrusion["blocked"],
        **{f"line_{k}": v for k, v in intrusion.items()},
        "inflation_radius_m": round(radius, 4),
        "grid_step_m": step_m,
    }
    if path:
        bottleneck = bottleneck_clearance(grid, scenario.start_xy,
                                          scenario.goal_xy, radius)
        result["min_corridor_width_m"] = round(2.0 * bottleneck, 4)
        result["bottleneck_clearance_m"] = round(bottleneck, 4)
        # The en-route pinch: the same measure with the two contractually fixed
        # endpoints excluded, so it reflects obstacle placement rather than the
        # arena's wall-to-goal distance.
        enroute = bottleneck_clearance(
            grid, scenario.start_xy, scenario.goal_xy, radius,
            exclude_radius_m=scenario.goal_tolerance_m)
        result["min_corridor_width_enroute_m"] = round(2.0 * enroute, 4)
        result.update({f"cart_{k}": v for k, v in
                       cart_interaction(scenario, path).items()})
    if reference is not None:
        result["iou_to_reference"] = round(iou_static(scenario, reference, step_m), 4)
        result["dissimilarity_to_reference"] = round(
            1.0 - iou_static(scenario, reference, step_m), 4)
        displacement = mean_displacement(reference, scenario)
        result["mean_obstacle_displacement_m"] = (
            round(displacement, 4) if np.isfinite(displacement) else None)
    return result, path, grid
