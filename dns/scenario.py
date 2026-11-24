"""Scenario configuration and the DNS <-> Webots coordinate transform.

Frames
------
DNS frame (Spencer's contract, schema ``fixed-yaw-frd-sectors-v1``)
    +x forward, +y right, planar, yaw fixed. Sector *i* is centred at
    ``i * 30 deg`` measured from +x rotating towards +y, so sector 0 is straight
    ahead, 3 is right, 6 is behind and 9 is left.

Webots world frame
    ENU, the R2025a default: +x east, +y north, +z up. The drone is spawned with
    yaw 0, which puts its body +x along world +x and its body +y (left) along
    world +y.

With yaw held at zero the drone's nose is world +x, so "forward" agrees in both
frames. "Right" of a +x-facing, +z-up vehicle is world **-y**, which gives the
whole transform::

    x_dns  =  x_webots          vx_dns  =  vx_webots
    y_dns  = -y_webots          vy_dns  = -vy_webots

A DNS bearing therefore maps to a Webots body bearing of the opposite sign::

    theta_webots = -theta_dns

This module is the only place those signs are written down. Everything else
calls :func:`to_dns`, :func:`to_webots` and :func:`bearing_to_webots`.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import numpy as np

SCENARIO_SCHEMA = "dns-scenario-v1"

# --------------------------------------------------------------------------
# coordinate transform
# --------------------------------------------------------------------------
#: Element-wise sign flip taking a planar DNS vector to a planar Webots vector.
#: The transform is an involution, so the same array converts in both directions.
_FLIP = np.array([1.0, -1.0])


def to_webots(xy_dns: Sequence[float]) -> np.ndarray:
    """Planar DNS (forward, right) -> planar Webots world (east, north)."""
    return np.asarray(xy_dns, dtype=float) * _FLIP


def to_dns(xy_webots: Sequence[float]) -> np.ndarray:
    """Planar Webots world (east, north) -> planar DNS (forward, right)."""
    return np.asarray(xy_webots, dtype=float) * _FLIP


def bearing_to_webots(theta_dns: float | np.ndarray) -> float | np.ndarray:
    """DNS bearing (from +x towards +y/right) -> Webots body bearing."""
    return -np.asarray(theta_dns, dtype=float)


def bearing_to_dns(theta_webots: float | np.ndarray) -> float | np.ndarray:
    """Webots body bearing (from +x towards +y/left) -> DNS bearing."""
    return -np.asarray(theta_webots, dtype=float)


# --------------------------------------------------------------------------
# configuration
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class Motion:
    """Scripted obstacle motion. ``static`` until Stage F enables the rest."""
    type: str = "static"
    axis: str = "y"               # "x" or "y", the DNS axis the motion runs along
    amplitude_m: float = 0.0
    period_s: float = 4.0
    phase_rad: float = 0.0
    speed_mps: float = 0.0        # used by "linear" (back-and-forth) motion

    @staticmethod
    def parse(raw: dict[str, Any] | None) -> "Motion":
        raw = dict(raw or {})
        kind = raw.pop("type", "static")
        known = {f for f in Motion.__dataclass_fields__ if f != "type"}
        return Motion(type=kind, **{k: v for k, v in raw.items() if k in known})


@dataclass(frozen=True)
class PlacedObstacle:
    """An obstacle's footprint at one instant, with the geometry queries.

    Separated from :class:`Obstacle` because a moving obstacle has a different
    footprint at every time, and both the collision test and the analytic
    ray-cast reference need to ask geometric questions about a *placed* shape.

    Distances are to the **surface**; a point inside the shape returns 0.
    """
    centre: tuple[float, float]
    shape: str                    # "cylinder" or "box"
    radius_m: float               # cylinder radius, or the box's bounding radius
    half_size: tuple[float, float] = (0.0, 0.0)   # box half-extents, DNS axes
    yaw_rad: float = 0.0          # box rotation about +z

    def _to_local(self, points: np.ndarray) -> np.ndarray:
        """World points -> the box's own axis-aligned frame."""
        delta = np.asarray(points, dtype=float) - np.asarray(self.centre)
        if abs(self.yaw_rad) < 1e-12:
            return delta
        c, s = math.cos(-self.yaw_rad), math.sin(-self.yaw_rad)
        rotation = np.array([[c, -s], [s, c]])
        return delta @ rotation.T

    def surface_distance(self, points) -> np.ndarray:
        """Distance from each point to the shape's surface, 0 if inside."""
        points = np.atleast_2d(np.asarray(points, dtype=float))
        if self.shape == "cylinder":
            radial = np.linalg.norm(points - np.asarray(self.centre), axis=1)
            return np.maximum(radial - self.radius_m, 0.0)
        local = np.abs(self._to_local(points))
        outside = np.maximum(local - np.asarray(self.half_size), 0.0)
        return np.linalg.norm(outside, axis=1)

    def ray_distance(self, origin, directions, max_range: float) -> np.ndarray:
        """Distance along each unit direction to the first surface hit.

        ``inf`` where the ray misses. Circles use the standard quadratic;
        boxes use the slab method in the box's own frame.
        """
        origin = np.asarray(origin, dtype=float)
        directions = np.atleast_2d(np.asarray(directions, dtype=float))
        if self.shape == "cylinder":
            relative = np.asarray(self.centre) - origin
            project = directions @ relative
            discriminant = (self.radius_m ** 2
                            - (float(relative @ relative) - project ** 2))
            entry = project - np.sqrt(np.maximum(discriminant, 0.0))
            return np.where((discriminant >= 0) & (entry >= 0), entry, np.inf)

        # Slab method. Rotating the ray instead of the box keeps the test
        # axis-aligned, which is where the slab method is exact.
        local_origin = self._to_local(origin[None, :])[0]
        if abs(self.yaw_rad) < 1e-12:
            local_dirs = directions
        else:
            c, s = math.cos(-self.yaw_rad), math.sin(-self.yaw_rad)
            local_dirs = directions @ np.array([[c, -s], [s, c]]).T

        half = np.asarray(self.half_size, dtype=float)
        with np.errstate(divide="ignore", invalid="ignore"):
            inverse = 1.0 / local_dirs
            t1 = (-half - local_origin) * inverse
            t2 = (half - local_origin) * inverse
        near = np.nanmax(np.minimum(t1, t2), axis=1)
        far = np.nanmin(np.maximum(t1, t2), axis=1)
        # A ray parallel to a slab and outside it never hits.
        parallel_outside = np.any(
            (np.abs(local_dirs) < 1e-12) & (np.abs(local_origin) > half), axis=1)
        hit = (far >= np.maximum(near, 0.0)) & (far >= 0.0) & ~parallel_outside
        entry = np.where(near >= 0.0, near, 0.0)
        return np.where(hit, entry, np.inf)


@dataclass(frozen=True)
class Obstacle:
    """A static or scripted-moving obstacle.

    ``shape`` is ``"cylinder"`` (a structural column or drum) or ``"box"`` (a
    storage rack, crate or cart). Boxes exist because a warehouse scene drawn
    with rectangular racks but *collided* as discs would report contact where
    there is clearly free aisle, which would quietly corrupt every collision
    statistic. ``label`` and ``purpose`` are carried through to the figures and
    tables so each obstacle's reason for existing is documented with it.
    """
    id: str
    xy: tuple[float, float]       # DNS frame, the motion centre
    radius_m: float = 0.5         # cylinder radius; ignored for boxes
    shape: str = "cylinder"
    size_xy: tuple[float, float] = (0.6, 0.6)     # box footprint, DNS axes
    yaw_rad: float = 0.0
    height_m: float = 1.8
    colour: tuple[float, float, float] = (0.8, 0.3, 0.2)
    label: str = ""
    purpose: str = ""
    motion: Motion = field(default_factory=Motion)

    @property
    def bounding_radius(self) -> float:
        """Smallest circle containing the footprint. Used for quick rejects."""
        if self.shape == "cylinder":
            return self.radius_m
        return float(math.hypot(self.size_xy[0] / 2.0, self.size_xy[1] / 2.0))

    def placed_at(self, t: float) -> PlacedObstacle:
        centre = self.centre_at(t)
        return PlacedObstacle(
            centre=(float(centre[0]), float(centre[1])), shape=self.shape,
            radius_m=self.bounding_radius,
            half_size=(self.size_xy[0] / 2.0, self.size_xy[1] / 2.0),
            yaw_rad=self.yaw_rad)

    def centre_at(self, t: float) -> np.ndarray:
        """DNS centre at simulation time ``t`` seconds. Deterministic in ``t``."""
        base = np.asarray(self.xy, dtype=float)
        m = self.motion
        if m.type == "static" or m.amplitude_m == 0.0:
            return base
        index = 0 if m.axis == "x" else 1
        offset = base.copy()
        if m.type == "oscillate":
            # smooth sinusoid, matching planar.py's moving-obstacle model
            omega = 2.0 * np.pi / m.period_s
            offset[index] += m.amplitude_m * np.sin(omega * t + m.phase_rad)
        elif m.type == "linear":
            # constant-speed back-and-forth. One full cycle covers 4 * amplitude
            # of travel, so an explicit speed fixes the period and vice versa.
            period = (4.0 * m.amplitude_m / m.speed_mps) if m.speed_mps else m.period_s
            u = ((t / period) + m.phase_rad / (2.0 * np.pi)) % 1.0
            offset[index] += m.amplitude_m * (1.0 - 4.0 * abs(u - 0.5))
        else:
            raise ValueError(f"Unknown obstacle motion type: {m.type!r}")
        return offset

    @staticmethod
    def parse(raw: dict[str, Any]) -> "Obstacle":
        shape = str(raw.get("shape", "cylinder"))
        if shape not in ("cylinder", "box"):
            raise ValueError(f"obstacle {raw.get('id')!r}: unknown shape {shape!r}")
        if shape == "cylinder" and "radius_m" not in raw:
            raise ValueError(f"obstacle {raw.get('id')!r}: cylinder needs radius_m")
        if shape == "box" and "size_xy" not in raw:
            raise ValueError(f"obstacle {raw.get('id')!r}: box needs size_xy")
        size = raw.get("size_xy", (0.6, 0.6))
        return Obstacle(
            id=str(raw["id"]),
            xy=(float(raw["xy"][0]), float(raw["xy"][1])),
            radius_m=float(raw.get("radius_m", 0.5)),
            shape=shape,
            size_xy=(float(size[0]), float(size[1])),
            yaw_rad=float(raw.get("yaw_rad", 0.0)),
            height_m=float(raw.get("height_m", 1.8)),
            colour=tuple(float(c) for c in raw.get("colour", (0.8, 0.3, 0.2))),  # type: ignore[arg-type]
            label=str(raw.get("label", "")),
            purpose=str(raw.get("purpose", "")),
            motion=Motion.parse(raw.get("motion")),
        )


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    low_xy: tuple[float, float]
    high_xy: tuple[float, float]
    wall_height_m: float
    wall_thickness_m: float
    start_xy: tuple[float, float]
    cruise_altitude_m: float
    collision_radius_m: float
    fixed_yaw_rad: float
    goal_xy: tuple[float, float]
    goal_tolerance_m: float
    obstacles: tuple[Obstacle, ...]
    command_period_s: float
    max_speed_mps: float
    basic_timestep_ms: int
    settle_timeout_s: float
    settle_speed_tol_mps: float
    settle_altitude_tol_m: float
    dynamics_mode: str
    sensing_mode: str
    sensing_max_range_m: float
    sensing_sectors: int
    collision_mode: str
    viewpoint_position: tuple[float, float, float]
    viewpoint_orientation: tuple[float, float, float, float]
    #: How the lidar is attached to the airframe.
    #:
    #: ``stabilised``
    #:     An ideal gimbal: the mount is counter-rotated every control tick so
    #:     the scan plane stays horizontal whatever the airframe is doing. This
    #:     is what makes the flat 12-sector contract in ``navigation.py``
    #:     actually true, and it matches the 2-D world Spencer's policies were
    #:     trained in.
    #: ``body_fixed``
    #:     Bolted to the airframe, so the scan plane tilts with it. Physically
    #:     what a cheap fixed scanner does, and **measurably destructive**: see
    #:     ``results/lidar_attitude.json`` and ``docs/SENSOR_ATTITUDE.md``.
    #:     Retained deliberately as the independent variable for scenario S7.
    lidar_mount_mode: str = "stabilised"
    #: Lidar mount height above the drone body origin [m]. Must match
    #: ``lidarHeight`` in ``protos/DnsQuadcopter.proto``.
    lidar_height_m: float = 0.14
    #: Beams in one 360 deg sweep. 720 gives 0.5 deg spacing and 60 beams per
    #: 30 deg sector. Beam spacing bounds how finely an obstacle edge can be
    #: localised, so it sets the worst-case grazing error where a cylinder
    #: overlaps a sector boundary by less than one beam pitch.
    lidar_resolution: int = 720
    #: Camera tracks the drone. Off by default: a 10 x 6 m arena fits in one
    #: fixed wide shot, and a static frame makes the trajectory legible in a
    #: recording, whereas a following camera hides how far the drone travelled.
    viewpoint_follow: bool = False
    #: Optional offscreen demonstration camera on the drone. It carries no mass
    #: or collider, and the backend world-locks it, so it changes nothing that
    #: is measured -- it exists only to record footage.
    #: Paint a 1 m reference grid on the floor. Visual only, no collider, far
    #: below the scan plane, so it gives scale cues without touching sensing.
    floor_grid: bool = True
    camera_enabled: bool = False
    camera_width: int = 1280
    camera_height: int = 720
    source_path: Path | None = None

    # ---- derived -------------------------------------------------------
    @property
    def steps_per_command(self) -> int:
        """Number of Webots basic timesteps inside one 0.2 s control period."""
        exact = self.command_period_s * 1000.0 / self.basic_timestep_ms
        count = int(round(exact))
        if abs(exact - count) > 1e-9:
            raise ValueError(
                f"command_period_s={self.command_period_s} is not an exact multiple "
                f"of basic_timestep_ms={self.basic_timestep_ms}; the simulation clock "
                "would drift away from Spencer's DT check."
            )
        return count

    @property
    def size_xy(self) -> np.ndarray:
        return np.asarray(self.high_xy, dtype=float) - np.asarray(self.low_xy, dtype=float)

    def obstacle_centres(self, t: float) -> np.ndarray:
        """``(n, 2)`` DNS centres at time ``t``. Empty ``(0, 2)`` if none."""
        if not self.obstacles:
            return np.zeros((0, 2))
        return np.stack([o.centre_at(t) for o in self.obstacles])

    @property
    def obstacle_radii(self) -> np.ndarray:
        """Bounding radii. For a box this circumscribes the footprint, so it
        is only valid as a conservative reject -- use :meth:`placed_at` for
        any test that decides a collision or a range."""
        return np.array([o.bounding_radius for o in self.obstacles], dtype=float)

    def placed_at(self, t: float) -> list[PlacedObstacle]:
        """Every obstacle's exact footprint at simulation time ``t``."""
        return [o.placed_at(t) for o in self.obstacles]

    def clearance_at(self, point_xy, t: float) -> float:
        """Ground-truth distance from a point to the nearest obstacle surface.

        This is supervisor knowledge used for scoring and for the feasibility
        checks below. It never enters a policy observation.
        """
        placed = self.placed_at(t)
        if not placed:
            return float("inf")
        point = np.asarray(point_xy, dtype=float)[None, :]
        return float(min(p.surface_distance(point)[0] for p in placed))

    #: Airframe geometry the scan plane has to clear, from DnsQuadcopter.proto:
    #: rotor discs at z = 0.015 reaching out to a 0.1773 m tip radius.
    ROTOR_PLANE_Z_M = 0.015
    ROTOR_TIP_RADIUS_M = 0.1773
    #: Autopilot tilt authority, Gains.max_tilt_rad. Duplicated as a constant so
    #: a scenario can be validated without a Webots session.
    MAX_TILT_RAD = 0.45

    @staticmethod
    def _motion_samples(obstacle: Obstacle, count: int = 61) -> np.ndarray:
        """Times covering one full period of an obstacle's motion."""
        motion = obstacle.motion
        if motion.type == "static" or not motion.amplitude_m:
            return np.zeros(1)
        period = ((4.0 * motion.amplitude_m / motion.speed_mps)
                  if motion.speed_mps else motion.period_s)
        return np.linspace(0.0, period, count)

    def _check_obstacle_separation(self) -> None:
        """Obstacles must not overlap each other at any point in their motion.

        Two interpenetrating solids would render as one fused object, give the
        lidar a surface that matches neither footprint, and make the
        ground-truth clearance meaningless.
        """
        for i, first in enumerate(self.obstacles):
            for second in self.obstacles[i + 1:]:
                times = np.unique(np.concatenate(
                    (self._motion_samples(first), self._motion_samples(second))))
                worst = min(
                    float(first.placed_at(t).surface_distance(
                        np.asarray(second.placed_at(t).centre)[None, :])[0])
                    - second.bounding_radius
                    for t in times)
                if worst <= 0.0:
                    raise ValueError(
                        f"obstacles {first.id} and {second.id} overlap during "
                        f"their motion (worst surface gap {worst:.3f} m)")

    #: Altitude the controller may lose or gain transiently. The settle
    #: tolerance is 0.02 m, so 0.10 m is a conservative allowance.
    ALTITUDE_MARGIN_M = 0.10

    def _check_obstacle_visibility(self) -> None:
        """Every obstacle must actually intersect the lidar's scan plane.

        The lidar is a single razor-thin layer, so an obstacle shorter than the
        sensor is invisible to it no matter how solid it is to the collision
        test. That combination -- collidable but unsensable -- is the worst
        possible scenario design: the policy is punished for hitting something
        it was never given the information to avoid.
        """
        if self.sensing_mode != "lidar":
            return
        scan_height = self.cruise_altitude_m + self.lidar_height_m
        for obstacle in self.obstacles:
            if obstacle.height_m <= scan_height + self.ALTITUDE_MARGIN_M:
                raise ValueError(
                    f"obstacle {obstacle.id} is {obstacle.height_m} m tall but "
                    f"the scan plane rides at {scan_height:.3f} m; it would be "
                    f"solid to the collision test and invisible to the lidar. "
                    f"Raise it above "
                    f"{scan_height + self.ALTITUDE_MARGIN_M:.3f} m."
                )

    def _check_lidar_clearances(self) -> None:
        """Reject geometry where the drone would blind or mislead its own lidar.

        Two failure modes, both found by measurement rather than inspection
        (``results/lidar_attitude.json``):

        *self-occlusion* -- a stabilised mount holds the scan plane level while
        the airframe rolls, so relative to the body the plane dips by
        ``tip_radius * tan(tilt)`` on the low side. Mount the lidar too low and
        it range-finds its own rotors, reporting a constant 0.177 m.

        *wall over-shoot* -- a body-fixed plane tilts with the airframe, so a
        wall only slightly taller than the sensor drops out of the scan and
        Webots' out-of-range ``inf`` is encoded as "observed clear to 5 m".
        """
        if self.sensing_mode != "lidar":
            return
        tan_tilt = float(np.tan(self.MAX_TILT_RAD))

        if self.lidar_mount_mode == "stabilised":
            clearance = self.lidar_height_m - self.ROTOR_PLANE_Z_M
            needed = self.ROTOR_TIP_RADIUS_M * tan_tilt
            if clearance <= needed:
                raise ValueError(
                    f"lidar mount at {self.lidar_height_m} m sits {clearance:.4f} m "
                    f"above the rotor plane, but a levelled scan plane drops "
                    f"{needed:.4f} m at the {self.ROTOR_TIP_RADIUS_M} m rotor tip "
                    f"radius under the {self.MAX_TILT_RAD} rad tilt limit. The lidar "
                    "would range-find its own rotors. Raise sensing.mount_height_m "
                    f"above {self.ROTOR_PLANE_Z_M + needed:.4f} m."
                )
        else:
            headroom = self.wall_height_m - (self.cruise_altitude_m + self.lidar_height_m)
            if headroom <= 0:
                raise ValueError(
                    f"body-fixed lidar rides at "
                    f"{self.cruise_altitude_m + self.lidar_height_m:.3f} m but the walls "
                    f"are only {self.wall_height_m} m tall, so the arena boundary is "
                    "never in the scan plane at all"
                )

    def validate(self) -> None:
        low = np.asarray(self.low_xy, dtype=float)
        high = np.asarray(self.high_xy, dtype=float)
        if np.any(high <= low):
            raise ValueError("arena high_xy must exceed low_xy on both axes")
        r = self.collision_radius_m
        for label, point in (("start", self.start_xy), ("goal", self.goal_xy)):
            p = np.asarray(point, dtype=float)
            if np.any(p < low + r) or np.any(p > high - r):
                raise ValueError(
                    f"{label} point {point} is within the {r} m clearance radius of a wall"
                )
        # Start and goal must be clear of every obstacle over its whole sweep,
        # not just at its nominal centre: a moving obstacle that parks on the
        # goal halfway through an episode is an invalid scenario.
        for obstacle in self.obstacles:
            for label, point in (("start", self.start_xy), ("goal", self.goal_xy)):
                samples = self._motion_samples(obstacle)
                gap = min(float(obstacle.placed_at(t).surface_distance(
                    np.asarray(point, dtype=float)[None, :])[0]) for t in samples)
                if gap <= r:
                    raise ValueError(
                        f"obstacle {obstacle.id} comes within the drone's "
                        f"{r} m collision radius of the {label} point "
                        f"(surface gap {gap:.3f} m)"
                    )
        if self.dynamics_mode not in ("physical", "kinematic"):
            raise ValueError(f"unknown dynamics mode {self.dynamics_mode!r}")
        if self.sensing_mode not in ("none", "lidar", "geometric"):
            raise ValueError(f"unknown sensing mode {self.sensing_mode!r}")
        if self.lidar_mount_mode not in ("stabilised", "body_fixed"):
            raise ValueError(f"unknown lidar mount mode {self.lidar_mount_mode!r}")
        self._check_obstacle_separation()
        self._check_lidar_clearances()
        self._check_obstacle_visibility()
        if self.collision_mode not in ("geometric", "physical", "either"):
            raise ValueError(f"unknown collision mode {self.collision_mode!r}")
        self.steps_per_command  # raises when the timestep does not divide cleanly

    # ---- io ------------------------------------------------------------
    @staticmethod
    def load(path: str | Path) -> "Scenario":
        path = Path(path).resolve()
        # utf-8-sig, not utf-8: Windows editors and PowerShell's -Encoding utf8
        # prepend a byte-order mark, which json.loads rejects outright. The
        # codec strips a BOM when present and is otherwise identical.
        raw = json.loads(path.read_text(encoding="utf-8-sig"))
        if raw.get("schema") != SCENARIO_SCHEMA:
            raise ValueError(
                f"{path} declares schema {raw.get('schema')!r}, expected {SCENARIO_SCHEMA!r}"
            )
        arena, drone = raw["arena"], raw["drone"]
        goal, control = raw["goal"], raw["control"]
        sensing = raw.get("sensing", {})
        viewpoint = raw.get("viewpoint", {})
        scenario = Scenario(
            name=raw.get("name", path.stem),
            description=raw.get("description", ""),
            low_xy=tuple(float(v) for v in arena["low_xy"]),        # type: ignore[arg-type]
            high_xy=tuple(float(v) for v in arena["high_xy"]),      # type: ignore[arg-type]
            wall_height_m=float(arena.get("wall_height_m", 1.2)),
            wall_thickness_m=float(arena.get("wall_thickness_m", 0.1)),
            start_xy=tuple(float(v) for v in drone["start_xy"]),    # type: ignore[arg-type]
            cruise_altitude_m=float(drone.get("cruise_altitude_m", 1.0)),
            collision_radius_m=float(drone.get("collision_radius_m", 0.18)),
            fixed_yaw_rad=float(drone.get("fixed_yaw_rad", 0.0)),
            goal_xy=tuple(float(v) for v in goal["xy"]),            # type: ignore[arg-type]
            goal_tolerance_m=float(goal.get("tolerance_m", 0.4)),
            obstacles=tuple(Obstacle.parse(o) for o in raw.get("obstacles", [])),
            command_period_s=float(control.get("command_period_s", 0.2)),
            max_speed_mps=float(control.get("max_speed_mps", 1.2)),
            basic_timestep_ms=int(control.get("basic_timestep_ms", 8)),
            settle_timeout_s=float(control.get("settle_timeout_s", 6.0)),
            settle_speed_tol_mps=float(control.get("settle_speed_tol_mps", 0.02)),
            settle_altitude_tol_m=float(control.get("settle_altitude_tol_m", 0.02)),
            dynamics_mode=str(raw.get("dynamics", {}).get("mode", "physical")),
            sensing_mode=str(sensing.get("mode", "none")),
            sensing_max_range_m=float(sensing.get("max_range_m", 5.0)),
            sensing_sectors=int(sensing.get("sectors", 12)),
            lidar_mount_mode=str(sensing.get("mount", "stabilised")),
            lidar_height_m=float(sensing.get("mount_height_m", 0.14)),
            lidar_resolution=int(sensing.get("beams", 720)),
            collision_mode=str(raw.get("collision", {}).get("mode", "geometric")),
            viewpoint_position=tuple(                                # type: ignore[arg-type]
                float(v) for v in viewpoint.get("position", (5.0, -11.5, 9.0))),
            viewpoint_orientation=tuple(                             # type: ignore[arg-type]
                float(v) for v in viewpoint.get("orientation", (0.46, 0.13, 0.88, 0.62))),
            viewpoint_follow=bool(viewpoint.get("follow", False)),
            floor_grid=bool(raw.get("arena", {}).get("floor_grid", True)),
            camera_enabled=bool(raw.get("camera", {}).get("enabled", False)),
            camera_width=int(raw.get("camera", {}).get("width", 1280)),
            camera_height=int(raw.get("camera", {}).get("height", 720)),
            source_path=path,
        )
        scenario.validate()
        return scenario


DEFAULT_SCENARIO_PATH = Path(__file__).resolve().parents[1] / "config" / "scenario_default.json"


def load_default() -> Scenario:
    return Scenario.load(DEFAULT_SCENARIO_PATH)
