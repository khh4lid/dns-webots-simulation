"""Obstacle sensing: 12 x 30 deg sectors, metres, in the DNS frame.

Three modes, kept strictly separate so nothing fabricated can leak into a
result that looks measured:

``none``
    Every sector reported as ``(0.0, False)`` -- "not observed". This is the
    honest default for milestone 1: the sensor-processing component is owned by
    the sensors/obstacle-detection teammate, and Spencer's ``validate()``
    accepts unknown sectors exactly this way.

``lidar``
    Real returns from the Webots ``Lidar`` device on the drone, binned into
    sectors by Spencer's own ``navigation.pack_scan``. The index -> bearing
    mapping is **calibrated against measured data**, not assumed; see
    :func:`fit_lidar_mapping` and ``tests/client_calibrate.py``.

``geometric``
    Analytic ray-cast against the arena walls and obstacle discs using
    supervisor ground truth, mirroring the maths in Spencer's ``planar.py``.
    This is a reference/debug signal, **not a sensor model**, and every record
    it produces is labelled ``geometric`` in the output.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .spencer_link import RANGE, SECTORS, pack_scan

#: Ray offsets within one sector, matching planar.py's 3-ray sampling.
_SUBRAY_DEG = (-10.0, 0.0, 10.0)


def sector_bearings_dns(sectors: int = SECTORS) -> np.ndarray:
    """Centre bearing of each sector, radians, DNS convention (+x towards +y)."""
    return np.arange(sectors) * (2.0 * math.pi / sectors)


def unobserved(sectors: int = SECTORS) -> tuple[np.ndarray, np.ndarray]:
    """The ``(0.0, False)`` all-unknown scan."""
    return np.zeros(sectors), np.zeros(sectors, dtype=bool)


# ---------------------------------------------------------------------------
# analytic ray-cast (ground truth)
# ---------------------------------------------------------------------------
def cast_rays(origin_dns: np.ndarray, bearings_dns: np.ndarray, low: np.ndarray,
              high: np.ndarray, placed, max_range: float = RANGE) -> np.ndarray:
    """Distance from ``origin_dns`` to the first surface along each bearing.

    Walls are the axis-aligned arena bounds; ``placed`` is a sequence of
    :class:`~dns.scenario.PlacedObstacle`, each of which answers its own
    ray query, so cylinders and boxes are both handled exactly rather than a
    box being approximated by its bounding circle.
    """
    origin = np.asarray(origin_dns, dtype=float)
    rays = np.stack((np.cos(bearings_dns), np.sin(bearings_dns)), axis=-1)

    with np.errstate(divide="ignore", invalid="ignore"):
        wall = np.where(np.abs(rays) > 1e-12,
                        (np.where(rays > 0, high, low) - origin) / rays, np.inf)
    distance = wall.min(axis=1)

    for obstacle in placed:
        distance = np.minimum(
            distance, obstacle.ray_distance(origin, rays, max_range))

    return np.clip(distance, 0.0, max_range)


def geometric_scan(origin_dns: np.ndarray, low: np.ndarray, high: np.ndarray,
                   placed, max_range: float = RANGE,
                   sectors: int = SECTORS) -> tuple[np.ndarray, np.ndarray]:
    """Ground-truth 12-sector scan. Returns ``(ranges_m, valid)``."""
    offsets = np.radians(_SUBRAY_DEG)
    bearings = (sector_bearings_dns(sectors)[:, None] + offsets[None, :]).reshape(-1)
    distances = cast_rays(origin_dns, bearings, low, high, placed, max_range)
    ranges = distances.reshape(sectors, len(offsets)).min(axis=1)
    return ranges, np.ones(sectors, dtype=bool)


# ---------------------------------------------------------------------------
# lidar
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class LidarMapping:
    """Maps a horizontal lidar index onto a DNS bearing.

    ``bearing_dns(i) = direction * (2*pi*i/resolution) + offset``

    Webots documents the range image as "scan lines running from left to
    right", which fixes the ordering but not the sign or the origin once the
    device sits on a robot with its own axes. Both are therefore *measured* by
    :func:`fit_lidar_mapping` rather than assumed.
    """
    resolution: int
    direction: float = 1.0
    offset: float = math.pi
    rms_error_m: float = float("nan")

    @classmethod
    def calibrated(cls, resolution: int = 360) -> "LidarMapping":
        """The mapping measured for the DnsQuadcopter lidar.

        ``tests/client_calibrate.py`` fitted ``direction=+1`` and
        ``offset=+180.00 deg`` against known arena geometry with an RMS
        residual of 0.027 m over 321 finite beams. Re-run that test if the
        lidar's mounting or resolution changes.
        """
        return cls(resolution=resolution, direction=1.0, offset=math.pi,
                   rms_error_m=0.0274)

    def bearings_dns(self) -> np.ndarray:
        index = np.arange(self.resolution)
        return self.direction * (2.0 * math.pi * index / self.resolution) + self.offset

    def to_dict(self) -> dict:
        return {"resolution": self.resolution, "direction": self.direction,
                "offset_rad": self.offset, "offset_deg": math.degrees(self.offset),
                "rms_error_m": self.rms_error_m}


def fit_lidar_mapping(measured: np.ndarray, origin_dns: np.ndarray, low: np.ndarray,
                      high: np.ndarray, placed,
                      max_range: float = RANGE) -> LidarMapping:
    """Recover the index -> bearing mapping from one real scan.

    The drone sits at a known pose in a known arena, so the distance along every
    bearing can be computed analytically. Searching the two unknowns -- sweep
    direction and angular origin -- for the best match against the *measured*
    range image yields the mapping, and the residual RMS is reported so a poor
    fit cannot pass silently.
    """
    measured = np.asarray(measured, dtype=float)
    resolution = len(measured)
    finite = np.isfinite(measured)
    if finite.sum() < 8:
        raise ValueError("not enough finite lidar returns to calibrate")

    best: LidarMapping | None = None
    for direction in (1.0, -1.0):
        for step in range(resolution):
            offset = 2.0 * math.pi * step / resolution
            bearings = direction * (2.0 * math.pi * np.arange(resolution) / resolution) + offset
            expected = cast_rays(origin_dns, bearings, low, high, placed, max_range)
            # Compare only where the sensor returned a real distance; an
            # out-of-range beam carries no positional information.
            usable = finite & (expected < max_range - 1e-6)
            if usable.sum() < 8:
                continue
            rms = float(np.sqrt(np.mean((measured[usable] - expected[usable]) ** 2)))
            if best is None or rms < best.rms_error_m:
                best = LidarMapping(resolution, direction, offset, rms)
    if best is None:
        raise ValueError("lidar calibration found no usable beams")
    return best


def lidar_scan(range_image, mapping: LidarMapping,
               max_range: float = RANGE) -> tuple[np.ndarray, np.ndarray]:
    """Bin a raw Webots range image into 12 sectors with Spencer's ``pack_scan``.

    Beams the device reported are passed through in metres with ``valid=True``;
    Webots returns ``inf`` beyond ``maxRange`` and ``pack_scan`` turns that into
    "observed clear to RANGE", which is exactly the ``(5.0, True)`` encoding
    Spencer specified. No value is invented and nothing is normalised here --
    ``navigation.observation`` is the only place scaling happens.
    """
    distances = np.asarray(range_image, dtype=float)
    if len(distances) != mapping.resolution:
        raise ValueError(
            f"range image has {len(distances)} beams, mapping expects {mapping.resolution}")
    bearings = mapping.bearings_dns()
    valid = ~np.isnan(distances)
    clipped = np.where(np.isfinite(distances), np.clip(distances, 0.0, max_range), np.inf)
    ranges, seen = pack_scan(bearings, clipped, valid)
    return np.clip(ranges, 0.0, max_range), seen
