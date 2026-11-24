# ADR-007 — Gimbal-stabilised LiDAR mount

**Status:** Accepted with a stated limitation

## Context
Spencer's 12-sector contract assumes a horizontal scan plane. A body-fixed
scanner tilts with the airframe, so the plane dips and the sector ranges stop
meaning what the contract says.

## Options considered
1. Body-fixed mount — what a cheap fixed scanner does.
2. Ideal gimbal — counter-rotate the mount every control tick.

## Decision
**Ideal gimbal (`lidar_mount_mode = "stabilised"`).** The mount is
counter-rotated each tick so the scan plane stays horizontal whatever the
airframe is doing.

## Evidence
Measured in v1 (`docs/SENSOR_ATTITUDE.md`): a body-fixed plane lets a wall only
slightly taller than the sensor drop out of the scan, and Webots' out-of-range
`inf` is then encoded as "observed clear to 5 m". The mount height is also
constrained — at 0.075 m the levelled plane clipped the rear rotor and two
sectors read a constant 0.177 m, exactly the rotor tip radius; the clearance is
now asserted in `Scenario._check_lidar_clearances`.

## Consequences
+ The sector contract is actually true, matching the 2-D world the policies
  were trained in.
− **An ideal gimbal is a best case, not a real sensor.** A real stabilised mount
  has finite bandwidth and residual error. Ideal-ON and body-fixed-OFF bracket a
  real gimbal; the bracket experiment is listed as not yet run.

---

**Related:** ADR-003 · [ADR index](README.md) ·
[decision log](DECISIONS_LOG.md)
