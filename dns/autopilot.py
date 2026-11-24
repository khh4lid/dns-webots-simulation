"""Cascaded quadcopter autopilot: horizontal velocity -> attitude -> rotor thrust.

The PPO policy issues one horizontal velocity setpoint every 0.2 s. A real
quadcopter cannot track that instantaneously -- it has to tilt before it can
accelerate -- so the setpoint is held while this autopilot closes three nested
loops at the Webots basic timestep (125 Hz at 8 ms):

    velocity  ->  tilt target        (outer, slow)
    attitude  ->  body torque        (inner, fast)
    altitude  ->  collective thrust  (independent)

Nothing here teleports the drone or writes its position: every motion comes out
of rotor thrust applied by Webots' ``Propeller`` nodes to a rigid body.

Sign conventions, cross-checked against Webots' own ``mavic2pro.c`` sample
(``projects/robots/dji/mavic/controllers/mavic2pro``) and re-verified
empirically by ``tests/client_calibrate.py``:

* Webots robot frame: +x front, +y left, +z up. InertialUnit gives
  roll about +x, pitch about +y, yaw about +z.
* **positive pitch tilts the nose down and accelerates the drone along +x**
* **positive roll drops the right side and accelerates it along -y**
* DNS +y is "right" = Webots -y, so a positive roll accelerates towards DNS +y.

Both horizontal axes therefore use a *positive* tilt target for a positive
demand, which keeps the outer loop symmetric.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

GRAVITY = 9.81

#: Rotor order must match ``DnsQuadcopter.proto``: m0 front-left (CCW),
#: m1 front-right (CW), m2 rear-left (CW), m3 rear-right (CCW).
MOTOR_NAMES = ("m0", "m1", "m2", "m3")
#: Hub position signs as multiples of ``arm_offset``: (x, y) in the robot frame.
ROTOR_XY_SIGNS = ((+1, +1), (+1, -1), (-1, +1), (-1, -1))
#: Sign of the commanded motor velocity; +1 spins CCW about +z.
ROTOR_SPIN = (+1, -1, -1, +1)


@dataclass(frozen=True)
class VehicleParams:
    """Physical constants, read back from the PROTO so there is one source.

    The rotor discs are graphical only, so the airframe is a single rigid body
    and ``mass``/``inertia`` describe the whole vehicle.
    """
    mass: float = 0.55
    inertia: tuple[float, float, float] = (0.00272, 0.00272, 0.00513)
    arm_offset: float = 0.09
    thrust_constant: float = 1.5e-05
    torque_constant: float = 2.4e-07
    motor_max_velocity: float = 600.0
    collision_radius: float = 0.18
    lidar_max_range: float = 5.0
    lidar_resolution: int = 360

    @property
    def max_rotor_thrust(self) -> float:
        """Thrust of one rotor at ``motor_max_velocity`` [N]."""
        return self.thrust_constant * self.motor_max_velocity ** 2

    @property
    def hover_rotor_thrust(self) -> float:
        return self.mass * GRAVITY / 4.0

    @property
    def thrust_to_weight(self) -> float:
        return 4.0 * self.max_rotor_thrust / (self.mass * GRAVITY)

    @property
    def yaw_moment_per_newton(self) -> float:
        """Reaction yaw moment produced per newton of rotor thrust [m]."""
        return self.torque_constant / self.thrust_constant

    @staticmethod
    def from_proto(node: Any) -> "VehicleParams":
        """Read the parameters of a ``DnsQuadcopter`` PROTO instance.

        ``node`` is the supervisor node for ``DEF DNS_DRONE``. Falling back to
        the dataclass defaults would hide a world/code mismatch, so a missing
        field is an error.
        """
        def scalar(name: str) -> float:
            handle = node.getField(name)
            if handle is None:
                raise KeyError(f"DnsQuadcopter PROTO has no field {name!r}")
            return float(handle.getSFFloat())

        inertia_field = node.getField("inertia")
        if inertia_field is None:
            raise KeyError("DnsQuadcopter PROTO has no field 'inertia'")
        inertia = tuple(float(v) for v in inertia_field.getSFVec3f())

        resolution_field = node.getField("lidarResolution")
        if resolution_field is None:
            raise KeyError("DnsQuadcopter PROTO has no field 'lidarResolution'")

        return VehicleParams(
            mass=scalar("mass"),
            inertia=inertia,                                   # type: ignore[arg-type]
            arm_offset=scalar("armOffset"),
            thrust_constant=scalar("thrustConstant"),
            torque_constant=scalar("torqueConstant"),
            motor_max_velocity=scalar("motorMaxVelocity"),
            collision_radius=scalar("collisionRadius"),
            lidar_max_range=scalar("lidarMaxRange"),
            lidar_resolution=int(resolution_field.getSFInt32()),
        )


@dataclass
class Gains:
    """Controller gains.

    Attitude and yaw gains are specified as closed-loop second-order terms and
    multiplied by the relevant principal inertia inside the controller, so they
    stay meaningful if the airframe is re-scaled.
    """
    # attitude: omega_n ~ 12 rad/s, zeta ~ 0.9
    kp_attitude: float = 144.0
    kd_attitude: float = 21.6
    # yaw: softer, it only has to hold zero
    kp_yaw: float = 40.0
    kd_yaw: float = 12.0
    # altitude, as a cascade: position error -> climb-rate target -> thrust.
    # A single stiff PD on altitude commands ~15 m/s^2 from a 1 m error, which
    # a thrust-to-weight of 4 happily delivers and then overshoots badly.
    kp_altitude: float = 2.0             # [1/s] altitude error -> climb rate
    kp_climb: float = 5.0                # [1/s] climb-rate error -> acceleration
    max_climb_rate: float = 1.0          # [m/s]
    # horizontal velocity -> tilt
    kp_velocity: float = 4.0
    # With drag fed forward the integrator only has to absorb modelling error,
    # so it is kept small: a large one winds up during the acceleration phase
    # and then overshoots the setpoint while it unwinds.
    ki_velocity: float = 1.2
    integral_limit: float = 0.6          # [m/s * s]
    max_tilt_rad: float = 0.45           # ~25.8 deg -> ~4.75 m/s^2 of authority
    #: Effective linear damping of the world, 1/s. Webots' ``Damping.linear``
    #: is a per-second retention factor, so ``linear 0.5`` (halve the velocity
    #: every second) is an exponential rate of ln(2) = 0.693 1/s. Measured as
    #: 0.6953 from terminal velocity in tests/client_drag_cal.py. Fed forward so
    #: the velocity loop does not have to integrate against a known force.
    linear_damping: float = 0.6931
    #: Sign of the yaw mixing term. **Measured, not assumed**: driving rotors 0
    #: and 3 (both CCW) heavy produces a yaw rate of -1.77 rad/s, so Webots
    #: applies the propeller reaction torque opposite to the naive convention.
    #: With the wrong sign the yaw loop is positive feedback, which stays
    #: marginally stable in a hover and diverges during a roll manoeuvre.
    #: See ``tests/client_calibrate.py`` and ``results/calibration.json``.
    yaw_mix_sign: float = -1.0


@dataclass
class AutopilotState:
    """Everything the controller saw and did on the last inner-loop tick."""
    position_w: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity_w: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rates: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tilt_target: np.ndarray = field(default_factory=lambda: np.zeros(2))   # roll, pitch
    thrust_total: float = 0.0
    rotor_thrusts: np.ndarray = field(default_factory=lambda: np.zeros(4))
    saturated: bool = False


class Autopilot:
    """Closes the inner loops. The caller owns ``robot.step``."""

    def __init__(self, robot: Any, params: VehicleParams, gains: Gains | None = None,
                 timestep_ms: int | None = None):
        self.robot = robot
        self.params = params
        self.gains = gains or Gains()
        self.timestep_ms = int(timestep_ms or robot.getBasicTimeStep())
        self.dt = self.timestep_ms / 1000.0

        self.gps = robot.getDevice("gps")
        self.gyro = robot.getDevice("gyro")
        self.imu = robot.getDevice("imu")
        for device in (self.gps, self.gyro, self.imu):
            device.enable(self.timestep_ms)

        self.motors = []
        for name in MOTOR_NAMES:
            motor = robot.getDevice(name)
            motor.setPosition(float("inf"))      # velocity control mode
            motor.setVelocity(0.0)
            self.motors.append(motor)

        self.state = AutopilotState()
        self._integral = np.zeros(2)

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Clear integrator memory. Call after every supervisor reposition."""
        self._integral[:] = 0.0
        self.state = AutopilotState()
        for motor in self.motors:
            motor.setVelocity(0.0)

    def idle(self) -> None:
        """Stop all rotors (used while the physics is being reset)."""
        for motor in self.motors:
            motor.setVelocity(0.0)

    # ------------------------------------------------------------------
    def read_sensors(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        position = np.asarray(self.gps.getValues(), dtype=float)
        velocity = np.asarray(self.gps.getSpeedVector(), dtype=float)
        rpy = np.asarray(self.imu.getRollPitchYaw(), dtype=float)
        rates = np.asarray(self.gyro.getValues(), dtype=float)
        return position, velocity, rpy, rates

    # ------------------------------------------------------------------
    def update(self, velocity_cmd_dns: np.ndarray, target_altitude: float,
               target_yaw: float = 0.0) -> AutopilotState:
        """One inner-loop tick. ``velocity_cmd_dns`` is ``[vx, vy]`` in m/s, DNS axes."""
        g = self.gains
        p = self.params
        position, velocity_w, rpy, rates = self.read_sensors()

        if not (np.all(np.isfinite(position)) and np.all(np.isfinite(velocity_w))
                and np.all(np.isfinite(rpy)) and np.all(np.isfinite(rates))):
            # Sensors are not populated until the first step after enable().
            self.idle()
            return self.state

        roll, pitch, yaw = rpy
        # Webots world -> DNS planar velocity: +y_dns (right) is -y_webots.
        velocity_dns = np.array([velocity_w[0], -velocity_w[1]])

        # ---- outer loop: velocity error -> desired horizontal acceleration
        error = np.asarray(velocity_cmd_dns, dtype=float) - velocity_dns
        self._integral = np.clip(self._integral + error * self.dt,
                                 -g.integral_limit, g.integral_limit)
        # Feed the known aerodynamic drag forward so the integrator only has to
        # absorb modelling error, not a force we can compute.
        accel_des = (g.kp_velocity * error + g.ki_velocity * self._integral
                     + g.linear_damping * velocity_dns)

        # ---- desired tilt. A tilt theta buys g*tan(theta) of lateral accel.
        # +pitch -> +x_dns, +roll -> +y_dns (see module docstring).
        # The *magnitude* is limited, not each axis separately: clipping the
        # axes independently lets a diagonal command reach sqrt(2) times the
        # intended tilt, which drives the attitude loop past the small-angle
        # region it is tuned for and the vehicle tumbles.
        limit = g.max_tilt_rad
        accel_limit = GRAVITY * math.tan(limit)
        demand = float(np.linalg.norm(accel_des))
        if demand > accel_limit:
            accel_des = accel_des * (accel_limit / demand)

        pitch_target = math.atan2(accel_des[0], GRAVITY)
        roll_target = math.atan2(accel_des[1] * math.cos(pitch_target), GRAVITY)

        # Anti-windup: stop integrating once the tilt demand is saturated.
        if demand > accel_limit:
            for axis in range(2):
                if np.sign(error[axis]) == np.sign(self._integral[axis]):
                    self._integral[axis] -= error[axis] * self.dt

        # ---- inner loop: attitude error -> body torques
        ixx, iyy, izz = p.inertia
        tau_x = ixx * (g.kp_attitude * (roll_target - roll) - g.kd_attitude * rates[0])
        tau_y = iyy * (g.kp_attitude * (pitch_target - pitch) - g.kd_attitude * rates[1])
        yaw_error = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
        tau_z = izz * (g.kp_yaw * yaw_error - g.kd_yaw * rates[2])

        # ---- altitude cascade -> collective thrust, compensated for tilt
        climb_target = float(np.clip(g.kp_altitude * (target_altitude - position[2]),
                                     -g.max_climb_rate, g.max_climb_rate))
        accel_z = (g.kp_climb * (climb_target - velocity_w[2])
                   + g.linear_damping * velocity_w[2])
        tilt_cos = max(math.cos(roll) * math.cos(pitch), 0.5)
        thrust_total = p.mass * (GRAVITY + accel_z) / tilt_cos
        thrust_total = float(np.clip(thrust_total, 0.0, 4.0 * p.max_rotor_thrust))

        # ---- mixing: (T, tau) -> per-rotor thrust
        a = p.arm_offset
        c = p.yaw_moment_per_newton
        yaw_term = g.yaw_mix_sign * tau_z / (4.0 * c)
        roll_term = tau_x / (4.0 * a)
        pitch_term = tau_y / (4.0 * a)
        quarter = thrust_total / 4.0
        thrusts = np.array([
            quarter + roll_term - pitch_term + yaw_term,   # m0 front left,  CCW
            quarter - roll_term - pitch_term - yaw_term,   # m1 front right, CW
            quarter + roll_term + pitch_term - yaw_term,   # m2 rear  left,  CW
            quarter - roll_term + pitch_term + yaw_term,   # m3 rear  right, CCW
        ])
        clipped = np.clip(thrusts, 0.0, p.max_rotor_thrust)
        saturated = bool(np.any(np.abs(clipped - thrusts) > 1e-9))

        # ---- invert the propeller model: f = k_t * w^2
        speeds = np.sqrt(clipped / p.thrust_constant)
        for motor, speed, spin in zip(self.motors, speeds, ROTOR_SPIN):
            motor.setVelocity(float(spin * speed))

        self.state = AutopilotState(
            position_w=position, velocity_w=velocity_w, rpy=rpy, rates=rates,
            tilt_target=np.array([roll_target, pitch_target]),
            thrust_total=thrust_total, rotor_thrusts=clipped, saturated=saturated)
        return self.state

    # ------------------------------------------------------------------
    # lidar mount stabilisation
    # ------------------------------------------------------------------
    @staticmethod
    def level_mount_rotation(orientation_world_body) -> list[float]:
        """Mount rotation that levels the sensor, as a Webots axis-angle.

        Takes the body's **world orientation matrix** rather than roll/pitch
        angles, which keeps the result independent of any Euler convention.
        Reconstructing ``R_body`` from ``InertialUnit`` angles would require
        committing to an intrinsic ordering, and ``Rz Ry Rx`` versus ``Rz Rx
        Ry`` differ by a roll-pitch commutator term that vanishes for pure-roll
        and pure-pitch manoeuvres and so would survive single-axis testing.

        The mount is a child of the airframe, so the sensor's world orientation
        is ``R_body @ R_mount``. We want that to equal a *level* frame that
        still yaws with the vehicle::

            x_level = horizontal projection of the body's forward axis
            z_level = world up
            y_level = z_level x x_level

            R_body @ R_mount = R_level   ->   R_mount = R_body^T @ R_level

        Keeping ``x_level`` aligned with the body heading means levelling the
        plane never disturbs the sector-to-bearing mapping.
        """
        R = np.asarray(orientation_world_body, dtype=float).reshape(3, 3)
        forward = R[:, 0]
        horizontal = np.array([forward[0], forward[1], 0.0])
        norm = float(np.linalg.norm(horizontal))
        if not np.all(np.isfinite(R)) or norm < 1e-6:
            # Body forward is vertical: no level frame shares its heading.
            return [0.0, 1.0, 0.0, 0.0]
        x_level = horizontal / norm
        z_level = np.array([0.0, 0.0, 1.0])
        y_level = np.cross(z_level, x_level)
        R_level = np.column_stack((x_level, y_level, z_level))
        R_mount = R.T @ R_level

        # Rotation matrix -> axis-angle.
        trace = float(np.trace(R_mount))
        angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
        if angle < 1e-9:
            return [0.0, 1.0, 0.0, 0.0]
        sin_a = math.sin(angle)
        if abs(sin_a) < 1e-9:
            # angle ~ pi: recover the axis from the symmetric part.
            diagonal = np.clip((np.diag(R_mount) + 1.0) / 2.0, 0.0, None)
            axis = np.sqrt(diagonal)
            dominant = int(np.argmax(axis))
            if axis[dominant] < 1e-9:
                return [0.0, 1.0, 0.0, 0.0]
            axis = axis * math.copysign(1.0, R_mount[
                (dominant + 2) % 3, (dominant + 1) % 3] or 1.0)
            axis = axis / float(np.linalg.norm(axis))
            return [float(axis[0]), float(axis[1]), float(axis[2]), angle]
        axis = np.array([R_mount[2, 1] - R_mount[1, 2],
                         R_mount[0, 2] - R_mount[2, 0],
                         R_mount[1, 0] - R_mount[0, 1]]) / (2.0 * sin_a)
        axis = axis / float(np.linalg.norm(axis))
        return [float(axis[0]), float(axis[1]), float(axis[2]), angle]

    def stabilise_lidar(self, mount_field: Any, drone_node: Any) -> list[float]:
        """Hold the lidar's scan plane horizontal. Returns the rotation written.

        ``mount_field`` is the ``lidarMountRotation`` field of the drone's PROTO
        instance. Writing an exposed PROTO field updates it in place; the
        ``DNS_LIDAR_MOUNT`` node reached through ``getFromProtoDef`` is
        read-only (Webots raises), which ``tests/client_gimbal_probe.py``
        establishes empirically.

        The orientation used is the one Webots reports at the start of the
        current tick, so the mount lags the airframe by one timestep (8 ms).
        The residual is therefore of order ``omega * dt`` and is measured in
        ``results/lidar_attitude.json``.
        """
        rotation = self.level_mount_rotation(drone_node.getOrientation())
        mount_field.setSFRotation(rotation)
        return rotation

    # ------------------------------------------------------------------
    def set_rotor_thrusts(self, thrusts) -> np.ndarray:
        """Drive the rotors directly. Used only by the calibration test."""
        clipped = np.clip(np.asarray(thrusts, dtype=float), 0.0, self.params.max_rotor_thrust)
        speeds = np.sqrt(clipped / self.params.thrust_constant)
        for motor, speed, spin in zip(self.motors, speeds, ROTOR_SPIN):
            motor.setVelocity(float(spin * speed))
        return clipped
