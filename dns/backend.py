"""Webots implementation of Spencer's ``navigation.Backend`` protocol.

A trained PPO policy drives this exactly as it drives ``planar.PlanarRoom``::

    from dns.launcher import webots_session
    with webots_session():
        from dns.backend import WebotsBackend
        from navigation import Navigation
        env = Navigation(WebotsBackend())
        obs, info = env.reset(seed=12, scene="static")
        obs, reward, terminated, truncated, info = env.step([0.5, -0.25])

Contract, all in the DNS frame (+x forward, +y right, fixed altitude and yaw):

* ``reset(seed, scene)`` places the drone at the scenario start, lets it settle
  and returns a :class:`navigation.Frame` with ``time_s == 0``;
* ``advance(velocity_mps, dt)`` holds ``[vx, vy]`` m/s for exactly ``dt`` of
  simulation time and returns the new frame;
* positions, velocities and ranges are reported in **metres and m/s**, never
  normalised -- ``navigation.observation`` is the only place scaling happens.

Two dynamics modes, and which one produced a frame is always recorded:

``physical``
    The quadcopter flies. Rotor thrust from Webots ``Propeller`` nodes moves a
    rigid body; a cascaded autopilot tracks the commanded velocity at 125 Hz.

``kinematic``
    **Interface test only, not a flight model.** The drone is moved at exactly
    the commanded velocity. Simulation time, obstacle motion, collision
    geometry and sensing all behave normally, so the full PPO contract can be
    exercised independently of flight-control tuning. Every frame produced this
    way is labelled ``dynamics="kinematic"`` in :meth:`WebotsBackend.record`.
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from . import sensing
from .autopilot import Autopilot, Gains, VehicleParams
from .scenario import Scenario, load_default, to_dns, to_webots
from .spencer_link import RANGE, SECTORS, Frame

#: Steps discarded after a supervisor reposition. The Webots GPS derives its
#: speed vector by differencing positions, so the step spanning a teleport
#: reports jump/timestep -- hundreds of m/s -- which would saturate every
#: control loop at once.
TELEPORT_FLUSH_STEPS = 3


class BackendError(RuntimeError):
    """Raised when the simulator cannot honour the contract."""


def matrix_to_axis_angle(R) -> list[float]:
    """3x3 rotation matrix -> Webots ``[ax, ay, az, angle]``."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    angle = math.acos(max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0)))
    if angle < 1e-9:
        return [0.0, 1.0, 0.0, 0.0]
    sin_a = math.sin(angle)
    if abs(sin_a) < 1e-9:                       # angle ~ pi
        diagonal = np.clip((np.diag(R) + 1.0) / 2.0, 0.0, None)
        axis = np.sqrt(diagonal)
        norm = float(np.linalg.norm(axis))
        if norm < 1e-9:
            return [0.0, 1.0, 0.0, 0.0]
        axis = axis / norm
        return [float(axis[0]), float(axis[1]), float(axis[2]), float(angle)]
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    axis = axis / (2.0 * sin_a)
    axis = axis / float(np.linalg.norm(axis))
    return [float(axis[0]), float(axis[1]), float(axis[2]), float(angle)]


@dataclass
class StepRecord:
    """Everything observed around one control step, for logging and handoff."""
    time_s: float
    xy: list
    goal_xy: list
    velocity: list
    commanded_velocity: list
    ranges: list
    valid: list
    collision: bool
    altitude_m: float
    yaw_rad: float
    dynamics: str
    sensing: str
    contact: bool
    lidar_mount: str = "n/a"
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    extras: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "time_s": self.time_s, "xy": self.xy, "goal_xy": self.goal_xy,
            "velocity": self.velocity, "commanded_velocity_mps": self.commanded_velocity,
            "ranges": self.ranges, "valid": self.valid, "collision": self.collision,
            "altitude_m": self.altitude_m, "yaw_rad": self.yaw_rad,
            "roll_rad": self.roll_rad, "pitch_rad": self.pitch_rad,
            "dynamics": self.dynamics, "sensing": self.sensing,
            "lidar_mount": self.lidar_mount,
            "contact": self.contact, **self.extras,
        }


class WebotsBackend:
    """Spencer's ``Backend`` protocol, backed by a running Webots simulation."""

    def __init__(self, scenario: Scenario | None = None, robot: Any = None,
                 gains: Gains | None = None):
        self.scenario = scenario or load_default()
        if robot is None:
            from controller import Supervisor      # imported late: needs the env set
            robot = Supervisor()
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        if self.timestep != self.scenario.basic_timestep_ms:
            raise BackendError(
                f"world basicTimeStep is {self.timestep} ms but the scenario declares "
                f"{self.scenario.basic_timestep_ms} ms; regenerate the world with "
                "python -m dns.worldgen")
        self.steps_per_command = self.scenario.steps_per_command

        self.drone = robot.getFromDef("DNS_DRONE")
        if self.drone is None:
            raise BackendError("world has no DEF DNS_DRONE node")
        self.params = VehicleParams.from_proto(self.drone)
        self.autopilot = Autopilot(robot, self.params, gains or Gains(), self.timestep)

        self.obstacle_nodes = []
        for obstacle in self.scenario.obstacles:
            node = robot.getFromDef(f"DNS_OBSTACLE_{obstacle.id.upper()}")
            if node is None:
                raise BackendError(f"world has no node for obstacle {obstacle.id!r}")
            self.obstacle_nodes.append(node)

        self.lidar = None
        self.lidar_mount_field = None
        if self.scenario.sensing_mode == "lidar":
            self.lidar = robot.getDevice("lidar")
            if self.lidar is None:
                raise BackendError("sensing mode 'lidar' but the drone has no lidar device")
            self.lidar.enable(self.timestep)
            self.lidar_mapping = sensing.LidarMapping.calibrated(
                self.lidar.getHorizontalResolution())
            if self.scenario.lidar_mount_mode == "stabilised":
                self.lidar_mount_field = self.drone.getField("lidarMountRotation")
                if self.lidar_mount_field is None:
                    raise BackendError(
                        "lidar mount mode 'stabilised' needs the DnsQuadcopter PROTO "
                        "field 'lidarMountRotation'; regenerate the world with "
                        "python -m dns.worldgen against the current protos/")

        #: Optional ``callable(backend)`` invoked after every physics step.
        #: Used by the demonstration recorder to grab camera frames from
        #: inside a normal ``advance``; stepping the simulation by hand
        #: alongside ``advance`` would double-advance the clock and trip
        #: Spencer's "Unsynchronised frame" guard.
        self.step_hook = None

        self.scene_camera = None
        self._camera_fields = None
        self._camera_eye_w = None
        self._camera_target_w = None

        self._epoch = 0.0           # robot.getTime() at the start of the episode
        self._steps = 0
        self._commanded = np.zeros(2)
        self._last_record: StepRecord | None = None
        self.records: list[StepRecord] = []

    # ------------------------------------------------------------------
    # geometry helpers
    # ------------------------------------------------------------------
    @property
    def episode_time(self) -> float:
        """Simulation seconds since this episode's reset."""
        return self.robot.getTime() - self._epoch

    def position_dns(self) -> np.ndarray:
        return to_dns(np.asarray(self.drone.getField("translation").getSFVec3f())[:2])

    def altitude(self) -> float:
        return float(self.drone.getField("translation").getSFVec3f()[2])

    def _obstacle_centres(self, t: float) -> np.ndarray:
        return self.scenario.obstacle_centres(t)

    def _move_obstacles(self, t: float) -> None:
        """Drive scripted obstacle motion from the scenario definition.

        Obstacles are kinematic: their pose is a known function of simulation
        time, which keeps a scene exactly reproducible for a given seed.
        """
        for obstacle, node in zip(self.scenario.obstacles, self.obstacle_nodes):
            if obstacle.motion.type == "static":
                continue
            centre = to_webots(obstacle.centre_at(t))
            node.getField("translation").setSFVec3f(
                [float(centre[0]), float(centre[1]), obstacle.height_m / 2.0])

    # ------------------------------------------------------------------
    # demonstration camera
    # ------------------------------------------------------------------
    def attach_scene_camera(self, eye_w, target_w) -> bool:
        """World-lock the optional chase camera to a fixed studio viewpoint.

        The ``Camera`` has to be a child of the robot, so by default it flies
        with the drone. Writing its mount pose with the inverse of the airframe
        pose cancels that: the camera sits still at ``eye_w`` looking at
        ``target_w`` (both in Webots world coordinates) no matter what the
        drone does, which is what makes a trajectory legible on video.

        Returns ``False`` when the world has no ``scene_camera`` device, so a
        caller can fall back without special-casing.
        """
        camera = self.robot.getDevice("scene_camera")
        if camera is None:
            return False
        offset_field = self.drone.getField("sceneCameraOffset")
        rotation_field = self.drone.getField("sceneCameraRotation")
        if offset_field is None or rotation_field is None:
            raise BackendError(
                "the DnsQuadcopter PROTO has no sceneCameraOffset/"
                "sceneCameraRotation fields; regenerate the world")
        camera.enable(self.timestep)
        self.scene_camera = camera
        self._camera_eye_w = np.asarray(eye_w, dtype=float)
        self._camera_target_w = np.asarray(target_w, dtype=float)
        self._camera_fields = (offset_field, rotation_field)
        self._update_scene_camera()
        return True

    def _update_scene_camera(self) -> None:
        """Re-solve the mount pose so the camera stays world-locked."""
        if getattr(self, "_camera_fields", None) is None:
            return
        offset_field, rotation_field = self._camera_fields
        R = np.asarray(self.drone.getOrientation(), dtype=float).reshape(3, 3)
        position = np.asarray(self.drone.getField("translation").getSFVec3f(),
                              dtype=float)
        if not (np.all(np.isfinite(R)) and np.all(np.isfinite(position))):
            return

        # Translation: the eye point expressed in the drone's own frame.
        offset = R.T @ (self._camera_eye_w - position)
        offset_field.setSFVec3f([float(v) for v in offset])

        # Rotation: a Webots camera looks along its local +x with +z up, so
        # build that basis in world axes and pull it back into the body frame.
        forward = self._camera_target_w - self._camera_eye_w
        norm = float(np.linalg.norm(forward))
        if norm < 1e-9:
            return
        forward /= norm
        up = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(up, forward))) > 0.999:
            up = np.array([0.0, 1.0, 0.0])
        right = np.cross(forward, up)
        right /= np.linalg.norm(right)
        cam_up = np.cross(right, forward)
        R_world_cam = np.column_stack((forward, -right, cam_up))
        R_mount = R.T @ R_world_cam
        rotation_field.setSFRotation(matrix_to_axis_angle(R_mount))

    # ------------------------------------------------------------------
    # collision
    # ------------------------------------------------------------------
    def _geometric_collision(self, start_dns, end_dns, t_start, t_end) -> bool:
        """Swept test of the drone's motion against walls and obstacle solids.

        Over one physics step both the drone and a moving obstacle travel in a
        straight line, so their *relative* position is linear in the step
        fraction. Distance to a convex set is a convex function, so distance
        from that linearly-moving point to a fixed convex footprint is convex
        in the fraction and has a single minimum -- which a ternary search
        finds to machine precision.

        This replaces the closest-point-on-segment formula used when every
        obstacle was a disc. That formula has no meaning for a rectangular
        rack, and approximating a rack by its bounding circle would report
        collisions in free aisle space.
        """
        low = np.asarray(self.scenario.low_xy, dtype=float)
        high = np.asarray(self.scenario.high_xy, dtype=float)
        radius = self.scenario.collision_radius_m

        if np.any(end_dns <= low + radius) or np.any(end_dns >= high - radius):
            return True

        obstacles = self.scenario.obstacles
        if not obstacles:
            return False

        start = np.asarray(start_dns, dtype=float)
        end = np.asarray(end_dns, dtype=float)
        travel = end - start

        for obstacle in obstacles:
            begin = obstacle.placed_at(t_start)
            finish = obstacle.placed_at(t_end)
            centre_start = np.asarray(begin.centre)
            centre_shift = np.asarray(finish.centre) - centre_start

            # Cheap exact reject on bounding circles first. This is the old
            # closed-form segment/moving-point test, and it clears a distant
            # obstacle in a few flops. Only a candidate that survives it is
            # worth the iterative exact-shape search, which otherwise costs
            # thousands of distance evaluations per physics step per obstacle.
            relative_start = start - centre_start
            delta = travel - centre_shift
            denominator = float(delta @ delta)
            if denominator > 1e-14:
                fraction = float(np.clip(
                    -(relative_start @ delta) / denominator, 0.0, 1.0))
            else:
                fraction = 0.0
            closest = relative_start + fraction * delta
            if float(np.linalg.norm(closest)) > radius + begin.radius_m:
                continue

            if begin.shape == "cylinder":
                # The bounding circle *is* the shape, so the reject was exact.
                return True

            # Exact shape test. Distance from a linearly-moving point to a
            # fixed convex footprint is convex in the step fraction, so a
            # ternary search converges on the single minimum. 40 iterations
            # shrink the bracket by (2/3)^40, far below micrometre resolution
            # on a step the drone covers in 0.24 m.
            def gap(f: float) -> float:
                point = start + f * travel
                centre = centre_start + f * centre_shift
                moved = dataclasses.replace(
                    begin, centre=(float(centre[0]), float(centre[1])))
                return float(moved.surface_distance(point[None, :])[0])

            lo, hi = 0.0, 1.0
            for _ in range(40):
                a = lo + (hi - lo) / 3.0
                b = hi - (hi - lo) / 3.0
                if gap(a) < gap(b):
                    hi = b
                else:
                    lo = a
            if gap((lo + hi) / 2.0) <= radius:
                return True
        return False

    def _contact_collision(self) -> bool:
        """True while Webots reports physical contact points on the airframe."""
        try:
            return len(self.drone.getContactPoints(True)) > 0
        except Exception:
            return False

    # ------------------------------------------------------------------
    # sensing
    # ------------------------------------------------------------------
    def _scan(self, position_dns: np.ndarray, t: float) -> tuple[np.ndarray, np.ndarray]:
        mode = self.scenario.sensing_mode
        if mode == "none":
            return sensing.unobserved(SECTORS)
        if mode == "geometric":
            return sensing.geometric_scan(
                position_dns, np.asarray(self.scenario.low_xy),
                np.asarray(self.scenario.high_xy), self.scenario.placed_at(t),
                self.scenario.sensing_max_range_m, SECTORS)
        if mode == "lidar":
            image = self.lidar.getRangeImage()
            if image is None:
                return sensing.unobserved(SECTORS)
            return sensing.lidar_scan(image, self.lidar_mapping,
                                      self.scenario.sensing_max_range_m)
        raise BackendError(f"unknown sensing mode {mode!r}")

    # ------------------------------------------------------------------
    # frame construction
    # ------------------------------------------------------------------
    def _measure_velocity(self) -> np.ndarray:
        """Measured planar velocity in the DNS frame, m/s."""
        if self.scenario.dynamics_mode == "kinematic":
            return self._commanded.copy()
        velocity = np.asarray(self.autopilot.gps.getSpeedVector(), dtype=float)
        if not np.all(np.isfinite(velocity)):
            return np.zeros(2)
        return np.array([velocity[0], -velocity[1]])

    def _yaw(self) -> float:
        rpy = self.autopilot.imu.getRollPitchYaw()
        return float(rpy[2]) if np.all(np.isfinite(rpy)) else 0.0

    def _build(self, collision: bool, contact: bool) -> Frame:
        t = self.episode_time
        position = self.position_dns()
        velocity = self._measure_velocity()
        ranges, valid = self._scan(position, t)
        frame = Frame(time_s=t, xy=position, goal_xy=np.asarray(self.scenario.goal_xy,
                                                                dtype=float),
                      velocity=velocity, ranges=ranges, valid=valid, collision=collision)
        self._last_record = StepRecord(
            time_s=t, xy=position.tolist(), goal_xy=list(self.scenario.goal_xy),
            velocity=velocity.tolist(), commanded_velocity=self._commanded.tolist(),
            ranges=ranges.tolist(), valid=[bool(v) for v in valid], collision=collision,
            altitude_m=self.altitude(), yaw_rad=self._yaw(),
            roll_rad=float(self.autopilot.state.rpy[0]),
            pitch_rad=float(self.autopilot.state.rpy[1]),
            dynamics=self.scenario.dynamics_mode, sensing=self.scenario.sensing_mode,
            lidar_mount=(self.scenario.lidar_mount_mode
                         if self.scenario.sensing_mode == "lidar" else "n/a"),
            contact=contact,
            extras={"step": self._steps,
                    "sim_time_s": self.robot.getTime(),
                    "remaining_m": float(np.linalg.norm(
                        np.asarray(self.scenario.goal_xy) - position))})
        self.records.append(self._last_record)
        return frame

    @property
    def record(self) -> StepRecord | None:
        """The record for the most recently produced frame."""
        return self._last_record

    # ------------------------------------------------------------------
    # stepping
    # ------------------------------------------------------------------
    def _step_once(self) -> None:
        # Re-level the scan plane before the physics advances, so the scan
        # Webots renders during this step is taken from a horizontal plane. In
        # kinematic mode the airframe is pinned level and this is a no-op.
        if self.lidar_mount_field is not None:
            self.autopilot.stabilise_lidar(self.lidar_mount_field, self.drone)
        self._update_scene_camera()
        if self.robot.step(self.timestep) == -1:
            raise BackendError("Webots ended the simulation")
        if self.step_hook is not None:
            self.step_hook(self)

    def _pin(self, xy_dns, altitude: float) -> None:
        """Write the drone's pose and clear its momentum."""
        w = to_webots(xy_dns)
        self.drone.getField("translation").setSFVec3f(
            [float(w[0]), float(w[1]), float(altitude)])
        self.drone.getField("rotation").setSFRotation(
            [0.0, 0.0, 1.0, float(self.scenario.fixed_yaw_rad)])
        self.drone.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def _teleport(self, xy_dns, altitude: float) -> None:
        self._pin(xy_dns, altitude)
        self.drone.resetPhysics()
        # Re-pin on every flush step: a reset can land the airframe overlapping
        # a wall it had crashed into, and the contact impulse would otherwise
        # push it off the start point before the episode begins.
        for _ in range(TELEPORT_FLUSH_STEPS):
            self.autopilot.idle()
            self._step_once()
            self._pin(xy_dns, altitude)
        self.autopilot.reset()

    def _settle(self) -> None:
        """Hold a zero-velocity hover until the airframe is steady."""
        if self.scenario.dynamics_mode == "kinematic":
            return
        deadline = self.robot.getTime() + self.scenario.settle_timeout_s
        target = self.scenario.cruise_altitude_m
        while self.robot.getTime() < deadline:
            state = self.autopilot.update(np.zeros(2), target)
            self._step_once()
            speed = float(np.linalg.norm(state.velocity_w))
            if (speed < self.scenario.settle_speed_tol_mps
                    and abs(state.position_w[2] - target) < self.scenario.settle_altitude_tol_m):
                return
        raise BackendError(
            f"drone did not settle within {self.scenario.settle_timeout_s} s "
            f"(altitude {self.autopilot.state.position_w[2]:.3f} m, "
            f"speed {np.linalg.norm(self.autopilot.state.velocity_w):.3f} m/s). "
            "Use dynamics mode 'kinematic' for a flight-control-independent "
            "interface test.")

    # ------------------------------------------------------------------
    # Backend protocol
    # ------------------------------------------------------------------
    def reset(self, seed: int = 0, scene: str = "static") -> Frame:
        """Return the drone to the scenario start and begin a new episode."""
        self.records = []
        self._steps = 0
        self._commanded = np.zeros(2)
        self.scene = scene

        self._move_obstacles(0.0)
        self._teleport(self.scenario.start_xy, self.scenario.cruise_altitude_m)
        self._settle()
        # One more step so the lidar holds a scan taken from the settled pose.
        if self.scenario.dynamics_mode == "physical":
            self.autopilot.update(np.zeros(2), self.scenario.cruise_altitude_m)
        self._step_once()

        self._epoch = self.robot.getTime()
        self._seed = seed
        return self._build(collision=False, contact=self._contact_collision())

    def advance(self, velocity_mps, dt: float) -> Frame:
        """Hold ``[vx, vy]`` for ``dt`` seconds of simulation time."""
        command = np.asarray(velocity_mps, dtype=float)
        if command.shape != (2,) or not np.all(np.isfinite(command)):
            raise BackendError("expected two finite velocity components")
        speed_cap = self.scenario.max_speed_mps
        if np.any(np.abs(command) > speed_cap + 1e-9):
            raise BackendError(
                f"commanded {command.tolist()} m/s exceeds the {speed_cap} m/s per-axis cap")
        steps = int(round(dt * 1000.0 / self.timestep))
        if abs(steps * self.timestep / 1000.0 - dt) > 1e-9:
            raise BackendError(
                f"dt={dt} s is not a whole number of {self.timestep} ms timesteps")

        self._commanded = command
        start_position = self.position_dns()
        start_time = self.episode_time
        collided = False
        contact = False
        target_altitude = self.scenario.cruise_altitude_m

        for _ in range(steps):
            before = self.position_dns()
            before_t = self.episode_time
            if self.scenario.dynamics_mode == "physical":
                self.autopilot.update(command, target_altitude)
                self._step_once()
            else:
                # Labelled interface-test mode: integrate the command exactly.
                # The pose is re-asserted after the step so gravity and contact
                # forces cannot perturb it -- this is deliberately not a flight
                # model, and every frame it produces is labelled "kinematic".
                nxt = before + command * (self.timestep / 1000.0)
                self._step_once()
                self._pin(nxt, target_altitude)
            self._move_obstacles(self.episode_time)
            collided |= self._geometric_collision(before, self.position_dns(),
                                                  before_t, self.episode_time)
            contact |= self._contact_collision()

        self._steps += 1
        mode = self.scenario.collision_mode
        reported = {"geometric": collided, "physical": contact,
                    "either": collided or contact}[mode]
        _ = start_position, start_time
        return self._build(collision=reported, contact=contact)


def make_backend(scenario: Scenario | None = None, **kwargs) -> WebotsBackend:
    """Factory matching ``evaluate.episode(..., backend_factory=...)``."""
    return WebotsBackend(scenario, **kwargs)
