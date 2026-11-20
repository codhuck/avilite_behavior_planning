"""Selectable low-speed parking with explicit geometry and obstacle inputs."""

import math
from dataclasses import dataclass
from enum import Enum

import numpy as np
import shapely
from avlite.c20_planning.c21_planning_model import LocalPlan
from avlite.c20_planning.c23_local_planning_strategy import LocalPlanningStrategy
from avlite.c30_control.c32_control_strategy import ControlStrategy
from avlite.c50_common.c51_capabilities import StackCapability
from shapely.geometry import Polygon

from ..execution.commands import DirectionalAckermannCommand


def angle_error(value):
    return (value + math.pi) % (2 * math.pi) - math.pi


def footprint(pose, length, width, margin=0.0):
    """Body-centered rectangle, using AVLite's ego pose convention."""
    x, y, yaw = pose
    corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])
    corners = corners * [length / 2 + margin, width / 2 + margin]
    c, s = math.cos(yaw), math.sin(yaw)
    return Polygon(corners @ np.array([[c, s], [-s, c]]) + [x, y])


class ParkingType(str, Enum):
    PERPENDICULAR_FORWARD = "perpendicular_forward"
    PERPENDICULAR_REVERSE = "perpendicular_reverse"
    PARALLEL = "parallel"


@dataclass(frozen=True)
class ParkingRequest:
    """Known bay and drivable area; all coordinates are in the localization frame."""

    goal: tuple[float, float, float]
    bay: Polygon
    drivable: Polygon
    parking_type: ParkingType = ParkingType.PERPENDICULAR_FORWARD

    def __post_init__(self):
        object.__setattr__(self, "parking_type", ParkingType(self.parking_type))
        if len(self.goal) != 3 or not np.isfinite(self.goal).all():
            raise ValueError("Parking goal must be a finite x, y, yaw pose")
        for shape in (self.bay, self.drivable):
            if shape.is_empty or not shape.is_valid or shape.area <= 0:
                raise ValueError("Parking areas must be valid nonempty polygons")


class ParkingPlanner(LocalPlanningStrategy):
    """Explicit parking manoeuvre with fresh occupancy and bounded tracking error."""

    stack_requirements = frozenset({StackCapability.LOCALIZATION})

    def __init__(self, global_plan, pm=None, env=None):
        pm = pm if pm is not None else env
        if pm is None:
            raise ValueError("Parking requires a perception model")
        super().__init__(global_plan, pm)
        self.rear_offset = 0.0
        self.sensor_occupancy = None
        self.sensor_ground_z = None
        self.request = None
        self.poses = None
        self.state = "PARKING_IDLE"
        self.reason = "no_request"
        self.obstacles = None
        self._observed_time = None
        self._plan = LocalPlan()

    def request_parking(self, request):
        if (
            not math.isfinite(self.pm.ego_vehicle.velocity)
            or abs(self.pm.ego_vehicle.velocity) > 0.05
        ):
            raise ValueError("Start parking only while stopped")
        self.request = request
        self.poses = None
        self.state, self.reason = "PARKING_PLAN", "requested"
        self._progress = 0
        self._terminal = False
        self._segments = []
        self._segment = 0
        self._shift_since = None
        self._attempted_revision = None
        self._search_time = None

    def update_obstacles(self, polygons, simulation_time):
        """Explicit observation snapshot; None is unknown, [] is observed clear."""
        if not math.isfinite(simulation_time):
            raise ValueError("Observation time must be finite")
        if polygons is not None:
            polygons = list(polygons)
            if any(p.is_empty or not p.is_valid for p in polygons):
                raise ValueError("Invalid obstacle polygon")
        fingerprint = None if polygons is None else tuple(p.wkb for p in polygons)
        if fingerprint != getattr(self, "_obstacle_fingerprint", None):
            self._revision = getattr(self, "_revision", 0) + 1
        self._obstacle_fingerprint = fingerprint
        self.obstacles = polygons
        self._observed_time = simulation_time

    def _clear(self, poses, margin=0.15):
        ego = self.pm.ego_vehicle
        # Dense samples plus 0.15 m inflation cover the <=0.05 m sampling gap.
        poses = np.asarray(poses)
        corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])
        corners = corners * [ego.length / 2 + margin, ego.width / 2 + margin]
        c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
        x = poses[:, 0, None] + c[:, None] * corners[:, 0] - s[:, None] * corners[:, 1]
        y = poses[:, 1, None] + s[:, None] * corners[:, 0] + c[:, None] * corners[:, 1]
        bodies = shapely.polygons(np.stack([x, y], axis=-1))
        if not np.all(shapely.covers(self.request.drivable, bodies)):
            return False
        return not any(np.any(shapely.intersects(bodies, p)) for p in self.obstacles)

    @property
    def direction(self):
        if getattr(self, "_segments", None):
            return self._segments[self._segment][0]
        return (
            1
            if self.request is None
            or self.request.parking_type == ParkingType.PERPENDICULAR_FORWARD
            else -1
        )

    def cancel(self):
        """Hold a stop until a new explicit request; do not resume on clear sensors."""
        self._terminal = True
        self.state, self.reason = "PARKING_CANCELLED", "user_cancelled"

    def set_global_plan(self, global_plan, ego_xy=None):
        super().set_global_plan(global_plan, ego_xy)
        self.reset()

    def _generate(self):
        ego = self.pm.ego_vehicle
        yaw = self.request.goal[2]
        start = np.array([ego.x, ego.y]) - self.rear_offset * np.array(
            [math.cos(ego.theta), math.sin(ego.theta)]
        )
        goal = np.array(self.request.goal[:2]) - self.rear_offset * np.array(
            [math.cos(yaw), math.sin(yaw)]
        )
        reverse_angle = math.pi if self.direction < 0 else 0.0
        travel_yaw = yaw + reverse_angle
        direction = np.array([math.cos(travel_yaw), math.sin(travel_yaw)])
        if not self.request.bay.covers(footprint(self.request.goal, ego.length, ego.width, 0.15)):
            return None
        candidates = []
        alignment = (
            (0.0, 0.5, 1.0) if self.request.parking_type == ParkingType.PARALLEL else (3.0, 1.5)
        )
        for straight in alignment:
            entry = goal - straight * direction
            distance = np.linalg.norm(entry - start)
            if distance < 0.5 or distance > 40:
                continue
            for scale in (0.25, 0.35, 0.5, 0.7, 0.9):
                for end_scale in (0.35, 0.5, 0.7):
                    control = np.array(
                        [
                            start,
                            start
                            + distance
                            * scale
                            * np.array(
                                [
                                    math.cos(ego.theta + reverse_angle),
                                    math.sin(ego.theta + reverse_angle),
                                ]
                            ),
                            entry - distance * end_scale * direction,
                            entry,
                        ]
                    )
                    t = np.linspace(0, 1, max(200, int(distance * 100)))[:, None]
                    curve = (
                        (1 - t) ** 3 * control[0]
                        + 3 * (1 - t) ** 2 * t * control[1]
                        + 3 * (1 - t) * t**2 * control[2]
                        + t**3 * control[3]
                    )
                    points = curve
                    if straight:
                        points = np.vstack(
                            [
                                curve,
                                entry
                                + np.linspace(0.01, straight, max(2, int(straight * 100)))[:, None]
                                * direction,
                            ]
                        )
                    delta = np.diff(points, axis=0)
                    ds = np.linalg.norm(delta, axis=1)
                    if np.any(ds < 1e-6) or np.max(ds) > 0.05:
                        continue
                    headings = np.unwrap(np.arctan2(delta[:, 1], delta[:, 0]))
                    curvature = np.diff(headings) / ((ds[:-1] + ds[1:]) / 2)
                    if np.max(np.abs(curvature)) > 0.20:
                        continue
                    body_headings = np.r_[headings - reverse_angle, yaw]
                    centers = points + self.rear_offset * np.column_stack(
                        [np.cos(body_headings), np.sin(body_headings)]
                    )
                    poses = np.column_stack([centers, body_headings])
                    if self._clear(poses, margin=0.35 if self.rear_offset else 0.15):
                        candidates.append((float(ds.sum()), poses))
        if candidates:
            poses = min(candidates, key=lambda item: item[0])[1]
            self._segments = [(self.direction, poses)]
            return poses
        if self.request.parking_type == ParkingType.PARALLEL or getattr(
            self, "allow_multi_maneuver", False
        ):
            from .search import search

            # Search outward from the constrained bay, then reverse the motion.
            # This avoids spending the search budget on the open approach aisle.
            outward, reason = search(
                (*goal, yaw),
                (*start, ego.theta),
                ego.length,
                ego.width,
                self.request.drivable,
                self.obstacles,
                rear_offset=self.rear_offset,
                clearance=max(
                    0.35 if self.rear_offset else 0.18, getattr(self, "search_clearance", 0)
                ),
                final_direction=1,
                max_nodes=50000,
                timeout=None
                if getattr(self, "deterministic_search", False)
                else getattr(self, "search_timeout", 10),
                max_expansions=getattr(self, "search_expansions", 1500)
                if getattr(self, "deterministic_search", False)
                else None,
                position_tolerance=getattr(self, "search_position_tolerance", 0.18),
                refine_goal=getattr(self, "refine_search_endpoint", False),
                initial_direction=(
                    -1 if self.request.parking_type == ParkingType.PERPENDICULAR_FORWARD else 1
                )
                if getattr(self, "allow_multi_maneuver", False)
                and self.request.parking_type != ParkingType.PARALLEL
                else None,
            )
            segments = None
            if outward is not None:
                segments = []
                for direction, points in reversed(outward):
                    body = points[::-1].copy()
                    body[:, :2] += self.rear_offset * np.column_stack(
                        [np.cos(body[:, 2]), np.sin(body[:, 2])]
                    )
                    segments.append((-direction, body))
            self.reason = reason
            if segments:
                self._segments = segments
                return segments[0][1]
        return None

    def replan(self, perception_model=None, sensors=None):
        if perception_model is not None:
            self.pm = perception_model
        ego = self.pm.ego_vehicle
        now = getattr(self.pm, "simulation_time", None)
        target_speed = 0.0
        if self.sensor_occupancy is not None and self.request is not None:
            frame_time = getattr(sensors, "stamp", None)
            cloud = getattr(sensors, "lidar", None)
            if getattr(self.sensor_occupancy, "multi_sensor", False):
                named = getattr(sensors, "lidars", {})
                if named:
                    cloud = tuple(named.values())
            observation = self.sensor_occupancy.observe(
                cloud,
                ego,
                self.request.drivable,
                frame_time,
                self.sensor_ground_z,
            )
            self.pm.parking_occupancy = observation
            sensor_key = (frame_time, observation.stamp)
            previous_key = getattr(self, "_sensor_key", None)
            out_of_order = (
                observation.stamp is not None
                and previous_key is not None
                and previous_key[1] is not None
                and observation.stamp < previous_key[1]
            )
            if observation.status == "observed" and not out_of_order and sensor_key != previous_key:
                self._sensor_key = sensor_key
                self._sensor_received_at = now
            if observation.status == "observed" and now is not None and not out_of_order:
                age = frame_time - observation.stamp
                obstacles = [] if observation.blocked.is_empty else [observation.blocked]
                self.update_obstacles(obstacles, self._sensor_received_at - age)
            elif now is not None:
                self.update_obstacles(None, now)
        if getattr(self, "_terminal", False):
            pass
        elif not np.isfinite([ego.x, ego.y, ego.theta, ego.velocity]).all():
            self.state, self.reason = "PARKING_BLOCKED", "invalid_localization"
        elif self.request is None:
            self.state, self.reason = "PARKING_IDLE", "no_request"
        elif (
            self.obstacles is None
            or now is None
            or self._observed_time is None
            or not 0 <= now - self._observed_time <= 0.3
        ):
            self.state, self.reason = "PARKING_BLOCKED", "unknown_or_stale_obstacles"
        else:
            revision = getattr(self, "_revision", 0)
            if (
                self.poses is None
                and getattr(self, "_attempted_revision", None) != revision
                and not getattr(self, "_suspend_parking_search", False)
                and (
                    getattr(self, "_search_time", None) is None
                    or now < self._search_time
                    or now - self._search_time >= getattr(self, "search_retry_interval", 0.0)
                )
            ):
                self._search_time = now
                self._attempted_revision = revision
                self._progress = 0
                self._segment = 0
                self._segments = []
                self._shift_since = None
                self.poses = self._generate()
            if self.poses is None:
                self.state, self.reason = (
                    "PARKING_BLOCKED",
                    "no_feasible_forward_path"
                    if self.request.parking_type == ParkingType.PERPENDICULAR_FORWARD
                    else "no_feasible_path_within_budget",
                )
            else:
                previous = getattr(self, "_progress", 0)
                end = min(len(self.poses), previous + 200)
                idx = previous + int(
                    np.argmin(np.linalg.norm(self.poses[previous:end, :2] - [ego.x, ego.y], axis=1))
                )
                self._progress = idx
                error = float(np.linalg.norm(self.poses[idx, :2] - [ego.x, ego.y]))
                remaining = np.sum(np.linalg.norm(np.diff(self.poses[idx:, :2], axis=0), axis=1))
                endpoint = self.poses[-1]
                distance = math.hypot(ego.x - endpoint[0], ego.y - endpoint[1])
                heading = abs(angle_error(ego.theta - endpoint[2]))
                clear = self._clear([(ego.x, ego.y, ego.theta)]) and self._clear(self.poses[idx:])
                if not getattr(self, "defer_future_visibility", False):
                    clear = clear and all(
                        self._clear(p) for _, p in self._segments[self._segment + 1 :]
                    )
                if not clear:
                    self.state, self.reason = "PARKING_BLOCKED", "obstacle_or_boundary"
                    self._shift_since = None
                elif error > 0.4:
                    self.state, self.reason = "PARKING_BLOCKED", "tracking_error"
                elif (
                    self._segment + 1 >= len(self._segments)
                    and idx >= len(self.poses) - 2
                    and math.hypot(ego.x - self.request.goal[0], ego.y - self.request.goal[1])
                    < 0.20
                    and abs(angle_error(ego.theta - self.request.goal[2])) < math.radians(5)
                    and self.request.bay.covers(
                        footprint((ego.x, ego.y, ego.theta), ego.length, ego.width)
                    )
                ):
                    # The final goal has its own existing 20 cm / 5 degree
                    # contract. A terminal waypoint's 10 cm switching gate must
                    # not reject a valid stopped goal because of lateral error.
                    self.state = "PARKED" if abs(ego.velocity) < 0.02 else "PARKING_STOP"
                    self.reason = "goal_reached"
                    self._terminal = self.state == "PARKED"
                elif distance < 0.10 and heading < math.radians(5):
                    if self._segment + 1 < len(self._segments):
                        self.state, self.reason = "PARKING_SHIFT", "stop_before_direction_change"
                        if abs(ego.velocity) < 0.02:
                            if self._shift_since is None:
                                self._shift_since = now
                            if now - self._shift_since >= 0.5:
                                self._segment += 1
                                self.poses = self._segments[self._segment][1]
                                self._progress = 0
                                self._shift_since = None
                        else:
                            self._shift_since = None
                    else:
                        goal_distance = math.hypot(
                            ego.x - self.request.goal[0], ego.y - self.request.goal[1]
                        )
                        body = footprint((ego.x, ego.y, ego.theta), ego.length, ego.width)
                        if (
                            goal_distance < 0.20
                            and abs(angle_error(ego.theta - self.request.goal[2])) < math.radians(5)
                            and self.request.bay.covers(body)
                        ):
                            self.state = "PARKED" if abs(ego.velocity) < 0.02 else "PARKING_STOP"
                            self.reason = "goal_reached"
                            self._terminal = self.state == "PARKED"
                        else:
                            self.state, self.reason = "PARKING_BLOCKED", "goal_pose_not_reached"
                elif idx >= len(self.poses) - 2:
                    self.state, self.reason = "PARKING_BLOCKED", "goal_pose_not_reached"
                else:
                    self.state, self.reason = (
                        "PARKING_APPROACH",
                        "following_" + self.request.parking_type.value,
                    )
                    target_speed = min(
                        0.4 if self.rear_offset else 0.6, remaining * 0.7, distance * 0.7
                    )
        if (
            self.state == "PARKING_BLOCKED"
            and self.reason in ("goal_pose_not_reached", "tracking_error")
            and abs(ego.velocity) < 0.02
            and not getattr(self, "_suspend_parking_search", False)
        ):
            # A missed endpoint cannot be repaired by retaining the exhausted
            # path forever. Stop first, then retry from the measured pose with
            # the normal search budget and fresh collision/visibility checks.
            self.poses = None
            self._segments = []
            self._segment = self._progress = 0
            self._attempted_revision = None
            self._shift_since = None
            self._search_time = now
        poses = self.poses
        if poses is None:
            poses = np.array([[ego.x, ego.y, ego.theta], [ego.x + 0.01, ego.y, ego.theta]])
        self._plan = LocalPlan(
            path=[tuple(p[:2]) for p in poses], velocity=[target_speed] * len(poses)
        )
        self._plan.parking_poses = poses
        self._plan.parking_direction = self.direction
        self._plan.parking_progress = getattr(self, "_progress", 0)
        self._plan.behavior_state = self.state
        self._plan.behavior_reason = self.reason
        return self._plan

    def get_local_plan(self):
        return self._plan

    def reset(self, wp=0):
        self._sensor_key = None
        self._sensor_received_at = None
        self._terminal = False
        self._progress = 0
        self._segments = []
        self._segment = 0
        self._shift_since = None
        self._attempted_revision = None
        self._search_time = None
        self.request = None
        self.poses = None
        self.obstacles = None
        self._observed_time = None
        self.state, self.reason = "PARKING_IDLE", "no_request"
        self._plan = LocalPlan()


class ParkingController(ControlStrategy):
    """Low-speed tracking with explicit drive direction and wheel-angle output."""

    def __init__(self, tj=None):
        super().__init__(tj)
        self.ego_max_acceleration = 3.0
        self.ego_min_acceleration = -6.0
        self.rear_offset = 0.0
        self.reset()

    def control(self, ego, plan=None, control_dt=None, perception_model=None, sensors=None):
        dt = 0.05 if control_dt is None else control_dt
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Control timestep must be positive")
        poses = getattr(plan, "parking_poses", None)
        direction = getattr(plan, "parking_direction", 1)
        steer = target = 0.0
        valid = np.isfinite([ego.x, ego.y, ego.theta, ego.velocity]).all()
        longitudinal = getattr(ego, "longitudinal_velocity", None)
        valid = valid and (longitudinal is None or math.isfinite(longitudinal))
        if (
            valid
            and poses is not None
            and len(poses) >= 2
            and np.isfinite(poses).all()
            and np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1).sum() > 1e-6
        ):
            idx = min(getattr(plan, "parking_progress", 0), len(poses) - 1)
            target = plan.velocity[idx]
            if self.rear_offset:
                rear = np.array(
                    [
                        ego.x - self.rear_offset * math.cos(ego.theta),
                        ego.y - self.rear_offset * math.sin(ego.theta),
                    ]
                )
                reference = poses[:, :2] - self.rear_offset * np.column_stack(
                    [np.cos(poses[:, 2]), np.sin(poses[:, 2])]
                )
                lengths = np.r_[
                    0, np.cumsum(np.linalg.norm(np.diff(reference[idx:], axis=0), axis=1))
                ]
                look = idx + min(int(np.searchsorted(lengths, 0.8)), len(lengths) - 1)
                delta = reference[look] - rear
                travel_heading = ego.theta + (math.pi if direction < 0 else 0)
                alpha = angle_error(math.atan2(delta[1], delta[0]) - travel_heading)
                steer = math.atan(
                    direction
                    * self.ego_distance_front_axle
                    * 2
                    * math.sin(alpha)
                    / max(0.3, np.linalg.norm(delta))
                )
            else:
                near = min(idx + 5, len(poses) - 1)
                span = np.linalg.norm(poses[near, :2] - poses[idx, :2])
                curvature = angle_error(poses[near, 2] - poses[idx, 2]) / max(span, 1e-3)
                yaw = poses[idx, 2]
                lateral = -(ego.x - poses[idx, 0]) * math.sin(yaw) + (
                    ego.y - poses[idx, 1]
                ) * math.cos(yaw)
                steer = (
                    math.atan(direction * self.ego_distance_front_axle * curvature)
                    + direction * 1.5 * angle_error(yaw - ego.theta)
                    - 0.8 * lateral
                )
        speed = abs(ego.velocity) if math.isfinite(ego.velocity) else 0.0
        progress_speed = direction * longitudinal if longitudinal is not None and valid else speed
        if target <= 0:
            self._integral = 0.0
            acceleration = -max(0.5, 2 * speed)
        else:
            if progress_speed > target + 0.05:
                self._integral = min(self._integral, 0.0)
            integral_limit = 0.5 if self.rear_offset else 1.5
            self._integral = float(
                np.clip(self._integral + (target - progress_speed) * dt, -1, integral_limit)
            )
            measured_acceleration = (
                0.0 if self._last_speed is None else (progress_speed - self._last_speed) / dt
            )
            damping = 0.5 * measured_acceleration if self.rear_offset else 0.0
            acceleration = float(
                np.clip(3 * (target - progress_speed) + self._integral - damping, -1.5, 2.5)
            )
        self._last_speed = progress_speed
        acceleration = max(acceleration, -speed / dt)
        return DirectionalAckermannCommand(
            steer=float(np.clip(steer, -0.6, 0.6)),
            acceleration=acceleration,
            direction=direction,
            hold_brake=target <= 0,
        )

    def reset(self):
        self._integral = 0.0
        self._last_speed = None
