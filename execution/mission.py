"""A stopped handover from a prepared approach route to surveyed parking."""

import json
import logging
import math
from dataclasses import dataclass
from pathlib import Path

from avlite.c40_execution.c43_task_strategy import TaskStrategy
from shapely.geometry import LineString, Point, Polygon

from ..parking.planner import ParkingPlanner, ParkingType, angle_error
from ..settings import PluginSettings
from .parking_task import ParkingTask

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class ParkingMission:
    approach: tuple
    parking_type: str
    spaces: tuple
    wait_timeout: float = 15.0
    progress_timeout: float = 30.0
    settle_time: float = 0.5

    @classmethod
    def from_file(cls, filename):
        data = json.loads(Path(filename).expanduser().read_text())
        if not isinstance(data, dict):
            raise ValueError("Mission must be a JSON object")
        allowed = {
            "version",
            "approach",
            "parking_type",
            "spaces",
            "wait_timeout",
            "progress_timeout",
            "settle_time",
        }
        if data.get("version") != 1 or set(data) - allowed:
            raise ValueError("Mission requires version 1 and supported fields")
        approach = tuple(float(v) for v in data["approach"])
        if len(approach) != 3 or not all(math.isfinite(v) for v in approach):
            raise ValueError("Mission approach must be finite [x, y, yaw]")
        spaces = data["spaces"]
        if (
            not isinstance(spaces, list)
            or not spaces
            or any(not isinstance(s, str) or not s for s in spaces)
            or len(set(spaces)) != len(spaces)
        ):
            raise ValueError("Mission spaces must be unique names in preference order")
        values = {
            k: float(data.get(k, default))
            for k, default in (("wait_timeout", 15), ("progress_timeout", 30), ("settle_time", 0.5))
        }
        if any(not math.isfinite(v) or v <= 0 for v in values.values()):
            raise ValueError("Mission timeouts must be positive finite seconds")
        return cls(approach, ParkingType(data["parking_type"]).value, tuple(spaces), **values)


class ParkingMissionTask(TaskStrategy):
    """Approach, stop, select a free bay, park and hold. Never teleports the vehicle.

    Requires PlanningSyncExecuter. The configured road planner owns the approach;
    this task only changes it after a continuous physical standstill. Timeouts
    latch a stop and require an explicit retry while execution is paused.
    """

    def __init__(self):
        self.parking = ParkingTask()
        self.reset()

    def reset(self):
        self.parking.reset()
        self.spec = None
        self.state, self.reason = "IDLE", "mission_not_configured"
        self.space = None
        self.candidate_results = {}
        self.history = []
        self._world = self._planner = self._route = None
        self._settled = None
        self._last_time = None
        self._progress_pose = None
        self._progress_time = 0.0
        self._state_since = 0.0
        self._retry_state = "APPROACH"
        self._approach_planner = self._approach_controller = None

    def prepare_reset(self, ex):
        if self._approach_planner is not None and self._world is ex.world:
            ex.local_planner = self._approach_planner
            ex.controller = self._approach_controller
            if hasattr(ex.world, "ego_controller"):
                ex.world.ego_controller = ex.controller
            elif hasattr(ex.world, "controller"):
                ex.world.controller = ex.controller

    def _transition(self, ex, state, reason):
        if (state, reason) == (self.state, self.reason):
            return
        if state != self.state:
            self._state_since = float(ex.elapsed_sim_time)
        self.state, self.reason = state, reason
        self.history.append(
            dict(time=float(ex.elapsed_sim_time), state=state, reason=reason, space=self.space)
        )
        del self.history[:-100]
        log.info("Mission %s: %s%s", state, reason, f" ({self.space})" if self.space else "")

    def _block(self, ex, reason):
        if self.state != "BLOCKED":
            self._retry_state = self.state
        self._transition(ex, "BLOCKED", reason)
        ex.hold_motion(self, reason)

    def _poses(self, ex):
        poses = (ex.pm.ego_vehicle, ex.world.get_ego_state())
        for ego in poses:
            if not all(math.isfinite(v) for v in (ego.x, ego.y, ego.theta, ego.velocity)):
                raise ValueError("invalid_localization")
        return poses

    def _load(self, ex):
        self.spec = ParkingMission.from_file(PluginSettings.mission_file)
        if PluginSettings.parking_lidar_occupancy:
            raise ValueError("mission_requires_complete_parking_observation")
        scene = self.parking.parking_spaces()
        if scene.get("demo"):
            raise ValueError("mission_obstacles_must_exist_before_selection")
        for name in self.spec.spaces:
            if (
                name not in scene["spaces"]
                or self.spec.parking_type not in scene["spaces"][name]["types"]
            ):
                raise ValueError(f"unsupported_parking_space:{name}")
        route = ex.local_planner.global_plan
        if LineString(route.path).distance(Point(*self.spec.approach[:2])) > 0.5:
            raise ValueError("approach_not_on_route")
        self._world, self._planner, self._route = ex.world, ex.local_planner, route
        self._approach_planner, self._approach_controller = ex.local_planner, ex.controller
        self._progress_time = float(ex.elapsed_sim_time)
        self._transition(ex, "APPROACH", "following_approach_route")

    def before_step(self, ex, *, tasks_enabled=True):
        """Validate ownership and apply holds before the next control command."""
        if not PluginSettings.mission_file and self.spec is None:
            return
        if not tasks_enabled:
            self._block(ex, "mission_task_disabled")
            return
        if self.state in {"BLOCKED", "CANCELLED", "COMPLETE"}:
            ex.hold_motion(self, self.reason)
            return
        try:
            if self.spec is None:
                self._load(ex)
            now = float(ex.elapsed_sim_time)
            if not math.isfinite(now) or (self._last_time is not None and now < self._last_time):
                raise ValueError("simulation_clock_reversed")
            self._last_time = now
            if (
                ex.world is not self._world
                or ex.local_planner is not self._planner
                or ex.local_planner.global_plan is not self._route
            ):
                raise ValueError("mission_stack_changed")
            poses = self._poses(ex)
            if self.state == "APPROACH":
                ego = poses[0]
                distance = math.hypot(ego.x - self.spec.approach[0], ego.y - self.spec.approach[1])
                if distance <= max(0.3, ego.velocity**2 / 6.0 + 0.2):
                    self._transition(ex, "SETTLING", "braking_at_parking_approach")
                    ex.hold_motion(self, self.reason)
            elif self.state == "PARKING":
                self.parking.execute(ex)
        except (ValueError, KeyError, TypeError, OSError) as exc:
            self._block(ex, str(exc))

    def execute(self, ex, event=None):
        if PluginSettings.mission_file and not callable(getattr(ex, "hold_motion", None)):
            raise ValueError("ParkingMissionTask requires PlanningSyncExecuter")
        if self.state in {"IDLE", "BLOCKED", "CANCELLED", "COMPLETE"}:
            return
        now = float(ex.elapsed_sim_time)
        try:
            if (
                ex.world is not self._world
                or ex.local_planner is not self._planner
                or ex.local_planner.global_plan is not self._route
            ):
                raise ValueError("mission_stack_changed")
            poses = self._poses(ex)
            ego = poses[0]
            if (
                self._progress_pose is None
                or math.hypot(ego.x - self._progress_pose[0], ego.y - self._progress_pose[1])
                >= 0.15
            ):
                self._progress_pose = (ego.x, ego.y)
                self._progress_time = now
            stopped = all(abs(p.velocity) <= 0.02 for p in poses)
            if self.state == "SETTLING":
                if now - self._state_since > self.spec.wait_timeout:
                    self._block(ex, "standstill_timeout")
                    return
                if not stopped:
                    self._settled = None
                    return
                if any(
                    math.hypot(p.x - self.spec.approach[0], p.y - self.spec.approach[1]) > 0.6
                    or abs(angle_error(p.theta - self.spec.approach[2])) > math.radians(10)
                    for p in poses
                ):
                    self._block(ex, "approach_pose_outside_tolerance")
                    return
                if self._settled is None:
                    self._settled = now
                if now - self._settled >= self.spec.settle_time:
                    self._transition(ex, "SELECT_SPACE", "checking_parking_occupancy")
            if self.state == "SELECT_SPACE":
                if not stopped:
                    self._block(ex, "vehicle_moved_during_selection")
                    return
                available = self.parking.available_spaces(ex, self.spec.parking_type)
                failures = []
                self.candidate_results = {}
                for name in self.spec.spaces:
                    if name not in available:
                        self.candidate_results[name] = "not_confirmed_free"
                        continue
                    try:
                        self.parking.start_parking(
                            ex, self.spec.parking_type, name, require_feasible=True
                        )
                    except ValueError as exc:
                        failures.append(f"{name}: {exc}")
                        self.candidate_results[name] = str(exc)
                        continue
                    self.space = name
                    self.candidate_results[name] = "accepted"
                    self._planner = ex.local_planner
                    self._progress_time = now
                    self._transition(ex, "PARKING", "parking_path_accepted")
                    ex.release_motion(self)
                    return
                observation = self.parking.observation(ex)
                if observation is None or observation.polygons is None:
                    reason = "parking_observation_unavailable"
                elif not math.isfinite(observation.stamp):
                    reason = "parking_observation_invalid"
                elif observation.stamp > now:
                    reason = "parking_observation_from_future"
                elif now - observation.stamp > 0.3:
                    reason = "parking_observation_stale"
                else:
                    reason = "no_feasible_parking_path" if failures else "no_confirmed_free_space"
                self._transition(ex, "SELECT_SPACE", reason)
                if now - self._state_since >= self.spec.wait_timeout:
                    self._block(ex, reason + "_timeout")
            elif self.state == "PARKING":
                planner = ex.local_planner
                if planner.state == "PARKED" and stopped:
                    goal = planner.request.goal
                    if any(
                        math.hypot(p.x - goal[0], p.y - goal[1]) > 0.2
                        or abs(angle_error(p.theta - goal[2])) > math.radians(5)
                        or not planner.request.bay.covers(Polygon(p.get_bb_corners()))
                        for p in poses
                    ):
                        self._block(ex, "parking_completion_pose_mismatch")
                        return
                    self._transition(ex, "COMPLETE", "parked_in_selected_space")
                    ex.hold_motion(self, self.reason)
                    return
                if planner.state == "PARKING_CANCELLED":
                    self.cancel(ex)
                    return
                self._transition(ex, "PARKING", planner.reason)
            if (
                self.state in {"APPROACH", "PARKING"}
                and now - self._progress_time > self.spec.progress_timeout
            ):
                self._block(ex, "no_progress_timeout")
        except (ValueError, KeyError, TypeError, OSError) as exc:
            self._block(ex, str(exc))

    def cancel(self, ex):
        if not ex.stopped and not getattr(ex.task_runner, "_in_step", False):
            raise ValueError("Pause execution before cancelling the mission")
        self.parking.cancel_parking(ex)
        self._transition(ex, "CANCELLED", "mission_cancelled")
        ex.hold_motion(self, self.reason)

    def retry(self, ex):
        """Explicit paused retry; a changed stack or cancelled mission needs a reload."""
        if not ex.stopped or self.state != "BLOCKED" or self.spec is None:
            raise ValueError("Pause a blocked, configured mission before retrying")
        if ex.world is not self._world or ex.local_planner is not self._planner:
            raise ValueError("Reload the mission after changing its stack")
        if any(abs(p.velocity) > 0.02 for p in self._poses(ex)):
            raise ValueError("Stop the vehicle before retrying")
        self._settled = None
        self._progress_time = float(ex.elapsed_sim_time)
        target = "PARKING" if isinstance(ex.local_planner, ParkingPlanner) else self._retry_state
        self._transition(ex, target, "operator_retry")
        if target in {"APPROACH", "PARKING"}:
            ex.release_motion(self)
