"""AVLite parking adapter for bounded active observation manoeuvres."""

import math

import numpy as np
from avlite.c20_planning.c21_planning_model import LocalPlan
from shapely.geometry import GeometryCollection

from .dispatch import Maneuver, VisibilityDispatcher, VisibilitySnapshot, expected_gain
from .planner import ParkingPlanner
from .scan import ScanVisibility
from .view import TaskVisibility, beam_gain


class VisibilityParkingPlanner(ParkingPlanner):
    """Execute short checked straight/curved views before retrying the requested parking.

    No change of the user's parking type occurs implicitly. Observation paths
    are fixed until completion; each remaining path is checked every replan.
    Unknown geometry can still prevent all motion, which is a valid refusal.
    """

    def __init__(self, global_plan, pm=None, env=None):
        super().__init__(global_plan, pm, env)
        self.arbiter = VisibilityDispatcher(allow_prefix=True, adaptive_speed=True)
        self.allow_observation = True
        # Opt-in research variants; historical default behaviour remains explicit.
        self.view_objective = "legacy"
        self.view_lidar_geometry = ()
        self.view_scan_parameters = {}
        self.view_min_advantage = 0.01
        self.deterministic_search = False
        self.search_expansions = 1500
        self.recovery_search_expansions = 6000
        self.allow_multi_maneuver = True
        self.defer_future_visibility = True
        self.search_clearance = self.arbiter.margin + self.arbiter.unseen_speed * (
            self.arbiter.latency + 0.05 / self.arbiter.deceleration
        )
        self.refine_search_endpoint = True
        self._view = None
        self.cruise_speed = 0.4
        # Noisy clouds change geometry every frame. Retry hypotheses at a bounded
        # rate; fresh occupancy and command arbitration still run on every tick.
        self.search_retry_interval = 2.0
        self.search_timeout = 2.0
        self.recovery_search_timeout = 5.0
        self._search_failures = 0

    def _generate(self):
        """Generate a geometric hypothesis; arbitration alone releases motion.

        Unknown space is allowed only in the proposed future path. Every actual
        command is checked against current observations by the dispatcher.
        """
        observation = getattr(self.pm, "parking_occupancy", None)
        if observation is None or observation.status != "observed":
            return None
        saved = self.obstacles
        timeout = self.search_timeout
        expansions = getattr(self, "search_expansions", 1500)
        if self._search_failures >= 2 and abs(self.pm.ego_vehicle.velocity) < 0.02:
            self.search_timeout = max(timeout, self.recovery_search_timeout)
            if getattr(self, "deterministic_search", False):
                self.search_expansions = max(expansions, self.recovery_search_expansions)
        occupied = getattr(observation, "occupied", None)
        self.obstacles = [] if occupied is None or occupied.is_empty else [occupied]
        try:
            poses = super()._generate()
            self._search_failures = self._search_failures + 1 if poses is None else 0
            return poses
        finally:
            self.obstacles = saved
            self.search_timeout = timeout
            self.search_expansions = expansions

    def request_parking(self, request):
        super().request_parking(request)
        self.arbiter.reset()
        self._view = None
        self._search_failures = 0

    def reset(self, wp=0):
        super().reset(wp)
        if hasattr(self, "arbiter"):
            self.arbiter.reset()
        self._view = None
        self._search_failures = 0

    def _clear(self, poses, margin=0.15):
        # Geometric proposals use the minimum checked creep speed. Arbitration
        # applies the larger actual/requested-speed envelope on every cycle.
        minimum = self.arbiter.margin + self.arbiter.unseen_speed * (
            self.arbiter.latency + 0.05 / self.arbiter.deceleration
        )
        # The base planner checks geometry of a hypothesis. Raster visibility
        # is checked exactly once by arbitration before any motion is released.
        observation = getattr(self.pm, "parking_occupancy", None)
        occupied = getattr(observation, "occupied", None)
        saved = self.obstacles
        self.obstacles = [] if occupied is None or occupied.is_empty else [occupied]
        try:
            return super()._clear(poses, max(margin, minimum))
        finally:
            self.obstacles = saved

    def _output(self, poses, direction, speed, reason, progress=0):
        self.state = "PARKING_OBSERVE" if speed > 0 else "PARKING_BLOCKED"
        self.reason = reason
        plan = LocalPlan(path=[tuple(p[:2]) for p in poses], velocity=[speed] * len(poses))
        plan.parking_poses = poses
        plan.parking_direction = direction
        plan.parking_progress = progress
        plan.behavior_state, plan.behavior_reason = self.state, reason
        self._plan = plan
        return plan

    def replan(self, perception_model=None, sensors=None):
        if self.pm is not None:
            self.pm.parking_arbitration = None
        self._suspend_parking_search = self._view is not None
        plan = super().replan(perception_model, sensors)
        self.base_reason = self.reason
        if self.request is None or getattr(self, "_terminal", False):
            return plan
        observation = getattr(self.pm, "parking_occupancy", None)
        now = getattr(self.pm, "simulation_time", None)
        if observation is None or observation.status != "observed" or self._observed_time is None:
            plan.velocity = [0.0] * len(plan.velocity)
            plan.behavior_state = self.state = "PARKING_BLOCKED"
            plan.behavior_reason = self.reason = "missing_visibility_observation"
            return plan
        occupied = getattr(observation, "occupied", None)
        snapshot = VisibilitySnapshot(
            getattr(observation, "traversable", observation.free),
            occupied if occupied is not None else GeometryCollection(),
            self._observed_time,
            getattr(observation, "sources", ()),
            getattr(self.sensor_occupancy, "minimum_support", 1),
            getattr(self.sensor_occupancy, "hazard_support_radius", 0.0),
        )
        ego = self.pm.ego_vehicle
        pose = np.array([ego.x, ego.y, ego.theta])
        allowed = {self.request.parking_type.value}
        candidates = []
        if self._view is not None:
            view = self._view
            idx = int(np.argmin(np.linalg.norm(view.poses[:, :2] - pose[:2], axis=1)))
            if np.linalg.norm(view.poses[-1, :2] - pose[:2]) < 0.1:
                if abs(ego.velocity) < 0.02:
                    self.arbiter.complete_view(view.poses[-1])
                    self._view = None
                    aligned = False
                    if self.poses is not None:
                        nearest = int(
                            np.argmin(np.linalg.norm(self.poses[:, :2] - pose[:2], axis=1))
                        )
                        error = self.poses[nearest] - pose
                        aligned = np.linalg.norm(error[:2]) < 0.15 and abs(
                            math.atan2(math.sin(error[2]), math.cos(error[2]))
                        ) < math.radians(5)
                        if aligned:
                            self._progress = nearest
                    if not aligned:
                        self.poses = None
                        self._attempted_revision = None
                return self._output(view.poses, view.direction, 0, "view_complete_stop", idx)
            remaining = view.poses[idx:].copy()
            if len(remaining) > 1:
                speed = min(self.cruise_speed, np.linalg.norm(remaining[-1, :2] - pose[:2]) * 0.5)
                candidates = [
                    Maneuver(
                        view.name,
                        "observe",
                        view.parking_type,
                        remaining,
                        view.direction,
                        speed,
                        view.gain,
                        rear_offset=self.rear_offset,
                    )
                ]
        elif self.poses is not None and (
            self.state == "PARKING_APPROACH" or self.reason == "obstacle_or_boundary"
        ):
            idx = getattr(self, "_progress", 0)
            poses = self.poses[idx:].copy()
            if len(poses) > 1:
                candidates = [
                    Maneuver(
                        "requested",
                        "park",
                        self.request.parking_type.value,
                        poses,
                        self.direction,
                        min(self.cruise_speed, max(plan.velocity))
                        if max(plan.velocity) > 0
                        else self.cruise_speed,
                        rear_offset=self.rear_offset,
                    )
                ]
        if (
            self.allow_observation
            and self._view is None
            and abs(ego.velocity) < 0.02
            and self.state in ("PARKING_BLOCKED", "PARKING_APPROACH")
        ):
            objective = getattr(self, "view_objective", "legacy")
            if objective not in ("legacy", "task", "task_beams", "task_scan"):
                raise ValueError("Unknown observation objective")
            task = None
            if objective in ("task", "task_beams", "task_scan") and self.poses is not None:
                remaining = self.poses[getattr(self, "_progress", 0) :]
                if len(remaining) > 1:
                    task = TaskVisibility(snapshot, remaining, ego.width, ego.length)
            scan = None
            if objective == "task_scan" and task and self.view_lidar_geometry:
                scan = ScanVisibility(
                    task,
                    self.view_lidar_geometry,
                    minimum_support=snapshot.minimum_support,
                    hazard_support_radius=snapshot.hazard_support_radius,
                    body_length=ego.length,
                    body_width=ego.width,
                    **self.view_scan_parameters,
                )
            baseline = expected_gain(pose, snapshot, self.request.bay)
            for direction in (1, -1):
                for distance in (0.25, 0.5, 1.0, 2.5):
                    offset = np.linspace(0, distance, int(distance / 0.04) + 1) * direction
                    for curvature in (0.0, -0.2, 0.2):
                        yaw = ego.theta + offset * curvature
                        rear_x = ego.x - self.rear_offset * math.cos(ego.theta)
                        rear_y = ego.y - self.rear_offset * math.sin(ego.theta)
                        if curvature:
                            x = rear_x + (np.sin(yaw) - math.sin(ego.theta)) / curvature
                            y = rear_y - (np.cos(yaw) - math.cos(ego.theta)) / curvature
                        else:
                            x = rear_x + offset * math.cos(ego.theta)
                            y = rear_y + offset * math.sin(ego.theta)
                        poses = np.column_stack(
                            (
                                x + self.rear_offset * np.cos(yaw),
                                y + self.rear_offset * np.sin(yaw),
                                yaw,
                            )
                        )
                        gain = expected_gain(poses[-1], snapshot, self.request.bay) - baseline
                        # A local reposition can make the geometric parking
                        # problem feasible even when coarse visibility samples
                        # predict no new cells. Reward measured goal progress
                        # as well; this affects ranking, never the safety gate.
                        # Respect a multi-point parking manoeuvre's current
                        # staging endpoint, which may lie away from the bay.
                        goal = (
                            self.poses[-1, :2]
                            if self.poses is not None
                            else np.asarray(self.request.goal[:2])
                        )
                        progress = np.linalg.norm(pose[:2] - goal) - np.linalg.norm(
                            poses[-1, :2] - goal
                        )
                        gain = max(0.0, gain) + progress
                        if objective in ("task", "task_beams", "task_scan"):
                            # No guessed route when the geometric planner has no hypothesis.
                            gain = task.evaluate(pose, poses[-1]).weighted_gain if task else 0.0
                            if objective == "task_beams":
                                geometry = getattr(self, "view_lidar_geometry", ())
                                gain = (
                                    beam_gain(
                                        task,
                                        pose,
                                        poses[-1],
                                        geometry,
                                        snapshot.minimum_support,
                                        snapshot.hazard_support_radius,
                                    )
                                    if task and geometry
                                    else 0.0
                                )
                        if objective == "task_scan":
                            travel = float(
                                np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1).sum()
                            )
                            # Dispatcher divides gain by 1 + travel. Preserve the
                            # explicit cost-normalized advantage over staying.
                            gain = (
                                max(0.0, scan.advantage(pose, poses[-1], travel)) * (1 + travel)
                                if scan
                                else 0.0
                            )
                        candidates.append(
                            Maneuver(
                                f"view_{direction}_{distance}_{curvature}",
                                "observe",
                                self.request.parking_type.value,
                                poses,
                                direction,
                                self.cruise_speed,
                                gain,
                                rear_offset=self.rear_offset,
                            )
                        )
        if (
            not candidates
            and self.state not in ("PARKING_BLOCKED", "PARKING_APPROACH")
            and self._view is None
        ):
            return plan
        decision = self.arbiter.select(
            candidates,
            snapshot,
            now,
            pose,
            ego.velocity,
            ego.length,
            ego.width,
            allowed,
            **(
                {"min_view_utility": self.view_min_advantage}
                if getattr(self, "view_objective", "legacy") == "task_scan"
                else {}
            ),
        )
        self.pm.parking_arbitration = decision
        if decision.state == "PARK":
            candidate = decision.maneuver
            # The controller receives only certified poses. Taper to zero at
            # their endpoint, even when a longer geometric hypothesis exists.
            poses = candidate.poses
            remaining = np.r_[
                np.cumsum(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)[::-1])[::-1], 0
            ]
            plan = self._output(poses, candidate.direction, candidate.speed, "observed_prefix")
            plan.velocity = np.minimum(candidate.speed, remaining * 0.5).tolist()
            plan.behavior_state = self.state = "PARKING_APPROACH"
            return plan
        if decision.state == "OBSERVE":
            if self._view is None:
                self._view = decision.maneuver
            candidate = decision.maneuver
            return self._output(
                candidate.poses, candidate.direction, candidate.speed, decision.reason
            )
        plan.velocity = [0.0] * len(plan.velocity)
        plan.behavior_state = self.state = "PARKING_BLOCKED"
        plan.behavior_reason = self.reason = decision.reason
        if (
            self._view is None
            and self.poses is not None
            and abs(ego.velocity) < 0.02
            and self.base_reason == "obstacle_or_boundary"
            and decision.reason == "no_admissible_maneuver"
        ):
            # Keep a usable checked prefix, but do not pin a stopped vehicle to
            # a hypothesis that fresh occupied evidence has made unusable.
            self.poses = None
            self._segments = []
            self._segment = self._progress = 0
            self._attempted_revision = None
            self._shift_since = None
            self._search_time = now
        if self._view is not None and abs(ego.velocity) < 0.02:
            # A stopped rejected view must not permanently lock out all other
            # views. Count the failed attempt against the finite retry budget.
            self.arbiter.complete_view(self._view.poses[-1])
            self._view = None
        return plan
