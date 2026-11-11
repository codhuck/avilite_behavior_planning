"""Map rules select behavior; the velocity stage enforces their constraints."""

from __future__ import annotations

import copy
import logging
import math
from dataclasses import dataclass

import numpy as np
from avlite.c20_planning.c21_planning_model import LocalBehavior
from avlite.c20_planning.c23_local_planning_strategy import LocalBehavioralPlanningStrategy
from avlite.c20_planning.c27_local_behavioral_and_velocity_planners import VelocityLocalPlanner
from avlite.c50_common.c51_capabilities import StackCapability
from avlite.c50_common.c54_trajectory_tracker import slice_trajectory_horizon
from shapely.geometry import LineString, Point, Polygon

from .regulatory import (
    RegulatoryFeature,
    RegulatoryKind,
    extract_regulatory_features,
    road_st_to_xy,
)
from .settings import PluginSettings

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class RouteRule:
    feature: RegulatoryFeature
    route_s: float
    route_d: float
    exit_s: float | None = None


class HDMapBehaviorPlanner(LocalBehavioralPlanningStrategy):
    """Stop dwell, light gating and crosswalk yielding, using simulation time.

    SensorRegulatoryDetector supplies RGB/LiDAR observations; the legacy
    CarlaRegulatoryTask remains available for ground-truth comparisons.
    Other bridges need an equivalent provider; missing data is never a green
    light or an empty pedestrian crossing.
    """

    world_requirements = frozenset()
    stack_requirements = frozenset({StackCapability.MAP_HD, StackCapability.LOCALIZATION})
    stack_capabilities = frozenset()

    def __init__(self, global_plan, env):
        self.env = env
        self.events = []
        self.set_global_plan(global_plan)

    def _resolve_route_lanes(self):
        lanes = getattr(self.global_plan, "lane_path", None) or []
        if lanes:
            return lanes
        uids = getattr(self.global_plan, "lane_uids", [])
        if uids:
            by_uid = self.env.map.lane_by_uid
            missing = [uid for uid in uids if uid not in by_uid]
            if missing:
                raise ValueError(f"Route lanes are absent from the loaded HD map: {missing}")
            return [by_uid[uid] for uid in uids]
        locate = getattr(self.env.map, "find_nearest_lane", None)
        if not callable(locate):
            return []
        found = {}
        for x, y in self.global_plan.path:
            lane = locate(x, y)
            if lane is not None and len(lane.center_line.T):
                distance = np.min(np.linalg.norm(lane.center_line.T - [x, y], axis=1))
                if distance <= max(0.5, lane.width / 2):
                    found[lane.uid] = lane
        return list(found.values())

    def set_global_plan(self, global_plan, ego_xy=None):
        if len(global_plan.path) >= 2 and not LineString(global_plan.path).is_simple:
            raise ValueError(
                "HD-map behavior requires a non-self-intersecting route: "
                "repeated passages need progress-aware map matching. Choose another goal."
            )
        self.global_plan = global_plan
        self.env.regulatory_route = list(global_plan.path)
        self.features = (
            extract_regulatory_features(self.env.map) if self.env.map is not None else []
        )
        self._route_lanes = self._resolve_route_lanes()
        self.route_rules = self._project_rules()
        self.route_stops = [
            r for r in self.route_rules if r.feature.kind == RegulatoryKind.STOP_SIGN
        ]
        self.reset()
        log.info(
            "HD-map route rules: %s",
            [(r.feature.kind.value, r.feature.id, round(r.route_s, 1)) for r in self.route_rules],
        )

    def reset(self, wp=0):
        self.generation = getattr(self, "generation", 0) + 1
        self._completed = set()
        self._stopped_since = {}
        self._last_decision = None
        self._waiting_key = None
        self._waiting_since = None
        self.events.clear()

    def _route_lane_allows(self, feature):
        lanes = [lane for lane in self._route_lanes if str(lane.road_id) == feature.road_id]
        if feature.lane_validity:
            return any(
                min(a, b) <= int(lane.id) <= max(a, b)
                for lane in lanes
                for a, b in feature.lane_validity
            )
        if feature.orientation not in {"+", "-"}:
            return bool(lanes)
        return any((int(lane.id) < 0) == (feature.orientation == "+") for lane in lanes)

    def _project_rules(self):
        tj = self.global_plan.trajectory
        if tj is None or len(self.global_plan.path) < 2:
            return []
        route = LineString(self.global_plan.path)
        roads = {str(r.id): r for r in getattr(self.env.map, "roads", ())}
        rules = []
        for f in self.features:
            exit_s = None
            if f.kind == RegulatoryKind.CROSSWALK:
                if len(f.polygon) < 3:
                    continue
                polygon = Polygon(f.polygon)
                if not polygon.is_valid or polygon.area < 0.1 or not route.intersects(polygon):
                    continue
                intersection = route.intersection(polygon)
                pieces = (
                    list(intersection.geoms) if hasattr(intersection, "geoms") else [intersection]
                )
                coords = [xy for piece in pieces for xy in piece.coords]
                if not coords:
                    continue
                s, d = min(route.project(Point(xy)) for xy in coords), 0.0
                exit_s = max(route.project(Point(xy)) for xy in coords)
            else:
                if not self._route_lane_allows(f):
                    continue
                x, y = f.x, f.y
                if f.stop_line_s is not None and f.road_id in roads:
                    x, y = road_st_to_xy(roads[f.road_id], f.stop_line_s, 0)
                s, d = tj.convert_xy_to_sd(x, y)
                if not 0 <= s <= tj.track_end_s:
                    continue
            rules.append(RouteRule(f, float(s), float(d), exit_s))
        # References on approach and junction connectors may share a signal ID.
        unique = {}
        for rule in sorted(rules, key=lambda r: r.route_s):
            unique.setdefault((rule.feature.kind, rule.feature.id), rule)
        return list(unique.values())

    def _event(self, event, **details):
        record = {
            "time": float(getattr(self.env, "simulation_time", 0.0)),
            "event": event,
            **details,
        }
        self.events.append(record)
        log.info("HDMAP %s", record)

    def plan_behavior(self, plan):
        ego = self.env.ego_vehicle
        ego_s, _ = self.global_plan.trajectory.convert_xy_to_sd(ego.x, ego.y)
        now = float(getattr(self.env, "simulation_time", 0.0))
        obs = getattr(self.env, "regulatory_observation", None)
        fresh = obs is not None and 0 <= now - obs.time <= PluginSettings.observation_max_age
        plan.behavior = LocalBehavior.CRUISE
        plan.behavior_state, plan.behavior_reason = "CRUISE", "route_clear"
        plan.rule_id, plan.rule_distance, plan.speed_limit = None, None, None
        plan.decision_time = now
        if getattr(obs, "source", None) == "rgb_lidar" and (
            not fresh or obs.diagnostics.get("status") != "ok"
        ):
            plan.behavior = LocalBehavior.STOP
            plan.behavior_state = "PERCEPTION_STOP"
            plan.behavior_reason = "sensor_" + (
                obs.diagnostics.get("status", "unknown") if fresh else "stale"
            )
            plan.rule_distance = 0.0
            plan.stop_feature_id = None
            plan.stop_distance = 0.0
            plan.stop_target_s = ego_s
            plan.stop_target_xy = (ego.x, ego.y)
            plan.behavior_phase = (
                "WAITING" if abs(ego.velocity) <= PluginSettings.stopped_speed else "APPROACHING"
            )
            plan.wait_duration = 0.0
            decision = (plan.behavior_state, plan.behavior_reason)
            if decision != self._last_decision:
                self._event(
                    "decision",
                    state=decision[0],
                    reason=decision[1],
                    rule_id=None,
                    distance=0.0,
                    speed=float(ego.velocity),
                    phase=plan.behavior_phase,
                )
                self._last_decision = decision
            return plan
        for rule in self.route_rules:
            f = rule.feature
            if f.id in self._completed:
                continue
            target_s = max(0.0, rule.route_s - float(ego.length) / 2 - PluginSettings.stop_buffer)
            distance = float(target_s - ego_s)
            if distance > PluginSettings.stop_lookahead:
                continue
            reason, hold = f.kind.value, False
            if f.kind == RegulatoryKind.STOP_SIGN:
                arrived = abs(distance) <= PluginSettings.arrival_tolerance
                if arrived and abs(ego.velocity) <= PluginSettings.stopped_speed:
                    began = self._stopped_since.setdefault(f.id, now)
                    if now - began >= PluginSettings.stop_hold_time:
                        self._completed.add(f.id)
                        self._event(
                            "stop_completed", rule_id=f.id, distance=distance, hold_time=now - began
                        )
                        continue
                    state, hold = "HOLD_STOP", True
                else:
                    self._stopped_since.pop(f.id, None)
                    state = "APPROACH_STOP"
                if distance < -PluginSettings.arrival_tolerance:
                    state, reason = "VIOLATION", "stop_target_overrun"
            elif f.kind == RegulatoryKind.TRAFFIC_LIGHT:
                color = obs.lights.get(f.id, "Unknown") if fresh else "Unknown"
                if color == "Green":
                    if ego_s + float(ego.length) / 2 > rule.route_s:
                        self._completed.add(f.id)
                        self._event("light_passed_green", rule_id=f.id)
                    continue
                state, reason = "WAIT_LIGHT", "light_" + color.lower()
            else:
                # Keep the crossing active until the rear bumper clears its
                # actual route intersection, including wide/diagonal crossings.
                if rule.exit_s is not None and ego_s - float(ego.length) / 2 > rule.exit_s:
                    continue
                occupied = None
                if fresh and hasattr(obs, "crossings"):
                    occupied = obs.crossings.get(f.id)
                elif fresh:
                    area = Polygon(f.polygon).buffer(PluginSettings.pedestrian_margin)
                    occupied = any(area.covers(Point(p[0], p[1])) for p in obs.pedestrians)
                if occupied is False:
                    plan.speed_limit = PluginSettings.crosswalk_speed
                    plan.behavior_state, plan.behavior_reason = "SLOW_CROSSWALK", "crosswalk_clear"
                    plan.rule_id, plan.rule_distance = f.id, distance
                    continue
                state, reason = (
                    "YIELD_PEDESTRIAN",
                    "crosswalk_occupied" if occupied else "crosswalk_unknown",
                )
            plan.behavior = LocalBehavior.STOP
            plan.behavior_state, plan.behavior_reason = state, reason
            plan.rule_id, plan.rule_distance = f.id, distance
            plan.stop_feature_id = f.id
            plan.stop_distance = 0.0 if hold else max(0.0, distance)
            plan.stop_target_s = target_s
            plan.stop_target_xy = self.global_plan.trajectory.convert_sd_to_xy(target_s, 0.0)
            break
        stopping = plan.behavior == LocalBehavior.STOP
        waiting = (
            stopping
            and abs(ego.velocity) <= PluginSettings.stopped_speed
            and plan.rule_distance <= PluginSettings.arrival_tolerance
        )
        key = (plan.rule_id, plan.behavior_reason) if waiting else None
        if key != self._waiting_key:
            self._waiting_key = key
            self._waiting_since = now if waiting else None
        plan.behavior_phase = "WAITING" if waiting else ("APPROACHING" if stopping else "MOVING")
        plan.wait_duration = (
            max(0.0, now - self._waiting_since) if self._waiting_since is not None else 0.0
        )
        goal = self.global_plan.path[-1]
        if (
            not stopping
            and abs(ego.velocity) <= PluginSettings.stopped_speed
            and math.hypot(ego.x - goal[0], ego.y - goal[1]) <= 0.8
            and ego_s >= self.global_plan.trajectory.track_end_s - 0.8
        ):
            plan.behavior_state = "ARRIVED"
            plan.behavior_reason = "route_complete"
            plan.behavior_phase = "ARRIVED"
        decision = (plan.behavior_state, plan.behavior_reason, plan.rule_id, plan.behavior_phase)
        if decision != self._last_decision:
            self._event(
                "decision",
                state=decision[0],
                reason=decision[1],
                rule_id=decision[2],
                distance=plan.rule_distance,
                speed=float(ego.velocity),
                phase=plan.behavior_phase,
            )
            self._last_decision = decision
        return plan


class HDMapStopBehaviorPlanner(HDMapBehaviorPlanner):
    """Compatibility name for existing profiles."""


class HDMapRuleVelocityPlanner(VelocityLocalPlanner):
    """Obstacle profiling and stop envelope with a control-response margin."""

    def apply_speed_match(self, trajectory, collision_idx, target_vel, ref_velocity=None):
        if target_vel > 1e-6:
            return super().apply_speed_match(trajectory, collision_idx, target_vel, ref_velocity)
        # A static obstacle requires a distance-based stop envelope, not a
        # profile seeded with the measured ego speed. Seeding with numerical
        # standstill noise otherwise locks every subsequent replan at zero.
        reference = trajectory.velocity if ref_velocity is None else ref_velocity
        trajectory.velocity = np.array(reference, dtype=float, copy=True)
        self._finalize_velocity(trajectory, ref_velocity, collision_idx, 0.0)

    @staticmethod
    def curve_speed_profile(tj, velocity):
        """Limit bends and propagate braking upstream on arc-length samples."""
        s = np.asarray(tj.path_s, dtype=float)
        if len(s) < 3:
            return velocity
        keep = np.r_[True, np.diff(s) > 1e-6]
        unique_s = s[keep]
        if len(unique_s) < 3:
            return velocity
        x, y = np.asarray(tj.path_x)[keep], np.asarray(tj.path_y)[keep]
        heading = np.unwrap(np.arctan2(np.gradient(y, unique_s), np.gradient(x, unique_s)))
        curvature = np.abs(np.gradient(heading, unique_s))
        cap = np.sqrt(PluginSettings.max_lateral_acceleration / np.maximum(curvature, 1e-6))
        result = np.minimum(velocity, np.interp(s, unique_s, cap))
        # Slowing only at the bend is too late: retain distance to decelerate.
        for i in range(len(result) - 2, -1, -1):
            ds = max(0.0, s[i + 1] - s[i])
            result[i] = min(
                result[i], math.sqrt(result[i + 1] ** 2 + 2 * PluginSettings.max_deceleration * ds)
            )
        return result

    def plan_velocity(self, plan):
        source = plan.as_trajectory()
        if source is not None:
            source = copy.deepcopy(source)
            ego = self.pm.ego_vehicle
            source.update_waypoint_by_xy(ego.x, ego.y)
            # ReferencePathPlanner supplies the whole route. Collision checking
            # must not let a vehicle on the already-traversed prefix stop us.
            plan.trajectory = slice_trajectory_horizon(source, max_points=0)
            # Keep the existing center-based steering contract; slicing must
            # not enable Stanley's optional parent-trajectory lookahead mode.
            plan.trajectory.parent_trajectory = None
            plan.path = list(plan.trajectory.path)
            plan.velocity = list(plan.trajectory.velocity)
        plan = super().plan_velocity(plan)
        tj = plan.as_trajectory()
        if tj is None or not len(tj.velocity):
            return plan
        velocity = np.array(tj.velocity, dtype=float, copy=True)
        velocity = self.curve_speed_profile(tj, velocity)
        if getattr(plan, "speed_limit", None) is not None:
            velocity = np.minimum(velocity, plan.speed_limit)
        if getattr(plan.behavior, "name", None) == "STOP" and hasattr(plan, "stop_distance"):
            ego = self.pm.ego_vehicle
            ego_s, _ = tj.convert_xy_to_sd(ego.x, ego.y)
            decel = max(0.1, float(PluginSettings.max_deceleration))
            distance = max(0.0, plan.stop_distance - 0.5 * PluginSettings.arrival_tolerance)
            for i in range(len(velocity)):
                remaining = max(0.0, distance - max(0.0, tj.path_s[i] - ego_s))
                cap = math.sqrt((0.8 * decel) ** 2 + 2 * decel * remaining) - 0.8 * decel
                velocity[i] = min(velocity[i], cap)
        tj.velocity = velocity
        plan.velocity, plan.trajectory = list(velocity), tj
        return plan
