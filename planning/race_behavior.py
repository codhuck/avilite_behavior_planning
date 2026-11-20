"""Conservative race behavior with validated lattice passing proposals."""

import copy
import math

import numpy as np
from avlite.c20_planning.c21_planning_model import LocalBehavior
from avlite.c20_planning.c23_local_planning_strategy import (
    LocalBehavioralPlanningStrategy,
    LocalPathPlanningStrategy,
    LocalPlanningStrategy,
)
from avlite.c20_planning.c26_local_path_planners import ReferencePathPlanner
from avlite.c20_planning.c28_local_lattice_planners import ShortestPathLatticePlanner
from avlite.c20_planning.c29_settings import PlanningSettings
from avlite.c50_common.c54_trajectory_tracker import TrajectoryTracker
from avlite.c50_common.c55_collision_checking import check_collision_2d

from .hdmap_behavior import HDMapRuleVelocityPlanner


def forward_gap(route, origin, target):
    gap = target - origin
    if route.race_mode and np.linalg.norm(np.asarray(route.path[0]) - route.path[-1]) < 10:
        gap %= route.trajectory.track_end_s
    return gap


class RaceBehaviorPlanner(LocalBehavioralPlanningStrategy):
    world_requirements = frozenset()
    stack_requirements = frozenset()
    stack_capabilities = frozenset()

    def __init__(self, global_plan, env):
        self.env = env
        self.set_global_plan(global_plan)

    def set_global_plan(self, global_plan, ego_xy=None):
        self.global_plan = global_plan
        self.reset()

    def reset(self, wp=0):
        self.events = []
        self.passing_id = None
        self.returning = False

    def plan_behavior(self, plan):
        ego = self.env.ego_vehicle
        tj = self.global_plan.trajectory
        s, d = tj.convert_xy_to_sd(ego.x, ego.y)
        was_passing = self.passing_id is not None
        leaders = []
        for agent in self.env.agent_vehicles:
            sa, da = tj.convert_xy_to_sd(agent.x, agent.y)
            gap = forward_gap(self.global_plan, s, sa)
            if agent.agent_id == self.passing_id:
                # Keep the same rival while alongside. Only return after the
                # rear bumper has cleared it, not merely after moving sideways.
                signed_gap = gap
                if signed_gap > tj.track_end_s / 2:
                    signed_gap -= tj.track_end_s
                if signed_gap > -(ego.length + agent.length) / 2 - 3.0:
                    leaders = [(signed_gap, agent)]
                    break
                self.passing_id = None
                self.returning = True
            if 0 < gap < 40 and abs(da - d) < (ego.width + agent.width) / 2 + 0.5:
                leaders.append((gap, agent))
        plan.behavior = LocalBehavior.CRUISE
        plan.behavior_state = "RACE"
        plan.behavior_reason = "track_clear"
        plan.speed_limit = None
        plan.race_leader = None
        plan.request_overtake = False
        plan.lateral_maneuver = was_passing or self.returning
        if not leaders:
            self.passing_id = None
            if was_passing:
                self.returning = True
            if self.returning and abs(d) > 0.5:
                plan.behavior_state = "RETURN"
                plan.behavior_reason = "off_reference"
            elif abs(d) <= 0.5:
                self.returning = False
            return plan
        gap, lead = min(leaders, key=lambda item: item[0])
        plan.race_leader = lead.agent_id
        plan.race_gap = gap
        bumper_gap = gap - (ego.length + lead.length) / 2
        desired = 5.0 + 1.5 * max(0.0, ego.velocity)
        plan.follow_speed = max(0.0, lead.velocity + 0.5 * (bumper_gap - desired))
        plan.speed_limit = plan.follow_speed
        plan.behavior = LocalBehavior.FOLLOW
        plan.behavior_state = "FOLLOW"
        plan.behavior_reason = "leader_ahead"
        heading_diff = abs((lead.theta - ego.theta + math.pi) % (2 * math.pi) - math.pi)
        reference = float(tj.velocity[tj.current_wp])
        plan.request_overtake = (
            heading_diff < math.pi / 3
            and reference > lead.velocity + 0.5
            and (bumper_gap > 3.0 or lead.agent_id == self.passing_id)
        )
        lead_s, lead_d = tj.convert_xy_to_sd(lead.x, lead.y)
        left = self.global_plan.left_boundary_d
        right = self.global_plan.right_boundary_d
        if len(left) != len(tj.path_s) or len(right) != len(tj.path_s):
            plan.request_overtake = False
        else:
            bounds = sorted(
                (
                    float(np.interp(lead_s, tj.path_s, left)),
                    float(np.interp(lead_s, tj.path_s, right)),
                )
            )
            margin = ego.width / 2 + 0.5
            separation = (ego.width + lead.width) / 2 + 0.5
            room = (
                bounds[1] - margin - lead_d > separation
                or lead_d - (bounds[0] + margin) > separation
            )
            plan.request_overtake = plan.request_overtake and room
            if not room:
                plan.behavior_reason = "track_too_narrow_to_pass"
        # A constant-offset passing corridor is not reliable through tight
        # corners: its inner radius and the vehicle's swept body differ sharply
        # from the reference. Defer starting a pass until the horizon is clear.
        if plan.request_overtake and self.passing_id is None:
            samples = np.arange(s, s + 81.0, 1.0)
            if np.linalg.norm(np.asarray(tj.path[0]) - tj.path[-1]) < 10:
                samples %= tj.track_end_s
            headings = np.unwrap(np.interp(samples, tj.path_s, np.unwrap(tj.path_heading)))
            if np.max(np.abs(np.diff(headings))) > 0.03:
                plan.request_overtake = False
                plan.behavior_reason = "tight_curve_ahead"
        return plan


class RacePathPlanner(LocalPlanningStrategy, LocalPathPlanningStrategy):
    """Reference following, or a collision/boundary checked lattice proposal."""

    def __init__(self, global_plan, env, setting=PlanningSettings):
        super().__init__(global_plan, env, setting)
        self.reference = ReferencePathPlanner(global_plan, env, setting)
        self.lattice = ShortestPathLatticePlanner(global_plan, env, setting)

    def set_global_plan(self, global_plan, ego_xy=None):
        super().set_global_plan(global_plan, ego_xy)
        for p in (self.reference, self.lattice):
            p.set_global_plan(global_plan, ego_xy)

    def reset(self, wp=0):
        super().reset(wp)
        for p in (self.reference, self.lattice):
            p.reset(wp)

    def replan(self, perception_model=None, sensors=None):
        if perception_model is not None:
            self.pm = perception_model

    def valid_candidate(self, candidate):
        tj = candidate.as_trajectory()
        if tj is None or len(tj.path) < 2:
            return False
        if check_collision_2d(self.pm, tj)[0]:
            return False
        route = self.global_trajectory
        left = self.global_plan.left_boundary_d
        right = self.global_plan.right_boundary_d
        if len(left) != len(route.path_s) or len(right) != len(route.path_s):
            return False
        ego = self.pm.ego_vehicle
        for i, (x, y) in enumerate(tj.path):
            s, d = route.convert_xy_to_sd(x, y)
            ref_heading = float(np.interp(s, route.path_s, np.unwrap(route.path_heading)))
            angle = float(tj.path_heading[i]) - ref_heading
            margin = (
                abs(math.cos(angle)) * ego.width / 2 + abs(math.sin(angle)) * ego.length / 2 + 0.5
            )
            lo = min(
                float(np.interp(s, route.path_s, left)), float(np.interp(s, route.path_s, right))
            )
            hi = max(
                float(np.interp(s, route.path_s, left)), float(np.interp(s, route.path_s, right))
            )
            if d - margin < lo or d + margin > hi:
                return False
        return True

    def plan_path(self, plan):
        for p in (self.reference, self.lattice):
            p.pm = self.pm
        ego = self.pm.ego_vehicle
        if plan.request_overtake:
            _, ego_d = self.global_trajectory.convert_xy_to_sd(ego.x, ego.y)
            lead = next((a for a in self.pm.agent_vehicles if a.agent_id == plan.race_leader), None)
            if lead is not None:
                _, lead_d = self.global_trajectory.convert_xy_to_sd(lead.x, lead.y)
                if abs(ego_d - lead_d) > (ego.width + lead.width) / 2 + 0.5:
                    # Once alongside, keep the passing corridor until the
                    # behavior stage confirms rear-bumper clearance.
                    hold = self.reference_segment(copy.copy(plan), ego_d)
                    if self.valid_candidate(hold):
                        hold.behavior = LocalBehavior.OVERTAKE
                        hold.behavior_state = "OVERTAKE"
                        hold.behavior_reason = "hold_passing_corridor"
                        hold.speed_limit = None
                        hold.selected_path_planner = "PassingCorridor"
                        return hold
        if plan.request_overtake or plan.behavior_state == "RETURN":
            self.lattice.step(ego)
            proposal = self.lattice.plan_path(copy.copy(plan))
            if self.valid_candidate(proposal):
                offsets = [
                    self.global_trajectory.convert_xy_to_sd(x, y)[1] for x, y in proposal.path
                ]
                # Do not call a straight or short follow trajectory an overtake.
                s0, _ = self.global_trajectory.convert_xy_to_sd(ego.x, ego.y)
                se, _ = self.global_trajectory.convert_xy_to_sd(*proposal.path[-1])
                passes = (
                    max(abs(d) for d in offsets) > ego.width + 0.5
                    and forward_gap(self.global_plan, s0, se)
                    > getattr(plan, "race_gap", float("inf")) + ego.length
                )
                if plan.behavior_state == "RETURN" or passes:
                    plan.path = proposal.path
                    plan.velocity = proposal.velocity
                    plan.trajectory = proposal.trajectory
                    plan.selected_path_planner = "ShortestPathLatticePlanner"
                    if plan.request_overtake:
                        plan.behavior = LocalBehavior.OVERTAKE
                        plan.behavior_state = "OVERTAKE"
                        plan.behavior_reason = "validated_lattice_path"
                        plan.speed_limit = None
                    return plan
        plan.selected_path_planner = "ReferencePathPlanner"
        if plan.request_overtake:
            plan.behavior_reason = "no_valid_passing_path"
        _, ego_d = self.global_trajectory.convert_xy_to_sd(ego.x, ego.y)
        if abs(ego_d) > 0.5 and getattr(plan, "lateral_maneuver", False):
            # Never snap back to the centerline after a failed passing/return
            # proposal. Hold the current corridor until a checked return exists.
            hold = self.reference_segment(copy.copy(plan), ego_d)
            if self.valid_candidate(hold):
                hold.selected_path_planner = "PassingCorridor"
                hold.behavior_reason = "waiting_for_safe_return"
                return hold
            # No safe lateral maneuver: brake along the current heading rather
            # than generate an unchecked cut-in towards another vehicle.
            plan.path = [(ego.x, ego.y), (ego.x + math.cos(ego.theta), ego.y + math.sin(ego.theta))]
            plan.velocity = [0.0, 0.0]
            plan.trajectory = TrajectoryTracker(path=plan.path, velocity=plan.velocity)
            plan.speed_limit = 0.0
            plan.selected_path_planner = "EmergencyStop"
            plan.behavior_reason = "no_safe_corridor"
            return plan
        return self.reference_segment(plan)

    def reference_segment(self, plan, lateral_offset=0.0):
        ego = self.pm.ego_vehicle
        # A closed race route must wrap at the lap seam. The generic reference
        # stage plus a forward-only slice otherwise leaves a one-point stop.
        route = self.global_trajectory
        route.update_waypoint_by_xy(ego.x, ego.y)
        points = []
        speeds = []
        distance = 0.0
        closed = np.linalg.norm(np.asarray(route.path[0]) - route.path[-1]) < 10
        for offset in range(len(route.path)):
            idx = route.current_wp + offset
            if idx >= len(route.path) and not closed:
                break
            idx %= len(route.path)
            if lateral_offset:
                point = route.convert_sd_to_xy(route.path_s[idx], lateral_offset)
            else:
                point = route.path[idx]
            if points:
                ds = float(np.linalg.norm(np.asarray(point) - points[-1]))
                if ds < 1e-5:
                    continue
                distance += ds
            points.append(point)
            speeds.append(float(route.velocity[idx]))
            if distance >= 80.0:
                break
        plan.path = points
        plan.velocity = speeds
        plan.trajectory = TrajectoryTracker(path=points, velocity=speeds)
        return plan


class RaceVelocityPlanner(HDMapRuleVelocityPlanner):
    """Enforce behavior limits as well as obstacle and curvature limits."""

    def apply_speed_match(self, trajectory, collision_idx, target_vel, ref_velocity=None):
        if target_vel <= 1e-6:
            return super().apply_speed_match(trajectory, collision_idx, target_vel, ref_velocity)
        # Track the leader's speed plus a continuous gap correction. Seeding
        # the profile with measured ego speed alternates between holding a
        # near-zero speed and catching up at reference speed.
        reference = trajectory.velocity if ref_velocity is None else ref_velocity
        velocity = np.array(reference, dtype=float, copy=True)
        margin = self._standoff_margin(trajectory, collision_idx)
        follow_target = max(0.0, target_vel + 0.3 * margin)
        velocity[trajectory.current_wp : collision_idx + 1] = np.minimum(
            velocity[trajectory.current_wp : collision_idx + 1], follow_target
        )
        trajectory.velocity = velocity
        self._finalize_velocity(trajectory, ref_velocity, collision_idx, target_vel)
