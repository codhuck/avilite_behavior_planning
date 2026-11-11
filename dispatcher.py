"""Map-aware selection of global planners and complete local pipelines."""

import logging

from avlite.c10_perception.c11_perception_model import HDMap, RaceMap
from avlite.c20_planning.c22_global_planning_strategy import GlobalPlannerStrategy
from avlite.c20_planning.c23_local_planning_strategy import LocalPlanningPipeline
from avlite.c20_planning.c25_global_race_planners import GlobalCenterlineRacePlanner
from avlite.c20_planning.c26_local_path_planners import ReferencePathPlanner  # noqa: F401 -- register strategy
from avlite.c20_planning.c27_local_behavioral_and_velocity_planners import (
    CruiseBehavioralPlanner,  # noqa: F401 -- register strategy
    VelocityLocalPlanner,  # noqa: F401 -- register strategy
)
from avlite.c20_planning.c28_local_lattice_planners import ShortestPathLatticePlanner  # noqa: F401 -- register strategies
from avlite.c20_planning.c29_settings import PlanningSettings

from .global_planner import HDMapLaneFollowingPlanner
from .hdmap_behavior import HDMapBehaviorPlanner, HDMapRuleVelocityPlanner  # noqa: F401 -- register strategies
from .race_behavior import RaceBehaviorPlanner, RacePathPlanner, RaceVelocityPlanner  # noqa: F401 -- register strategies

log = logging.getLogger(__name__)


def map_mode(map):
    if isinstance(map, HDMap):
        return "road"
    if isinstance(map, RaceMap):
        return "race"
    raise ValueError(
        "PlannerDispatcher requires an HDMap or RaceMap; unknown maps are not racing tracks"
    )


class GlobalPlannerDispatcher(GlobalPlannerStrategy):
    def __init__(self, map):
        super().__init__(map)
        self.mode = map_mode(map)
        self.delegate = (
            HDMapLaneFollowingPlanner(map, max_velocity=3.0)
            if self.mode == "road"
            else GlobalCenterlineRacePlanner(map, max_velocity=10.0)
        )

    @property
    def hdmap(self):
        if self.mode != "road":
            raise AttributeError("Race dispatcher has no HD map")
        return self.delegate.hdmap

    @property
    def stack_requirements(self):
        return self.delegate.stack_requirements

    @property
    def world_requirements(self):
        return self.delegate.world_requirements

    def plan(self, perception_model=None, sensors=None):
        if self.start_point is not None and self.goal_point is not None:
            self.delegate.set_start_goal(self.start_point, self.goal_point)
        self.global_plan = self.delegate.plan(perception_model, sensors)
        log.info("DISPATCH global mode=%s planner=%s", self.mode, type(self.delegate).__name__)
        return self.global_plan


class PlannerDispatcher(LocalPlanningPipeline):
    """Switch only on route replacement, never drop road rules on missing data."""

    MODES = {
        "road": ("HDMapBehaviorPlanner", "ReferencePathPlanner", "HDMapRuleVelocityPlanner"),
        "race": ("RaceBehaviorPlanner", "RacePathPlanner", "RaceVelocityPlanner"),
    }

    def __init__(self, global_plan, env, setting=PlanningSettings):
        self._source_setting = setting.model_copy(deep=True)
        self.mode = map_mode(env.map)
        if self.mode == "race" and not global_plan.race_mode:
            raise ValueError("RaceMap requires a race-mode global plan")
        selected = self._source_setting.model_copy(deep=True)
        (
            selected.c23_behavioral_strategy,
            selected.c23_path_strategy,
            selected.c23_velocity_strategy,
        ) = self.MODES[self.mode]
        super().__init__(global_plan, env, selected)
        if any(stage is None for stage in self._stages):
            raise ValueError("Dispatcher stage is unavailable: " + self.mode)
        self._selected_map = env.map
        self._last_race_decision = None
        log.info("DISPATCH local mode=%s stages=%s", self.mode, self.MODES[self.mode])

    def set_global_plan(self, global_plan, ego_xy=None):
        # Build first: a failed selection must not partially overwrite stages.
        candidate = type(self)(global_plan, self.pm, self._source_setting)
        if ego_xy is not None:
            LocalPlanningPipeline.set_global_plan(candidate, global_plan, ego_xy)
        self.__dict__.update(candidate.__dict__)

    @property
    def selection_status(self):
        """Describe the actual delegates without running another planning step."""
        plan = self._working_plan
        path = getattr(plan, "selected_path_planner", type(self._path).__name__)
        state = getattr(plan, "behavior_state", "READY")
        reason = getattr(plan, "behavior_reason", "awaiting_replan")
        traffic = ""
        if self.mode == "race":
            tj = self.global_trajectory
            ego = self.pm.ego_vehicle
            s, _ = tj.convert_xy_to_sd(ego.x, ego.y)
            distances = []
            for agent in self.pm.agent_vehicles:
                other_s, _ = tj.convert_xy_to_sd(agent.x, agent.y)
                gap = (other_s - s + tj.track_end_s / 2) % tj.track_end_s - tj.track_end_s / 2
                distances.append(f"#{agent.agent_id}: {gap:+.0f} m")
            traffic = f" | Cars: {len(distances)} (" + ", ".join(distances) + ")"
        return (
            f"{self.mode.upper()} | {type(self._behavioral).__name__} / "
            f"{path} / {type(self._velocity).__name__}\n{state}: {reason}{traffic}"
        )

    def replan(self, perception_model=None, sensors=None):
        pm = self.pm if perception_model is None else perception_model
        if pm.map is not self._selected_map:
            self._working_plan = None
            raise ValueError("Map changed: replace the global route before replanning")
        super().replan(pm, sensors)
        if self.mode == "race" and self._working_plan.behavior_state == "OVERTAKE":
            self._behavioral.passing_id = self._working_plan.race_leader
        if self.mode == "race":
            decision = (
                self._working_plan.behavior_state,
                self._working_plan.behavior_reason,
                self._working_plan.race_leader,
            )
            if decision != self._last_race_decision:
                log.info(
                    "RACE state=%s reason=%s rival=%s path=%s",
                    *decision,
                    self._working_plan.selected_path_planner,
                )
                self._last_race_decision = decision
        self._working_plan.planner_mode = self.mode
        self._working_plan.path_planner = type(self._path).__name__
