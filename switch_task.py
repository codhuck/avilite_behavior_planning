"""Replace a complete planning stack at a synchronous execution boundary."""

import copy
import logging
from dataclasses import dataclass

import numpy as np
from avlite.c20_planning.c22_global_planning_strategy import GlobalPlannerStrategy
from avlite.c20_planning.c23_local_planning_strategy import LocalPlanningStrategy
from avlite.c20_planning.c29_settings import PlanningSettings
from avlite.c40_execution.c41_world_bridge import is_world_capability_enabled
from avlite.c40_execution.c43_task_strategy import StackEvent, TaskSchedule, TaskStrategy
from avlite.c40_execution.c44_sync_executer import SyncExecuter
from avlite.c50_common.c51_capabilities import satisfies_requirements

from .dispatcher import GlobalPlannerDispatcher

log = logging.getLogger(__name__)


def validate_world_requirements(executer, modules):
    """Core assembly warns about missing sensors; handovers must refuse them."""
    available = {c for c in executer.world.world_capabilities if is_world_capability_enabled(c)}
    for module in modules:
        if not satisfies_requirements(module.world_requirements, available):
            raise ValueError("World does not provide the selected strategy's required sensors")


@dataclass
class SelectPlanningStack(StackEvent):
    """Request a route and automatic planner selection for the current map.

    expected_map is an identity token: an old request cannot affect a new world.
    The task does not load worlds, teleport the vehicle, or change perception.
    """

    global_plan: object
    expected_map: object
    planning_settings: object = None
    global_strategy: str = "GlobalPlannerDispatcher"
    local_strategy: str = "PlannerDispatcher"


class PlannerSwitchTask(TaskStrategy):
    schedule = TaskSchedule.ON_EVENT
    listen_events = frozenset({SelectPlanningStack})

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_result = None

    def select_current_route(self, executer):
        """UI entry point; selection still travels through the execution task runner."""
        if not getattr(executer, "call_tasks", True):
            raise ValueError("Enable execution tasks before selecting planners")
        if executer.local_planner is None:
            raise ValueError("Load a global route before selecting planners")
        executer.task_runner.notify(
            SelectPlanningStack(executer.local_planner.global_plan, executer.pm.map)
        )

    def execute(self, executer, event=None):
        if not isinstance(event, SelectPlanningStack):
            return
        # Worker-thread executers need their own synchronization protocol.
        if not isinstance(executer, SyncExecuter):
            raise ValueError("PlannerSwitchTask requires SyncExecuter")
        if not callable(getattr(executer, "invalidate_planning", None)):
            raise ValueError("Select PlanningSyncExecuter for planner handovers on stock AVLite")
        if event.expected_map is not executer.pm.map:
            raise ValueError("Stale planner switch: map changed")
        ego = executer.pm.ego_vehicle
        if not np.isfinite(ego.velocity) or abs(ego.velocity) > 0.1:
            raise ValueError("Stop the vehicle before replacing the planning stack")
        # PM can describe the beginning of the tick; the world has already moved.
        world_speed = executer.world.get_ego_state().velocity
        if not np.isfinite(world_speed) or abs(world_speed) > 0.1:
            raise ValueError("Stop the vehicle before replacing the planning stack")
        if not np.isfinite([ego.x, ego.y, ego.theta]).all():
            raise ValueError("Invalid ego pose")
        route = copy.deepcopy(event.global_plan)
        points = np.asarray(route.path)
        if (
            points.ndim != 2
            or points.shape[1] != 2
            or len(points) < 2
            or not np.isfinite(points).all()
        ):
            raise ValueError("Planner switch requires a finite nonempty route")
        if np.min(np.linalg.norm(points - [ego.x, ego.y], axis=1)) > 5.0:
            raise ValueError("New route is too far from the current vehicle")
        for values in (route.velocity, route.left_boundary_d, route.right_boundary_d):
            if not np.isfinite(np.asarray(values)).all():
                raise ValueError("Route speed and boundaries must be finite")
        setting = event.planning_settings
        if setting is None:
            setting = getattr(executer.local_planner, "_source_setting", PlanningSettings)
        # Select explicit AVLite-registered strategies; unknown names never fall back.
        global_cls = GlobalPlannerStrategy.registry.get(event.global_strategy)
        local_cls = LocalPlanningStrategy.registry.get(event.local_strategy)
        if global_cls is None or local_cls is None:
            raise ValueError("Unknown global or local planning strategy")
        global_planner = global_cls(executer.pm.map)
        global_planner.global_plan = route
        if isinstance(global_planner, GlobalPlannerDispatcher):
            global_planner.delegate.global_plan = route
        local_planner = local_cls(route, executer.pm, setting.model_copy(deep=True))
        local_planner.set_global_plan(route, ego_xy=(ego.x, ego.y))
        # Validate using a shallow executor view before publishing either pointer.
        candidate = copy.copy(executer)
        candidate.global_planner = global_planner
        candidate.local_planner = local_planner
        candidate.validate_stack()
        validate_world_requirements(executer, (global_planner, local_planner))
        local_planner.step(ego)
        local_planner.replan(executer.pm)
        plan = local_planner.get_local_plan()
        if (
            plan is None
            or not len(plan.path)
            or not np.isfinite(np.asarray(plan.path)).all()
            or not np.isfinite(np.asarray(plan.velocity)).all()
        ):
            raise ValueError("Selected planner produced no finite local plan")
        if executer.controller is not None:
            executer.controller.reset()
        previous = type(executer.local_planner).__name__
        executer.global_planner = global_planner
        executer.local_planner = local_planner
        executer.invalidate_planning()
        self.last_result = dict(
            previous=previous,
            mode=getattr(local_planner, "mode", "configured"),
            global_planner=type(getattr(global_planner, "delegate", global_planner)).__name__,
            stages=tuple(
                type(stage).__name__
                for stage in getattr(local_planner, "_stages", ())
                if stage is not None
            ),
        )
        log.info("DISPATCH stack replaced: %s", self.last_result)


@dataclass
class SelectPerceptionStack(StackEvent):
    """Install a freshly constructed perception strategy while execution is paused.

    The caller constructs the strategy against the current perception model.
    Construction/model loading happens before enqueueing, so it cannot block a
    moving execution tick. The expected map guards against stale UI requests.
    """

    perception: object
    expected_map: object


class PerceptionSwitchTask(TaskStrategy):
    """Validated stopped handover; discard observations from the former producer."""

    schedule = TaskSchedule.ON_EVENT
    listen_events = frozenset({SelectPerceptionStack})

    def __init__(self):
        self.last_result = None

    def reset(self):
        self.last_result = None

    def execute(self, executer, event=None):
        from avlite.c10_perception.c12_perception_strategy import PerceptionStrategy

        if not isinstance(event, SelectPerceptionStack):
            return
        if not isinstance(executer, SyncExecuter):
            raise ValueError("PerceptionSwitchTask requires SyncExecuter")
        if not callable(getattr(executer, "refresh_pipeline", None)):
            raise ValueError("Select PlanningSyncExecuter for perception handovers on stock AVLite")
        if not executer.stopped or abs(executer.pm.ego_vehicle.velocity) > 0.02:
            raise ValueError("Pause execution and stop before replacing perception")
        if not np.isfinite(executer.pm.ego_vehicle.velocity):
            raise ValueError("Invalid ego velocity")
        # A paused executor can still hold a pre-tick PM snapshot. Read the
        # current bridge speed as for planner handovers before discarding data.
        world_speed = executer.world.get_ego_state().velocity
        if not np.isfinite(world_speed) or abs(world_speed) > 0.02:
            raise ValueError("Stop the vehicle before replacing perception")
        if event.expected_map is not executer.pm.map:
            raise ValueError("Stale perception switch: map changed")
        perception = event.perception
        if not isinstance(perception, PerceptionStrategy):
            raise ValueError("Expected a PerceptionStrategy instance")
        if perception is executer.perception:
            raise ValueError("Construct a new perception strategy before handover")
        if perception.perception_model is not executer.pm:
            raise ValueError("Candidate must use the current perception model")
        candidate = copy.copy(executer)
        candidate.perception = perception
        candidate.validate_stack()
        validate_world_requirements(executer, (perception,))
        # All refusal paths precede publication. Do not run detector inference,
        # constructor or a third-party reset callback during this boundary.
        executer.refresh_pipeline()
        previous = type(executer.perception).__name__
        executer.perception = perception
        executer._last_cmd = None
        executer.pm.agent_vehicles = []
        executer.pm.static_obstacles = []
        for field in ("regulatory_observation", "parking_observation", "parking_occupancy"):
            setattr(executer.pm, field, None)
        self.last_result = dict(previous=previous, current=type(perception).__name__)
        log.info("DISPATCH perception replaced: %s", self.last_result)
