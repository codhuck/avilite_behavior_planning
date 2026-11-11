"""Replace a complete planning stack at a synchronous execution boundary."""

import copy
import logging
from dataclasses import dataclass

import numpy as np
from avlite.c20_planning.c29_settings import PlanningSettings
from avlite.c40_execution.c43_task_strategy import StackEvent, TaskSchedule, TaskStrategy
from avlite.c40_execution.c44_sync_executer import SyncExecuter

from .dispatcher import GlobalPlannerDispatcher, PlannerDispatcher

log = logging.getLogger(__name__)


@dataclass
class SelectPlanningStack(StackEvent):
    """Request a route and automatic planner selection for the current map.

    expected_map is an identity token: an old request cannot affect a new world.
    The task does not load worlds, teleport the vehicle, or change perception.
    """

    global_plan: object
    expected_map: object


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
        if event.expected_map is not executer.pm.map:
            raise ValueError("Stale planner switch: map changed")
        ego = executer.pm.ego_vehicle
        if not np.isfinite(ego.velocity) or abs(ego.velocity) > 0.1:
            raise ValueError("Stop the vehicle before replacing the planning stack")
        route = copy.deepcopy(event.global_plan)
        points = np.asarray(route.path)
        if len(points) < 2 or not np.isfinite(points).all():
            raise ValueError("Planner switch requires a finite nonempty route")
        if np.min(np.linalg.norm(points - [ego.x, ego.y], axis=1)) > 5.0:
            raise ValueError("New route is too far from the current vehicle")
        setting = getattr(executer.local_planner, "_source_setting", PlanningSettings)
        global_planner = GlobalPlannerDispatcher(executer.pm.map)
        global_planner.global_plan = route
        global_planner.delegate.global_plan = route
        local_planner = PlannerDispatcher(route, executer.pm, setting)
        local_planner.set_global_plan(route, ego_xy=(ego.x, ego.y))
        local_planner.step(ego)
        local_planner.replan(executer.pm)
        # Validate using a shallow executor view before publishing either pointer.
        candidate = copy.copy(executer)
        candidate.global_planner = global_planner
        candidate.local_planner = local_planner
        candidate.validate_stack()
        if executer.controller is not None:
            executer.controller.reset()
        previous = type(executer.local_planner).__name__
        executer.global_planner = global_planner
        executer.local_planner = local_planner
        executer._last_cmd = None
        self.last_result = dict(
            previous=previous,
            mode=local_planner.mode,
            global_planner=type(global_planner.delegate).__name__,
            stages=local_planner.MODES[local_planner.mode],
        )
        log.info("DISPATCH stack replaced: %s", self.last_result)
