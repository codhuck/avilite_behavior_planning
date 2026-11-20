"""Simulator-independent task coordination of registered planning strategies.

The task owns selection, validation and transition reporting. Perception,
behaviour, path generation and control remain normal AVLite strategies.
Requests must originate on the synchronous execution thread or while paused.
"""

import copy
import logging
from collections import deque
from dataclasses import dataclass

from avlite.c20_planning.c23_local_planning_strategy import (
    LocalBehavioralPlanningStrategy,
    LocalPathPlanningStrategy,
    LocalVelocityPlanningStrategy,
)
from avlite.c20_planning.c29_settings import PlanningSettings
from avlite.c40_execution.c43_task_strategy import (
    ExecutionReset,
    ExecutionStarted,
    StackEvent,
    TaskSchedule,
    TaskStrategy,
)
from avlite.c50_common.c51_capabilities import AnyOf, StackCapability

from ..planning.dispatcher import map_mode
from ..settings import PluginSettings
from .switch_task import PlannerSwitchTask, SelectPlanningStack

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class PlanningConfiguration:
    """Names of ordinary AVLite strategies; no simulator or actor handles.

    Extensions can supply additional configurations to the coordinator's
    constructor. Local planners must accept (route, perception_model, setting).
    """

    map_mode: str
    global_planner: str
    local_planner: str
    behavior: str
    path: str
    velocity: str


PLANNING_CONFIGURATIONS = {
    "road-rules": PlanningConfiguration(
        "road",
        "HDMapLaneFollowingPlanner",
        "BehaviorPlanningPipeline",
        "HDMapBehaviorPlanner",
        "ReferencePathPlanner",
        "HDMapRuleVelocityPlanner",
    ),
    "road-tree": PlanningConfiguration(
        "road",
        "HDMapLaneFollowingPlanner",
        "BehaviorPlanningPipeline",
        "HDMapBehaviorTreePlanner",
        "ReferencePathPlanner",
        "HDMapRuleVelocityPlanner",
    ),
    "race": PlanningConfiguration(
        "race",
        "GlobalCenterlineRacePlanner",
        "BehaviorPlanningPipeline",
        "RaceBehaviorPlanner",
        "RacePathPlanner",
        "RaceVelocityPlanner",
    ),
}


@dataclass(frozen=True)
class SelectPlanningStrategy(StackEvent):
    """Select a configuration on this map and route, without changing the world.

    expected_planner protects queued requests against another task's handover.
    The route is copied when submitted via request(), and again before install.
    """

    strategy: str
    global_plan: object
    expected_map: object
    expected_planner: object


@dataclass(frozen=True)
class PlanningStrategySelected(StackEvent):
    strategy: str
    previous: str
    stages: tuple[str, ...]


@dataclass(frozen=True)
class PlanningStrategyRejected(StackEvent):
    strategy: str
    reason: str


class PlanningCoordinatorTask(TaskStrategy):
    """Select on start or explicit domain events; preserve the stack on refusal.

    No periodic re-selection, automatic permission to move, world reload or
    perception reset. Restarting the same stack preserves regulatory dwell
    timers. A scenario monitor may emit SelectPlanningStrategy after stopping.
    """

    schedule = TaskSchedule.ON_EVENT
    listen_events = frozenset({ExecutionStarted, ExecutionReset, SelectPlanningStrategy})
    stack_requirements = frozenset(
        {
            StackCapability.LOCALIZATION,
            AnyOf(StackCapability.MAP_HD, StackCapability.MAP_RACE_TRACK),
        }
    )

    def __init__(self, configurations=None, initial_strategy=None):
        self.configurations = dict(
            PLANNING_CONFIGURATIONS if configurations is None else configurations
        )
        self.initial_strategy = initial_strategy or PluginSettings.coordinator_initial_strategy
        self.reset()

    def reset(self):
        self.active_strategy = None
        self.last_result = None
        self.history = deque(maxlen=100)
        self._installed = None

    def available_strategies(self, executer):
        mode = map_mode(executer.pm.map)
        return ["auto"] + [k for k, v in self.configurations.items() if v.map_mode == mode]

    def request(self, executer, strategy="auto", global_plan=None):
        """Queue through TaskRunner; call from the tick thread or a paused UI."""
        if not getattr(executer, "call_tasks", True):
            raise ValueError("Enable execution tasks before selecting planners")
        if self not in executer.task_runner.tasks:
            raise ValueError("Add PlanningCoordinatorTask to the executor first")
        if global_plan is None:
            if executer.local_planner is None:
                raise ValueError("Load a global route before selecting planners")
            global_plan = executer.local_planner.global_plan
        event = SelectPlanningStrategy(
            strategy, copy.deepcopy(global_plan), executer.pm.map, executer.local_planner
        )
        executer.task_runner.notify(event)

    def select_current_route(self, executer):
        """Compatibility with the visualizer's planner selection action."""
        self.request(executer, self.initial_strategy)
        if self.last_result and self.last_result["status"] == "rejected":
            raise ValueError(self.last_result["reason"])

    def _resolve(self, executer, name):
        mode = map_mode(executer.pm.map)
        if name == "auto":
            name = "race" if mode == "race" else "road-tree"
        config = self.configurations.get(name)
        if config is None:
            raise ValueError(f"Unknown planning configuration: {name}")
        if config.map_mode != mode:
            raise ValueError(
                f"Configuration {name} requires {config.map_mode}, current map is {mode}"
            )
        for registry, stage in (
            (LocalBehavioralPlanningStrategy.registry, config.behavior),
            (LocalPathPlanningStrategy.registry, config.path),
            (LocalVelocityPlanningStrategy.registry, config.velocity),
        ):
            if not stage or stage not in registry:
                raise ValueError(f"Planning stage is not registered: {stage}")
        return name, config

    def execute(self, executer, event=None):
        starting = isinstance(event, ExecutionStarted)
        if isinstance(event, ExecutionReset):
            self.reset()
            return
        if isinstance(event, ExecutionStarted):
            if self._installed is not None:
                if executer.pm.map is not self._installed[0]:
                    raise ValueError(
                        "Map changed; explicitly select a compatible route and strategy"
                    )
                # Another task (e.g. ParkingTask) may own the current planners.
                # Start/resume is not a request to take ownership back.
                if (
                    executer.global_planner is not self._installed[1]
                    or executer.local_planner is not self._installed[2]
                ):
                    self.active_strategy = None
                    self.last_result = None
                return
            if executer.local_planner is None:
                self._reject(executer, self.initial_strategy, "Load a global route before starting")
                raise ValueError("Load a global route before starting")
            event = SelectPlanningStrategy(
                self.initial_strategy,
                executer.local_planner.global_plan,
                executer.pm.map,
                executer.local_planner,
            )
        if not isinstance(event, SelectPlanningStrategy):
            return
        try:
            if event.expected_map is not executer.pm.map:
                raise ValueError("Stale selection: map changed")
            if event.expected_planner is not executer.local_planner:
                raise ValueError("Stale selection: planning stack changed")
            name, config = self._resolve(executer, event.strategy)
            route = copy.deepcopy(event.global_plan)
            if config.map_mode == "road":
                route.race_mode = False
            elif not route.race_mode:
                raise ValueError("Race configuration requires a race-mode route")
            setting = getattr(
                executer.local_planner, "_source_setting", PlanningSettings
            ).model_copy(deep=True)
            setting.c23_behavioral_strategy = config.behavior
            setting.c23_path_strategy = config.path
            setting.c23_velocity_strategy = config.velocity
            switch = PlannerSwitchTask()
            switch.execute(
                executer,
                SelectPlanningStack(
                    route,
                    event.expected_map,
                    setting,
                    config.global_planner,
                    config.local_planner,
                ),
            )
            # Preserve parameters for a subsequent selection from this pipeline.
            executer.local_planner._source_setting = setting
        except Exception as exc:
            # Plugin constructor/inference/validation failures are ordinary refused
            # requests. They are reported, not silently replaced with a fallback.
            self._reject(executer, event.strategy, f"{type(exc).__name__}: {exc}")
            if starting:
                raise ValueError(f"Initial planning selection failed: {exc}") from exc
            return
        self.active_strategy = name
        self._installed = (executer.pm.map, executer.global_planner, executer.local_planner)
        self.last_result = dict(
            status="selected",
            strategy=name,
            time=float(executer.elapsed_sim_time),
            **switch.last_result,
        )
        self.last_result["mode"] = config.map_mode
        self.history.append(dict(self.last_result))
        log.info("COORDINATOR %s", self.last_result)
        executer.task_runner.notify(
            PlanningStrategySelected(
                name, switch.last_result["previous"], switch.last_result["stages"]
            )
        )

    def _reject(self, executer, strategy, reason):
        self.last_result = dict(
            status="rejected",
            strategy=strategy,
            reason=reason,
            time=float(executer.elapsed_sim_time),
        )
        self.history.append(dict(self.last_result))
        log.warning("COORDINATOR %s", self.last_result)
        executer.task_runner.notify(PlanningStrategyRejected(strategy, reason))
