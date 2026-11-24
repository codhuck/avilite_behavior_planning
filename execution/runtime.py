"""Registered adapters for unmodified AVLite; no monkey patches or core writes."""

import inspect
import math

import numpy as np
from avlite.c20_planning.c23_local_planning_strategy import (
    LocalPlanningPipeline,
    LocalPlanningStrategy,
)
from avlite.c30_control.c31_control_model import ControlCommand
from avlite.c30_control.c34_stanley import StanleyController
from avlite.c40_execution.c43_task_strategy import ExecutionStarted
from avlite.c40_execution.c44_sync_executer import SyncExecuter
from avlite.c40_execution.c46_basic_sim import BasicSim

from .commands import DirectionalAckermannCommand

_STEP_SIGNATURE = inspect.signature(SyncExecuter.step)


class BehaviorPlanningPipeline(LocalPlanningPipeline):
    """Propagate route, reset and perception to every registered stage."""

    @property
    def stages(self):
        """Public access for coordination, telemetry and capability inspection."""
        return self._stages

    def bind_perception(self, perception_model):
        self.pm = perception_model
        for stage in self.stages:
            for attr in ("pm", "env"):
                if hasattr(stage, attr):
                    setattr(stage, attr, perception_model)
            guard = getattr(stage, "guard", None)
            if guard is not None and hasattr(guard, "env"):
                guard.env = perception_model

    def set_global_plan(self, global_plan, ego_xy=None):
        LocalPlanningStrategy.set_global_plan(self, global_plan, ego_xy=ego_xy)
        for stage in self._stages:
            update = getattr(stage, "set_global_plan", None)
            if callable(update):
                update(global_plan, ego_xy=ego_xy)
        self._working_plan = None

    def reset(self, wp=0):
        LocalPlanningStrategy.reset(self, wp)
        for stage in self._stages:
            reset = getattr(stage, "reset", None)
            if callable(reset):
                reset(wp)
        self._working_plan = None

    def replan(self, perception_model=None, sensors=None):
        if perception_model is not None:
            for stage in self._stages:
                for attr in ("pm", "env"):
                    if hasattr(stage, attr):
                        setattr(stage, attr, perception_model)
        return super().replan(perception_model, sensors)


class PlanningSyncExecuter(SyncExecuter):
    """Provide startup events, simulation timestamps and fresh handover ticks."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._planning_start_sent = False
        self._fresh_planning = False
        self._fresh_perception = False
        self.motion_holds = {}

    def hold_motion(self, owner, reason):
        """Latch braking across paced control ticks until the owner releases it."""
        self.motion_holds[owner] = reason

    def release_motion(self, owner):
        if owner in self.motion_holds:
            del self.motion_holds[owner]
            self.invalidate_planning()

    def _replan_step(self, sensors):
        if self.call_tasks:
            for task in self.task_runner.tasks:
                before_replan = getattr(task, "before_replan", None)
                if callable(before_replan):
                    before_replan(self, sensors)
        return super()._replan_step(sensors)

    def _simulate_step(self, dt):
        if self.motion_holds:
            previous = self._last_cmd
            if isinstance(previous, DirectionalAckermannCommand):
                self._last_cmd = DirectionalAckermannCommand(
                    steer=previous.steer,
                    acceleration=-3.0,
                    direction=previous.direction,
                    hold_brake=True,
                )
            else:
                self._last_cmd = ControlCommand(
                    steer=getattr(previous, "steer", 0.0),
                    acceleration=-3.0,
                )
        return super()._simulate_step(dt)

    def invalidate_planning(self):
        self._fresh_planning = True
        parent = getattr(super(), "invalidate_planning", None)
        if parent is not None:
            parent()
        else:
            self._last_cmd = None

    def refresh_pipeline(self):
        if not self.stopped:
            raise ValueError("Pause execution before refreshing the pipeline")
        parent = getattr(super(), "refresh_pipeline", None)
        if parent is not None:
            parent()
        else:
            self.invalidate_planning()
        self._fresh_perception = True

    def step(self, *args, **kwargs):
        bound = _STEP_SIGNATURE.bind(self, *args, **kwargs)
        bound.apply_defaults()
        kwargs = dict(bound.arguments)
        del kwargs["self"]
        self.call_tasks = kwargs["call_tasks"]
        if not self._planning_start_sent and kwargs.get("call_tasks", True):
            start = getattr(super(), "start_execution", None)
            if start is not None:
                start()
            else:
                self.task_runner.notify(ExecutionStarted())
            self._planning_start_sent = True
        for task in self.task_runner.tasks:
            before_step = getattr(task, "before_step", None)
            if callable(before_step):
                before_step(self, tasks_enabled=kwargs["call_tasks"])
        # These overrides apply only to the next enabled stage, not subsequent
        # timing policy. Clear before super so an event during the tick persists.
        if self._fresh_planning:
            kwargs["pace_replan"] = False
            kwargs["pace_control"] = False
            if kwargs.get("call_replan", True) and kwargs.get("call_control", True):
                self._fresh_planning = False
        if self._fresh_perception:
            kwargs["pace_perception"] = False
            kwargs["localization_dt"] = 0.0
            if kwargs.get("call_perceive", True) and kwargs.get("call_localize", True):
                self._fresh_perception = False
        self.pm.simulation_time = float(self.elapsed_sim_time)
        return super().step(**kwargs)

    def reset(self):
        for task in self.task_runner.tasks:
            prepare_reset = getattr(task, "prepare_reset", None)
            if callable(prepare_reset):
                prepare_reset(self)
        super().reset()
        self._planning_start_sent = False
        self._fresh_planning = True
        self._fresh_perception = True
        self.motion_holds = {}
        self.pm.environment_observation = None
        self.pm.regulatory_observation = None


class BehaviorStanleyController(StanleyController):
    """Keep the standard steering law; zero target speed never adds throttle."""

    def control(self, ego, plan=None, control_dt=None, perception_model=None, sensors=None):
        command = super().control(
            ego,
            plan=plan,
            control_dt=control_dt,
            perception_model=perception_model,
            sensors=sensors,
        )
        if self.tj is not None and self.tj.velocity[self.tj.current_wp] <= 0:
            self.cte_v_sum = 0.0
            command.acceleration = min(0.0, command.acceleration)
        return command


class PlanningBasicSim(BasicSim):
    """BasicSim with an explicit signed parking command and standstill floor."""

    supports_directional_parking = True

    def control_ego_state(self, cmd, dt=0.01):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Control timestep must be positive")
        if not isinstance(cmd, DirectionalAckermannCommand):
            super().control_ego_state(cmd, dt)
            # Negative acceleration is braking; reverse requires direction=-1.
            self.ego_state.velocity = max(0.0, self.ego_state.velocity)
            self.ego_state.longitudinal_velocity = self.ego_state.velocity
            return
        previous = getattr(self, "_drive_direction", 1)
        speed = abs(self.ego_state.velocity)
        changing = previous != cmd.direction and speed > 0.02
        direction = previous if changing else cmd.direction
        acceleration = -2.0 if changing or cmd.hold_brake else cmd.acceleration
        new_speed = max(0.0, speed + acceleration * dt)
        distance = direction * (speed + new_speed) * 0.5 * dt
        wheelbase = self.ego_controller.ego_distance_front_axle if self.ego_controller else 2.5
        turn = distance / wheelbase * math.tan(float(np.clip(cmd.steer, -0.6, 0.6)))
        self.ego_state.x += distance * math.cos(self.ego_state.theta + turn / 2)
        self.ego_state.y += distance * math.sin(self.ego_state.theta + turn / 2)
        self.ego_state.theta += turn
        self.ego_state.velocity = new_speed
        self.ego_state.longitudinal_velocity = direction * new_speed
        self._drive_direction = direction

    def reset(self):
        super().reset()
        self._drive_direction = 1
        self.ego_state.longitudinal_velocity = None
