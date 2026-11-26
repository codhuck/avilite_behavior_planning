"""Scenario adapter using AVLite perception, coordination, planning and control.

Actors and observations are scripted, not camera/LiDAR detections. The physical
truth used by the evaluator is never delayed with the perception channel.
"""

import math
from contextlib import contextmanager

import numpy as np
from avlite.c10_perception.c11_perception_model import AgentState, EgoState, HDMap, PerceptionModel
from avlite.c10_perception.c12_perception_strategy import PerceptionStrategy
from avlite.c20_planning.c21_planning_model import GlobalPlan
from avlite.c20_planning.c26_local_path_planners import ReferencePathPlanner
from avlite.c20_planning.c29_settings import PlanningSettings
from avlite.c40_execution.c43_task_strategy import ExecutionStarted
from avlite.c40_execution.c49_settings import ExecutionSettings, ExecutionSettingsSchema
from avlite.c50_common.c51_capabilities import StackCapability
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

from ..execution.coordinator import PlanningCoordinatorTask
from ..execution.runtime import BehaviorStanleyController, PlanningBasicSim, PlanningSyncExecuter
from ..perception.environment import (
    CROSSING_OBSERVATIONS,
    SIGNAL_OBSERVATIONS,
    CrossingOccupancy,
    EnvironmentObservation,
    TrafficSignal,
)
from ..perception.regulatory import RegulatoryKind, extract_regulatory_features
from ..planning.dispatcher import GlobalPlannerDispatcher
from ..settings import PluginSettings
from .evaluation import ScenarioAudit, footprint
from .timeline import actor_states, observation_time, signal_states


@contextmanager
def isolated_settings(spec):
    """Run serially: AVLite settings are process globals, restored even on failure."""
    objects = (PluginSettings, PlanningSettings, ExecutionSettings)
    originals = [obj.model_copy(deep=True) for obj in objects]
    try:
        for obj in objects:
            defaults = type(obj)()
            for key in type(obj).model_fields:
                setattr(obj, key, getattr(defaults, key))
        PluginSettings.cruise_speed_limit = spec.cruise_speed
        PluginSettings.stop_buffer = 2.0
        PluginSettings.required_observation_origin = "scenario"
        ExecutionSettings.c41_world_stack_capabilities = ["LOCALIZATION"]
        ExecutionSettings.c41_world_capabilities = None
        yield
    finally:
        for obj, original in zip(objects, originals):
            for key in type(obj).model_fields:
                setattr(obj, key, getattr(original, key))


class ScriptedScenarioPerception(PerceptionStrategy, abstract=True):
    """Explicit synthetic observation provider; not offered as a GUI strategy."""

    stack_capabilities = frozenset(
        {StackCapability.DETECTION, SIGNAL_OBSERVATIONS, CROSSING_OBSERVATIONS}
    )

    def __init__(self, pm, spec, crossings):
        super().__init__(pm)
        self.spec, self.crossings = spec, crossings

    def perceive(self, perception_model=None, sensors=None):
        pm = self.perception_model if perception_model is None else perception_model
        now = float(pm.simulation_time)
        stamp = observation_time(self.spec, now)
        if stamp is None:
            pm.environment_observation = EnvironmentObservation(
                now, source="scenario", origin="scenario", status="dropout"
            )
            pm.agent_vehicles = []
            return pm
        actors = actor_states(self.spec, stamp)
        markers = [
            TrafficSignal(id="observed-" + key, map_ref=key, state=state)
            for key, state in signal_states(self.spec, stamp).items()
        ]
        markers += [
            CrossingOccupancy(
                id="observed-" + key,
                map_ref=key,
                occupied=any(
                    footprint(a).intersects(area.buffer(1.5))
                    for a in actors
                    if a["kind"] == "pedestrian"
                ),
            )
            for key, area in self.crossings.items()
        ]
        pm.environment_observation = EnvironmentObservation(
            stamp, tuple(markers), source="scenario", origin="scenario"
        )
        pm.agent_vehicles = [
            AgentState(
                x=a["x"],
                y=a["y"],
                theta=a["heading"],
                velocity=a["speed"],
                length=a["length"],
                width=a["width"],
            )
            for a in actors
        ]
        return pm


class BasicSimScenario:
    """One fresh AVLite stack per run. No reuse of user profiles or live simulator."""

    def __init__(self, spec):
        self.spec = spec
        self.map = HDMap.from_path(spec.map)
        driving = [lane for lane in self.map.lanes if lane.type == "driving"]
        if len(self.map.roads) != 1 or len(driving) != 1:
            raise ValueError("This BasicSim adapter supports one straight driving lane")
        geometry = self.map.roads[0].road_element.findall("planView/geometry")
        widths = driving[0].lane_element.findall("width")
        if (
            not geometry
            or any(g.find("line") is None for g in geometry)
            or len(widths) != 1
            or any(float(widths[0].get(key, "0")) != 0 for key in ("b", "c", "d"))
        ):
            raise ValueError("This BasicSim adapter requires a straight, constant-width lane")
        self.route = GlobalPlan.from_file(spec.route)
        path = self.route.path
        if len(path) < 2 or not np.isfinite(np.asarray(path)).all():
            raise ValueError("Scenario needs a finite route with at least two points")
        self.line = LineString(path)
        if not self.line.is_simple or self.line.length <= 0:
            raise ValueError("BasicSim road scenarios require a non-self-intersecting route")
        features = extract_regulatory_features(self.map)
        self.crossings = {
            f.id: Polygon(f.polygon) for f in features if f.kind == RegulatoryKind.CROSSWALK
        }
        mapped_signals = {f.id for f in features if f.kind == RegulatoryKind.TRAFFIC_LIGHT}
        if mapped_signals != {s.id for s in spec.signals}:
            raise ValueError("Specify exactly the traffic-light IDs in the scenario map")
        if any(f.kind == RegulatoryKind.STOP_SIGN for f in features):
            raise ValueError("STOP-sign dwell auditing is not supported by this adapter yet")
        # Lane width is the parser's constant-width approximation; no claim for
        # exact OpenDRIVE tapered boundaries. Generated fixtures have fixed widths.
        self.drivable = unary_union(
            [
                LineString(lane.center_line.T).buffer(lane.width / 2, cap_style=2)
                for lane in self.map.lanes
                if lane.type == "driving" and lane.width > 0
            ]
        )
        if self.drivable.is_empty or not self.drivable.covers(self.line):
            raise ValueError("Route must lie in the map's driving lanes")
        self.lights = {
            s.id: self.line.project(Point(f.x, f.y))
            for s in spec.signals
            for f in features
            if f.id == s.id and f.kind == RegulatoryKind.TRAFFIC_LIGHT
        }

    def run(self, emit):
        with isolated_settings(self.spec):
            x, y = self.route.path[0]
            heading = math.atan2(self.route.path[1][1] - y, self.route.path[1][0] - x)
            pm = PerceptionModel(map=self.map, ego_vehicle=EgoState(x=x, y=y, theta=heading))
            coordinator = PlanningCoordinatorTask(initial_strategy=self.spec.strategy)
            world = PlanningBasicSim(
                pm.ego_vehicle,
                pm=pm,
                map=self.map,
                setting=ExecutionSettingsSchema(c46_npc_control=False),
            )
            ex = PlanningSyncExecuter(
                pm,
                perception=ScriptedScenarioPerception(pm, self.spec, self.crossings),
                world=world,
                global_planner=GlobalPlannerDispatcher(self.map),
                local_planner=ReferencePathPlanner(self.route, pm),
                controller=BehaviorStanleyController(),
                tasks=[coordinator],
            )
            audit = ScenarioAudit(self.spec, self.line, self.crossings, self.lights, self.drivable)
            try:
                ex.task_runner.notify(ExecutionStarted())
                if (
                    coordinator.last_result is None
                    or coordinator.last_result["status"] != "selected"
                ):
                    raise RuntimeError(f"Scenario planning setup failed: {coordinator.last_result}")
                for tick in range(round(self.spec.duration / self.spec.dt) + 1):
                    now = tick * self.spec.dt
                    ego = world.get_ego_state()
                    local = ex.local_planner.get_local_plan()
                    frame = dict(
                        time=now,
                        ego=dict(
                            x=float(ego.x),
                            y=float(ego.y),
                            heading=float(ego.theta),
                            speed=float(ego.velocity),
                            length=float(ego.length),
                            width=float(ego.width),
                        ),
                        actors=actor_states(self.spec, now),
                        signals=signal_states(self.spec, now),
                        behavior=getattr(local, "behavior_state", "INITIAL"),
                        reason=getattr(local, "behavior_reason", "initial"),
                        observation_origin=getattr(
                            getattr(pm, "environment_observation", None), "origin", None
                        ),
                        observation_status=getattr(
                            getattr(pm, "environment_observation", None), "status", None
                        ),
                        observation_time=getattr(
                            getattr(pm, "environment_observation", None), "time", None
                        ),
                    )
                    audit.update(frame)
                    emit(frame)
                    if audit.metrics["completed"] or tick == round(
                        self.spec.duration / self.spec.dt
                    ):
                        break
                    ex.step(
                        sim_dt=self.spec.dt,
                        perception_dt=self.spec.dt,
                        replan_dt=self.spec.dt,
                        control_dt=self.spec.dt,
                        pace_perception=False,
                        pace_replan=False,
                        pace_control=False,
                    )
                return dict(
                    **audit.result(),
                    sim_seconds=now,
                    selection=coordinator.last_result,
                    scope="AVLite BasicSim; scripted observations and actor motion; no sensor accuracy claim",
                )
            finally:
                ex.stop()
