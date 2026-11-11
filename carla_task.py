"""CARLA ground-truth adapter and per-cycle diagnostic recording.

This is a simulation observation source, not camera-based perception. It never
ticks the world, changes lights or applies vehicle controls.
"""

import csv
import json
import logging
import uuid
from datetime import datetime, timezone
from pathlib import Path

from avlite.c40_execution.c43_task_strategy import TaskStrategy

from .observations import RegulatoryObservation
from .settings import PluginSettings

log = logging.getLogger(__name__)


def observe_carla(world, simulation_time):
    actors = world.get_actors()
    lights = {}
    for actor in actors.filter("traffic.traffic_light*"):
        lights[str(actor.get_opendrive_id())] = str(actor.get_state()).split(".")[-1]
    pedestrians = []
    for actor in actors.filter("walker.pedestrian.*"):
        p = actor.get_location()
        pedestrians.append((p.x, -p.y))
    return RegulatoryObservation(float(simulation_time), lights, pedestrians)


class CarlaRegulatoryTask(TaskStrategy):
    """Refresh observations after each tick and record decisions/actual motion."""

    use_ground_truth = True

    def __init__(self):
        self.reset()

    def reset(self):
        self.run_path = None
        self._event_cursor = 0
        self._generation = None

    def execute(self, executer, event=None):
        executer.pm.simulation_time = float(executer.elapsed_sim_time)
        world = getattr(executer.world, "world", None)
        if world is None:
            return
        if self.use_ground_truth:
            if (
                getattr(getattr(executer.pm, "regulatory_observation", None), "source", None)
                == "rgb_lidar"
            ):
                raise RuntimeError(
                    "CarlaRegulatoryTask cannot overwrite sensor perception; select SensorTelemetryTask"
                )
            try:
                executer.pm.regulatory_observation = observe_carla(world, executer.elapsed_sim_time)
            except RuntimeError as exc:
                executer.pm.regulatory_observation = None
                log.warning("Regulatory observations unavailable: %s", exc)
        local = executer.local_planner
        if local is None:
            return
        plan = local.get_local_plan()
        if not hasattr(plan, "behavior_state"):
            return
        behavior = getattr(local, "_behavioral", None)
        generation = (id(local), id(behavior), getattr(behavior, "generation", None))
        if generation != self._generation:
            self.reset()
            self._generation = generation
        ego = executer.world.get_ego_state()
        tj = plan.as_trajectory()
        tj.update_waypoint_by_xy(ego.x, ego.y)
        cmd = getattr(executer, "_last_cmd", None)
        s, d = local.global_trajectory.convert_xy_to_sd(ego.x, ego.y)
        row = dict(
            time=executer.elapsed_sim_time,
            decision_time=plan.decision_time,
            x=ego.x,
            y=ego.y,
            speed=ego.velocity,
            route_s=s,
            lateral_error=d,
            target_speed=float(tj.velocity[tj.current_wp]),
            acceleration=getattr(cmd, "acceleration", None),
            state=plan.behavior_state,
            reason=plan.behavior_reason,
            phase=plan.behavior_phase,
            wait_duration=plan.wait_duration,
            planner_mode=getattr(plan, "planner_mode", "road"),
            path_planner=getattr(plan, "path_planner", ""),
            rule_id=plan.rule_id,
            rule_distance=plan.rule_distance,
            observation_source=getattr(
                executer.pm.regulatory_observation, "source", "ground_truth"
            ),
            lights=json.dumps(getattr(executer.pm.regulatory_observation, "lights", {})),
            pedestrian_count=len(getattr(executer.pm.regulatory_observation, "pedestrians", [])),
        )
        if self.run_path is None:
            folder = Path(PluginSettings.telemetry_directory)
            folder.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            self.run_path = folder / f"drive_{stamp}_{uuid.uuid4().hex[:6]}"
            with self.run_path.with_suffix(".csv").open("w", newline="") as f:
                csv.DictWriter(f, fieldnames=list(row)).writeheader()
            log.info("Behavior telemetry: %s.csv", self.run_path)
        with self.run_path.with_suffix(".csv").open("a", newline="") as f:
            csv.DictWriter(f, fieldnames=list(row)).writerow(row)
        events = getattr(behavior, "events", [])
        if self._event_cursor > len(events):
            self._event_cursor = 0
        if events[self._event_cursor :]:
            with self.run_path.with_suffix(".jsonl").open("a") as f:
                for record in events[self._event_cursor :]:
                    f.write(json.dumps(record) + "\n")
            self._event_cursor = len(events)


class SensorTelemetryTask(CarlaRegulatoryTask):
    """Record existing perception decisions without querying dynamic actors."""

    use_ground_truth = False
