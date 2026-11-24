"""Synchronous parking handover and fresh occupancy delivery."""

import copy
import json
import math
from dataclasses import dataclass
from pathlib import Path

from avlite.c10_perception.c11_perception_model import AgentState
from avlite.c30_control import c31_control_model
from avlite.c40_execution.c43_task_strategy import TaskStrategy
from avlite.c40_execution.c44_sync_executer import SyncExecuter
from avlite.c40_execution.c46_basic_sim import BasicSim
from shapely.geometry import Polygon

from ..parking.mission_observation import MissionLidarObservation
from ..parking.occupancy import FusedLidarParkingOccupancy, LidarParkingOccupancy
from ..parking.planner import ParkingController, ParkingPlanner, ParkingRequest, ParkingType
from ..parking.visibility import VisibilityParkingPlanner
from ..settings import PluginSettings


@dataclass(frozen=True)
class ParkingObservation:
    """Perception producer supplies complete occupied polygons and acquisition time.

    None means unknown. An empty tuple is an explicitly observed clear region.
    This contract is not satisfied by merely receiving zero neural detections.
    """

    polygons: tuple | None
    stamp: float


class ParkingTask(TaskStrategy):
    """User-selected parking, with synchronous stopped handover only."""

    parking_types = tuple(mode.value for mode in ParkingType)

    def __init__(self):
        self.reset()

    def reset(self):
        world = getattr(self, "_world", None)
        owned = getattr(self, "_demo_agents", [])
        if world is not None and isinstance(world, BasicSim):
            world.pm.agent_vehicles[:] = [
                a for a in world.pm.agent_vehicles if not any(a is own for own in owned)
            ]
        self._demo_agents = []
        self._world = None
        self.last_result = None
        self.sensor_view = None

    def parking_spaces(self):
        path = Path(PluginSettings.parking_scene_file).expanduser()
        if not PluginSettings.parking_scene_file or not path.is_file():
            raise ValueError("Set parking_scene_file to a surveyed parking-area JSON")
        data = json.loads(path.read_text())
        if data.get("version") != 1 or not data.get("spaces"):
            raise ValueError("Parking scene must have version 1 and named spaces")
        return data

    @staticmethod
    def lidar_provider():
        if PluginSettings.parking_simulator_obstacles:
            raise ValueError("Select either LiDAR occupancy or simulator occupancy")
        ground = PluginSettings.parking_ground_z
        if ground is None or not math.isfinite(ground):
            raise ValueError("Set finite surveyed parking_ground_z for the flat parking area")
        return (
            FusedLidarParkingOccupancy(
                minimum_support=PluginSettings.parking_minimum_sensor_support,
                hazard_support_radius=PluginSettings.parking_hazard_support_radius,
            )
            if PluginSettings.parking_multi_lidar
            else LidarParkingOccupancy()
        )

    def configure_mission_lidar(self, scene):
        self.sensor_view = MissionLidarObservation(
            self.lidar_provider(), Polygon(scene["drivable"]), PluginSettings.parking_ground_z
        )

    def observe_sensors(self, ex, sensors):
        if self.sensor_view is None:
            return
        self.sensor_view.observe(sensors, ex.pm.ego_vehicle, float(ex.elapsed_sim_time))
        ex.pm.parking_occupancy = self.sensor_view.occupancy
        if self._world is ex.world and isinstance(ex.local_planner, ParkingPlanner):
            self.sensor_view.apply(ex.local_planner, float(ex.elapsed_sim_time))

    def space_statuses(self, executer, parking_type):
        """Classify only from the selected source; unknown never means clear."""
        mode = ParkingType(parking_type).value
        spaces = {
            name: Polygon(space["bay"])
            for name, space in self.parking_spaces()["spaces"].items()
            if mode in space["types"]
        }
        now = float(executer.elapsed_sim_time)
        if self.sensor_view is not None:
            return {name: self.sensor_view.classify(bay, now) for name, bay in spaces.items()}
        if PluginSettings.parking_lidar_occupancy:
            # Manual parking owns its observer inside the planner. Automatic
            # missions use sensor_view above, including before handover.
            planner = executer.local_planner
            occupancy = getattr(executer.pm, "parking_occupancy", None)
            stamp = getattr(planner, "_observed_time", None)
            if (
                isinstance(planner, ParkingPlanner)
                and planner.sensor_occupancy is not None
                and planner.obstacles is not None
                and occupancy is not None
                and occupancy.status == "observed"
                and stamp is not None
                and 0 <= now - stamp <= 0.3
            ):
                return {
                    name: "occupied"
                    if occupancy.occupied is not None
                    and occupancy.occupied.intersects(bay.buffer(0.15))
                    else "free"
                    if occupancy.permits(bay)
                    else "unknown"
                    for name, bay in spaces.items()
                }
            return {name: "missing_or_stale_lidar" for name in spaces}
        observation = self.observation(executer)
        if observation is None or observation.polygons is None:
            reason = "unknown"
        elif not math.isfinite(observation.stamp):
            reason = "invalid_observation"
        elif not 0 <= now - observation.stamp <= 0.3:
            reason = "stale_or_future_observation"
        elif any(p.is_empty or not p.is_valid for p in observation.polygons):
            reason = "invalid_observation"
        else:
            return {
                name: "occupied"
                if any(bay.buffer(0.15).intersects(p) for p in observation.polygons)
                else "free"
                for name, bay in spaces.items()
            }
        return {name: reason for name in spaces}

    def available_spaces(self, executer, parking_type):
        return [
            name
            for name, status in self.space_statuses(executer, parking_type).items()
            if status == "free"
        ]

    def start_parking(self, executer, parking_type, space_id, *, require_feasible=False):
        if not isinstance(executer, SyncExecuter):
            raise ValueError("Parking requires SyncExecuter")
        if not executer.stopped and not getattr(executer.task_runner, "_in_step", False):
            raise ValueError("Pause execution before selecting a parking manoeuvre")
        for ego in (executer.pm.ego_vehicle, executer.world.get_ego_state()):
            if not all(math.isfinite(v) for v in (ego.x, ego.y, ego.theta, ego.velocity)):
                raise ValueError("Parking handover requires finite localization")
            if abs(ego.velocity) > 0.02:
                raise ValueError("Stop the vehicle before parking handover")
        if not isinstance(executer.world, BasicSim) and not callable(
            getattr(executer.world, "parking_geometry", None)
        ):
            raise ValueError("World bridge does not support directional parking")
        if (
            isinstance(executer.world, BasicSim)
            and not getattr(executer.world, "supports_directional_parking", False)
            and not hasattr(c31_control_model, "DirectionalAckermannCommand")
        ):
            raise ValueError("Select PlanningBasicSim for directional parking on stock AVLite")
        data = self.parking_spaces()
        space = data["spaces"][space_id]
        mode = ParkingType(parking_type)
        if mode.value not in space["types"]:
            raise ValueError("This space is not configured for the selected parking type")
        goal = space["goals"][mode.value]
        request = ParkingRequest(
            tuple(goal), Polygon(space["bay"]), Polygon(data["drivable"]), mode
        )
        route = executer.local_planner.global_plan
        planner_type = (
            VisibilityParkingPlanner
            if PluginSettings.parking_active_observation
            else ParkingPlanner
        )
        if PluginSettings.parking_active_observation and not PluginSettings.parking_lidar_occupancy:
            raise ValueError("Active observation requires sensor occupancy")
        planner = planner_type(route, executer.pm)
        controller = ParkingController()
        if callable(getattr(executer.world, "parking_geometry", None)):
            geometry = executer.world.parking_geometry()
            controller.ego_distance_front_axle = geometry["wheelbase"]
            controller.rear_offset = geometry["rear_offset"]
            planner.rear_offset = geometry["rear_offset"]
        if PluginSettings.parking_lidar_occupancy:
            provider = self.lidar_provider()
            if require_feasible:
                if self.sensor_view is None:
                    raise ValueError("Mission LiDAR observation is not configured")
                if PluginSettings.parking_active_observation:
                    raise ValueError("Mission selection requires a fully observed parking path")
            else:
                planner.sensor_occupancy = provider
                planner.sensor_ground_z = PluginSettings.parking_ground_z
        planner.request_parking(request)
        if require_feasible:
            if data.get("demo"):
                raise ValueError("Mission obstacles must exist before selecting a space")
            executer.pm.simulation_time = executer.elapsed_sim_time
            if PluginSettings.parking_lidar_occupancy:
                status = self.sensor_view.classify(request.bay, executer.elapsed_sim_time)
                if status != "free":
                    raise ValueError("Parking space is " + status)
                self.sensor_view.apply(planner, executer.elapsed_sim_time)
            else:
                observation = self.observation(executer)
                if observation is None or observation.polygons is None:
                    raise ValueError("Parking handover requires a complete occupancy observation")
                if not 0 <= executer.elapsed_sim_time - observation.stamp <= 0.3:
                    raise ValueError("Parking observation is stale or from the future")
                planner.update_obstacles(observation.polygons, observation.stamp)
            planner.replan()
            if planner.state == "PARKING_BLOCKED":
                raise ValueError(planner.reason)
        candidate = copy.copy(executer)
        candidate.local_planner, candidate.controller = planner, controller
        candidate.validate_stack()
        executer.local_planner, executer.controller = planner, controller
        if isinstance(executer.world, BasicSim):
            executer.world.ego_controller = controller
        else:
            executer.world.controller = controller
        executer._last_cmd = None
        invalidate = getattr(executer, "invalidate_planning", None)
        if callable(invalidate):
            invalidate()
        sensor_view = self.sensor_view if require_feasible else None
        self.reset()
        self.sensor_view = sensor_view
        self._world = executer.world
        if data.get("demo") and PluginSettings.parking_simulator_obstacles:
            for fields in space.get("parked_vehicles", []):
                agent = AgentState(**fields)
                self._world.pm.add_agent_vehicle(agent)
                self._demo_agents.append(agent)
        self.last_result = {"space": space_id, "type": mode.value}

    def cancel_parking(self, executer):
        if isinstance(executer.local_planner, ParkingPlanner):
            executer.local_planner.cancel()
            executer._last_cmd = None

    def observation(self, executer):
        """Return the producer's observation, or explicit BasicSim demo geometry."""
        observation = getattr(executer.pm, "parking_observation", None)
        if PluginSettings.parking_simulator_obstacles:
            if not isinstance(executer.world, BasicSim):
                raise ValueError("Simulator parking occupancy is restricted to BasicSim")
            agents = executer.world.pm.agent_vehicles + executer.world.pm.static_obstacles
            observation = ParkingObservation(
                tuple(Polygon(a.get_bb_corners()) for a in agents), executer.elapsed_sim_time
            )
        return observation

    def execute(self, executer, event=None):
        planner = executer.local_planner
        if not isinstance(planner, ParkingPlanner):
            return
        if self._world is not None and self._world is not executer.world:
            planner.cancel()
            return
        self._world = executer.world
        executer.pm.simulation_time = executer.elapsed_sim_time
        if self.sensor_view is not None:
            self.sensor_view.apply(planner, executer.elapsed_sim_time)
            return
        observation = self.observation(executer)
        if observation is not None:
            planner.update_obstacles(observation.polygons, observation.stamp)
