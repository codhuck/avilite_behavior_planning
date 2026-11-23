"""Static mission checks. No world, executor or sensor is constructed."""

import json
import math
from pathlib import Path

import yaml
from avlite.c60_apps.c68_paths import DataPaths
from shapely.geometry import LineString, Point, Polygon

from ..settings import PluginSettingsSchema
from .mission import ParkingMission


def _finite(values):
    return all(not isinstance(v, bool) and math.isfinite(float(v)) for v in values)


def _polygon(points):
    if not isinstance(points, list) or len(points) < 3:
        raise ValueError("Expected at least three [x, y] points")
    if any(len(p) != 2 or not _finite(p) for p in points):
        raise ValueError("Polygon coordinates must be finite [x, y] points")
    polygon = Polygon(points)
    if not polygon.is_valid or polygon.is_empty or polygon.area <= 0:
        raise ValueError("Polygon must be valid and have positive area")
    return polygon


def diagnose_mission(profile_path):
    """Check a complete YAML profile using the runtime's path conventions.

    Mission/scene paths are relative to the working directory. Route paths use
    AVLite DataPaths, including its user-data and bundled-data lookup. Absolute
    paths are recommended for generated profiles. Live observations, braking
    performance and parking-path feasibility require separate runtime checks.
    """
    report = {
        "profile": str(Path(profile_path).expanduser().resolve()),
        "working_directory": str(Path.cwd()),
        "files": {},
        "errors": [],
        "warnings": [],
    }

    def error(field, message):
        report["errors"].append({"field": field, "message": str(message)})

    def warn(field, message):
        report["warnings"].append({"field": field, "message": str(message)})

    def read_file(field, filename, *, route=False, yaml_file=False):
        try:
            if not isinstance(filename, str) or not filename.strip():
                raise ValueError("A nonempty file path is required")
            path = Path(DataPaths.resolve_stored(filename) if route else filename).expanduser()
            report["files"][field] = str(path.resolve())
            text = path.read_text()
            data = yaml.safe_load(text) if yaml_file else json.loads(text)
            if not isinstance(data, dict):
                raise ValueError("Expected a mapping/object at the document root")
            return data
        except (OSError, UnicodeError, ValueError, yaml.YAMLError) as exc:
            error(field, exc)
            return None

    def finish():
        report["passed"] = not report["errors"]
        return report

    profile = read_file("profile", str(profile_path), yaml_file=True)
    if profile is None:
        return finish()
    execution = profile.get("c40_execution")
    if not isinstance(execution, dict):
        error("c40_execution", "Expected an explicit execution settings mapping")
        execution = {}
    plugins = profile.get("plugins", {})
    settings = plugins.get("avlite_behavior_hdmap") if isinstance(plugins, dict) else None
    if not isinstance(settings, dict):
        error("plugins.avlite_behavior_hdmap", "Expected plugin settings mapping")
        settings = {}
    try:
        parsed = PluginSettingsSchema.model_validate(settings)
    except ValueError as exc:
        error("plugins.avlite_behavior_hdmap", exc)
        parsed = None
    unknown = set(settings) - set(PluginSettingsSchema.model_fields)
    if unknown:
        error(
            "plugins.avlite_behavior_hdmap",
            f"Unknown settings: {', '.join(sorted(map(str, unknown)))}",
        )
    if "c40_start_pose" in execution:
        try:
            pose = execution["c40_start_pose"]
            if len(pose) != 3 or not _finite(pose):
                raise ValueError("Start pose must be finite [x, y, yaw]")
        except (ValueError, TypeError) as exc:
            error("c40_start_pose", exc)
    if execution.get("c40_executer_type") != "PlanningSyncExecuter":
        error("c40_executer_type", "ParkingMissionTask requires PlanningSyncExecuter")
    tasks = execution.get("c40_execution_tasks", [])
    if not isinstance(tasks, list) or any(not isinstance(t, str) for t in tasks):
        error("c40_execution_tasks", "Expected a list of task names")
        tasks = []
    if tasks.count("ParkingMissionTask") != 1:
        error("c40_execution_tasks", "Include exactly one ParkingMissionTask")
    if len(tasks) != len(set(tasks)):
        error("c40_execution_tasks", "Duplicate tasks are not supported")
    for field in ("c40_bridge", "c40_local_planner", "c40_controller"):
        if not isinstance(execution.get(field), str) or not execution[field].strip():
            error(field, "Specify the approach stack explicitly")
    apps = profile.get("c69_apps", {})
    registration = apps.get("c62_community_plugins", {}) if isinstance(apps, dict) else {}
    if not isinstance(registration, dict) or not registration.get("avlite-behavior-hdmap"):
        error("c62_community_plugins", "Register avlite-behavior-hdmap in this profile")
    if isinstance(apps, dict) and apps.get("c62_load_plugins") is False:
        error("c62_load_plugins", "Plugin loading is disabled")
    if parsed is not None:
        if parsed.parking_lidar_occupancy or parsed.parking_active_observation:
            error(
                "parking_lidar_occupancy",
                "Automatic mission selection requires complete ParkingObservation, not partial LiDAR coverage",
            )
        if parsed.parking_simulator_obstacles and execution.get("c40_bridge") != "PlanningBasicSim":
            error("parking_simulator_obstacles", "Simulator occupancy requires PlanningBasicSim")
        if "ParkingDemoTrafficTask" in tasks:
            if (
                not parsed.parking_simulator_obstacles
                or execution.get("c40_bridge") != "PlanningBasicSim"
            ):
                error(
                    "ParkingDemoTrafficTask",
                    "Demo traffic requires PlanningBasicSim and simulator occupancy",
                )
        if not parsed.parking_simulator_obstacles:
            warn(
                "parking_observation",
                "A live producer must supply fresh complete ParkingObservation; this check cannot verify it",
            )
        if parsed.decision_status_file and "DecisionStatusTask" not in tasks:
            warn("decision_status_file", "No DecisionStatusTask is configured to write this file")

    mission_data = read_file("mission_file", settings.get("mission_file"))
    mission = None
    if mission_data is not None:
        try:
            mission = ParkingMission.from_file(report["files"]["mission_file"])
            if mission.wait_timeout <= mission.settle_time:
                error(
                    "wait_timeout",
                    "Must exceed settle_time so a continuous standstill can be accepted",
                )
        except (KeyError, TypeError, ValueError, OSError) as exc:
            error("mission_file", f"Invalid mission: {exc}")

    scene = read_file("parking_scene_file", settings.get("parking_scene_file"))
    area = None
    if scene is not None:
        if scene.get("version") != 1:
            error("parking_scene_file.version", "Expected version 1")
        if scene.get("demo"):
            error(
                "parking_scene_file.demo",
                "Mission obstacles must exist before space selection; disable the manual-handover demo flag",
            )
        try:
            area = _polygon(scene.get("drivable"))
        except (ValueError, TypeError) as exc:
            error("parking_scene_file.drivable", exc)
        spaces = scene.get("spaces")
        if not isinstance(spaces, dict) or not spaces:
            error("parking_scene_file.spaces", "Expected named surveyed parking spaces")
            spaces = {}
        for name in mission.spaces if mission else ():
            field = f"spaces.{name}"
            try:
                space = spaces[name]
                if not isinstance(space, dict) or mission.parking_type not in space.get(
                    "types", []
                ):
                    raise ValueError(f"Space does not support {mission.parking_type}")
                bay = _polygon(space["bay"])
                goal = space["goals"][mission.parking_type]
                if len(goal) != 3 or not _finite(goal):
                    raise ValueError("Goal must be finite [x, y, yaw]")
                if not bay.covers(Point(float(goal[0]), float(goal[1]))):
                    raise ValueError("Goal centre lies outside the bay")
                if area is not None and not area.covers(bay):
                    raise ValueError("Bay extends outside the drivable area")
            except (KeyError, ValueError, TypeError) as exc:
                error(field, f"Invalid or missing parking space: {exc}")

    route = read_file("c40_global_trajectory", execution.get("c40_global_trajectory"), route=True)
    if route is not None:
        try:
            points = route["ReferenceLine"]
            if not isinstance(points, list) or len(points) < 2:
                raise ValueError("ReferenceLine requires at least two points")
            if any(len(p) < 2 or not _finite(p) for p in points):
                raise ValueError("Route points must have finite x/y coordinates")
            xy = [(float(p[0]), float(p[1])) for p in points]
            if any(a == b for a, b in zip(xy, xy[1:])):
                raise ValueError("Adjacent route points must be distinct")
            for field in ("ReferenceSpeed", "LeftBound", "RightBound"):
                values = route[field]
                if (
                    not isinstance(values, list)
                    or len(values) != len(points)
                    or not _finite(values)
                ):
                    raise ValueError(f"{field} must have one finite value per route point")
            if any(float(v) < 0 for v in route["ReferenceSpeed"]):
                raise ValueError("Approach speeds must be nonnegative")
            if any(float(a) <= float(b) for a, b in zip(route["LeftBound"], route["RightBound"])):
                raise ValueError("LeftBound must be greater than RightBound")
            line = LineString(xy)
            if mission is not None:
                approach = Point(*mission.approach[:2])
                if line.distance(approach) > 0.5:
                    error("mission.approach", "Approach is more than 0.5 m from the route")
                if line.length - line.project(approach) < 3:
                    warn(
                        "mission.approach",
                        "Route ends close to the approach; check that the controller can reach the handover pose",
                    )
        except (KeyError, TypeError, ValueError) as exc:
            error("c40_global_trajectory", f"Invalid route: {exc}")
    if parsed is not None and parsed.decision_status_file:
        destination = str(Path(parsed.decision_status_file).expanduser().resolve())
        if destination in report["files"].values():
            error(
                "decision_status_file",
                "Status output would overwrite a configuration or route file",
            )
    return finish()
