"""Mission selection from calibrated synthetic returns, not simulator obstacles."""

# ruff: noqa: F811 -- imported pytest fixtures are injected by argument name
import copy
import importlib
import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from avlite.c10_perception.c11_perception_model import EgoState
from avlite.c50_common.c52_world_sensor_datatypes import Lidar, SensorFrame
from shapely.geometry import Point, Polygon, box

from .test_missions import profiles, runner, stack, tick, until  # noqa: F401


def frame(points, stamp, pose=None):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if pose is not None:
        transform = np.linalg.inv(pose.pose_matrix())
        points = points @ transform[:3, :3].T + transform[:3, 3]
    cloud = Lidar(points=points, stamp=stamp, sensor_name="front")
    return SensorFrame(lidars={"front": cloud}, primary_lidar_name="front", stamp=stamp)


def grid(area):
    a, b, c, d = area.bounds
    x, y = np.meshgrid(np.arange(a + 0.125, c, 0.25), np.arange(b + 0.125, d, 0.25))
    return np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])


@pytest.fixture
def view(runner):
    api = importlib.import_module(runner.plugin.__name__ + ".parking.mission_observation")
    provider = importlib.import_module(runner.plugin.__name__ + ".parking.occupancy")
    return api.MissionLidarObservation(provider.LidarParkingOccupancy(), box(-10, -10, 10, 10), 0)


def test_distinguishes_free_occupied_and_unknown(view):
    points = np.vstack([grid(box(0, 0, 4, 6)), [[6.1, 1, 1]]])
    view.observe(frame(points, 100), EgoState(), 0)
    assert view.classify(box(0.5, 0.5, 3.5, 5.5), 0) == "free"
    assert view.classify(box(5.5, 0.5, 7, 3), 0) == "occupied"
    assert view.classify(box(-5, -5, -2, -2), 0) == "unknown"


def test_hazard_without_ground_is_occupied(view):
    view.observe(frame([[1, 1, 1]], 100), EgoState(), 0)
    assert view.classify(box(0, 0, 2, 2), 0) == "occupied"
    assert view.classify(box(-5, -5, -2, -2), 0) == "unknown"


def test_repeated_frame_does_not_refresh_or_reproject(view):
    source = frame(grid(box(0, 0, 4, 6)), 100)
    view.observe(source, EgoState(), 0)
    original = view.occupancy
    view.observe(source, EgoState(x=20), 0.2)
    assert view.occupancy is original and view.stamp == 0
    view.observe(source, EgoState(), 0.31)
    assert view.classify(box(1, 1, 2, 2), 0.31) == "stale_lidar"


@pytest.mark.parametrize("stamp", [99.9, 105, float("nan"), None])
def test_invalid_clock_never_clears_space(view, stamp):
    view.observe(frame(grid(box(0, 0, 4, 6)), 100), EgoState(), 0)
    view.observe(frame(grid(box(0, 0, 4, 6)), stamp), EgoState(), 0.1)
    assert view.classify(box(1, 1, 2, 2), 0.1) != "free"


def test_new_assembly_time_does_not_refresh_old_cloud(view):
    view.observe(frame(grid(box(0, 0, 4, 6)), 100), EgoState(), 0)
    source = frame(grid(box(0, 0, 4, 6)), 100)
    source.stamp = 100.1
    view.observe(source, EgoState(), 0.1)
    assert view.reason(0.1) == "unsynchronized_lidar"


def test_delayed_progressing_frames_keep_original_clock_offset(view):
    points = grid(box(0, 0, 4, 6))
    view.observe(frame(points, 100), EgoState(), 0)
    view.observe(frame(points, 100.01), EgoState(), 0.4)
    assert view.reason(0.4) == "stale_or_future_lidar"
    view.observe(frame(points, 100.5), EgoState(), 0.5)
    assert view.classify(box(1, 1, 2, 2), 0.5) == "free"


def test_dropped_and_empty_frames_do_not_keep_clearance(view):
    view.observe(frame(grid(box(0, 0, 4, 6)), 100), EgoState(), 0)
    view.observe(None, EgoState(), 0.1)
    assert view.reason(0.1) != "observed"
    view.observe(frame([], 100.2), EgoState(), 0.2)
    assert view.reason(0.2) == "empty_or_invalid_cloud"


def configure(ex, mission, runner, monkeypatch, points, *, at_approach=True):
    settings = runner.plugin.PluginSettings
    settings.parking_simulator_obstacles = False
    settings.parking_lidar_occupancy = True
    settings.parking_multi_lidar = False
    settings.parking_ground_z = 0
    ex.task_runner.tasks[:] = [mission, ex.task_runner.tasks[-1]]
    # Start at the surveyed handover point to isolate selection and parking.
    data = json.loads(Path(settings.mission_file).read_text())
    ego = ex.ego_state
    if at_approach:
        ego.x, ego.y, ego.theta = data["approach"]
        ego.velocity = 0
    calls = []

    def snapshot():
        calls.append(ex.elapsed_sim_time)
        return frame(points(), 100 + ex.elapsed_sim_time, ex.ego_state)

    monkeypatch.setattr(ex.world, "get_sensor_frame", snapshot)
    return calls


def test_unknown_then_new_scan_selects_without_ground_truth(stack, runner, monkeypatch):
    ex, mission, status = stack
    scene = mission.parking.parking_spaces()
    points = [np.array([[30, 20, 0]])]
    calls = configure(ex, mission, runner, monkeypatch, lambda: points[0])
    # Even a supposedly clear complete observation cannot override LiDAR mode.
    ex.pm.parking_observation = runner.plugin.ParkingObservation((), 0)
    until(ex, mission, "SELECT_SPACE")
    assert mission.candidate_results["demo_bay"] != "free"
    assert ex.motion_holds and ex.ego_state.velocity == 0
    points[0] = grid(Polygon(scene["drivable"]))
    tick(ex)
    assert mission.state == "PARKING", mission.reason
    assert mission.space == "demo_bay"
    assert status.snapshot["observation_source"] == "LiDAR ground coverage"
    assert len(calls) == len(set(calls)), "The mission fetched an extra sensor snapshot"
    assert ex.local_planner.sensor_occupancy is None  # same observer survives the handover


def test_clear_bay_with_unknown_approach_path_does_not_handover(stack, runner, monkeypatch):
    ex, mission, _ = stack
    scene = mission.parking.parking_spaces()
    bay = Polygon(scene["spaces"]["demo_bay"]["bay"])
    configure(ex, mission, runner, monkeypatch, lambda: grid(bay.buffer(0.5)))
    original = ex.local_planner
    until(ex, mission, "SELECT_SPACE")
    assert ex.local_planner is original and ex.motion_holds
    assert "no_feasible" in mission.candidate_results["demo_bay"]


def test_occupied_first_choice_selects_observed_second(stack, runner, monkeypatch):
    ex, mission, _ = stack
    path = Path(runner.plugin.PluginSettings.parking_scene_file)
    scene = json.loads(path.read_text())
    scene["spaces"]["occupied"] = copy.deepcopy(scene["spaces"]["demo_bay"])
    scene["spaces"]["occupied"]["bay"] = [[-5, 8], [-2, 8], [-2, 14], [-5, 14]]
    path.write_text(json.dumps(scene))
    path = Path(runner.plugin.PluginSettings.mission_file)
    data = json.loads(path.read_text())
    data["spaces"] = ["occupied", "demo_bay"]
    path.write_text(json.dumps(data))
    points = np.vstack([grid(Polygon(scene["drivable"])), [[-3.5, 11, 1]]])
    configure(ex, mission, runner, monkeypatch, lambda: points)
    until(ex, mission, "PARKING")
    assert mission.space == "demo_bay"
    assert mission.candidate_results["occupied"] == "occupied"


def test_frozen_scan_after_handover_brakes_and_latches(stack, runner, monkeypatch):
    ex, mission, _ = stack
    points = grid(Polygon(mission.parking.parking_spaces()["drivable"]))
    configure(ex, mission, runner, monkeypatch, lambda: points)
    until(ex, mission, "PARKING")
    frozen = ex.world.get_sensor_frame()
    monkeypatch.setattr(ex.world, "get_sensor_frame", lambda: frozen)
    mission.spec = replace(mission.spec, progress_timeout=1)
    until(ex, mission, "BLOCKED", limit=100)
    assert ex.ego_state.velocity <= 0.02
    assert mission.reason == "no_progress_timeout"
    assert mission.parking.sensor_view.reason(ex.elapsed_sim_time) == "stale_lidar"


def test_no_replan_cannot_run_indefinitely_on_previous_coverage(stack, runner, monkeypatch):
    ex, mission, _ = stack
    points = grid(Polygon(mission.parking.parking_spaces()["drivable"]))
    configure(ex, mission, runner, monkeypatch, lambda: points)
    until(ex, mission, "PARKING")
    for _ in range(30):
        tick(ex, call_replan=False)
    assert ex.ego_state.velocity == 0
    assert ex.motion_holds[mission.parking] == "stale_lidar"


def test_localized_pose_and_mount_transform_are_used(view):
    pose = EgoState(x=4, y=-3, theta=0.7)
    source = frame([[1.1, 1.1, 0]], 100, pose)
    source.lidar.base_to_sensor[2, 3] = 1
    source.lidar.points[:, 2] -= 1
    view.observe(source, pose, 0)
    assert view.occupancy.free.covers(Point(1.1, 1.1))


@pytest.mark.parametrize("mode", [0, 1, 2])
def test_sensor_mission_completes_each_parking_type(runner, profiles, monkeypatch, mode):
    ex, mission, _ = runner.create_stack(profiles[mode])
    runner.plugin.PluginSettings.decision_status_file = ""
    area = Polygon(mission.parking.parking_spaces()["drivable"])
    points = grid(area)
    configure(ex, mission, runner, monkeypatch, lambda: points, at_approach=False)
    start = np.array([ex.ego_state.x, ex.ego_state.y])
    approach_distance = 0.0
    try:
        for _ in range(2400):
            tick(ex)
            if mission.state in {"APPROACH", "SETTLING"}:
                approach_distance = max(
                    approach_distance,
                    float(np.linalg.norm(np.array([ex.ego_state.x, ex.ego_state.y]) - start)),
                )
            assert area.covers(Polygon(ex.ego_state.get_bb_corners()))
            assert mission.state != "BLOCKED", mission.reason
            if mission.state == "COMPLETE":
                break
        assert mission.state == "COMPLETE", mission.reason
        assert approach_distance > 7
        assert ex.ego_state.velocity <= 0.02
        assert ex.motion_holds and mission.space == "demo_bay"
    finally:
        ex.stop()


def test_multi_lidar_requires_independent_synchronized_support(view, runner):
    api = importlib.import_module(runner.plugin.__name__ + ".parking.occupancy")
    view.provider = api.FusedLidarParkingOccupancy(minimum_support=2)
    points = grid(box(3, 0, 7, 6))
    source = frame(points, 100)
    view.observe(source, EgoState(), 0)
    assert view.classify(box(4, 1, 6, 5), 0) != "free"
    second = Lidar(points=points.copy(), stamp=100.1, sensor_name="rear")
    source = frame(points, 100.1)
    source.lidars["rear"] = second
    view.observe(source, EgoState(), 0.1)
    assert view.classify(box(4, 1, 6, 5), 0.1) == "free"
    source.stamp = source.lidar.stamp = 100.2
    view.observe(source, EgoState(), 0.2)
    assert view.classify(box(4, 1, 6, 5), 0.2) != "free"


def test_reset_discards_sensor_clock_and_coverage(stack, runner, monkeypatch):
    ex, mission, _ = stack
    points = grid(Polygon(mission.parking.parking_spaces()["drivable"]))
    configure(ex, mission, runner, monkeypatch, lambda: points)
    until(ex, mission, "PARKING")
    ex.reset()
    assert mission.parking.sensor_view is None
    monkeypatch.setattr(ex.world, "get_sensor_frame", lambda: None)
    tick(ex)
    assert mission.parking.sensor_view.reason(ex.elapsed_sim_time) != "observed"
    assert mission.parking.available_spaces(ex, "perpendicular_forward") == []


def test_new_hazard_after_handover_blocks_existing_path(stack, runner, monkeypatch):
    ex, mission, _ = stack
    ground = grid(Polygon(mission.parking.parking_spaces()["drivable"]))
    points = [ground]
    configure(ex, mission, runner, monkeypatch, lambda: points[0])
    until(ex, mission, "PARKING")
    planner = ex.local_planner
    middle = planner.poses[len(planner.poses) // 2]
    points[0] = np.vstack([ground, [[middle[0], middle[1], 1]]])
    tick(ex)
    assert planner.state == "PARKING_BLOCKED"
    assert planner.reason == "obstacle_or_boundary"
    assert ex.ego_state.velocity <= 0.02
    assert max(planner.get_local_plan().velocity) == 0


def test_unknown_bay_wait_timeout_stays_latched_after_clear_scan(stack, runner, monkeypatch):
    ex, mission, _ = stack
    scene = mission.parking.parking_spaces()
    points = [np.array([[30, 20, 0]])]
    configure(ex, mission, runner, monkeypatch, lambda: points[0])
    tick(ex)
    mission.spec = replace(mission.spec, wait_timeout=1.0)
    until(ex, mission, "BLOCKED")
    points[0] = grid(Polygon(scene["drivable"]))
    for _ in range(10):
        tick(ex)
    assert mission.state == "BLOCKED" and ex.ego_state.velocity == 0
    assert ex.motion_holds


def test_lidar_settings_conflict_blocks_before_motion(stack, runner):
    ex, mission, _ = stack
    runner.plugin.PluginSettings.parking_lidar_occupancy = True
    runner.plugin.PluginSettings.parking_ground_z = 0
    tick(ex)
    assert mission.state == "BLOCKED"
    assert "either LiDAR" in mission.reason
    assert ex.ego_state.velocity == 0


def test_manual_space_query_keeps_its_planner_owned_freshness_check(stack, runner, monkeypatch):
    ex, mission, _ = stack
    points = grid(Polygon(mission.parking.parking_spaces()["drivable"]))
    configure(ex, mission, runner, monkeypatch, lambda: points)
    until(ex, mission, "PARKING")
    planner = ex.local_planner
    planner.sensor_occupancy = mission.parking.sensor_view.provider
    mission.parking.sensor_view = None
    assert mission.parking.available_spaces(ex, "perpendicular_forward") == ["demo_bay"]
    planner.update_obstacles(None, ex.elapsed_sim_time)
    assert mission.parking.available_spaces(ex, "perpendicular_forward") == []


def test_clock_offset_roundoff_does_not_produce_future_observation(view):
    points = grid(box(0, 0, 4, 6))
    view.observe(frame(points, 100), EgoState(), 0)
    view.observe(frame(points, 100.15), EgoState(), 0.15)
    assert 0 <= 0.15 - view.stamp < 1e-6
    assert view.classify(box(1, 1, 2, 2), 0.15) == "free"
