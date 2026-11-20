"""Parking refusals and lifecycle checks; no CARLA server required."""

import importlib.util
import math
import sys
from pathlib import Path

import pytest
from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
from avlite.c20_planning.c21_planning_model import GlobalPlan
from avlite.c50_common.c54_trajectory_tracker import TrajectoryTracker
from shapely.geometry import box


@pytest.fixture(scope="module")
def parking():
    spec = importlib.util.spec_from_file_location(
        "parking_test_plugin",
        Path(__file__).parents[1] / "__init__.py",
        submodule_search_locations=[str(Path(__file__).parents[1])],
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.parking


@pytest.fixture
def planner(parking):
    route = [(-12.0, 0.0), (0.0, 11.0)]
    global_plan = GlobalPlan(path=route, trajectory=TrajectoryTracker(path=route, velocity=[0, 0]))
    pm = PerceptionModel(ego_vehicle=EgoState(x=-12, y=0, theta=0))
    pm.simulation_time = 0.0
    planner = parking.ParkingPlanner(global_plan, pm)
    planner.request_parking(
        parking.ParkingRequest((0.0, 11.0, math.pi / 2), box(-1.6, 8, 1.6, 14), box(-16, -5, 8, 15))
    )
    return planner


def test_unknown_is_not_clear(planner):
    plan = planner.replan()
    assert planner.reason == "unknown_or_stale_obstacles"
    assert max(plan.velocity) == 0


@pytest.mark.parametrize("stamp", [-1.0, 1.0])
def test_stale_and_future_snapshots_stop(planner, stamp):
    planner.update_obstacles([], stamp)
    assert max(planner.replan().velocity) == 0
    assert planner.state == "PARKING_BLOCKED"


def test_occupied_bay_refused(planner):
    planner.update_obstacles([box(-1, 9, 1, 13)], 0.0)
    assert max(planner.replan().velocity) == 0
    assert planner.reason == "no_feasible_forward_path"


def test_bay_too_small_refused(planner, parking):
    planner.request_parking(
        parking.ParkingRequest((0, 11, math.pi / 2), box(-0.5, 10, 0.5, 12), box(-16, -5, 8, 15))
    )
    planner.update_obstacles([], 0.0)
    assert max(planner.replan().velocity) == 0
    assert planner.poses is None


def test_moving_request_rejected(planner):
    planner.pm.ego_vehicle.velocity = 1.0
    with pytest.raises(ValueError, match="stopped"):
        planner.request_parking(planner.request)


def test_reset_clears_request_and_observations(planner):
    planner.update_obstacles([], 0.0)
    planner.replan()
    assert planner.poses is not None
    planner.reset()
    assert planner.request is None and planner.obstacles is None
    assert max(planner.replan().velocity) == 0


def test_controller_braking_cannot_reverse(planner, parking):
    ego = planner.pm.ego_vehicle
    ego.velocity = 0.001
    cmd = parking.ParkingController().control(ego, planner.replan(), control_dt=0.05)
    assert ego.velocity + cmd.acceleration * 0.05 >= 0


def test_nonfinite_goal_rejected(parking):
    with pytest.raises(ValueError):
        parking.ParkingRequest((float("nan"), 0, 0), box(0, 0, 4, 5), box(-5, -5, 10, 10))


def test_factory_constructor_accepts_env(planner, parking):
    result = parking.ParkingPlanner(global_plan=planner.global_plan, env=planner.pm)
    assert result.pm is planner.pm


def test_cancel_is_latched(planner):
    planner.update_obstacles([], 0.0)
    planner.replan()
    planner.cancel()
    assert max(planner.replan().velocity) == 0
    assert planner.state == "PARKING_CANCELLED"


def test_route_change_invalidates_request(planner):
    planner.set_global_plan(planner.global_plan)
    assert planner.request is None
    assert planner.poses is None


def test_parked_is_latched(planner):
    ego = planner.pm.ego_vehicle
    ego.x, ego.y, ego.theta = planner.request.goal
    planner.state = "PARKED"
    planner._terminal = True
    planner.replan()
    assert planner.state == "PARKED"
    assert max(planner.get_local_plan().velocity) == 0


def test_type_is_validated(parking):
    with pytest.raises(ValueError):
        parking.ParkingRequest((0, 0, 0), box(-3, -2, 3, 2), box(-10, -10, 10, 10), "unknown")


def test_zero_control_timestep_rejected(planner, parking):
    with pytest.raises(ValueError):
        parking.ParkingController().control(planner.pm.ego_vehicle, planner.replan(), control_dt=0)


def test_basic_sim_reverse_and_direction_interlock(parking):
    plugin = sys.modules[parking.__package__]
    DirectionalAckermannCommand = parking.DirectionalAckermannCommand
    BasicSim = plugin.PlanningBasicSim
    from avlite.c40_execution.c49_settings import ExecutionSettingsSchema

    ego = EgoState(velocity=1)
    world = BasicSim(
        ego,
        PerceptionModel(ego_vehicle=ego),
        setting=ExecutionSettingsSchema(c46_npc_control=False),
    )
    world.control_ego_state(DirectionalAckermannCommand(direction=-1, acceleration=1), 0.05)
    assert ego.x > 0 and world._drive_direction == 1
    for _ in range(50):
        world.control_ego_state(DirectionalAckermannCommand(direction=-1, acceleration=1), 0.05)
    assert world._drive_direction == -1 and ego.x < 0
    for _ in range(100):
        world.control_ego_state(DirectionalAckermannCommand(direction=-1, acceleration=-2), 0.05)
    assert ego.velocity == 0


def test_sensor_mode_cannot_reuse_cached_frame(planner):
    from types import SimpleNamespace

    import numpy as np
    from avlite.c50_common.c52_world_sensor_datatypes import Lidar, SensorFrame

    # A producer exposing an empty blocked geometry is observed clear here;
    # this test checks transport freshness independently of cloud processing.
    planner.sensor_occupancy = SimpleNamespace(
        observe=lambda *args: SimpleNamespace(
            blocked=box(0, 0, 1, 1).difference(box(0, 0, 1, 1)), stamp=1000.0, status="observed"
        )
    )
    frame = SensorFrame(
        lidars={"scan": Lidar(points=np.zeros((1, 4)), stamp=1000)},
        primary_lidar_name="scan",
        stamp=1000,
    )
    planner.replan(sensors=frame)
    assert planner._observed_time == 0
    planner.pm.simulation_time = 1
    assert max(planner.replan(sensors=frame).velocity) == 0
    assert planner.reason == "unknown_or_stale_obstacles"


def test_regenerating_after_observation_resets_segment_progress(planner):
    planner.update_obstacles([], 0)
    planner._progress = 100000
    planner._segment = 17
    planner.poses = None
    plan = planner.replan()
    assert planner._segment == 0
    assert planner._progress < len(plan.path)
    assert planner.state == "PARKING_APPROACH"


def test_controller_does_not_accelerate_from_stored_integral_when_overspeed(planner, parking):
    planner.update_obstacles([], 0.0)
    plan = planner.replan()
    plan.velocity = [0.1] * len(plan.velocity)
    ego = planner.pm.ego_vehicle
    ego.velocity = 0.4
    controller = parking.ParkingController()
    controller.rear_offset = 1.2
    controller._integral = 1.5
    command = controller.control(ego, plan, control_dt=0.1)
    assert command.acceleration < 0
    assert controller._integral <= 0


@pytest.mark.parametrize("speed,state", [(0.0, "PARKED"), (0.1, "PARKING_STOP")])
def test_final_goal_contract_does_not_require_ten_cm(planner, speed, state):
    import numpy as np

    ego = planner.pm.ego_vehicle
    ego.x, ego.y, ego.theta = 0.109, 11, math.pi / 2 + math.radians(4.39)
    ego.velocity = speed
    planner.poses = np.array(
        [[0, 10.9, math.pi / 2], [0, 10.95, math.pi / 2], [0, 11, math.pi / 2]]
    )
    planner.update_obstacles([], 0.0)
    plan = planner.replan()
    assert planner.state == state
    assert max(plan.velocity) == 0


@pytest.mark.parametrize("offset,angle", [(0.21, 0), (0.109, 6)])
def test_final_goal_contract_still_rejects_pose_error(planner, offset, angle):
    import numpy as np

    ego = planner.pm.ego_vehicle
    ego.x, ego.y, ego.theta = offset, 11, math.pi / 2 + math.radians(angle)
    planner.poses = np.array(
        [[0, 10.9, math.pi / 2], [0, 10.95, math.pi / 2], [0, 11, math.pi / 2]]
    )
    planner.update_obstacles([], 0.0)
    planner.replan()
    assert planner.state == "PARKING_BLOCKED"


def test_search_retries_bounded_despite_changing_noise(planner, monkeypatch):
    calls = []
    monkeypatch.setattr(planner, "_generate", lambda: calls.append(planner.pm.simulation_time))
    planner.search_retry_interval = 2.0
    for tick in range(31):
        now = tick / 10
        planner.pm.simulation_time = now
        planner.update_obstacles([box(6 + now / 100, 12, 7, 13)], now)
        plan = planner.replan()
        assert max(plan.velocity) == 0
    assert calls == [0, 2]
    planner.request_parking(planner.request)
    planner.replan()
    assert calls == [0, 2, 3]


@pytest.mark.parametrize("speed", [0.0, 0.1])
def test_missed_endpoint_replans_only_after_stopping(planner, monkeypatch, speed):
    import numpy as np

    ego = planner.pm.ego_vehicle
    ego.x, ego.y, ego.theta = 0.25, 11, math.pi / 2
    ego.velocity = speed
    planner.search_retry_interval = 2.0
    planner.poses = np.array([[0, 10.9, math.pi / 2], [0, 11, math.pi / 2]])
    planner.update_obstacles([], 0.0)
    assert max(planner.replan().velocity) == 0
    assert planner.state == "PARKING_BLOCKED"
    if speed:
        assert planner.poses is not None
        return
    assert planner.poses is None
    calls = []
    monkeypatch.setattr(planner, "_generate", lambda: calls.append((ego.x, ego.y)))
    for now in (0.1, 1.9, 2.0):
        planner.pm.simulation_time = now
        planner.update_obstacles([], now)
        assert max(planner.replan().velocity) == 0
    assert calls == [(0.25, 11)]


@pytest.mark.parametrize("direction", [1, -1])
def test_controller_executes_two_point_certified_prefix(planner, parking, direction):
    import numpy as np
    from avlite.c20_planning.c21_planning_model import LocalPlan

    ego = planner.pm.ego_vehicle
    poses = np.array([[ego.x, ego.y, ego.theta], [ego.x + direction * 0.04, ego.y, ego.theta]])
    plan = LocalPlan(path=[tuple(p[:2]) for p in poses], velocity=[0.05, 0.05])
    plan.parking_poses = poses
    plan.parking_direction = direction
    controller = parking.ParkingController()
    controller.rear_offset = 1.25
    command = controller.control(ego, plan, control_dt=0.1)
    assert command.acceleration > 0
    assert not command.hold_brake
    assert command.direction == direction
    assert abs(command.steer) < 1e-6
    plan.parking_poses[1] = plan.parking_poses[0]
    command = controller.control(ego, plan, control_dt=0.1)
    assert command.hold_brake


@pytest.mark.parametrize("direction", [1, -1])
def test_controller_distinguishes_rollback_from_progress(planner, parking, direction):
    import numpy as np
    from avlite.c20_planning.c21_planning_model import LocalPlan

    ego = planner.pm.ego_vehicle
    poses = np.array([[ego.x, ego.y, 0], [ego.x + direction, ego.y, 0]])
    plan = LocalPlan(path=[tuple(p[:2]) for p in poses], velocity=[0.05, 0.05])
    plan.parking_poses = poses
    plan.parking_direction = direction
    ego.velocity = 0.04
    commands = []
    for longitudinal in [direction * 0.04, -direction * 0.04]:
        ego.longitudinal_velocity = longitudinal
        controller = parking.ParkingController()
        commands.append(controller.control(ego, plan, control_dt=0.1))
    assert commands[1].acceleration > commands[0].acceleration + 0.2
    ego.longitudinal_velocity = float("nan")
    assert controller.control(ego, plan, control_dt=0.1).hold_brake
