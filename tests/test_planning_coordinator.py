"""Real task runner / BasicSim handovers and simulator-independent sensor contracts."""

import importlib.util
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def demo():
    path = ROOT / "tools/coordinator_demo.py"
    spec = importlib.util.spec_from_file_location("standalone_coordinator_checks", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def stack(demo):
    from avlite.c40_execution.c49_settings import ExecutionSettings

    previous = ExecutionSettings.model_copy(deep=True)
    ex, task = demo.create_stack()
    yield ex, task
    ex.stop()
    for field in type(previous).model_fields:
        setattr(ExecutionSettings, field, getattr(previous, field))


def test_start_switch_and_resume_preserve_state(stack, demo):
    ex, task = stack
    ex.task_runner.notify(demo.ExecutionStarted())
    assert task.active_strategy == "road-tree"
    assert task.last_result["mode"] == "road"
    assert type(ex.local_planner._behavioral).__name__ == "HDMapBehaviorTreePlanner"
    old = ex.local_planner
    old._behavioral.test_marker = 42
    ex.task_runner.notify(demo.ExecutionStarted())
    assert ex.local_planner is old and old._behavioral.test_marker == 42
    task.request(ex, "road-rules")
    assert task.active_strategy == "road-rules"
    assert type(ex.local_planner._behavioral).__name__ == "HDMapBehaviorPlanner"
    assert len(task.history) == 2
    assert ex._last_cmd is None


@pytest.mark.parametrize("speed", [0.4, float("nan"), float("inf")])
def test_perception_handover_checks_actual_bridge_speed(stack, demo, monkeypatch, speed):
    from avlite.c10_perception.c12_perception_strategy import PerceptionPipeline

    ex, _ = stack
    ex.stop()
    ex.pm.ego_vehicle.velocity = 0
    current = ex.perception
    command = ex._last_cmd = object()
    observation = ex.pm.regulatory_observation = object()
    monkeypatch.setattr(ex.world, "get_ego_state", lambda: demo.EgoState(velocity=speed))
    candidate = PerceptionPipeline(ex.pm)
    event = demo.plugin.SelectPerceptionStack(candidate, ex.pm.map)
    with pytest.raises(ValueError, match="Stop the vehicle"):
        demo.plugin.PerceptionSwitchTask().execute(ex, event)
    assert ex.perception is current
    assert ex._last_cmd is command
    assert ex.pm.regulatory_observation is observation


def test_resume_does_not_claim_another_tasks_global_planner(stack, demo):
    ex, task = stack
    ex.task_runner.notify(demo.ExecutionStarted())
    replacement = object()
    ex.global_planner = replacement
    local = ex.local_planner
    ex.task_runner.notify(demo.ExecutionStarted())
    assert ex.global_planner is replacement and ex.local_planner is local
    assert task.active_strategy is None and task.last_result is None


@pytest.mark.parametrize(
    "problem",
    [
        "moving",
        "world_moving",
        "stale_map",
        "stale_planner",
        "unknown",
        "wrong_map",
        "bad_route",
        "bad_pose",
        "missing_stage",
        "constructor",
        "capability",
    ],
)
def test_refused_request_preserves_stack(stack, demo, problem, monkeypatch):
    ex, task = stack
    module = sys.modules[demo.NAME + ".execution.coordinator"]
    event = module.SelectPlanningStrategy(
        "road-tree", ex.local_planner.global_plan, ex.pm.map, ex.local_planner
    )
    old = (ex.global_planner, ex.local_planner, ex.controller, ex.perception, ex.world)
    ex._last_cmd = command = object()
    if problem == "moving":
        ex.pm.ego_vehicle.velocity = 2.0
    elif problem == "world_moving":
        fake = replace(ex.world.get_ego_state(), velocity=1.0)
        monkeypatch.setattr(ex.world, "get_ego_state", lambda: fake)
    elif problem == "stale_map":
        event = replace(event, expected_map=object())
    elif problem == "stale_planner":
        event = replace(event, expected_planner=object())
    elif problem == "unknown":
        event = replace(event, strategy="missing")
    elif problem == "wrong_map":
        event = replace(event, strategy="race")
    elif problem == "bad_route":
        import copy

        route = copy.deepcopy(event.global_plan)
        route.path = [(float("nan"), 0)]
        event = replace(event, global_plan=route)
    elif problem == "bad_pose":
        ex.pm.ego_vehicle.theta = float("nan")
    elif problem == "missing_stage":
        task.configurations["road-tree"] = replace(task.configurations["road-tree"], path="missing")
    elif problem == "constructor":

        def fail(*args, **kwargs):
            raise RuntimeError("constructor failure")

        from avlite.c20_planning.c22_global_planning_strategy import (
            GlobalPlannerStrategy,
        )

        monkeypatch.setitem(GlobalPlannerStrategy.registry, "HDMapLaneFollowingPlanner", fail)
    else:

        def fail():
            raise ValueError("missing capability")

        monkeypatch.setattr(ex, "validate_stack", fail)
    ex.task_runner.notify(event)
    assert task.last_result["status"] == "rejected"
    assert all(
        a is b
        for a, b in zip(
            old,
            (
                ex.global_planner,
                ex.local_planner,
                ex.controller,
                ex.perception,
                ex.world,
            ),
        )
    )
    assert ex._last_cmd is command


def test_two_queued_requests_do_not_overwrite_each_other(stack, demo):
    from avlite.c40_execution.c43_task_strategy import TaskStrategy

    ex, task = stack

    class Monitor(TaskStrategy):
        def execute(self, executer, event=None):
            task.request(executer, "road-tree")
            task.request(executer, "road-rules")

    ex.task_runner.tasks.insert(0, Monitor())
    ex.task_runner.step(ex)
    assert [x["status"] for x in task.history] == ["selected", "rejected"]
    assert task.active_strategy == "road-tree"
    assert "planning stack changed" in task.last_result["reason"]


def test_disabled_tasks_do_not_publish(stack):
    ex, task = stack
    ex.call_tasks = False
    with pytest.raises(ValueError, match="Enable execution tasks"):
        task.request(ex)
    assert task.last_result is None


def test_invalid_start_fails_before_drive(stack, demo):
    ex, task = stack
    task.initial_strategy = "missing"
    with pytest.raises(ValueError, match="Initial planning selection failed"):
        ex.task_runner.notify(demo.ExecutionStarted())
    assert ex.elapsed_sim_time == 0


def test_success_and_failure_events_can_be_observed(stack, demo):
    from avlite.c40_execution.c43_task_strategy import TaskSchedule, TaskStrategy

    module = sys.modules[demo.NAME + ".execution.coordinator"]
    ex, task = stack
    events = []

    class Observer(TaskStrategy):
        schedule = TaskSchedule.ON_EVENT
        listen_events = frozenset(
            {module.PlanningStrategySelected, module.PlanningStrategyRejected}
        )

        def execute(self, executer, event=None):
            events.append(event)

    ex.task_runner.tasks.append(Observer())
    task.request(ex, "road-tree")
    task.request(ex, "missing")
    assert isinstance(events[0], module.PlanningStrategySelected)
    assert isinstance(events[1], module.PlanningStrategyRejected)


def test_handover_forces_next_plan_and_control_even_with_long_period(stack, demo):
    ex, task = stack
    ex.step(sim_dt=0.05, replan_dt=100, control_dt=100)
    demo.brake(ex)
    task.request(ex, "road-tree")
    assert task.last_result["status"] == "selected"
    assert ex._last_cmd is None
    ex.step(sim_dt=0.05, replan_dt=100, control_dt=100)
    assert ex._last_cmd is not None


def test_declared_world_requirement_is_enforced(stack, demo):
    from avlite.c20_planning.c23_local_planning_strategy import LocalPlanningPipeline
    from avlite.c50_common.c51_capabilities import WorldCapability

    ex, task = stack

    class NeedsCamera(LocalPlanningPipeline):
        world_requirements = frozenset({WorldCapability.CAMERA_RGB})

    task.configurations["camera-road"] = replace(
        task.configurations["road-tree"], local_planner="NeedsCamera"
    )
    task.request(ex, "camera-road")
    assert task.last_result["status"] == "rejected"
    assert "required sensors" in task.last_result["reason"]


@pytest.mark.parametrize("mode", ["road", "race"])
def test_basicsim_drives_after_coordinated_handover(demo, mode):
    result = demo.run(mode)
    assert result["passed"], result
    if mode == "race":
        assert result["max_scripted_rivals"] == 2


def sensor_frame(stamp=10.0):
    from avlite.c50_common.c52_world_sensor_datatypes import Camera, Lidar, SensorFrame

    rng = np.random.default_rng(9)
    rgb = rng.integers(25, 230, (72, 128, 3), dtype=np.uint8)
    points = np.c_[
        rng.uniform(2, 25, 200),
        rng.uniform(-10, 10, 200),
        np.full(200, -0.3),
        np.ones(200),
    ].astype(np.float32)
    points[:30, :3] = [10.0, 0.0, 1.0]
    camera = Camera(
        width=128,
        height=72,
        rgb=rgb,
        stamp=stamp,
        intrinsic=np.array([[64, 0, 64], [0, 64, 36], [0, 0, 1]]),
        base_to_sensor=np.array([[0, 0, 1, 1.5], [-1, 0, 0, 0], [0, -1, 0, 1.4], [0, 0, 0, 1.0]]),
    )
    return SensorFrame(
        cameras={"front": camera},
        lidars={"roof": Lidar(points=points, stamp=stamp)},
        primary_camera_name="front",
        primary_lidar_name="roof",
        stamp=stamp,
    )


@pytest.mark.parametrize(
    "fault,reason",
    [
        ("cloud_shape", "invalid_lidar"),
        ("cloud_dtype", "invalid_lidar"),
        ("rgb", "invalid_image"),
        ("intrinsic", "invalid_calibration"),
        ("reflection", "invalid_calibration"),
        ("mount", "invalid_calibration"),
    ],
)
def test_malformed_noncarla_frames_fail_closed(demo, fault, reason):
    module = sys.modules[demo.NAME + ".perception.detector"]
    detector = module.SensorRegulatoryDetector(
        detector=lambda _: pytest.fail("must reject before inference"), geometry={}
    )
    pm = demo.PerceptionModel(ego_vehicle=demo.EgoState())
    pm.simulation_time = 10.0
    frame = sensor_frame()
    if fault == "cloud_shape":
        frame.lidar.points = frame.lidar.points[:, :2]
    elif fault == "cloud_dtype":
        frame.lidar.points = frame.lidar.points.astype(np.float64)
    elif fault == "rgb":
        frame.camera.rgb = frame.camera.rgb[:, :, 0]
    elif fault == "intrinsic":
        frame.camera.intrinsic[0, 0] = 0
    elif fault == "reflection":
        frame.lidar.base_to_sensor[0, 0] = -1.0
    else:
        frame.camera.base_to_sensor[0, 0] = np.nan
    detector.detect(pm, frame)
    assert pm.regulatory_observation.diagnostics["status"] == reason
    assert not pm.regulatory_observation.lights
    assert not pm.regulatory_observation.crossings


def test_sensor_bridge_replacement_keeps_same_perception_and_planner(stack, demo):
    """Integration contract, not an accuracy claim: deterministic detections + real fusion."""
    from avlite.c40_execution.c41_world_bridge import WorldBridge
    from avlite.c40_execution.c49_settings import ExecutionSettings
    from avlite.c50_common.c51_capabilities import StackCapability, WorldCapability

    ex, task = stack
    module = sys.modules[demo.NAME + ".perception.detector"]

    class FrameBridge(WorldBridge):
        world_capabilities = frozenset({WorldCapability.CAMERA_RGB, WorldCapability.LIDAR_3D})
        stack_capabilities = frozenset({StackCapability.LOCALIZATION})

        def control_ego_state(self, cmd, dt=0.05):
            pass

        def get_sensor_frame(self):
            return self.frame

        def get_ground_truth_perception_model(self):
            raise AssertionError("Ground truth must not be queried")

    class SecondFrameBridge(FrameBridge):
        pass

    ExecutionSettings.c41_world_stack_capabilities = ["LOCALIZATION"]
    detector = module.SensorRegulatoryDetector(
        detector=lambda _: [module.Detection("person", 0.95, (45, 20, 78, 66))],
        geometry={},
        signal_backend="trained_rgb",
    )
    ex.perception._detector = detector
    ex.world = FrameBridge(ex.pm.ego_vehicle, map=ex.pm.map)
    ex.world.frame = sensor_frame(10.0)
    task.request(ex, "road-tree")
    assert task.last_result["status"] == "selected"
    planner = ex.local_planner
    ex.step(sim_dt=0.05, pace_perception=False, pace_replan=False, pace_control=False)
    assert ex.pm.regulatory_observation.diagnostics["status"] == "ok"
    assert len(ex.pm.regulatory_observation.pedestrians) == 1
    assert ex.perception._detector is detector
    ex.world = SecondFrameBridge(ex.pm.ego_vehicle, map=ex.pm.map)
    ex.world.frame = sensor_frame(10.05)
    ex.step(sim_dt=0.05, pace_perception=False, pace_replan=False, pace_control=False)
    assert ex.pm.regulatory_observation.diagnostics["status"] == "ok"
    assert ex.local_planner is planner
    ex.world.frame = sensor_frame(10.1)
    ex.world.frame.camera.rgb[:] = 0
    ex.step(sim_dt=0.05, pace_perception=False, pace_replan=False, pace_control=False)
    assert ex.local_planner.get_local_plan().behavior_state == "PERCEPTION_STOP"


def test_resume_does_not_steal_another_tasks_planner(stack, demo):
    ex, task = stack
    ex.task_runner.notify(demo.ExecutionStarted())
    replacement = demo.ReferencePathPlanner(ex.local_planner.global_plan, ex.pm)
    ex.local_planner = replacement
    ex.task_runner.notify(demo.ExecutionStarted())
    assert ex.local_planner is replacement
    assert task.active_strategy is None


def test_task_reset_clears_old_selection(stack, demo):
    ex, task = stack
    ex.task_runner.notify(demo.ExecutionStarted())
    ex.task_runner.reset()
    assert task.active_strategy is None and not task.history
    ex.task_runner.notify(demo.ExecutionStarted())
    assert task.active_strategy == "road-tree"
