"""Exercise ordinary AVLite executors, including actual threaded workers."""

import importlib.util
import time
from pathlib import Path

import numpy as np
import pytest
from avlite.c40_execution.c44_sync_executer import SyncExecuter
from avlite.c40_execution.c45_async_threaded_executer import AsyncThreadedExecuter


@pytest.fixture(scope="module")
def demo():
    path = Path(__file__).parents[1] / "tools/coordinator_demo.py"
    spec = importlib.util.spec_from_file_location("execution_modes_demo", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def advance(ex, distance=0.3):
    start = np.array([ex.ego_state.x, ex.ego_state.y])
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        ex.step(
            sim_dt=0.05, perception_dt=0.02, replan_dt=0.02, control_dt=0.02, call_perceive=True
        )
        if np.linalg.norm(np.array([ex.ego_state.x, ex.ego_state.y]) - start) > distance:
            return
        if isinstance(ex, AsyncThreadedExecuter):
            time.sleep(0.01)
    pytest.fail("Executor did not resume motion within eight seconds")


def select(ex, task, strategy):
    previous = len(task.history)
    task.request(ex, strategy)
    deadline = time.monotonic() + 3
    while len(task.history) == previous and time.monotonic() < deadline:
        time.sleep(0.005)
    assert len(task.history) == previous + 1, "No selection result was delivered"


@pytest.mark.parametrize("executor", [SyncExecuter, AsyncThreadedExecuter])
@pytest.mark.parametrize("mode", ["road", "race"])
def test_start_drive_stop_replace_resume(demo, executor, mode):
    if not hasattr(executor, "stack_update"):
        pytest.skip("Requires the proposed core execution API")
    ex, task = demo.create_stack(mode, executor=executor)
    try:
        world = ex.world
        advance(ex)
        assert task.last_result["status"] == "selected"
        if mode == "race":
            assert len(ex.pm.agent_vehicles) == 2
        original = ex.local_planner
        select(ex, task, "road-rules" if mode == "road" else "race")
        assert task.last_result["status"] == "rejected"
        assert ex.local_planner is original
        ex.stop()
        demo.brake(ex)
        select(ex, task, "road-rules" if mode == "road" else "race")
        assert task.last_result["status"] == "selected", task.last_result
        assert ex.local_planner is not original
        assert ex.world is world
        assert ex._last_cmd is None
        advance(ex)
        assert len(task.history) == 3  # Resume must not repeat startup selection.
    finally:
        ex.stop()
    if isinstance(ex, AsyncThreadedExecuter):
        assert not ex.threads_started
        assert not any(t.is_alive() for t in ex.threads)


def test_stopped_flag_is_not_a_join_barrier(demo):
    if not hasattr(AsyncThreadedExecuter, "stack_update"):
        pytest.skip("Requires the proposed core execution API")
    import threading

    ex, task = demo.create_stack(executor=AsyncThreadedExecuter)
    release = threading.Event()
    worker = threading.Thread(target=lambda: release.wait(5))
    worker.start()
    ex.threads = [worker]
    ex.stopped = True
    original = ex.local_planner
    try:
        task.request(ex, "road-rules")
        assert task.last_result["status"] == "rejected"
        assert "workers" in task.last_result["reason"]
        assert ex.local_planner is original
    finally:
        release.set()
        worker.join(2)
        ex.stop()


def test_failed_candidate_does_not_change_source_observations(demo, monkeypatch):
    from avlite.c20_planning.c23_local_planning_strategy import LocalPlanningStrategy

    ex, task = demo.create_stack()
    ex.pm.regulatory_route = ["original route"]
    ex.pm.regulatory_observation = {"original": True}
    original = ex.local_planner

    class BrokenCandidate:
        def __init__(self, route, pm, setting):
            pm.regulatory_route = []
            pm.regulatory_observation.clear()
            raise ValueError("candidate failed after mutation")

    monkeypatch.setitem(LocalPlanningStrategy.registry, "BehaviorPlanningPipeline", BrokenCandidate)
    try:
        task.request(ex, "road-rules")
        assert task.last_result["status"] == "rejected"
        assert ex.pm.regulatory_route == ["original route"]
        assert ex.pm.regulatory_observation == {"original": True}
        assert ex.local_planner is original
    finally:
        ex.stop()


def test_perception_swap_cannot_use_old_command_without_new_observation(demo):
    from avlite.c10_perception.c12_perception_strategy import PerceptionPipeline

    if not hasattr(SyncExecuter, "stack_update"):
        pytest.skip("Requires the proposed core execution API")
    ex, _ = demo.create_stack(executor=SyncExecuter)
    try:
        advance(ex)
        ex.stop()
        demo.brake(ex)
        old = ex.perception
        ex.pm.environment_observation = object()
        replacement = PerceptionPipeline(ex.pm)
        event = demo.plugin.SelectPerceptionStack(replacement, ex.pm.map)
        demo.plugin.PerceptionSwitchTask().execute(ex, event)
        assert ex.perception is replacement and ex.perception is not old
        assert ex.pm.environment_observation is None
        pose = (ex.ego_state.x, ex.ego_state.y)
        for _ in range(3):
            ex.step(sim_dt=0.05, call_perceive=False)
        assert (ex.ego_state.x, ex.ego_state.y) == pose
        assert ex._last_cmd is None
        advance(ex)
    finally:
        ex.stop()


def test_reset_discards_observations_from_previous_run(demo):
    ex, _ = demo.create_stack()
    try:
        ex.pm.environment_observation = object()
        ex.pm.regulatory_observation = object()
        ex.reset()
        assert ex.pm.environment_observation is None
        assert ex.pm.regulatory_observation is None
    finally:
        ex.stop()
