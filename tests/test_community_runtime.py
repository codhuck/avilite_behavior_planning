"""Exercise runtime adapters against both stock and development AVLite."""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml
from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
from avlite.c40_execution.c49_settings import ExecutionSettingsSchema


@pytest.fixture(scope="module")
def demo():
    path = Path(__file__).parents[1] / "tools/coordinator_demo.py"
    spec = importlib.util.spec_from_file_location("community_runtime_demo", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("mode", ["road", "race"])
def test_stopped_handover_runs_fresh_tick(demo, mode):
    ex, task = demo.create_stack(mode)
    try:
        original_world = ex.world
        original_perception = ex.perception
        ex.step(sim_dt=0.05, replan_dt=100, control_dt=100)
        assert task.last_result["status"] == "selected"
        demo.brake(ex)
        task.request(ex, "road-rules" if mode == "road" else "race")
        assert task.last_result["status"] == "selected"
        assert ex._last_cmd is None
        ex.step(sim_dt=0.05, replan_dt=100, control_dt=100)
        assert ex._last_cmd is not None
        assert ex.world is original_world and ex.perception is original_perception
        assert ex.pm.simulation_time == pytest.approx(ex.elapsed_sim_time - 0.05)
    finally:
        ex.stop()


def test_restart_does_not_select_again(demo):
    ex, task = demo.create_stack("road")
    try:
        ex.step(sim_dt=0.05)
        planner = ex.local_planner
        count = len(task.history)
        ex.stop()
        ex.step(sim_dt=0.05)
        assert ex.local_planner is planner and len(task.history) == count
    finally:
        ex.stop()


def test_profile_is_portable_and_only_registers_itself():
    path = Path(__file__).parents[1] / "avlite-behavior-hdmap.yaml"
    profile = yaml.safe_load(path.read_text())
    assert profile["c69_apps"]["c62_community_plugins"] == {
        "avlite-behavior-hdmap": "avlite-behavior-hdmap"
    }
    assert "/home/" not in path.read_text()
    assert profile["c40_execution"]["c40_bridge"] == "PlanningBasicSim"
    assert profile["plugins"]["avlite_behavior_hdmap"]["cruise_speed_limit"] == 3.0


@pytest.mark.parametrize("dt", [0, -1, np.nan, np.inf])
def test_invalid_timestep_does_not_move_vehicle(demo, dt):
    ego = EgoState(velocity=1)
    world = demo.plugin.PlanningBasicSim(
        ego,
        pm=PerceptionModel(ego_vehicle=ego),
        setting=ExecutionSettingsSchema(c46_npc_control=False),
    )
    command = sys.modules[demo.plugin.__name__ + ".commands"].DirectionalAckermannCommand()
    with pytest.raises(ValueError):
        world.control_ego_state(command, dt)
    assert (ego.x, ego.y, ego.velocity) == (0, 0, 1)


def test_speed_limit_applies_to_reference(demo, monkeypatch):
    monkeypatch.setattr(demo.plugin.PluginSettings, "cruise_speed_limit", 0.7)
    ex, _ = demo.create_stack("race")
    try:
        ex.step(sim_dt=0.05)
        ex.local_planner.replan(ex.pm)
        plan = ex.local_planner.get_local_plan()
        assert max(plan.as_trajectory().velocity) <= 0.7
    finally:
        ex.stop()
