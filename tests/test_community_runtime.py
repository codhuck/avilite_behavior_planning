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
    command = sys.modules[
        demo.plugin.__name__ + ".execution.commands"
    ].DirectionalAckermannCommand()
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


def test_default_signal_geometry_is_independent_of_cwd(demo, tmp_path, monkeypatch):
    import json

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(demo.plugin.PluginSettings, "sensor_signal_geometry", "")
    detector = demo.plugin.SensorRegulatoryDetector(
        detector=lambda image: [], signal_backend="carla_aperture"
    )
    expected = json.loads((demo.PLUGIN / "data/Town10HD_Opt_signals.json").read_text())
    assert detector.geometry == expected
    assert detector.geometry


def test_profile_generator_resolves_assets_from_tools_directory(tmp_path, monkeypatch):
    root = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location("layout_profile", root / "tools/profile_setup.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    road = tmp_path / "road.xodr"
    road.write_text("<OpenDRIVE/>")
    bridge = tmp_path / "bridge"
    bridge.mkdir()
    (bridge / "__init__.py").touch()
    monkeypatch.chdir(tmp_path)
    output = module.write_profile(tmp_path / "profile.yaml", road, bridge)
    profile = yaml.safe_load(output.read_text())
    assert profile["c69_apps"]["c62_community_plugins"]["avlite-behavior-hdmap"] == str(root)
    assert Path(profile["c40_execution"]["c40_global_trajectory"]).is_file()
    assert Path(profile["plugins"]["avlite_behavior_hdmap"]["sensor_signal_geometry"]).is_file()


def test_evidence_snapshot_includes_nested_code_without_run_artifacts(tmp_path):
    spec = importlib.util.spec_from_file_location(
        "layout_sources", Path(__file__).parents[1] / "tools/_sources.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    included = ["__init__.py", "planning/road.py", "parking/search.py", "tools/check.py"]
    excluded = ["logs/snapshot/old.py", ".venv/lib/dependency.py", "parking/__pycache__/old.py"]
    for relative in included + excluded:
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    assert {p.relative_to(tmp_path).as_posix() for p in module.source_files(tmp_path)} == set(
        included
    )
