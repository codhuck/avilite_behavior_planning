"""Mission handovers, failure latches and the decision-status output."""

import copy
import importlib.util
import json
import math
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import pytest
import yaml
from avlite.c10_perception.c11_perception_model import EgoState
from avlite.c30_control.c31_control_model import ControlCommand
from avlite.c40_execution.c49_settings import ExecutionSettings
from shapely.geometry import box


@pytest.fixture(scope="module")
def runner():
    tools = Path(__file__).parents[1] / "tools"
    sys.path.insert(0, str(tools))
    try:
        spec = importlib.util.spec_from_file_location(
            "mission_test_runner", tools / "check_missions.py"
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        yield module
    finally:
        sys.path.remove(str(tools))


@pytest.fixture
def profiles(runner, tmp_path):
    settings = runner.plugin.PluginSettings.model_dump()
    execution = ExecutionSettings.model_dump()
    try:
        yield runner.prepare(tmp_path / "profiles")
    finally:
        for key, value in settings.items():
            setattr(runner.plugin.PluginSettings, key, value)
        for key, value in execution.items():
            setattr(ExecutionSettings, key, value)


@pytest.fixture
def stack(runner, profiles):
    ex, mission, status = runner.create_stack(profiles[0])
    runner.plugin.PluginSettings.decision_status_file = ""
    try:
        yield ex, mission, status
    finally:
        ex.stop()


def tick(ex, **kwargs):
    ex.step(sim_dt=0.05, control_dt=0.05, replan_dt=0.05, **kwargs)


def until(ex, mission, state, limit=1200):
    for _ in range(limit):
        tick(ex)
        if mission.state == state:
            return
        assert mission.state != "BLOCKED", mission.reason
    pytest.fail(f"Did not reach {state}: {mission.state} / {mission.reason}")


@pytest.mark.parametrize("mode", [0, 1, 2])
def test_complete_drive_and_park(runner, profiles, mode):
    result = runner.run(profiles[mode])
    assert result["passed"], result["reason"]
    assert result["approach_distance_m"] > 7
    transitions = [r["state"] for r in result["history"]]
    assert transitions[:4] == ["APPROACH", "SETTLING", "SELECT_SPACE", "PARKING"]
    assert transitions[-1] == "COMPLETE"


@pytest.mark.parametrize(
    "case,reason",
    [
        ("occupied", "no_confirmed_free_space_timeout"),
        ("unknown", "parking_observation_unavailable_timeout"),
        ("stale", "parking_observation_stale_timeout"),
        ("free_later", "parked_in_selected_space"),
    ],
)
def test_observation_and_occupancy_outcomes(runner, profiles, case, reason):
    result = runner.run(profiles[0], case)
    assert result["passed"] and result["reason"] == reason


@pytest.mark.parametrize(
    "field,value",
    [
        ("approach", [float("nan"), 0, 0]),
        ("parking_type", "diagonal"),
        ("spaces", ["a", "a"]),
        ("wait_timeout", -1),
        ("progress_timeout", float("inf")),
        ("settle_time", 0),
        ("unknown_option", True),
    ],
)
def test_invalid_mission_brakes_before_first_command(stack, runner, field, value):
    ex, mission, _ = stack
    path = Path(runner.plugin.PluginSettings.mission_file)
    data = json.loads(path.read_text())
    data[field] = value
    path.write_text(json.dumps(data))
    position = (ex.ego_state.x, ex.ego_state.y)
    tick(ex)
    assert mission.state == "BLOCKED"
    assert ex._last_cmd.acceleration <= 0
    assert (ex.ego_state.x, ex.ego_state.y) == position


def test_missing_mission_is_visible_and_stationary(stack, runner):
    ex, mission, _ = stack
    runner.plugin.PluginSettings.mission_file += ".missing"
    tick(ex)
    assert mission.state == "BLOCKED" and ex.ego_state.velocity == 0


def test_hold_overrides_old_command_even_without_control_tick(stack):
    ex, _, _ = stack
    ex.hold_motion("test", "operator_hold")
    ex._last_cmd = ControlCommand(acceleration=10)
    ex.ego_state.velocity = 1
    ex._simulate_step(0.1)
    assert ex.ego_state.velocity < 1
    ex.release_motion("test")
    assert ex._last_cmd is None and ex._fresh_planning


def test_pause_resume_preserves_mission_progress(stack):
    ex, mission, _ = stack
    for _ in range(20):
        tick(ex)
    history = list(mission.history)
    ex.stop()
    tick(ex)
    assert mission.state == "APPROACH" and mission.history == history


def test_disabled_tasks_stop_and_do_not_auto_resume(stack):
    ex, mission, _ = stack
    tick(ex, call_tasks=False)
    assert mission.state == "BLOCKED" and ex.ego_state.velocity == 0
    tick(ex)
    assert mission.reason == "mission_task_disabled"


def test_route_replacement_blocks_before_next_acceleration(stack):
    ex, mission, _ = stack
    tick(ex)
    ex.local_planner.global_plan = copy.copy(ex.local_planner.global_plan)
    tick(ex)
    assert mission.reason == "mission_stack_changed"
    assert ex._last_cmd.acceleration <= 0


def test_world_speed_checked_before_handover(stack, runner, monkeypatch):
    ex, mission, _ = stack
    ex.stop()
    monkeypatch.setattr(ex.world, "get_ego_state", lambda: EgoState(velocity=1))
    original = ex.local_planner
    with pytest.raises(ValueError, match="Stop the vehicle"):
        mission.parking.start_parking(ex, "perpendicular_forward", "demo_bay")
    assert ex.local_planner is original


def test_nonfinite_localization_rejected(stack, monkeypatch):
    ex, mission, _ = stack
    ex.stop()
    monkeypatch.setattr(ex.world, "get_ego_state", lambda: EgoState(x=float("nan")))
    with pytest.raises(ValueError, match="finite localization"):
        mission.parking.start_parking(ex, "perpendicular_forward", "demo_bay")


def test_cancel_latches_and_requires_paused_api(stack):
    ex, mission, _ = stack
    tick(ex)
    with pytest.raises(ValueError, match="Pause"):
        mission.cancel(ex)
    ex.stop()
    mission.cancel(ex)
    for _ in range(20):
        tick(ex)
    assert mission.state == "CANCELLED" and ex.ego_state.velocity == 0


def test_parked_stack_restored_before_reset(stack):
    ex, mission, _ = stack
    approach = ex.local_planner
    until(ex, mission, "PARKING")
    assert ex.local_planner is not approach
    ex.reset()
    assert ex.local_planner is approach and mission.state == "IDLE"
    assert not ex.motion_holds
    tick(ex)
    assert mission.state == "APPROACH"


def test_candidate_preflight_failure_retains_stack(stack, runner):
    ex, mission, _ = stack
    ex.stop()
    ego = ex.ego_state
    ego.x, ego.y, ego.theta = -12, 0, 0
    runner.plugin.PluginSettings.parking_simulator_obstacles = False
    ex.pm.parking_observation = runner.plugin.ParkingObservation((box(-1, 9, 1, 13),), 0)
    original = (ex.local_planner, ex.controller)
    with pytest.raises(ValueError, match="no_feasible"):
        mission.parking.start_parking(
            ex, "perpendicular_forward", "demo_bay", require_feasible=True
        )
    assert (ex.local_planner, ex.controller) == original


def test_selection_works_before_parking_planner_exists(stack, runner):
    ex, mission, _ = stack
    assert mission.parking.available_spaces(ex, "perpendicular_forward") == ["demo_bay"]
    runner.plugin.PluginSettings.parking_simulator_obstacles = False
    for stamp in (-1, 1, float("nan")):
        ex.pm.parking_observation = runner.plugin.ParkingObservation((), stamp)
        assert mission.parking.available_spaces(ex, "perpendicular_forward") == []


def test_occupied_first_choice_uses_second_candidate(stack, runner):
    ex, mission, _ = stack
    scene_path = Path(runner.plugin.PluginSettings.parking_scene_file)
    scene = json.loads(scene_path.read_text())
    scene["spaces"]["occupied"] = copy.deepcopy(scene["spaces"]["demo_bay"])
    scene["spaces"]["occupied"]["bay"] = [[-5, 8], [-2, 8], [-2, 14], [-5, 14]]
    scene["spaces"]["occupied"]["goals"]["perpendicular_forward"] = [-3.5, 11, math.pi / 2]
    scene_path.write_text(json.dumps(scene))
    path = Path(runner.plugin.PluginSettings.mission_file)
    data = json.loads(path.read_text())
    data["spaces"] = ["occupied", "demo_bay"]
    path.write_text(json.dumps(data))
    until(ex, mission, "PARKING")
    assert mission.space == "demo_bay"
    assert mission.candidate_results == {"occupied": "occupied", "demo_bay": "accepted"}


def test_status_file_contains_actual_decision(stack, runner, tmp_path):
    ex, mission, status = stack
    path = tmp_path / "monitor" / "status.json"
    runner.plugin.PluginSettings.decision_status_file = str(path)
    tick(ex)
    data = json.loads(path.read_text())
    assert data["state"] == mission.state == "APPROACH"
    assert data["run_id"] == status.run_id
    assert data["observation_source"] == "BasicSim demo"
    assert data["observation_age_s"] == 0
    assert len(list(path.parent.iterdir())) == 1


def test_status_write_failure_does_not_abort_driving(stack, runner, tmp_path):
    ex, mission, status = stack
    runner.plugin.PluginSettings.decision_status_file = str(tmp_path)
    tick(ex)
    assert mission.state == "APPROACH" and status._write_error


def test_profile_generation_refuses_overwrite(runner, profiles):
    before = profiles[0].read_bytes()
    with pytest.raises(FileExistsError):
        runner.prepare(profiles[0].parent)
    assert profiles[0].read_bytes() == before
    profile = yaml.safe_load(before)
    assert profile["c40_execution"]["c40_execution_tasks"][-1] == "DecisionStatusTask"


def test_blocked_selection_requires_explicit_retry(stack, runner):
    ex, mission, _ = stack
    tick(ex)
    runner.plugin.PluginSettings.parking_simulator_obstacles = False
    mission.spec = replace(mission.spec, wait_timeout=2.0)
    until(ex, mission, "BLOCKED")
    assert mission.reason == "parking_observation_unavailable_timeout"
    ex.pm.parking_observation = runner.plugin.ParkingObservation((), ex.elapsed_sim_time)
    tick(ex)
    assert mission.state == "BLOCKED"
    with pytest.raises(ValueError, match="Pause"):
        mission.retry(ex)
    ex.stop()
    mission.retry(ex)
    assert mission.state == "SELECT_SPACE"
    tick(ex)
    assert mission.state == "PARKING" and not ex.motion_holds


def test_loss_of_observations_during_parking_stops(stack, runner):
    ex, mission, _ = stack
    until(ex, mission, "PARKING")
    mission.spec = replace(mission.spec, progress_timeout=1.0)
    runner.plugin.PluginSettings.parking_simulator_obstacles = False
    until(ex, mission, "BLOCKED", limit=100)
    assert mission.reason == "no_progress_timeout"
    assert ex.ego_state.velocity <= 0.02


def test_wrong_approach_heading_never_hands_over(stack, runner):
    ex, mission, _ = stack
    path = Path(runner.plugin.PluginSettings.mission_file)
    data = json.loads(path.read_text())
    data["approach"][2] = math.pi
    path.write_text(json.dumps(data))
    planner = ex.local_planner
    until(ex, mission, "BLOCKED")
    assert mission.reason == "approach_pose_outside_tolerance"
    assert ex.local_planner is planner


def test_infeasible_first_choice_uses_second_candidate(stack, runner):
    ex, mission, _ = stack
    scene_path = Path(runner.plugin.PluginSettings.parking_scene_file)
    scene = json.loads(scene_path.read_text())
    scene["spaces"]["narrow"] = copy.deepcopy(scene["spaces"]["demo_bay"])
    scene["spaces"]["narrow"]["bay"] = [[-0.1, 9], [0.1, 9], [0.1, 12], [-0.1, 12]]
    scene_path.write_text(json.dumps(scene))
    path = Path(runner.plugin.PluginSettings.mission_file)
    data = json.loads(path.read_text())
    data["spaces"] = ["narrow", "demo_bay"]
    path.write_text(json.dumps(data))
    until(ex, mission, "PARKING")
    assert mission.space == "demo_bay"
    assert "no_feasible" in mission.candidate_results["narrow"]


def test_completion_rechecks_physical_pose(stack):
    ex, mission, _ = stack
    until(ex, mission, "PARKING")
    ex.local_planner.state = "PARKED"
    ex.local_planner._terminal = True
    tick(ex)
    assert mission.state == "BLOCKED"
    assert mission.reason == "parking_completion_pose_mismatch"


def test_malformed_root_does_not_start_vehicle(stack, runner):
    ex, mission, _ = stack
    Path(runner.plugin.PluginSettings.mission_file).write_text("[]")
    tick(ex)
    assert mission.state == "BLOCKED" and ex.ego_state.velocity == 0


def test_monitor_prints_decision_without_display(stack, runner, tmp_path):
    ex, _, _ = stack
    path = tmp_path / "monitor.json"
    runner.plugin.PluginSettings.decision_status_file = str(path)
    tick(ex)
    result = subprocess.run(
        [
            sys.executable,
            str(Path(__file__).parents[1] / "tools/mission_monitor.py"),
            "--status",
            str(path),
            "--once",
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    assert "State: APPROACH" in result.stdout
    assert "following approach route" in result.stdout
    assert "Observation: BasicSim demo" in result.stdout
