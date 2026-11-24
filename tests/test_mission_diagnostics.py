"""Static preflight must report faults without changing files or live settings."""

import importlib
import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml


@pytest.fixture(scope="module")
def helpers():
    tools = Path(__file__).parents[1] / "tools"
    sys.path.insert(0, str(tools))
    try:
        bootstrap = importlib.import_module("_bootstrap")
        plugin = bootstrap.load_plugin()
        diagnostics = importlib.import_module(plugin.__name__ + ".execution.mission_diagnostics")
        spec = importlib.util.spec_from_file_location(
            "diagnostic_profiles", tools / "prepare_mission_profiles.py"
        )
        generator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(generator)
        yield plugin, diagnostics, generator
    finally:
        sys.path.remove(str(tools))


@pytest.fixture
def files(helpers, tmp_path):
    _, _, generator = helpers
    profiles = generator.prepare(tmp_path / "profiles")
    profile = yaml.safe_load(profiles[0].read_text())
    settings = profile["plugins"]["avlite_behavior_hdmap"]
    return profiles, profile, settings


def write_profile(files):
    profiles, profile, _ = files
    profiles[0].write_text(yaml.safe_dump(profile))
    return profiles[0]


@pytest.mark.parametrize("mode", [0, 1, 2])
def test_generated_profiles_pass_without_mutation(helpers, files, mode):
    plugin, diagnostics, _ = helpers
    root = files[0][0].parent
    before = {p: p.read_bytes() for p in root.rglob("*") if p.is_file()}
    settings = plugin.PluginSettings.model_dump()
    report = diagnostics.diagnose_mission(files[0][mode])
    assert report["passed"] and not report["errors"] and not report["warnings"]
    assert settings == plugin.PluginSettings.model_dump()
    assert before == {p: p.read_bytes() for p in root.rglob("*") if p.is_file()}


def test_reports_independent_missing_files_together(helpers, files):
    _, diagnostics, _ = helpers
    profiles, profile, settings = files
    for field in ("mission_file", "parking_scene_file"):
        settings[field] += ".missing"
    profile["c40_execution"]["c40_global_trajectory"] += ".missing"
    report = diagnostics.diagnose_mission(write_profile(files))
    assert {e["field"] for e in report["errors"]} == {
        "mission_file",
        "parking_scene_file",
        "c40_global_trajectory",
    }


@pytest.mark.parametrize(
    "field,value,expected",
    [
        ("c40_executer_type", "AsyncThreadedExecuter", "c40_executer_type"),
        ("c40_execution_tasks", [], "c40_execution_tasks"),
        (
            "c40_execution_tasks",
            ["ParkingMissionTask", "ParkingMissionTask"],
            "c40_execution_tasks",
        ),
        ("c40_execution_tasks", None, "c40_execution_tasks"),
        ("c40_start_pose", [0, float("nan"), 0], "c40_start_pose"),
        ("c40_bridge", "CarlaBridge", "parking_simulator_obstacles"),
    ],
)
def test_incompatible_execution_settings(helpers, files, field, value, expected):
    _, diagnostics, _ = helpers
    files[1]["c40_execution"][field] = value
    report = diagnostics.diagnose_mission(write_profile(files))
    assert expected in {e["field"] for e in report["errors"]}


@pytest.mark.parametrize(
    "filename,field,value,expected",
    [
        ("mission_file", "approach", [500, 500, 0], "mission.approach"),
        ("mission_file", "parking_type", "diagonal", "mission_file"),
        ("mission_file", "spaces", ["missing_space"], "spaces.missing_space"),
        ("mission_file", "wait_timeout", 0.1, "wait_timeout"),
        ("parking_scene_file", "demo", True, "parking_scene_file.demo"),
        ("parking_scene_file", "drivable", [[0, 0], [1, 1], [2, 2]], "parking_scene_file.drivable"),
        ("parking_scene_file", "spaces", [], "parking_scene_file.spaces"),
    ],
)
def test_invalid_mission_or_scene(helpers, files, filename, field, value, expected):
    _, diagnostics, _ = helpers
    path = Path(files[2][filename])
    data = json.loads(path.read_text())
    data[field] = value
    path.write_text(json.dumps(data))
    report = diagnostics.diagnose_mission(files[0][0])
    assert expected in {e["field"] for e in report["errors"]}


def test_goal_outside_bay_is_rejected(helpers, files):
    _, diagnostics, _ = helpers
    path = Path(files[2]["parking_scene_file"])
    data = json.loads(path.read_text())
    data["spaces"]["demo_bay"]["goals"]["perpendicular_forward"] = [100, 100, 0]
    path.write_text(json.dumps(data))
    report = diagnostics.diagnose_mission(files[0][0])
    assert "spaces.demo_bay" in {e["field"] for e in report["errors"]}


@pytest.mark.parametrize(
    "field,value",
    [("ReferenceSpeed", [1]), ("LeftBound", [float("nan")]), ("ReferenceLine", [[0, 0], [0, 0]])],
)
def test_malformed_route_is_rejected(helpers, files, field, value):
    _, diagnostics, _ = helpers
    path = Path(files[1]["c40_execution"]["c40_global_trajectory"])
    data = json.loads(path.read_text())
    data[field] = value
    path.write_text(json.dumps(data))
    report = diagnostics.diagnose_mission(files[0][0])
    assert "c40_global_trajectory" in {e["field"] for e in report["errors"]}


def test_protects_inputs_from_status_output(helpers, files):
    _, diagnostics, _ = helpers
    files[2]["decision_status_file"] = files[2]["mission_file"]
    report = diagnostics.diagnose_mission(write_profile(files))
    assert "decision_status_file" in {e["field"] for e in report["errors"]}


def test_relative_route_uses_avlite_data_lookup(helpers, files, monkeypatch):
    _, diagnostics, _ = helpers
    path = Path(files[1]["c40_execution"]["c40_global_trajectory"])
    monkeypatch.setenv("AVLITE_DATA_DIR", str(path.parent))
    files[1]["c40_execution"]["c40_global_trajectory"] = "data/" + path.name
    report = diagnostics.diagnose_mission(write_profile(files))
    assert report["passed"]
    assert report["files"]["c40_global_trajectory"] == str(path)


def test_cli_json_and_nonzero_exit_for_invalid_profile(files):
    Path(files[2]["mission_file"]).unlink()
    command = [
        sys.executable,
        str(Path(__file__).parents[1] / "tools/diagnose_mission.py"),
        "--profile",
        str(files[0][0]),
        "--json",
    ]
    result = subprocess.run(
        command, capture_output=True, text=True, env={**os.environ, "DISPLAY": ""}
    )
    assert result.returncode == 1
    report = json.loads(result.stdout)
    assert not report["passed"] and "Traceback" not in result.stderr


def test_damaged_yaml_and_missing_profile_are_readable_errors(helpers, tmp_path):
    _, diagnostics, _ = helpers
    path = tmp_path / "profile.yaml"
    assert not diagnostics.diagnose_mission(path)["passed"]
    path.write_text("c40_execution: [broken")
    assert not diagnostics.diagnose_mission(path)["passed"]


def test_lidar_mission_profile_checks_calibration_and_conflicts(helpers, files):
    _, diagnostics, _ = helpers
    _, profile, settings = files
    profile["c40_execution"]["c40_execution_tasks"] = ["ParkingMissionTask", "DecisionStatusTask"]
    settings["parking_lidar_occupancy"] = True
    report = diagnostics.diagnose_mission(write_profile(files))
    assert {"parking_ground_z", "parking_simulator_obstacles"} <= {
        e["field"] for e in report["errors"]
    }
    settings["parking_ground_z"] = 0.0
    settings["parking_simulator_obstacles"] = False
    report = diagnostics.diagnose_mission(write_profile(files))
    assert report["passed"]
    assert any(e["field"] == "parking_lidar_occupancy" for e in report["warnings"])
    settings["parking_active_observation"] = True
    report = diagnostics.diagnose_mission(write_profile(files))
    assert "parking_active_observation" in {e["field"] for e in report["errors"]}
