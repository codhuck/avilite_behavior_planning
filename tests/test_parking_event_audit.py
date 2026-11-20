"""Independent coordinate and failure checks for the raw-cloud event audit."""

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from shapely.geometry import box, mapping


@pytest.fixture
def recording(tmp_path):
    module_path = Path(__file__).parents[1] / "tools" / "audit_parking_events.py"
    spec = importlib.util.spec_from_file_location("parking_event_audit", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = dict(type="parallel", repeat=0, scenario=dict(id="crossing", pedestrian=True))
    (tmp_path / "results.json").write_text(json.dumps([result]))
    directory = tmp_path / "crossing"
    directory.mkdir()
    pose = [10, 20, 0, np.pi / 2]
    geometry = dict(
        pose=pose, obstacles=[mapping(box(0, 0, 1, 1))] * 2 + [mapping(box(10, 21, 11, 22))]
    )
    (directory / "parallel_0_geometry_200.json").write_text(json.dumps(geometry))
    mount = np.eye(4)
    mount[:3, 3] = [1, 0, 0.5]
    for index in range(9):
        np.savez(
            directory / f"parallel_0_event_200_cloud_{index}.npz",
            points=[[0.5, -0.5, 0.5, 1], [0.5, -0.5, -0.5, 1]],
            stamp=20.0,
            base_to_sensor=mount,
            ego_pose=pose,
        )
    return module, tmp_path


def test_audit_uses_mount_pose_and_excludes_ground(recording):
    module, directory = recording
    result = module.audit(directory)
    assert result["all_events_supported"]
    assert result["rows"][0]["total_points"] == 9


def test_audit_rejects_unsynchronized_sensor(recording):
    module, directory = recording
    path = directory / "crossing" / "parallel_0_event_200_cloud_8.npz"
    with np.load(path) as source:
        data = dict(source)
    data["stamp"] = 19.9
    np.savez(path, **data)
    assert not module.audit(directory)["all_events_supported"]


def test_audit_does_not_ignore_case_with_missing_recording(recording):
    module, directory = recording
    path = directory / "results.json"
    results = json.loads(path.read_text())
    results.append(dict(type="parallel", repeat=0, scenario=dict(id="missing", pedestrian=True)))
    path.write_text(json.dumps(results))
    assert not module.audit(directory)["all_events_supported"]


def test_roof_only_requires_one_recorded_device_but_surround_requires_nine(recording):
    module, directory = recording
    for index in range(1, 9):
        (directory / "crossing" / f"parallel_0_event_200_cloud_{index}.npz").unlink()
    assert not module.audit(directory)["all_events_supported"]
    path = directory / "results.json"
    results = json.loads(path.read_text())
    results[0]["policy_configuration"] = dict(roof_only=True)
    path.write_text(json.dumps(results))
    assert module.audit(directory)["all_events_supported"]
