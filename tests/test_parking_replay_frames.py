"""Legacy replay must not mix later events or another trial into its initial frame."""

import importlib
from pathlib import Path

import numpy as np
import pytest


@pytest.fixture
def replay(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).parents[1] / "tools"))
    return importlib.import_module("replay_parking_coverage")


def cloud(directory, name, stamp):
    np.savez(directory / name, points=[[1, 0, 0, 1]], stamp=stamp, base_to_sensor=np.eye(4))


def test_initial_replay_excludes_events_and_other_trials(replay, tmp_path):
    cloud(tmp_path, "parallel_0_cloud_0.npz", 1)
    cloud(tmp_path, "parallel_0_cloud_1.npz", 1)
    cloud(tmp_path, "parallel_0_event_200_cloud_0.npz", 21)
    cloud(tmp_path, "parallel_1_cloud_0.npz", 50)
    result = replay.initial_clouds(tmp_path, "parallel_0")
    assert len(result) == 2
    assert {c.stamp for c in result} == {1}
    assert [c.sensor_name for c in result] == ["lidar_0", "lidar_1"]


def test_initial_replay_rejects_missing_or_misaligned_acquisitions(replay, tmp_path):
    with pytest.raises(ValueError, match="acquisition time"):
        replay.initial_clouds(tmp_path, "parallel_0")
    cloud(tmp_path, "parallel_0_cloud_0.npz", 1)
    cloud(tmp_path, "parallel_0_cloud_1.npz", 1.1)
    with pytest.raises(ValueError, match="acquisition time"):
        replay.initial_clouds(tmp_path, "parallel_0")
