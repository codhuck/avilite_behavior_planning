"""Replay must not mix measurements from different capture times."""

import importlib
import sys
from pathlib import Path

import numpy as np
import pytest


@pytest.fixture
def api(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).parents[1] / "tools"))
    monkeypatch.delitem(sys.modules, "compare_parking_evidence", raising=False)
    return importlib.import_module("compare_parking_evidence")


def test_initial_frame_excludes_event_clouds(api, tmp_path):
    (tmp_path / "parallel_0_geometry_0.json").write_text('{"pose": [0, 0, 0, 0]}')
    for i in range(9):
        for prefix, stamp in (("parallel_0", 1), ("parallel_0_event_200", 21)):
            np.savez(
                tmp_path / f"{prefix}_cloud_{i}.npz",
                points=np.zeros((1, 4)),
                stamp=stamp,
                base_to_sensor=np.eye(4),
            )
    _, clouds, _, hashes = api.load_frame(tmp_path, "parallel", 0, {"length": 4, "width": 2})
    assert len(clouds) == 9
    assert all(c.stamp == 1 for c in clouds)
    assert len(hashes) == 10


def test_mismatched_capture_time_is_rejected(api, tmp_path):
    (tmp_path / "parallel_0_geometry_0.json").write_text('{"pose": [0, 0, 0, 0]}')
    for i in range(9):
        np.savez(
            tmp_path / f"parallel_0_cloud_{i}.npz",
            points=np.zeros((1, 4)),
            stamp=2 if i == 8 else 1,
            base_to_sensor=np.eye(4),
        )
    with pytest.raises(ValueError, match="timestamps"):
        api.load_frame(tmp_path, "parallel", 0, {"length": 4, "width": 2})
