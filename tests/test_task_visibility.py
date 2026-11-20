"""View utility cannot turn predicted information into motion permission."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from shapely.geometry import GeometryCollection, box


def module(name):
    spec = importlib.util.spec_from_file_location(
        name,
        Path(__file__).parents[1]
        / {
            "scan_visibility": "parking/scan.py",
            "task_visibility": "parking/view.py",
            "visibility_dispatch": "parking/dispatch.py",
            "parking_search": "parking/search.py",
        }[name],
    )
    obj = importlib.util.module_from_spec(spec)
    sys.modules[name] = obj
    spec.loader.exec_module(obj)
    return obj


def test_zero_gain_without_change_of_view():
    api = module("task_visibility")
    snapshot = SimpleNamespace(free=box(-2, -2, 1, 2), occupied=box(2, -0.5, 3, 0.5))
    path = np.column_stack((np.linspace(0, 8, 81), np.zeros(81), np.zeros(81)))
    scorer = api.TaskVisibility(snapshot, path, 2, 4)
    assert scorer.evaluate([0, 0, 0], [0, 0, 0]).weighted_gain == 0
    assert scorer.evaluate([0, 0, 0], [0, 3, 0]).weighted_gain > 0


def test_no_gain_for_fully_observed_task():
    api = module("task_visibility")
    snapshot = SimpleNamespace(free=box(-10, -10, 20, 10), occupied=GeometryCollection())
    scorer = api.TaskVisibility(snapshot, [[0, 0, 0], [5, 0, 0]], 2, 4)
    assert scorer.evaluate([0, 0, 0], [1, 0, 0]).samples == 0


def test_remote_unknown_space_does_not_count():
    api = module("task_visibility")
    path = np.column_stack((np.linspace(0, 5, 51), np.zeros(51), np.zeros(51)))
    snapshot = SimpleNamespace(free=box(-10, -3, 10, 3), occupied=GeometryCollection())
    assert (
        api.TaskVisibility(snapshot, path, 2, 4).evaluate([0, 0, 0], [0, 4, 0]).weighted_gain == 0
    )


def test_gain_does_not_mutate_snapshot_or_bypass_gate():
    api = module("task_visibility")
    gate = module("visibility_dispatch")
    snapshot = gate.VisibilitySnapshot(box(-3, -2, 1, 2), box(2, -0.5, 3, 0.5), 0)
    original = snapshot.free.wkb
    path = np.column_stack((np.linspace(0, 8, 201), np.zeros(201), np.zeros(201)))
    api.TaskVisibility(snapshot, path, 2, 4).evaluate([0, 0, 0], [0, 3, 0])
    candidate = gate.Maneuver("unsafe", "observe", "parallel", path, 1, 0.1, 1e6)
    decision = gate.VisibilityDispatcher().select(
        [candidate], snapshot, 0, [0, 0, 0], 0, 4, 2, {"parallel"}
    )
    assert decision.state == "STOP"
    assert original == snapshot.free.wkb


@pytest.mark.parametrize("spacing", [0, -1, float("nan")])
def test_invalid_spacing_rejected(spacing):
    api = module("task_visibility")
    snapshot = SimpleNamespace(free=box(-1, -1, 1, 1), occupied=GeometryCollection())
    with pytest.raises(ValueError):
        api.TaskVisibility(snapshot, [[0, 0, 0], [1, 0, 0]], 2, 4, spacing=spacing)


def test_fixed_expansion_search_ignores_wall_clock(monkeypatch):
    api = module("parking_search")
    calls = []
    for increment in [0.00001, 100.0]:
        clock = iter(np.arange(10000) * increment)
        monkeypatch.setattr(api.time, "monotonic", lambda: next(clock))
        paths, reason = api.search(
            (0, 0, 0),
            (6, 0, 0),
            3.7,
            1.8,
            box(-5, -5, 12, 5),
            [],
            final_direction=1,
            timeout=None,
            max_expansions=100,
        )
        assert reason == "found"
        calls.append(np.vstack([p for _, p in paths]))
    np.testing.assert_array_equal(*calls)


def test_expansion_budget_refuses_without_guessing_path():
    api = module("parking_search")
    paths, reason = api.search(
        (0, 0, 0),
        (6, 0, 0),
        3.7,
        1.8,
        box(-5, -5, 12, 5),
        [],
        final_direction=1,
        timeout=None,
        max_expansions=1,
    )
    assert paths is None and reason == "search_budget_exhausted"


def test_beam_ring_predicts_ground_intersection_and_support():
    api = module("task_visibility")
    task = SimpleNamespace(
        points=np.array([[1.0, 0], [2.0, 0]]), spacing=0.1, occupied=GeometryCollection()
    )
    sensor = api.GroundLidar(0, 0, 1, (-45.0,), 10)
    np.testing.assert_array_equal(
        api.beam_visibility(task, [0, 0, 0], [sensor], hazard_support_radius=0), [True, False]
    )
    np.testing.assert_array_equal(
        api.beam_visibility(task, [0, 0, 0], [sensor], minimum_support=2, hazard_support_radius=0),
        [False, False],
    )


def test_known_occluder_blocks_beam_prediction():
    api = module("task_visibility")
    task = SimpleNamespace(
        points=np.array([[1.0, 0]]), spacing=0.1, occupied=box(0.4, -0.1, 0.6, 0.1)
    )
    sensor = api.GroundLidar(0, 0, 1, (-45.0,), 10)
    assert not api.beam_visibility(task, [0, 0, 0], [sensor], hazard_support_radius=0)[0]


def test_beam_gain_zero_for_identical_view():
    api = module("task_visibility")
    task = SimpleNamespace(
        points=np.array([[1.0, 0]]), weights=np.ones(1), spacing=0.1, occupied=GeometryCollection()
    )
    sensor = api.GroundLidar(0, 0, 1, (-45.0,), 10)
    assert api.beam_gain(task, [0, 0, 0], [0, 0, 0], [sensor], hazard_support_radius=0) == 0
