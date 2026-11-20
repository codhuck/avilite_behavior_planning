"""Finite-ray ranking: sampling, device identity, occlusion and refusal boundary."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from shapely.geometry import GeometryCollection, box


def load(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).parents[1] / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def model():
    return load("scan_visibility"), load("task_visibility")


def task(points, occupied=None):
    return SimpleNamespace(
        points=np.array(points, dtype=float).reshape(-1, 2),
        weights=np.ones(len(points)),
        spacing=0.1,
        occupied=GeometryCollection() if occupied is None else occupied,
    )


def scorer(model, target, sensors=None, **kw):
    api, geometry = model
    return api.ScanVisibility(
        target,
        sensors or [geometry.GroundLidar(0, 0, 1, (-45,), 10)],
        azimuth_samples=4,
        phases=1,
        body_length=0.1,
        body_width=0.1,
        body_top=0.1,
        body_bottom=0,
        resolution=0.1,
        hazard_support_radius=0,
        **kw,
    )


def test_sparse_azimuth_is_not_continuous_ring(model):
    # Both points lie at r=1, but only the first receives one of four beams.
    target = task([[1.01, 0.01], [0.71, 0.71]])
    assert scorer(model, target).evaluate([0, 0, 0]).expected_area == pytest.approx(0.01)


def test_range_is_slant_not_horizontal(model):
    _, geometry = model
    target = task([[1.01, 0.01]])
    sensor = geometry.GroundLidar(0, 0, 1, (-45,), 1.1)
    assert scorer(model, target, [sensor]).evaluate([0, 0, 0]).expected_area == 0


def test_distinct_devices_not_duplicate_beams(model):
    _, geometry = model
    target = task([[1.01, 0.01]])
    sensor = geometry.GroundLidar(0, 0, 1, (-45, -45, -45), 10)
    assert scorer(model, target, [sensor], minimum_support=2).evaluate([0, 0, 0]).expected_area == 0
    with pytest.raises(ValueError, match="distinct"):
        scorer(model, target, [sensor, sensor])


def test_known_blocker_masks_prediction(model):
    target = task([[1.01, 0.01]], box(0.4, -0.1, 0.6, 0.1))
    assert scorer(model, target).evaluate([0, 0, 0]).expected_area == 0


def test_slab_self_occlusion(model):
    api, _ = model
    ends = np.array([[2, 0, 0], [-3, 0, 0], [2, 3, 0]])
    blocked = api._blocked_by_body(np.array([2, 0, 0.5]), ends, 3.7, 1.8, 0.1, 1.5)
    np.testing.assert_array_equal(blocked, [False, True, False])


def test_stationary_predicted_support_is_not_removed(model):
    obj = scorer(model, task([[1.01, 0.01]]))
    assert obj.evaluate([0, 0, 0]).expected_area > 0
    assert obj.advantage([0, 0, 0], [0, 0, 0], 0) == 0
    assert obj.advantage([0, 0, 0], [0, 0, 0], 1) < 0


def test_movement_can_measure_previously_unmeasured_cell(model):
    obj = scorer(model, task([[2.01, 0.01]]))
    assert obj.evaluate([0, 0, 0]).expected_area == 0
    assert obj.advantage([0, 0, 0], [1, 0, 0], 1) > 0


def test_empty_task_and_negative_coordinates(model):
    assert scorer(model, task([])).evaluate([0, 0, 0]).expected_area == 0
    assert scorer(model, task([[-0.99, -1.99]])).evaluate([-2, -2, 0]).expected_area > 0


@pytest.mark.parametrize("pose", [[0, 0], [0, 0, float("nan")], [0, 0, 0, 0]])
def test_invalid_pose_rejected(model, pose):
    with pytest.raises(ValueError):
        scorer(model, task([])).evaluate(pose)


def test_prediction_cannot_authorize_unknown_motion(model):
    gate = load("visibility_dispatch")
    free = box(-2.1, -1.1, 2.1, 1.1)
    snapshot = gate.VisibilitySnapshot(free, GeometryCollection(), 0)
    before = snapshot.free.wkb
    scorer(model, task([[2.01, 0.01]])).evaluate([1, 0, 0])
    x = np.linspace(0, 1, 26)
    candidate = gate.Maneuver(
        "view", "observe", "parallel", np.column_stack((x, x * 0, x * 0)), 1, 0.1, 1e6
    )
    result = gate.VisibilityDispatcher().select(
        [candidate], snapshot, 0, [0, 0, 0], 0, 4, 2, {"parallel"}
    )
    assert result.state == "STOP"
    assert snapshot.free.wkb == before


def test_normalized_gain_floor_and_safety_gate(model):
    gate = load("visibility_dispatch")
    snapshot = gate.VisibilitySnapshot(box(-10, -10, 10, 10), GeometryCollection(), 0)
    x = np.linspace(0, 1, 26)
    candidate = gate.Maneuver(
        "view", "observe", "parallel", np.column_stack((x, x * 0, x * 0)), 1, 0.1, 0.03
    )
    dispatcher = gate.VisibilityDispatcher()
    args = ([candidate], snapshot, 0, [0, 0, 0], 0, 4, 2, {"parallel"})
    assert dispatcher.select(*args).state == "STOP"  # Legacy threshold unchanged.
    assert dispatcher.select(*args, min_view_utility=0.01).state == "OBSERVE"
    assert dispatcher.select(*args, min_view_utility=0.02).state == "STOP"
    assert (
        dispatcher.select(*args, min_view_utility=float("nan")).reason
        == "invalid_view_utility_threshold"
    )


@pytest.mark.parametrize("speed,expected", [(0, [1500, 1500, 6000, 1500]), (0.1, [1500] * 4)])
def test_deterministic_recovery_budget_is_bounded_and_stopped_only(monkeypatch, speed, expected):
    import importlib
    import types

    package = types.ModuleType("scan_budget_test")
    package.__path__ = [str(Path(__file__).parents[1])]
    monkeypatch.setitem(sys.modules, package.__name__, package)
    api = importlib.import_module(package.__name__ + ".visibility_parking")
    planner = object.__new__(api.VisibilityParkingPlanner)
    planner.pm = SimpleNamespace(
        ego_vehicle=SimpleNamespace(velocity=speed),
        parking_occupancy=SimpleNamespace(status="observed", occupied=GeometryCollection()),
    )
    planner.obstacles = []
    planner.search_timeout = 2
    planner.recovery_search_timeout = 5
    planner.search_expansions = 1500
    planner.recovery_search_expansions = 6000
    planner.deterministic_search = True
    planner._search_failures = 0
    seen = []

    def generate(self):
        seen.append(self.search_expansions)
        return np.zeros((2, 3)) if len(seen) == 3 else None

    monkeypatch.setattr(api.ParkingPlanner, "_generate", generate)
    for _ in range(4):
        planner._generate()
        assert planner.search_expansions == 1500
        assert planner.search_timeout == 2
    assert seen == expected
