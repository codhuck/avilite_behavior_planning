"""Sensor-only occupancy checks: unknown regions never become free by omission."""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest
from avlite.c10_perception.c11_perception_model import EgoState
from avlite.c50_common.c52_world_sensor_datatypes import Lidar
from shapely.geometry import Point, box


@pytest.fixture
def provider():
    spec = importlib.util.spec_from_file_location(
        "parking_occupancy_test", Path(__file__).parents[1] / "parking/occupancy.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.LidarParkingOccupancy()


def cloud(points, stamp=10):
    return Lidar(points=np.asarray(points, dtype=float).reshape(-1, 4), stamp=stamp)


@pytest.mark.parametrize("stamp", [9, 11, None])
def test_missing_stale_future_stops(provider, stamp):
    result = provider.observe(cloud([[0, 0, 0, 1]], stamp), EgoState(), box(-2, -2, 2, 2), 10, 0)
    assert result.free.is_empty


def test_empty_is_unknown(provider):
    result = provider.observe(cloud([]), EgoState(), box(-2, -2, 2, 2), 10, 0)
    assert result.status == "empty_or_invalid_cloud"
    assert result.free.is_empty


def test_ground_return_does_not_clear_unseen_cells(provider):
    result = provider.observe(cloud([[0.1, 0.1, 0, 1]]), EgoState(), box(-2, -2, 2, 2), 10, 0)
    assert result.free.covers(Point(0.1, 0.1))
    assert result.blocked.covers(Point(1, 1))


def test_object_overrides_ground_in_same_cell(provider):
    result = provider.observe(
        cloud([[0.1, 0.1, 0, 1], [0.1, 0.1, 0.5, 1]]), EgoState(), box(-2, -2, 2, 2), 10, 0
    )
    assert result.free.is_empty


def test_mount_and_localization_are_applied(provider):
    lidar = cloud([[0.1, 0.1, -1, 1]])
    lidar.base_to_sensor[2, 3] = 1
    result = provider.observe(lidar, EgoState(x=5, y=3), box(0, 0, 10, 10), 10, 0)
    assert result.free.covers(Point(5.1, 3.1))
    assert result.blocked.covers(Point(0.1, 0.1))


def test_nonfinite_cloud_rejected(provider):
    result = provider.observe(cloud([[np.nan, 0, 0, 1]]), EgoState(), box(-2, -2, 2, 2), 10, 0)
    assert result.free.is_empty


def test_clearance_required_for_candidate(provider):
    points = [[x, y, 0, 1] for x in np.arange(0.125, 3, 0.25) for y in np.arange(0.125, 5, 0.25)]
    result = provider.observe(cloud(points), EgoState(), box(-2, -2, 8, 8), 10, 0)
    assert result.permits(box(0.5, 0.5, 2.5, 4.5))
    assert not result.permits(box(0, 0, 3, 5))


def test_ground_beneath_connected_object_surface_is_blocked(provider):
    points = [[x, y, 0, 1] for x in np.arange(0.125, 3, 0.25) for y in np.arange(0.125, 3, 0.25)]
    points += [[x, y, 1, 1] for x in np.arange(0.125, 3, 0.25) for y in [0.125, 2.875]]
    points += [[x, y, 1, 1] for y in np.arange(0.125, 3, 0.25) for x in [0.125, 2.875]]
    result = provider.observe(cloud(points), EgoState(), box(-2, -2, 8, 8), 10, 0)
    assert result.blocked.covers(box(0.5, 0.5, 2.5, 2.5))


@pytest.mark.parametrize("now, ground", [(None, 0), (10, None), (float("nan"), 0)])
def test_unknown_clock_or_ground_blocks(provider, now, ground):
    result = provider.observe(cloud([[0.1, 0.1, 0, 1]]), EgoState(), box(-2, -2, 2, 2), now, ground)
    assert result.free.is_empty


def fused_provider():
    spec = importlib.util.spec_from_file_location(
        "parking_fusion_tests", Path(__file__).parents[1] / "parking/occupancy.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.FusedLidarParkingOccupancy()


def test_fusion_obstacle_wins_across_devices():
    provider = fused_provider()
    ego = EgoState(length=2, width=1)
    result = provider.observe(
        [cloud([[3.1, 0.1, 0, 1]]), cloud([[3.1, 0.1, 0.5, 1]])], ego, box(-5, -5, 5, 5), 10, 0
    )
    assert not result.traversable.covers(Point(3.1, 0.1))


def test_fusion_self_exception_does_not_clear_neighboring_blind_space():
    result = fused_provider().observe(
        [cloud([[3.1, 0.1, 0, 1]])], EgoState(length=2, width=1), box(-5, -5, 5, 5), 10, 0
    )
    assert result.traversable.covers(Point(0, 0))
    assert not result.free.covers(Point(0, 0))
    assert not result.traversable.covers(Point(1.1, 0))
    assert not result.blocked.covers(Point(0, 0))


def test_fusion_rejects_wrong_acquisition_pose_time_and_does_not_persist_free():
    provider = fused_provider()
    ego = EgoState(length=2, width=1)
    area = box(-5, -5, 5, 5)
    assert provider.observe([cloud([[3.1, 0.1, 0, 1]])], ego, area, 10, 0).status == "observed"
    result = provider.observe([cloud([[3.1, 0.1, 0, 1]], stamp=9.99)], ego, area, 10, 0)
    assert result.free.is_empty
    assert result.self_footprint is None


def test_fusion_calibrates_each_sensor_before_combining():
    source = cloud([[0.1, 0.1, 0, 1]])
    source.base_to_sensor[0, 3] = 3
    result = fused_provider().observe(
        [source], EgoState(length=2, width=1), box(-5, -5, 5, 5), 10, 0
    )
    assert result.free.covers(Point(3.1, 0.1))
    assert not result.free.covers(Point(0.1, 0.1))


def test_redundant_support_counts_devices_not_number_of_returns():
    provider = fused_provider()
    provider.minimum_support = 2
    ego, area = EgoState(length=2, width=1), box(-5, -5, 5, 5)
    one = cloud([[3.1, 0.1, 0, 1]] * 100)
    one.sensor_name = "front"
    two = cloud([[3.1, 0.1, 0, 1]])
    two.sensor_name = "side"
    assert not provider.observe([one, one], ego, area, 10, 0).free.covers(Point(3.1, 0.1))
    assert provider.observe([one, two], ego, area, 10, 0).free.covers(Point(3.1, 0.1))
    two.stamp = 9.9
    assert not provider.observe([one, two], ego, area, 10, 0).free.covers(Point(3.1, 0.1))


def test_missing_ground_calibration_is_not_a_runtime_crash():
    result = fused_provider().observe(
        [cloud([[3, 0, 0, 1]])], EgoState(), box(-5, -5, 5, 5), 10, None
    )
    assert result.status == "invalid_clock_or_ground"
    assert result.traversable.is_empty


def test_run_length_geometry_keeps_holes_and_matches_cell_union():
    from shapely.ops import unary_union

    provider = fused_provider()
    api = sys.modules[provider.__class__.__module__]
    rng = np.random.default_rng(2027)
    cells = {tuple(v) for v in rng.integers(-10, 10, size=(200, 2))}
    expected = unary_union(
        [box(x * 0.25, y * 0.25, (x + 1) * 0.25, (y + 1) * 0.25) for x, y in cells]
    )
    actual = api.ground_cells_geometry(cells, 0.25)
    assert expected.equals(actual)


def test_adaptive_support_requires_corroboration_near_hazards_only():
    provider = fused_provider()
    provider.hazard_support_radius = 2
    ego, area = EgoState(length=2, width=1), box(-6, -6, 6, 6)
    one = cloud([[3.1, 0, 0.5, 1], [4.1, 0, 0, 1], [-3.1, 0, 0, 1]])
    one.sensor_name = "front"
    result = provider.observe([one], ego, area, 10, 0)
    assert not result.free.covers(Point(4.1, 0))
    assert result.free.covers(Point(-3.1, 0))
    two = cloud([[4.1, 0, 0, 1]])
    two.sensor_name = "side"
    assert provider.observe([one, two], ego, area, 10, 0).free.covers(Point(4.1, 0))


def test_fragmented_vehicle_surfaces_do_not_leave_free_ground_inside_cluster():
    source = cloud([[3.1, 0, 0.5, 1], [4.6, 0, 0.5, 1], [3.85, 0.1, 0, 1], [-3.1, 0, 0, 1]])
    result = fused_provider().observe(
        [source], EgoState(length=2, width=1), box(-6, -6, 6, 6), 10, 0
    )
    assert not result.free.covers(Point(3.85, 0.1))
    assert result.occupied.covers(Point(3.85, 0.1))
    assert result.free.covers(Point(-3.1, 0))


def test_noisy_self_surface_mask_does_not_clear_surrounding_space():
    provider = fused_provider()
    ego, area = EgoState(length=2, width=1), box(-6, -6, 6, 6)
    source = cloud([[1.015, 0, 0.5, 1], [3.1, 0, 0, 1]])
    result = provider.observe([source], ego, area, 10, 0)
    assert result.occupied.is_empty
    assert not result.free.covers(Point(1.015, 0))
    assert not result.traversable.covers(Point(1.015, 0))
    assert result.self_footprint.equals(box(-1, -0.5, 1, 0.5))
    source = cloud([[1.1, 0, 0.5, 1], [3.1, 0, 0, 1]])
    assert provider.observe([source], ego, area, 10, 0).occupied.covers(Point(1.1, 0))


def test_missing_frame_clears_support_diagnostics():
    provider = fused_provider()
    ego, area = EgoState(length=2, width=1), box(-6, -6, 6, 6)
    provider.observe([cloud([[3.1, 0, 0, 1]])], ego, area, 10, 0)
    assert provider.support_histogram
    provider.observe([], ego, area, 11, 0)
    assert provider.support_histogram == {}
