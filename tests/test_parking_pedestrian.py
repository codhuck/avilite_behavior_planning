"""A parked-car overlap or a dead walker must not become a passing crossing."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest
from shapely.affinity import rotate, translate
from shapely.geometry import LineString, box


@pytest.fixture
def staging():
    path = Path(__file__).parents[1] / "tools" / "parking_pedestrian.py"
    spec = importlib.util.spec_from_file_location("parking_pedestrian_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("yaw", [0, np.deg2rad(11), np.pi / 2])
def test_crossing_avoids_neighbours_and_ego(staging, yaw):
    bodies = [
        translate(rotate(body, yaw, origin=(0, 0), use_radians=True), 50, 4)
        for body in [box(-2, -1, 2, 1), box(3, 1, 5, 3)]
    ]
    origin, direction = staging.crossing_origin([50, 4], yaw, bodies)
    corridor = LineString([origin, origin + 5 * direction]).buffer(0.4)
    assert all(not corridor.intersects(body) for body in bodies)
    expected = np.array([2.5, 2.5]) @ np.array(
        [[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]]
    ) + [50, 4]
    assert np.allclose(origin, expected)


def test_impossible_crossing_is_rejected(staging):
    with pytest.raises(ValueError, match="corridor"):
        staging.crossing_origin([0, 0], 0, [box(-10, -10, 10, 10)])


def test_continuity_rejects_disappearance_without_rejecting_pause(staging):
    assert staging.valid_walker_motion([58, 4, 1], [58, 4, 1], 0.1, 1)
    assert staging.valid_walker_motion([58, 4, 1], [58, 4.1, 1], 0.1, 1)
    assert not staging.valid_walker_motion([58, 4, 1], [0, 0, 0], 0.1, 1)
    assert not staging.valid_walker_motion([58, 4, 1], [58, 4, -10], 0.1, 1)
    assert not staging.valid_walker_motion(None, [float("nan"), 4, 1], 0.1, 1)
