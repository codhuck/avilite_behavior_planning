"""Search invariance and collision checks in transformed parking scenes."""

import importlib.util
import math
from pathlib import Path

import numpy as np
from shapely.affinity import affine_transform
from shapely.geometry import box


def test_search_is_rigid_transform_equivariant():
    spec = importlib.util.spec_from_file_location(
        "parking_search_test", Path(__file__).parents[1] / "parking/search.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    baseline = None
    for angle, dx, dy in [(0, 0, 0), (0.37, 20.3, -9.1), (-0.12, -3, 5)]:
        c, s = math.cos(angle), math.sin(angle)
        rotation = np.array([[c, -s], [s, c]])

        def transform(p):
            xy = rotation @ np.array(p[:2]) + [dx, dy]
            return (*xy, p[2] + angle)

        segments, reason = module.search(
            transform((0, 0, 0)),
            transform((6, 0, 0)),
            3.7,
            1.8,
            affine_transform(box(-5, -5, 12, 5), [c, -s, s, c, dx, dy]),
            [],
            final_direction=1,
            timeout=2,
        )
        assert reason == "found"
        points = np.vstack([p for _, p in segments])
        points[:, :2] = (points[:, :2] - [dx, dy]) @ rotation
        points[:, 2] -= angle
        if baseline is None:
            baseline = points
        else:
            np.testing.assert_allclose(points, baseline, atol=1e-10)


def test_near_goal_refinement_failure_keeps_expanding():
    spec = importlib.util.spec_from_file_location(
        "parking_search_near_test", Path(__file__).parents[1] / "parking/search.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    segments, reason = module.search(
        (0, 0, 0),
        (0.65, 0, 0),
        2,
        1,
        box(-5, -5, 5, 5),
        [],
        final_direction=1,
        refine_goal=True,
        timeout=2,
    )
    assert reason == "found"
    assert np.linalg.norm(segments[-1][1][-1, :2] - [0.65, 0]) < 0.01
    assert abs(segments[-1][1][-1, 2]) < math.radians(0.5)
