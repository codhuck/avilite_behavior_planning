"""Arbitration boundary tests independent of CARLA and privileged world state."""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest
from shapely.geometry import GeometryCollection, box


@pytest.fixture
def api():
    spec = importlib.util.spec_from_file_location(
        "visibility_dispatch_tests", Path(__file__).parents[1] / "visibility_dispatch.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def maneuver(api, kind="park", end=2, gain=1):
    x = np.linspace(0, end, int(abs(end) / 0.04) + 1)
    return api.Maneuver(
        kind, kind, "parallel", np.column_stack((x, x * 0, x * 0)), 1 if end > 0 else -1, 0.2, gain
    )


def select(
    api, candidates, free=None, occupied=None, age=0, velocity=0, allowed=None, dispatcher=None
):
    snapshot = api.VisibilitySnapshot(
        box(-5, -5, 10, 5) if free is None else free,
        GeometryCollection() if occupied is None else occupied,
        0,
    )
    dispatcher = dispatcher or api.VisibilityDispatcher()
    return dispatcher.select(
        candidates,
        snapshot,
        age,
        [0, 0, 0],
        velocity,
        4,
        2,
        {"parallel"} if allowed is None else allowed,
    )


def test_parking_has_priority_over_speculative_gain(api):
    result = select(api, [maneuver(api, "observe", gain=100), maneuver(api)])
    assert result.state == "PARK"


def test_partial_coverage_does_not_allow_parking(api):
    result = select(api, [maneuver(api)], free=box(-5, -5, 2, 5))
    assert result.state == "STOP"


def test_checked_observation_selected(api):
    result = select(
        api, [maneuver(api, end=7), maneuver(api, "observe", end=1)], free=box(-5, -5, 5, 5)
    )
    assert result.state == "OBSERVE"


@pytest.mark.parametrize("age", [-0.01, 0.31, float("nan")])
def test_invalid_age_stops(api, age):
    assert select(api, [maneuver(api)], age=age).state == "STOP"


def test_stopping_envelope_can_reject_currently_clear_path(api):
    candidate = maneuver(api)
    free = box(-3, -1.5, 6, 1.5)
    assert (
        select(
            api, [candidate], free=free, dispatcher=api.VisibilityDispatcher(unseen_speed=0)
        ).state
        == "PARK"
    )
    assert select(api, [candidate], free=free, velocity=1).state == "STOP"


def test_obstacle_overrides_free_mask(api):
    assert select(api, [maneuver(api)], occupied=box(0.5, -0.2, 1, 0.2)).state == "STOP"


def test_disallowed_parking_type_rejected(api):
    assert select(api, [maneuver(api)], allowed={"perpendicular_reverse"}).state == "STOP"


def test_budget_and_visited_views_prevent_loop(api):
    dispatcher = api.VisibilityDispatcher(max_views=1)
    dispatcher.complete_view([1, 0, 0])
    result = select(api, [maneuver(api, "observe")], dispatcher=dispatcher)
    assert result.reason == "observation_budget_exhausted"
    dispatcher.reset()
    assert select(api, [maneuver(api, "observe")], dispatcher=dispatcher).state == "OBSERVE"


def test_lateral_slide_rejected(api):
    candidate = maneuver(api)
    candidate.poses[:, [0, 1]] = candidate.poses[:, [1, 0]]
    assert select(api, [candidate]).state == "STOP"


def test_out_of_order_frame_rejected(api):
    dispatcher = api.VisibilityDispatcher()
    dispatcher.last_stamp = 1
    assert select(api, [maneuver(api)], dispatcher=dispatcher).reason == "out_of_order_observation"


def test_expected_gain_is_occlusion_sensitive(api):
    snapshot = api.VisibilitySnapshot(box(-3, -3, 8, 0.5), box(2, 1, 4, 4), 0)
    target = box(6, 3, 8, 5)
    assert api.expected_gain([6, 0, 0], snapshot, target) > api.expected_gain(
        [0, 0, 0], snapshot, target
    )


def test_visibility_adapter_is_registered_and_cancellation_latched():
    import importlib
    import types

    from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
    from avlite.c20_planning.c21_planning_model import GlobalPlan
    from avlite.c20_planning.c23_local_planning_strategy import LocalPlanningStrategy
    from avlite.c50_common.c54_trajectory_tracker import TrajectoryTracker

    package = types.ModuleType("visibility_adapter_test")
    package.__path__ = [str(Path(__file__).parents[1])]
    sys.modules[package.__name__] = package
    module = importlib.import_module(package.__name__ + ".visibility_parking")
    parking = importlib.import_module(package.__name__ + ".parking")
    pm = PerceptionModel(ego_vehicle=EgoState(x=-9, y=0))
    pm.simulation_time = 0
    path = [(-9, 0), (0, 11)]
    planner = module.VisibilityParkingPlanner(
        GlobalPlan(path=path, trajectory=TrajectoryTracker(path=path, velocity=[0, 0])), pm
    )
    assert "VisibilityParkingPlanner" in LocalPlanningStrategy.registry
    planner.request_parking(
        parking.ParkingRequest((0, 11, np.pi / 2), box(-1.6, 8, 1.6, 14), box(-16, -5, 8, 15))
    )
    # Even legacy explicit occupancy does not bypass the visibility requirement.
    planner.update_obstacles([], 0)
    assert max(planner.replan().velocity) == 0
    assert planner.reason == "missing_visibility_observation"
    planner.cancel()
    assert max(planner.replan().velocity) == 0
    assert planner.state == "PARKING_CANCELLED"


def test_search_respects_requested_entry_direction():
    spec = importlib.util.spec_from_file_location(
        "visibility_search_tests", Path(__file__).parents[1] / "parking_search.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result, reason = module.search(
        (0, 0, 0),
        (2, 0, 0),
        2,
        1,
        box(-10, -10, 10, 10),
        [],
        final_direction=1,
        initial_direction=-1,
        timeout=2,
    )
    assert result is not None, reason
    assert result[0][0] == -1
    assert result[-1][0] == 1


def test_endpoint_refinement_is_exact_and_collision_checked():
    spec = importlib.util.spec_from_file_location(
        "visibility_refinement_test", Path(__file__).parents[1] / "parking_search.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    edges = [(1, module.rollout((0, 0, 0), 1, 0, 1))]
    goal = (0.95, 0.015, 0.02)
    refined = module.refine_endpoint(edges, goal, lambda a, b: True)
    assert refined is not None
    end = refined[-1][1][-1]
    assert np.linalg.norm(end[:2] - goal[:2]) < 0.01
    assert abs(module.wrap(end[2] - goal[2])) < np.deg2rad(0.5)
    assert module.refine_endpoint(edges, goal, lambda a, b: False) is None


def test_prefix_never_releases_unobserved_tail(api):
    candidate = maneuver(api, end=5)
    snapshot = api.VisibilitySnapshot(box(-5, -5, 4, 5), GeometryCollection(), 10)
    dispatcher = api.VisibilityDispatcher(allow_prefix=True)
    result = dispatcher.select([candidate], snapshot, 10, [0, 0, 0], 0, 2, 1, {"parallel"})
    assert result.state == "PARK"
    assert result.maneuver.poses[-1, 0] < 2.5
    assert len(result.maneuver.poses) < len(candidate.poses)
    assert dispatcher.admissible(result.maneuver, snapshot, 10, [0, 0, 0], 0, 2, 1)


def test_prefix_does_not_release_if_current_stop_envelope_is_unknown(api):
    candidate = maneuver(api, end=5)
    snapshot = api.VisibilitySnapshot(box(-0.1, -5, 4, 5), GeometryCollection(), 10)
    result = api.VisibilityDispatcher(allow_prefix=True).select(
        [candidate], snapshot, 10, [0, 0, 0], 0, 2, 1, {"parallel"}
    )
    assert result.state == "STOP"


def test_speed_is_reduced_only_when_stopping_envelope_is_certified(api):
    from dataclasses import replace

    candidate = replace(maneuver(api, end=3), speed=0.4)
    snapshot = api.VisibilitySnapshot(box(-5, -1, 8, 1), GeometryCollection(), 10)
    dispatcher = api.VisibilityDispatcher(allow_prefix=True, adaptive_speed=True)
    decision = dispatcher.select([candidate], snapshot, 10, [0, 0, 0], 0, 2, 1, {"parallel"})
    assert decision.state == "PARK"
    assert decision.maneuver.speed == 0.1
    assert decision.certificate.speed_bound == 0.1
    # A lower command cannot retroactively reduce actual stopping distance.
    moving = dispatcher.select([candidate], snapshot, 10, [0, 0, 0], 0.4, 2, 1, {"parallel"})
    assert moving.state == "STOP"
    assert moving.certificate is None


@pytest.mark.parametrize("speed,expected", [(0, [2, 2, 5, 2]), (0.1, [2, 2, 2, 2])])
def test_search_budget_escalates_only_while_stopped(monkeypatch, speed, expected):
    import importlib
    import types

    package = types.ModuleType("visibility_budget_test")
    package.__path__ = [str(Path(__file__).parents[1])]
    monkeypatch.setitem(sys.modules, package.__name__, package)
    module = importlib.import_module(package.__name__ + ".visibility_parking")
    planner = object.__new__(module.VisibilityParkingPlanner)
    planner.pm = types.SimpleNamespace(
        ego_vehicle=types.SimpleNamespace(velocity=speed),
        parking_occupancy=types.SimpleNamespace(status="observed", occupied=GeometryCollection()),
    )
    planner.obstacles = [box(20, 20, 21, 21)]
    saved = planner.obstacles
    planner.search_timeout = 2
    planner.recovery_search_timeout = 5
    planner._search_failures = 0
    calls = []

    def generate(self):
        calls.append(self.search_timeout)
        return np.zeros((2, 3)) if len(calls) == 3 else None

    monkeypatch.setattr(module.ParkingPlanner, "_generate", generate)
    for _ in range(4):
        planner._generate()
        assert planner.search_timeout == 2
        assert planner.obstacles is saved
    assert calls == expected


@pytest.mark.parametrize(
    "speed,reason,discard",
    [
        (0, "no_admissible_maneuver", True),
        (0.1, "no_admissible_maneuver", False),
        (0, "stale_or_future_observation", False),
    ],
)
@pytest.mark.parametrize("allow_observation", [False, True])
def test_stopped_rejected_obstructed_path_is_discarded(
    monkeypatch, speed, reason, discard, allow_observation
):
    import importlib
    import types

    from avlite.c20_planning.c21_planning_model import LocalPlan

    package = types.ModuleType("visibility_obstruction_test")
    package.__path__ = [str(Path(__file__).parents[1])]
    monkeypatch.setitem(sys.modules, package.__name__, package)
    module = importlib.import_module(package.__name__ + ".visibility_parking")
    planner = object.__new__(module.VisibilityParkingPlanner)
    planner.pm = types.SimpleNamespace(
        ego_vehicle=types.SimpleNamespace(x=0, y=0, theta=0, velocity=speed, length=4, width=2),
        simulation_time=5,
        parking_occupancy=types.SimpleNamespace(
            status="observed", free=box(-10, -10, 10, 10), occupied=GeometryCollection()
        ),
    )
    planner._view = None
    planner.allow_observation = allow_observation
    planner._observed_time = 5
    planner.sensor_occupancy = None
    planner.rear_offset = 0
    planner.cruise_speed = 0.4
    planner._terminal = False
    planner.request = types.SimpleNamespace(
        parking_type=types.SimpleNamespace(value="parallel"), bay=box(0, 0, 5, 5), goal=(4, 0, 0)
    )
    planner.poses = np.array([[0, 0, 0], [0.04, 0, 0]])
    candidates_seen = []

    def decide(candidates, *args):
        candidates_seen.extend(candidates)
        return types.SimpleNamespace(state="STOP", reason=reason)

    planner.arbiter = types.SimpleNamespace(select=decide)

    def replan(self, *args):
        self.state, self.reason = "PARKING_BLOCKED", "obstacle_or_boundary"
        return LocalPlan(path=[(0, 0), (0.04, 0)], velocity=[0, 0])

    monkeypatch.setattr(module.ParkingPlanner, "replan", replan)
    output = planner.replan()
    assert max(output.velocity) == 0
    assert (planner.poses is None) == discard
    assert any(c.kind == "observe" for c in candidates_seen) == (allow_observation and speed < 0.02)
    if discard:
        assert planner._search_time == 5
        assert planner._attempted_revision is None


@pytest.mark.parametrize("measured", [0.0, 0.15, 0.35])
def test_lower_requested_speed_preserves_admissible_fixed_path(api, measured):
    """A fixed path/snapshot has monotone stopping-distance and inflation bounds."""
    from dataclasses import replace

    for half_width in (1.5, 1.8, 2.2, 3.0):
        free = box(-4, -half_width, 6, half_width)
        accepted = []
        for speed in (0.05, 0.1, 0.2, 0.4):
            candidate = replace(maneuver(api), speed=speed)
            accepted.append(select(api, [candidate], free=free, velocity=measured).state == "PARK")
        assert accepted == sorted(accepted, reverse=True)
