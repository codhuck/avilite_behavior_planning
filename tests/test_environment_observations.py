"""Public perception contract, independent producers and capability refusal."""

import importlib.util
import pickle
import sys
from dataclasses import FrozenInstanceError, dataclass
from pathlib import Path
from types import SimpleNamespace

import pytest
from avlite.c10_perception.c12_perception_strategy import DetectionStrategy, PerceptionPipeline
from avlite.c10_perception.c19_settings import PerceptionSettingsSchema
from avlite.c50_common.c51_capabilities import StackCapability, satisfies_requirements


@pytest.fixture(scope="module")
def api():
    path = Path(__file__).parents[1] / "tools/coordinator_demo.py"
    spec = importlib.util.spec_from_file_location("environment_contract_demo", path)
    demo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(demo)
    return SimpleNamespace(
        demo=demo,
        env=sys.modules[demo.plugin.__name__ + ".perception.environment"],
        observations=sys.modules[demo.plugin.__name__ + ".perception.observations"],
        rules=sys.modules[demo.plugin.__name__ + ".planning.hdmap_behavior"],
    )


def test_extensible_marker_without_core_enum(api):
    @dataclass(frozen=True, kw_only=True)
    class DockMarker(api.env.EnvironmentMarker):
        berth: str = "A"

    dock = DockMarker(id="dock-1", position=(1.0, 2.0, 3.0))
    observation = api.env.EnvironmentObservation(2.0, (dock,), "underwater-camera")
    assert observation.markers[0].berth == "A"
    assert observation.is_fresh(2.1, 0.2)
    with pytest.raises(FrozenInstanceError):
        dock.id = "changed"


def test_snapshot_is_pickle_transportable(api):
    env = api.env
    original = env.EnvironmentObservation(
        2.0,
        (env.TrafficSignal(id="42", state="Red"), env.CrossingOccupancy(id="9", occupied=None)),
        "camera",
    )
    assert pickle.loads(pickle.dumps(original)) == original


@pytest.mark.parametrize("now", [1.9, 2.6, float("nan"), float("inf")])
def test_old_future_or_invalid_clock_is_not_fresh(api, now):
    assert not api.env.EnvironmentObservation(2.0).is_fresh(now, 0.5)


def test_invalid_snapshot_rejected(api):
    env = api.env
    with pytest.raises(ValueError):
        env.EnvironmentObservation(float("nan"))
    with pytest.raises(ValueError):
        env.EnvironmentObservation(1.0, [env.TrafficSignal(id="a")])
    with pytest.raises(ValueError):
        env.EnvironmentObservation(1.0, (env.TrafficSignal(id="a"), env.TrafficSignal(id="a")))
    with pytest.raises(ValueError):
        env.TrafficSignal(id="a", confidence=float("nan"))
    with pytest.raises(ValueError):
        env.CrossingOccupancy(id="a", occupied="false")


def test_unknown_or_wrong_frame_never_falls_back_to_green_truth(api):
    pm = SimpleNamespace(
        regulatory_observation=api.observations.RegulatoryObservation(1.0, {"42": "Green"}),
        environment_observation=api.env.EnvironmentObservation(1.0, (), "camera"),
    )
    assert api.observations.regulatory_observation(pm).lights == {}
    pm.environment_observation = api.env.EnvironmentObservation(
        1.0, (api.env.TrafficSignal(id="42", state="Green"),), "camera", "camera_optical"
    )
    assert api.observations.regulatory_observation(pm).lights == {}


def test_independent_producer_and_pipeline_capability_validation(api):
    env = api.env

    class AlternativeSignals(DetectionStrategy):
        stack_capabilities = frozenset({env.SIGNAL_OBSERVATIONS})

        def detect(self, perception_model=None, sensors=None, **kwargs):
            perception_model.environment_observation = env.EnvironmentObservation(
                1.0, (env.TrafficSignal(id="42", state="Red"),), "alternative-camera"
            )
            return perception_model

    ex, _ = api.demo.create_stack()
    try:
        planner = api.demo.plugin.HDMapBehaviorPlanner(ex.local_planner.global_plan, ex.pm)
        planner.route_rules = [
            SimpleNamespace(feature=SimpleNamespace(kind=api.rules.RegulatoryKind.TRAFFIC_LIGHT))
        ]
        requirements = planner.stack_requirements
        assert env.SIGNAL_OBSERVATIONS in requirements
        assert not satisfies_requirements(
            requirements,
            {StackCapability.MAP_HD, StackCapability.LOCALIZATION, StackCapability.DETECTION},
        )
        detector = AlternativeSignals()
        pipeline = PerceptionPipeline(
            ex.pm,
            PerceptionSettingsSchema(
                c12_detection_strategy="AlternativeSignals",
                c12_tracking_strategy="",
                c12_prediction_strategy="",
            ),
        )
        assert env.SIGNAL_OBSERVATIONS in pipeline.stack_capabilities
        detector.detect(ex.pm)
        assert api.observations.regulatory_observation(ex.pm).lights == {"42": "Red"}
        ex.local_planner = planner
        with pytest.raises(ValueError, match="ENVIRONMENT_SIGNAL_OBSERVATIONS"):
            ex.validate_stack()
        ex.perception = pipeline
        ex.validate_stack()
        planner.route_rules.append(
            SimpleNamespace(feature=SimpleNamespace(kind=api.rules.RegulatoryKind.CROSSWALK))
        )
        with pytest.raises(ValueError, match="ENVIRONMENT_CROSSING_OBSERVATIONS"):
            ex.validate_stack()
    finally:
        ex.stop()


def test_failed_sensor_frame_replaces_previous_snapshot_with_unknown(api):
    ex, _ = api.demo.create_stack()
    try:
        ex.pm.environment_observation = api.env.EnvironmentObservation(
            0.0, (api.env.TrafficSignal(id="42", state="Green"),), "rgb_lidar"
        )
        detector = api.demo.plugin.SensorRegulatoryDetector(detector=lambda image: [], geometry={})
        detector.detect(ex.pm, sensors=None)
        observation = ex.pm.environment_observation
        assert not observation.markers
        assert observation.status == "missing_or_unsynchronised_sensors"
        assert api.observations.regulatory_observation(ex.pm).lights == {}
        assert {
            api.env.SIGNAL_OBSERVATIONS,
            api.env.CROSSING_OBSERVATIONS,
        } <= detector.stack_capabilities
    finally:
        ex.stop()


@pytest.mark.parametrize(
    "state,stamp,status,expected",
    [
        ("Red", 2.0, "ok", "WAIT_LIGHT"),
        ("Green", 2.0, "ok", "CRUISE"),
        ("Green", 0.0, "ok", "PERCEPTION_STOP"),
        ("Green", 3.0, "ok", "PERCEPTION_STOP"),
        ("Green", 2.0, "camera_fault", "PERCEPTION_STOP"),
        ("Unknown", 2.0, "ok", "WAIT_LIGHT"),
    ],
)
def test_planner_consumes_independent_signal_snapshot(api, state, stamp, status, expected):
    from avlite.c20_planning.c21_planning_model import LocalPlan

    ex, _ = api.demo.create_stack()
    try:
        planner = api.demo.plugin.HDMapBehaviorPlanner(ex.local_planner.global_plan, ex.pm)
        feature = api.rules.RegulatoryFeature(
            id="signal",
            kind=api.rules.RegulatoryKind.TRAFFIC_LIGHT,
            road_id="0",
            s=10,
            t=0,
            x=10,
            y=0,
        )
        planner.route_rules = [api.rules.RouteRule(feature, 10, 0)]
        ex.pm.simulation_time = 2.0
        ex.pm.environment_observation = api.env.EnvironmentObservation(
            stamp,
            (api.env.TrafficSignal(id="signal", state=state),),
            "independent-camera",
            status=status,
        )
        plan = planner.plan_behavior(LocalPlan.from_trajectory(planner.global_plan.trajectory))
        assert plan.behavior_state == expected
    finally:
        ex.stop()


@pytest.mark.parametrize(
    "occupied,expected",
    [(True, "YIELD_PEDESTRIAN"), (False, "SLOW_CROSSWALK"), (None, "YIELD_PEDESTRIAN")],
)
def test_planner_uses_observed_crossing_state(api, occupied, expected):
    from avlite.c20_planning.c21_planning_model import LocalPlan

    ex, _ = api.demo.create_stack()
    try:
        planner = api.demo.plugin.HDMapBehaviorPlanner(ex.local_planner.global_plan, ex.pm)
        feature = api.rules.RegulatoryFeature(
            id="crossing",
            kind=api.rules.RegulatoryKind.CROSSWALK,
            road_id="0",
            s=10,
            t=0,
            x=10,
            y=0,
        )
        planner.route_rules = [api.rules.RouteRule(feature, 10, 0, 14)]
        ex.pm.simulation_time = 2.0
        ex.pm.environment_observation = api.env.EnvironmentObservation(
            2.0, (api.env.CrossingOccupancy(id="crossing", occupied=occupied),), "independent-lidar"
        )
        plan = planner.plan_behavior(LocalPlan.from_trajectory(planner.global_plan.trajectory))
        assert plan.behavior_state == expected
    finally:
        ex.stop()
