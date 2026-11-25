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
        (
            env.TrafficSignal(id="track-42", map_ref="42", state="Red"),
            env.CrossingOccupancy(id="9", occupied=None),
        ),
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
                1.0,
                (env.TrafficSignal(id="track-42", map_ref="42", state="Red"),),
                "alternative-camera",
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
            (api.env.TrafficSignal(id="detected-signal", map_ref="signal", state=state),),
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
            2.0,
            (
                api.env.CrossingOccupancy(
                    id="detected-crossing", map_ref="crossing", occupied=occupied
                ),
            ),
            "independent-lidar",
        )
        plan = planner.plan_behavior(LocalPlan.from_trajectory(planner.global_plan.trajectory))
        assert plan.behavior_state == expected
    finally:
        ex.stop()


@pytest.fixture
def sensor_case(api, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        "regulatory_frame_fixture", Path(__file__).with_name("test_planning_coordinator.py")
    )
    frames = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(frames)
    settings = sys.modules[api.demo.plugin.__name__ + ".settings"].PluginSettings
    monkeypatch.setattr(settings, "sensor_clock_offset", None)
    monkeypatch.setattr(settings, "observation_max_age", 0.3)
    detector_module = sys.modules[api.demo.plugin.__name__ + ".perception.detector"]
    ex, _ = api.demo.create_stack()
    ex.pm.simulation_time = 10.0
    detector = detector_module.SensorRegulatoryDetector(
        detector=lambda _: [], geometry={}, signal_backend="carla_aperture"
    )
    try:
        yield SimpleNamespace(
            pm=ex.pm, detector=detector, frame=frames.sensor_frame, settings=settings
        )
    finally:
        ex.stop()


def test_first_frame_keeps_sensor_age_relative_to_assembly(sensor_case):
    case = sensor_case
    frame = case.frame(19.8)
    frame.stamp = 20.0
    case.detector.detect(case.pm, frame)
    observation = case.pm.environment_observation
    assert observation.status == "ok"
    assert observation.time == pytest.approx(9.8)
    assert observation.origin == "sensor"
    assert case.pm.regulatory_observation.sensor_stamp == 19.8


def test_first_stale_frame_is_rejected_before_inference(sensor_case):
    case = sensor_case
    frame = case.frame(19.0)
    frame.stamp = 20.0
    case.detector.detector = lambda _: pytest.fail("Stale frame reached inference")
    case.detector.detect(case.pm, frame)
    assert case.pm.environment_observation.status == "stale_or_future_frame"
    assert not case.pm.environment_observation.markers


def test_inference_delay_does_not_refresh_evidence_time(sensor_case):
    case = sensor_case

    def infer(_):
        case.pm.simulation_time += 0.6
        return []

    case.detector.detector = infer
    case.detector.detect(case.pm, case.frame(20.0))
    observation = case.pm.environment_observation
    assert observation.status == "ok"
    assert observation.time == 10.0
    assert not observation.is_fresh(case.pm.simulation_time, 0.3)


def test_explicit_clock_offset_does_not_anchor_stale_first_frame(sensor_case, monkeypatch):
    case = sensor_case
    monkeypatch.setattr(case.settings, "sensor_clock_offset", -10.0)
    frame = case.frame(19.0)
    frame.stamp = None
    case.detector.detect(case.pm, frame)
    assert case.pm.environment_observation.status == "stale_or_future_frame"
    assert case.detector.clock_anchor is None
    case.detector.detect(case.pm, case.frame(20.0))
    assert case.pm.environment_observation.time == 10.0
    assert case.pm.environment_observation.status == "ok"


@pytest.mark.parametrize("stamp", [None, float("nan"), float("inf"), -1.0])
def test_automatic_clock_requires_valid_assembly_stamp(sensor_case, stamp):
    case = sensor_case
    frame = case.frame(20.0)
    frame.stamp = stamp
    case.detector.detect(case.pm, frame)
    assert case.pm.environment_observation.status == "missing_or_invalid_frame_clock"


def test_oldest_input_stamp_is_retained(sensor_case):
    case = sensor_case
    frame = case.frame(20.0)
    frame.camera.stamp = 20.00002
    frame.stamp = 20.00002
    case.detector.detect(case.pm, frame)
    assert case.pm.regulatory_observation.sensor_stamp == 20.0
    assert case.pm.environment_observation.time == pytest.approx(9.99998)


def test_future_capture_is_not_a_valid_clock_anchor(sensor_case):
    case = sensor_case
    frame = case.frame(20.1)
    frame.stamp = 20.0
    case.detector.detect(case.pm, frame)
    assert case.pm.environment_observation.status == "capture_after_frame_assembly"
    assert case.detector.clock_anchor is None


def test_repeated_frame_loses_previous_observations(sensor_case):
    case = sensor_case
    case.detector.detect(case.pm, case.frame(20.0))
    case.pm.simulation_time = 10.1
    case.detector.detect(case.pm, case.frame(20.0))
    assert case.pm.environment_observation.status == "repeated_or_out_of_order_frame"
    assert not case.pm.environment_observation.markers


def test_runtime_reset_reinitializes_clock_mapping(sensor_case):
    case = sensor_case
    case.detector.detect(case.pm, case.frame(20.0))
    case.pm.simulation_time = 0.0
    case.detector.detect(case.pm, case.frame(100.0))
    assert case.pm.environment_observation.status == "ok"
    assert case.pm.environment_observation.time == 0.0
    assert case.detector.clock_anchor == (100.0, 0.0)


def test_unexpected_producer_failure_publishes_unknown(sensor_case):
    case = sensor_case

    def fail(*_):
        raise RuntimeError("provider unavailable")

    case.detector.observe = fail
    with pytest.raises(RuntimeError, match="provider unavailable"):
        case.detector.detect(case.pm, case.frame())
    assert case.pm.environment_observation.origin == "sensor"
    assert case.pm.environment_observation.status == "perception_error"
    assert not case.pm.environment_observation.markers


def test_map_association_is_separate_from_detection_id(api):
    env = api.env
    pm = SimpleNamespace(
        environment_observation=env.EnvironmentObservation(
            1.0,
            (
                env.TrafficSignal(id="tracked-51", map_ref="map-7", state="Green"),
                env.CrossingOccupancy(id="tracked-area-4", map_ref="map-8", occupied=False),
                env.TrafficSignal(id="map-9", state="Green"),
                env.CrossingOccupancy(id="map-10", occupied=False),
            ),
            origin="sensor",
        )
    )
    observed = api.observations.regulatory_observation(pm)
    assert observed.lights == {"map-7": "Green"}
    assert observed.crossings == {"map-8": False}
    assert observed.origin == "sensor"


@pytest.mark.parametrize("order", [("Green", "Red", "Green"), ("Red", "Green", "Red")])
def test_conflicting_heads_cannot_authorize_green(api, order):
    env = api.env
    pm = SimpleNamespace(
        environment_observation=env.EnvironmentObservation(
            1.0,
            tuple(env.TrafficSignal(id=str(i), map_ref="s", state=s) for i, s in enumerate(order)),
        )
    )
    assert api.observations.regulatory_observation(pm).lights == {"s": "Unknown"}


def test_conflicting_crossing_reports_are_unknown(api):
    env = api.env
    pm = SimpleNamespace(
        environment_observation=env.EnvironmentObservation(
            1.0,
            tuple(
                env.CrossingOccupancy(id=str(i), map_ref="c", occupied=value)
                for i, value in enumerate((False, True, False))
            ),
        )
    )
    assert api.observations.regulatory_observation(pm).crossings == {"c": None}


@pytest.mark.parametrize("origin", ["sensor", "simulator_truth", "scenario", "unknown"])
def test_legacy_adapter_preserves_explicit_origin_and_map_reference(api, origin):
    old = api.observations.RegulatoryObservation(
        2.0, {"map-light": "Red"}, source="provider-1", origin=origin
    )
    snapshot = api.observations.environment_observation(old)
    assert snapshot.origin == origin
    assert snapshot.source == "provider-1"
    assert snapshot.markers[0].id != "map-light"
    assert snapshot.markers[0].map_ref == "map-light"
    assert (
        api.observations.regulatory_observation(
            SimpleNamespace(environment_observation=snapshot)
        ).lights
        == old.lights
    )
    assert pickle.loads(pickle.dumps(snapshot)) == snapshot


def test_unlabelled_legacy_source_is_not_assumed_truth_or_sensor(api):
    old = api.observations.RegulatoryObservation(2.0, {})
    snapshot = api.observations.environment_observation(old)
    assert snapshot.origin == "unknown"
    assert snapshot.source == "legacy"


@pytest.mark.parametrize("kind", ["TrafficSignal", "CrossingOccupancy"])
@pytest.mark.parametrize("reference", ["", "  ", 42])
def test_invalid_map_reference_rejected(api, kind, reference):
    with pytest.raises(ValueError, match="Map reference"):
        getattr(api.env, kind)(id="detected", map_ref=reference)


@pytest.mark.parametrize("origin", ["ground_truth", "camera", None, []])
def test_invalid_origin_rejected(api, origin):
    with pytest.raises(ValueError, match="origin"):
        api.env.EnvironmentObservation(1.0, origin=origin)


@pytest.mark.parametrize("origin", ["sensor", "simulator_truth", "scenario", "unknown"])
@pytest.mark.parametrize("legacy", [False, True])
def test_sensor_only_planner_checks_origin(api, monkeypatch, origin, legacy):
    from avlite.c20_planning.c21_planning_model import LocalPlan

    settings = sys.modules[api.demo.plugin.__name__ + ".settings"].PluginSettings
    monkeypatch.setattr(settings, "required_observation_origin", "sensor")
    ex, _ = api.demo.create_stack()
    try:
        planner = api.demo.plugin.HDMapBehaviorPlanner(ex.local_planner.global_plan, ex.pm)
        ex.pm.simulation_time = 1.0
        if legacy:
            ex.pm.regulatory_observation = api.observations.RegulatoryObservation(
                1.0, origin=origin
            )
        else:
            ex.pm.environment_observation = api.env.EnvironmentObservation(1.0, origin=origin)
        plan = planner.plan_behavior(LocalPlan.from_trajectory(planner.global_plan.trajectory))
        if origin == "sensor":
            assert plan.behavior_state != "PERCEPTION_STOP"
        else:
            assert plan.behavior_state == "PERCEPTION_STOP"
            assert plan.behavior_reason == "observation_origin_" + origin
    finally:
        ex.stop()


def test_sensor_only_planner_handles_missing_observation(api, monkeypatch):
    from avlite.c20_planning.c21_planning_model import LocalPlan

    settings = sys.modules[api.demo.plugin.__name__ + ".settings"].PluginSettings
    monkeypatch.setattr(settings, "required_observation_origin", "sensor")
    ex, _ = api.demo.create_stack()
    try:
        planner = api.demo.plugin.HDMapBehaviorPlanner(ex.local_planner.global_plan, ex.pm)
        plan = planner.plan_behavior(LocalPlan.from_trajectory(planner.global_plan.trajectory))
        assert plan.behavior_state == "PERCEPTION_STOP"
    finally:
        ex.stop()


def test_carla_adapter_labels_truth_and_refuses_sensor_overwrite(api):
    module = sys.modules[api.demo.plugin.__name__ + ".execution.carla_task"]
    world = SimpleNamespace(get_actors=lambda: SimpleNamespace(filter=lambda _: []))
    observation = module.observe_carla(world, 3.0)
    assert observation.origin == "simulator_truth"
    assert observation.source == "carla"
    pm = SimpleNamespace(
        regulatory_observation=None,
        environment_observation=api.env.EnvironmentObservation(
            3.0, source="independent-sensor", origin="sensor"
        ),
    )
    ex = SimpleNamespace(pm=pm, elapsed_sim_time=3.0, world=SimpleNamespace(world=world))
    with pytest.raises(RuntimeError, match="cannot overwrite sensor"):
        module.CarlaRegulatoryTask().execute(ex)
    assert pm.environment_observation.origin == "sensor"
