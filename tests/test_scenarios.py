"""Scenario reproducibility, independent failure detection and real AVLite runs."""

import importlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from shapely.geometry import LineString, box

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def api():
    spec = importlib.util.spec_from_file_location(
        "scenario_test_loader", ROOT / "tools/coordinator_demo.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    name = module.plugin.__name__ + ".scenarios."
    return SimpleNamespace(
        schema=importlib.import_module(name + "schema"),
        runner=importlib.import_module(name + "runner"),
        audit=importlib.import_module(name + "evaluation"),
        timeline=importlib.import_module(name + "timeline"),
        backend=importlib.import_module(name + "basicsim"),
        plugin=module.plugin,
    )


def short_spec(api, **changes):
    spec, _ = api.schema.load_scenario(ROOT / "data/scenarios/light-and-crossing.yaml")
    data = spec.model_dump()
    data.update(
        duration=2.0,
        actors=[],
        signals=[dict(id="light", initial="Green")],
        criteria=dict(require_completion=False, max_stopped_seconds=2),
    )
    data.update(changes)
    return api.schema.ScenarioSpec.model_validate(data)


def test_seed_resolution_is_order_independent_and_replayable(api):
    schema = api.schema
    original = {
        "parameters": {"a": {"uniform": [1, 5]}, "b": {"choice": [2, 4]}},
        "values": ["$a", "$b"],
    }
    changed_order = dict(original, parameters=dict(reversed(list(original["parameters"].items()))))
    assert schema.resolve_parameters(original, 19) == schema.resolve_parameters(changed_order, 19)
    assert schema.resolve_parameters(original, 19) != schema.resolve_parameters(original, 20)
    assert original["values"] == ["$a", "$b"]


@pytest.mark.parametrize(
    "parameters",
    [
        {"a": {"uniform": [3, 1]}},
        {"a": {"choice": []}},
        {"a": {"uniform": [0, float("inf")]}},
        {"a": {"eval": "1+1"}},
    ],
)
def test_invalid_distributions_are_refused(api, parameters):
    with pytest.raises(ValueError):
        api.schema.resolve_parameters(dict(parameters=parameters), 0)


@pytest.mark.parametrize(
    "changes",
    [
        dict(typo=True),
        dict(dt=0),
        dict(duration=2.03),
        dict(faults=[dict(start=1, duration=2, kind="dropout")]),
        dict(faults=[dict(start=0, duration=1, kind="delay")]),
        dict(
            faults=[
                dict(start=0, duration=1, kind="dropout"),
                dict(start=0.5, duration=1, kind="dropout"),
            ]
        ),
    ],
)
def test_invalid_case_is_rejected_before_running(api, changes):
    with pytest.raises(ValueError):
        short_spec(api, **changes)


def test_delayed_evidence_does_not_delay_world_truth(api):
    spec = short_spec(
        api,
        actors=[
            dict(id="walker", kind="pedestrian", x=48, y=-6, heading=1.5707963267948966, speed=1)
        ],
        faults=[dict(start=0.5, duration=1, kind="delay", latency=0.4)],
    )
    stamp = api.timeline.observation_time(spec, 1)
    assert stamp == pytest.approx(0.6)
    assert api.timeline.actor_states(spec, 1)[0]["y"] == pytest.approx(-5)
    assert api.timeline.actor_states(spec, stamp)[0]["y"] == pytest.approx(-5.4)
    assert api.timeline.observation_time(spec, 1.5) == 1.5


def frame(time, x, actors=(), speed=3, signal="Red", y=0):
    return dict(
        time=time,
        ego=dict(x=x, y=y, heading=0, speed=speed, length=1, width=1),
        actors=list(actors),
        signals=dict(light=signal),
        behavior="ARRIVED",
    )


def audit(api, spec=None):
    spec = short_spec(api) if spec is None else spec
    return api.audit.ScenarioAudit(
        spec,
        LineString([(0, 0), (20, 0)]),
        {"crossing": box(10, -2, 12, 2)},
        {"light": 5},
        box(-5, -2, 25, 2),
    )


def test_audit_does_not_trust_arrived_flag_and_counts_red_entry_once(api):
    monitor = audit(
        api,
        short_spec(
            api, criteria=dict(require_completion=True), signals=[dict(id="light", initial="Red")]
        ),
    )
    for t, x in [(0, 4), (0.05, 4.7), (0.1, 5.2)]:
        monitor.update(frame(t, x))
    result = monitor.result()
    assert result["metrics"]["red_light_entries"] == 1
    assert not result["metrics"]["completed"]
    assert not result["passed"]


def test_contact_between_tick_endpoints_is_detected(api):
    monitor = audit(api)
    actor = dict(
        id="walker", kind="pedestrian", x=2, y=0, heading=0, speed=0, length=0.2, width=0.2
    )
    monitor.update(frame(0, 0, [actor]))
    monitor.update(frame(0.05, 4, [actor]))
    assert monitor.result()["metrics"]["collisions"] == 1


def test_occupied_crossing_entry_and_offroad_are_independent(api):
    monitor = audit(api)
    actor = dict(
        id="walker", kind="pedestrian", x=11, y=1.5, heading=0, speed=0, length=0.5, width=0.5
    )
    monitor.update(frame(0, 9, [actor]))
    monitor.update(frame(0.05, 10, [actor]))
    monitor.update(frame(0.1, 11, [actor], y=-3))
    metrics = monitor.result()["metrics"]
    assert metrics["occupied_crossing_entries"] == 1
    assert metrics["offroad_seconds"] == pytest.approx(0.05)
    assert metrics["collisions"] == 0


def test_real_run_replay_and_source_guard(api, tmp_path, monkeypatch):
    # Defaults must serialize identically before and after a manifest round-trip.
    spec = short_spec(
        api, actors=[dict(id="parked", kind="vehicle", x=42, y=-5.5, length=5, width=2)]
    )
    before = api.plugin.PluginSettings.model_dump()
    result = api.runner.run_scenario(spec, tmp_path / "original")
    assert result["passed"], result
    assert result["metrics"]["progress_m"] > 1
    assert api.plugin.PluginSettings.model_dump() == before
    repeated = api.runner.replay(tmp_path / "original/manifest.json", tmp_path / "replay")
    assert repeated["passed"] and repeated["replay_trace_matches"]
    with pytest.raises(FileExistsError):
        api.runner.run_scenario(spec, tmp_path / "original")
    identity = api.runner.source_identity()
    monkeypatch.setattr(
        api.runner, "source_identity", lambda: dict(identity, fingerprint="different")
    )
    with pytest.raises(ValueError, match="Source or dependency"):
        api.runner.replay(tmp_path / "original/manifest.json", tmp_path / "changed")
    asset = tmp_path / "original/assets/map.xodr"
    asset.write_text(asset.read_text() + "\n")
    with pytest.raises(ValueError, match="asset changed"):
        api.runner.replay(tmp_path / "original/manifest.json", tmp_path / "tampered")


def test_inference_failure_is_reported_and_settings_restored(api, tmp_path, monkeypatch):
    before = api.plugin.PluginSettings.model_dump()

    def fail(*args, **kwargs):
        raise RuntimeError("injected provider failure")

    monkeypatch.setattr(api.backend.ScriptedScenarioPerception, "perceive", fail)
    result = api.runner.run_scenario(short_spec(api), tmp_path / "error")
    assert result["status"] == "error" and not result["passed"]
    assert "injected provider failure" in result["failures"][0]
    assert api.plugin.PluginSettings.model_dump() == before
    assert json.loads((tmp_path / "error/result.json").read_text())["status"] == "error"


def test_suite_pairs_identical_conditions_across_strategies(api, tmp_path):
    spec = short_spec(api)
    case = tmp_path / "case.yaml"
    case.write_text(json.dumps(spec.model_dump()))
    summary = api.runner.run_suite([case], [1, 2], ["road-rules", "road-tree"], tmp_path / "suite")
    assert summary["passed"], summary
    assert summary["total"] == 4
    assert summary["runs"][0]["case_id"] == summary["runs"][1]["case_id"]
    assert summary["runs"][2]["case_id"] == summary["runs"][3]["case_id"]
    assert summary["runs"][0]["case_id"] != summary["runs"][2]["case_id"]


def test_unknown_backend_does_not_create_output(api, tmp_path):
    with pytest.raises(ValueError, match="Unsupported scenario backend"):
        api.runner.run_scenario(short_spec(api, backend="carla"), tmp_path / "unsupported")
    assert not (tmp_path / "unsupported").exists()


def test_signal_change_inside_tick_is_evaluated_at_entry(api):
    spec = short_spec(
        api, signals=[dict(id="light", initial="Red", changes=[dict(time=0.025, state="Green")])]
    )
    monitor = audit(api, spec)
    monitor.update(frame(0, 4))
    monitor.update(frame(0.05, 4.75, signal="Green"))
    assert monitor.metrics["red_light_entries"] == 0
    monitor = audit(api, spec)
    monitor.update(frame(0, 4.4))
    monitor.update(frame(0.05, 4.75, signal="Green"))
    assert monitor.metrics["red_light_entries"] == 1


def test_empty_suite_cannot_pass(api, tmp_path):
    with pytest.raises(ValueError, match="at least one"):
        api.runner.run_suite([], [0], ["road-rules"], tmp_path / "empty")
