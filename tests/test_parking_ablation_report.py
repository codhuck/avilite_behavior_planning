"""Incomplete or mismatched comparisons cannot silently become positive evidence."""

import importlib
import sys
from copy import deepcopy
from pathlib import Path

import pytest


@pytest.fixture
def api(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).parents[1] / "tools"))
    monkeypatch.delitem(sys.modules, "report_parking_ablation", raising=False)
    return importlib.import_module("report_parking_ablation")


def data():
    scenes = [
        dict(id=p, pair_id="one", ablation_policy=p, seed=1) for p in ("full", "no_observation")
    ]
    rows = [
        dict(
            scenario=s,
            final_state="PARKED" if s["id"] == "full" else "PARKING_BLOCKED",
            position_error=0.1 if s["id"] == "full" else 2,
            heading_error=1,
            collisions=0,
            body_violations=0,
            passed=s["id"] == "full",
            wall_seconds=12,
            fixed_delta_seconds=0.1,
            telemetry=[dict(time=10, state="PARKING_APPROACH", speed=0, reason="test")],
        )
        for s in scenes
    ]
    return dict(suite=scenes), rows


def test_refusal_is_counted_and_has_no_success_time(api):
    p, rows = data()
    r = api.analyze(p, rows)
    assert r["total"] == 2
    assert r["policies"]["no_observation"]["completed"] == 0
    assert r["policies"]["no_observation"]["success_time_median"] is None
    assert r["paired_differences"][0]["completion_difference"] == 1
    assert r["paired_differences"][0]["time_difference_full_minus_alternative"] is None


def test_incomplete_and_duplicate_results_are_rejected(api):
    p, rows = data()
    with pytest.raises(ValueError, match="Incomplete"):
        api.analyze(p, rows[:1])
    with pytest.raises(ValueError, match="Duplicate"):
        api.analyze(p, rows + rows[:1])


def test_different_exogenous_scene_is_not_paired(api):
    p, rows = data()
    p = deepcopy(p)
    p["suite"][1]["seed"] = 2
    rows[1]["scenario"]["seed"] = 2
    with pytest.raises(ValueError, match="exogenous"):
        api.analyze(p, rows)


def test_nonconflicting_pedestrian_does_not_require_forced_stop(api):
    row = dict(
        final_state="PARKED",
        position_error=0.1,
        heading_error=1,
        collisions=0,
        body_violations=0,
        scenario={"pedestrian": True},
        pedestrian_encounter_valid=True,
        pedestrian_crossed=True,
        maximum_pedestrian_overlap=0,
        pedestrian_stop_frames=0,
    )
    assert api.completion(row)
    row["pedestrian_encounter_valid"] = False
    assert not api.completion(row)


def test_clearance_context_distinguishes_stopped_ego_from_moving_ego(api):
    protocol, rows = data()
    rows[0]["telemetry"] = [
        dict(time=1, state="PARKING_APPROACH", speed=0.1, reason="test", pedestrian_clearance=1.2),
        dict(time=2, state="PARKING_BLOCKED", speed=0, reason="test", pedestrian_clearance=0.2),
    ]
    result = api.analyze(protocol, rows)["trials"][0]
    assert result["minimum_clearance_context"]["speed"] == 0
    assert result["minimum_clearance_context"]["pedestrian_clearance"] == 0.2
    assert result["minimum_pedestrian_clearance_while_ego_moving"] == 1.2
