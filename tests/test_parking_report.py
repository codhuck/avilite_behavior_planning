"""Report audits must distinguish a continuous crossing from actor disappearance."""

import importlib.util
from pathlib import Path

import pytest


@pytest.fixture
def summarize():
    path = Path(__file__).parents[1] / "tools" / "report_sensor_parking.py"
    spec = importlib.util.spec_from_file_location("parking_report_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.summarize


def test_disappearing_actor_invalidates_encounter(summarize):
    base = dict(
        state="PARKING_BLOCKED",
        reason="no_admissible_maneuver",
        speed=0,
        certificate=None,
        pedestrian_crossed=False,
        pedestrian_paused=False,
        pedestrian_clearance=2,
    )
    result = dict(
        passed=False,
        final_state="PARKING_BLOCKED",
        fixed_delta_seconds=0.1,
        scenario=dict(pedestrian=True, pedestrian_speed=1),
        telemetry=[
            dict(base, time=21.9, pedestrian_position=[58.5, 3.9]),
            dict(base, time=22, pedestrian_position=[0, 0]),
        ],
    )
    report = summarize(result)
    assert report["pedestrian_position_discontinuities"] == [22]
    assert not report["pedestrian_encounter_valid"]
    result["telemetry"][1]["pedestrian_position"] = [58.5, 4]
    assert summarize(result)["pedestrian_encounter_valid"]
