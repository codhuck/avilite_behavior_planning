"""Paired experiments must alter only the declared component and fixed encounter."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


@pytest.fixture
def api(monkeypatch):
    path = Path(__file__).parents[1] / "tools" / "parking_ablation.py"
    spec = importlib.util.spec_from_file_location("parking_ablation_test", path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


def test_each_baseline_changes_only_declared_component(api):
    expected = {
        "roof_only": {"roof_only"},
        "uniform_one": {"hazard_support_radius"},
        "uniform_two": {"minimum_support", "hazard_support_radius"},
        "no_speed_search": {"adaptive_speed"},
        "no_observation": {"allow_observation"},
        "full_segment": {"allow_prefix"},
    }
    base = api.FULL.metadata()
    for name, changed in expected.items():
        p = api.POLICIES[name]
        assert {k for k, v in p.metadata().items() if v != base[k]} == changed
        planner = SimpleNamespace(arbiter=SimpleNamespace())
        p.configure(planner)
        assert planner.allow_observation == p.allow_observation
        assert planner.arbiter.adaptive_speed == p.adaptive_speed
        assert planner.arbiter.allow_prefix == p.allow_prefix
    assert api.FULL.metadata() == base


@pytest.mark.parametrize("direction", [[0, 0], [1, 1], [float("nan"), 0], [1]])
def test_invalid_fixed_encounter_is_rejected(api, direction):
    with pytest.raises(ValueError):
        api.fixed_crossing({"pedestrian_origin": [40, 1], "pedestrian_direction": direction})


def test_encounter_does_not_depend_on_policy_or_ego(api):
    scene = {"pedestrian_origin": [40, 1], "pedestrian_direction": [0, -1]}
    for name in api.POLICIES:
        origin, direction = api.fixed_crossing(dict(scene, ablation_policy=name))
        assert np.array_equal(origin, [40, 1])
        assert np.array_equal(direction, [0, -1])
