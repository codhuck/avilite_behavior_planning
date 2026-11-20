"""Resumed trials must retain failures and cannot silently select successful retries."""

import copy
import importlib.util
import json
from pathlib import Path

import pytest


@pytest.fixture
def merge():
    path = Path(__file__).parents[1] / "tools" / "merge_parking_trials.py"
    spec = importlib.util.spec_from_file_location("parking_trial_merge", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.assemble


@pytest.fixture
def recordings(tmp_path):
    protocol = dict(
        suite=[dict(id="first", seed=1), dict(id="second", seed=2)],
        args=dict(repeats=1, dt=0.1, output="unused", suite="full", case=None),
        source_sha256={"parking/planner.py": "fixed-source"},
    )
    reference = tmp_path / "reference.json"
    reference.write_text(json.dumps(protocol))
    sources = []
    for i, case in enumerate(protocol["suite"]):
        folder = tmp_path / f"part{i}"
        (folder / case["id"]).mkdir(parents=True)
        (folder / case["id"] / "cloud.txt").write_text(f"original points {i}")
        partial = copy.deepcopy(protocol)
        partial["suite"] = [case]
        partial["args"]["output"] = str(folder)
        partial["args"]["suite"] = f"part{i}"
        (folder / "protocol.json").write_text(json.dumps(partial))
        result = dict(scenario=case, repeat=0, passed=i == 0, telemetry=[dict(time=1)])
        (folder / "results.json").write_text(json.dumps([result]))
        sources.append(folder)
    return reference, sources, tmp_path / "assembled"


def test_merge_keeps_failed_trial_and_artifacts(merge, recordings):
    reference, sources, output = recordings
    rows = merge(reference, sources, output)
    assert [row["passed"] for row in rows] == [True, False]
    assert (output / "second" / "cloud.txt").read_text() == "original points 1"
    assert (output / "protocol.json").read_bytes() == reference.read_bytes()
    assert json.loads((output / "provenance.json").read_text())["passed"] == 1
    with pytest.raises(FileExistsError):
        merge(reference, sources, output)


@pytest.mark.parametrize("change", ["runtime", "options", "scenario", "missing", "duplicate"])
def test_merge_rejects_incompatible_or_selected_trials(merge, recordings, change):
    reference, sources, output = recordings
    p = sources[1] / "protocol.json"
    protocol = json.loads(p.read_text())
    if change == "runtime":
        protocol["source_sha256"]["parking/planner.py"] = "changed-source"
    elif change == "options":
        protocol["args"]["dt"] = 0.05
    elif change == "scenario":
        protocol["suite"][0]["seed"] = 99
    elif change == "missing":
        sources = sources[:1]
    else:
        sources = sources + sources[:1]
    p.write_text(json.dumps(protocol))
    with pytest.raises(ValueError):
        merge(reference, sources, output)
    assert not output.exists()
