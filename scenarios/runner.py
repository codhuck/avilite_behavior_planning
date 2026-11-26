"""Artifact-producing scenario runner and replay, independent of a simulator SDK."""

import hashlib
import importlib.metadata
import json
import platform
import shlex
import shutil
import subprocess
from pathlib import Path

from .schema import ScenarioSpec, load_scenario

ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_identity():
    import avlite

    core = Path(avlite.__file__).resolve().parent
    sources = {}
    for label, root, files in (
        (
            "plugin",
            ROOT,
            [
                *ROOT.glob("*.py"),
                *(
                    p
                    for d in (
                        "scenarios",
                        "execution",
                        "perception",
                        "planning",
                        "parking",
                        "tools",
                    )
                    for p in (ROOT / d).rglob("*.py")
                ),
            ],
        ),
        ("core", core, core.rglob("*.py")),
    ):
        for path in sorted(files):
            if "__pycache__" not in path.parts:
                sources[f"{label}/{path.relative_to(root)}"] = sha256(path)
    versions = {
        name: importlib.metadata.version(name)
        for name in ("numpy", "pydantic", "shapely", "scipy", "networkx", "PyYAML")
    }
    versions["python"] = platform.python_version()
    fingerprint = hashlib.sha256(
        json.dumps(dict(sources=sources, versions=versions), sort_keys=True).encode()
    ).hexdigest()
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, capture_output=True)
    return dict(
        fingerprint=fingerprint,
        files=sources,
        versions=versions,
        plugin_commit=commit.stdout.strip() if commit.returncode == 0 else None,
    )


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def create_backend(spec):
    if spec.backend == "basicsim":
        from .basicsim import BasicSimScenario

        return BasicSimScenario(spec)
    raise ValueError(f"Unsupported scenario backend: {spec.backend}")


def run_scenario(spec, output, sampled=None, replay_of=None, identity=None):
    """Resolve assets once, then run serially; an existing output is never overwritten."""
    backend = create_backend(spec)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / "assets").mkdir()
    stored = spec.model_dump()
    assets = {}
    for key in ("map", "route"):
        src = Path(getattr(spec, key))
        relative = "assets/" + key + src.suffix
        shutil.copyfile(src, output / relative)
        stored[key] = relative
        assets[relative] = sha256(output / relative)
    case = {k: v for k, v in stored.items() if k != "strategy"}
    case_id = hashlib.sha256(
        json.dumps(dict(scenario=case, assets=assets), sort_keys=True).encode()
    ).hexdigest()
    identity = source_identity() if identity is None else identity
    scenario_hash = hashlib.sha256(json.dumps(stored, sort_keys=True).encode()).hexdigest()
    manifest = dict(
        scenario_hash=scenario_hash,
        version=1,
        scenario=stored,
        parameters=sampled or {},
        assets=assets,
        case_id=case_id,
        source=identity,
        replay_of=replay_of,
    )
    write_json(output / "manifest.json", manifest)
    replay_command = shlex.join(
        [
            "python",
            str(ROOT / "tools/run_scenarios.py"),
            "replay",
            str(output / "manifest.json"),
            "--output",
            str(output.with_name(output.name + "-replay")),
        ]
    )
    try:
        with (output / "trace.jsonl").open("w") as stream:

            def emit(frame):
                stream.write(json.dumps(frame, sort_keys=True, allow_nan=False) + "\n")

            result = backend.run(emit)
        result["status"] = "passed" if result["passed"] else "failed"
    except Exception as error:
        result = dict(
            passed=False, status="error", failures=[f"{type(error).__name__}: {error}"], metrics={}
        )
    result.update(
        case_id=case_id,
        scenario=spec.name,
        seed=spec.seed,
        strategy=spec.strategy,
        replay_command=replay_command,
        source_fingerprint=identity["fingerprint"],
    )
    if (output / "trace.jsonl").exists():
        result["trace_sha256"] = sha256(output / "trace.jsonl")
    write_json(output / "result.json", result)
    lines = [
        f"# {spec.name}",
        "",
        f"Result: **{result['status']}**",
        "",
        f"Strategy: `{spec.strategy}`. Seed: `{spec.seed}`.",
        "",
        "Scripted BasicSim observations; this run does not measure sensor recognition accuracy.",
        "",
        "| Metric | Value |",
        "| --- | --- |",
    ]
    lines += [f"| {name} | {value} |" for name, value in result["metrics"].items()]
    lines += [
        "",
        *["- " + failure for failure in result["failures"]],
        "",
        "Replay:",
        "",
        "```bash",
        replay_command,
        "```",
        "",
    ]
    (output / "report.md").write_text("\n".join(lines))
    return result


def replay(manifest_path, output, allow_source_change=False):
    path = Path(manifest_path).resolve()
    manifest = json.loads(path.read_text())
    if manifest.get("version") != 1:
        raise ValueError("Unsupported replay manifest version")
    scenario = dict(manifest["scenario"])
    if (
        hashlib.sha256(json.dumps(scenario, sort_keys=True).encode()).hexdigest()
        != manifest["scenario_hash"]
    ):
        raise ValueError("Resolved scenario changed after the run")
    case = {key: value for key, value in scenario.items() if key != "strategy"}
    case_id = hashlib.sha256(
        json.dumps(dict(scenario=case, assets=manifest["assets"]), sort_keys=True).encode()
    ).hexdigest()
    if case_id != manifest["case_id"]:
        raise ValueError("Resolved scenario differs from the recorded case ID")
    for key in ("map", "route"):
        relative = scenario[key]
        asset = (path.parent / relative).resolve()
        if not asset.is_relative_to(path.parent) or not asset.is_file():
            raise ValueError("Replay asset is absent or outside the saved run")
        if sha256(asset) != manifest["assets"].get(relative):
            raise ValueError(f"Replay asset changed: {relative}")
        scenario[key] = str(asset)
    identity = source_identity()
    changed = identity["fingerprint"] != manifest["source"]["fingerprint"]
    if changed and not allow_source_change:
        raise ValueError(
            "Source or dependency versions changed; use --allow-source-change for an explicit comparison"
        )
    spec = ScenarioSpec.model_validate(scenario)
    result = run_scenario(
        spec,
        output,
        manifest["parameters"],
        replay_of=dict(manifest=str(path), source_changed=changed),
        identity=identity,
    )
    original = path.parent / "result.json"
    if original.is_file():
        expected = json.loads(original.read_text()).get("trace_sha256")
        result["replay_trace_matches"] = expected is not None and expected == result.get(
            "trace_sha256"
        )
        if not changed and not result["replay_trace_matches"]:
            result["passed"], result["status"] = False, "failed"
            result["failures"].append(
                "Replay trace differs under the recorded source/dependency fingerprint"
            )
        write_json(Path(output) / "result.json", result)
        with (Path(output) / "report.md").open("a") as stream:
            stream.write(
                f"\nReplay trace matches original: {result['replay_trace_matches']}. Final status: {result['status']}.\n"
            )
    return result


def run_suite(paths, seeds, strategies, output):
    if not paths or not seeds or not strategies:
        raise ValueError("A suite needs at least one scenario, seed and strategy")
    # Validate all parameter realizations before creating output or running any stack.
    cases = [
        (path, *load_scenario(path, seed, strategy))
        for path in paths
        for seed in seeds
        for strategy in strategies
    ]
    for _, spec, _ in cases:
        create_backend(spec)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    identity, results = source_identity(), []
    for index, (_, spec, sampled) in enumerate(cases):
        directory = output / f"{index:03d}-{spec.name}-{spec.strategy}-seed{spec.seed}"
        results.append(
            dict(
                directory=directory.name,
                **run_scenario(spec, directory, sampled, identity=identity),
            )
        )
    summary = dict(
        passed=all(r["passed"] for r in results),
        runs=results,
        passed_count=sum(r["passed"] for r in results),
        total=len(results),
    )
    write_json(output / "summary.json", summary)
    lines = [
        "# Scenario comparison",
        "",
        "Cases share resolved inputs when `case_id` matches; actor motion does not react to ego.",
        "",
        "| Scenario | Seed | Strategy | Result | Stopped seconds |",
        "| --- | --- | --- | --- | --- |",
    ]
    lines += [
        f"| {r['scenario']} | {r['seed']} | {r['strategy']} | [{r['status']}]({r['directory']}/report.md) | {r['metrics'].get('stopped_seconds', 'n/a')} |"
        for r in results
    ]
    (output / "report.md").write_text("\n".join(lines) + "\n")
    return summary
