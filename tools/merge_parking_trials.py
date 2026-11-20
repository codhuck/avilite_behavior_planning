"""Assemble completed, compatible parking trials without choosing among retries."""

# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import hashlib
    import json
    import shutil
    import tempfile
    from pathlib import Path

    def assemble(protocol_path, recordings, output):
        reference_bytes = protocol_path.read_bytes()
        reference = json.loads(reference_bytes)
        cases = {case["id"]: case for case in reference["suite"]}
        if len(cases) != len(reference["suite"]):
            raise ValueError("Duplicate scenario IDs in reference protocol")
        repeats = reference["args"]["repeats"]
        expected = {(case, repeat) for case in cases for repeat in range(repeats)}
        trials, owners, provenance = {}, {}, []
        ignored_options = {"suite", "case", "output"}
        settings = {k: v for k, v in reference["args"].items() if k not in ignored_options}
        for recording in recordings:
            protocol_bytes = (recording / "protocol.json").read_bytes()
            source = json.loads(protocol_bytes)
            if source["source_sha256"] != reference["source_sha256"]:
                raise ValueError(f"Runtime source mismatch: {recording}")
            options = {k: v for k, v in source["args"].items() if k not in ignored_options}
            if options != settings:
                raise ValueError(f"Execution option mismatch: {recording}")
            if any(case != cases.get(case["id"]) for case in source["suite"]):
                raise ValueError(f"Scenario definition mismatch: {recording}")
            result_bytes = (recording / "results.json").read_bytes()
            declared_cases = {case["id"] for case in source["suite"]}
            for result in json.loads(result_bytes):
                case = result["scenario"]["id"]
                key = (case, result["repeat"])
                if (
                    case not in declared_cases
                    or result["scenario"] != cases.get(case)
                    or key not in expected
                ):
                    raise ValueError(f"Unexpected trial: {key}")
                if key in trials:
                    raise ValueError(f"Duplicate completed trial: {key}")
                if not result.get("telemetry") or not isinstance(result.get("passed"), bool):
                    raise ValueError(f"Incomplete result: {key}")
                if not case.replace("_", "").isalnum():
                    raise ValueError(f"Invalid case ID: {case}")
                if case in owners and owners[case] != recording:
                    raise ValueError(f"Case artifacts split across recordings: {case}")
                if not (recording / case).is_dir():
                    raise ValueError(f"Missing case artifacts: {recording / case}")
                owners[case] = recording
                trials[key] = result
            provenance.append(
                dict(
                    recording=str(recording.resolve()),
                    protocol_sha256=hashlib.sha256(protocol_bytes).hexdigest(),
                    results_sha256=hashlib.sha256(result_bytes).hexdigest(),
                )
            )
        missing = expected - trials.keys()
        if missing:
            raise ValueError(f"Missing completed trials: {sorted(missing)}")
        if output.exists():
            raise FileExistsError(f"Refusing to overwrite {output}")
        output.parent.mkdir(parents=True, exist_ok=True)
        staging = Path(tempfile.mkdtemp(prefix=output.name + "-", dir=output.parent))
        try:
            for case, owner in owners.items():
                shutil.copytree(owner / case, staging / case)
            ordered = [trials[(case, repeat)] for case in cases for repeat in range(repeats)]
            (staging / "protocol.json").write_bytes(reference_bytes)
            (staging / "results.json").write_text(json.dumps(ordered, indent=2) + "\n")
            (staging / "provenance.json").write_text(
                json.dumps(
                    dict(
                        operation="Assembly of disjoint completed trials; not an uninterrupted run",
                        reference_protocol=str(protocol_path.resolve()),
                        reference_sha256=hashlib.sha256(reference_bytes).hexdigest(),
                        sources=provenance,
                        trials=len(ordered),
                        passed=sum(r["passed"] for r in ordered),
                        policy="No duplicates, missing cases, changed runtime or changed conditions accepted",
                    ),
                    indent=2,
                )
                + "\n"
            )
            staging.rename(output)
        finally:
            if staging.exists():
                shutil.rmtree(staging)
        return ordered

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--protocol", type=Path, required=True)
        parser.add_argument("--output", type=Path, required=True)
        parser.add_argument("recordings", type=Path, nargs="+")
        args = parser.parse_args()
        rows = assemble(args.protocol, args.recordings, args.output)
        print(json.dumps(dict(completed=len(rows), passed=sum(r["passed"] for r in rows))))

    if __name__ == "__main__":
        main()
