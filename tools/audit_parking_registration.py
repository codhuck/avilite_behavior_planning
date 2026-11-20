"""Verify registration, execution protocol, frozen copies, and current runtime hashes."""

# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import hashlib
    import json
    from pathlib import Path

    def audit(recording, workspace):
        registration = json.loads((recording / "registration.json").read_text())
        protocol = json.loads((recording / "protocol.json").read_text())
        expected = registration["source_sha256"]
        executed = {
            str((Path("avlite-behavior-hdmap") / name)): digest
            for name, digest in protocol["source_sha256"].items()
        }
        # Normalize relative parent components without depending on the caller's cwd.
        executed = {
            str((workspace / name).resolve().relative_to(workspace.resolve())): digest
            for name, digest in executed.items()
        }
        mismatches = []
        for name, digest in expected.items():
            for label, path in (
                ("snapshot", recording / "source_snapshot" / name),
                ("current_runtime", workspace / name),
            ):
                if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                    mismatches.append(dict(file=name, location=label))
        return dict(
            scenario_order_matches=registration["suite"] == protocol["suite"],
            registered_sources_match_execution=expected == executed,
            registered_source_count=len(expected),
            source_mismatches=mismatches,
        )

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("recording", type=Path)
        parser.add_argument("--output", type=Path, required=True)
        args = parser.parse_args()
        result = audit(args.recording, Path(__file__).resolve().parents[2])
        result["passed"] = (
            result["scenario_order_matches"]
            and result["registered_sources_match_execution"]
            and not result["source_mismatches"]
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result))
        return 0 if result["passed"] else 1

    if __name__ == "__main__":
        raise SystemExit(main())
