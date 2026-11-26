"""Run, compare or replay declarative scenarios through AVLite."""

# ruff: noqa: E402 -- tools must not run while AVLite discovers plugin modules
if not __name__.startswith("avlite.plugins."):
    import argparse
    import importlib
    import json
    import logging
    import sys
    from pathlib import Path

    ROOT = Path(__file__).resolve().parents[1]
    if (ROOT.parent / "avlite/__init__.py").is_file():
        sys.path.insert(0, str(ROOT.parent))
    from _bootstrap import load_plugin

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        commands = parser.add_subparsers(dest="command", required=True)
        run = commands.add_parser("run", help="Run one resolved scenario")
        run.add_argument("scenario", type=Path)
        run.add_argument("--seed", type=int)
        run.add_argument("--strategy", choices=["road-rules", "road-tree"])
        repeat = commands.add_parser("replay", help="Replay saved inputs and check trace identity")
        repeat.add_argument("manifest", type=Path)
        repeat.add_argument("--allow-source-change", action="store_true")
        suite = commands.add_parser("suite", help="Compare the same parameter realizations")
        suite.add_argument("scenarios", nargs="+", type=Path)
        suite.add_argument("--seeds", type=int, nargs="+", default=[0])
        suite.add_argument(
            "--strategies",
            nargs="+",
            choices=["road-rules", "road-tree"],
            default=["road-rules", "road-tree"],
        )
        for command in (run, repeat, suite):
            command.add_argument("--output", type=Path, required=True)
        args = parser.parse_args()
        logging.basicConfig(level=logging.ERROR)
        plugin = load_plugin()
        module = importlib.import_module(plugin.__name__ + ".scenarios.runner")
        try:
            if args.command == "run":
                spec, sampled = module.load_scenario(args.scenario, args.seed, args.strategy)
                result = module.run_scenario(spec, args.output, sampled)
            elif args.command == "replay":
                result = module.replay(args.manifest, args.output, args.allow_source_change)
            else:
                if len(args.scenarios) * len(args.seeds) * len(args.strategies) > 128:
                    parser.error("Limit each suite to 128 runs")
                result = module.run_suite(args.scenarios, args.seeds, args.strategies, args.output)
        except (ValueError, OSError, KeyError) as error:
            parser.exit(2, f"Scenario input error: {error}\n")
        print(
            json.dumps(
                dict(
                    passed=result["passed"],
                    output=str(args.output),
                    failures=result.get("failures", []),
                    total=result.get("total", 1),
                ),
                indent=2,
            )
        )
        raise SystemExit(0 if result["passed"] else 1)

    if __name__ == "__main__":
        main()
