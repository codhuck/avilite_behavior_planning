"""Check mission files and configuration without launching AVLite or a simulator."""

if not __name__.startswith("avlite.plugins."):
    import argparse
    import importlib
    import json
    from pathlib import Path

    from _bootstrap import load_plugin

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument(
            "--profile", required=True, type=Path, help="Complete YAML profile file"
        )
        parser.add_argument("--json", action="store_true", help="Print a machine-readable report")
        args = parser.parse_args()
        plugin = load_plugin()
        diagnostics = importlib.import_module(plugin.__name__ + ".execution.mission_diagnostics")
        result = diagnostics.diagnose_mission(args.profile)
        if args.json:
            print(json.dumps(result, indent=2, allow_nan=False))
        else:
            print(f"Mission preflight: {'PASS' if result['passed'] else 'FAIL'}")
            for key, path in result["files"].items():
                print(f"  {key}: {path}")
            for label in ("errors", "warnings"):
                for item in result[label]:
                    print(f"  {label[:-1].upper()} [{item['field']}] {item['message']}")
            print(
                "Static checks only; live occupancy, bridge support and path feasibility remain runtime checks."
            )
        raise SystemExit(0 if result["passed"] else 1)

    if __name__ == "__main__":
        main()
