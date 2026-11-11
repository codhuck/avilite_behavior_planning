"""Install model artifacts explicitly; importing this package never downloads."""

import argparse
import json

from .assets import digest, resolve_model


def main():
    p = argparse.ArgumentParser()
    p.add_argument("command", choices=["setup", "check"])
    p.add_argument(
        "--models",
        nargs="+",
        choices=["trained_rgb", "autoware", "yolo"],
        default=["trained_rgb", "yolo"],
    )
    args = p.parse_args()
    result = {}
    for name in args.models:
        path = resolve_model(name, download=args.command == "setup")
        result[name] = {"path": str(path), "sha256": digest(path)}
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
