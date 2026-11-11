"""One-command runtime setup; optional creation of a new demo profile."""

import argparse
import json
import subprocess
import sys
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--check", action="store_true", help="Verify only: no pip install or downloads")
    p.add_argument(
        "--autoware", action="store_true", help="Also install the experimental Autoware model"
    )
    p.add_argument(
        "--profile-output", type=Path, help="Write a new portable Town10 profile; never overwrites"
    )
    p.add_argument("--map-path", type=Path, help="Matching Town10HD_Opt OpenDRIVE file")
    p.add_argument("--bridge-path", type=Path, help="Installed CARLA bridge plugin directory")
    args = p.parse_args()
    if args.profile_output and (not args.map_path or not args.bridge_path):
        p.error("--profile-output requires --map-path and --bridge-path")
    if args.check and args.profile_output:
        p.error("--check does not write profiles")
    if args.profile_output and args.profile_output.expanduser().exists():
        p.error("Profile already exists; choose a new filename")
    root = Path(__file__).resolve().parent
    if not args.check:
        subprocess.run(
            [sys.executable, "-m", "pip", "install", "-r", str(root / "requirements.txt")],
            cwd=root,
            check=True,
        )
    from avlite_signal_perception.assets import digest, resolve_model
    from avlite_signal_perception.provider import create_classifier

    result = {}
    for name in ["trained_rgb", "yolo"] + (["autoware"] if args.autoware else []):
        path = resolve_model(name, download=not args.check)
        result[name] = {"path": str(path), "sha256": digest(path)}
    create_classifier("trained_rgb")
    if args.profile_output:
        from profile_setup import write_profile

        result["profile"] = str(write_profile(args.profile_output, args.map_path, args.bridge_path))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
