"""Check optimized occupancy against a frozen implementation on actual LiDAR."""

# ruff: noqa: E402
# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import hashlib
    import importlib.util
    import json
    import sys
    import time
    from pathlib import Path

    PLUGIN = Path(__file__).resolve().parents[1]
    sys.path[:0] = [str(PLUGIN), str(PLUGIN.parent)]
    import numpy as np
    from avlite.c10_perception.c11_perception_model import EgoState
    from avlite.c50_common.c52_world_sensor_datatypes import Lidar
    from shapely.geometry import box

    def load(path, name):
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        return module

    def main():
        p = argparse.ArgumentParser(description=__doc__)
        p.add_argument("recording", type=Path)
        p.add_argument("--before", type=Path, required=True)
        p.add_argument("--output", type=Path, required=True)
        p.add_argument(
            "--dynamic", action="store_true", help="Compare saved event frames from executed drives"
        )
        a = p.parse_args()
        if a.output.exists():
            raise ValueError("Use a new output")
        paths = [a.before, PLUGIN / "parking_occupancy.py"]
        modules = [load(path, f"occupancy_comparison_{i}") for i, path in enumerate(paths)]
        rows = []
        inputs = []
        for result in json.loads((a.recording / "results.json").read_text()):
            if a.dynamic:
                root = a.recording / result["scenario"]["id"]
                inputs.extend(
                    (
                        str(file.relative_to(a.recording)),
                        file.parent,
                        file.name[: -len("_cloud_0.npz")],
                        "ego_pose",
                    )
                    for file in sorted(root.glob("*_event_*_cloud_0.npz"))
                )
            else:
                root = a.recording / result["scene"]["id"]
                inputs.append((result["scene"]["id"], root, "initial", "pose"))
        for n, (label, root, prefix, pose_key) in enumerate(inputs):
            clouds = []
            for i in range(9):
                with np.load(root / f"{prefix}_cloud_{i}.npz") as raw:
                    pose = raw[pose_key]
                    clouds.append(
                        Lidar(
                            points=raw["points"],
                            stamp=float(raw["stamp"]),
                            base_to_sensor=raw["base_to_sensor"],
                            sensor_name=f"lidar_{i}",
                        )
                    )
            ego = EgoState(
                x=pose[0],
                y=pose[1],
                z=pose[2],
                theta=pose[3],
                length=3.705369472503662,
                width=1.788678526878357,
            )
            providers = [
                m.FusedLidarParkingOccupancy(minimum_support=1, hazard_support_radius=2)
                for m in modules
            ]
            observed, timing = {}, {0: [], 1: []}
            for repeat in range(3):
                for i in [0, 1] if (repeat + n) % 2 else [1, 0]:
                    start = time.perf_counter()
                    observed[i] = providers[i].observe(
                        clouds, ego, box(30, -6, 70, 18), clouds[0].stamp, 0
                    )
                    timing[i].append(time.perf_counter() - start)
            old, new = observed[0], observed[1]
            assert (
                old.status == new.status and old.sources == new.sources and old.stamp == new.stamp
            )
            assert old.free.equals(new.free) and old.occupied.equals(new.occupied)
            assert old.blocked.equals(new.blocked) and old.traversable.equals(new.traversable)
            assert all(
                getattr(old, key).wkb == getattr(new, key).wkb
                for key in ("free", "occupied", "blocked", "traversable")
            ), "Geometry serialization changed"
            assert providers[0].support_histogram == providers[1].support_histogram
            rows.append(
                {
                    "scene": label,
                    "geometry_equal": True,
                    "wkb_equal": True,
                    "before_median_seconds": float(np.median(timing[0])),
                    "after_median_seconds": float(np.median(timing[1])),
                }
            )
        report = {
            "scope": __doc__,
            "source_hashes": {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
            "rows": rows,
            "mean_before_seconds": float(np.mean([r["before_median_seconds"] for r in rows])),
            "mean_after_seconds": float(np.mean([r["after_median_seconds"] for r in rows])),
            "all_geometry_equal": True,
        }
        a.output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps({k: v for k, v in report.items() if k != "rows"}, indent=2))

    if __name__ == "__main__":
        main()
