"""Matched-frame support ablation on saved LiDAR; no closed-loop success claims."""

# ruff: noqa: E402
# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import hashlib
    import json
    import sys
    from dataclasses import asdict
    from pathlib import Path

    import numpy as np
    from shapely import union_all
    from shapely.affinity import affine_transform
    from shapely.geometry import GeometryCollection, box, shape

    PLUGIN = Path(__file__).resolve().parents[1]
    sys.path[:0] = [str(PLUGIN), str(PLUGIN.parent)]
    from avlite.c10_perception.c11_perception_model import EgoState
    from avlite.c50_common.c52_world_sensor_datatypes import Lidar

    from parking_occupancy import FusedLidarParkingOccupancy
    from visibility_dispatch import Maneuver, VisibilityDispatcher, VisibilitySnapshot

    def load_frame(directory, mode, step, dimensions):
        """Never combine initial and event clouds from different acquisitions."""
        geometry = directory / f"{mode}_0_geometry_{step}.json"
        data = json.loads(geometry.read_text())
        prefix = f"{mode}_0_" + (f"event_{step}_" if step else "")
        files = sorted(directory.glob(prefix + "cloud_*.npz"))
        if len(files) != 9:
            raise ValueError(f"Expected nine clouds for {geometry}, got {len(files)}")
        clouds = []
        for i, file in enumerate(files):
            with np.load(file) as raw:
                if "pose" in raw and not np.allclose(raw["pose"], data["pose"], atol=1e-5):
                    raise ValueError("Cloud pose does not match evaluator frame")
                clouds.append(
                    Lidar(
                        points=raw["points"],
                        stamp=float(raw["stamp"]),
                        base_to_sensor=raw["base_to_sensor"],
                        sensor_name=f"lidar_{i}",
                    )
                )
        if not np.allclose([c.stamp for c in clouds], clouds[0].stamp, atol=1e-6, rtol=0):
            raise ValueError("Mixed acquisition timestamps")
        x, y, z, yaw = data["pose"]
        ego = EgoState(
            x=x, y=y, z=z, theta=yaw, length=dimensions["length"], width=dimensions["width"]
        )
        hashes = {str(f): hashlib.sha256(f.read_bytes()).hexdigest() for f in [geometry, *files]}
        return ego, clouds, data, hashes

    def region(scene):
        angle = np.deg2rad(scene.get("rotation_degrees", 0))
        c, s = np.cos(angle), np.sin(angle)
        dx, dy = scene.get("translation", [0, 0])
        return affine_transform(
            box(30, -6, 70, 18), [c, -s, s, c, 52 - 52 * c + 3 * s + dx, 3 - 52 * s - 3 * c + dy]
        )

    def evaluate(ego, clouds, area, data):
        configs = {
            "adaptive": (clouds, 1, 2),
            "uniform_one": (clouds, 1, 0),
            "uniform_two": (clouds, 2, 0),
            "roof_only": (clouds[:1], 1, 2),
            "roof_uniform_one": (clouds[:1], 1, 0),
        }
        labels = union_all([shape(o) for o in data["obstacles"]])
        now = clouds[0].stamp
        rows, free = [], {}
        for name, (sources, support, radius) in configs.items():
            provider = FusedLidarParkingOccupancy(
                minimum_support=support, hazard_support_radius=radius
            )
            observation = provider.observe(sources, ego, area, now, 0)
            free[name] = observation.free
            snapshot = VisibilitySnapshot(
                observation.traversable,
                observation.occupied if observation.occupied is not None else GeometryCollection(),
                now,
                observation.sources,
                support,
                radius,
            )
            decisions = []
            for speed in (0.05, 0.1, 0.2, 0.4):
                candidates = []
                for direction in (1, -1):
                    distance = np.linspace(0, 3, 76) * direction
                    poses = np.column_stack(
                        (
                            ego.x + distance * np.cos(ego.theta),
                            ego.y + distance * np.sin(ego.theta),
                            np.full(len(distance), ego.theta),
                        )
                    )
                    candidates.append(
                        Maneuver(f"probe_{direction}", "park", "probe", poses, direction, speed)
                    )
                d = VisibilityDispatcher(allow_prefix=True).select(
                    candidates,
                    snapshot,
                    now,
                    [ego.x, ego.y, ego.theta],
                    0,
                    ego.length,
                    ego.width,
                    {"probe"},
                )
                decisions.append(
                    dict(
                        speed=speed,
                        state=d.state,
                        certificate=asdict(d.certificate) if d.certificate else None,
                    )
                )
            rows.append(
                dict(
                    policy=name,
                    free_area=observation.free.area,
                    false_free_overlap=observation.free.intersection(labels).area,
                    bay_coverage=observation.free.intersection(shape(data["bay"])).area
                    / shape(data["bay"]).area,
                    decisions=decisions,
                )
            )
        # Holding data and hazard construction fixed, stronger support only removes cells.
        nested = (
            free["uniform_two"].difference(free["adaptive"]).area < 1e-8
            and free["adaptive"].difference(free["uniform_one"]).area < 1e-8
        )
        return rows, bool(nested)

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("recording", type=Path)
        parser.add_argument("--output", type=Path, required=True)
        args = parser.parse_args()
        results = json.loads((args.recording / "results.json").read_text())
        frames = []
        hashes = {}
        for result in results:
            scene = result["scenario"]
            for step in (0, 200):
                ego, clouds, data, source = load_frame(
                    args.recording / scene["id"], result["type"], step, result["vehicle_geometry"]
                )
                rows, nested = evaluate(ego, clouds, region(scene), data)
                frames.append(
                    dict(case=scene["id"], step=step, rows=rows, nested_support_sets=nested)
                )
                hashes.update(source)
        report = dict(
            scope="Matched recorded observations; stationary straight probes, not executed parking",
            frames=frames,
            source_sha256=hashes,
            all_support_sets_nested=all(f["nested_support_sets"] for f in frames),
        )
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(
            json.dumps(
                dict(frames=len(frames), all_support_sets_nested=report["all_support_sets_nested"])
            )
        )

    if __name__ == "__main__":
        main()
