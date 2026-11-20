"""Counterfactual sensor-loss/speed study on recorded CARLA LiDAR, without driving.

Actor boxes are evaluator labels only. Decisions certify short straight probes,
not completed parking. Each cloud retains its mount and acquisition timestamp.
"""

# ruff: noqa: E402
# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import json
    import sys
    from dataclasses import asdict, replace
    from pathlib import Path

    import numpy as np
    from shapely.geometry import GeometryCollection, box, shape

    PLUGIN = Path(__file__).resolve().parents[1]
    sys.path[:0] = [str(PLUGIN), str(PLUGIN.parent)]
    from avlite.c10_perception.c11_perception_model import EgoState
    from avlite.c50_common.c52_world_sensor_datatypes import Lidar

    from parking_occupancy import FusedLidarParkingOccupancy
    from visibility_dispatch import Maneuver, VisibilityDispatcher, VisibilitySnapshot

    def initial_clouds(recording, prefix):
        """Load only the acquisition paired with the selected initial geometry."""
        clouds = []
        for path in sorted(recording.glob(f"{prefix}_cloud_*.npz")):
            with np.load(path) as raw:
                clouds.append(
                    Lidar(
                        points=raw["points"],
                        stamp=float(raw["stamp"]),
                        base_to_sensor=raw["base_to_sensor"],
                        sensor_name=f"lidar_{path.stem.rsplit('_', 1)[1]}",
                    )
                )
        stamps = [c.stamp for c in clouds]
        if not stamps or not np.isfinite(stamps).all() or max(stamps) - min(stamps) > 1e-5:
            raise ValueError("Initial clouds must be present and share one acquisition time")
        return clouds

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("recording", type=Path)
        parser.add_argument("--obstacle-padding", type=float, default=0.35)
        parser.add_argument("--hazard-support-radius", type=float, default=0.0)
        parser.add_argument("--output", type=Path, default=Path("logs/parking_coverage_replay"))
        args = parser.parse_args()
        geometry = sorted(args.recording.glob("*_geometry_0.json"))[0]
        data = json.loads(geometry.read_text())
        x, y, z, yaw = data["pose"]
        vertices = np.asarray(shape(data["ego"]).exterior.coords)
        ego = EgoState(
            x=x,
            y=y,
            z=z,
            theta=yaw,
            length=np.linalg.norm(vertices[1] - vertices[0]),
            width=np.linalg.norm(vertices[2] - vertices[1]),
        )
        clouds = initial_clouds(args.recording, geometry.name.removesuffix("_geometry_0.json"))
        now = clouds[0].stamp
        cases = {"all": clouds, "roof_only": clouds[:1], "none": []}
        for i in range(len(clouds)):
            cases[f"drop_{i}"] = [c for j, c in enumerate(clouds) if j != i]
            cases[f"stale_{i}"] = [
                replace(c, stamp=now - 0.5) if i == j else c for j, c in enumerate(clouds)
            ]
        rows = []
        for name, sources in cases.items():
            for support in (1, 2):
                provider = FusedLidarParkingOccupancy(
                    minimum_support=support,
                    obstacle_padding=args.obstacle_padding,
                    hazard_support_radius=args.hazard_support_radius,
                )
                observation = provider.observe(sources, ego, box(30, -6, 70, 18), now, 0)
                snapshot = VisibilitySnapshot(
                    observation.traversable,
                    observation.occupied
                    if observation.occupied is not None
                    else GeometryCollection(),
                    now,
                    observation.sources,
                    support,
                )
                for speed in (0.05, 0.1, 0.2, 0.4):
                    candidates = []
                    for direction in (1, -1):
                        distance = np.linspace(0, 3, 76) * direction
                        poses = np.column_stack(
                            (
                                x + distance * np.cos(yaw),
                                y + distance * np.sin(yaw),
                                np.full(len(distance), yaw),
                            )
                        )
                        candidates.append(
                            Maneuver(f"probe_{direction}", "park", "probe", poses, direction, speed)
                        )
                    dispatcher = VisibilityDispatcher(allow_prefix=True)
                    decision = dispatcher.select(
                        candidates, snapshot, now, [x, y, yaw], 0, ego.length, ego.width, {"probe"}
                    )
                    rows.append(
                        dict(
                            case=name,
                            minimum_support=support,
                            requested_speed=speed,
                            state=decision.state,
                            reason=decision.reason,
                            certificate=asdict(decision.certificate)
                            if decision.certificate
                            else None,
                            free_area=observation.free.area,
                            evaluator_box_overlap=sum(
                                observation.free.intersection(shape(o)).area
                                for o in data["obstacles"]
                            ),
                            support_histogram=provider.support_histogram,
                            rejections=dispatcher.rejections,
                        )
                    )
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "results.json").write_text(
            json.dumps(
                dict(
                    scope="Recorded-cloud counterfactual decisions only; no vehicle motion",
                    recording=str(args.recording),
                    obstacle_padding=args.obstacle_padding,
                    rows=rows,
                    hazard_support_radius=args.hazard_support_radius,
                ),
                indent=2,
            )
            + "\n"
        )
        for row in rows:
            if row["case"] in ("all", "roof_only"):
                print(
                    row["case"],
                    row["minimum_support"],
                    row["requested_speed"],
                    row["state"],
                    round(row["certificate"]["released_length"], 2) if row["certificate"] else 0,
                    round(row["evaluator_box_overlap"], 4),
                )

    if __name__ == "__main__":
        main()
