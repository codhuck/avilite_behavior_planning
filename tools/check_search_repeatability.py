"""Repeat geometry search on identical saved measurements; not physics determinism."""

# ruff: noqa: E402
# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import hashlib
    import json
    import sys
    import time
    from pathlib import Path

    PLUGIN = Path(__file__).resolve().parents[1]
    sys.path[:0] = [str(PLUGIN), str(PLUGIN.parent)]
    import numpy as np
    from _bootstrap import load_plugin
    from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
    from avlite.c20_planning.c21_planning_model import GlobalPlan
    from avlite.c50_common.c52_world_sensor_datatypes import Lidar
    from avlite.c50_common.c54_trajectory_tracker import TrajectoryTracker
    from shapely.geometry import box

    load_plugin()

    from avlite_coordination_demo.parking.occupancy import FusedLidarParkingOccupancy
    from avlite_coordination_demo.parking.planner import ParkingPlanner, ParkingRequest

    def main():
        p = argparse.ArgumentParser(description=__doc__)
        p.add_argument("recording", type=Path)
        p.add_argument("--output", type=Path, required=True)
        args = p.parse_args()
        rows = []
        for result in json.loads((args.recording / "results.json").read_text()):
            mode = result["scene"]["mode"]
            directory = args.recording / result["scene"]["id"]
            clouds = []
            for i in range(9):
                with np.load(directory / f"initial_cloud_{i}.npz") as raw:
                    pose = raw["pose"]
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
            area = box(30, -6, 70, 18)
            observation = FusedLidarParkingOccupancy(
                minimum_support=1, hazard_support_radius=2
            ).observe(clouds, ego, area, clouds[0].stamp, 0)
            parallel = mode == "parallel"
            reverse = mode == "perpendicular_reverse"
            goal = (52, 0, 0) if parallel else (52, 11, -np.pi / 2 if reverse else np.pi / 2)
            bay = box(48.2, -1.5, 55.8, 1.5) if parallel else box(50.4, 8, 53.6, 14)
            repeats = []
            for _ in range(3):
                start = time.perf_counter()
                route = GlobalPlan(
                    path=[pose[:2], goal[:2]],
                    trajectory=TrajectoryTracker(path=[pose[:2], goal[:2]], velocity=[0, 0]),
                )
                planner = ParkingPlanner(route, PerceptionModel(ego_vehicle=ego))
                planner.rear_offset = 1.25325
                planner.allow_multi_maneuver = True
                planner.deterministic_search = True
                planner.search_expansions = 1500
                planner.refine_search_endpoint = True
                planner.obstacles = [observation.occupied]
                planner.request_parking(ParkingRequest(goal, bay, area, mode))
                path = planner._generate()
                repeats.append(
                    {
                        "path_sha256": hashlib.sha256(path.tobytes()).hexdigest()
                        if path is not None
                        else None,
                        "wall_seconds": time.perf_counter() - start,
                    }
                )
            rows.append(
                {
                    "scene": result["scene"]["id"],
                    "repeatable": len({r["path_sha256"] for r in repeats}) == 1,
                    "repeats": repeats,
                }
            )
            print(rows[-1]["scene"], rows[-1]["repeatable"], flush=True)
        report = {
            "scope": __doc__,
            "expansion_budget": 1500,
            "rows": rows,
            "all_repeatable": all(r["repeatable"] for r in rows),
        }
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        if not report["all_repeatable"]:
            raise SystemExit(1)

    if __name__ == "__main__":
        main()
