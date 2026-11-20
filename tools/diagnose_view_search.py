"""Post-hoc search-budget diagnosis from saved initial clouds; no future inputs."""

# ruff: noqa: E402
# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import importlib
    import json
    import sys
    import time
    import types
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
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("recording", type=Path)
        parser.add_argument("--output", type=Path, required=True)
        parser.add_argument(
            "--adapter",
            action="store_true",
            help="Exercise stopped recovery in the real visibility adapter",
        )
        args = parser.parse_args()
        if args.output.exists():
            raise ValueError("Use a new output file")
        rows = []
        for row in json.loads((args.recording / "results.json").read_text()):
            if row["proposal_available"]:
                continue
            root = args.recording / row["scene"]["id"]
            clouds = []
            for index in range(9):
                with np.load(root / f"initial_cloud_{index}.npz") as raw:
                    pose = raw["pose"]
                    clouds.append(
                        Lidar(
                            points=raw["points"],
                            stamp=float(raw["stamp"]),
                            base_to_sensor=raw["base_to_sensor"],
                            sensor_name=f"lidar_{index}",
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
            mode = row["scene"]["mode"]
            parallel = mode == "parallel"
            goal = (
                (52, 0, 0)
                if parallel
                else (52, 11, -np.pi / 2 if mode == "perpendicular_reverse" else np.pi / 2)
            )
            bay = box(48.2, -1.5, 55.8, 1.5) if parallel else box(50.4, 8, 53.6, 14)
            trials = []
            for budget in [1500] if args.adapter else [1500, 6000]:
                route = GlobalPlan(
                    path=[pose[:2], goal[:2]],
                    trajectory=TrajectoryTracker(path=[pose[:2], goal[:2]], velocity=[0, 0]),
                )
                cls = ParkingPlanner
                if args.adapter:
                    package = types.ModuleType("scan_search_diagnostic")
                    package.__path__ = [str(PLUGIN)]
                    sys.modules[package.__name__] = package
                    cls = importlib.import_module(
                        package.__name__ + ".parking.visibility"
                    ).VisibilityParkingPlanner
                pm = PerceptionModel(ego_vehicle=ego)
                pm.parking_occupancy = observation
                planner = cls(route, pm)
                planner.rear_offset = 1.25325
                planner.allow_multi_maneuver = True
                planner.deterministic_search = True
                planner.search_expansions = budget
                planner.refine_search_endpoint = True
                planner.obstacles = [observation.occupied]
                planner.request_parking(ParkingRequest(goal, bay, area, mode))
                started = time.perf_counter()
                attempts = []
                for attempt in range(3 if args.adapter else 1):
                    path = planner._generate()
                    attempts.append(
                        {
                            "attempt": attempt + 1,
                            "proposal": path is not None,
                            "reason": planner.reason,
                        }
                    )
                    if path is not None:
                        break
                trials.append(
                    {
                        "expansions": budget,
                        "attempts": attempts,
                        "proposal": path is not None,
                        "reason": planner.reason,
                        "wall_seconds": time.perf_counter() - started,
                    }
                )
            rows.append({"scene": row["scene"]["id"], "trials": trials})
            print(rows[-1], flush=True)
        args.output.write_text(
            json.dumps({"scope": __doc__, "adapter": args.adapter, "rows": rows}, indent=2) + "\n"
        )

    if __name__ == "__main__":
        main()
