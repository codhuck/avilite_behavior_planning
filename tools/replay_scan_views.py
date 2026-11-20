"""Development replay of finite-ray view ranking on retained counterfactual scans.

These recordings informed the model: this is NOT independent validation.
Future clouds/outcomes are used only after scores and selections are computed.
"""

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
    from avlite.c10_perception.c11_perception_model import EgoState
    from avlite.c50_common.c52_world_sensor_datatypes import Lidar
    from shapely.geometry import box

    from parking_occupancy import FusedLidarParkingOccupancy
    from scan_visibility import ScanVisibility
    from task_visibility import GroundLidar, TaskVisibility
    from visibility_dispatch import VisibilitySnapshot

    def load_task(directory):
        clouds, geometry = [], []
        for i in range(9):
            with np.load(directory / f"initial_cloud_{i}.npz") as raw:
                pose = raw["pose"]
                mount = raw["base_to_sensor"]
                clouds.append(
                    Lidar(
                        points=raw["points"],
                        stamp=float(raw["stamp"]),
                        base_to_sensor=mount,
                        sensor_name=f"lidar_{i}",
                    )
                )
                geometry.append(
                    GroundLidar(
                        mount[0, 3],
                        mount[1, 3],
                        pose[2] + mount[2, 3],
                        tuple(np.linspace(-45 if i == 0 else -85, 10, 64)),
                        30 if i == 0 else 15,
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
        observation = FusedLidarParkingOccupancy(
            minimum_support=1, hazard_support_radius=2
        ).observe(clouds, ego, box(30, -6, 70, 18), clouds[0].stamp, 0)
        snapshot = VisibilitySnapshot(
            observation.traversable, observation.occupied, observation.stamp
        )
        task = TaskVisibility(snapshot, np.load(directory / "task_path.npy"), ego.width, ego.length)
        return task, geometry, np.array([ego.x, ego.y, ego.theta]), ego

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("recording", type=Path)
        parser.add_argument("--output", type=Path, required=True)
        args = parser.parse_args()
        if args.output.exists():
            raise ValueError("Use a new report path")
        rows = []
        source = args.recording / "results.json"
        for row in json.loads(source.read_text()):
            if not row["proposal_available"]:
                rows.append({"scene": row["scene"]["id"], "status": "no_proposal"})
                continue
            task, sensors, pose, ego = load_task(args.recording / row["scene"]["id"])
            start = time.perf_counter()
            scorer = ScanVisibility(task, sensors, body_length=ego.length, body_width=ego.width)
            predictions = []
            for candidate in row["candidates"]:
                if not candidate["admissible"]:
                    continue
                utility = scorer.evaluate(candidate["endpoint"])
                predictions.append(
                    {
                        "name": candidate["name"],
                        "advantage": scorer.advantage(
                            pose, candidate["endpoint"], candidate["cost"] - 1
                        ),
                        "expected_area": utility.expected_area,
                        "phase_min": utility.phase_min,
                        "phase_max": utility.phase_max,
                    }
                )
            eligible = [c for c in predictions if c["advantage"] > 0]
            chosen = max(eligible, key=lambda c: c["advantage"])["name"] if eligible else "stay"
            seconds = time.perf_counter() - start
            # Read recorded outcome only after all predictions and selection.
            actual = (
                row["stationary_gain"]
                if chosen == "stay"
                else next(
                    c["acquired_weighted_gain"] for c in row["candidates"] if c["name"] == chosen
                )
            )
            result = {
                "scene": row["scene"]["id"],
                "status": "evaluated",
                "choice": chosen,
                "acquired_gain": actual,
                "stationary_gain": row["stationary_gain"],
                "bay_gain": row["choices"]["bay_area"]["gain"],
                "predicted_stay": scorer.evaluate(pose).expected_area,
                "wall_seconds": seconds,
                "predictions": predictions,
            }
            rows.append(result)
            print(result["scene"], chosen, round(actual, 5), round(seconds, 2), flush=True)
        valid = [r for r in rows if r["status"] == "evaluated"]
        report = {
            "scope": __doc__,
            "input_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "source_sha256": {
                p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                for p in [
                    PLUGIN / "scan_visibility.py",
                    PLUGIN / "task_visibility.py",
                    Path(__file__),
                ]
            },
            "rows": rows,
            "summary": {
                key: float(np.mean([r[key] for r in valid]))
                for key in ["acquired_gain", "stationary_gain", "bay_gain", "wall_seconds"]
            },
            "moves": sum(r["choice"] != "stay" for r in valid),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report["summary"], indent=2))

    if __name__ == "__main__":
        main()
