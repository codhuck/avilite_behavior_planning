"""Re-score saved one-step CARLA acquisitions; labels remain separate from ranking."""

# ruff: noqa: E402
# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import json
    import sys
    from pathlib import Path

    PLUGIN = Path(__file__).resolve().parents[1]
    sys.path[:0] = [str(PLUGIN), str(PLUGIN.parent)]
    import numpy as np
    from _bootstrap import load_plugin
    from avlite.c10_perception.c11_perception_model import EgoState
    from avlite.c50_common.c52_world_sensor_datatypes import Lidar
    from shapely.geometry import box

    load_plugin()

    from avlite_coordination_demo.parking.dispatch import VisibilitySnapshot
    from avlite_coordination_demo.parking.occupancy import FusedLidarParkingOccupancy
    from avlite_coordination_demo.parking.view import GroundLidar, TaskVisibility, beam_gain

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("recording", type=Path)
        parser.add_argument("--output", type=Path, required=True)
        args = parser.parse_args()
        results = json.loads((args.recording / "results.json").read_text())
        rows = []
        for result in results:
            row = dict(
                scene=result["scene"]["id"],
                proposal_available=result["proposal_available"],
                choices=result["choices"],
            )
            if not result["proposal_available"]:
                row["choices"] = {
                    method: {"name": "no_proposal", "gain": 0.0}
                    for method in ["none", "bay_area", "task", "task_beams"]
                }
                rows.append(row)
                continue
            directory = args.recording / result["scene"]["id"]
            clouds = []
            sensors = []
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
                    sensors.append(
                        GroundLidar(
                            float(mount[0, 3]),
                            float(mount[1, 3]),
                            float(pose[2] + mount[2, 3]),
                            tuple(np.linspace(-45 if i == 0 else -85, 10, 64)),
                            30 if i == 0 else 15,
                        )
                    )
            ego = EgoState(
                x=pose[0],
                y=pose[1],
                z=pose[2],
                theta=pose[3],
                length=3.7053694725,
                width=1.7886785269,
            )
            observation = FusedLidarParkingOccupancy(
                minimum_support=1, hazard_support_radius=2
            ).observe(clouds, ego, box(30, -6, 70, 18), clouds[0].stamp, 0)
            snapshot = VisibilitySnapshot(
                observation.traversable, observation.occupied, clouds[0].stamp
            )
            task = TaskVisibility(
                snapshot, np.load(directory / "task_path.npy"), ego.width, ego.length
            )
            candidates = []
            for candidate in result["candidates"]:
                if not candidate["admissible"]:
                    continue
                score = (
                    beam_gain(task, [ego.x, ego.y, ego.theta], candidate["endpoint"], sensors)
                    / candidate["cost"]
                )
                candidates.append(
                    dict(
                        name=candidate["name"],
                        score=score,
                        gain=candidate["acquired_weighted_gain"],
                    )
                )
            winner = max(candidates, key=lambda r: r["score"]) if candidates else None
            if winner is not None and winner["score"] <= 0:
                winner = None
            row["choices"]["task_beams"] = {
                "name": winner["name"] if winner else "stay",
                "gain": winner["gain"] if winner else 0.0,
            }
            row["beam_candidates"] = candidates
            rows.append(row)
        summary = {
            method: dict(
                mean_gain=float(np.mean([r["choices"][method]["gain"] for r in rows])),
                selections=sum(
                    r["choices"][method]["name"] not in ["stay", "no_proposal"] for r in rows
                ),
            )
            for method in ["none", "bay_area", "task", "task_beams"]
        }
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "report.json").write_text(
            json.dumps(
                {
                    "scope": "Post-hoc beam-aware rescore of development acquisitions; not independent validation",
                    "rows": rows,
                    "summary": summary,
                },
                indent=2,
            )
            + "\n"
        )
        print(json.dumps(summary, indent=2))

    if __name__ == "__main__":
        main()
