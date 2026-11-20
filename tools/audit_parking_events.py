"""Audit saved CARLA crossing clouds against evaluator-only pedestrian boxes."""

# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import json
    from pathlib import Path

    import numpy as np
    import shapely
    from shapely.geometry import shape

    def audit(recording):
        results = json.loads((recording / "results.json").read_text())
        rows = []
        expected_cases = {
            result["scenario"]["id"]
            for result in results
            if result.get("scenario", {}).get("pedestrian")
        }
        for result in results:
            scenario = result.get("scenario", {})
            if not scenario.get("pedestrian"):
                continue
            directory = recording / scenario["id"]
            prefix = f"{result['type']}_{result['repeat']}"
            for geometry_path in sorted(directory.glob(f"{prefix}_geometry_*.json")):
                step = int(geometry_path.stem.rsplit("_", 1)[1])
                clouds = sorted(directory.glob(f"{prefix}_event_{step}_cloud_*.npz"))
                if not clouds:
                    continue
                geometry = json.loads(geometry_path.read_text())
                # The runner stores two neighbouring vehicles followed by its walker.
                if len(geometry["obstacles"]) != 3:
                    raise ValueError("Expected two vehicle labels and one pedestrian label")
                pedestrian = shape(geometry["obstacles"][-1])
                counts, stamps, poses = {}, [], []
                for path in clouds:
                    with np.load(path) as data:
                        points = data["points"][:, :3]
                        mount = data["base_to_sensor"]
                        base = points @ mount[:3, :3].T + mount[:3, 3]
                        x, y, z, yaw = data["ego_pose"]
                        cosine, sine = np.cos(yaw), np.sin(yaw)
                        xy = base[:, :2] @ np.array([[cosine, sine], [-sine, cosine]])
                        xy += [x, y]
                        above_ground = (base[:, 2] + z > 0.15) & (base[:, 2] + z < 2.5)
                        xmin, ymin, xmax, ymax = pedestrian.bounds
                        inside_bounds = (
                            above_ground
                            & (xy[:, 0] >= xmin)
                            & (xy[:, 0] <= xmax)
                            & (xy[:, 1] >= ymin)
                            & (xy[:, 1] <= ymax)
                        )
                        count = int(
                            np.sum(shapely.covers(pedestrian, shapely.points(xy[inside_bounds])))
                        )
                        counts[path.stem.rsplit("_", 1)[1]] = count
                        stamps.append(float(data["stamp"]))
                        poses.append(data["ego_pose"].tolist())
                rows.append(
                    dict(
                        case=scenario["id"],
                        step=step,
                        devices=len(clouds),
                        expected_devices=1
                        if result.get("policy_configuration", {}).get("roof_only")
                        else 9,
                        above_ground_points_in_pedestrian_box=counts,
                        total_points=sum(counts.values()),
                        synchronized=max(stamps) - min(stamps) < 1e-5,
                        pose_matches_evaluation=all(
                            np.allclose(p, geometry["pose"], atol=1e-6) for p in poses
                        ),
                    )
                )
        return dict(
            scope="Offline raw-cloud evidence at saved matching frames; labels are evaluator-only",
            rows=rows,
            expected_cases=sorted(expected_cases),
            missing_event_cases=sorted(expected_cases - {row["case"] for row in rows}),
            note="A trial ending before a saved matching event has no raw event-cloud audit; telemetry is separate evidence",
            all_events_supported=bool(rows)
            and {row["case"] for row in rows} == expected_cases
            and all(
                row["devices"] == row["expected_devices"]
                and row["total_points"] > 0
                and row["synchronized"]
                and row["pose_matches_evaluation"]
                for row in rows
            ),
        )

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("recording", type=Path)
        parser.add_argument("--output", type=Path, required=True)
        args = parser.parse_args()
        result = audit(args.recording)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result))
        return 0 if result["all_events_supported"] else 1

    if __name__ == "__main__":
        raise SystemExit(main())
