"""Generate three drive-to-parking BasicSim profiles and status-file locations."""

if not __name__.startswith("avlite.plugins."):
    import argparse
    import json
    import math
    from pathlib import Path

    import yaml
    from prepare_parking_profiles import prepare as prepare_parking

    def prepare(directory):
        if any(Path(directory).expanduser().glob("mission-*.yaml")):
            raise FileExistsError("Choose a new directory; mission profiles already exist")
        profiles = prepare_parking(directory)
        outputs = []
        for filename in profiles:
            profile = yaml.safe_load(filename.read_text())
            config = profile["c40_execution"]
            settings = profile["plugins"]["avlite_behavior_hdmap"]
            approach = config["c40_start_pose"]
            c, s = math.cos(approach[2]), math.sin(approach[2])
            start = [approach[0] - 8 * c, approach[1] - 8 * s, approach[2]]
            config["c40_start_pose"] = start
            config["c40_local_planner"] = "ReferencePathPlanner"
            config["c40_global_planner"] = "SurveyedRoutePlanner"
            config["c40_controller"] = "BehaviorStanleyController"
            config["c40_execution_tasks"] = [
                "ParkingDemoTrafficTask",
                "ParkingMissionTask",
                "DecisionStatusTask",
            ]
            route = [[start[0] + i * 0.1 * c, start[1] + i * 0.1 * s, 0] for i in range(131)]
            Path(config["c40_global_trajectory"]).write_text(
                json.dumps(
                    {
                        "ReferenceLine": route,
                        "ReferenceSpeed": [1.0] * 130 + [0.0],
                        "LeftBound": [20.0] * 131,
                        "RightBound": [-20.0] * 131,
                    }
                )
            )
            scene_path = Path(settings["parking_scene_file"])
            scene = json.loads(scene_path.read_text())
            scene["demo"] = False
            space = scene["spaces"]["demo_bay"]
            mode = space["types"][0]
            scene["demo_vehicles"] = space.pop("parked_vehicles")
            xs = [p[0] for p in scene["drivable"]] + [start[0] - 3, start[0] + 3]
            ys = [p[1] for p in scene["drivable"]] + [start[1] - 3, start[1] + 3]
            scene["drivable"] = [
                [min(xs), min(ys)],
                [max(xs), min(ys)],
                [max(xs), max(ys)],
                [min(xs), max(ys)],
            ]
            scene_path.write_text(json.dumps(scene, indent=2))
            mission_path = scene_path.with_name(f"{mode}-mission.json")
            mission_path.write_text(
                json.dumps(
                    {
                        "version": 1,
                        "approach": approach,
                        "parking_type": mode,
                        "spaces": ["demo_bay"],
                    },
                    indent=2,
                )
            )
            settings["mission_file"] = str(mission_path)
            settings["decision_status_file"] = str(scene_path.with_name(f"{mode}-status.json"))
            output = filename.with_name(f"mission-{mode}.yaml")
            profile["c69_apps"]["c60_selected_profile"] = output.stem
            output.write_text(yaml.safe_dump(profile, sort_keys=False))
            filename.unlink()
            outputs.append(output)
        return outputs

    if __name__ == "__main__":
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--output-dir", type=Path, required=True)
        for result in prepare(parser.parse_args().output_dir):
            print(result)
