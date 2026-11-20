"""Generate three BasicSim parking profiles without overwriting existing files."""

# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import json
    import math
    from pathlib import Path

    import yaml

    def prepare(directory):
        root = Path(__file__).resolve().parents[1]
        directory = Path(directory).expanduser().resolve()
        modes = ["perpendicular_forward", "perpendicular_reverse", "parallel"]
        outputs = [directory / f"parking-{mode}.yaml" for mode in modes]
        assets = directory / "parking-assets"
        if any(p.exists() for p in outputs) or assets.exists():
            raise FileExistsError("Choose a new output directory; profiles/assets already exist")
        assets.mkdir(parents=True)
        for mode, output in zip(modes, outputs):
            parallel = mode == "parallel"
            reverse = mode == "perpendicular_reverse"
            start = [9, 3, 0] if parallel else [-12, 0, math.pi if reverse else 0]
            goal = [0, 0, 0] if parallel else [0, 11, -math.pi / 2 if reverse else math.pi / 2]
            bay = (
                [[-3.8, -1.5], [3.8, -1.5], [3.8, 1.5], [-3.8, 1.5]]
                if parallel
                else [[-1.6, 8], [1.6, 8], [1.6, 14], [-1.6, 14]]
            )
            area = (
                [[-14, -4], [15, -4], [15, 9], [-14, 9]]
                if parallel
                else [[-16, -5], [8, -5], [8, 15], [-16, 15]]
            )
            cars = (
                [dict(x=-6.25, y=0, theta=0), dict(x=6.25, y=0, theta=0)]
                if parallel
                else [
                    dict(x=-3.5, y=10.5, theta=math.pi / 2),
                    dict(x=3.5, y=10.5, theta=math.pi / 2),
                ]
            )
            scene = {
                "version": 1,
                "demo": True,
                "drivable": area,
                "spaces": {
                    "demo_bay": {
                        "types": [mode],
                        "goals": {mode: goal},
                        "bay": bay,
                        "parked_vehicles": cars,
                    }
                },
            }
            scene_path = assets / f"{mode}.json"
            scene_path.write_text(json.dumps(scene, indent=2))
            trajectory = assets / f"{mode}-route.json"
            trajectory.write_text(
                json.dumps(
                    {
                        "ReferenceLine": [[*start[:2], 0], [*goal[:2], 0]],
                        "ReferenceSpeed": [0, 0],
                        "LeftBound": [20, 20],
                        "RightBound": [-20, -20],
                    }
                )
            )
            profile = {
                "c69_apps": {
                    "c60_selected_profile": output.stem,
                    "c62_load_plugins": True,
                    "c62_community_plugins": {"avlite-behavior-hdmap": str(root)},
                    "c62_default_plugins": ["p60_visualizer_tk"],
                },
                "c40_execution": {
                    "c40_bridge": "PlanningBasicSim",
                    "c40_map": "",
                    "c40_global_trajectory": str(trajectory),
                    "c40_global_planner": "",
                    "c40_local_planner": "ParkingPlanner",
                    "c40_controller": "ParkingController",
                    "c40_executer_type": "PlanningSyncExecuter",
                    "c40_perception": "",
                    "c40_localization": "",
                    "c40_mapping": "",
                    "c40_execution_tasks": ["ParkingTask"],
                    "c40_start_pose": start,
                    "c40_sim_dt": 0.05,
                    "c40_control_dt": 0.05,
                    "c40_replan_dt": 0.05,
                    "c40_pace_sim": True,
                    "c46_npc_control": False,
                    "c41_world_capabilities": [],
                    "c41_world_stack_capabilities": ["LOCALIZATION", "DETECTION"],
                },
                "plugins": {
                    "avlite_behavior_hdmap": {
                        "parking_scene_file": str(scene_path),
                        "parking_simulator_obstacles": True,
                    }
                },
            }
            output.write_text(yaml.safe_dump(profile, sort_keys=False))
        return outputs

    if __name__ == "__main__":
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--output-dir", type=Path, required=True)
        for result in prepare(parser.parse_args().output_dir):
            print(result)
