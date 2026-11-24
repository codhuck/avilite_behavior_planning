"""Write a new machine-local BasicSim coordinator profile (no pip or CARLA)."""

# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    from pathlib import Path

    import numpy as np
    import yaml
    from coordinator_demo import PLUGIN, GlobalPlan, fixture_paths

    def write_profile(output, mode, executor="PlanningSyncExecuter"):
        output = Path(output).expanduser().resolve()
        map_path, route_path = fixture_paths(mode)
        for path in (map_path, route_path):
            if not path.is_file():
                raise FileNotFoundError(path)
        route = GlobalPlan.from_file(route_path)
        delta = np.asarray(route.path[1]) - route.path[0]
        yaw = float(np.arctan2(delta[1], delta[0]))
        tasks = ["PlanningCoordinatorTask"]
        if mode == "race":
            tasks.append("RaceTrafficTask")
        config = {
            "c69_apps": {
                "c60_selected_profile": output.stem,
                "c62_community_plugins": {"avlite-behavior-hdmap": str(PLUGIN)},
                "c62_default_plugins": [
                    "p60_headless_mode",
                    "p60_setting_cli",
                    "p60_visualizer_tk",
                ],
                "c62_load_plugins": True,
            },
            "c10_perception": {
                "c12_detection_strategy": "",
                "c12_tracking_strategy": "",
                "c12_prediction_strategy": "ConstantVelocityPrediction",
            },
            "c20_planning": {
                "c23_behavioral_strategy": (
                    "RaceBehaviorPlanner" if mode == "race" else "HDMapBehaviorTreePlanner"
                ),
                "c23_path_strategy": "RacePathPlanner"
                if mode == "race"
                else "ReferencePathPlanner",
                "c23_velocity_strategy": (
                    "RaceVelocityPlanner" if mode == "race" else "HDMapRuleVelocityPlanner"
                ),
            },
            "c40_execution": {
                "c40_bridge": "PlanningBasicSim",
                "c40_executer_type": executor,
                "c40_perception": "PerceptionPipeline",
                "c40_localization": "",
                "c40_mapping": "",
                "c40_global_planner": (
                    "GlobalCenterlineRacePlanner" if mode == "race" else "HDMapLaneFollowingPlanner"
                ),
                "c40_local_planner": "BehaviorPlanningPipeline",
                "c40_controller": "BehaviorStanleyController",
                "c40_execution_tasks": tasks,
                "c40_map": str(map_path),
                "c40_global_trajectory": str(route_path),
                "c40_start_pose": [float(route.path[0][0]), float(route.path[0][1]), yaw],
                "c40_sim_dt": 0.05,
                "c40_perception_dt": 0.05,
                "c40_control_dt": 0.05,
                "c40_replan_dt": 0.05,
                "c40_pace_sim": True,
                "c40_pace_control": True,
                "c40_pace_perception": True,
                "c40_pace_replan": True,
                "c41_world_capabilities": None,
                "c41_world_stack_capabilities": ["LOCALIZATION", "DETECTION", "TRACKING"],
                "c46_npc_control": False,
            },
            "plugins": {"avlite_behavior_hdmap": {"coordinator_initial_strategy": "auto"}},
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("x") as stream:
            yaml.safe_dump(config, stream, sort_keys=False)
        return output

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--mode", choices=["road", "race"], required=True)
        parser.add_argument("--output", type=Path, required=True)
        parser.add_argument(
            "--executor",
            choices=["PlanningSyncExecuter", "SyncExecuter", "AsyncThreadedExecuter"],
            default="PlanningSyncExecuter",
            help="Stock 0.6.7 uses the legacy adapter; standard executors require the local execution API patch.",
        )
        args = parser.parse_args()
        print(write_profile(args.output, args.mode, args.executor))

    if __name__ == "__main__":
        main()
