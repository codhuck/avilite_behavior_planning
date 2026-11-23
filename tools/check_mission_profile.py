"""Load a mission through the normal AVLite factory and drive it to completion."""

# ruff: noqa: E402 -- resolve the local AVLite checkout when present
if not __name__.startswith("avlite.plugins."):
    import argparse
    import importlib
    import json
    import sys
    from pathlib import Path

    ROOT = Path(__file__).resolve().parents[1]
    if (ROOT.parent / "avlite/__init__.py").is_file():
        sys.path.insert(0, str(ROOT.parent))

    from avlite.c40_execution.c49_settings import ExecutionSettings as Settings
    from avlite.c60_apps.c62_factory import executor_factory, load_stack_settings
    from shapely.geometry import Polygon

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--profile", required=True)
        parser.add_argument("--output", required=True, type=Path)
        args = parser.parse_args()
        if args.output.exists():
            parser.error("Output exists; choose a new file")
        importlib.import_module("avlite.plugins")
        load_stack_settings(args.profile, load_plugins=True)
        if Settings.c40_bridge != "PlanningBasicSim":
            raise ValueError("This check only runs the BasicSim mission fixture")
        keys = {
            "executer_type": "c40_executer_type",
            "bridge": "c40_bridge",
            "perception_strategy_name": "c40_perception",
            "localization_strategy_name": "c40_localization",
            "mapping_strategy_name": "c40_mapping",
            "global_planner_strategy_name": "c40_global_planner",
            "local_planner_strategy_name": "c40_local_planner",
            "controller_strategy_name": "c40_controller",
            "map_file": "c40_map",
            "default_global_trajectory_file": "c40_global_trajectory",
        }
        ex = executor_factory(**{key: getattr(Settings, value) for key, value in keys.items()})
        collisions = 0
        try:
            mission = next(
                t for t in ex.task_runner.tasks if type(t).__name__ == "ParkingMissionTask"
            )
            source = Path(sys.modules[type(mission).__module__].__file__).resolve()
            assert source == ROOT / "execution/mission.py", source
            for _ in range(2400):
                ex.step(sim_dt=0.05, control_dt=0.05, replan_dt=0.05, perception_dt=0.05)
                body = Polygon(ex.ego_state.get_bb_corners())
                collisions += sum(
                    body.intersects(Polygon(a.get_bb_corners())) for a in ex.world.pm.agent_vehicles
                )
                if mission.state in {"COMPLETE", "BLOCKED"}:
                    break
            result = dict(
                passed=mission.state == "COMPLETE" and collisions == 0,
                profile=args.profile,
                state=mission.state,
                reason=mission.reason,
                collisions=collisions,
                source=str(source),
                core_source=str(Path(sys.modules["avlite"].__file__).resolve()),
                seconds=ex.elapsed_sim_time,
                history=mission.history,
            )
        finally:
            ex.stop()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        raise SystemExit(0 if result["passed"] else 1)

    if __name__ == "__main__":
        main()
