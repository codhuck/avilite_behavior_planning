"""Load a generated BasicSim profile through AVLite's actual factory and run it."""

# ruff: noqa: E402 -- support an unpacked matching source bundle
# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import importlib
    import json
    import sys
    import time
    from pathlib import Path

    PLUGIN = Path(__file__).resolve().parents[1]
    if (PLUGIN.parent / "avlite/__init__.py").is_file():
        sys.path.insert(0, str(PLUGIN.parent))

    from avlite.c40_execution.c49_settings import ExecutionSettings as Settings
    from avlite.c60_apps.c62_factory import executor_factory, load_stack_settings
    from avlite.c60_apps.c65_setting_utils import profile_file_path

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument(
            "--profile", required=True, help="Profile name without .yaml; uses AVLITE_CONFIG_DIR"
        )
        parser.add_argument("--output", type=Path, required=True)
        args = parser.parse_args()
        if args.output.exists():
            parser.error("Output exists; choose another path")
        if not Path(profile_file_path(args.profile)).is_file():
            parser.error("Profile not found; pass its name without .yaml and set AVLITE_CONFIG_DIR")
        # Match the normal application bootstrap before community discovery.
        importlib.import_module("avlite.plugins")
        load_stack_settings(args.profile, load_plugins=True)
        if Settings.c40_bridge not in {"BasicSim", "PlanningBasicSim"}:
            raise ValueError("This check only runs BasicSim profiles")
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
        try:
            task = next(
                t for t in ex.task_runner.tasks if type(t).__name__ == "PlanningCoordinatorTask"
            )
            source = Path(sys.modules[type(task).__module__].__file__).resolve()
            assert source == PLUGIN / "execution/coordinator.py", source
            deadline = time.monotonic() + 20
            while ex.elapsed_sim_time < 3.0:
                if time.monotonic() >= deadline:
                    raise TimeoutError("Executor did not advance three simulation seconds")
                ex.step(
                    sim_dt=0.05,
                    perception_dt=0.05,
                    control_dt=0.05,
                    replan_dt=0.05,
                    call_perceive=True,
                )
                if ex.ui_poll_delay is not None:
                    time.sleep(min(ex.ui_poll_delay, 0.02))
            assert task.last_result["status"] == "selected", task.last_result
            assert ex.ego_state.velocity > 0.05
            if task.active_strategy == "race":
                assert len(ex.pm.agent_vehicles) == 2, "Scripted rivals did not reach perception"
            assert "carla" not in sys.modules
            result = dict(
                passed=True,
                profile=args.profile,
                source=str(source),
                strategy=task.active_strategy,
                sim_seconds=ex.elapsed_sim_time,
                speed=ex.ego_state.velocity,
                carla_imported=False,
                core_source=str(Path(sys.modules["avlite"].__file__).resolve()),
                rivals=len(ex.pm.agent_vehicles),
                executor=type(ex).__name__,
                selection=task.last_result,
            )
        finally:
            ex.stop()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))

    if __name__ == "__main__":
        main()
