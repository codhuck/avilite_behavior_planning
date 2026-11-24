"""Run stopped planner handovers in BasicSim, without CARLA or model downloads.

Use --mode road or --mode race; JSON contains transitions and measured movement.
The small road is a synthetic coordination fixture, not a perception benchmark.
"""

# ruff: noqa: E402 -- load the explicitly selected sibling plugin first

# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import importlib.util
    import json
    import logging
    import sys
    from pathlib import Path

    import numpy as np

    PLUGIN = Path(__file__).resolve().parents[1]
    if (PLUGIN.parent / "avlite/__init__.py").is_file():
        sys.path.insert(0, str(PLUGIN.parent))
    NAME = "avlite_coordination_demo"
    if NAME not in sys.modules:
        spec = importlib.util.spec_from_file_location(
            NAME, PLUGIN / "__init__.py", submodule_search_locations=[str(PLUGIN)]
        )
        plugin = importlib.util.module_from_spec(spec)
        sys.modules[NAME] = plugin
        spec.loader.exec_module(plugin)
    else:
        plugin = sys.modules[NAME]

    from avlite.c10_perception.c11_perception_model import EgoState, HDMap, PerceptionModel, RaceMap
    from avlite.c10_perception.c12_perception_strategy import PerceptionPipeline
    from avlite.c20_planning.c21_planning_model import GlobalPlan
    from avlite.c20_planning.c26_local_path_planners import ReferencePathPlanner
    from avlite.c20_planning.c29_settings import PlanningSettings
    from avlite.c30_control.c31_control_model import ControlCommand
    from avlite.c40_execution.c43_task_strategy import ExecutionStarted
    from avlite.c40_execution.c49_settings import ExecutionSettings, ExecutionSettingsSchema

    def fixture_paths(mode):
        import avlite

        data = Path(avlite.__file__).resolve().parent / "data"
        if mode == "race":
            return (
                data / "race_boundary_yas_marina.map.json",
                PLUGIN / "data/yas_marina_dispatcher.json",
            )
        return PLUGIN / "data/coordinator_road.xodr", PLUGIN / "data/coordinator_road.json"

    def create_stack(mode="road", executor=None):
        # Explicit settings; never load the user's active profile or launch a bridge.
        ExecutionSettings.c41_world_stack_capabilities = ["LOCALIZATION", "DETECTION", "TRACKING"]
        ExecutionSettings.c41_world_capabilities = None
        map_path, route_path = fixture_paths(mode)
        road_map = (RaceMap if mode == "race" else HDMap).from_path(map_path)
        route = GlobalPlan.from_file(route_path)
        route.race_mode = mode == "race"
        theta = float(np.arctan2(*(np.asarray(route.path[1]) - route.path[0])[::-1]))
        ego = EgoState(x=route.path[0][0], y=route.path[0][1], theta=theta)
        pm = PerceptionModel(map=road_map, ego_vehicle=ego)
        controller = plugin.BehaviorStanleyController()
        world = plugin.PlanningBasicSim(
            ego, pm=pm, map=road_map, setting=ExecutionSettingsSchema(c46_npc_control=False)
        )
        task = plugin.PlanningCoordinatorTask()
        local = ReferencePathPlanner(route, pm, PlanningSettings.model_copy(deep=True))
        tasks = [task]
        if mode == "race":
            tasks.append(plugin.RaceTrafficTask())
        ex = (executor or plugin.PlanningSyncExecuter)(
            pm,
            perception=PerceptionPipeline(pm),
            world=world,
            global_planner=plugin.GlobalPlannerDispatcher(road_map),
            local_planner=local,
            controller=controller,
            tasks=tasks,
        )
        return ex, task

    def drive(ex, steps):
        start = np.array([ex.ego_state.x, ex.ego_state.y])
        max_vehicles = 0
        for _ in range(steps):
            ex.step(sim_dt=0.05, perception_dt=0.05, control_dt=0.05, replan_dt=0.05)
            max_vehicles = max(max_vehicles, len(ex.pm.agent_vehicles))
        return float(
            np.linalg.norm(np.array([ex.ego_state.x, ex.ego_state.y]) - start)
        ), max_vehicles

    def brake(ex):
        # Test harness physically stops BasicSim; coordinator never teleports or
        # interprets pausing the UI as evidence that the vehicle is stationary.
        for _ in range(400):
            speed = ex.world.get_ego_state().velocity
            if abs(speed) < 0.001:
                break
            ex.world.control_ego_state(
                ControlCommand(
                    acceleration=-min(2.0, abs(speed) / 0.05) * np.sign(speed), steer=0.0
                ),
                dt=0.05,
            )
            ex.elapsed_sim_time += 0.05
        else:
            raise RuntimeError("BasicSim fixture did not stop")
        ex.pm.ego_vehicle.copy_from(ex.world.get_ego_state())
        ex.stop()

    def run(mode):
        ex, task = create_stack(mode)
        try:
            world = ex.world
            ex.task_runner.notify(ExecutionStarted())
            assert task.last_result["status"] == "selected", task.last_result
            start = np.array([ex.ego_state.x, ex.ego_state.y])
            agents = 0
            # Exercise the guard while actually moving, not at an arbitrary tick
            # where the race planner may already have stopped behind a rival.
            for moving_tick in range(160):
                _, count = drive(ex, 1)
                agents = max(agents, count)
                if (
                    abs(ex.world.get_ego_state().velocity) > 0.2
                    and abs(ex.pm.ego_vehicle.velocity) > 0.2
                ):
                    break
            else:
                raise RuntimeError("Fixture never reached the moving-request precondition")
            moving_speed = float(ex.world.get_ego_state().velocity)
            old = ex.local_planner
            task.request(ex, "road-rules" if mode == "road" else "race")
            moving_rejected = task.last_result["status"] == "rejected" and ex.local_planner is old
            _, count = drive(ex, 159 - moving_tick)
            agents = max(agents, count)
            first = float(np.linalg.norm(np.array([ex.ego_state.x, ex.ego_state.y]) - start))
            brake(ex)
            task.request(ex, "road-rules" if mode == "road" else "race")
            assert task.last_result["status"] == "selected", task.last_result
            assert ex.local_planner is not old and ex.world is world
            second, more_agents = drive(ex, 120)
            return dict(
                mode=mode,
                passed=moving_rejected and first > 5 and second > 5,
                first_displacement_m=first,
                resumed_displacement_m=second,
                moving_request_rejected=moving_rejected,
                moving_request_speed_mps=moving_speed,
                same_world=ex.world is world,
                max_scripted_rivals=max(agents, more_agents),
                transitions=list(task.history),
                sim_seconds=ex.elapsed_sim_time,
            )
        finally:
            ex.stop()

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--mode", choices=["road", "race", "both"], default="both")
        parser.add_argument("--repeat", type=int, default=3)
        parser.add_argument("--output", type=Path, required=True)
        args = parser.parse_args()
        if args.repeat < 1:
            parser.error("--repeat must be positive")
        if args.output.exists():
            parser.error("Output exists; choose another path")
        logging.basicConfig(level=logging.WARNING)
        modes = ["road", "race"] if args.mode == "both" else [args.mode]
        results = [dict(repeat=i + 1, **run(mode)) for mode in modes for i in range(args.repeat)]
        result = dict(
            passed=all(r["passed"] for r in results),
            scope="BasicSim coordination only; no sensor accuracy or arbitrary-traffic claim",
            runs=results,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        raise SystemExit(0 if result["passed"] else 1)

    if __name__ == "__main__":
        main()
