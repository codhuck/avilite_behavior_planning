"""Drive complete parking missions in BasicSim and record body-level checks."""

# ruff: noqa: E402 -- load the selected checkout before its registered strategies
if not __name__.startswith("avlite.plugins."):
    import argparse
    import json
    import math
    import sys
    import tempfile
    from pathlib import Path

    import yaml

    ROOT = Path(__file__).resolve().parents[1]
    if (ROOT.parent / "avlite/__init__.py").is_file():
        sys.path.insert(0, str(ROOT.parent))
    from avlite.c10_perception.c11_perception_model import AgentState, EgoState, PerceptionModel
    from avlite.c20_planning.c21_planning_model import GlobalPlan
    from avlite.c20_planning.c26_local_path_planners import ReferencePathPlanner
    from avlite.c40_execution.c49_settings import ExecutionSettings, ExecutionSettingsSchema
    from coordinator_demo import plugin
    from prepare_mission_profiles import prepare
    from shapely.geometry import Polygon

    def create_stack(profile_path):
        profile = yaml.safe_load(Path(profile_path).read_text())
        config = profile["c40_execution"]
        for key, value in profile["plugins"]["avlite_behavior_hdmap"].items():
            setattr(plugin.PluginSettings, key, value)
        plugin.PluginSettings.parking_lidar_occupancy = False
        plugin.PluginSettings.parking_active_observation = False
        ExecutionSettings.c41_world_stack_capabilities = ["LOCALIZATION", "DETECTION"]
        ExecutionSettings.c41_world_capabilities = None
        x, y, theta = config["c40_start_pose"]
        ego = EgoState(x=x, y=y, theta=theta)
        pm = PerceptionModel(ego_vehicle=ego)
        route = GlobalPlan.from_file(config["c40_global_trajectory"])
        controller = plugin.BehaviorStanleyController()
        world = plugin.PlanningBasicSim(
            ego,
            pm=pm,
            controller=controller,
            setting=ExecutionSettingsSchema(c46_npc_control=False),
        )
        mission = plugin.ParkingMissionTask()
        status = plugin.DecisionStatusTask()
        ex = plugin.PlanningSyncExecuter(
            pm,
            world=world,
            global_planner=plugin.SurveyedRoutePlanner(route=route),
            local_planner=ReferencePathPlanner(route, pm),
            controller=controller,
            tasks=[plugin.ParkingDemoTrafficTask(), mission, status],
        )
        return ex, mission, status

    def run(profile_path, case="clear"):
        ex, mission, status = create_stack(profile_path)
        scene = json.loads(Path(plugin.PluginSettings.parking_scene_file).read_text())
        space = scene["spaces"]["demo_bay"]
        goal = space["goals"][space["types"][0]]
        blocking = None
        if case in {"occupied", "free_later"}:
            blocking = AgentState(x=goal[0], y=goal[1], theta=goal[2])
            ex.world.pm.add_agent_vehicle(blocking)
        collisions = boundary_violations = 0
        start = (ex.ego_state.x, ex.ego_state.y)
        approach_distance = 0.0
        parked_at = None
        records = []
        area = Polygon(scene["drivable"])
        try:
            for _ in range(2400):
                ex.step(
                    sim_dt=0.05, perception_dt=0.05, control_dt=0.05, replan_dt=0.05, pace_sim=True
                )
                now = ex.elapsed_sim_time
                if (
                    case == "free_later"
                    and mission.state == "SELECT_SPACE"
                    and now - mission._state_since > 2
                ):
                    ex.world.pm.agent_vehicles[:] = [
                        a for a in ex.world.pm.agent_vehicles if a is not blocking
                    ]
                if case in {"unknown", "stale"} and now >= 0.1:
                    plugin.PluginSettings.parking_simulator_obstacles = False
                    ex.pm.parking_observation = (
                        None
                        if case == "unknown"
                        else plugin.ParkingObservation(
                            tuple(Polygon(a.get_bb_corners()) for a in ex.world.pm.agent_vehicles),
                            0.0,
                        )
                    )
                if mission.state in {"APPROACH", "SETTLING"}:
                    approach_distance = max(
                        approach_distance,
                        math.hypot(ex.ego_state.x - start[0], ex.ego_state.y - start[1]),
                    )
                body = Polygon(ex.ego_state.get_bb_corners())
                collisions += int(
                    any(
                        body.intersects(Polygon(a.get_bb_corners()))
                        for a in ex.world.pm.agent_vehicles
                    )
                )
                boundary_violations += int(not area.covers(body))
                records.append(
                    dict(
                        time=now,
                        x=ex.ego_state.x,
                        y=ex.ego_state.y,
                        speed=ex.ego_state.velocity,
                        state=mission.state,
                        reason=mission.reason,
                    )
                )
                if mission.state in {"COMPLETE", "BLOCKED"}:
                    parked_at = parked_at if parked_at is not None else now
                    if now - parked_at >= 1.0:
                        break
            expected = "BLOCKED" if case in {"occupied", "unknown", "stale"} else "COMPLETE"
            passed = (
                mission.state == expected
                and approach_distance > 7
                and abs(ex.ego_state.velocity) <= 0.02
                and collisions == boundary_violations == 0
            )
            return dict(
                case=case,
                mode=space["types"][0],
                passed=passed,
                state=mission.state,
                reason=mission.reason,
                collisions=collisions,
                boundary_violations=boundary_violations,
                approach_distance_m=approach_distance,
                simulation_seconds=ex.elapsed_sim_time,
                history=mission.history,
                telemetry=records,
                decision=status.snapshot,
            )
        finally:
            ex.stop()

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--output", required=True, type=Path)
        args = parser.parse_args()
        if args.output.exists():
            parser.error("Output exists; choose a new file")
        results = []
        with tempfile.TemporaryDirectory(prefix="avlite-missions-") as tmp:
            for profile in prepare(tmp):
                for case in ("clear", "occupied", "free_later", "unknown", "stale"):
                    result = run(profile, case)
                    results.append(result)
                    print(
                        json.dumps(
                            {
                                k: v
                                for k, v in result.items()
                                if k not in {"history", "telemetry", "decision"}
                            }
                        ),
                        flush=True,
                    )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results, indent=2, allow_nan=False) + "\n")
        raise SystemExit(0 if all(r["passed"] for r in results) else 1)

    if __name__ == "__main__":
        main()
