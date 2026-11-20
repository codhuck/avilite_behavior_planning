"""Repeatable forward parking checks using AVLite BasicSim and body geometry."""

# ruff: noqa: E402 -- resolve the sibling plugin before importing its strategies

# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import json
    import math
    import sys
    from pathlib import Path

    PLUGIN = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(PLUGIN))
    # Support this development workspace as well as an installed AVLite package.
    if (PLUGIN.parent / "avlite" / "__init__.py").is_file():
        sys.path.insert(0, str(PLUGIN.parent))

    from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
    from avlite.c20_planning.c21_planning_model import GlobalPlan
    from avlite.c40_execution.c49_settings import ExecutionSettingsSchema
    from avlite.c50_common.c54_trajectory_tracker import TrajectoryTracker
    from coordinator_demo import plugin
    from shapely.affinity import affine_transform
    from shapely.geometry import Polygon, box

    BasicSim = plugin.PlanningBasicSim
    ParkingController = plugin.ParkingController
    ParkingPlanner = plugin.ParkingPlanner
    ParkingRequest = plugin.ParkingRequest
    ParkingType = plugin.ParkingType
    angle_error = sys.modules[plugin.__name__ + ".parking.planner"].angle_error

    def run_scenario(
        name,
        mirror=1,
        rotation=0,
        occupied=False,
        intrusion=False,
        stale=False,
        parking_type="perpendicular_forward",
    ):
        parking_type = ParkingType(parking_type)
        c, s = math.cos(rotation), math.sin(rotation)
        transform = [c, -s * mirror, s, c * mirror, 0, 0]

        def point(x, y):
            return c * x - s * mirror * y, s * x + c * mirror * y

        def shape(polygon):
            return affine_transform(polygon, transform)

        parallel = parking_type == ParkingType.PARALLEL
        reverse = parking_type == ParkingType.PERPENDICULAR_REVERSE
        start, goal = (point(9, 3), point(0, 0)) if parallel else (point(-12, 0), point(0, 11))
        yaw = (
            rotation
            if parallel
            else angle_error(rotation + mirror * math.pi / 2 + (math.pi if reverse else 0))
        )
        ego = EgoState(x=start[0], y=start[1], theta=rotation + (math.pi if reverse else 0))
        pm = PerceptionModel(ego_vehicle=ego)
        global_plan = GlobalPlan(
            path=[start, goal], trajectory=TrajectoryTracker(path=[start, goal], velocity=[0, 0])
        )
        planner = ParkingPlanner(global_plan, pm)
        controller = ParkingController()
        world = BasicSim(ego, pm, controller, ExecutionSettingsSchema(c46_npc_control=False))
        bay = shape(box(-3.8, -1.5, 3.8, 1.5) if parallel else box(-1.6, 8, 1.6, 14))
        lot = shape(box(-14, -4, 15, 9) if parallel else box(-16, -5, 8, 15))
        request = ParkingRequest((*goal, yaw), bay, lot, parking_type)
        planner.request_parking(request)
        parked_cars = [shape(box(-4.5, 8, -2.5, 13)), shape(box(2.5, 8, 4.5, 13))]
        if parallel:
            parked_cars = [shape(box(-8.5, -1, -4, 1)), shape(box(4, -1, 8.5, 1))]
        records = []
        collisions = 0
        boundary_violations = 0
        for i in range(1600):
            time = i * 0.05
            obstacles = list(parked_cars)
            if occupied or (intrusion and 4 <= time < 10):
                obstacles.append(shape(box(-2, -1, 2, 1) if parallel else box(-1, 9, 1, 13)))
            pm.simulation_time = time
            if not stale or time < 2:
                planner.update_obstacles(obstacles, time)
            plan = planner.replan()
            world.control_ego_state(controller.control(ego, plan, control_dt=0.05), 0.05)
            body = Polygon(ego.get_bb_corners())
            collisions += int(any(body.intersects(obstacle) for obstacle in obstacles))
            boundary_violations += int(not lot.covers(body))
            records.append(
                dict(
                    time=time,
                    x=ego.x,
                    y=ego.y,
                    yaw=ego.theta,
                    speed=ego.velocity,
                    state=planner.state,
                    reason=planner.reason,
                )
            )
            if i > 100 and (occupied or stale):
                break
            if planner.state == "PARKED" and ego.velocity < 0.005:
                break
        distance = math.hypot(ego.x - goal[0], ego.y - goal[1])
        heading = abs(angle_error(ego.theta - yaw))
        if occupied or stale:
            outcome = planner.state == "PARKING_BLOCKED" and abs(ego.velocity) < 0.005
            if occupied:
                outcome = outcome and math.hypot(ego.x - start[0], ego.y - start[1]) < 1e-6
        else:
            outcome = (
                planner.state == "PARKED"
                and distance < 0.15
                and heading < math.radians(5)
                and bay.covers(Polygon(ego.get_bb_corners()))
            )
            if intrusion:
                outcome = outcome and any(
                    r["state"] == "PARKING_BLOCKED" and r["speed"] < 0.05
                    for r in records
                    if 4 <= r["time"] < 10
                )
        return dict(
            parking_type=parking_type.value,
            name=name,
            passed=bool(outcome and not collisions and not boundary_violations),
            collisions=collisions,
            boundary_violations=boundary_violations,
            position_error=distance,
            heading_error_degrees=math.degrees(heading),
            final_state=planner.state,
            seconds=records[-1]["time"],
            telemetry=records,
        )

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument(
            "--parking-type", choices=[t.value for t in ParkingType] + ["all"], default="all"
        )
        parser.add_argument("--output", type=Path, default=Path("logs/parking"))
        args = parser.parse_args()
        cases = [
            ("left", {}),
            ("right", {"mirror": -1}),
            ("rotated", {"rotation": 0.7}),
            ("occupied", {"occupied": True}),
            ("intrusion_resume", {"intrusion": True}),
            ("stale_observations", {"stale": True}),
        ]
        results = []
        args.output.mkdir(parents=True, exist_ok=True)
        modes = (
            list(ParkingType) if args.parking_type == "all" else [ParkingType(args.parking_type)]
        )
        for mode in modes:
            for name, kwargs in cases:
                result = run_scenario(name, parking_type=mode, **kwargs)
                results.append(result)
                print({k: v for k, v in result.items() if k != "telemetry"}, flush=True)
        (args.output / "results.json").write_text(json.dumps(results, indent=2))
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(8, 7))
        for result in results:
            if result["parking_type"] != "perpendicular_forward" or result["name"] not in (
                "left",
                "intrusion_resume",
            ):
                continue
            ax.plot(
                [r["x"] for r in result["telemetry"]],
                [r["y"] for r in result["telemetry"]],
                label=result["name"],
            )
        for polygon, color in [
            (box(-1.6, 8, 1.6, 14), "green"),
            (box(-4.5, 8, -2.5, 13), "gray"),
            (box(2.5, 8, 4.5, 13), "gray"),
        ]:
            x, y = polygon.exterior.xy
            ax.plot(x, y, color=color)
        ax.set_aspect("equal")
        ax.legend()
        ax.set(xlabel="x (m)", ylabel="y (m)", title="Forward bay parking — BasicSim")
        fig.savefig(args.output / "paths.png", dpi=150)
        plt.close(fig)
        return 0 if all(r["passed"] for r in results) else 1

    if __name__ == "__main__":
        raise SystemExit(main())
