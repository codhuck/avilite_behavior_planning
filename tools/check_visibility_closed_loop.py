"""Closed-loop BasicSim with an ideal planar range scanner, not CARLA LiDAR.

The scanner uses environment geometry to synthesize line-of-sight returns.
The planner receives only its free/occupied observation. This separates policy
and controller evaluation from the unresolved real 3D perception coverage gap.
"""

# ruff: noqa: E402
# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import importlib
    import json
    import math
    import sys
    import time
    import types
    from pathlib import Path

    import numpy as np
    from shapely.geometry import LineString, Point, Polygon, box
    from shapely.ops import unary_union

    ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(ROOT))
    package = types.ModuleType("visibility_experiment")
    package.__path__ = [str(Path(__file__).resolve().parents[1])]
    sys.modules[package.__name__] = package
    parking = importlib.import_module(package.__name__ + ".parking.planner")
    active = importlib.import_module(package.__name__ + ".parking.visibility")
    occupancy = importlib.import_module(package.__name__ + ".parking.occupancy")
    from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
    from avlite.c20_planning.c21_planning_model import GlobalPlan
    from avlite.c40_execution.c46_basic_sim import BasicSim
    from avlite.c40_execution.c49_settings import ExecutionSettingsSchema
    from avlite.c50_common.c54_trajectory_tracker import TrajectoryTracker

    class IdealPlanarScanner:
        def __init__(self, obstacles, radius=12):
            self.obstacles = unary_union(obstacles)
            self.radius = radius
            self.cache = None

        def observe(self, lidar, pose, area, now, ground_z):
            origin = Point(pose.x, pose.y)
            # Cache geometry only; each call models a fresh acquisition of this
            # static synthetic environment. No real/simulated LiDAR cache is reused.
            key = (round(pose.x, 3), round(pose.y, 3))
            if self.cache is None or key != self.cache[0]:
                endpoints = []
                returns = []
                for angle in np.linspace(0, 2 * math.pi, 720, endpoint=False):
                    delta = np.array([math.cos(angle), math.sin(angle)])
                    end = np.array([pose.x, pose.y]) + self.radius * delta
                    hit = LineString([(pose.x, pose.y), end]).intersection(self.obstacles)
                    distance = self.radius if hit.is_empty else max(0, origin.distance(hit) - 0.02)
                    endpoint = np.array([pose.x, pose.y]) + distance * delta
                    endpoints.append(endpoint)
                    if not hit.is_empty:
                        returns.append(Point(endpoint).buffer(0.08))
                free = Polygon(endpoints).intersection(area).difference(self.obstacles)
                occupied = unary_union(returns)
                self.cache = (key, free, occupied)
            return occupancy.ParkingOccupancy(
                area.difference(self.cache[1]), self.cache[1], now, "observed", self.cache[2]
            )

    def run(policy, start=None, parking_type="perpendicular_forward", wall_budget=180):
        parallel = parking_type == "parallel"
        reverse = parking_type == "perpendicular_reverse"
        start = start or ((9, 3, 0) if parallel else (-9, 0, math.pi if reverse else 0))
        ego = EgoState(x=start[0], y=start[1], theta=start[2])
        pm = PerceptionModel(ego_vehicle=ego)
        goal = (0, 0, 0) if parallel else (0, 11, -math.pi / 2 if reverse else math.pi / 2)
        path = [tuple(start[:2]), goal[:2]]
        route = GlobalPlan(path=path, trajectory=TrajectoryTracker(path=path, velocity=[0, 0]))
        cls = active.VisibilityParkingPlanner if policy == "active" else parking.ParkingPlanner
        planner = cls(route, pm)
        controller = parking.ParkingController()
        world = BasicSim(ego, pm, controller, ExecutionSettingsSchema(c46_npc_control=False))
        obstacles = [box(-4.5, 8, -2.5, 13), box(2.5, 8, 4.5, 13)]
        if parallel:
            obstacles = [box(-8.5, -1, -4, 1), box(4, -1, 8.5, 1)]
        planner.sensor_occupancy = IdealPlanarScanner(obstacles)
        planner.sensor_ground_z = 0
        area = box(-14, -4, 15, 9) if parallel else box(-16, -5, 8, 15)
        bay = box(-3.8, -1.5, 3.8, 1.5) if parallel else box(-1.6, 8, 1.6, 14)
        planner.request_parking(parking.ParkingRequest(goal, bay, area, parking_type))
        rows = []
        collisions = 0
        started = time.monotonic()
        termination = "simulation_budget"
        for step in range(4000):
            if time.monotonic() - started > wall_budget:
                termination = "wall_budget"
                break
            now = step * 0.05
            pm.simulation_time = now
            sensors = types.SimpleNamespace(stamp=now, lidar=None)
            plan = planner.replan(sensors=sensors)
            world.control_ego_state(controller.control(ego, plan, 0.05), 0.05)
            footprint = Polygon(ego.get_bb_corners())
            collisions += int(
                any(footprint.intersects(p) for p in obstacles) or not area.covers(footprint)
            )
            rows.append(
                dict(
                    t=now,
                    x=ego.x,
                    y=ego.y,
                    yaw=ego.theta,
                    v=ego.velocity,
                    state=planner.state,
                    reason=planner.reason,
                )
            )
            if planner.state == "PARKED":
                termination = "parked"
                break
            if step > 100 and all(abs(r["v"]) < 0.005 for r in rows[-100:]):
                termination = "stationary"
                break
        return dict(
            policy=policy,
            termination=termination,
            wall_seconds=time.monotonic() - started,
            parking_type=parking_type,
            parked=planner.state == "PARKED",
            collisions=collisions,
            goal_error=math.hypot(ego.x - goal[0], ego.y - goal[1]),
            state=planner.state,
            reason=planner.reason,
            rejections=getattr(getattr(planner, "arbiter", None), "rejections", {}),
            base_reason=getattr(planner, "base_reason", planner.reason),
            segments=[
                (direction, points[0].tolist(), points[-1].tolist())
                for direction, points in getattr(planner, "_segments", [])
            ],
            telemetry=rows,
        )

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--start", type=float, nargs=3)
        parser.add_argument(
            "--wall-budget",
            type=float,
            default=180,
            help="Wall seconds per policy, checked between planner calls",
        )
        parser.add_argument(
            "--parking-type",
            choices=[m.value for m in parking.ParkingType],
            default="perpendicular_forward",
        )
        parser.add_argument(
            "--output", type=Path, default=ROOT / "logs/visibility_closed_loop_segments"
        )
        args = parser.parse_args()
        if not math.isfinite(args.wall_budget) or args.wall_budget <= 0:
            parser.error("--wall-budget must be positive and finite")
        output = args.output
        output.mkdir(parents=True, exist_ok=True)
        for policy in ("conservative", "active"):
            result = run(policy, args.start, args.parking_type, args.wall_budget)
            (output / (policy + ".json")).write_text(json.dumps(result, indent=2) + "\n")
            print({k: v for k, v in result.items() if k != "telemetry"}, flush=True)

    if __name__ == "__main__":
        main()
