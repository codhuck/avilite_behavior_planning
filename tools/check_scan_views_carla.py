"""Prospective one-step view-selection audit using acquired CARLA LiDAR.

Static counterfactual endpoint acquisitions are NOT executed parking trials.
Ranking sees only the common initial observation. Future scans score selections.
"""

# ruff: noqa: E402
# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import hashlib
    import json
    import math
    import queue
    import shutil
    import sys
    import time
    from pathlib import Path

    from _sources import source_files

    PLUGIN = Path(__file__).resolve().parents[1]
    sys.path[:0] = [str(PLUGIN), str(PLUGIN.parent)]
    import carla
    import numpy as np
    import shapely
    from _bootstrap import load_plugin
    from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
    from avlite.c20_planning.c21_planning_model import GlobalPlan
    from avlite.c50_common.c52_world_sensor_datatypes import Lidar
    from avlite.c50_common.c54_trajectory_tracker import TrajectoryTracker
    from check_parking_carla import PAD
    from shapely.geometry import GeometryCollection, box

    load_plugin()

    from avlite_coordination_demo.parking.dispatch import (
        Maneuver,
        VisibilityDispatcher,
        VisibilitySnapshot,
        expected_gain,
    )
    from avlite_coordination_demo.parking.occupancy import FusedLidarParkingOccupancy
    from avlite_coordination_demo.parking.planner import ParkingPlanner, ParkingRequest
    from avlite_coordination_demo.parking.scan import ScanVisibility
    from avlite_coordination_demo.parking.search import rollout
    from avlite_coordination_demo.parking.view import GroundLidar, TaskVisibility, beam_gain

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--output", type=Path, required=True)
        args = parser.parse_args()
        if args.output.exists():
            raise ValueError("Preserve existing experiments; use a new output directory")
        args.output.mkdir(parents=True)
        scenes = [
            dict(id=f"{mode}_{seed}_{offset:+.2f}", mode=mode, seed=seed, offset=offset)
            for mode in ["perpendicular_forward", "perpendicular_reverse", "parallel"]
            for offset in [-0.3, 0.3, 0.7]
            for seed in [7501, 7502]
        ]
        protocol = {
            "scope": __doc__,
            "scenes": scenes,
            "methods": ["none", "bay_area", "task", "task_beams", "task_scan"],
            "hypothesis": "Frozen finite-ray raster scoring with stationary control improves acquired task area per motion cost over bay-area ranking on new perturbed layouts.",
            "primary": "Paired acquired weighted gain / (1 + view length); fallback uses measured fresh stationary gain; no-proposal cases retained separately.",
            "limits": "Eighteen new configurations: three types x three offsets/layouts x two new seeds; static actors; same pad; no completed parking claims.",
            "scan_model": {
                "azimuth_samples": 250,
                "phases": 4,
                "resolution": 0.25,
                "min_advantage": 0.01,
                "body_bottom": 0.15,
                "body_top": 1.6,
            },
            "extra_occluders": "offset -.3: (46.5,4,pi/2) perpendicular or (52,-3.8,0) parallel; offset .3: (57.5,5,0) perpendicular or (52,-3.8,0) parallel; offset .7: none",
            "spacing": 0.4,
            "horizon": 8,
            "decay": 4,
            "candidates": "Straight and curvature +/-0.2; +/-0.5 and +/-1 metre; speed .1",
            "sources": {
                str(p.relative_to(PLUGIN)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in [*source_files(PLUGIN), Path(__file__)]
            },
        }
        (args.output / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
        snapshot_dir = args.output / "source_snapshot"
        snapshot_dir.mkdir()
        for relative in protocol["sources"]:
            target = snapshot_dir / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(PLUGIN / relative, target)
        client = carla.Client("127.0.0.1", 2000)
        client.set_timeout(60)
        old = client.get_world()
        if any(a.type_id.startswith(("vehicle.", "sensor.", "walker.")) for a in old.get_actors()):
            raise RuntimeError("Exclusive server required")
        original_map = old.get_map().name.rsplit("/", 1)[-1]
        actors = []
        results = []
        try:
            world = client.generate_opendrive_world(PAD)
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.1
            settings.no_rendering_mode = True
            world.apply_settings(settings)
            for scenario in scenes:
                start_time = time.monotonic()
                directory = args.output / scenario["id"]
                directory.mkdir()
                mode = scenario["mode"]
                parallel = mode == "parallel"
                reverse = mode == "perpendicular_reverse"
                start = (61.6, 3.0, 0) if parallel else (41.8, -0.3, math.pi if reverse else 0)
                goal = (
                    (52, 0, 0) if parallel else (52, 11, -math.pi / 2 if reverse else math.pi / 2)
                )
                bp = world.get_blueprint_library().find("vehicle.audi.a2")

                def spawn(p):
                    a = world.spawn_actor(
                        bp,
                        carla.Transform(
                            carla.Location(x=p[0], y=-p[1], z=0.3),
                            carla.Rotation(yaw=-math.degrees(p[2])),
                        ),
                    )
                    actors.append(a)
                    a.apply_control(carla.VehicleControl(hand_brake=True))
                    return a

                vehicle = spawn(start)
                neighbors = (
                    [(45.75, scenario["offset"], 0), (58.25, -scenario["offset"], 0)]
                    if parallel
                    else [
                        (48.5 + scenario["offset"], 11, math.pi / 2),
                        (55.5 - scenario["offset"], 11, math.pi / 2),
                    ]
                )
                if scenario["offset"] != 0.7:
                    neighbors.append(
                        (52, -3.8, 0)
                        if parallel
                        else ((46.5, 4, math.pi / 2) if scenario["offset"] < 0 else (57.5, 5, 0))
                    )
                row_layout = [list(p) for p in neighbors]
                for p in neighbors:
                    spawn(p)
                for _ in range(30):
                    world.tick()
                for a in actors:
                    a.set_simulate_physics(False)
                transform = vehicle.get_transform()
                z = transform.location.z
                length, width = vehicle.bounding_box.extent.x * 2, vehicle.bounding_box.extent.y * 2
                area = box(30, -6, 70, 18)
                bay = box(48.2, -1.5, 55.8, 1.5) if parallel else box(50.4, 8, 53.6, 14)
                mounts = (
                    [(0, 0, 2.3)]
                    + [
                        (x, y, 0.55)
                        for x in (-length / 2 - 0.05, length / 2 + 0.05)
                        for y in (-width / 2 - 0.05, width / 2 + 0.05)
                    ]
                    + [
                        (length / 2 + 0.05, 0, 0.55),
                        (-length / 2 - 0.05, 0, 0.55),
                        (0, width / 2 + 0.05, 0.55),
                        (0, -width / 2 - 0.05, 0.55),
                    ]
                )
                devices = []
                for i, (x, y, h) in enumerate(mounts):
                    lidar_bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
                    for key, value in dict(
                        channels=64,
                        points_per_second=160000,
                        rotation_frequency=10,
                        range=30 if i == 0 else 15,
                        upper_fov=10,
                        lower_fov=-45 if i == 0 else -85,
                        dropoff_general_rate=0.02,
                        noise_stddev=0.01,
                        noise_seed=scenario["seed"] + 997 * i,
                    ).items():
                        lidar_bp.set_attribute(key, str(value))
                    sensor = world.spawn_actor(
                        lidar_bp, carla.Transform(carla.Location(x=x, y=-y, z=h)), attach_to=vehicle
                    )
                    actors.append(sensor)
                    messages = queue.Queue()
                    sensor.listen(messages.put)
                    mount = np.eye(4)
                    mount[:3, 3] = (x, y, h)
                    devices.append((messages, mount))
                provider = FusedLidarParkingOccupancy(minimum_support=1, hazard_support_radius=2)

                def capture(pose, name):
                    vehicle.set_transform(
                        carla.Transform(
                            carla.Location(x=float(pose[0]), y=float(-pose[1]), z=z),
                            carla.Rotation(yaw=float(-math.degrees(pose[2]))),
                        )
                    )
                    world.tick()
                    frame = world.tick()
                    actual = vehicle.get_transform()
                    measured_pose = np.array(
                        [actual.location.x, -actual.location.y, -math.radians(actual.rotation.yaw)]
                    )
                    yaw_error = math.atan2(
                        math.sin(measured_pose[2] - pose[2]), math.cos(measured_pose[2] - pose[2])
                    )
                    if (
                        np.linalg.norm(measured_pose[:2] - pose[:2]) > 0.001
                        or abs(yaw_error) > 1e-4
                    ):
                        raise RuntimeError("Capture pose differs from requested endpoint")
                    clouds = []
                    for index, (messages, mount) in enumerate(devices):
                        while True:
                            message = messages.get(timeout=10)
                            if message.frame == frame:
                                break
                            if message.frame > frame:
                                raise RuntimeError("Frame alignment error")
                        points = (
                            np.frombuffer(message.raw_data, dtype=np.float32).reshape(-1, 4).copy()
                        )
                        points[:, 1] *= -1
                        np.savez_compressed(
                            directory / f"{name}_cloud_{index}.npz",
                            points=points,
                            stamp=message.timestamp,
                            pose=np.r_[measured_pose[:2], actual.location.z, measured_pose[2]],
                            requested_pose=np.r_[pose[:2], z, pose[2]],
                            frame=message.frame,
                            base_to_sensor=mount,
                        )
                        clouds.append(
                            Lidar(
                                points=points,
                                stamp=message.timestamp,
                                base_to_sensor=mount,
                                sensor_name=f"lidar_{index}",
                            )
                        )
                    ego = EgoState(
                        x=float(pose[0]),
                        y=float(pose[1]),
                        z=z,
                        theta=float(pose[2]),
                        length=length,
                        width=width,
                    )
                    return ego, provider.observe(clouds, ego, area, clouds[0].stamp, 0)

                ego, initial = capture(start, "initial")
                snapshot = VisibilitySnapshot(
                    initial.traversable,
                    initial.occupied if initial.occupied is not None else GeometryCollection(),
                    initial.stamp,
                    initial.sources,
                )
                route = GlobalPlan(
                    path=[start[:2], goal[:2]],
                    trajectory=TrajectoryTracker(path=[start[:2], goal[:2]], velocity=[0, 0]),
                )
                planner = ParkingPlanner(route, PerceptionModel(ego_vehicle=ego))
                planner.rear_offset = 1.25325
                planner.allow_multi_maneuver = True
                planner.deterministic_search = True
                planner.search_expansions = 1500
                planner.refine_search_endpoint = True
                planner.obstacles = [snapshot.occupied]
                planner.request_parking(ParkingRequest(goal, bay, area, mode))
                path = planner._generate()
                row = {
                    "scene": scenario,
                    "proposal_available": path is not None,
                    "neighbors": row_layout,
                    "candidates": [],
                    "choices": {},
                    "wall_seconds": 0,
                }
                if path is not None:
                    task = TaskVisibility(snapshot, path, width, length)
                    baseline = expected_gain(start, snapshot, bay)
                    initial_free = initial.free
                    sensors = [
                        GroundLidar(
                            x,
                            y,
                            z + h,
                            tuple(np.linspace(-45 if i == 0 else -85, 10, 64)),
                            30 if i == 0 else 15,
                        )
                        for i, (x, y, h) in enumerate(mounts)
                    ]
                    scan = ScanVisibility(task, sensors, body_length=length, body_width=width)
                    row["predicted_stationary_gain"] = scan.evaluate(start).expected_area
                    _, stationary = capture(start, "stationary")
                    seen = shapely.intersects_xy(
                        stationary.free, task.points[:, 0], task.points[:, 1]
                    )
                    stationary_gain = float(task.weights[seen].sum() * task.spacing**2)
                    row["stationary_gain"] = stationary_gain
                    for direction in [1, -1]:
                        for distance in [0.5, 1.0]:
                            for curvature in [0.0, -0.2, 0.2]:
                                rear = (
                                    start[0] - planner.rear_offset * math.cos(start[2]),
                                    start[1] - planner.rear_offset * math.sin(start[2]),
                                    start[2],
                                )
                                poses = rollout(rear, direction, curvature, distance)
                                poses[:, :2] += planner.rear_offset * np.column_stack(
                                    (np.cos(poses[:, 2]), np.sin(poses[:, 2]))
                                )
                                name = f"view_{direction}_{distance}_{curvature}"
                                candidate = Maneuver(
                                    name,
                                    "observe",
                                    mode,
                                    poses,
                                    direction,
                                    0.1,
                                    1,
                                    rear_offset=planner.rear_offset,
                                )
                                gate = VisibilityDispatcher()
                                safe = gate.admissible(
                                    candidate,
                                    snapshot,
                                    snapshot.stamp,
                                    np.asarray(start),
                                    0,
                                    length,
                                    width,
                                )
                                item = {
                                    "name": name,
                                    "admissible": bool(safe),
                                    "rejection": gate.rejections.get(name),
                                    "endpoint": poses[-1].tolist(),
                                }
                                if safe:
                                    cost = 1 + float(
                                        np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1).sum()
                                    )
                                    item["cost"] = cost
                                    item["task_score"] = (
                                        task.evaluate(start, poses[-1]).weighted_gain / cost
                                    )
                                    item["beam_score"] = (
                                        beam_gain(task, start, poses[-1], sensors) / cost
                                    )
                                    item["bay_score"] = (
                                        max(0.0, expected_gain(poses[-1], snapshot, bay) - baseline)
                                        / cost
                                    )
                                    item["scan_score"] = scan.advantage(start, poses[-1], cost - 1)
                                    item["scan_phase_range"] = [
                                        scan.evaluate(poses[-1]).phase_min,
                                        scan.evaluate(poses[-1]).phase_max,
                                    ]
                                    _, observation = capture(poses[-1], name)
                                    known = shapely.intersects_xy(
                                        observation.free, task.points[:, 0], task.points[:, 1]
                                    )
                                    item["acquired_weighted_gain"] = (
                                        float(task.weights[known].sum() * task.spacing**2) / cost
                                    )
                                    item["acquired_free_area"] = observation.free.difference(
                                        initial_free
                                    ).area
                                row["candidates"].append(item)
                    for method in ["none", "bay_area", "task", "task_beams", "task_scan"]:
                        key = {
                            "task": "task_score",
                            "task_beams": "beam_score",
                            "task_scan": "scan_score",
                        }.get(method, "bay_score")
                        choices = [
                            r
                            for r in row["candidates"]
                            if r["admissible"]
                            and r[key] >= (0.01 if method == "task_scan" else 1e-12)
                        ]
                        winner = (
                            max(choices, key=lambda r: r[key])
                            if method != "none" and choices
                            else None
                        )
                        row["choices"][method] = {
                            "name": winner["name"] if winner else "stay",
                            "gain": winner["acquired_weighted_gain"] if winner else stationary_gain,
                            "improvement_over_stationary": winner["acquired_weighted_gain"]
                            - stationary_gain
                            if winner
                            else 0.0,
                        }
                    row["task_samples"] = len(task.points)
                    np.save(directory / "task_path.npy", path)
                if path is None:
                    row["choices"] = {
                        method: {
                            "name": "no_proposal",
                            "gain": None,
                            "improvement_over_stationary": None,
                        }
                        for method in protocol["methods"]
                    }
                row["wall_seconds"] = time.monotonic() - start_time
                results.append(row)
                (args.output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
                print(scenario["id"], row["proposal_available"], row["choices"], flush=True)
                for a in reversed(actors):
                    if a.type_id.startswith("sensor."):
                        a.stop()
                    a.destroy()
                actors.clear()
                world.tick()
        finally:
            for a in reversed(actors):
                try:
                    a.destroy()
                except RuntimeError:
                    pass
            client.load_world(original_map)

    if __name__ == "__main__":
        main()
