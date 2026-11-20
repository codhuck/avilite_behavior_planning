"""Measure parking observability using CARLA ray-cast LiDAR on a flat pad.

Exclusive idle server required. Ground-truth actor boxes are evaluation labels
only; the occupancy provider receives point clouds, calibration and ego pose.
This is a perception probe, not a completed parking-drive acceptance test.
"""

# ruff: noqa: E402 -- standalone workspace tool
# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import json
    import queue
    import sys
    from pathlib import Path

    import numpy as np

    PLUGIN = Path(__file__).resolve().parents[1]
    sys.path[:0] = [str(PLUGIN), str(PLUGIN.parent)]
    import carla
    from avlite.c10_perception.c11_perception_model import EgoState
    from avlite.c50_common.c52_world_sensor_datatypes import Lidar
    from check_parking_carla import PAD
    from shapely.geometry import box

    from parking_occupancy import LidarParkingOccupancy

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--channels", type=int, default=64)
        parser.add_argument("--points-per-second", type=int, default=300000)
        parser.add_argument("--output", type=Path, default=Path("logs/parking_lidar_probe"))
        args = parser.parse_args()
        client = carla.Client("localhost", 2000)
        client.set_timeout(60)
        original = client.get_world()
        if original.get_actors().filter("vehicle.*"):
            raise RuntimeError("Use an idle server with no vehicles")
        name = original.get_map().name.split("/")[-1]
        if "OpenDrive" in name:
            name = "Town10HD_Opt"
        actors = []
        results = []
        try:
            world = client.generate_opendrive_world(PAD)
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            settings.no_rendering_mode = True
            world.apply_settings(settings)
            bp = world.get_blueprint_library().find("vehicle.audi.a2")
            ego = world.spawn_actor(bp, carla.Transform(carla.Location(x=40, z=0.3)))
            actors.append(ego)
            obstacle = world.spawn_actor(bp, carla.Transform(carla.Location(x=48, z=0.3)))
            actors.append(obstacle)
            for actor in actors:
                actor.apply_control(carla.VehicleControl(hand_brake=True))
            for _ in range(30):
                world.tick()
            lidar_bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
            for key, value in dict(
                channels=args.channels,
                points_per_second=args.points_per_second,
                rotation_frequency=20,
                range=30,
                upper_fov=10,
                lower_fov=-45,
                dropoff_general_rate=0,
                noise_stddev=0,
            ).items():
                lidar_bp.set_attribute(key, str(value))
            lidar_actor = world.spawn_actor(
                lidar_bp, carla.Transform(carla.Location(z=2.3)), attach_to=ego
            )
            actors.append(lidar_actor)
            messages = queue.Queue()
            lidar_actor.listen(messages.put)
            area = box(34, -8, 54, 8)
            provider = LidarParkingOccupancy()
            for _ in range(5):
                frame = world.tick()
                while True:
                    measurement = messages.get(timeout=10)
                    if measurement.frame == frame:
                        break
                points = np.frombuffer(measurement.raw_data, dtype=np.float32).reshape(-1, 4).copy()
                points[:, 1] *= -1
                mount = np.eye(4)
                mount[2, 3] = 2.3
                snapshot = Lidar(points=points, stamp=measurement.timestamp, base_to_sensor=mount)
                t = ego.get_transform()
                pose = EgoState(x=t.location.x, y=-t.location.y, z=t.location.z, theta=0)
                result = provider.observe(snapshot, pose, area, measurement.timestamp, 0)
                # Fixed surveyed test bay and separate evaluator label.
                bay = box(43, 2, 49, 5)
                extent = obstacle.bounding_box.extent
                obstacle_label = box(48 - extent.x, -extent.y, 48 + extent.x, extent.y)
                results.append(
                    dict(
                        points=len(points),
                        status=result.status,
                        free_fraction=result.free.area / area.area,
                        bay_observed_free=result.permits(bay),
                        obstacle_free_fraction=result.free.intersection(obstacle_label).area
                        / obstacle_label.area,
                    )
                )
            output = args.output
            output.mkdir(parents=True, exist_ok=True)
            (output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
            print(json.dumps(results, indent=2))
        finally:
            for actor in reversed(actors):
                if actor.is_alive:
                    actor.destroy()
            client.load_world(name)

    if __name__ == "__main__":
        main()
