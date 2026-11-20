"""Physics check on a generated flat CARLA test pad (exclusive server required)."""

# ruff: noqa: E402 -- workspace imports after locating the plugin
# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import hashlib
    import importlib
    import json
    import math
    import queue
    import sys
    import time
    import types
    from dataclasses import asdict
    from pathlib import Path

    PLUGIN = Path(__file__).resolve().parents[1]
    sys.path[:0] = [str(PLUGIN), str(PLUGIN.parent)]
    import carla
    import numpy as np
    from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
    from avlite.c20_planning.c21_planning_model import GlobalPlan
    from avlite.c50_common.c52_world_sensor_datatypes import Lidar
    from avlite.c50_common.c54_trajectory_tracker import TrajectoryTracker
    from parking_ablation import POLICIES, fixed_crossing
    from parking_pedestrian import crossing_origin, valid_walker_motion
    from shapely.affinity import affine_transform
    from shapely.geometry import Polygon, box

    from parking import ParkingController, ParkingPlanner, ParkingRequest, ParkingType, angle_error

    PAD = """<?xml version="1.0" standalone="yes"?><OpenDRIVE><header revMajor="1" revMinor="4" name="parking-pad" version="1" north="30" south="-30" east="120" west="0"/><road name="Parking test pad" length="120" id="1" junction="-1"><link/><type s="0" type="town"/><planView><geometry s="0" x="0" y="0" hdg="0" length="120"><line/></geometry></planView><elevationProfile><elevation s="0" a="0" b="0" c="0" d="0"/></elevationProfile><lanes><laneSection s="0"><left><lane id="1" type="driving" level="false"><link/><width sOffset="0" a="30" b="0" c="0" d="0"/></lane></left><center><lane id="0" type="none" level="false"><roadMark sOffset="0" type="none" weight="standard" color="standard" width="0"/></lane></center><right><lane id="-1" type="driving" level="false"><link/><width sOffset="0" a="30" b="0" c="0" d="0"/></lane></right></laneSection></lanes></road></OpenDRIVE>"""

    def run(
        world,
        mode,
        repeat=0,
        sensor=False,
        active=False,
        channels=64,
        output=None,
        rig=False,
        hazard_support_radius=2.0,
        start_override=None,
        rig_layout="corners",
        blackout_start=None,
        blackout_frames=20,
        dt=0.05,
        scenario=None,
    ):
        scenario = scenario or {}
        policy_name = scenario.get("ablation_policy")
        policy = POLICIES[policy_name] if policy_name else None
        if policy and not (sensor and active and rig and rig_layout == "surround"):
            raise ValueError("Ablations require --sensor --active --rig --rig-layout surround")
        wall_start = time.monotonic()
        parallel = mode == ParkingType.PARALLEL
        reverse = mode == ParkingType.PERPENDICULAR_REVERSE
        start = (61, 3, 0) if parallel else (40, 0, math.pi if reverse else 0)
        if start_override is not None:
            start = tuple(start_override)
        goal = (52, 0, 0) if parallel else (52, 11, -math.pi / 2 if reverse else math.pi / 2)
        if repeat:
            start = (start[0], start[1] + 0.1, start[2] + math.radians(1))
        delta = scenario.get("start_delta", [0, 0, 0])
        start = tuple(a + b for a, b in zip(start, delta))
        rotation = math.radians(scenario.get("rotation_degrees", 0))
        shift = scenario.get("translation", [0, 0])
        cosine, sine = math.cos(rotation), math.sin(rotation)
        matrix = [
            cosine,
            -sine,
            sine,
            cosine,
            52 - 52 * cosine + 3 * sine + shift[0],
            3 - 52 * sine - 3 * cosine + shift[1],
        ]

        def transform(pose):
            x, y, yaw = pose
            return (
                cosine * x - sine * y + matrix[4],
                sine * x + cosine * y + matrix[5],
                yaw + rotation,
            )

        start, goal = transform(start), transform(goal)
        blueprint = world.get_blueprint_library().find("vehicle.audi.a2")
        vehicle = world.spawn_actor(
            blueprint,
            carla.Transform(
                carla.Location(x=start[0], y=-start[1], z=0.3),
                carla.Rotation(yaw=-math.degrees(start[2])),
            ),
        )
        actors = [vehicle]
        hits = []
        contact_details = []
        try:
            collision = world.spawn_actor(
                world.get_blueprint_library().find("sensor.other.collision"),
                carla.Transform(),
                attach_to=vehicle,
            )
            actors.append(collision)

            def record_collision(event):
                hits.append(event.frame)
                contact_details.append(
                    {
                        "frame": event.frame,
                        "timestamp": event.timestamp,
                        "other_actor_id": event.other_actor.id,
                        "other_actor_type": event.other_actor.type_id,
                        "normal_impulse": [
                            event.normal_impulse.x,
                            event.normal_impulse.y,
                            event.normal_impulse.z,
                        ],
                    }
                )

            collision.listen(record_collision)
            positions = (
                [(45.75, 0, 0), (58.25, 0, 0)]
                if parallel
                else [(48.5, 11, math.pi / 2), (55.5, 11, math.pi / 2)]
            )
            for position, offset in zip(
                positions, scenario.get("neighbour_offsets", [[0, 0], [0, 0]])
            ):
                x, y, yaw = transform(
                    (position[0] + offset[0], position[1] + offset[1], position[2])
                )
                actor = world.spawn_actor(
                    blueprint,
                    carla.Transform(
                        carla.Location(x=x, y=-y, z=0.3), carla.Rotation(yaw=-math.degrees(yaw))
                    ),
                )
                actor.apply_control(carla.VehicleControl(hand_brake=True))
                actors.append(actor)
            for _ in range(40):
                world.tick()
            # Ignore initial settling contacts with the road.
            hits.clear()
            contact_details.clear()
            bbox = vehicle.bounding_box
            ego = EgoState(length=bbox.extent.x * 2, width=bbox.extent.y * 2)
            pm = PerceptionModel(ego_vehicle=ego)
            route = [start[:2], goal[:2]]
            planner_class = ParkingPlanner
            if active:
                package = types.ModuleType("parking_carla_experiment")
                package.__path__ = [str(PLUGIN)]
                sys.modules[package.__name__] = package
                planner_class = importlib.import_module(
                    package.__name__ + ".visibility_parking"
                ).VisibilityParkingPlanner
            planner = planner_class(
                GlobalPlan(path=route, trajectory=TrajectoryTracker(path=route, velocity=[0, 0])),
                pm,
            )
            controller = ParkingController()
            if policy:
                policy.configure(planner)
            devices = []
            if sensor:
                from parking_occupancy import FusedLidarParkingOccupancy, LidarParkingOccupancy

                planner.sensor_occupancy = (
                    FusedLidarParkingOccupancy(hazard_support_radius=hazard_support_radius)
                    if rig
                    else LidarParkingOccupancy()
                )
                planner.sensor_ground_z = 0.0
                if policy:
                    planner.sensor_occupancy = FusedLidarParkingOccupancy(
                        minimum_support=policy.minimum_support,
                        hazard_support_radius=policy.hazard_support_radius,
                    )
                mounts = [(0, 0, 2.3)]
                if rig:
                    mounts += [
                        (x, y, 0.55)
                        for x in (-ego.length / 2 - 0.05, ego.length / 2 + 0.05)
                        for y in (-ego.width / 2 - 0.05, ego.width / 2 + 0.05)
                    ]
                if rig and rig_layout == "surround":
                    mounts += [
                        (ego.length / 2 + 0.05, 0, 0.55),
                        (-ego.length / 2 - 0.05, 0, 0.55),
                        (0, ego.width / 2 + 0.05, 0.55),
                        (0, -ego.width / 2 - 0.05, 0.55),
                    ]
                if policy and policy.roof_only:
                    mounts = mounts[:1]
                for index, (x, y, z) in enumerate(mounts):
                    bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
                    for key, value in dict(
                        channels=channels if index == 0 else 64,
                        points_per_second=int(
                            (channels * 5000 if index == 0 else 320000) * 0.05 / dt
                        ),
                        rotation_frequency=1 / dt,
                        range=30 if index == 0 else 15,
                        upper_fov=10,
                        lower_fov=-45 if index == 0 else -85,
                        dropoff_general_rate=scenario.get("dropoff", 0),
                        noise_stddev=scenario.get("noise_stddev", 0),
                        noise_seed=scenario.get("seed", 0) + index * 997,
                    ).items():
                        bp.set_attribute(key, str(value))
                    actor = world.spawn_actor(
                        bp, carla.Transform(carla.Location(x=x, y=-y, z=z)), attach_to=vehicle
                    )
                    actors.append(actor)
                    messages = queue.Queue()
                    actor.listen(messages.put)
                    mount = np.eye(4)
                    mount[:3, 3] = (x, y, z)
                    devices.append((messages, mount))
            physics = vehicle.get_physics_control()
            if len(physics.wheels) != 4:
                raise ValueError("Parking requires a four-wheel vehicle")
            positions = np.array(
                [[w.position.x, w.position.y, w.position.z] for w in physics.wheels]
            )
            controller.ego_distance_front_axle = float(
                np.linalg.norm(positions[:2].mean(axis=0) - positions[2:].mean(axis=0)) / 100
            )
            controller.rear_offset = controller.ego_distance_front_axle / 2
            planner.rear_offset = controller.rear_offset
            steer_limit = max(math.radians(w.max_steer_angle) for w in physics.wheels)

            def update():
                t = vehicle.get_transform()
                v = vehicle.get_velocity()
                ego.x, ego.y, ego.theta = t.location.x, -t.location.y, -math.radians(t.rotation.yaw)
                ego.z = t.location.z
                ego.velocity = math.hypot(v.x, v.y)
                ego.longitudinal_velocity = v.x * math.cos(ego.theta) - v.y * math.sin(ego.theta)

            update()
            if scenario.get("view_objective"):
                if not (active and sensor and rig and rig_layout == "surround"):
                    raise ValueError("View-objective trials require active surround LiDAR")
                from task_visibility import GroundLidar

                planner.view_objective = scenario["view_objective"]
                planner.deterministic_search = True
                planner.search_expansions = 1500
                planner.view_lidar_geometry = tuple(
                    GroundLidar(
                        x,
                        y,
                        ego.z + z,
                        tuple(
                            np.linspace(
                                -45 if index == 0 else -85, 10, channels if index == 0 else 64
                            )
                        ),
                        30 if index == 0 else 15,
                    )
                    for index, (x, y, z) in enumerate(mounts)
                )
                planner.view_scan_parameters = {"azimuth_samples": 250, "phases": 4}
            bay = box(48.2, -1.5, 55.8, 1.5) if parallel else box(50.4, 8, 53.6, 14)
            lot = box(30, -6, 70, 18)
            bay, lot = affine_transform(bay, matrix), affine_transform(lot, matrix)
            planner.request_parking(ParkingRequest(goal, bay, lot, mode))
            rows = []
            violations = 0
            walker = None
            walker_origin = None
            walker_direction = None
            walker_crossed = False
            walker_spawn_step = None
            walker_pause_step = None
            walker_paused = False
            min_pedestrian_clearance = None
            max_pedestrian_overlap = 0.0
            pedestrian_observed_frames = 0
            pedestrian_stop_frames = 0
            scenario_error = None
            previous_walker_position = None
            max_steps = int(scenario.get("max_sim_seconds", 600 if sensor else 240) / dt)
            for i in range(max_steps):
                if scenario.get("pedestrian") and walker is None and i * dt >= 12:
                    # Scenario actor placement only; its state is never supplied to planning.
                    travel_yaw = ego.theta + (math.pi if vehicle.get_control().reverse else 0)
                    staging_bodies = [
                        Polygon(
                            [
                                (v.x, -v.y)
                                for v in actor.bounding_box.get_world_vertices(
                                    actor.get_transform()
                                )
                            ]
                        ).convex_hull
                        for actor in actors
                        if actor.type_id.startswith("vehicle.")
                    ]
                    try:
                        if "pedestrian_origin" in scenario:
                            walker_origin, walker_direction = fixed_crossing(scenario)
                        else:
                            walker_origin, walker_direction = crossing_origin(
                                [ego.x, ego.y], travel_yaw, staging_bodies
                            )
                    except ValueError as error:
                        scenario_error = str(error)
                        break
                    bp = world.get_blueprint_library().find("walker.pedestrian.0001")
                    if bp.has_attribute("is_invincible"):
                        bp.set_attribute("is_invincible", "false")
                    walker = world.spawn_actor(
                        bp,
                        carla.Transform(
                            carla.Location(
                                x=float(walker_origin[0]), y=float(-walker_origin[1]), z=1.0
                            )
                        ),
                    )
                    actors.append(walker)
                    walker_spawn_step = i
                if walker is not None:
                    # CARLA's client actor cache is populated by the first tick;
                    # before it, is_alive/transform can still contain sentinel data.
                    fresh_actor = i == walker_spawn_step
                    location = (
                        carla.Location(x=float(walker_origin[0]), y=float(-walker_origin[1]), z=1)
                        if fresh_actor
                        else walker.get_location()
                    )
                    walker_position = [location.x, -location.y, location.z]
                    if not fresh_actor and (
                        not walker.is_alive
                        or not valid_walker_motion(
                            previous_walker_position,
                            walker_position,
                            dt,
                            scenario.get("pedestrian_speed", 0.6),
                        )
                    ):
                        scenario_error = "Pedestrian disappeared or moved discontinuously"
                        break
                    if not fresh_actor:
                        previous_walker_position = walker_position
                    travelled = np.dot(
                        np.array([location.x, -location.y]) - walker_origin, walker_direction
                    )
                    walker_crossed = bool(travelled >= 5.0)
                    if travelled >= 2.4 and walker_pause_step is None:
                        walker_pause_step = i
                    walker_paused = walker_pause_step is not None and (
                        (i - walker_pause_step) * dt < scenario.get("pedestrian_pause_seconds", 0)
                    )
                    walker.apply_control(
                        carla.WalkerControl(
                            direction=carla.Vector3D(
                                x=float(walker_direction[0]), y=float(-walker_direction[1])
                            ),
                            speed=0.0
                            if walker_crossed or walker_paused
                            else float(scenario.get("pedestrian_speed", 0.6)),
                        )
                    )
                sensors = None
                if sensor:
                    frame = world.tick()
                    update()  # Capture-time pose for saved clouds, not the previous tick's pose.
                    clouds = []
                    for index, (messages, mount) in enumerate(devices):
                        while True:
                            measurement = messages.get(timeout=10)
                            if measurement.frame == frame:
                                break
                            if measurement.frame > frame:
                                raise RuntimeError(
                                    "LiDAR frame advanced beyond requested world tick"
                                )
                        points = (
                            np.frombuffer(measurement.raw_data, dtype=np.float32)
                            .reshape(-1, 4)
                            .copy()
                        )
                        points[:, 1] *= -1
                        clouds.append(
                            Lidar(
                                points=points,
                                stamp=measurement.timestamp,
                                base_to_sensor=mount,
                                sensor_name=f"lidar_{index}",
                            )
                        )
                        if i == 0 and output is not None:
                            output.mkdir(parents=True, exist_ok=True)
                            np.savez_compressed(
                                output / f"{mode.value}_{repeat}_cloud_{index}.npz",
                                points=points,
                                stamp=measurement.timestamp,
                                base_to_sensor=mount,
                            )
                        if (
                            output is not None
                            and walker_spawn_step is not None
                            and i - walker_spawn_step in (20, 50, 80)
                        ):
                            np.savez_compressed(
                                output / f"{mode.value}_{repeat}_event_{i}_cloud_{index}.npz",
                                points=points,
                                stamp=measurement.timestamp,
                                base_to_sensor=mount,
                                ego_pose=[ego.x, ego.y, ego.z, ego.theta],
                            )
                    if (
                        blackout_start is not None
                        and blackout_start <= i < blackout_start + blackout_frames
                    ):
                        clouds = []
                    sensors = types.SimpleNamespace(
                        stamp=measurement.timestamp,
                        lidar=clouds if rig else (clouds[0] if clouds else None),
                    )
                update()
                pm.simulation_time = world.get_snapshot().timestamp.elapsed_seconds
                obstacles = []
                for actor in actors[2:]:
                    if not actor.type_id.startswith(("vehicle.", "walker.")):
                        continue
                    verts = actor.bounding_box.get_world_vertices(actor.get_transform())
                    obstacles.append(Polygon([(v.x, -v.y) for v in verts]).convex_hull)
                if not sensor:
                    planner.update_obstacles(obstacles, pm.simulation_time)
                plan = planner.replan(sensors=sensors)
                observation = getattr(pm, "parking_occupancy", None)
                diagnostics = {}
                if sensor and observation is not None:
                    body = Polygon(ego.get_bb_corners())
                    diagnostics = dict(
                        ego_unobserved_area=body.difference(observation.free).area,
                        bay_free_fraction=observation.free.intersection(bay).area / bay.area,
                        false_free_obstacle_area=sum(
                            observation.free.intersection(p).area for p in obstacles
                        ),
                        points=sum(len(c.points) for c in clouds),
                        sources=list(getattr(observation, "sources", ())),
                        frame=measurement.frame,
                        acquisition_time=measurement.timestamp,
                    )
                    if walker is not None:
                        vertices = walker.bounding_box.get_world_vertices(walker.get_transform())
                        pedestrian_body = Polygon([(v.x, -v.y) for v in vertices]).convex_hull
                        clearance = body.distance(pedestrian_body)
                        min_pedestrian_clearance = (
                            clearance
                            if min_pedestrian_clearance is None
                            else min(min_pedestrian_clearance, clearance)
                        )
                        overlap = body.intersection(pedestrian_body).area
                        max_pedestrian_overlap = max(max_pedestrian_overlap, overlap)
                        seen = observation.occupied is not None and observation.occupied.intersects(
                            pedestrian_body
                        )
                        pedestrian_observed_frames += int(seen)
                        stopped_near = not walker_crossed and clearance < 5 and ego.velocity < 0.01
                        pedestrian_stop_frames += int(stopped_near)
                        diagnostics.update(
                            pedestrian_clearance=clearance,
                            pedestrian_overlap=overlap,
                            pedestrian_observed=bool(seen),
                            pedestrian_crossed=walker_crossed,
                            pedestrian_paused=walker_paused,
                            pedestrian_position=[walker.get_location().x, -walker.get_location().y],
                        )
                    if i % 50 == 0 and output is not None:
                        from shapely import to_geojson

                        (output / f"{mode.value}_{repeat}_geometry_{i}.json").write_text(
                            json.dumps(
                                dict(
                                    pose=[ego.x, ego.y, ego.z, ego.theta],
                                    free=json.loads(to_geojson(observation.free)),
                                    occupied=json.loads(to_geojson(observation.occupied))
                                    if observation.occupied is not None
                                    else None,
                                    ego=json.loads(to_geojson(body)),
                                    bay=json.loads(to_geojson(bay)),
                                    obstacles=[json.loads(to_geojson(p)) for p in obstacles],
                                ),
                                indent=2,
                            )
                        )
                cmd = controller.control(ego, plan, dt)
                reverse_cmd = cmd.direction < 0
                changing = vehicle.get_control().reverse != reverse_cmd and ego.velocity > 0.02
                throttle = max(0, min(1, cmd.acceleration / 3))
                brake = max(0, min(1, -cmd.acceleration / 6))
                if max(plan.velocity) == 0:
                    brake = max(brake, 0.3)
                if changing:
                    throttle, brake, reverse_cmd = 0, 1, vehicle.get_control().reverse
                vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=throttle,
                        brake=brake,
                        steer=max(-1, min(1, -cmd.steer / steer_limit)),
                        reverse=reverse_cmd,
                        manual_gear_shift=True,
                        gear=-1 if reverse_cmd else 1,
                    )
                )
                if not sensor:
                    world.tick()
                update()
                body = Polygon(ego.get_bb_corners())
                violations += int(
                    not lot.covers(body) or any(body.intersects(p) for p in obstacles)
                )
                rows.append(
                    dict(
                        time=i * dt,
                        x=ego.x,
                        y=ego.y,
                        yaw=ego.theta,
                        speed=ego.velocity,
                        longitudinal_velocity=ego.longitudinal_velocity,
                        state=planner.state,
                        reason=planner.reason,
                        base_reason=getattr(planner, "base_reason", planner.reason),
                        certificate=asdict(pm.parking_arbitration.certificate)
                        if getattr(pm, "parking_arbitration", None) is not None
                        and pm.parking_arbitration.certificate is not None
                        else None,
                        direction=cmd.direction,
                        control_steer=cmd.steer,
                        control_acceleration=cmd.acceleration,
                        segment_index=getattr(planner, "_segment", 0),
                        segment_count=len(getattr(planner, "_segments", [])),
                        geometric_endpoint=planner.poses[-1].tolist()
                        if planner.poses is not None
                        else None,
                        **diagnostics,
                        occupancy_status=getattr(
                            getattr(pm, "parking_occupancy", None), "status", None
                        ),
                        free_area=getattr(
                            getattr(getattr(pm, "parking_occupancy", None), "free", None),
                            "area",
                            None,
                        ),
                        rejections=dict(
                            getattr(getattr(planner, "arbiter", None), "rejections", {})
                        ),
                    )
                )
                if i % 50 == 0 and output is not None:
                    (output / f"{mode.value}_{repeat}_checkpoint.json").write_text(
                        json.dumps(
                            dict(
                                completed=False,
                                collisions=len(hits),
                                body_violations=violations,
                                telemetry=rows,
                            )
                        )
                    )
                if i % 200 == 0:
                    print(
                        mode.value,
                        i,
                        planner.state,
                        planner.reason,
                        round(ego.velocity, 3),
                        flush=True,
                    )
                if scenario.get("stop_on_contact") and (hits or violations):
                    break
                if (
                    planner.state == "PARKING_BLOCKED"
                    and i > (100 if sensor else 400)
                    and all(r["speed"] < 0.01 for r in rows[-60:])
                    and (
                        not scenario.get("pedestrian")
                        or (walker_crossed and i > walker_spawn_step + 200)
                    )
                ):
                    break
                if planner.state == "PARKED" and ego.velocity < 0.01:
                    break
            distance = math.hypot(ego.x - goal[0], ego.y - goal[1])
            heading = abs(angle_error(ego.theta - goal[2]))
            return dict(
                vehicle_geometry=dict(
                    length=ego.length,
                    width=ego.width,
                    wheelbase=controller.ego_distance_front_axle,
                    rear_offset=controller.rear_offset,
                ),
                type=mode.value,
                scenario=scenario,
                ablation_policy=policy_name,
                policy_configuration=policy.metadata() if policy else None,
                scenario_error=scenario_error,
                goal_pose=goal,
                wall_seconds=time.monotonic() - wall_start,
                pedestrian_crossed=walker_crossed,
                pedestrian_observed_frames=pedestrian_observed_frames,
                pedestrian_stop_frames=pedestrian_stop_frames,
                pedestrian_spawn_step=walker_spawn_step,
                pedestrian_origin=walker_origin.tolist() if walker_origin is not None else None,
                pedestrian_direction=walker_direction.tolist()
                if walker_direction is not None
                else None,
                minimum_pedestrian_clearance=min_pedestrian_clearance,
                maximum_pedestrian_overlap=max_pedestrian_overlap,
                initial_pose=start,
                blackout_start=blackout_start,
                blackout_frames=blackout_frames,
                fixed_delta_seconds=dt,
                hazard_support_radius=hazard_support_radius,
                sensor_only_obstacles=sensor,
                near_field_rig=rig,
                rig_layout=rig_layout,
                active_observation=active,
                view_objective=getattr(planner, "view_objective", None),
                deterministic_search=getattr(planner, "deterministic_search", False),
                lidar_channels=channels if sensor else None,
                localization="CARLA ground truth; perception uses LiDAR only"
                if sensor
                else "CARLA ground truth",
                repeat=repeat,
                passed=planner.state == "PARKED"
                and distance < 0.2
                and heading < math.radians(5)
                and not hits
                and not violations
                and scenario_error is None
                and (
                    not scenario.get("pedestrian")
                    or (
                        walker_crossed
                        and pedestrian_observed_frames > 0
                        and pedestrian_stop_frames > 0
                        and max_pedestrian_overlap == 0
                    )
                ),
                position_error=distance,
                heading_error=math.degrees(heading),
                collisions=len(hits),
                collision_events=contact_details,
                body_violations=violations,
                final_state=planner.state,
                telemetry=rows,
            )
        finally:
            for actor in reversed(actors):
                if actor.is_alive:
                    if actor.type_id.startswith("sensor."):
                        actor.stop()
                    actor.destroy()
            # Flush actor destruction before the next case spawns at a nearby pose.
            world.tick()

    def freeze_sources(output, paths):
        """Store exactly the bytes hashed in the protocol before executing trials."""
        hashes = {}
        for path in paths:
            data = path.read_bytes()
            relative = path.relative_to(PLUGIN.parent)
            destination = output / "source_snapshot" / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(data)
            key = (
                str(path.relative_to(PLUGIN))
                if path.is_relative_to(PLUGIN)
                else "../" + str(relative)
            )
            hashes[key] = hashlib.sha256(data).hexdigest()
        return hashes

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--repeats", type=int, default=1)
        parser.add_argument("--start", type=float, nargs=3)
        parser.add_argument("--blackout-start", type=int)
        parser.add_argument("--blackout-frames", type=int, default=20)
        parser.add_argument("--dt", type=float, default=0.05)
        parser.add_argument("--sensor", action="store_true")
        parser.add_argument("--rig", action="store_true")
        parser.add_argument("--rig-layout", choices=["corners", "surround"], default="corners")
        parser.add_argument("--hazard-support-radius", type=float, default=2.0)
        parser.add_argument("--active", action="store_true")
        parser.add_argument("--channels", type=int, default=64)
        parser.add_argument("--mode", choices=[m.value for m in ParkingType])
        parser.add_argument("--output", type=Path, default=Path("logs/parking_carla"))
        parser.add_argument(
            "--suite", type=Path, help="Frozen JSON list of scenario configurations"
        )
        parser.add_argument("--case", help="Run one named case from --suite")
        args = parser.parse_args()
        if not 0.01 <= args.dt <= 0.1:
            parser.error("--dt must be between .01 and .1")
        if args.active and not args.sensor:
            parser.error("--active requires --sensor")
        scenarios = json.loads(args.suite.read_text()) if args.suite else [None]
        if args.case:
            scenarios = [s for s in scenarios if s and s["id"] == args.case]
            if not scenarios:
                parser.error("Unknown --case")
        for scenario in scenarios:
            if scenario is None:
                continue
            if not args.sensor:
                parser.error("--suite requires --sensor")
            if (
                not 0 <= scenario.get("dropoff", 0) < 1
                or not 0 <= scenario.get("noise_stddev", 0) <= 0.5
            ):
                parser.error("Invalid sensor noise/dropoff")
            ParkingType(scenario["mode"])
            if scenario.get("view_objective") not in (
                None,
                "legacy",
                "task",
                "task_beams",
                "task_scan",
            ):
                parser.error("Unknown view objective")
            if "ablation_policy" in scenario:
                if scenario["ablation_policy"] not in POLICIES:
                    parser.error("Unknown ablation policy")
                if not (args.sensor and args.active and args.rig and args.rig_layout == "surround"):
                    parser.error("Ablations require --sensor --active --rig --rig-layout surround")
            if "pedestrian_origin" in scenario:
                fixed_crossing(scenario)
            if not scenario["id"].replace("_", "").isalnum():
                parser.error("Scenario ID must contain letters, numbers or underscores")
        client = carla.Client("localhost", 2000)
        client.set_timeout(60)
        original = client.get_world()
        # A newly connected client's actor cache can be empty until its first
        # snapshot. Fail on a non-ticking server instead of overwriting its world.
        original.wait_for_tick(10)
        if original.get_actors().filter("vehicle.*"):
            raise RuntimeError("Server must be idle with no vehicles")
        original_map = original.get_map().name
        if "OpenDrive" in original_map:
            original_map = "Town10HD_Opt"
        results = []
        args.output.mkdir(parents=True, exist_ok=True)
        if any(args.output.iterdir()):
            raise RuntimeError("Output directory must be empty; preserve previous trial evidence")
        if args.suite:
            (args.output / "protocol.json").write_text(
                json.dumps(
                    dict(
                        suite=scenarios,
                        args={
                            key: str(value) if isinstance(value, Path) else value
                            for key, value in vars(args).items()
                        },
                        source_sha256=freeze_sources(
                            args.output,
                            sorted(PLUGIN.glob("*.py"))
                            + [
                                Path(__file__),
                                Path(__file__).with_name("parking_pedestrian.py"),
                                Path(__file__).with_name("parking_ablation.py"),
                                PLUGIN.parent / "avlite/c10_perception/c11_perception_model.py",
                                PLUGIN.parent / "avlite-bridge-carla/carla_bridge.py",
                            ],
                        ),
                    ),
                    indent=2,
                )
                + "\n"
            )
        try:
            world = client.generate_opendrive_world(PAD)
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = args.dt
            settings.no_rendering_mode = True
            world.apply_settings(settings)
            for scenario in scenarios:
                for repeat in range(args.repeats):
                    modes = (
                        [ParkingType(scenario["mode"])]
                        if scenario
                        else ([ParkingType(args.mode)] if args.mode else ParkingType)
                    )
                    for mode in modes:
                        case_output = args.output / scenario["id"] if scenario else args.output
                        result = run(
                            world,
                            mode,
                            repeat,
                            args.sensor,
                            args.active,
                            args.channels,
                            case_output,
                            args.rig,
                            args.hazard_support_radius,
                            args.start,
                            args.rig_layout,
                            args.blackout_start,
                            args.blackout_frames,
                            args.dt,
                            scenario,
                        )
                        results.append(result)
                        print({k: v for k, v in result.items() if k != "telemetry"}, flush=True)
                        args.output.mkdir(parents=True, exist_ok=True)
                        (args.output / "results.json").write_text(json.dumps(results, indent=2))
                        if scenario:
                            (case_output / "results.json").write_text(
                                json.dumps([result], indent=2)
                            )
        finally:
            client.load_world(
                original_map.split("/")[-1] if "OpenDrive" not in original_map else "Town10HD_Opt"
            )
        return 0 if results and all(r["passed"] for r in results) else 1

    if __name__ == "__main__":
        raise SystemExit(main())
