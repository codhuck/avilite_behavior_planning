"""Exercise task handovers with real CARLA RGB/LiDAR and sensor perception.

Requires an idle Town10HD_Opt server and the installed detector weights. No map
reload, traffic-light manipulation or dynamic ground-truth perception is used.
The harness owns braking and sensor-fault injection; normal driving uses AVLite.
"""

# ruff: noqa: E402 -- select matching core and plugin sources explicitly
# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import hashlib
    import json
    import os
    import sys
    import time
    from dataclasses import replace
    from pathlib import Path

    PLUGIN = Path(__file__).resolve().parents[1]
    ROOT = PLUGIN.parent
    sys.path[:0] = [str(ROOT), str(PLUGIN)]

    import carla
    import numpy as np
    import yaml
    from avlite.c30_control.c31_control_model import ControlCommand
    from avlite.c40_execution.c43_task_strategy import ExecutionStarted
    from avlite.c40_execution.c49_settings import ExecutionSettings as Settings
    from avlite.c60_apps.c62_factory import executor_factory, load_stack_settings
    from avlite.c60_apps.c67_plugin_env import PluginEnv

    from profile_setup import write_profile

    def require_idle(client):
        world = client.get_world()
        if world.get_map().name.rsplit("/", 1)[-1] != "Town10HD_Opt":
            raise RuntimeError("Load Town10HD_Opt on an idle server before this check")
        owned_types = ("vehicle.", "sensor.", "walker.")
        actors = [
            (a.id, a.type_id) for a in world.get_actors() if a.type_id.startswith(owned_types)
        ]
        if actors:
            raise RuntimeError(f"Exclusive idle server required; existing actors: {actors}")
        return world

    def build_executor():
        fields = {
            "executer_type": "c40_executer_type",
            "bridge": "c40_bridge",
            "perception_strategy_name": "c40_perception",
            "localization_strategy_name": "c40_localization",
            "mapping_strategy_name": "c40_mapping",
            "global_planner_strategy_name": "c40_global_planner",
            "local_planner_strategy_name": "c40_local_planner",
            "controller_strategy_name": "c40_controller",
            "default_global_trajectory_file": "c40_global_trajectory",
            "map_file": "c40_map",
        }
        return executor_factory(**{key: getattr(Settings, field) for key, field in fields.items()})

    def forbidden(*args, **kwargs):
        raise AssertionError("Dynamic ground-truth perception was called")

    def run(client, directory):
        require_idle(client)
        ex = build_executor()
        rows, collisions = [], []
        collision_sensor = None
        original_frame = ex.world.get_sensor_frame
        started = time.monotonic()
        try:
            task = next(
                t for t in ex.task_runner.tasks if type(t).__name__ == "PlanningCoordinatorTask"
            )
            detector = ex.perception._detector
            package = sys.modules[type(task).__module__.rsplit(".", 1)[0]]
            ex.world.get_ground_truth_perception_model = forbidden
            sys.modules[package.__name__ + ".carla_task"].observe_carla = forbidden
            assert {str(getattr(c, "name", c)) for c in Settings.c41_world_stack_capabilities} == {
                "LOCALIZATION"
            }
            for obj, expected in (
                (task, PLUGIN / "coordinator.py"),
                (ex.world, ROOT / "avlite-bridge-carla/carla_bridge.py"),
            ):
                assert Path(sys.modules[type(obj).__module__].__file__).resolve() == expected
            ex.task_runner.notify(ExecutionStarted())
            assert task.active_strategy == "road-tree", task.last_result
            vehicle_id = ex.world.vehicle.id
            collision_sensor = ex.world.world.spawn_actor(
                ex.world.world.get_blueprint_library().find("sensor.other.collision"),
                carla.Transform(),
                attach_to=ex.world.vehicle,
            )
            collision_sensor.listen(lambda event: collisions.append(int(event.frame)))

            def tick(phase, brake=False):
                if brake:
                    ex._last_cmd = ControlCommand(acceleration=-8.0, steer=0.0)
                ex.step(
                    sim_dt=0.05,
                    perception_dt=0.05,
                    control_dt=0.05,
                    replan_dt=0.05,
                    call_control=not brake,
                    pace_perception=False,
                    pace_replan=False,
                )
                observation = ex.pm.regulatory_observation
                assert observation.source == "rgb_lidar"
                plan = ex.local_planner.get_local_plan()
                row = dict(
                    phase=phase,
                    time=ex.elapsed_sim_time,
                    x=float(ex.ego_state.x),
                    y=float(ex.ego_state.y),
                    speed=float(ex.world.get_ego_state().velocity),
                    sensor_status=observation.diagnostics.get("status"),
                    sensor_stamp=observation.sensor_stamp,
                    state=plan.behavior_state,
                    strategy=task.active_strategy,
                )
                rows.append(row)
                if len(rows) % 40 == 0:
                    print(json.dumps(row), flush=True)
                return row

            initial = np.array([ex.ego_state.x, ex.ego_state.y])
            for _ in range(160):
                row = tick("initial-drive")
                if row["speed"] > 0.3 and ex.pm.ego_vehicle.velocity > 0.2:
                    break
            else:
                raise AssertionError(f"No initial motion: {rows[-1]}")
            old_planner = ex.local_planner
            task.request(ex, "road-rules")
            assert task.last_result["status"] == "rejected" and ex.local_planner is old_planner
            moving_speed = row["speed"]
            for _ in range(160):
                tick("initial-drive")
                if np.linalg.norm(np.array([ex.ego_state.x, ex.ego_state.y]) - initial) > 6:
                    break
            else:
                raise AssertionError("Initial drive did not cover six metres")
            for _ in range(100):
                tick("harness-brake", brake=True)
                if (
                    abs(ex.world.get_ego_state().velocity) < 0.02
                    and abs(ex.pm.ego_vehicle.velocity) < 0.02
                ):
                    break
            else:
                raise AssertionError("Vehicle did not stop under harness braking")
            ex.stop()
            task.request(ex, "unknown-strategy")
            assert task.last_result["status"] == "rejected" and ex.local_planner is old_planner
            task.request(ex, "road-rules")
            assert task.active_strategy == "road-rules", task.last_result
            assert ex.local_planner is not old_planner
            assert ex.world.vehicle.id == vehicle_id and ex.perception._detector is detector
            selected = ex.local_planner
            ex.task_runner.notify(ExecutionStarted())
            assert ex.local_planner is selected

            def corrupt_frame(kind):
                frame = original_frame()
                name = frame.primary_camera_name
                cameras = dict(frame.cameras)
                if kind == "missing-lidar":
                    lidars = dict(frame.lidars)
                    lidars[frame.primary_lidar_name] = replace(frame.lidar, points=None)
                    return replace(frame, lidars=lidars)
                if kind == "black-camera":
                    cameras[name] = replace(frame.camera, rgb=np.zeros_like(frame.camera.rgb))
                elif kind == "unsynchronised":
                    cameras[name] = replace(frame.camera, stamp=frame.camera.stamp - 0.2)
                elif kind == "invalid-calibration":
                    cameras[name] = replace(frame.camera, intrinsic=np.zeros((3, 3)))
                else:
                    raise ValueError(kind)
                return replace(frame, cameras=cameras)

            faults = []
            for kind, diagnostic in (
                ("black-camera", "unusable_image"),
                ("missing-lidar", "missing_or_unsynchronised_sensors"),
                ("unsynchronised", "missing_or_unsynchronised_sensors"),
                ("invalid-calibration", "invalid_calibration"),
            ):
                ex.world.get_sensor_frame = lambda kind=kind: corrupt_frame(kind)
                fault = tick(kind)
                assert fault["sensor_status"] == diagnostic, fault
                assert fault["state"] == "PERCEPTION_STOP", fault
                faults.append(fault)
                ex.world.get_sensor_frame = original_frame
                healthy = tick("healthy-camera")
                assert healthy["sensor_status"] == "ok", healthy
            resume = np.array([ex.ego_state.x, ex.ego_state.y])
            for _ in range(200):
                tick("recovery-drive")
                if np.linalg.norm(np.array([ex.ego_state.x, ex.ego_state.y]) - resume) > 5:
                    break
            else:
                raise AssertionError(f"No recovery motion: {rows[-1]}")
            assert rows[-1]["sensor_status"] == "ok"
            assert not collisions, collisions
            result = dict(
                passed=True,
                vehicle_id=vehicle_id,
                moving_request_speed=moving_speed,
                same_vehicle=True,
                same_detector=True,
                dynamic_ground_truth_traps=True,
                injected_faults=faults,
                recovery_distance=float(
                    np.linalg.norm(np.array([ex.ego_state.x, ex.ego_state.y]) - resume)
                ),
                collisions=collisions,
                history=list(task.history),
                ticks=len(rows),
                wall_seconds=time.monotonic() - started,
            )
        finally:
            ex.world.get_sensor_frame = original_frame
            if collision_sensor is not None:
                collision_sensor.stop()
                collision_sensor.destroy()
            ex.stop()
            ex.world.close()
            (directory / "frames.json").write_text(json.dumps(rows, indent=2) + "\n")
        require_idle(client)
        return result

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--output", required=True, type=Path)
        parser.add_argument("--repeat", type=int, default=3)
        args = parser.parse_args()
        if args.repeat < 1:
            parser.error("--repeat must be positive")
        # Core may re-exec once to source an installed ROS environment. Do this
        # before creating output; re-execution must not collide with its own files.
        PluginEnv().apply()
        directory = args.output.resolve()
        directory.mkdir(parents=True, exist_ok=False)
        profiles = directory / "profiles"
        path = write_profile(
            profiles / "coordinator-live.yaml",
            ROOT / "avlite/data/Town10HD_Opt.xodr",
            ROOT / "avlite-bridge-carla",
            behavior_tree=True,
        )
        config = yaml.safe_load(path.read_text())
        config["plugins"]["avlite_behavior_hdmap"]["telemetry_directory"] = str(
            directory / "telemetry"
        )
        path.write_text(yaml.safe_dump(config, sort_keys=False))
        os.environ["AVLITE_CONFIG_DIR"] = str(profiles)
        load_stack_settings("coordinator-live", load_plugins=True)
        client = carla.Client("127.0.0.1", 2000)
        client.set_timeout(20)
        result = dict(passed=False, server=client.get_server_version(), runs=[], scope=__doc__)
        result["sources"] = {
            str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [
                *PLUGIN.glob("*.py"),
                Path(__file__),
                ROOT / "avlite-bridge-carla/carla_bridge.py",
            ]
        }
        try:
            for repeat in range(args.repeat):
                trial = directory / f"run-{repeat + 1}"
                trial.mkdir()
                result["runs"].append(run(client, trial))
            result["passed"] = True
        except Exception as exc:
            result["error"] = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result, indent=2), flush=True)

    if __name__ == "__main__":
        main()
