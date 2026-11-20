"""Validate the actual AVLite CARLA bridge's optional multi-LiDAR frame contract.

Exclusive idle server required. This checks integration and sensor failure
handling, not a parking drive. The created vehicle/sensors are removed on exit.
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
    import types
    from pathlib import Path

    import carla
    import numpy as np

    ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(ROOT))
    from avlite.c40_execution.c49_settings import ExecutionSettings

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--output", type=Path, default=Path("logs/parking_bridge_rig"))
        parser.add_argument(
            "--bridge-path",
            type=Path,
            default=ROOT / "avlite-bridge-carla",
            help="Matching CARLA bridge plugin checkout",
        )
        args = parser.parse_args()
        bridge_path = args.bridge_path.expanduser().resolve()
        if not (bridge_path / "carla_bridge.py").is_file():
            parser.error("CARLA bridge not found; set --bridge-path to its checkout")
        package = types.ModuleType("parking_bridge_validation")
        package.__path__ = [str(bridge_path)]
        sys.modules[package.__name__] = package
        module = importlib.import_module(package.__name__ + ".carla_bridge")

        client = carla.Client("localhost", 2000)
        client.set_timeout(30)
        world = client.get_world()
        if world.get_actors().filter("vehicle.*"):
            raise RuntimeError("Use an idle exclusive CARLA server")
        spawn = world.get_map().get_spawn_points()[0]
        settings = module.PluginSettings
        settings.npc_count = 0
        settings.parking_lidar_layout = "surround"
        settings.sensor_timeout = 10
        ExecutionSettings.c40_sim_dt = 0.1
        ExecutionSettings.c41_world_capabilities = None
        ego = module.EgoState(
            x=spawn.location.x, y=-spawn.location.y, theta=-math.radians(spawn.rotation.yaw)
        )
        bridge = module.CarlaBridge(
            ego, scene_name=world.get_map().name, timeout=30, sync_mode=True
        )
        rows = []
        try:
            for _ in range(10):
                bridge.step(0.1)
                frame = bridge.get_sensor_frame()
                assert len(frame.lidars) == 9
                assert all(
                    c.points is not None and abs(c.stamp - frame.camera.stamp) < 1e-6
                    for c in frame.lidars.values()
                )
                assert len({c.sensor_id for c in frame.lidars.values()}) == 9
                assert all(np.isfinite(c.base_to_sensor).all() for c in frame.lidars.values())
                rows.append(
                    dict(
                        frame=frame.carla_frame,
                        stamp=frame.camera.stamp,
                        points={name: len(c.points) for name, c in frame.lidars.items()},
                    )
                )
            bridge._extra_lidar_sensors[0].stop()
            bridge.sensor_timeout = 0.5
            stopped = False
            try:
                bridge.step(0.1)
            except TimeoutError:
                stopped = True
            assert stopped, "An incomplete multi-sensor frame was incorrectly published"
            args.output.mkdir(parents=True, exist_ok=True)
            result = dict(
                passed=True,
                frames=rows,
                incomplete_frame_rejected=stopped,
                scope="Actual bridge integration; not a parking completion test",
            )
            (args.output / "results.json").write_text(json.dumps(result, indent=2) + "\n")
            print("PASS: 10 exact-frame RGB + nine-LiDAR snapshots; missing device rejected")
        finally:
            bridge.close()

    if __name__ == "__main__":
        main()
