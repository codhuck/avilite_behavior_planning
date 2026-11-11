"""Generate the Town10 demo profile for this checkout without developer paths."""

from pathlib import Path

import yaml


def write_profile(output, map_path, bridge_path, root=None):
    root = Path(root or Path(__file__).parent).resolve()
    output = Path(output).expanduser().resolve()
    map_path = Path(map_path).expanduser().resolve()
    bridge_path = Path(bridge_path).expanduser().resolve()
    if not map_path.is_file():
        raise FileNotFoundError(map_path)
    if not (bridge_path / "__init__.py").is_file():
        raise FileNotFoundError(bridge_path / "__init__.py")
    config = yaml.safe_load((root / "profiles/sensor-road-trained.yaml").read_text())
    config["c69_apps"]["c60_selected_profile"] = output.stem
    config["c69_apps"]["c62_community_plugins"] = {
        "avlite-behavior-hdmap": str(root),
        "avlite-bridge-carla": str(bridge_path),
    }
    config["c40_execution"]["c40_map"] = str(map_path)
    config["c40_execution"]["c40_global_trajectory"] = str(root / "data/sensor_demo.json")
    settings = config["plugins"]["avlite_behavior_hdmap"]
    settings["sensor_signal_geometry"] = str(root / "data/Town10HD_Opt_signals.json")
    settings["telemetry_directory"] = str(Path.home() / ".local/state/avlite" / output.stem)
    output.parent.mkdir(parents=True, exist_ok=True)
    # Never overwrite an existing user's profile implicitly.
    with output.open("x") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    return output
