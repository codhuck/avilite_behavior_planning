# AVLite HD-map behavior plugin

Behavior planning and map-aware planner selection for AVLite. The plugin adds
STOP handling, traffic-light waiting, crosswalk yielding and experimental race
following/passing. An RGB/LiDAR detector supplies dynamic observations; a separate
Python package provides replaceable traffic-light classifiers.

**Plugin name:** `avlite-behavior-hdmap`

## Requirements

- Python 3.10 or newer and a compatible AVLite checkout. The tested workspace
  uses AVLite 0.6.6 with planning/execution integration changes; compatibility
  with an unmodified released AVLite installation has not been established.
- For the city demo: the CARLA bridge, a running CARLA server with its matching
  Python API, and `Town10HD_Opt.xodr`. Validation used CARLA 0.9.16.
- For the race demo: BasicSim and the Yas Marina map/reference assets.
- Python dependencies are listed in [requirements.txt](requirements.txt).
  The installer downloads the pinned YOLO model; traffic-light weights are bundled.

## Install

Clone into a directory named `avlite-behavior-hdmap` to preserve the plugin
registry name. Activate the Python environment used by AVLite:

```bash
git clone git@github.com:codhuck/avilite_behavior_planning.git avlite-behavior-hdmap
cd avlite-behavior-hdmap
```

Then install from this directory:

```bash
python install.py
python install.py --check
```

AVLite's plugin dependency installer can also install `requirements.txt` from
this directory. Installation does not start CARLA or select an AVLite profile.
See [the perception package guide](SIGNAL_PACKAGE.md) for standalone installation,
model downloads and alternative classifiers.

## Configure and run

Generate a Town10 sensor profile using paths on your machine:

```bash
python install.py \
  --profile-output ~/.config/avlite/sensor-road-trained.yaml \
  --map-path /path/to/Town10HD_Opt.xodr \
  --bridge-path /path/to/avlite-bridge-carla
```

Replace both `/path/to/…` values with existing paths. The generator resolves the
plugin directory, bundled route and signal geometry, and chooses a telemetry
folder. It refuses to overwrite an existing profile; choose another filename
if necessary. Historical YAML files in `profiles/` contain development-machine
paths and should not be copied unchanged.

1. Start the compatible CARLA server on `localhost:2000`.
2. Start AVLite with `python -m avlite` in its environment.
3. Select `sensor-road-trained` (or the filename you chose), then **Reload Stack**.
4. Press **Start**. Keep only one client responsible for advancing the simulation.

The generated profile uses the dispatchers below. On Town10 they select the
road planning stages listed in `c20_planning`:

```yaml
c10_perception:
  c12_detection_strategy: SensorRegulatoryDetector
c20_planning:
  c23_behavioral_strategy: HDMapBehaviorPlanner
  c23_path_strategy: ReferencePathPlanner
  c23_velocity_strategy: HDMapRuleVelocityPlanner
c40_execution:
  c40_global_planner: GlobalPlannerDispatcher
  c40_local_planner: PlannerDispatcher
  c40_execution_tasks: [SensorTelemetryTask, PlannerSwitchTask]
plugins:
  avlite_behavior_hdmap:
    sensor_signal_backend: trained_rgb
```

This is a configuration excerpt, not a complete replacement profile. Change
model settings with execution stopped, then reload the stack.

## Planner selection

| Map type | Global planner | Local behavior / path / velocity |
| --- | --- | --- |
| `HDMap` | `HDMapLaneFollowingPlanner` | `HDMapBehaviorPlanner` / `ReferencePathPlanner` / `HDMapRuleVelocityPlanner` |
| `RaceMap` | `GlobalCenterlineRacePlanner` | `RaceBehaviorPlanner` / `RacePathPlanner` / `RaceVelocityPlanner` |

`GlobalPlannerDispatcher` and `PlannerDispatcher` select by map type.
`PlannerSwitchTask` replaces the planning stack while stopped, using
`SyncExecuter`. It does not change the simulator, teleport the vehicle or switch
perception. Yas Marina runs in BasicSim, not a CARLA race world.
See [architecture and behavior](docs/architecture.md) for the contracts.

## Tests

From the parent AVLite workspace with its Python environment activated:

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python -m pytest -q -o addopts='' \
  test/plugins/test_signal_distribution.py \
  test/plugins/test_traffic_light_vision.py \
  test/plugins/test_sensor_regulatory.py \
  test/plugins/test_hdmap_behavior_plugin.py test/c40_execution
```

These tests live in the parent workspace, not the standalone perception wheel.
The style-cleanup run passed all 192 tests, including timing-sensitive tests.
Live CARLA checks and full-route results are documented separately in
[sensor validation](SENSOR_VALIDATION.md) and [planner validation](VALIDATION.md).
A unit-test pass is not a full-route or real-road validation.

Style checks, also from the parent workspace:

```bash
ruff check avlite-behavior-hdmap
ruff format --check avlite-behavior-hdmap
```

## Limitations

Dynamic light states and pedestrian observations in the sensor profile come
from RGB/LiDAR. Lane topology, STOP locations, crossing polygons and signal-head
geometry still come from the static map; the CARLA demo retains simulator ego
localization. Missing or stale observations cause conservative stopping.

The bundled model is CARLA-adapted. Real-road deployment, night/rain, arrow
signals and calibration errors are not validated. Road lane changes and
self-intersecting routes are unsupported; race passing covers scripted rivals,
not arbitrary traffic. Behavior uses a Python state machine, not BehaviorTree.CPP.

## Documentation

- [Architecture and behavior](docs/architecture.md)
- [RGB/LiDAR adapter and backend details](SENSOR_PERCEPTION.md)
- [Independent perception package, model selection and extension API](SIGNAL_PACKAGE.md)
- [Sensor validation and model limitations](SENSOR_VALIDATION.md)
- [Planner and dispatcher validation](VALIDATION.md)
- [Development history and earlier demo commands](docs/development-history.md)
