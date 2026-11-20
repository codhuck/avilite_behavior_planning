# avlite-behavior-hdmap

Road and race behavior planning for [AVLite](https://github.com/AV-Lab/avlite).
Registers planning strategies for STOP signs, traffic lights, crosswalks,
following and overtaking, plus a task that selects and switches between them.
Includes forward, reverse and parallel parking.

**Plugin name:** `avlite-behavior-hdmap`

## Install

Requires Python 3.10+ and AVLite 0.6.7. The plugin is not yet listed in the
community catalog. Activate the AVLite environment, then clone and install:

```bash
git clone https://github.com/codhuck/avilite_behavior_planning.git avlite-behavior-hdmap
cd avlite-behavior-hdmap
python -m pip install -r requirements.txt
```

Generate a BasicSim profile and start AVLite:

```bash
python tools/prepare_coordinator_profile.py --mode race \
  --output ~/.config/avlite/community-race.yaml
python -m avlite
```

Select **community-race**, click **Reload Stack**, then **Start**. The example
uses Yas Marina and two scripted rivals. Use `--mode road` with a different
output filename for the small road example. CARLA is not required for either.
The generator registers the plugin path and refuses to overwrite an existing
profile. If you use `AVLITE_CONFIG_DIR`, write the profile there instead.

## Configuration

The [recommended profile](avlite-behavior-hdmap.yaml) uses AVLite's bundled Yas
Marina map and limits reference speed to 3 m/s. The following excerpt shows the
plugin registration and execution adapters:

```yaml
c69_apps:
  c62_community_plugins:
    avlite-behavior-hdmap: avlite-behavior-hdmap
c40_execution:
  c40_bridge: PlanningBasicSim
  c40_executer_type: PlanningSyncExecuter
  c40_local_planner: BehaviorPlanningPipeline
  c40_controller: BehaviorStanleyController
  c40_execution_tasks: [PlanningCoordinatorTask, RaceTrafficTask]
plugins:
  avlite_behavior_hdmap:
    coordinator_initial_strategy: race
    cruise_speed_limit: 3.0
```

The name sentinel resolves under AVLite's managed plugin directory. For another
checkout location, use its path or generate a profile as shown above.

`coordinator_initial_strategy` selects the planning configuration:

| Value | Behavior |
| --- | --- |
| `auto` | `road-tree` on an HD map; `race` on a race map |
| `road-rules` | Road rules implemented as a state machine |
| `road-tree` | The same road rules composed as a reactive behavior tree |
| `race` | Following, passing and returning to the reference path |

Change startup settings with execution stopped, then reload the stack. Mission
tasks can also request a stopped handover through `PlanningCoordinatorTask`.
See [task coordination](docs/task-coordination.md) for the API.

## Sensors and parking

For camera/LiDAR perception, run `python install.py` to install and check the
model assets. `python install.py --check` verifies an existing installation.
The CARLA demo requires a matching server, Python API and bridge. Setup and
classifier options are in [classifier setup](docs/perception/classifier.md).

Parking uses `ParkingTask`, `ParkingPlanner` and `ParkingController`. Select the
type through the task API; [parking.md](docs/parking.md) describes the scene
format and examples. The custom **Parking…** dialog requires the development
GUI and is not included in stock AVLite.

## Tests

From this directory, with `pytest` and `ruff` installed:

```bash
python -m pytest --rootdir=.. --import-mode=importlib -q tests
ruff check .
ruff format --check .
python tools/coordinator_demo.py --mode both --repeat 3 --output logs/coordinator-check.json
```

See [testing](docs/testing.md) for profile checks, parking scenarios and recorded
results. The BasicSim examples use simulated observations. The camera model is
CARLA-adapted; real-road operation and arbitrary race traffic remain unvalidated.

## Repository layout

| Directory | Contents |
| --- | --- |
| `planning/` | Road and race planners, behavior tree and dispatcher |
| `parking/` | Parking planner/controller, search, occupancy and visibility |
| `perception/` | Regulatory geometry, observations and sensor detector |
| `execution/` | Coordination tasks, handovers and runtime adapters |
| `avlite_signal_perception/` | Separately installable traffic-light classifier |
| `profiles/`, `data/` | Example settings, routes and map fixtures |
| `tests/`, `tools/` | Tests, scenario runners and offline analysis |
| `docs/` | Setup guides, API reference and validation reports |

AVLite loads `__init__.py` and `settings.py` from the repository root.

## Documentation

[Documentation index](docs/README.md): architecture, task API, perception,
parking and experiment reports.

## License

[MIT](LICENSE).
