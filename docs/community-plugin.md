# AVLite integration

Tested with AVLite 0.6.7 at commit
`1df512a3718d78a4d737e07178a9db03cf5d4f05`.

## Plugin layout

The repository root is the plugin package. `__init__.py` exports the strategies;
`settings.py` defines their settings. AVLite registers the subclasses when it
loads the package.

| Path | Contents |
| --- | --- |
| `planning/` | Road and race strategies and planner dispatcher |
| `parking/` | Parking planner/controller, search, occupancy and view selection |
| `perception/` | Sensor detector, observation contracts and regulatory geometry |
| `execution/` | Tasks, handovers, commands and runtime adapters |
| `avlite_signal_perception/` | Independent traffic-light classifier package |
| `profiles/`, `data/` | Settings, routes, signal survey and coordination fixtures |
| `tools/`, `tests/` | Setup, scenario runners and regression tests |
| `docs/validation/`, `docs/research/`, `docs/history/` | Results, study notes and earlier development records |

The registry name is `avlite-behavior-hdmap`; the import namespace is
`avlite.plugins.avlite_behavior_hdmap`. `requirements.txt` also installs the
independent `avlite-signal-perception` Python package.

## Python imports

Import public strategies from the plugin root:

```python
from avlite.plugins.avlite_behavior_hdmap import ParkingTask, PlanningCoordinatorTask
```

Configuration class names and the recommended profile are unchanged. Internal
modules now live in the directories above; for example, `coordinator.py` is
`execution/coordinator.py` and `sensor_perception.py` is `perception/detector.py`.
Scripts that imported these internal modules need the new paths. Restart AVLite
after updating so Python does not retain modules from the previous layout.

## Runtime adapters

Use these classes with stock AVLite:

| Class | Added behavior |
| --- | --- |
| `BehaviorPlanningPipeline` | Route, reset and perception updates for all stages |
| `PlanningSyncExecuter` | Startup task event, simulation timestamps and fresh planning/control after handover |
| `BehaviorStanleyController` | Velocity-integral reset and throttle clamp at a zero-speed target |
| `PlanningBasicSim` | Reverse commands, braking before gear changes and a standstill speed floor |

The adapters subclass AVLite interfaces; installation does not modify core
files. On development cores that already provide `DirectionalAckermannCommand`,
the plugin reuses that class for compatibility with the CARLA bridge.

AVLite 0.6.7 scans Python files recursively. Tools skip their imports when loaded
under `avlite.plugins.*`, keeping CLI setup out of strategy discovery. Direct
execution and normal test imports are unaffected. AVLite excludes `tests/`.

## Profiles

`avlite-behavior-hdmap.yaml` is the recommended profile. It registers only this
plugin, refers to AVLite's bundled map assets and uses the install-name sentinel.
The profile generators resolve paths for checkouts outside the managed plugin
directory. Regenerate those profiles after moving a checkout.

The custom parking and planner-selection dialogs require the development GUI.
On stock AVLite, use plugin settings or the task API. CARLA sensor and parking
experiments also require the bridge interfaces described in their setup guides.

## Packaging

Build a standalone archive from the plugin directory:

```bash
python tools/build_community_release.py /path/to/releases/avlite-behavior-hdmap.zip
```

The output must be outside the plugin directory. The builder refuses to replace
an existing file and includes a SHA-256 source manifest. Caches, build outputs,
logs and downloaded ONNX models are excluded; bundled classifier NPZ files are
included. Install dependencies and fetch optional models on the target machine.

The plugin is listed in the community catalog. `.avlite-registry.yaml` contains
its registry metadata. Compatibility results and standalone verification
commands are in [testing.md](testing.md).
