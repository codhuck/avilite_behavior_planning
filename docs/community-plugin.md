# AVLite integration

Tested with AVLite 0.6.7 at commit
`1df512a3718d78a4d737e07178a9db03cf5d4f05`.

## Plugin layout

The repository root is the plugin package. `__init__.py` exports the strategies;
`settings.py` defines their settings. AVLite registers the subclasses when it
loads the package.

| Path | Contents |
| --- | --- |
| `coordinator.py`, `switch_task.py` | Configuration selection and stopped handovers |
| `hdmap_behavior.py`, `tree_planner.py`, `global_planner.py` | Road planning |
| `race_behavior.py`, `race_traffic.py` | Race planning and scripted traffic |
| `parking.py`, `parking_task.py` | Parking planner, controller and execution task |
| `sensor_perception.py`, `avlite_signal_perception/` | AVLite detector and independent classifier package |
| `runtime.py`, `commands.py` | Execution, control and simulation adapters |
| `data/` | Routes, signal survey and coordination fixtures |
| `tools/`, `tests/` | Scenario runners and regression tests |

The registry name is `avlite-behavior-hdmap`; the import namespace is
`avlite.plugins.avlite_behavior_hdmap`. `requirements.txt` also installs the
independent `avlite-signal-perception` Python package.

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

The repository is not yet listed in the community catalog. The proposed entry
is in `.avlite-registry.yaml`; submission requires the tested revision to be
published first. Compatibility results are in [testing.md](testing.md).
