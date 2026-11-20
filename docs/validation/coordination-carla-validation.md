# Coordinator follow-up — 19 November 2026

**ROS test correction, later on 19 November:** the earlier missing-plugin
diagnosis below was incorrect. The plugin was already installed under
`~/.local/share/avlite/plugins/avlite-executer-ROS2` and registered in the user
profile. The test hard-coded a repository-local clone path. It now uses
`PluginPaths.resolve`, retaining a hard failure if the plugin really is absent.
The same main selection without the exclusion passed **632 tests** (22.45 s).
Plugin registration and a real ROS 2 Humble local publish/subscribe exchange
also passed; this is not an end-to-end ROS driving trial. Evidence:
`logs/ros_plugin_20261119/{regressions-no-exclusions.txt,import.json,runtime.json}`.
The previously frozen archive and earlier logs retain their historical results.

The [18 November checks](../history/coordination-validation.md) covered BasicSim and
interfaces. This follow-up adds executed CARLA handovers with real RGB/LiDAR
acquisition and the trained RGB signal backend. All artifacts remain local.

## Changes

- Perception handover checks the current bridge speed as well as the PM snapshot.
  A paused executor with an outdated zero-speed snapshot cannot replace perception
  while the simulated vehicle is moving. Non-finite bridge speed also rejects.
- Start/resume relinquishes the coordinator's reported ownership when another
  task has replaced either the global or local planner, without replacing that
  task's planners.
- Transition records expose `road` or `race` rather than the generic `configured`
  label of a local pipeline.
- A reproducible live checker generates its own isolated profile, preserves
  failed outputs, records source hashes, and cleans up test actors. Environment
  initialization happens before output creation because core may re-execute
  Python once when sourcing an installed ROS distribution.

## Executed CARLA trials

Server/client: CARLA 0.9.16, Town10HD_Opt, synchronous 0.05 s steps during each
run. Camera: 1280 × 720 RGB; LiDAR: bridge's canonical non-semantic point cloud.
Initial configuration: `road-tree`; stopped replacement: `road-rules`.

| Run | Ticks | Speed at refused moving request | Distance after recovery | Collision events | Wall time |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 334 | 0.826 m/s | 5.018 m | 0 | 44.85 s |
| 2 | 332 | 0.825 m/s | 5.033 m | 0 | 44.06 s |
| 3 | 333 | 0.806 m/s | 5.081 m | 0 | 42.57 s |

All three runs passed on the same frozen runtime sources. These are repetitions
of one short handover scenario, not three independent traffic scenarios. The
vehicle drives six metres before explicit harness braking. Replacement happens
after both measured speeds are below 0.02 m/s. The harness does not teleport the
vehicle or use autopilot. Invalid strategy selection is rejected; repeated Start
preserves the installed planner. The vehicle and detector objects remain the
same across handover.

Each run injects the following failures into acquired frames at the bridge
boundary, then restores a real frame:

| Injected fault | Required diagnostic | Planner result |
| --- | --- | --- |
| Black RGB image | `unusable_image` | `PERCEPTION_STOP` |
| Missing LiDAR points | `missing_or_unsynchronised_sensors` | `PERCEPTION_STOP` |
| Camera timestamp offset by 0.2 s | `missing_or_unsynchronised_sensors` | `PERCEPTION_STOP` |
| Zero camera intrinsic matrix | `invalid_calibration` | `PERCEPTION_STOP` |

All 12 injections produced the expected diagnostic and stop state. Every
immediately restored frame was accepted as `ok`, and each run subsequently drove
more than five metres. Faults are introduced around standstill; this does not
measure braking distance after a high-speed sensor failure.

The detector performs actual inference. Dynamic ground-truth perception and
the CARLA regulatory observer are replaced by failing traps, and only the
`LOCALIZATION` world stack capability is enabled. Static map/signal geometry and
simulator ego localization remain inputs. No new traffic-light recognition
accuracy or pedestrian recall claim follows from these handover trials.

## Regression selection

- Main stack, plugin, Tk, profile and execution checks: **631 passed** (19.83 s).
- CARLA bridge's fake-server tests: **57 passed** (1.70 s), separately collected
  with its directory on `PYTHONPATH`.
- Focused planning/behavior checks during development: 121 passed; these overlap
  the main selection and are not added to its total.
- Ruff lint and format checks passed for the behavior plugin.

One unrelated test, `test/plugins/test_ros_step_pace_kwargs.py`, remains excluded:
the external `avlite-executer-ROS2` checkout it reads is absent. This is not a
claim that every test or every optional ROS integration passed.

The first combined collection failed because the bridge's profile test imports
`settings` directly. Running the bridge selection with its documented import
path resolves collection; no failing test was deleted. The first live checker
attempt also caught the ROS re-execution/output-directory collision, corrected
before driving. These original console logs are retained.

## Reproduction and evidence

Follow [the live checker instructions](../task-coordination.md#live-carla-regression).
For the regression selections:

```bash
python -m pytest -q -o addopts='' --import-mode=importlib \
  test/plugins test/c40_execution test/c60_apps avlite-behavior-hdmap/tests \
  --ignore=test/plugins/test_ros_step_pace_kwargs.py
PYTHONPATH="$PWD/avlite-bridge-carla:$PWD" \
python -m pytest -q -o addopts='' --import-mode=importlib avlite-bridge-carla/test
```

The main selection requires a graphical session for existing Tk tests. Set
`OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1` to keep resource use bounded.

Parent-workspace evidence is under `logs/coordinator_20261119/`:

- `live-final/result.json`: run outcomes, transitions and source SHA-256 values.
- `live-final/run-{1,2,3}/frames.json`: per-tick state, sensor diagnostics and speed.
- `regressions-final.txt` and `bridge-tests.txt`: test results.
- `server_after.json`: no remaining test actors; the original asynchronous mode,
  unset fixed step and rendering setting restored.

The existing CARLA server was reused and left running; no second simulator or
viewer was launched. Real-camera/vehicle validation, arbitrary traffic safety,
moving full-stack replacement and threaded executors remain outside this check.
