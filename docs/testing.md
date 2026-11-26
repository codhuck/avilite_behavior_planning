# Tests

Run commands from the plugin directory with the AVLite environment active.
Output filenames and profile names in these examples must be unused.

## Unit tests and style

```bash
python -m pip install pytest ruff
python -m pytest --rootdir=.. --import-mode=importlib -q tests
ruff check .
ruff format --check .
```

Pass `--rootdir=..` explicitly: pytest resolves the root before reading ini
`addopts`. The parent root directory lets pytest import the plugin's root `__init__.py`
correctly. These tests do not require CARLA or a display.

The coordinator acceptance tests ship in this repository; they do not require
AVLite's development test tree. To check the meeting's coordination and sensor
interface requirements separately:

```bash
python -m pytest --rootdir=.. --import-mode=importlib -q \
  tests/test_planning_coordinator.py tests/test_community_runtime.py
```

These checks cover task events, stopped replacement, refusal without changing
the current stack, capability checks, and replacement of a camera/LiDAR source
while keeping the same perception and planner. The two sensor sources are test
bridges with synthetic frames and deterministic person detections; the checks
exercise the interface and fusion, not detector accuracy. Follow with the
BasicSim runs below to verify physical movement around the handover.

## BasicSim

```bash
python tools/coordinator_demo.py --mode both --repeat 3 --output logs/coordination.json
python tools/check_parking.py --parking-type all --output logs/parking
```

Each coordination run drives, rejects a moving handover, brakes, switches
planners and drives again. The race fixture has two scripted rivals. Parking
covers three types with normal, mirrored, rotated, occupied, intrusion/resume
and stale-observation cases.

To check a generated profile through AVLite's loader and executor factory:

```bash
python tools/prepare_coordinator_profile.py --mode road \
  --output ~/.config/avlite/community-road.yaml
python tools/check_coordinator_profile.py --profile community-road \
  --output logs/road-factory.json
```

Use the same `AVLITE_CONFIG_DIR` for both commands if you override the default.
The checker verifies the loaded plugin path, runs 60 ticks and checks that the
vehicle moves. The CI workflow runs these checks against a pinned core revision.

## Declarative scenario checks

```bash
python tools/run_scenarios.py suite data/scenarios/light-and-crossing.yaml \
  data/scenarios/delayed-observations.yaml --seeds 0 1 \
  --strategies road-rules road-tree --output logs/scenario-suite
```

Use [scenario tests](scenarios.md) for replay, the intentionally blocked negative
case, observation fault semantics and adapter limitations.

## Package layout check — 20 November 2026

The reorganized package was checked against the same unmodified AVLite core.

| Check | Result |
| --- | --- |
| Standalone tests | 156 passed |
| Related workspace tests, including sensor and classifier regressions | 176 passed |
| Road/race handovers | 6/6 passed |
| Parking scenarios | 18/18 passed |
| Recommended, generated road and generated race profiles | 3/3 passed |
| CLI help from outside the checkout | 33 commands passed |
| Relative documentation links | 104 checked, none broken |

Reports are in `logs/plugin_layout_20261120/` in the development workspace.
The three added regressions cover default signal geometry after relocation,
profile asset paths and nested-code snapshots. These checks did not launch a
CARLA server or validate new driving behavior.

## Community-plugin release — 20 November 2026

Core: AVLite 0.6.7, commit `1df512a3718d78a4d737e07178a9db03cf5d4f05`.
The core checkout was unchanged. The plugin was copied to a separate directory
and tested with isolated configuration paths. Dependencies were installed in a
separate venv using existing system packages.

| Check | Result |
| --- | --- |
| Standalone tests, including the extracted release archive | 153 passed |
| Related development-workspace regressions | 124 passed |
| Road/race coordination | 6/6 passed |
| Parking scenarios | 18/18 passed |
| Recommended, generated road and generated race profiles | 3/3 passed |
| Ruff lint and formatting | Passed |
| Bundled classifier load from the archive | Passed |

Both race profiles contained two rivals. Profile checks did not import CARLA.
Parking runs recorded no sampled body collisions or drivable-boundary violations;
occupied and stale-input cases remained blocked as expected.

Reports are retained under `logs/community_plugin_20261120/` in the development
workspace. `completion.json` lists the individual files and archive hash. The full
workspace suite was not completed: initial Tk checks lacked display access and
subsequent runs exhausted disk space. The 124 related tests passed afterward.
Interactive GUI acceptance and new live CARLA runs were outside this check.
The [hosted CI run for the community-plugin release](https://github.com/codhuck/avilite_behavior_planning/actions/runs/44646871260) also passed.

## CARLA and earlier experiments

CARLA runners require an idle server and the matching bridge. Some parking
runners replace the map with a test pad; read their setup instructions first.
Results depend on the recorded source revision and configuration.

- [CARLA coordination](validation/coordination-carla-validation.md)
- [Sensor perception](validation/perception.md)
- [Road and race runs](validation/road-and-race.md)
- [Sensor parking](validation/sensor-parking-validation.md)
- [Parking robustness](validation/parking-robustness-validation.md)

BasicSim results do not measure camera accuracy. The reported CARLA experiments
use static surveyed geometry and simulator ego localization; real-road operation
has not been validated.
