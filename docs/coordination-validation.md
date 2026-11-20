# Coordination validation — 18 November 2026

For the subsequent live CARLA tests and fixes, see the
[19 November follow-up](coordination-carla-validation.md).
That follow-up also corrects the earlier ROS exclusion explanation: the plugin
was installed in the user directory; the test's hard-coded checkout path was
wrong. The corrected main selection now passes with that test included.

This record covers the architecture in [task coordination](task-coordination.md).
It does not supersede the separate CARLA perception or parking evaluations.
All changes and artifacts remain local; no push, upload or message was made.

## Evidence

| Check | Result | Evidence in the parent workspace |
| --- | --- | --- |
| Plugin, execution, GUI, registration and parking regression selection | 473 passed, 21.84 s | `logs/coordinator_20261118/regressions_final.txt` |
| Coordinator tests after correcting the demonstration harness | 29 passed, 6.98 s | `logs/coordinator_20261118/coordinator_after_harness_fix.txt` |
| Real BasicSim road handovers | 3/3 passed; tree → rules, same world, movement before and after | `logs/coordinator_20261118/basicsim_checked_speed.json` |
| Real BasicSim race handovers | 3/3 passed; two scripted rivals in each run | Same JSON |
| Actual factory/profile load and 60 execution ticks | Road and race passed; CARLA not imported | `logs/coordinator_20261118/profile-road.json`, `profile-race.json` |
| Canonical frame validation and sensor-provider substitution | Covered by coordinator and sensor regression tests | `test/plugins/test_planning_coordinator.py`, `test_sensor_regulatory.py` |

Road runs travelled 23.24 m before the stopped handover and 16.74 m afterwards.
Race runs travelled 15.18–19.74 m before and 10.39–10.84 m afterwards. The moving
request was made at a measured 0.28 m/s, above the 0.1 m/s replacement threshold,
and rejected in all six final runs. These are short transition tests, not full
laps or a claim of collision-free racing. Identical scenarios were repeated;
these are not six independently designed scenarios.

The provider-substitution test supplies canonical RGB/LiDAR frames from two
different non-CARLA bridge classes to the same perception/planning components.
Person boxes are deterministic test inputs; the test exercises fusion and
interface independence, not detector accuracy. A black frame produces unknown
perception and a stop. Frame layout, calibration, freshness, missing capabilities,
failed constructors and stale/competing handover requests are also tested.

## Failures retained rather than hidden

The initial broad invocation could not access the graphical session and collected
the hyphenated plugin package without importlib mode. It also encountered an
unrelated ROS test requiring an absent external checkout,
`avlite-community-plugins/avlite-executer-ROS2/p41_ros_launcher.py`.
The final command uses importlib collection and display access. Only
`test/plugins/test_ros_step_pace_kwargs.py` is excluded because that ROS plugin is
not installed. Thus 473 passing tests are a defined selection, not the entire
repository or ROS integration. The original output remains in
`logs/coordinator_20261118/regressions.txt`.

One initial repeated race run failed the harness's assumption that a car must
still be moving at tick 160. The planner had stopped and the coordinator correctly
allowed replacement. The corrected harness waits for measured speed above
0.2 m/s before checking refusal. Both the unsuccessful report
`basicsim_final.json` and corrected report `basicsim_checked_speed.json` remain
available. No planner guard was weakened to make the demonstration pass.

## Reproduction

With the matching source tree and dependencies, run:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python avlite-behavior-hdmap/tools/coordinator_demo.py \
  --repeat 3 --output logs/coordinator-new-run.json
python -m pytest -q -o addopts='' --import-mode=importlib \
  test/plugins test/c40_execution test/c60_apps/test_c63_plugin_import_paths.py \
  avlite-behavior-hdmap/tests \
  --ignore=test/plugins/test_ros_step_pace_kwargs.py
```

The broad test selection needs a working graphical session for Tk tests.
BasicSim runs and profile checks do not. For the graphical application, use the
generated profiles described in the coordination guide. Startup callback order
and failure handling are tested; no manual GUI driving session was performed in
this follow-up.

## Boundaries

There was no new CARLA driving run or real-camera evaluation in this follow-up.
The road fixture has no traffic signals or pedestrians. BasicSim's agent states
are simulator-provided and are not used as evidence of sensor-based perception.
Real deployment still requires calibrated, synchronized frames, localization,
actual recordings and independent accuracy/safety evaluation. Full planner-pair
replacement supports a stationary `SyncExecuter`; moving or concurrent full
replacement and automatic map/world migration are deliberately unsupported.
