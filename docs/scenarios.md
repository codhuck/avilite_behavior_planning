# Reproducible road scenarios

A scenario describes the inputs and acceptance criteria for a run. The runner
resolves its parameters, creates a fresh AVLite stack, records the physical
trace and evaluates it independently of the planner's reported state.

The first adapter uses BasicSim, `PlanningSyncExecuter`,
`PlanningCoordinatorTask`, the selected behavior/path/velocity stages and
`BehaviorStanleyController`. This is a headless AVLite run. It does not start
another GUI, modify the active profile or require a CARLA server.

## Run and compare

From the plugin checkout, with its dependencies and AVLite installed:

```bash
python tools/run_scenarios.py run data/scenarios/light-and-crossing.yaml \
  --seed 0 --output logs/scenario-one

python tools/run_scenarios.py suite \
  data/scenarios/light-and-crossing.yaml \
  data/scenarios/delayed-observations.yaml \
  --seeds 0 1 2 --strategies road-rules road-tree \
  --output logs/scenario-comparison

python tools/run_scenarios.py replay logs/scenario-one/manifest.json \
  --output logs/scenario-one-replay
```

Output directories must be new. Exit codes are 0 for passed criteria, 1 for a
failed run or runtime error, and 2 for invalid inputs. A suite reports each run
separately and fails if any run fails. It does not count a timeout as success.

`road-rules` and `road-tree` implement the same road rules with different
structure. Comparing them checks equivalent behavior; it is not a comparison
against an independent state-of-the-art planner.

## Included cases

| File | Conditions | Expected result |
| --- | --- | --- |
| `light-and-crossing.yaml` | Red-to-green light, moving pedestrian, parked van | Stop/yield as needed, then finish |
| `delayed-observations.yaml` | Same scene with a delayed observation interval and a dropout | Stop on unusable evidence, then finish |
| `blocked-crossing.yaml` | Stationary pedestrian occupying the crossing | Fail completion/waiting criteria while avoiding contact |

The negative case is intentional:

```bash
python tools/run_scenarios.py run data/scenarios/blocked-crossing.yaml \
  --output logs/blocked-crossing
```

Expect exit code 1. Keeping the car stopped is correct behavior here; the test
shows that a collision-free but unfinished drive is not a successful mission.
Do not relax the criteria simply to turn this report green.

## Format

See the complete [example](../data/scenarios/light-and-crossing.yaml). Version 1
accepts JSON or YAML. Unknown fields, invalid event times, duplicate actor IDs,
unknown parameter references and overlapping faults are rejected.

- `map`, `route`: files relative to the scenario file. The route supplies reference
  geometry and speed, including its terminal braking profile. The runner does
  not silently add a goal stop to a route that requests nonzero terminal speed.
- `dt`, `duration`: fixed simulation timestep and time budget in seconds. Duration
  must be a multiple of dt. The runner does not pace against wall-clock time.
- `actors`: world x/y in metres, heading in radians, speed in m/s, body dimensions,
  and motion start time. Actors exist from the beginning, remain stationary
  before `start_time`, then move at constant velocity. They do not react to ego.
  Supply realistic dimensions for vehicles; the default 0.6 m body is pedestrian-sized.
- `signals`: map signal IDs, initial states, and strictly ordered time/state changes.
- `faults`: `dropout` or `delay`, with start/duration and delay latency in seconds.
- `criteria`: completion, contact-episode limits, non-green signal entries,
  occupied-crossing entries, total stopped time and off-road time.

Parameters use `uniform: [minimum, maximum]` or `choice: [value, ...]`. A value
such as `$pedestrian_speed` references one sampled parameter. Sampling uses a
private seeded generator in sorted parameter-name order. There is no expression
execution. Discrete event times can be supplied with `choice`; actor start times
need not align with dt. Physical actor position is evaluated at each tick from
absolute simulation time, without cumulative integration drift.

## Observation and truth channels

`ScriptedScenarioPerception` is an ordinary AVLite perception strategy. It
publishes signal/crossing observations with `origin="scenario"` and actor states
through the PM. A delay supplies past actor positions and past signal states
with their original timestamp. A dropout publishes an empty fault snapshot.
Neither is labelled as sensor inference. Actual actor motion and the evaluator's
signal timeline continue unaffected by these faults.

The parked van has a physical footprint but does **not** occlude these scripted
observations. This adapter does not render images, run object detection, or model
camera/LiDAR noise. Occlusion and sensor-recognition experiments need another
adapter and cannot be inferred from these results.

## Evaluation and artifacts

Each run writes:

- `manifest.json`: fully resolved input, sampled parameters, copied-asset hashes,
  code/dependency fingerprints and seed;
- `assets/`: exact map and route bytes used for the run;
- `trace.jsonl`: ego pose/speed, physical actors, signal states, behavior reason,
  observation origin/status and timestamp at each simulation tick;
- `result.json`, `report.md`: criteria, failure reasons, metrics, events and a replay
  command. The suite also writes `summary.json` and a comparison report.

Completion requires reaching the route endpoint and stopping. An `ARRIVED`
message alone is insufficient. Contact is checked on oriented 2D footprints at
tick endpoints and intermediate relative-motion samples. A red-light entry is
counted when the front bumper crosses the signal station on a non-green state;
the signal schedule is evaluated at the interpolated entry time. Occupied-crossing
entries use the ego body and pedestrian footprints. Off-road time measures the
body against the driving-lane polygon with a 5 cm numerical tolerance.

Contact and crossing checks are discrete approximations, not continuous
collision detection. Crosswalk occupancy here covers the whole crossing.
Yellow, Off and Unknown count as non-green; this is not a jurisdiction-specific
yellow-light policy. Total stopped time includes required waiting, so its limit
must suit the scene. No STOP-sign dwell metric is implemented in this adapter.

Replay verifies the saved input and assets, then checks source/dependency
fingerprints and the trace hash. Changed code is rejected by default. Use
`--allow-source-change` only for an explicit before/after comparison; the report
records the change and does not claim identical reproduction. A fingerprint
cannot guarantee determinism across all hardware. A trace mismatch under the
same recorded fingerprint fails the replay check.

## Extension boundary

`ScenarioSpec` and parameter resolution do not import CARLA. The orchestration
layer delegates to an adapter with `run(emit)`; that adapter creates its runtime,
emits physical trace records and returns evaluation results. Backend selection
is explicit in `scenarios/runner.py`. Unsupported backends fail before running.

The shipped BasicSim adapter deliberately supports one straight, constant-width
driving lane, mapped traffic lights and crossings. It refuses multi-road maps,
curved/tapered lanes and STOP signs rather than claiming to audit them correctly.
Existing race and parking tools remain available separately; they have not been
migrated into this format. Runs are serial because AVLite settings are process
globals; the adapter restores them after success or failure.

A CARLA adapter, reactive actors, broader road geometry, rendered sensor faults
and automatic optimization toward difficult cases are subsequent extensions.
The shared-core interface remains a proposal. Nothing here changes AVLite core
or claims that the maintainers have accepted a scenario API.
