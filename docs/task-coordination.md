# Task coordination

`PlanningCoordinatorTask` selects a global planner and a local planning pipeline
through AVLite's task runner. It handles startup selection and explicit requests
while keeping the current world bridge, perception and controller.

## Configurations

| Name | Global planner | Behavior | Path | Velocity |
| --- | --- | --- | --- | --- |
| `road-rules` | `HDMapLaneFollowingPlanner` | `HDMapBehaviorPlanner` | `ReferencePathPlanner` | `HDMapRuleVelocityPlanner` |
| `road-tree` | `HDMapLaneFollowingPlanner` | `HDMapBehaviorTreePlanner` | `ReferencePathPlanner` | `HDMapRuleVelocityPlanner` |
| `race` | `GlobalCenterlineRacePlanner` | `RaceBehaviorPlanner` | `RacePathPlanner` | `RaceVelocityPlanner` |

All three use `BehaviorPlanningPipeline`. The default `auto` selection resolves
to `road-tree` for `HDMap` and `race` for `RaceMap`. Other map types are rejected.
Road/race transitions within one map are not implemented.

On stock AVLite 0.6.7, select `PlanningSyncExecuter`. The generated BasicSim
profiles also select `BehaviorStanleyController` and `PlanningBasicSim`.
See [runtime adapters](community-plugin.md#runtime-adapters). The local core also
supports ordinary sync and paused threaded execution through the
[executor transaction API](executor-compatibility.md).

## Startup

Add one `PlanningCoordinatorTask` to `c40_execution_tasks` and set
`coordinator_initial_strategy` in plugin settings. The executor emits
`ExecutionStarted` before its first enabled task tick or worker launch.
The legacy stock adapter supplies that lifecycle for AVLite 0.6.7.

The coordinator selects once. Resuming preserves dwell timers and leaves a
stack installed by another task, such as parking, in place. Reset clears the
selection history. A failed initial selection raises before driving starts.

## Request a handover

Call from a synchronous execution task or from the UI while paused. The vehicle
must be stopped; pausing execution alone does not satisfy the speed checks.
Threaded executors must also finish joining every worker.

```python
from avlite.plugins.avlite_behavior_hdmap import PlanningCoordinatorTask

coordinator = next(
    task for task in executer.task_runner.tasks if isinstance(task, PlanningCoordinatorTask)
)
coordinator.request(executer, "road-tree")
```

Pass `global_plan=route` to replace the route as well. Otherwise the current
route is copied. During `TaskRunner.step`, the request is queued until the end
of the task cycle. Outside a cycle, dispatch is immediate. Check the outcome
event when submitting a queued request.

Before installing the candidate, the task checks:

- The map and current planner still match the request.
- The configuration matches the map and all named stages are registered.
- Ego pose is finite; perception and bridge speeds are both at most 0.1 m/s.
- The route is finite and passes within 5 m of the current position.
- Required sensors/capabilities and the first local plan are valid.

A successful handover resets controller state, clears the held command and
forces fresh planning/control on the next enabled tick. Keep both stages enabled
when resuming. A rejected request keeps the current planners and observations. Candidate
construction uses an isolated PM snapshot.

`last_result` contains the latest outcome; `history` retains up to 100 entries.
The task emits `PlanningStrategySelected` or `PlanningStrategyRejected` for
other tasks to consume. A rejection reports the reason; it does not apply brakes
or restart execution. Threaded requests during execution are rejected; request again after stopping
and joining workers.

## Add a configuration

Register the component classes with AVLite, then extend the coordinator before
execution:

```python
from avlite.plugins.avlite_behavior_hdmap import PlanningConfiguration

coordinator.configurations["my-road"] = PlanningConfiguration(
    map_mode="road",
    global_planner="MyRegisteredGlobalPlanner",
    local_planner="BehaviorPlanningPipeline",
    behavior="MyRegisteredBehavior",
    path="ReferencePathPlanner",
    velocity="HDMapRuleVelocityPlanner",
)
```

Global planners accept a map. Local pipelines accept
`(route, perception_model, setting)`. Missing stages reject the request. Candidate
constructors and replanning callbacks must not mutate unrelated executor state.

`GlobalPlannerDispatcher` and `PlannerDispatcher` remain available for profiles
that select directly by map type. `PlannerSwitchTask` provides the lower-level
handover through `SelectPlanningStack`.

## Profiles

From the plugin directory:

```bash
python tools/prepare_coordinator_profile.py --mode road \
  --output ~/.config/avlite/coordinator-road.yaml
python tools/prepare_coordinator_profile.py --mode race \
  --output ~/.config/avlite/coordinator-race.yaml
```

Select the generated profile in AVLite, reload and press Start. Generators
resolve local paths and refuse to overwrite existing files. Regenerate after
moving a checkout. Use `AVLITE_CONFIG_DIR` consistently if set.

The custom strategy chooser belongs to the development GUI. With stock AVLite,
use `coordinator_initial_strategy` for startup selection or the task API for a
handover. The Town10 profile generator uses `road-rules`, or `road-tree` with
`install.py --behavior-tree`; both retain the selected perception backend.

## Sensor input

Behavior consumes `PerceptionModel` observations. The bridge converts sensor
messages into AVLite's `SensorFrame`, `Camera` and `Lidar` types:

| Input | Format |
| --- | --- |
| RGB | Rectified `(H, W, 3)` `uint8`, RGB order, matching pinhole intrinsics and dimensions |
| LiDAR | `(N, 4)` `float32`: x, y, z in metres, then intensity |
| Mounts | Device-to-body transforms in `base_to_sensor`; optical camera axes are right/down/forward |
| Timing | Acquisition stamps in a common clock, explicit primary device names and fresh frame objects |

The current detector requires RGB/LiDAR stamps within 0.1 ms, at least 100 finite
LiDAR points, valid calibration and bounded frame age. Asynchronous sensors need
upstream synchronization. Signal association also requires an HD map, static
signal-head geometry and an acquisition-aligned ego pose. Malformed, stale or
missing inputs produce a stop; dynamic simulator queries are not used to fill
missing observations. See [RGB/LiDAR perception](perception/sensors.md).

`PerceptionSwitchTask` separately handles detector replacement. It requires a
paused executor and finite speeds at most 0.02 m/s in both perception and bridge
state. Its API is described in [architecture](architecture.md#replacing-perception).

## Tests

[Testing](testing.md) covers BasicSim handovers and profile loading.
[CARLA coordination validation](validation/coordination-carla-validation.md) records the
sensor fault-injection runs. BasicSim checks exercise task integration;
classifier accuracy is covered by the separate perception tests.
