# Drive-to-parking missions

`ParkingMissionTask` follows a prepared approach route, stops the vehicle,
chooses a confirmed free space, and hands control to the parking planner.
The parking type and candidate spaces are supplied by the operator. It does
not detect parking markings or find an arbitrary parking lot.

## Run the BasicSim example

From the plugin directory, in the AVLite environment:

```bash
python tools/prepare_mission_profiles.py --output-dir ~/.config/avlite
python -m avlite
```

The generator creates `mission-perpendicular_forward`,
`mission-perpendicular_reverse` and `mission-parallel`. Select one, reload the
stack, and press Start. Each example drives an eight-metre approach and parks
between two stationary vehicles. Choose a fresh output directory if parking
assets already exist; the generator refuses to overwrite them. With
`AVLITE_CONFIG_DIR`, use that directory instead of `~/.config/avlite`.

Before starting, check the profile without opening a simulator:

```bash
python tools/diagnose_mission.py \
  --profile ~/.config/avlite/mission-perpendicular_forward.yaml
```

The report lists resolved input files and all independent errors it can find:
missing or malformed files, unsupported mission settings, missing space IDs,
invalid bay geometry, route dimensions and an approach point outside the route.
It also rejects a status-output path that would overwrite an input file.
Use `--json` for automation; errors return exit code 1, while warnings alone
return 0. The check does not modify profiles or connect to a world bridge.

Supply a complete YAML profile with explicit execution and plugin settings;
the command does not merge the active user's settings. Mission and scene paths
follow the runtime's working-directory convention. Route paths use AVLite's
user-data and bundled-data lookup. Absolute paths avoid this distinction.
Passing this check does not verify live sensor coverage, third-party bridge
support, braking performance or parking-path feasibility.

Open the read-only decision window in another terminal:

```bash
python tools/mission_monitor.py \
  --status ~/.config/avlite/parking-assets/perpendicular_forward-status.json
```

Change the status filename for another parking type. The window shows the
mission state, reason, selected space, speed, observation age and active brake
holds. Its wall-clock update age distinguishes a current record from a paused
or disconnected process. It never connects to CARLA or sends control commands.
Add `--once` to print the snapshot in a terminal without a graphical display.

## Mission file

```json
{
  "version": 1,
  "approach": [-12.0, 0.0, 0.0],
  "parking_type": "perpendicular_forward",
  "spaces": ["space_a", "space_b"],
  "wait_timeout": 15.0,
  "progress_timeout": 30.0,
  "settle_time": 0.5
}
```

Positions are metres in the localization frame; heading is radians. Space IDs
refer to `parking_scene_file`, using the [parking scene format](parking.md).
Candidates are tried in order. A free bay with an infeasible path is skipped.
The scene must not enable the older `demo` option, which spawns cars only at
manual handover. The example's separate `ParkingDemoTrafficTask` creates its
vehicles before the first approach tick.

Register these tasks with `PlanningSyncExecuter`:

```yaml
c40_execution:
  c40_executer_type: PlanningSyncExecuter
  c40_execution_tasks: [ParkingMissionTask, DecisionStatusTask]
plugins:
  avlite_behavior_hdmap:
    mission_file: /path/to/mission.json
    parking_scene_file: /path/to/parking-area.json
    decision_status_file: /path/to/status.json
```

Keep the existing road planner and controller for the approach. The mission
target must lie on that route. The route should continue beyond the approach
point far enough for the road controller to reach it; do not place its zero-speed
endpoint before the approach point. The example uses `SurveyedRoutePlanner`
and `ReferencePathPlanner` in an empty approach corridor. These fixed-route
planners do not provide obstacle avoidance during the approach. In traffic,
use the road behavior stack and its perception producer instead.

## Observations and handover

There are two observation sources. Select one explicitly:

- `ParkingObservation` supplies complete occupied polygons for the surveyed area
  and a simulation-time acquisition stamp. An empty tuple means observed clear;
  `None` means unknown. Zero detections alone do not establish free space.
- LiDAR mode builds a fresh ground-coverage grid before space selection. Returns
  above or below the surveyed ground band are hazards. Cells without ground
  returns remain unknown. A bay must be covered, including a 0.15 m clearance,
  and the parking path must fit inside the observed traversable area. A clear
  bay alone is insufficient.

To enable LiDAR selection in a mission profile whose bridge supplies 3D LiDAR:

```yaml
plugins:
  avlite_behavior_hdmap:
    parking_lidar_occupancy: true
    parking_simulator_obstacles: false
    parking_ground_z: 0.0  # replace with surveyed elevation in localization coordinates
    parking_multi_lidar: false
    parking_active_observation: false
```

Use `PlanningSyncExecuter` with `ParkingMissionTask`. Remove
`ParkingDemoTrafficTask` when converting a demo profile. The generated BasicSim
profiles do not provide a calibrated 3D ground-return sensor; changing this
flag alone will not make them sensor demos.

The executor passes the same sensor
snapshot used by that planning tick, after localization; the task never fetches
another frame or advances the simulator. The bridge must provide calibrated
LiDAR mounts and a pose aligned with acquisition time. Assembly and acquisition
stamps must match within 10 microseconds, since this adapter has no pose history
for correcting delayed clouds. The first frame establishes the sensor-clock
offset from execution time; this requires a current first snapshot. Subsequent
frames keep that offset. Repeated frames keep their original geometry and age;
missing, out-of-order, future or stale data cannot authorize a handover.

For several synchronized devices, enable `parking_multi_lidar` and configure
`parking_minimum_sensor_support` and `parking_hazard_support_radius` as described
in [sensor parking](parking.md). Device support is counted independently, not
by the number of points. Sparse or occluded ground can prevent selection. The
mission waits at the approach point for a fresh observation; it does not drive
into an unknown area to improve visibility. Active observation manoeuvres
remain a separate manual parking mode and are rejected by mission diagnostics.

The observer survives handover and supplies each parking replan. If observations
age beyond 0.3 seconds, a brake hold also prevents motion when replanning is
paced or disabled. Fresh observations can clear that sensor hold; a mission
already latched `BLOCKED` still requires an explicit retry. The decision output
shows the source, observation age and per-space reasons such as `occupied`,
`unknown`, `stale_lidar` or `accepted`.

`parking_simulator_obstacles: true` is a BasicSim-only substitute for a complete
observation producer. The generated examples use it. They are not LiDAR or CARLA
results. The LiDAR mission tests use calibrated synthetic point clouds and
BasicSim motion; real CARLA sensor validation of this integration is still needed.

At the approach point, the mission applies a braking hold and checks both the
localized and bridge-reported pose. Handover requires speed at most 0.02 m/s
for the settling interval, position within 0.6 m and heading within 10 degrees.
The new parking path is checked before replacing the planner/controller. The
next tick forces fresh planning and control, including with paced execution.

The states are `APPROACH`, `SETTLING`, `SELECT_SPACE`, `PARKING`, `COMPLETE`,
`BLOCKED` and `CANCELLED`. All durations use simulation time. A temporarily
occupied space can become eligible while selecting. Exceeding a wait or
progress timeout latches `BLOCKED`; clearing the obstacle does not restart
the vehicle by itself. Disabling Tasks also latches a stop.

With execution paused and the vehicle stationary, the task API permits an
explicit retry:

```python
mission = next(t for t in ex.task_runner.tasks if isinstance(t, ParkingMissionTask))
mission.retry(ex)
```

`mission.cancel(ex)` cancels and holds the brake. Reload the stack to start a
new mission after cancellation or a changed world/route. Reset restores the
original approach planner before resetting the world. Pause/Start preserves
mission progress. Completion requires the vehicle to be stopped inside the
selected bay, within 0.2 m and five degrees of the goal, and holds the brake.

## Decision output and checks

`DecisionStatusTask` can also be used without a mission. It reports the road
plan's `behavior_state`, `behavior_reason`, rule ID, distance and observation
age. It does not infer why a planner stopped if that planner supplies no reason.
Decision changes appear at INFO level. `ex.decision_status` contains the latest
snapshot; the optional JSON file is replaced atomically, at most every 0.25
simulation seconds except on changes. Use a different file for each process.

```bash
python -m pytest --rootdir=.. --import-mode=importlib -q tests/test_missions.py tests/test_mission_lidar.py
python tools/check_missions.py --output logs/missions.json
python tools/check_mission_profile.py --profile mission-perpendicular_forward \
  --output logs/mission-factory.json
```

The scenario runner covers all three parking types with clear, occupied,
later-cleared, missing and stale observations. It checks physical approach
movement, terminal braking, body collisions and the drivable-area boundary.
It writes trajectories and state transitions alongside the result. CARLA and
real-sensor testing of this mission task remain separate work.

The LiDAR suite separately covers free/occupied/unknown space classification,
calibration transforms, repeated and asynchronous frames, sensor-clock offsets,
independent-device support, occupied-first-choice fallback, unknown paths,
new hazards after handover and braking when replanning is disabled. Its three
complete missions include more than seven metres of approach movement and
check the vehicle body against the drivable boundary on every tick.
