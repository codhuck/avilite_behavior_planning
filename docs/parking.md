# Parking

`ParkingTask` supports three parking types:

| Type | Motion |
| --- | --- |
| `perpendicular_forward` | Forward entry into a perpendicular bay |
| `perpendicular_reverse` | Reverse entry into a perpendicular bay |
| `parallel` | Reverse entry and forward/reverse correction between parked vehicles |

`ParkingPlanner` generates the manoeuvre; `ParkingController` follows it with
stops before direction changes. Parking starts on an explicit task request.

## Requirements

On stock AVLite 0.6.7, use `PlanningBasicSim` and `PlanningSyncExecuter`.
CARLA parking requires a bridge with directional-command support and
`parking_geometry()` for wheelbase and rear-axle offsets.

## Configuration

From the plugin directory:

```bash
python tools/prepare_parking_profiles.py --output-dir ~/.config/avlite
```

This creates `parking-perpendicular_forward`, `parking-perpendicular_reverse`
and `parking-parallel`, with their scene files under `parking-assets/`. Existing
profiles and assets are not overwritten. Use `AVLITE_CONFIG_DIR` as the output
directory if you override AVLite's default config location.

The generated profiles use `PlanningBasicSim` and simulator occupancy.
`ParkingTask` adds the neighbouring parked vehicles and removes its own vehicles
on reset. Each profile supplies an approach pose for its parking type.

## Select a manoeuvre

With the executor paused and the vehicle stationary, find its `ParkingTask`
and call:

```python
from avlite.plugins.avlite_behavior_hdmap import ParkingTask

task = next(t for t in executer.task_runner.tasks if isinstance(t, ParkingTask))
spaces = task.available_spaces(executer, "parallel")
if spaces:
    task.start_parking(executer, "parallel", spaces[0])
```

Choose a scene that permits the requested type. Selection does not teleport the
vehicle. Resume execution to follow the path.

The development GUI also provides a **Parking…** dialog: select the profile,
reload, press **Stop**, choose the space and type, then press **Start**. That
dialog is not part of stock AVLite. **Cancel parking** latches a stop until a new
request; an obstacle clearing does not cancel this latch.

## Space and occupancy contracts

Scene JSON uses `version: 1`, a `drivable` polygon and a `spaces` mapping. Each
space contains a `bay` polygon, allowed `types`, and a `goals` mapping from type
to `[x, y, yaw]`. Coordinates refer to the **body centre** in the localization
frame; yaw is in radians. See the generated JSON for a complete example.
`parking_scene_file` selects this file in plugin settings.

By default occupancy must come from `pm.parking_observation`, a
`ParkingObservation(polygons, stamp)` published by a perception provider.
`None` means unknown; an empty tuple means the relevant area was actually
observed clear, not merely that an object detector found nothing. The acquisition
stamp and `pm.simulation_time` must share a clock. Observations older than 0.3 s
or from the future command a stop. The provider must include all relevant
obstacles, including pedestrians and vehicles.

`parking_simulator_obstacles: true` is a **BasicSim-only demonstration option**.
The CARLA physics tests below also use known simulator obstacles for independent
planning/control validation. These are not tests of camera parking-space
recognition or sensor-derived free space. Existing city RGB/LiDAR outputs do not
yet provide the complete parking occupancy contract automatically.

## Planning and control

The planner first considers curvature-bounded geometric approaches. Parallel
parking can use a bounded hybrid search with forward/reverse circular primitives
and direction-change costs. It searches outward from the constrained bay and
reverses the resulting motion. Search exhaustion is an explicit refusal, not
proof that no conceivable manoeuvre exists.

Collision checks use oriented vehicle footprints, dense geometric samples and
swept primitive hulls. Planning for CARLA includes an additional clearance margin
to accommodate tracking error; runtime clearance thresholds are not relaxed.
The remaining path and current body are rechecked on fresh observations. A newly
occupied route stops motion and resumes only when observed clear. Tracking error
over 0.4 m blocks movement.

CARLA trajectories account for rear-axle geometry while space goals and collision
checks remain body-centred. The directional command separates reverse gear from
braking and uses wheel-angle steering in radians. Legacy road-control commands
retain their original meaning. The bridge refuses gear changes above 0.02 m/s;
the planner also stops at each cusp and waits 0.5 simulation seconds before the
next segment. Speed targets are capped at 0.6 m/s in BasicSim and 0.4 m/s with
rear-axle geometry.

`PARKED` is latched only when the complete body is inside the bay, the final
position is within 0.2 m, heading within 5 degrees and speed below 0.02 m/s.
Reset or route replacement invalidates the request and occupancy snapshot.
Cancellation is latched. New manoeuvres require a stopped vehicle and synchronous
handover; unsupported bridges are rejected.

## Reproduce validation

From the plugin directory:

```bash
python -m pytest tests/test_parking.py -q
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python tools/check_parking.py --parking-type all --output logs/parking
```

Choose one type with `--parking-type parallel`, `perpendicular_forward`, or
`perpendicular_reverse`. The runner writes per-step telemetry and a plot.

For CARLA, use an idle, exclusively owned 0.9.16 server on `localhost:2000`:

```bash
python tools/check_parking_carla.py --repeats 2 --output logs/parking_carla
```

This command replaces the map with a generated flat test pad, spawns an Audi A2,
two neighbouring vehicles and a collision sensor, and drives through the normal
CARLA vehicle-control API. It restores the original packaged map afterward;
for a previously generated OpenDRIVE world it returns to Town10HD_Opt. Do not run
it against another active session. It verifies physical controller behaviour,
not a complete city sensor pipeline or the graphical AVLite executor.

## Results — 12 November 2026

- BasicSim: 18 scenario combinations (six per type): normal, mirrored, rotated,
  occupied, obstacle introduced/removed, and stale observations.
- Actual AVLite executor: all three generated profiles reached `PARKED`.
- Actual Tk dialog: space/type selection and stopped planner/controller handover
  passed.
- CARLA: 6/6 completed manoeuvres, including a second run with a 0.1 m lateral
  and 1-degree heading perturbation. Maximum final error was 0.09931 m and
  1.333 degrees; zero collision events and body/boundary violations.

Evidence in the parent workspace: `logs/parking_acceptance/results.json` and
`logs/parking_carla_acceptance/results.json`. Earlier failed CARLA iterations
are retained separately and are not counted as passing results.

## Scope

The tested operating envelope is low-speed parking into known rectangular spaces
with sufficient manoeuvring room. It does not include automatic space detection,
real-road validation, arbitrary tight spaces, sloped surfaces, or prediction of
unobserved moving obstacles. Sampled checks are not a continuous-time safety
proof, and an obstacle appearing inside the available braking distance cannot be
guaranteed avoidable. CARLA tests use one vehicle model on a flat generated pad;
other vehicles and surfaces require their own validation.

## Experimental LiDAR occupancy

`parking_lidar_occupancy: true` selects a separate, opt-in flat-ground model.
Set `parking_ground_z` to the surveyed ground elevation and disable
`parking_simulator_obstacles`. The planner consumes the current canonical
`SensorFrame.lidar`; it does not read actor boxes. A calibrated mount and an
acquisition-aligned ego pose are required. Frame and LiDAR timestamps must share
a clock. Cached frames retain their original execution acquisition time and
cannot refresh a stale observation. Out-of-order acquisitions cause a stop.

Ground-return cells can be free; unseen cells remain blocked. Connected
above-ground surfaces are projected into filled obstacle regions to reduce
holes caused by returns underneath cars. This is not a guarantee: later
multi-sensor trials still show nonzero evaluator-box overlap. The model
assumes a level surveyed surface and does not establish real-world traversability
from reflectivity or classify parking markings. Thin objects between beams,
slopes, negative obstacles and calibration uncertainty need further validation.
It is disabled by default.

`ParkingTask.available_spaces(executer, parking_type)` filters **surveyed**
candidates by fresh observed clearance. This is not detection of new spaces or
painted boundaries, and this helper is not an automatic search/selection UI.

Run the observability probe on an exclusive idle CARLA server:

```bash
python tools/check_parking_lidar_carla.py
python tools/check_parking_lidar_carla.py \
  --channels 128 --points-per-second 1200000 \
  --output logs/parking_lidar_dense_probe
```

The tool temporarily replaces the map with a flat pad, measures five LiDAR
frames, removes its actors and reloads the original packaged map. It does not
perform a parking drive. Actor geometry is used only for independent evaluation.

On 12 November 2026, 64-channel scans observed 54.8% of the test area as free;
128-channel scans observed 76.2%. Neither confirmed the entire candidate bay as
free, so neither establishes sensor-only parking acceptance. After obstacle-mask
correction, the labelled neighbouring vehicle had 0% free overlap in both
configurations. The initial implementation had approximately 5.6% false free
overlap and was corrected; these are measurements on one flat-pad arrangement,
not detection recall across varied obstacles.

The six successful CARLA drives documented above still use simulator obstacle
geometry. They must not be reported as six sensor-only parking drives. Subsequent multi-sensor work is described in
[sensor-supported parking](sensor-supported-parking.md) and its
[validation record](sensor-parking-validation.md). It adds calibrated fusion,
speed/prefix arbitration and closed-loop drives for all three types. Unsurveyed
space recognition and sensor parking with moving pedestrians remain unvalidated.
