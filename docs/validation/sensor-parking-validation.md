# Multi-LiDAR parking validation

The later [robustness validation](parking-robustness-validation.md) records new
geometry configurations, moving pedestrians, range noise, retained failures and
subsequent fixes. Results below describe the earlier static experiments.

These experiments use CARLA 0.9.16 physics and ray-cast sensor returns on a
generated flat pad. They are development/acceptance checks, not a held-out
conference benchmark. The vehicle is an Audi A2, with two stationary neighbouring
vehicles. Actor boxes are evaluation labels only; the planner receives LiDAR
occupancy. Pose, bay geometry and ground elevation are supplied by the simulator
or survey. There are no moving pedestrians in this experiment.

## Implementation under test

The [method](../sensor-supported-parking.md) combines nine calibrated simultaneous
LiDARs, additional support requirements near hazards, short executable path
prefixes, speed selection and phase-aware observation moves. Current ego-body
occupancy is a separate initial-condition exception, not observed free space.
The controller uses explicit gear changes at rest, overspeed integral release
and measured-acceleration feedback.

## Development results

These runs span several code revisions. Do not combine them into an independent
success-rate estimate or describe them as repeated trials of one fixed policy.
Paths below are relative to the parent AVLite workspace.

| Recording | Result | Position / heading error | Collision contacts |
| --- | --- | --- | --- |
| `logs/visibility_carla_rig_v6` | Five-device layout stopped 4.16 m from goal | Incomplete | 0 |
| `logs/visibility_carla_entry_v7` | Five-device entry remained blocked | Incomplete | 0 |
| `logs/visibility_carla_entry_v8` | Nine-device entry had no useful selected view | Incomplete | 0 |
| `logs/visibility_carla_entry_v9` | Entry-only parking completed | 9.8 cm / 0.73° | 0 |
| `logs/visibility_carla_full_v10` | Full forward parking; observation blackout and recovery | 9.9 cm / 1.20° | 0 |
| `logs/visibility_carla_reverse_v11` | Full reverse parking completed | 6.7 cm / 0.63° | 0 |
| `logs/visibility_carla_parallel_v12` | Full parallel parking completed | 9.7 cm / 2.69° | 0 |

Earlier `rig_v1`–`rig_v3` also stopped. `rig_v4`, `rig_v5` and `parallel_v11`
were deliberately interrupted while debugging; their checkpoints are not
successful or completed trials. The v9 entry run starts near the bay and cannot
substitute for a complete initial approach. All saved failures remain in logs.

The forward v10 drive took 86.4 simulated seconds, reverse v11 49.7 s and
parallel v12 167.9 s. Parallel parking is still slow. The v10 run discarded
observations for ten frames at 0.1 s per frame: motion certificates were absent
throughout the blackout, the vehicle stopped, and execution resumed after
fresh observations returned. This is a controlled software observation blackout,
not proof of recovery from every physical sensor fault.

## Fixed-code acceptance

All three trials completed on the same final runtime implementation:

| Type | Position error | Heading error | Simulated time | Max false-free box overlap |
| --- | --- | --- | --- | --- |
| Perpendicular forward | 0.0974 m | 1.378° | 119.2 s | 0.8244 m² |
| Perpendicular reverse | 0.0990 m | 0.657° | 72.7 s | 0.5513 m² |
| Parallel | 0.0975 m | 2.853° | 163.5 s | 0.3900 m² |

Each trial recorded zero collision contacts and zero body-boundary violations.
Each ten-frame observation blackout removed all motion certificates; speed
fell below 0.01 m/s within one 0.1 s sample. Entry speeds differed, so these
three measurements are not a braking performance envelope. Every issued audit
certificate used a matching observation/check timestamp.

These final runs used parking prefixes and adaptive speed; none entered
`PARKING_OBSERVE`. They therefore validate the sensor/prefix path to completion,
not an independent benefit from active viewing. The near-bay development entry
experiments exercised that mechanism, but a matched active-view ablation is
still required. Parallel parking also recorded 33 blocked frames outside the
injected blackout, plus seven gear-shift frames.

The final three-mode run is saved in `logs/parking_sensor_acceptance/results.json`.
Its machine-readable summary and plots are generated in
`logs/parking_sensor_acceptance_report/` by the command below. Check `passed`
and `total` in `report.json`; checkpoints are explicitly incomplete and are
never counted by the report generator. Source hashes are recorded at report
generation time, so historical development runs must retain their revision
qualification above.

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python avlite-behavior-hdmap/tools/check_parking_carla.py \
  --sensor --active --rig --rig-layout surround --channels 64 --dt .1 \
  --blackout-start 100 --blackout-frames 10 \
  --output logs/parking_sensor_acceptance

MPLCONFIGDIR=/tmp/itsc-mpl \
python avlite-behavior-hdmap/tools/report_sensor_parking.py \
  logs/parking_sensor_acceptance --output logs/parking_sensor_acceptance_report
```

Use an exclusive idle CARLA server. The runner rejects a world containing
vehicles, creates its test pad and actors, then cleans up and restores the map.
It does not open AVLite or demonstrate a full GUI execution session.

### Conservative policy comparison

`logs/parking_sensor_conservative/results.json` records the same three initial
poses, bays, static vehicles and nine-device fusion with the ordinary
`ParkingPlanner` instead of `VisibilityParkingPlanner`. All three remained
`PARKING_BLOCKED`, with zero collisions: **0/3 completions versus 3/3** for the
proposed variant. The conservative run did not receive the injected blackout.
Its process exit code 1 reports unsuccessful parking, not a harness crash.

This is a bundled-policy comparison: search through unknown future space,
prefix checking, adaptive speed and observation selection differ together.
It does not isolate each mechanism, establish statistical significance, or
prove that active viewing caused the final successes. Reproduce the control:

```bash
python avlite-behavior-hdmap/tools/check_parking_carla.py \
  --sensor --rig --rig-layout surround --channels 64 --dt .1 \
  --output logs/parking_sensor_conservative
```

## Additional checks

- `logs/parking_support_basicsim/results.json`: 18 regression cases, including
  occupied bays, temporary intrusion and stale observations. These use the
  BasicSim model and do not validate CARLA pedestrian perception.
- `logs/parking_bridge_rig/results.json`: ten actual RGB/nine-LiDAR bridge
  frames; stopping one device prevents publication of an incomplete frame.
  Owned actors are cleaned up. A callback fault requires bridge reset.
- `logs/parking_sensor_replay_parallel/results.json`: 168 counterfactual probe
  decisions on a recorded initial cloud set, varying device loss, timestamp
  validity, support count and requested speed. These are not 168 parking drives.
- Unit/integration regression command covers the plugin, regulatory perception,
  executers, plugin loading and CARLA synchronization: 329 tests passed.

Run the regression suite from the parent AVLite checkout with its compatible
Python environment:

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python -m pytest avlite-behavior-hdmap/tests \
  test/plugins/test_signal_distribution.py test/plugins/test_traffic_light_vision.py \
  test/plugins/test_sensor_regulatory.py test/plugins/test_hdmap_behavior_plugin.py \
  test/c40_execution test/c60_apps/test_c63_plugins.py \
  test/c60_apps/test_c63_plugin_import_paths.py avlite-bridge-carla/test/test_sync.py \
  --rootdir=. -q -o addopts='' --import-mode=importlib --tb=short
```

## Limitations that affect interpretation

Collision-free trajectories do not establish perfect perception. Maximum
observed-free overlap with evaluator vehicle boxes was 0.747 m² in forward v10.
Boxes are conservative labels, but nonzero overlap must still be reported.
Ground returns beneath objects and correlated device errors remain unresolved
limitations of this projected occupancy representation. The plots include this
metric rather than presenting successful motion alone.

Low-speed control still oscillates in CARLA, particularly during parallel
parking. Completion and position tolerance do not establish comfortable motion
or a validated braking bound. The current acceleration-to-throttle conversion
and clearance assumptions need separate vehicle/control characterization.

Nine noiseless devices are an experimental coverage intervention. No claim is
made that nine devices are necessary, that fewer cannot work, or that this
layout transfers to a physical vehicle. Trials with noise, calibration errors,
slopes, moving hazards, varied vehicles, unsurveyed spaces and held-out layouts
are still needed. The current evidence supports a working research mechanism;
it does not establish formal safety, statistical superiority or paper readiness.
