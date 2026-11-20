# Parking validation with new geometry, pedestrians and LiDAR noise

The final protocols are five [regression cases](../experiments/parking_validation_v10_complete.json) and three [previously unused parameter combinations](../experiments/parking_validation_v11_holdout.json). They are reported separately. Historical failures and corrections below remain part of the evidence; passing development reruns do not replace them.

## Final regression results (v10)

All five cases passed on the same frozen runtime, assembled from two disjoint
invocations. These are development regressions, not independent generalization
results. The two difficult parallel cases changed both code and duration limit
relative to v9.

| Case | Result | Position error (m) | Heading error (°) | Simulated time (s) |
| --- | --- | ---: | ---: | ---: |
| Noisy forward perpendicular | Pass | 0.094 | 1.538 | 194.1 |
| Noisy reverse perpendicular | Pass | 0.097 | 0.948 | 119.9 |
| Parallel terminal regression | Pass | 0.094 | 0.790 | 221.7 |
| Noisy parallel follow-up | Pass | 0.095 | 1.629 | 334.2 |
| Noisy parallel search follow-up | Pass | 0.098 | 2.232 | 462.4 |

All five recorded zero collision contacts and body/boundary violations. All four
pedestrians completed continuous crossings; minimum body clearances were
0.870, 1.046, 1.843 and 1.114 m respectively. Saved matching raw clouds contained
351, 330, 106 and 148 above-ground points inside the evaluator walker boxes,
with all nine device timestamps and capture poses matching.

The noisy parallel follow-up recorded **zero commanded blocked-stop seconds near
the crossing pedestrian**, despite stopped samples during other manoeuvre states.
It therefore supports successful parking around a moving observed obstacle,
not a claim that every pedestrian trial forced a yielding stop. The other three
crossing cases recorded 2.3, 2.7 and 1.9 seconds of such commanded stops.

Artifacts are `logs/parking_validation_v10_complete/` (results, protocol,
provenance, source snapshot, hardware and checks) and
`logs/parking_validation_v10_complete_report/` (JSON report, PDF/PNG plots and
raw-cloud audit). Runtime hashes match, coverage is 5/5, and no failed trial was
dropped from either source invocation. Parking remains slow: 119.9–462.4 simulated
seconds, with 448–1413 seconds wall time per case. Maximum false-free overlap
with evaluator boxes is nonzero (0.330–0.579 m²); this measurement is retained
separately from physical contacts and must not be described as perfect perception.

## Previously unused scenarios on frozen code (v11)

Three manually constructed parameter combinations were registered before their
first execution, after the v10 implementation was frozen. All three passed on
that unchanged runtime. `logs/parking_validation_v11_holdout/registration.json`
contains the timestamp, complete case definitions, execution options and source
hashes. Its 25 hashes match the actual protocol and final runtime; all three
planned cases are present. No scenario was substituted or omitted.

| Parking type | Result | Position error (m) | Heading error (°) | Simulated time (s) | Minimum pedestrian clearance (m) |
| --- | --- | ---: | ---: | ---: | ---: |
| Perpendicular forward | Pass | 0.098 | 1.008 | 171.4 | 1.016 |
| Perpendicular reverse | Pass | 0.096 | 1.241 | 113.8 | 1.010 |
| Parallel | Pass | 0.099 | 0.721 | 251.0 | 1.312 |

Every case combined a different start/scene transform and neighbour placement
with 0.01 m range noise, 2% random ray dropoff and a real CARLA walker moving at
0.8 m/s with an eight-second pause. All three walkers completed continuous
crossings. No collision contacts, body/boundary violations or ego/pedestrian
box overlaps were recorded. Commanded blocked-stop time near an unfinished
crossing was 1.8, 1.7 and 1.2 s respectively. None of those commanded stops
coincided with the scripted pause; this is not a claim that the car waited for
the entire pause or crossing irrespective of spatial conflict.

The raw-cloud audit found 332, 329 and 149 above-ground returns inside matching
walker evaluator boxes at the saved 20-second events. All nine device timestamps
and capture poses matched. These boxes belong only to evaluation and scenario
construction; the planner's obstacle input is LiDAR. The matching cloud audit
and full-trajectory continuity checks establish different parts of the evidence.

Artifacts are `logs/parking_validation_v11_holdout/` (registration, protocol,
results, per-frame data, raw clouds and source snapshot) and
`logs/parking_validation_v11_holdout_report/` (JSON report, PDF/PNG figures and
raw-cloud audit). All motion certificates were fresh in these runs. There were
no missing observation frames, so that condition is covered by separate tests,
not demonstrated by these three drives. Wall time was 407–773 s per case.
Maximum false-free evaluator-box overlap remained 0.447–0.506 m².

These are independent of the latest tuning in the limited sense that their
parameter combinations had not previously been executed. They use the same
flat pad, car, surveyed bays and ground-truth localization. Three single runs
are not statistical evidence of generalization to unseen maps or real streets,
and no cross-policy superiority is inferred.

## Completed v7 results

All five planned trials completed, assembled from two invocations with identical
runtime hashes. Three passed. The two failures are retained; an interrupted
checkpoint is not counted as a trial.

| Case | Result | Position error (m) | Heading error (°) | Simulated time (s) |
| --- | --- | ---: | ---: | ---: |
| New noisy forward perpendicular | Pass | 0.095 | 1.249 | 131.8 |
| New noisy reverse perpendicular | Pass | 0.097 | 0.721 | 122.3 |
| Parallel terminal regression | Pass | 0.095 | 1.095 | 224.8 |
| New noisy parallel | Fail; invalid pedestrian encounter | 0.712 | 19.876 | 235.8 |
| Noisy parallel search regression | Fail; blocked | 2.250 | 7.743 | 265.6 |

All five recorded zero collision contacts and zero body/boundary violations.
This does not make the failed parking manoeuvres successful. The noisy parallel
new case spawned the walker inside a neighbouring vehicle: evaluator-box overlap
was 0.0347 m² at the saved 20-second snapshot. At 22 seconds its position jumped
to `(0, 0)`. Its apparent completed crossing is invalid, independently of the
vehicle's missed goal. The other three pedestrian traces remain continuous.

Retained artifacts (relative to the AVLite workspace):

- `logs/parking_validation_release_v7_complete/results.json`: all five outcomes
  and per-frame telemetry, including failures.
- `logs/parking_validation_release_v7_complete/provenance.json`: source runs and
  hashes; `source_snapshot/` retains the runtime files listed in the v7 protocol.
- `logs/parking_validation_release_v7_complete_report/report.json`: complete
  coverage, runtime verification at report generation, stop durations and
  pedestrian-position continuity audit.
- `logs/parking_validation_release_v7_complete_report/trials.pdf`: trajectories,
  speeds, false-free overlap and pedestrian clearance.
- `logs/parking_validation_release_v7_complete_report/pedestrian_cloud_audit.json`:
  synchronized nine-device raw-cloud evidence at the saved events. Counts inside
  pedestrian evaluator boxes were 348, 327, 382 and 194 above-ground points.
  A valid cloud at 20 seconds does **not** validate the later lifecycle of an actor.

## Recovery changes after v7

An exhausted path with an unmet goal, or excessive tracking error, now triggers
a new search from the measured pose after the vehicle stops. The old path is
discarded without relaxing the goal tolerance, collision checks or visibility
gate. Retry admission remains bounded. After two unsuccessful searches, a
stopped vehicle may use a five-second search budget instead of two seconds;
successful generation resets this escalation. Moving vehicles keep the original
budget. This is a soft wall-clock search budget, not a hard real-time guarantee;
synchronous CARLA is paused during computation. A physical-vehicle deployment
would need independent watchdog/braking and asynchronous planning validation.
Offline replay of the failed final snapshots found a replacement for
the missed-endpoint case in about 0.55 s; the other search exhausted two seconds
but found a path with five seconds allowed (about 3.16 s elapsed).

The evaluator now selects a five-metre walking corridor clear of vehicle bodies
with a 0.4 m margin, including the ego vehicle. It refuses impossible staging and
rejects dead/discontinuous walkers. These privileged actor polygons construct
the scenario only and are never passed to the sensor-based planner.

The first recovery invocation, `logs/parking_validation_v8_recovery`, was
incomplete: the continuity check consulted CARLA's new-actor cache before its
first tick and rejected the first encounter; the next spawn failed. This is a
harness failure, not a successful trial. The check now waits for that first
tick, and actor cleanup is flushed before the next case. The subsequent
`logs/parking_validation_v9_recovery` uses the two previously failed cases in
[`parking_validation_v8_recovery.json`](../experiments/parking_validation_v8_recovery.json).
These are regression reruns after development, **not new held-out evidence**.

### Completed recovery reruns (v9)

Both pedestrian encounters were continuous, completed, and supported by raw
LiDAR evidence (161 and 156 above-ground points in the saved evaluator boxes).
Neither parking trial passed: the first reached the 300-second time limit with
1.083 m / 12.260° goal error; the second stopped at 96.3 s with 7.885 m / 6.927°
error. Both recorded zero contacts and body violations. Minimum pedestrian
clearances were 1.479 m and 1.962 m; commanded blocked stops nearby totalled
0.7 s and 1.9 s respectively. See `logs/parking_validation_v9_recovery_report`.

Do not interpret the first case's direction changes as proof of endpoint
recovery: its recorded base reasons were ordinary path following, obstacle checks
and direction-change stops. At the time limit it was still following a
multi-segment manoeuvre. The second case exposed retention of a path made
unusable by newly observed geometry. The adapter now discards that hypothesis
only after a stopped `no_admissible_maneuver` decision with base reason
`obstacle_or_boundary`; usable checked prefixes and moving paths remain intact.

[`parking_validation_v10_duration.json`](../experiments/parking_validation_v10_duration.json)
repeats both cases with that correction and a separately declared 600-second
limit. Because both the implementation and the time limit changed, this is a
development follow-up, not an isolated causal test of the time limit. Its
telemetry additionally records segment indices/counts, geometric endpoints,
steering and acceleration commands. The runner waits for an initial CARLA
snapshot before checking server exclusivity: a newly connected actor cache can
otherwise misleadingly appear empty.

## Development protocol (v2)

The initial fixed configuration was
[`experiments/parking_validation_v2.json`](../experiments/parking_validation_v2.json).
It contains six cases: three new geometry configurations, and three different
configurations with a crossing pedestrian, range noise and random point loss.
Each group covers perpendicular forward, perpendicular reverse and parallel
parking. Start offsets, heading, world rotation/translation and asymmetric
neighbour placement differ from the previous acceptance cases.

The cases were defined before execution of this suite. They remain manually
constructed variations on a flat pad with the same car and surveyed bay sizes,
not held-out real-world maps or a random sample of street conditions. The
suite saves the exact configuration, command options and runtime source hashes
in `protocol.json` before starting. A failed case must remain in the results.

The moving actor is a real CARLA walker controlled by `WalkerControl`. It is
spawned ahead of the ego vehicle after 12 simulated seconds, crosses at
0.6 m/s, pauses for three seconds near the path centre, then completes its
crossing and remains in the scene. The script uses vehicle pose only to stage
the encounter. Walker position and bounding boxes are evaluator-only data;
the planner receives the same LiDAR interface as in the static experiments.
The parking module treats the walker as occupied space; this experiment does
not validate semantic pedestrian classification or pedestrian-intent prediction.

The noisy cases use CARLA ray-cast `noise_stddev=0.01` m and
`dropoff_general_rate=0.02`, with a distinct fixed seed for each of nine devices.
The [CARLA sensor reference](https://carla.readthedocs.io/en/latest/ref_sensors/)
defines range perturbation along each ray and random point dropoff. This is a
modest sensor-noise test, not a physical rain/fog model or a claim of robustness
at arbitrary noise levels. Localization and mount calibration remain exact.

## Failure found and correction

The first diagnostic run produced spurious obstacles from the ego body:
noisy returns extended 6–12 mm beyond the nominal bounding rectangle. Surface
completion inflated them into the stopping envelope, preventing all motion.

`FusedLidarParkingOccupancy.self_surface_tolerance` now defaults to 0.03 m.
This filters near-body **non-ground measurements** only. It does not expand the
self-footprint exception or synthesize free cells. Unknown surrounding cells
remain unknown. The tolerance assumes calibrated body bounds and cannot be
interpreted as reliable perception of objects touching the body. Returns
farther outside the band remain obstacles. Regression tests cover the distinction
and clearing support diagnostics on missing observations.

The two diagnostic pilots are separate from the fixed suite. Both were
deliberately interrupted and are **not successful parking trials**:
`logs/parking_validation_pilot` and `logs/parking_validation_pilot_v2`.
The second pilot verified sensor detection, stopping and resumed motion after
the pedestrian crossed; the final suite adds the three-second pedestrian pause.

## Measurements and acceptance

A case passes only when the car reaches the requested pose within 0.2 m and
5°, records no collision contact and no body/boundary intersection. A pedestrian
case additionally requires completed pedestrian crossing, observed occupied
evidence intersecting its evaluator box, at least one stopped near-pedestrian
frame, and zero ego/pedestrian box overlap. Stop duration and clearance are
reported separately; a pass is not a formal proof of yielding in every scene.

Telemetry includes pedestrian trajectory, clearance, overlap, occupied evidence,
pause state and motion decisions. Event LiDAR clouds are saved at 2, 5 and 8 s
after spawning, with capture-time pose, mount and timestamp. These supplement
the original cloud, coverage geometry snapshots and collision measurements.

## Reproduce

Use an exclusive idle CARLA 0.9.16 server and the compatible AVLite environment.
The idle world must still publish snapshots (normally asynchronous mode); a
paused synchronous session is refused rather than silently replaced. Close
other controlling clients before running the suite.

```bash
for parking_suite in v10_complete v11_holdout; do
  PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python avlite-behavior-hdmap/tools/check_parking_carla.py \
    --sensor --active --rig --rig-layout surround --dt .1 \
    --suite "avlite-behavior-hdmap/experiments/parking_validation_${parking_suite}.json" \
    --output "logs/parking_${parking_suite}_reproduction"

  MPLCONFIGDIR=/tmp/itsc-mpl \
  python avlite-behavior-hdmap/tools/report_sensor_parking.py \
    "logs/parking_${parking_suite}_reproduction" \
    --output "logs/parking_${parking_suite}_reproduction_report"

  python avlite-behavior-hdmap/tools/audit_parking_events.py \
    "logs/parking_${parking_suite}_reproduction" \
    --output "logs/parking_${parking_suite}_reproduction_report/pedestrian_cloud_audit.json"
done
```

Inspect each report's `suite_complete` and `passed` fields. The loop deliberately
continues across failed suites so that a failure does not omit later cases.

The runner restores the previous map and removes its actors. `--case ID` selects
one case from the JSON. Use a new output directory for another run; keep failed
and diagnostic results rather than replacing them. These nine-LiDAR closed-loop
runs are slow on the recorded laptop; wall time is substantially longer than
simulated time. Historical results require their recorded runtime versions:
running an old configuration with current code does not reproduce the old version.

## Retained development outcomes

The v2 suite completed five of six planned trials: four passes, one refusal near
the goal, and one interrupted parallel trial. It is **not a six-case successful
suite**. The report in `logs/parking_validation_holdout_v2_report` records the
missing case explicitly. The two completed noisy pedestrian trials had 381 and
329 above-ground raw LiDAR points inside the pedestrian evaluation box at the
saved audit frame; all nine device timestamps and capture poses matched.

Additional faults were identified:

- Final-pose acceptance (20 cm / 5 degrees / body inside bay) was hidden behind
  a stricter 10 cm intermediate-segment gate. The parallel vehicle stopped at
  10.9 cm / 4.39 degrees and was refused. Final-segment acceptance now uses the
  existing goal contract directly, after freshness, collision and tracking
  checks. Intermediate gear-change rules remain unchanged.
- Hybrid-search deduplication used world-aligned grid coordinates, and dominated
  queue entries were expanded. The grid now belongs to the manoeuvre frame and
  stale queue entries are skipped. Sensor hypotheses retry at most once per two
  simulated seconds, with a two-second search budget. Occupancy and command
  arbitration still run on every frame; retry suppression does not reuse a
  previous motion permission.
- CARLA wheel positions are world-space centimetres. Taking the extreme X
  coordinates made wheelbase depend on heading. Both the bridge and experiment
  runner now measure the distance between front and rear axle centres. The
  measured Audi A2 wheelbase is approximately 2.5065 m; the former calculation
  returned 2.674 m in one rotated scene. Four rotated-geometry tests cover this.

The attempted v3 run was interrupted during its first case when the wheelbase
fault was identified. Its checkpoint is retained in
`logs/parking_validation_release_v3`; it is not an acceptance result.

The v4 first regression was also interrupted during diagnosis. The search
reached nearby configurations but exhausted its budget before satisfying its
narrow grid endpoint gate. Exact endpoint refinement now starts within 0.8 m /
15 degrees, then must still converge to 1 cm / 0.5 degrees and pass the same
swept-body collision checks. This widens the numerical solver's starting region,
not the accepted endpoint error. Failed fitting falls back to expanding the
original node; it does not prune that branch. A close-goal regression test
covers this fallback. `logs/parking_validation_release_v4` is an interrupted
diagnostic, not a completed acceptance suite.

The v5 first regression exposed a separate command-interface deadlock: the
dispatcher released a safe two-point prefix, but `ParkingController` required
at least three points and held the brake. The recording is retained as an
interrupted diagnostic in `logs/parking_validation_release_v5`. The controller
now executes non-degenerate two-point prefixes in either gear and rejects a
zero-length prefix. This does not bypass the dispatcher's visibility certificate.

The v6 diagnostic exposed signed-speed feedback: CARLA reported alternating
forward/backward creep while reverse was commanded. Using speed magnitude made
rollback look like correct progress and reduced throttle. `EgoState` now has an
optional `longitudinal_velocity` (body-forward m/s). The bridge and runner supply
it; `ParkingController` uses velocity projected into the commanded gear for its
PI/acceleration feedback. Scalar `velocity` remains unchanged for existing
consumers and stopping checks. Missing signed odometry preserves the previous
fallback; non-finite supplied signed velocity holds the brake. The retained
`control_probe.json` in v6 shows the original gear, throttle and signed world
velocities. v6 was interrupted and is not an acceptance result.

## Release verification protocol

[`parking_validation_v7_release.json`](../experiments/parking_validation_v7_release.json)
contains two regression cases and three previously unexecuted geometry/noise
configurations. The new pedestrian cases use 1 m/s crossing speed and a 12-second
pause near the approach path. The regression crossing retains its original
0.6 m/s / 3-second pause. Actor staging uses the applied vehicle gear, including
observation manoeuvres. No runtime code is changed during the release suite.

To rerun the historical v7 version, restore its recorded runtime first and use
the reproduction commands with the v7 configuration and new output folders, such as `logs/parking_validation_reproduction` and
`logs/parking_validation_reproduction_report`. Do not overwrite the retained
release recordings. Conclusions must be based on the
completed report, including all failures and its `suite_complete` flag.

## Interpretation boundaries

The release cases combine geometry changes, noise and pedestrians. They test
closed-loop operation under those conditions, not a factorial estimate of each
component's effect. A stopped-speed sample alone is not evidence of yielding: the
report additionally counts stopped `PARKING_BLOCKED` frames without a motion
certificate while the pedestrian is nearby and has not finished crossing.

The projection assumes flat ground and essentially planar ego motion. No
localization-error, slope, mounting-error, adverse-weather or crowd robustness
is established here. The sensor rig remains nine devices, and residual
false-free overlap with evaluator boxes is reported rather than hidden.
Simulator success cannot establish readiness for a physical vehicle or
statistical superiority over another planner.

## Automated regression checks

The final implementation passed 392 selected tests covering the plugin,
regulatory perception, execution, plugin loading and CARLA bridge, plus all
18 BasicSim scenarios (including expected refusals for unsafe inputs). This is
the affected-component selection, not a claim that every test in AVLite ran.
Logs are `logs/parking_validation_v10_checks.txt` and
`logs/parking_basicsim_v10/results.json`.

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
MPLCONFIGDIR=/tmp/itsc-mpl python -m pytest \
  avlite-behavior-hdmap/tests \
  test/plugins/test_signal_distribution.py test/plugins/test_traffic_light_vision.py \
  test/plugins/test_sensor_regulatory.py test/plugins/test_hdmap_behavior_plugin.py \
  test/c40_execution test/c60_apps/test_c63_plugins.py \
  test/c60_apps/test_c63_plugin_import_paths.py avlite-bridge-carla/test/test_sync.py \
  test/c10_perception/test_c11_agent_ids.py \
  test/c10_perception/test_c11_prediction_model.py \
  test/c10_perception/test_c11_save_start_velocity.py \
  --rootdir=. -q -o addopts='' --import-mode=importlib --tb=short
```

## Continuing after interruption

The first v7 invocation completed the two perpendicular cases before the host
interruption. Its parallel checkpoint is not a completed trial. The remaining
three cases are rerun from their initial poses in
`logs/parking_validation_release_v7_resumed`, with unchanged runtime hashes and
scenario definitions. The two earlier completed drives are not repeated.

`tools/merge_parking_trials.py` assembles disjoint completed trials only: it
requires matching source hashes and execution options, exact case definitions,
complete coverage and no duplicate completed case/repeat. It preserves failed
results, copies the raw case artifacts and records source-file hashes in
`provenance.json`. The combined output is explicitly an assembly, not a claim
that all five drives ran without interruption.

Wall time includes CARLA physics, sensor generation, occupancy processing,
logging and actor setup; it is not an isolated planner-latency benchmark. The
certificate's speed value includes measured vehicle speed and must not be
interpreted as a tracking-error bound relative to a commanded target.
