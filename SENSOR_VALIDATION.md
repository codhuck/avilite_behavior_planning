# Sensor perception validation

## Trained RGB result — 2026-11-10

The original Autoware-only backend failed below. The new `trained_rgb` backend
passed the following CARLA checks after training and fusion corrections.

| Check | Result |
| --- | --- |
| Four signals, Red / Yellow / Green / Red | 16/16 passed |
| Short 65 m signal approach with pedestrian | Completed, 73.1 s simulation |
| Full 150 m route, three pedestrians, red → green | Completed, 121.4 s |
| Repeat, opposite pedestrian direction, yellow → green | Completed, 130.65 s |
| Collision / checked rule violations in final drives | 0 / 0 |
| STOP continuous dwell | 2.70 s / 2.75 s |
| Maximum lateral error | 0.071 m / 0.071 m |
| Goal error | 0.634 m / 0.645 m |
| Actual installed executor, dynamic GT APIs blocked | Passed |
| Black camera frame | PERCEPTION_STOP |
| Automated tests | 181 passed, 4 deselected |

Final evidence: `logs/signal_training/final_validation_summary.json`,
`final_drives_complete/`, `final_lights/`, and `executor_check.json`.
Both full drives and their snapshots were copied from temporary RAM storage and
verified byte-for-byte. Directories `closed_loop`, `near_drive`, `final_drives`,
and `final_drives_v2` are developmental failures or disk-interrupted attempts,
not successful acceptance evidence.

Training: 6,081 collected CARLA crops across four signal groups plus 813 public
real-image training crops. Only 3,702 images are in training (2,889 CARLA + 813
real); 1,590 are selection/validation and 1,602 are held out. Unknown/background
crops dominate the totals; aggregate crop accuracy alone is misleading.
The 949 intersection is excluded from fitting and checkpoint selection.

Per-signal fusion across held-out views: 70/72 correct, 2 Unknown, zero false
Green. Selection/validation views: 71/72 correct, 1 Unknown, zero false Green.
This is a small controlled-scene test. The *individual-crop* classifier still has
3 Unknown→Green errors on the held-out crop set; redundant-head fusion did not
produce false Green in the tested grouped views. See the full confusion matrices
in `training_report.json`; no universal safety claim follows from these results.

Real public validation crops: 172/232 correct, 59 Unknown, one Red→Yellow error,
zero false Green. These are crops, not full-frame street detection or regional
road validation. The trained model is not qualified for arrows, single-head
junctions, night/rain, calibration errors, or real-vehicle deployment.

The fix combines a learned RGB classifier, static-map ROI proposals, close-range
and pixel-shift training augmentation, and corrected fusion of unobservable
redundant heads. Unknown is abstention only when head identity is explicitly
surveyed; conflicting visible colours remain Unknown. Green requires two
distinct heads and three fresh frames. Confidence thresholds were not lowered.

Model/profile/setup and reproduction commands: [SENSOR_PERCEPTION.md](SENSOR_PERCEPTION.md).

---

## Earlier sensor checks — 10 November 2026

Implemented dynamic observation source: `SensorRegulatoryDetector`.
Installed profile: `sensor-road` (150 m urban route).

## Completed checks

| Check | Result | Evidence |
| --- | --- | --- |
| RGB signal classification, four heads/IDs × red/yellow/green | 12/12 cases passed after frame confirmation | `logs/sensor_lights_final/result.json` and original/annotated frames |
| 150 m drive, initially red light, three crossing pedestrians | Goal reached in 117.15 simulation seconds; no checked violations or collisions | `logs/sensor_demo_final/sensor_demo_1/result.json` |
| Repeated drive, initially yellow light, pedestrians crossing the other way | Goal reached in 121.90 simulation seconds; no checked violations or collisions | `logs/sensor_demo_final/sensor_demo_2/result.json` |
| Actual AVLite executor with dynamic GT methods replaced by throwing functions | Passed; RGB/LiDAR source used | `logs/sensor_profile_check.json` |
| Black-camera injection through the installed perception/planning pipeline | `PERCEPTION_STOP` | `logs/sensor_profile_check.json` |
| Execution and plugin regression suite | 154 passed, 4 deselected | Command below |

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
/path/to/user/miniconda3/envs/avlite-carla/bin/python -m pytest \
  test/c40_execution test/plugins/test_hdmap_behavior_plugin.py \
  test/plugins/test_sensor_regulatory.py -q --disable-warnings
```

Both drives used RGB/LiDAR observations for planning. Actor state was used only
by the test fixtures and independent violation checks. Each drive spawned three
physical pedestrians, waited for them, and verified that all three cleared.
The STOP audit measured continuous stopped intervals of 2.75 and 2.70 seconds.
Maximum lateral errors were 0.074 and 0.077 m; final position errors were about
0.68 m. These are vehicle tracking metrics, not perception accuracy metrics.

Median measured perception processing time was approximately 83–84 ms on CPU
with OpenCV limited to two threads. The configured perception period is 0.1 s.
Simulator rendering and the rest of the stack add overhead; this does not imply
real-time execution of the whole application.

Aggregate evidence: `logs/sensor_validation_summary.json`.
Each drive folder contains `perception.jsonl`, `telemetry.csv`, `events.json`,
`pedestrian_motion.json`, and annotated front-camera `sensor_*.jpg` snapshots.
Detection counts are predictions, not ground-truth object counts; no precision/
recall benchmark is claimed.

## Defects found and addressed

- Yellow signal housings confused a broad colour mask: classify the illuminated
  aperture inside a surveyed vertical three-lamp head instead.
- Missing sparse LiDAR returns on visible pedestrians blocked release: use visible
  feet and a LiDAR-estimated ground plane when possible, otherwise remain Unknown.
- Requiring visibility of the entire width of a multi-lane crossing caused false
  waits: inspect the route-conflict area with a six-metre lateral approach margin.
- Generic large following buffers caused an unnecessarily early obstacle stop
  near a STOP target: the low-speed city profile now uses 2 m / 2 m buffers plus
  body extents, with independent crossing/collision checks retained.
- Reused frames, sensor timestamp mismatch, future/stale data and camera failure
  must not be interpreted as Green or an empty crossing. Regression tests cover
  these cases and float-roundoff between CARLA and AVLite clocks.

Earlier directories (`sensor_city_probe`, `sensor_city_v2`, `sensor_city_final`,
and the partial `sensor_demo_validation`) are development diagnostics, not the
final acceptance results. The final two-drive acceptance run is `sensor_demo_final`.

## Scope and remaining limitations

The HD map, static surveyed head geometry and simulator ego localization remain.
Mapped STOP rules remain conservative even if the sign detector misses a sign.
This is not mapless driving or a replacement localization system.

The signal classifier is limited to vertical three-aspect heads in the tested
CARLA scene. Night/rain/glare, turn arrows, occlusion robustness and sensor pose
error require additional evaluation. The generic COCO detector is not fine-tuned
for CARLA. LiDAR free-space checks are sampled; ground-plane projection is an
approximation. These tests do not establish safety for arbitrary traffic.
Moving-vehicle traffic was not included in the two acceptance drives.
## Autoware backend: live CARLA check, 2026-11-10

**Functional validation FAILED.** These results apply to the experimental
`autoware` backend, separately from the earlier CARLA aperture classifier below.

- Live Town10HD_Opt, RGB/LiDAR: four route lights x Red/Yellow/Green = 12 cases.
  All final predictions were Unknown; four cases had no detected traffic light
  anywhere in the image. Remaining detections were classified as unknown.
  The static survey matched the loaded HD map.
- A 60-second sensor_demo run completed STOP 947 (2.75 s audited dwell) and
  allowed three walking pedestrians to clear. No recorded collision or checked
  violation; the route was not completed within this time limit.
- A focused 65 m signal approach stopped in WAIT_LIGHT / light_unknown.
  The independent oracle in the repeat recorded Green from about simulation
  second 16 while camera perception remained Unknown through second 24.
  The vehicle did not resume; this is a liveness failure, not a successful run.
- Actual AVLite executor: 30 steps with dynamic-GT perception APIs blocked
  passed. Black-camera injection caused PERCEPTION_STOP. This checks the
  integration and failure response, not traffic-light recognition accuracy.

Evidence: `logs/portable_signals/carla_validation_summary.json`, with individual
case images/results in `carla_live_lights/`, trajectories in `carla_live_drive/`,
and synchronized perception vs oracle data in `carla_signal_confirm/`.
`validation_only_light_states` is written to audit logs only, never supplied to
the perception or planning model.

Reproduce (run one client at a time on an empty Town10HD_Opt scene):

```bash
python tools/check_sensor_perception.py --signal-backend autoware --no-raw --output logs/portable_signals/carla_live_lights
python tools/run_hdmap_scenarios.py --routes sensor_signal_probe --repeats 1 --max-sim-time 25 --walking-pedestrian --sensor-perception --signal-backend autoware --output logs/portable_signals/carla_signal_confirm
python tools/check_sensor_profile.py --signal-backend autoware --output logs/portable_signals/carla_executor_check.json
```

Do not enable this backend as the normal driving profile yet. Detection and
state classification need further work before repeating closed-loop acceptance.

