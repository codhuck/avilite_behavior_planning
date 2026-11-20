# Requirements and evidence

The newer 18 November meeting is covered separately by
[task coordination](task-coordination.md) and its
[validation record](coordination-validation.md): registered coordinator task,
independent strategy stages, BasicSim checks and a canonical sensor-frame
contract. Nothing from that follow-up was published or sent.

This audit separates the recorded teaching discussion from later feature
requests. The source is the saved machine transcript at
`logs/dispatcher/audio_requirements_transcript.json` in the parent workspace;
it is not a manually verified quotation. The original audio file was not found
at its former Downloads path during this audit.

| Request | Implementation / evidence | Remaining boundary |
| --- | --- | --- |
| HD-map STOP, crossings, lights (audio 0:27–1:16, 8:51–9:49) | HDMapBehaviorPlanner, regulatory parser, local pipeline; city validation in VALIDATION.md | Static geometry still uses the map |
| Behavior tree (0:27–1:16) | Optional `HDMapBehaviorTreePlanner`, reactive Selector/Sequence runtime, rule traces and portable profile; [implementation](behavior-tree.md) | Reuses existing rule actions; not a new learned policy |
| Swap planning algorithms via execution task (0:15–0:27, 1:16–1:32) | PlannerSwitchTask, road/race dispatch; logs/dispatcher/execution_switch.json | Stopped SyncExecuter only; no mixed road/track topology transitions |
| Change perception algorithms (1:21–1:32) | PerceptionSwitchTask and SelectPerceptionStack; road/race executor regression tests | Requires paused SyncExecuter, a freshly constructed compatible strategy and updated core |
| Share plugin and reusable profiles (9:23–9:49) | Matching source bundle, SHA-256 manifest, portable profiles and installer; [instructions](source-bundle.md) | Bundle includes the modified core/bridge; publication of these new changes is separate |
| Validate using CARLA (9:49 onward) | Recorded city, signal and parking physics validations | Separate ground-truth and multi-LiDAR parking experiments; see sensor parking validation |
| Race track and rivals (later user request) | Race planner stages and scripted rivals; logs/race_full_traffic | Low-speed scripted traffic, not arbitrary racing |
| Sensor light colour and crossing occupancy (later user request) | RGB classifier and LiDAR/RGB adapter; SENSOR_VALIDATION.md | Real street recordings, calibration and deployment validation missing |
| Select parking type (later user request) | Three modes, Tk selector, 18 BasicSim scenarios and 6 CARLA physics drives | Known surveyed bays |
| Sensor-only parking (remaining integration) | Calibrated multi-LiDAR fusion, freshness checks and completed CARLA drives for all three types; [results](sensor-parking-validation.md) | Flat surveyed pad, simulator localization, residual false-free overlap; no automatic marking recognition |
| Visibility-aware parking research | Optional VisibilityParkingPlanner, independent dispatcher, five-policy decision benchmark and ideal planar BasicSim experiment; [study protocol](itsc-study.md) | Sensor-supported prefix/speed arbitration added and tested in CARLA; [geometry, walker and noise validation](parking-robustness-validation.md): 5/5 final regressions and 3/3 previously unused cases passed on frozen code; Later finite-ray study: 12/12 proposed endpoints improve the acquisition metric over bay scoring, but this is not parking success. A previous forward drive had contact; newer reruns must be reported separately. No executed observation-move benefit established; [current limits](finite-ray-view-planning.md) |
| Videos and presentation (earlier user request) | Existing artifacts documented in VALIDATION.md and parent presentations/ | They precede sensor-parking changes and do not demonstrate that feature |

Do not combine results from different dates into a claim that all requirements
have passed end-to-end. Passing unit tests or refusing unsafe motion is not a
successful autonomous parking manoeuvre. Real-vehicle testing also needs actual
sensor recordings/calibration and a vehicle test environment; it cannot be
established by CARLA results alone.

The [2026-11-17 follow-up](meeting-followup.md) records the behavior-tree, source-bundle and plugin-path corrections, including failed and successful live checks.
