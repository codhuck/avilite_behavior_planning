# Review of the 18 November recording

Reviewed on 24 November 2026 against the current local implementation.
The source is `Screencast from 11-18-2026 03:22:56 PM.webm`, duration 3:52.5.
It contains video with automatic captions and **no audio stream**. The following
is a reading of those captions, with approximate timestamps, not a verbatim
transcription. It is separate from the 3 November audio and the earlier
six-minute screen recording.

| Recording | Request | Current implementation |
| --- | --- | --- |
| 00:00–00:34 | A reusable execution task can replace planners | `PlanningCoordinatorTask`, `PlannerSwitchTask` |
| 00:36–00:48 | Keep it independent of CARLA; declare requirements/capabilities | AVLite interfaces, registries and capability checks |
| 01:02–01:22 | Separate the coordinating task from planners and detectors | Configurable global, behavior, path and velocity stages; separate detector |
| 01:56–02:12 | Accept camera/LiDAR through a clean SensorFrame interface | Canonical frame contract and non-CARLA provider substitution tests |
| 02:18–02:34 | Use BasicSim to test switching quickly | Road and race coordinator demonstrations |
| 02:36–03:06 | Package the task and planners as a plugin, exposing its strategies | Root exports, strategy registration, recommended profile and installation guide |
| 03:14–03:22 | Test the components together | Focused integration tests and repeated driving/handover runs |

Fresh checks in this review: **45 tests passed**, plus **six BasicSim runs**
(three road, three race). Every run rejected a replacement request while moving,
kept the same world, and drove after a stopped replacement. Each race run used
two scripted rivals. These are repeated short fixtures, not independent traffic
scenarios. No CARLA server was started during this review.

Evidence is in the parent workspace under `logs/zoom_review_20261124/`:
`review.json` (source hash and requirement mapping), `tests.xml`,
`handovers.json` and `handovers-console.txt`.

The concrete architecture and BasicSim-testing requests in this clip are
implemented. Whole-stack replacement requires a stopped synchronous executor;
automatic transitions between road and race maps are unsupported. The common
sensor interface still requires calibrated, synchronized inputs and compatible
localization. It does not establish real-world recognition accuracy.

Automatic LiDAR parking, viewpoint-selection experiments and article validation
were later extensions, not requirements stated in this recording. They should
not be listed as unfinished items from this meeting. The previous broad status
answer conflated those scopes.

## Standalone delivery check

The follow-up found a distribution gap: coordinator acceptance tests lived only
in the parent AVLite checkout. They now ship as
`tests/test_planning_coordinator.py`, including safe handover, failed-request
rollback and substitution of two non-CARLA sensor providers. The community guide
also no longer describes the already-listed plugin as awaiting submission.

A release archive was extracted outside the development checkout and installed
against unmodified AVLite 0.6.7, commit
`1df512a3718d78a4d737e07178a9db03cf5d4f05`. This used a separate virtual
environment with existing dependencies shared from the host; it was not a fresh
dependency download. All **277 plugin tests passed**. Both road and race profiles
loaded through the AVLite factory and drove without importing CARLA. Six further
BasicSim handover runs passed, with two scripted rivals in each race run.

Local catalog metadata now uses supported planner categories and a URL without
a fragment. The catalog schema has no `TaskStrategy` category; this does not
change the task classes exported by the plugin. The upstream catalog was not
modified or published during this check.

Follow-up evidence is in `logs/zoom_completion_20261124/` in the parent workspace:
`plugin-tests.xml`, `road-factory.json`, `race-factory.json`, `handovers.json`,
installation logs and `registry-validation.txt`. Sensor-provider tests use
synthetic frames and deterministic detections; they verify the interface, not
recognition accuracy. No new live CARLA or real-camera test was performed.
