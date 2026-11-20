# Task-aware view selection: implementation and evidence

Updated 14 November 2026. This is an experimental extension, not a validated
replacement for the historical parking policy. The historical 21-drive study
remains tied to its original runtime snapshot.

## Implementation

`VisibilityParkingPlanner.view_objective` accepts `legacy` (default), `task`, and
`task_beams`. The latter variants rank views by missing samples in the first 8 m
of a proposed swept-footprint corridor. Samples have 0.4 m spacing and exponential
progress weights with 4 m scale. No route is guessed when search has no proposal.

`task` uses incremental line of sight past known observed occluders. `task_beams`
uses calibrated vertical ground-return rings and per-device support requirements.
Set `planner.view_lidar_geometry` to a tuple of `task_visibility.GroundLidar`
objects: local x/y, height above surveyed ground, vertical angles in degrees and
sensor range. Missing calibration causes the beam variant to assign zero gain.
Level mounting, flat ground and full azimuth acquisition are assumptions; this
is not a general 3D sensor model. Neither variant may add predicted cells to
observed free space. The existing motion/stopping gate always remains in force.

For reproducible geometry-search work, set `deterministic_search = True` and
`search_expansions = 1500`. The mode disables the wall-time stopping condition
but retains node/expansion limits. It does not establish a physical execution
deadline or full CARLA determinism. Historical defaults remain unchanged.

## Evidence

- Development: `logs/task_view_carla_v1/`, six static scenes. The original stay
  score was defined as zero without measuring a fresh stationary scan.
- Post-hoc beam scoring: `logs/task_view_carla_v1_report/`, explicitly exploratory.
- Prospective follow-up: `logs/task_view_carla_v2_followup/`, 12 configurations,
  registered protocol and runtime snapshot, independent seeds/offsets relative
  to development on the same pad. Fresh stationary scan is a real control.
- Audit/report: `logs/task_view_carla_v2_report/report.json`: 142 acquisitions,
  nine devices each, 17,853,482 finite returns. Timestamp and stored-pose
  consistency and frozen-source checks pass. Raw clouds are retained.
- Two parallel configurations have no geometric proposal; they remain in results.
  View-gain means condition explicitly on the other ten configurations.
- Stationary mean gain 0.04990; bay visibility 0.13838; task line of sight 0.04990;
  task plus beam rings 0.07893. Gains are weighted observed-free area divided by
  1 + candidate length, not collision-free area, time savings or parking success.
- Beam scoring selects movement in two cases and beats bay scoring there; it does
  not outperform the baseline overall. Task line of sight selects no movement.
- Counterfactual endpoints are acquired by repositioning the static ego vehicle.
  Proposed motions pass the initial gate, but they are NOT executed closed-loop
  manoeuvres. No completion or dynamic safety claim follows from this audit.
- Search repeatability: `logs/task_view_search_repeatability.json`: three calls
  on each of six recorded inputs return identical paths or the same refusal.
- Regression selection: `logs/task_view_release_checks_v2.txt`, 426 passed.

## Reproduction

Use the `avlite-carla` environment and an exclusive CARLA 0.9.16 server. The live
runner refuses an occupied world and restores the initial map after completion.
Use a new output directory; original experiment folders must not be overwritten.
The archived source snapshot, rather than future working-tree changes, defines
this experiment's implementation.

```bash
python avlite-behavior-hdmap/tools/check_task_views_carla.py --output logs/task_views_new
python avlite-behavior-hdmap/tools/summarize_task_views.py logs/task_views_new --output logs/task_views_new_report
python avlite-behavior-hdmap/tools/check_search_repeatability.py logs/task_view_carla_v1 --output /tmp/search_repeatability.json
```

Remaining work for a stronger active-parking claim: independently designed
occlusion-rich layouts, execution of selected views under feedback, dynamic
hazards, calibrated latency and sensor/pose perturbations, external-method
baselines, and repeated completed parking trials. Do not tune against follow-up
outcomes and then relabel those same outcomes as independent validation.

## Follow-up, 16 November 2026

A finite-ray raster objective addresses the missing-measurement and azimuth
sampling mismatch. See [implementation and new prospective audit](finite-ray-view-planning.md).
The results above remain unchanged and are not merged with that follow-up.
