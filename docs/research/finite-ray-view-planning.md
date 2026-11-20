# Finite-ray view planning

Updated 16 November 2026. `task_scan` is an opt-in parking observation objective.
The historical default remains `legacy`. This changes view ranking, not the
measured-space motion authorization gate or the requested parking type.

## Why the previous score failed

Subtracting predicted visibility at the current pose discarded cells that the
model expected to see but the actual sensor had not observed. Continuous beam
rings also counted cells between discrete azimuth samples. The earlier audit
retains those negative results; its measurements informed this development and
are not reused as independent validation.

## Implementation

`scan_visibility.ScanVisibility` projects discrete calibrated ground rays into
the same 0.25 m world raster used by parking perception. It uses slant range,
counts each device once per cell, masks known occupied geometry, and excludes
rays intercepted by a conservative ego-body box. Four deterministic azimuth
phases expose sensitivity to sampling phase. Their mean is a heuristic expected
area, **not** a calibrated probability or safety confidence.

Only samples missing from the currently measured task corridor contribute.
The score is predicted acquired task area divided by `1 + path_length`, minus
predicted area from a fresh stationary scan. Motion must beat that stationary
alternative by `view_min_advantage` (default 0.01). The dispatcher uses the same
cost-normalized threshold and ranking. Its legacy raw-gain threshold is unchanged
for other objectives. Every selected motion must still pass fresh occupancy,
age, stopping, curvature, allowed-type, and observation-budget checks.

The calculation assumes level full-azimuth LiDARs, surveyed flat ground,
calibrated mounts and body dimensions. Unknown occluders, material effects,
dropout, pose error and uneven ground remain outside its prediction model.
Occupied geometry is treated as a conservative 2D ray blocker. The ego box is an
approximation, not a complete mesh. Predicted returns never enter observed free
space, and predicted visibility never constitutes permission to move.

## Configuration

Configure an instantiated `VisibilityParkingPlanner` after setting up perception:

```python
planner.view_objective = "task_scan"
planner.view_lidar_geometry = calibrated_ground_lidars
planner.view_scan_parameters = {
    "resolution": 0.25,
    "azimuth_samples": 250,
    "phases": 4,
    "body_bottom": 0.15,
    "body_top": 1.6,
}
planner.view_min_advantage = 0.01
```

`calibrated_ground_lidars` is a tuple of `task_visibility.GroundLidar` objects.
Each contains local x/y, height **above ground**, vertical angles in degrees,
and maximum slant range. Heights must include the ego reference's ground offset.
Use actual calibration and scan sampling; the numbers above describe the tested
CARLA rig, not arbitrary hardware. Body length/width and support requirements
are supplied by the planner from the ego model and occupancy snapshot. Missing
calibration or a missing geometric path gives no speculative view gain.

## Development replay

`logs/scan_views_development_vectorized.json` replays the retained v2 recordings.
On the ten cases with a proposal, acquired weighted area/cost was 0.16692 versus
0.13838 for bay-area ranking. This is a development result on familiar inputs.
Vectorization preserved all selections and gains, while mean scoring time for
12 candidates fell from 2.660 s to 0.168 s in these local runs. This is not an
end-to-end timing guarantee and not a controlled hardware benchmark.

## Prospective acquisition audit

`logs/scan_view_carla_v3/` contains a frozen protocol, runtime sources, raw clouds
and results for 18 configurations: three parking types, three neighbour layouts
and two new noise seeds. Initial poses differ from development; two layouts add
an occluding parked vehicle. The ground pad remains the same. Settings were
frozen before acquisition, with no parameter changes during the series.

Twelve configurations have geometric proposals. The other six remain recorded
as no-proposal outcomes, not successful or zero-gain manoeuvres. Conditional on
the twelve proposals:

| Ranking | Mean acquired weighted area / cost | Selected movements |
| --- | ---: | ---: |
| Fresh stationary scan | 0.12681 | 0/12 |
| Bay visibility | 0.14631 | 8/12 |
| Task line of sight | 0.13892 | 2/12 |
| Task plus beam rings | 0.15386 | 4/12 |
| Finite-ray raster | 0.19716 | 12/12 |

Finite-ray ranking beats bay ranking in all twelve scored configurations, with
34.8% larger mean acquired utility. This is **not** a parking completion or
collision-rate improvement. Closely related noise-seed pairs often yield the
same rasterized outcome, so twelve cases must not be treated as twelve
independent geometries for statistical claims.

The auditor checks 174 acquisitions, 1,566 per-device stored actual/requested
pose comparisons, source snapshots, synchronized timestamps, and selections.
The 21,918,371 finite returns and raw results are retained. Endpoint measurements
are acquired by repositioning a static ego car, not by executing the proposed
motion. Other actors are static in this audit.

## Reproduction

Run from the AVLite workspace using the `avlite-carla` Python environment. The
live runner requires an exclusive CARLA server and restores its previous map.
Always use a new output path.

```bash
python avlite-behavior-hdmap/tools/replay_scan_views.py logs/task_view_carla_v2_followup --output /tmp/scan_replay.json
python avlite-behavior-hdmap/tools/check_scan_views_carla.py --output logs/scan_views_new
python avlite-behavior-hdmap/tools/summarize_scan_views.py logs/scan_views_new --output logs/scan_views_new_report
```

The frozen source snapshot defines the acquired experiment; later working-tree
changes do not retroactively change its method. See the dynamic integration
report for executed manoeuvres and their separate failure conditions.

## Bounded recovery search

The visibility adapter now has `recovery_search_expansions = 6000`. With
`deterministic_search = True`, two unsuccessful searches permit the next stopped
attempt to use that bounded expansion limit. The original limit is restored
when the call returns; successful search clears the failure count. Moving ego
states never trigger the increase. The prior implementation increased only the
wall-time timeout, which deterministic search explicitly disables.

`logs/scan_view_search_diagnosis.json` confirms that all six no-proposal v3
inputs yield a geometric path with a 6000-expansion base search. This is a
post-hoc diagnosis, not a revision of the original 18-case outcomes.
`logs/scan_view_adapter_recovery.json` exercises the actual visibility adapter:
all six obtain proposals; four already succeed on its first attempt, while two
parallel cases require the third, expanded attempt. The base planner and the
visibility adapter have different clearance settings and are not interchangeable
search baselines. A recovered proposal still needs current observation checks
before any part may be executed.

The separately frozen dynamic v1 trials started before this recovery correction;
they validate the scan objective's integration under the earlier fixed budget,
not the new recovery behaviour. Recovery is covered by unit tests, replay of recorded initial clouds, and the
subsequent single parallel live rerun described below. Broader repeated live
regression remains necessary.

## Occupancy runtime optimization

Dense ground-return cell deduplication and per-point support membership now use
NumPy rather than repeated Python tuple construction. On all 18 retained v3
initial observations, free, occupied, blocked and traversable geometries are
topologically equal to the frozen original; timestamps, accepted sources and
support histograms also match. Three interleaved repetitions per implementation
and scene gave mean per-scene median times of 239 ms before and 159 ms after.
These are local measurements with concurrent CARLA activity, not a real-time
execution bound. Evidence: `logs/scan_occupancy_equivalence.json`.

The optimization preserves measurement semantics; it does not cache old free
space or relax sensor support. Dynamic v1 uses the original frozen occupancy
implementation. The updated live runner additionally records collision frame,
timestamp, other actor ID/type and impulse for future diagnostic runs; those
fields cannot be reconstructed retrospectively for v1 contacts.

## Executed dynamic integration, frozen v1

`logs/scan_dynamic_carla_v1_report/report.json` retains all three executed trials
with moving pedestrians, sensor noise and new seeds on known layouts:

| Parking type | Outcome | Simulation duration | Contacts | Final position error |
| --- | --- | ---: | ---: | ---: |
| Perpendicular forward | Blocked; contact registered | 14.2 s | 1 | 13.86 m |
| Perpendicular reverse | Parked; pedestrian crossed | 114.9 s | 0 | 0.097 m |
| Parallel | Blocked; pedestrian crossed | 32.1 s | 0 | 8.96 m |

Forward speed was below 0.01 m/s continuously from 12.8 s to termination. The
contact was registered despite positive evaluator-box separation (minimum
0.056 m); the runner did not retain the collision partner/event timestamp.
Therefore the contact cannot be dismissed on the basis of evaluator boxes or
attributed conclusively to an actor from proximity alone. The trial remains a
failure. Stopping does not guarantee that another actor will avoid a stationary
vehicle, and box clearance is not equivalent to CARLA collision geometry.

The reverse trial had minimum pedestrian box clearance 1.288 m, zero recorded
contacts and zero body/boundary violations. All three trials have zero recorded
body/boundary violations, but this does not override the forward contact.

No `PARKING_OBSERVE` state was executed in these three runs. They exercise the
new objective's integration and the parking gate, **not** successful execution
of its selected observation moves. Static acquired-utility improvements must
not be presented as improved closed-loop parking success. A separate parallel
rerun tests the subsequent recovery correction; it is not pooled with v1.


## Parallel recovery rerun, frozen v2

With the bounded expansion correction and equivalent optimized occupancy,
`logs/scan_dynamic_recovery_v2_report/report.json` records a successful repeat of
`scan_dynamic_2` with the same scenario settings and seed. The vehicle reaches
`PARKED` at 217.5 simulated seconds: position error 0.0961 m, heading error 1.85°,
zero recorded collision events, zero body/boundary violations, and a completed
pedestrian crossing. Minimum pedestrian box clearance is 1.461 m. The failed
v1 parallel trial remains in its original report; this rerun does not replace it
or establish a statistical success-rate improvement.

This run also executes no observation manoeuvres. It validates recovery and
ordinary parking integration with the objective enabled; closed-loop benefit
from actually executing finite-ray-selected views is still unproven. The
forward contact remains unresolved and is a priority for further dynamic-hazard
work. More geometry/seed repetitions, executed viewing actions, and a dynamic
hazard model are needed before stronger autonomy claims.

The final affected-component regression selection passes **441 tests** in
`logs/scan_view_release_checks_v3.txt`. The final map was restored to
`Town10HD_Opt`; no test vehicles, pedestrians or sensors remained. Quantitative
figures and raw provenance are stored alongside the separate static and dynamic
reports. The article's prior results were not overwritten.

A stricter replay also confirms byte-identical WKB geometry serialization on all
18 initial scans (`logs/scan_occupancy_equivalence_strict.json`), preserving the
geometry fingerprints used for replanning. Seven retained dynamic event frames
with pedestrians also pass geometry/source/support equality checks in
`logs/scan_occupancy_dynamic_equivalence.json`.

## Forward rerun, 2026-11-17

The unchanged `scan_dynamic_0` scenario (seed 7601) was rerun with the current
search-recovery implementation and optimized occupancy, after the earlier v1
contact. Evidence is in `logs/scan_forward_diagnosis_v3` and its `_report`
directory. Every frozen source was matched to the protocol SHA-256; the runner
now automatically snapshots source bytes before subsequent runs.

This rerun reached PARKED in 148.5 simulation seconds (485.1 wall seconds), with
position error 0.0945 m and heading error 1.509 degrees. It registered zero
contacts and zero body violations. The pedestrian crossed and the vehicle
resumed after waiting. There were no PARKING_OBSERVE frames.

The minimum pedestrian-box gap was only **0.0748 m**. Therefore this is a
successful rerun under the recorded completion/contact criteria, not evidence
that the previous close-encounter/contact issue is resolved robustly. The old
failed recording is retained. There is still no controlled demonstration of an
executed finite-ray observation manoeuvre improving parking completion.
