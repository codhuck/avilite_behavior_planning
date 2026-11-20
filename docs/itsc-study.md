# Visibility-aware parking arbitration: ITSC study

## Research question

Does choosing among user-authorized parking manoeuvres and bounded observation
manoeuvres reduce deadlock without increasing collisions, compared with a fixed
planner using the same sensor observations and stopping constraints?

The November 13 follow-up separates two measurable questions: how adaptive
distinct-device support changes free-space estimates on identical acquisitions,
and how individual arbitration components affect completion, clearance and
duration in paired closed-loop scenarios. Identical exogenous scenes do not
mean identical observations after policies take different trajectories.

This is a candidate contribution, not an established novelty claim. Active
perception and visibility-aware parking already have substantial prior work:

- Limited Visibility Aware Motion Planning for Autonomous Valet Parking Using
  Reachable Set Estimation (2021): https://www.mdpi.com/1424-8220/21/4/1520
- Safe Occlusion-aware Autonomous Driving via Game-Theoretic Active Perception
  (RSS 2021): https://roboticsproceedings.org/rss17/p066.pdf

The narrower proposed contribution is the joint arbitration contract: sensor
freshness, conditional stopping clearance, user-authorized manoeuvre types,
information-seeking candidates and bounded retries. A systematic literature
comparison is still needed before claiming this combination is new.

## Implemented method

`visibility_dispatch.py` is independent of CARLA and AVLite. Inputs are a
free-space estimate, observed occupied geometry, acquisition time, vehicle pose,
geometry/speed and kinematically valid candidate trajectories. A candidate is
rejected for incompatible type, nonfinite data, wrong starting pose, excessive
sample spacing/curvature or lateral sliding. Rear-axle offsets are supported.

Let observation age be `a`, speed bound `v`, command latency `tau`, minimum
assumed deceleration `b`, hidden-object motion bound `u` and base clearance `m`.
The conditional geometric check uses:

```
stop_time = tau + v / b
stop_distance = v * tau + v**2 / (2 * b)
inflation = m + u * (a + stop_time)
```

The full swept vehicle footprint, inflated by this amount, must lie inside
observed free space and avoid observed obstacles. Candidate path length must
also accommodate stopping distance. Defaults are `b=1 m/s²`, `tau=0.1 s`,
`u=1.5 m/s`, `m=0.15 m`; these are declared assumptions, not measured braking or
pedestrian guarantees. Sensor/localization error can violate the input contract.
The algorithm does not claim formal real-world safety.

The dispatcher first selects an admissible parking trajectory. Otherwise it
ranks admissible observation manoeuvres by predicted unknown target area visible
from their endpoints, normalized by distance. Known obstacle intersections mask
visibility rays. This information-gain estimate is a heuristic: it never turns
unknown area into free space. Repeated endpoints are suppressed and observation
attempts are capped. Failure to find a candidate commands STOP.

`VisibilityParkingPlanner` integrates this with AVLite. It offers straight and
constant-curvature observation primitives in both directions, rechecks the
remaining committed manoeuvre each cycle, stops before changing manoeuvres and
then replans parking from the new pose. Curvature-bounded hybrid search can
supply a multi-segment manoeuvre when a single approach curve fails. Its final
entry direction respects the selected parking type. Future segments may remain
unobserved, but only a currently observed, stopping-admissible prefix of the
current constant-direction segment is released. The controller tapers its speed
at that prefix endpoint, and continuation requires a fresh check. The next gear
segment is checked after a full stop. Cancellation is latched. The adapter
retains the user's selected parking type; the generic dispatcher can compare
multiple types only if the caller explicitly authorizes them.

Hybrid-search endpoints can be refined with bounded least-squares optimization
of the last primitives. Refined paths must pass the same collision callback,
reach within 1 cm and 0.5 degrees, and preserve gear choices. Failure to refine
continues the search; it does not attach an unchecked straight connector.

Enable via `ParkingTask` with:

```yaml
parking_active_observation: true
parking_lidar_occupancy: true
parking_simulator_obstacles: false
parking_ground_z: 0.0  # replace with surveyed elevation
```

These are plugin setting keys within the plugin's normal profile section. Use
the updated workspace and Parking dialog described in [parking.md](parking.md).
This option does not fix the unresolved sparse 3D LiDAR coverage problem: the
system may correctly refuse all manoeuvres with that sensor input.

## Reproducible experiments

From the parent AVLite workspace in its Python environment:

```bash
python avlite-behavior-hdmap/tools/benchmark_visibility_dispatch.py \
  --cases 100 --seed 2027 --output logs/visibility_dispatch_benchmark
python avlite-behavior-hdmap/tools/check_visibility_closed_loop.py
```

The first tool compares five policies on exactly the same observations and
candidate set: optimistic treatment of unknown space, fixed conservative
parking, conservative selection among all authorized parking candidates, fixed
parking plus active observation, and the complete dispatcher.
Unseen obstacles are accessible only to the evaluator. Raw decisions, ages,
unknown-area intersections and per-decision timing are written to CSV.

This is a **one-step selection experiment**, with straight candidate paths and
axis-aligned random coverage bounds. A PARK decision is not a completed parking
manoeuvre. The optimistic baseline deliberately ignores both age and unknown
space, so its failures cannot isolate these two effects. Its near-zero selection
time is not comparable to a safety-checking planner's time.

The second tool exercises the actual ParkingController and BasicSim. Its ideal
planar range scanner ray-casts environment geometry and returns a visibility
polygon and hit surfaces. It is not a 3D LiDAR model or a CARLA test. Cached
geometry in a static synthetic scene represents new ideal acquisitions, not
permission to refresh old real sensor data. Ground-truth polygons are retained
only in the scanner/evaluator; the planner receives sensor observations.

All failures must remain in reports. Do not merge earlier simulator-occupancy
CARLA parking successes into sensor-only or active-observation results.

## Preliminary evidence (12 November 2026)

The held-out decision seed 2028 contains 100 synthetic cases. Counts below are
selected actions, **not parking successes**. These layouts are simple random
rectangular coverage bounds, not a representative urban benchmark.

| Policy | Park | Observe | Stop | Evaluator path collisions |
| --- | ---: | ---: | ---: | ---: |
| Optimistic | 100 | 0 | 0 | 31 |
| Conservative, authorized alternatives | 27 | 0 | 73 | 0 |
| Fixed conservative | 11 | 0 | 89 | 0 |
| Fixed with observation | 11 | 23 | 66 | 0 |
| Dispatcher | 27 | 19 | 54 | 0 |

Raw evidence: parent workspace `logs/visibility_dispatch_holdout/`.
One complete perpendicular-forward BasicSim run with ideal planar observations
parked after 111.75 simulated seconds, with 9.03 cm position error and zero
collision samples (`logs/visibility_closed_loop_final/active.json`). The fixed
conservative policy stopped from the same initial pose. This is a single
development example, not statistical validation. A rerun after endpoint refinement also parked with 8.88 cm error and zero
collision samples (`logs/visibility_closed_loop_verified/active.json`).
Preserve both development runs separately; they are not independent held-out trials.

Reverse-entry trials did not complete: the original search produced a poorly
aligned initial segment, and a refined local retry exhausted its path budget.
An early parallel trial was interrupted and cannot be counted as completed.
The subsequent bounded parallel run stopped without parking or collisions: its
remaining parking segment intersected the occupied stopping envelope, and the
observation candidates lacked confirmed free space
(`logs/visibility_parallel_bounded/active.json`).
The runner now supports `--wall-budget` and records termination reasons so that
resource-limited trials remain visible. The limit is checked between planner
calls, so one search can extend it. No new CARLA sensor-only success is claimed.

The regression suite passed 314 tests. Unit/regression results establish tested
contracts, not field safety or submission readiness.

## Submission acceptance criteria (not yet achieved)

### Actual CARLA LiDAR follow-up

Two perpendicular-forward trials on the generated CARLA 0.9.16 pad used actual
ray-cast LiDAR: 64 channels/320,000 points per second and 128/640,000, both at
20 Hz, range 30 m, mount height 2.3 m, vertical field -45 to +10 degrees.
Noise and general dropoff were disabled. Both stopped without parking, collisions
or body-boundary violations. This is a refusal result, not a successful drive.

With 128 channels the first cloud contained 23,542 points; only 13.4% of the
surveyed bay was confirmed free. The unobserved ego footprint covered 6.63 m².
Free estimates overlapped evaluator vehicle bounding boxes by 0.11 m²; boxes
are conservative evaluation labels, but this positive overlap precludes claiming
zero false-free error. All observation candidates failed the observed stopping
envelope check. Increasing density alone did not resolve the blind region.

Reproduce with an idle, exclusive CARLA server:

```bash
python avlite-behavior-hdmap/tools/check_parking_carla.py \
  --sensor --active --mode perpendicular_forward --channels 128 \
  --output logs/visibility_carla_128
```

The runner refuses a server containing vehicles, temporarily replaces its map,
and restores the previous map after cleanup. Actor boxes are used only by the
evaluator in sensor mode. Localization, surveyed bay and flat ground elevation
remain privileged inputs. RGB is not used in this parking experiment. Initial
cloud, transforms, coverage geometry, rejection reasons and telemetry are saved;
the consolidated report is `logs/visibility_carla_study/report.json`.

**Historical conclusion for the roof-only experiments above:** next work must address near-field coverage and occupancy uncertainty, including
explicit treatment of the already occupied ego footprint, without declaring
adjacent unseen cells free. Additional calibrated low-mounted sensors and
occlusion-aware fusion need testing before releasing motion. At that stage sensor-only CARLA
parking and real-world transfer were unvalidated. Subsequent implementation and
actual multi-LiDAR drives are documented in
[sensor-supported parking](sensor-supported-parking.md) and the
[validation record](sensor-parking-validation.md); real-world transfer remains
unvalidated.

### Additional robustness study

The [robustness protocol and retained outcomes](parking-robustness-validation.md)
add real CARLA walkers, range noise, random ray loss and varied start/scene
transforms. Source hashes, failures and interrupted trials are retained. The
initial v7 series had three passes and two failures across five trials; one
failed trial also had invalid pedestrian staging. Following corrections, all
five final v10 regression cases parked with 0.094–0.098 m goal error, no collision
contacts and no body/boundary violations. Four included continuous walking
actors and noisy LiDAR. Parking took 119.9–462.4 simulated seconds; completion
alone is not evidence of fast or smooth control. These development reruns are
not additional held-out successes.

The separately registered v11 protocol uses three previously unexecuted
parameter combinations on the frozen runtime, one for each parking type. All
three completed with 0.096–0.099 m / 0.721–1.241° goal error, continuous pedestrian
crossings, and zero collision contacts or body/boundary violations. Raw-cloud
support and matching source hashes were verified. These are three single runs,
not a statistically powered comparison. This remains the same flat pad, vehicle and surveyed bay model, not a held-out
street map. See the complete robustness report for its results. This extends
scenario coverage but does not replace repeated cross-policy trials or
confidence intervals. Cite the exact version and include failed and invalid
encounters.

### Required before submission

1. Completed closed-loop parking across all three requested types, with fixed
   initial poses shared across policies and no pose teleport during a run.
2. Separate tuning and held-out layouts/seeds; repeated trials with intervals
   for success, collision, deadlock and time, not just a favourable example.
3. Occluding vehicles, pedestrian intrusion, occupied bays, delayed/missing
   frames, calibration perturbations and insufficient-observability refusals.
4. Ablations separating view selection, planner/type selection, freshness and
   stopping envelope; report compute cost and unsuccessful searches.
5. End-to-end CARLA sensor evaluation. Ideal planar observations establish only
   policy/controller behavior, not perception quality.
6. Real sensor replay/calibration if making transfer claims. Without it the
   paper must explicitly restrict conclusions to simulation.

The paper should report effect sizes and failure cases. No claim of statistical
significance, universal deadlock elimination, real-world readiness or guaranteed
acceptance at ITSC follows from the present implementation.

## Paired ablation follow-up (13 November 2026)

The v12 comparison is registered in
`logs/parking_ablation_v12_registration.json` before execution. It contains seven
policies on three matched geometries (21 drives): full, roof-only,
uniform single-source support, uniform two-source support, no adaptive speed
search, no observation manoeuvres, and complete-segment release. All retain the
same collision/visibility checks and controller except the explicitly disabled
component. `no_speed_search` disables the discrete lower-speed search; endpoint
speed tapering remains, so it is not a constant-speed vehicle baseline.

Unlike earlier actor staging, the walker origin and direction are fixed in world
coordinates from a reference recording, with the same spawn time, speed and
pause rule for every policy in each geometry. Scene transforms, initial pose,
neighbours, sensor seed and time limit also match. Run order is shuffled with a
fixed seed before execution. The geometries are known development layouts with
new noise seeds, not new street maps. There is one run per pair member; no
population-level significance is inferred from the three geometry pairs.

A parking-completion endpoint does not require stopping near a pedestrian whose
path never conflicts with the ego car. It does require the accepted final pose,
no contacts/body violations, and a valid completed walker encounter. The old
strict `passed` field, which also requires stopped samples near the walker,
remains in raw results. Both definitions are retained explicitly. Refusals and
invalid scenarios are not dropped, and refusal duration is never interpreted as
fast parking. Matched duration differences are reported only if both variants
complete; every success/failure pair is still reported.

The runtime is frozen throughout the series. Full results must be read from the
completed `logs/parking_ablation_v12_report/report.json`; a running checkpoint is
not a result. All cases use a 600 s maximum, early termination on a collision or
body violation, and the existing stopped/blocked rule after the walker crosses.
Wall time is descriptive only: the host also performs analysis and is not an
isolated computation benchmark.

### Matched observation analysis

`tools/compare_parking_evidence.py` evaluates the same saved observations for all
support rules: three v11 scenes at steps 0 and 200 (six matching acquisitions).
Only those steps have both saved clouds and matching evaluator geometry. It
never combines initial clouds with later event clouds. The evaluator reconstructs
the transformed manoeuvring region from the scenario and checks acquisition
alignment. Occupancy labels are evaluator-only and never passed to fusion.

The recorded six-frame comparison (`logs/parking_matched_evidence_v12/report.json`)
found the support-set nesting `F_two ⊆ F_adaptive ⊆ F_one` in every frame. Mean
free areas were 148.800, 296.045 and 304.830 m² respectively. Mean false-free box
overlaps were 0.01894, 0.01894 and 0.25345 m². These are descriptive means over
six selected frames, not independent-trial estimates or proof of perfect
perception. All three multi-device policies admitted the same 18 of 24 straight
probe/speed combinations. Roof-only admitted none. Thus the free-area tradeoff
is measurable here, but these probes alone show no action-selection advantage
for adaptive support over uniform two-source support. Closed-loop outcomes must
be assessed separately.

The nesting follows from support thresholds on fixed ground/hazard estimates,
not from a new probabilistic confidence model. Likewise, lowering requested
speed weakens stopping distance and inflation requirements for a fixed snapshot
and path, while measured speed remains a lower bound. These conditional
properties clarify the implementation; neither is a real-world safety theorem
or a claim of novel reachability theory.

### Interrupted-series continuation

The first v12 invocation retained five completed cases. Its next case was
interrupted and is not counted as complete. The remaining 16 cases resume in
`logs/parking_ablation_v12_resumed/` from their original initial poses, with the
same 26 registered source hashes and scenario order. The original checkpoint
is retained. Assembly must reject duplicate completed cases and preserve failed
outcomes. The series is not an uninterrupted invocation.

The restarted no-speed-search case diverged from its interrupted prefix by up
to 1.042 m over 138 common frames despite matching source hashes and seeds
(`logs/parking_ablation_v12_resumption_consistency.json`). The cause was not
isolated. The seed does not establish bitwise closed-loop reproducibility, and
one run per policy cannot disentangle policy effects from run variability.
The restarted trial stopped on evaluator pedestrian-box overlap while ego was
stationary, without a collision-sensor event. It remains a failed trial, and
should not be described as a measured moving collision or discarded as a pass.

Roof-only preserves the full policy's two-source requirement near hazards.
One device cannot satisfy that requirement. Its refusal therefore measures
dependence of this configured stack on multiple sensors, not impossibility of
parking with an independently designed single-LiDAR method.

An exploratory matched-frame control removes the two-source requirement from
roof-only sensing (`logs/parking_matched_evidence_v13_roof_control/`). Mean free
area rises to 232.576 m² and evaluator overlap to 0.09046 m², but admissions
remain 0/24. Thus relaxing this threshold alone does not release the probes in
these acquisitions. The original four-policy v12 report is preserved. This
fifth offline policy was added during analysis, not preregistered as a drive.


### Completed paired series (13 November 2026)

All 21 registered trials are assembled in `logs/parking_ablation_v12_complete/`.
The final registration audit passed for all 26 recorded runtime source files.
Reports retain failures and the original interrupted prefix is not a trial.

| Policy | Parked | Contact/body-violation trials | Median successful simulated time |
| --- | --- | --- | --- |
| Complete | 3/3 | 0/0 | 168.2 s |
| Roof only | 0/3 | 0/0 | — |
| Uniform one source | 3/3 | 0/0 | 168.9 s |
| Uniform two sources | 3/3 | 0/0 | 145.3 s |
| No speed search | 1/3 | 0/1 | 124.0 s |
| No observation | 3/3 | 0/0 | 135.6 s |
| Full segment | 0/3 | 0/0 | — |

Medians condition on completion; they cannot rank policies with different
completion counts. No-observation completed all three cases faster than the
complete policy, so these experiments do not demonstrate a benefit from viewing
moves. The reverse complete-policy trial never used a viewing move, and its time
difference includes execution variability. Initial point arrays in the divergent
restart matched exactly for all nine devices; the cause remains unresolved.

The sole body violation was 0.0015 m² of walker-box overlap with stationary ego,
without a collision-sensor event. It remains a failure. The complete policy's
minimum walker clearance was 0.279 m with ego stationary, not a comfortable
moving clearance or evidence of real-world safety.

Twenty saved event acquisitions passed alignment/device-count/positive walker
return checks (13–1272 points). The failed 13.7 s trial ended before the 20 s
recording, so its event audit is unavailable, not passed. Policy execution checks
passed. The affected-component regression selection passed 414 tests; see
`logs/parking_ablation_v12_release_checks.txt`.

Numerical outputs: `logs/parking_ablation_v12_report/report.json` and `trials.csv`.
The seven-page English manuscript and Overleaf sources are supplied separately as
`papers/visibility_parking_ablation.pdf` and
`papers/visibility_parking_ablation_overleaf.zip`. Repeated independent scenarios,
real sensor data, perturbed localization/calibration and latency evaluation
remain necessary; the work is not claimed submission-ready or statistically
superior to other parking methods.
