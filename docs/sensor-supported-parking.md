# Sensor-supported parking arbitration

The later [robustness validation](validation/parking-robustness-validation.md) records new
geometry configurations, moving pedestrians, range noise, retained failures and
subsequent fixes. Results below describe the earlier static experiments.

## Scope and research hypothesis

This experimental method joins calibrated LiDAR fusion, explicit evidence
requirements, speed selection and executable trajectory prefixes. It is an
engineering implementation and a candidate research contribution, not a claim
of proven novelty, formal safety or real-vehicle readiness.

Hypothesis: requiring corroborating sensors near observed hazards, while allowing
single-source support elsewhere and adapting speed to the certified path length,
can improve progress compared with uniform two-source requirements with fewer
false-free errors than unrestricted single-source fusion. This must be tested on
held-out layouts and moving hazards; tuning runs alone cannot establish it.

## Prior work and distinction to test

- [Look before you sweep (WAFR 2018)](https://arxiv.org/abs/1901.06109)
  addresses visibility-constrained motion for the whole robot. Observation before
  entering unknown space is not our novelty.
- [Limited Visibility Aware Motion Planning for Autonomous Valet Parking
  (2021)](https://www.mdpi.com/1424-8220/21/4/1520) already studies parking
  occlusions and reachable sets. Parking plus active perception is not new.
- [OA-MPC](https://arxiv.org/abs/2211.09156) incorporates unseen-agent reachability
  and terminal stopping constraints. Stopping at a certified horizon is not new.
- [APRO (2026)](https://arxiv.org/abs/2606.15046) combines occlusion reachability
  and active perception with set-based checks, and reports simulation, hardware
  and real parking-data replay. We have not reproduced or surpassed that work.

Our proposed focus is a sensor-level, auditable connection between independent
measurement support, sensor loss, admissible speed and planner arbitration.
A systematic full-text comparison and held-out evaluation are still required.

## Observation contract

Each cloud carries its own calibration and acquisition timestamp. Only clouds
at the supplied localization pose's timestamp are accepted; there is no stale
free-space persistence. Ground elevation is surveyed and the terrain is flat.
The current implementation receives simulator localization in CARLA experiments.

Ground returns confirm raster cells. Above-ground or below-ground returns are
hazards; their projected connected surfaces override ground from every device.
Support counts distinct devices, not repeated beams or duplicate clouds. Ground
observations near hazards can require two devices, independently of the minimum
support used elsewhere. This only removes weak evidence; it never invents free
space. Two sources can still share systematic error and are not a probability
of correctness or a proof that the vertical column is empty.

Space already occupied by the ego body is a separately recorded initial-condition
exception. It requires a valid collision-free starting pose and calibrated body
geometry. It does not expand the body or clear adjacent blind cells. Observed
free space and this exception remain separate in saved geometry and metrics.

## Motion contract

The geometric planner may propose a path through unknown space. That proposal
is not executable until the dispatcher checks its swept footprints against
current observations, obstacles and stopping clearance. The controller receives
only the longest certified prefix, with target speed tapering to zero at its end.
Changing gear still requires stopping. Any new command requires recertification.
Observation utility follows the current parking phase's endpoint, so a short
viewing move does not undo a necessary parallel-parking setup. A view ending
on the existing path retains that phase; off-path views trigger replanning.
CARLA uses explicit first/reverse gear and acceleration feedback with integral
release on overspeed to reduce low-speed launch surges.

Speed candidates are checked with the maximum of measured speed and requested
speed: requesting a lower speed does not erase current kinetic energy. If no
candidate is admissible, the command is STOP. Failed observation moves are
released only after stopping, so another bounded attempt can be considered.

Each released decision records a `MotionCertificate`: observation time, check
time, source identifiers, minimum support, speed bound, stopping distance,
clearance, released path length and endpoint. The word certificate means an
**audit record of the conditional geometric check**, not a formal safety proof.
Braking, hidden-object speed, pose and measurement assumptions remain essential.

For reproducibility, the clearance used by the implementation is
`margin + unseen_speed * (observation_age + latency + v / deceleration)`,
with `v = max(abs(measured_speed), candidate_speed)`. A released prefix must
also exceed `v * latency + v² / (2 * deceleration)` in rear-axle path length.
Defaults are 0.15 m margin, 1.5 m/s unseen-agent bound, 0.1 s latency and
1 m/s² braking deceleration. These are assumed bounds, not estimates validated
for every surface or pedestrian. The simulator controller can temporarily
exceed its requested speed; the next arbitration uses the measured speed.

Selection is lexicographic: prefer an admissible parking prefix; otherwise
choose a checked observation move; otherwise stop. Parking prefixes maximize
`min(candidate_speed, 0.5 * prefix_length)` across the requested speed and
lower candidates 0.2, 0.1 and 0.05 m/s. Observation moves combine predicted
visibility gain with progress toward the current phase endpoint, with eight
bounded attempts and suppression of revisited endpoints. This is a discrete
heuristic arbitration policy, not a globally optimal trajectory solver.

## Reproducibility

Merge these fields into the existing profile's `plugins` sections. The standard
plugin then reads all named frame LiDARs:

```yaml
plugins:
  avlite_behavior_hdmap:
    parking_lidar_occupancy: true
    parking_multi_lidar: true
    parking_active_observation: true
    parking_minimum_sensor_support: 1
    parking_hazard_support_radius: 2.0
    parking_ground_z: 0.0
    parking_simulator_obstacles: false
  avlite_bridge_carla:
    parking_lidar_layout: surround
```

Reload the stack while stopped. The default
`roof` preserves the ordinary bridge configuration. The bridge publishes an
RGB/roof/eight-near-field frame only when every configured device has delivered
the same simulator frame and acquisition time. A missing callback faults the
bridge; reset/reload is required after restoring the device. This differs from
the experiment's intentional observation blackout, which keeps ticking and
recovers automatically when observations resume.

The experiment runner creates its own rig on an idle exclusive CARLA server:

```bash
python avlite-behavior-hdmap/tools/check_parking_carla.py \
  --sensor --active --rig --rig-layout surround \
  --mode perpendicular_forward --output logs/parking_sensor_trial
```

`corners` uses one roof sensor and four low-mounted corner sensors; `surround`
adds four midpoint sensors. These are controlled sensor-layout experiments, not
a recommendation that a deployed car needs nine LiDAR units. This example disables noise and general
ray dropoff. The later paired study below uses range noise and ray dropout.

Recorded clouds support sensor-loss and speed counterfactuals:

```bash
python avlite-behavior-hdmap/tools/replay_parking_coverage.py \
  logs/visibility_carla_rig_v6 --hazard-support-radius 2 \
  --output logs/parking_coverage_replay_adaptive
```

This replay evaluates short probe decisions, not completed parking. Keep original
and counterfactual results distinct. Actor boxes are evaluator-only labels;
nonzero overlap is reported even when a drive avoids collisions.

## Evaluation status

The [paired study](research/itsc-study.md#completed-paired-series-13-november-2026)
completed 21 CARLA trials covering seven component policies and three parking
geometries, with fixed moving-walker trajectories, 0.01 m range noise and 0.02
ray dropout. Complete policy: 3/3; full-segment release: 0/3; no-observation: 3/3.
These are single runs per known geometry, not a generalization benchmark.
Observation-move benefit was not demonstrated. Failed trials remain in results.

`compare_parking_evidence.py` handles matched recorded frames and transformed
scenario geometry. The legacy replay example above uses initial clouds only,
without mixing in later event clouds, on its original untransformed pad.

## Remaining evaluation

Repeated trials, unseen layouts, real sensor recordings, localization and
calibration perturbations, dropped devices and timing faults remain necessary.
The comparison list below describes broader coverage, not completed claims:

Compare uniform one-source fusion, uniform two-source fusion, hazard-adaptive
support, fixed versus adaptive speed, full-path versus prefix checks and single
versus multiple sensors. Hold parking geometry and dynamics fixed. Include
perpendicular forward/reverse and parallel parking; occluding vehicles; moving
pedestrians; timestamp errors; dropped devices; occupied bays; calibration noise.

Report successful completion, collision contacts, boundary violations, false-free
area, deadlock, intervention, speed, braking after loss, simulated time and wall
compute time. Interrupted or budget-limited runs are not successes. Ground-truth
obstacle runs and ideal 2D scanner runs cannot validate sensor-only performance.

## Evidence and limits

See [sensor parking validation](validation/sensor-parking-validation.md) for the recorded
acceptance results, development failures and reproduction commands. Static
neighbouring cars occlude the bay, but there are no moving pedestrians in these
earlier LiDAR parking drives. Ground-truth localization, surveyed bay geometry and
flat ground are still supplied. The method does not detect painted spaces.

Ground returns beneath objects can still produce false-free overlap with
independent evaluator vehicle boxes. Corroboration and surface completion
reduce this on some recordings but do not eliminate it. A successful manoeuvre
is not proof that the occupancy estimate is correct. Volumetric free-space
reasoning, perturbed calibration, broader moving-hazard conditions and
repeated held-out trials remain necessary for stronger claims.
