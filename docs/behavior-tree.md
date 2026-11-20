# Reactive road behavior tree

`HDMapBehaviorTreePlanner` composes the STOP, traffic-light and crossing actions
from `HDMapBehaviorPlanner` as a reactive behavior tree. Select `road-tree` in
the coordinator to use it.

For every relevant rule in increasing route distance, a reactive selector ticks:

```text
RegulatoryRule (Selector)
├─ StopSign (Sequence): is STOP? → evaluate dwell/approach/violation
├─ TrafficLight (Sequence): is light? → evaluate current observation
└─ Crosswalk (Sequence): is crossing? → evaluate occupancy/clearance
```

A nonmatching branch fails, allowing the next branch. A matching action succeeds
with either a stop decision or permission to evaluate the next route rule.
SUCCESS means “rule evaluated”, not “permission to drive”. Unknown rule types
produce a stopping decision. The general runtime also supports RUNNING nodes;
these rule actions complete each tick and retain temporal state in the planner.

The common adapter checks RGB/LiDAR freshness before evaluating the tree,
projects rules onto the route, enforces STOP completion, and produces braking
targets. A stale or unhealthy sensor snapshot stops the vehicle before any rule
branch runs. No tree branch bypasses the velocity stage or collision checks.
Route reset clears completed stops, dwell timers and the current trace.

Sequences/selectors restart at their first child every tick. A newly true
higher-priority condition preempts a running lower-priority branch. Red/green
changes are reconsidered each tick; no prior green decision is latched forever.

## Configuration

Append `--behavior-tree` when generating a new profile with `install.py`:

```bash
python install.py --behavior-tree \
  --profile-output ~/.config/avlite/sensor-road-tree.yaml \
  --map-path /path/to/Town10HD_Opt.xodr \
  --bridge-path /path/to/avlite-bridge-carla
```

The generator sets `c20_planning.c23_behavioral_strategy` to
`HDMapBehaviorTreePlanner`. `PlannerDispatcher` honors this explicit choice for
HD maps and keeps the race pipeline on race maps. Existing profiles retain the
old road stage. Stop execution and reload the new profile in AVLite.

`LocalPlan.behavior_tree_trace` exposes `(node, status)` pairs per evaluated rule.
`SensorTelemetryTask` / `CarlaRegulatoryTask` write it as a JSON CSV field.
An empty trace with `PERCEPTION_STOP` indicates that the common sensor guard
stopped processing before tree evaluation.

## Tests

The existing regulatory tests run against both implementations for STOP dwell,
rolling stops, red/yellow/unknown/green lights, stale data, occupied/clear
crossings, waiting time and initial route position. Additional tests cover
reactive preemption, invalid node results, trace/reset and dispatcher selection.
See `test/plugins/test_hdmap_behavior_plugin.py` in the matching workspace.

The 2026-11-17 CARLA smoke check used `trained_rgb`, 30 executor steps, disabled
access to dynamic ground-truth observation providers, and injected a black
camera frame. It verified the tree stage and transition to `PERCEPTION_STOP`.
The checked bridge and behavior source paths/hashes are recorded in
`logs/meeting_tree_carla_fixed.json` in the parent workspace. This is an
integration check, not a complete route benchmark.
