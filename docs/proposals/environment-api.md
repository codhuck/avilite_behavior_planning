# Environment observations: discussion proposal

Status: proposed on 25 November 2026. This document refines the local contract;
it does not describe an accepted AVLite API or a completed core migration.

## Decision to request

Adopt a small, optional environment-observation contract in AVLite's shared data
layer. Use one canonical definition of each interoperable message type. Keep
inference, tracking, geometric association, fusion policy and driving rules in
strategies/plugins. Do not add another top-level strategy selector for this work.

The earlier note left road types inside the planning plugin. That is sufficient
for the local demonstration, but an independent detector would then need to
import our planning plugin or duplicate its types. Duplicates do not satisfy the
current consumer's `isinstance` checks. The recommendation here is to place the
agreed, data-only road messages in a separate optional core module as well.
This is a proposed change in ownership, not a claim that it has already happened.

## Proposed ownership

| Component | Owner | Responsibility |
| --- | --- | --- |
| `EnvironmentMarker`, `EnvironmentObservation` | Shared core data module | Common identity, coordinates, timestamps and validity |
| `TrafficSignal`, `CrossingOccupancy` | Optional shared road-data module | Canonical message schemas understood by independent producers and consumers |
| `TrafficSign` | Same road-data module, after its schema is agreed | Observed sign code and optional value/unit; no recognition algorithm |
| `PerceptionModel.environment_observation` | Core PM | Optional typed reference, default `None`; clear on reset |
| Extensible capability key | Core capability API | Equality, naming and requirement matching alongside existing enum capabilities |
| Detectors and fusion | Plugins / perception strategies | Produce observations from sensors; declare their actual capabilities |
| Route association and behavior | Planning/perception plugins, as configured | Decide which observed object applies to the route and what to do |
| Simulator truth producer | Explicit simulation adapter | Publish the same message shape with an explicit truth origin |

Suggested locations are `c50_common/c57_environment_observations.py` and
`c50_common/c58_road_observations.py`; filenames can follow the maintainers'
convention. Neither module should import CARLA, ROS, a neural network or a
planning strategy. Existing object/agent tracking stays in its current PM fields.

## Proposed fields

Keep the snapshot immutable. A producer constructs a new instance rather than
editing a dictionary while the planner reads it.

| Type | Fields | Meaning |
| --- | --- | --- |
| `EnvironmentMarker` | `id`, optional `position`, optional `confidence` | Stable producer-local ID; xyz in the snapshot frame; confidence in the reported semantic result |
| `EnvironmentObservation` | `time`, `markers`, `source`, `frame_id`, `status`, `origin` | Complete snapshot; common clock; producer identity; coordinate frame; health; sensor/truth/scenario provenance |
| `TrafficSignal` | Base fields, `state`, optional `map_ref` | Red/Yellow/Green/Off/Unknown; optional associated map-feature ID |
| `CrossingOccupancy` | Base fields, `occupied`, optional `map_ref` | `True` occupied, `False` observed clear, `None` unknown |
| Proposed `TrafficSign` | Base fields, `code`, optional `value`, `unit`, `map_ref` | Explicitly namespaced sign code; agreed units; missing/unsupported meaning remains unknown |

`confidence=None` means not supplied. A number is not automatically a calibrated
probability, and must not by itself establish that a crossing is safe to enter.
A point is optional: an associated map object can be useful without a new 3D
position estimate. Crossing geometry continues to come from the associated map
or a separately specified geometry provider; a point does not describe its area.

Identity and applicability must be separate. A detected signal's tracking ID is
not necessarily the map's signal ID. `map_ref` expresses an association; it does
not prove that a light controls the ego lane. The road planner must still check
route applicability. For the present route-based consumer, an unassociated
signal cannot authorize movement merely because it is green. The local implementation now separates these fields. The legacy adapter uses
map-feature dictionary keys as `map_ref` and generates prefixed marker IDs;
independent typed producers supply the association explicitly.

There is no closed enum of all environment object kinds. Other domains can add
subclasses, such as docking markers. A fixed set of states inside the agreed
traffic-signal schema is compatible with that extensibility. Arrow/aspect and
flashing semantics are not covered by the current vertical three-aspect model;
unsupported aspects must not be silently reduced to Green.

## Clock, frames and publication

1. Observation time should represent the evidence acquisition time in the
   executor's clock domain, not the later completion time of inference.
   `SensorFrame.stamp` is assembly time; individual sensor stamps are acquisition
   times. A fused snapshot uses the oldest evidence contributing to its values
   until per-marker timestamps are needed. Old observations must not receive a
   fresh timestamp merely because a producer publishes again.
2. Simulation uses simulation time. Hardware adapters map sensor timestamps to
   a common runtime clock. The contract should not require a simulator clock
   on a real vehicle. Reset clears observations and any clock-mapping state.
3. Positions have an explicit frame. The producer/association stage transforms
   to the consumer's requested frame. The initial road consumer accepts `map`;
   unknown or unsupported frames are unusable, not implicitly transformed.
4. Start with one configured publisher of the final PM field. With multiple
   detectors, an explicit perception fusion stage combines their results and
   publishes once. Independent plugins must not overwrite each other's PM value.
   A conflicting or expired input becomes unknown under an explicit fusion
   policy; do not silently pick the newest green signal. This proposal does not
   introduce a general fusion framework or asynchronous channel registry.
5. No observation, a missing required object, bad status, an unsupported frame,
   or stale/future evidence means unknown. Only positive evidence of an observed
   clear crossing produces `False`. A capability declaration is not permission
   to drive. The behavior policy handles unknown according to the scenario.
6. A sensor configuration never falls back to truth automatically. Explicit
   `origin` values distinguish sensor, simulator-truth and scripted observations.
   Producer identity (`source`) and evidence origin are separate concepts.

The local detector now preserves acquisition time through inference, using the
oldest contributing sensor stamp. An explicit `sensor_clock_offset` maps it into
the executor clock. The default simulation mode anchors frame assembly time;
it preserves capture-to-assembly age but cannot detect a delayed first packet
whose assembly is also old. Hardware synchronization and drift handling remain
adapter responsibilities. See [configuration details](../environment-observations.md).

`origin` and `map_ref` are implemented locally. Sensor profiles reject truth,
scripted and unlabelled evidence. Conflicting values for one map feature in a
snapshot become unknown. A general single-writer/fusion composition policy and
canonical core migration remain proposals.

## Capabilities

| Capability | Producer promises | Consumer requirement |
| --- | --- | --- |
| Existing `ENVIRONMENT_SIGNAL_OBSERVATIONS` | Signals in the agreed schema, including Unknown | Route contains traffic lights |
| Existing `ENVIRONMENT_CROSSING_OBSERVATIONS` | Crossing occupancy including explicit unknown | Route contains crossings |
| Proposed `ENVIRONMENT_SIGN_OBSERVATIONS` | Signs in the agreed schema | A selected behavior depends on live sign recognition |

Do not add capabilities for individual colors, STOP-versus-yield signs, model
brands or simulators. Do not claim sign recognition because a STOP sign was read
from an HD map. Our current STOP-only route does not need live sign perception.

Retain existing `StackCapability` enums for compatibility. Formalize the named
key currently used by the plugin as a supported extension of the core capability
API, including annotations, profile persistence and serializers. Keep the exact
existing signal/crossing names during migration. Core-owned declarations avoid
spelling drift across independent plugins. `DETECTION` alone does not imply any
of these semantic capabilities. The matching logic already accepts these keys
locally; full upstream typing/persistence/ROS integration is not yet established.

Missing provider at assembly or handover is a configuration error. Missing data
from an installed provider is a runtime unknown. These cases should have distinct
diagnostics. Truth adapters may advertise the same schema capabilities, but a
sensor-only profile must separately reject truth-origin producers.

## Migration and acceptance

After agreement, move canonical data types to core and re-export them from the
plugin for compatibility; do not create a second, similar class definition.
Keep the legacy `RegulatoryObservation` adapter during migration. Preserve the
rule that a present unusable sensor snapshot cannot fall back to legacy truth.

Before calling the shared API complete:

- An independent producer imports only shared core data types, not our planning
  plugin, and drives the same consumer using the declared capabilities.
- Delayed inference retains the acquisition timestamp; old, future and
  wrong-frame observations cannot authorize movement.
- Missing objects, conflicting signals, reset and producer replacement preserve
  unknown semantics. Explicit clear and unknown crossing states remain distinct.
- Provider identity, map association and sensor/truth provenance survive the
  selected serialization format. Pickle round-trip alone is not a ROS test.
- Existing enum capabilities and profiles still work; named keys survive reload
  and configuration round-trip. There is a documented ROS transport adapter
  before claiming support across ROS workers.

The current main-branch suite covers the local contract, independent test
producers, requirement refusal, stale/future/frame handling, and executor
handover. It also covers capture-time propagation, explicit provenance, map association
and conflicting observations locally. It does not establish a canonical core
API, hardware clock synchronization or ROS transport. The proposal requires no
change to the neural-agent branch.

## Questions for the meeting

1. Do we agree that shared data types, including common road messages, belong
   in the core while algorithms and behavior stay in plugins?
2. Can the core formally accept named capability keys alongside the existing
   enum, retaining the current signal/crossing names?
3. Is one final PM publisher with an optional fusion stage an acceptable first
   version, with transport adapters treated as a separate implementation step?

These are requests for architectural agreement, not a report that the team has
already approved the design. No message or PR has been sent.
