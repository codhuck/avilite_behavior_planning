# Environment observations

`perception/environment.py` contains a small proposed shared contract. It is
implemented in the plugin while the core API is under discussion. It does not
add another detector dropdown or replace AVLite's existing perception pipeline.

A producer assigns a complete `EnvironmentObservation` to
`perception_model.environment_observation`. The snapshot is immutable and has a
capture timestamp in the executor clock, coordinate frame, source, origin and status. Marker IDs are
unique within each concrete marker type. A missing marker means unknown, not
clear. A present snapshot takes precedence over the legacy regulatory field;
unusable sensor data never falls back to simulator ground truth.

Road extensions are `TrafficSignal` and `CrossingOccupancy`. Other plugins can
subclass `EnvironmentMarker`, for example for a docking marker, without editing
a central enum. Signal state is deliberately a fixed set within the road type.
No simulator handles, models or images belong in this message.

```python
from avlite.plugins.avlite_behavior_hdmap import (
    EnvironmentObservation,
    TrafficSignal,
    CrossingOccupancy,
)

pm.environment_observation = EnvironmentObservation(
    time=capture_time_in_executor_clock,
    source="front_camera_lidar",
    origin="sensor",
    frame_id="map",
    markers=(
        TrafficSignal(id="track-12", map_ref="signal-12", state="Red", confidence=0.94),
        CrossingOccupancy(id="observed-area-4", map_ref="crossing-4", occupied=None),
    ),
)
```

`None` crossing occupancy is unknown. `False` requires observed free space.
Road planning currently accepts the `map` frame only; another frame is rejected
as unusable rather than treated as map coordinates. Consumers reject stale and
future snapshots. The RGB/LiDAR detector publishes an empty, diagnostic snapshot
on invalid input instead of retaining an earlier green signal.

## Identity and origin

`id` identifies an observation within its producer. `map_ref` separately names
an associated map feature. Road rules ignore markers without an association;
a green signal with no `map_ref` cannot clear a mapped light. Existing producers
of shared messages must now supply this field. Legacy `RegulatoryObservation`
dictionaries remain supported: their keys are map references, and the adapter
creates prefixed marker IDs. This does not add object tracking to the detector.
Conflicting states for the same map reference become unknown, independent of
marker order. This is a snapshot conflict check, not a multi-producer fusion stage.

`source` names the producer; `origin` describes its evidence:
`sensor`, `simulator_truth`, `scenario`, or `unknown`. Untagged legacy producers
remain `unknown`. CARLA truth observations explicitly use `simulator_truth`.
Scripted producers should set `origin="scenario"` when constructing observations.
These labels are producer declarations, not proof of physical sensor provenance.

The three sensor-road profiles set `required_observation_origin: sensor`.
The behavior planner stops on missing or differently labelled evidence, even
on a route without lights. Other profiles default to `any` for compatibility.
Bad status, old evidence and unknown objects still apply after this origin check.
The CARLA truth task does not replace an existing sensor-origin observation.

## Capture time

The RGB/LiDAR detector timestamps valid observations with the older of the two
sensor acquisition times, mapped into the executor clock. Inference does not
refresh that timestamp. Repeated, future or stale frames publish an empty fault
snapshot, so a previous green result cannot persist through a failed update.
`is_fresh()` only checks time; consumers must also check status and applicability.

`sensor_clock_offset` is executor time minus sensor time, in seconds. Set it to
`0.0` when both use the same clock, or to a calibrated offset for hardware. With
an explicit offset, a stale first frame is rejected without establishing an
anchor from that frame.

The default `null` supports the simulation clock offset: the first frame's
assembly stamp is anchored to the current executor time. Sensor age relative
to assembly is preserved, including on the first frame. This cannot detect a
first packet whose assembly and sensor data have both been delayed in a queue.
Use a known clock offset for that case. This setting does not estimate hardware
clock drift or replace synchronization. Backward executor time resets the anchor.

## Capabilities

Providers advertise `SIGNAL_OBSERVATIONS` and/or `CROSSING_OBSERVATIONS` from
`perception.environment`. These are extensible named keys accepted by AVLite's
existing requirement checks, not new members added to its enum. A provider can
support only one. `DETECTION` alone does not satisfy either requirement.

The road planner requests the capabilities needed by its route. A STOP-only
route needs no live signal provider. Routes with lights or crossings require
the corresponding provider at assembly and handover. Sensor perception declares
both; the explicitly selected CARLA truth task also declares both. Telemetry
alone declares neither. A missing provider is a configuration error. An
installed provider with a missing observation is a runtime unknown state.

`RegulatoryObservation` remains supported for existing scenario adapters.
`regulatory_observation(pm)` translates a shared snapshot for the road rules;
the planner does not call the sensor detector. Model inference stays in
`SensorRegulatoryDetector`, which is a normal AVLite detection stage.

## Core proposal

The [25 November discussion proposal](proposals/environment-api.md) refines the
recommended ownership: canonical road message types should also be shared, so
independent producers do not depend on this planning plugin. The description
below records the existing local implementation; the proposed migration has
not been applied. [Meeting notes](proposals/environment-api-meeting.md) summarize
the design and distinguish implemented behavior from the remaining proposal.

Currently the marker types, road subclasses and named capability key all live
in this plugin. The discussion proposal recommends moving the canonical data
types and capability API into the core while keeping recognition algorithms in
plugins. The local development PM already clears its optional observation on
reset. Stock 0.6.7 can carry it as a plugin-added PM attribute; resetting or
replacing a sensor pipeline must discard that value.

Open questions for maintainers are the final module/name, whether multiple
producers publish separate channels, frame conversion ownership, and ROS wire
encoding. These names are not an accepted upstream API. The Python message is
pickle-round-trippable; that is not a ROS topic integration test. No core PR or
chat message has been sent.
