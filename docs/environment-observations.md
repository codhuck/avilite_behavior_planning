# Environment observations

`perception/environment.py` contains a small proposed shared contract. It is
implemented in the plugin while the core API is under discussion. It does not
add another detector dropdown or replace AVLite's existing perception pipeline.

A producer assigns a complete `EnvironmentObservation` to
`perception_model.environment_observation`. The snapshot is immutable and has a
simulation-clock timestamp, coordinate frame, source and status. Marker IDs are
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
    time=pm.simulation_time,
    source="front_camera_lidar",
    frame_id="map",
    markers=(
        TrafficSignal(id="signal-12", state="Red", confidence=0.94),
        CrossingOccupancy(id="crossing-4", occupied=None),
    ),
)
```

`None` crossing occupancy is unknown. `False` requires observed free space.
Road planning currently accepts the `map` frame only; another frame is rejected
as unusable rather than treated as map coordinates. Consumers reject stale and
future snapshots. The RGB/LiDAR detector publishes an empty, diagnostic snapshot
on invalid input instead of retaining an earlier green signal.

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

Proposed core additions are the domain-neutral marker and snapshot types, an
optional PM field, and an extensible named capability key. Road subclasses and
recognition algorithms stay in plugins. The local development PM already clears
its optional observation on reset. Stock 0.6.7 can carry it as a plugin-added PM
attribute; resetting or replacing a sensor pipeline must discard that value.

Open questions for maintainers are the final module/name, whether multiple
producers publish separate channels, frame conversion ownership, and ROS wire
encoding. These names are not an accepted upstream API. The Python message is
pickle-round-trippable; that is not a ROS topic integration test. No core PR or
chat message has been sent.
