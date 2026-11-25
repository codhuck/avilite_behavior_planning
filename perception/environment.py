"""Optional environment observations; candidate shared API for AVLite core.

A producer publishes one complete snapshot in PM.environment_observation.
Plugins extend EnvironmentMarker; the core does not enumerate object kinds.
Times use the executor clock, positions use frame_id coordinates.
"""

import math
from dataclasses import dataclass


class ObservationCapability(str):
    """An extensible capability key compatible with AVLite requirement checks."""

    @property
    def name(self):
        return str(self)


SIGNAL_OBSERVATIONS = ObservationCapability("ENVIRONMENT_SIGNAL_OBSERVATIONS")
CROSSING_OBSERVATIONS = ObservationCapability("ENVIRONMENT_CROSSING_OBSERVATIONS")


@dataclass(frozen=True, kw_only=True)
class EnvironmentMarker:
    id: str
    position: tuple[float, float, float] | None = None
    confidence: float | None = None

    def __post_init__(self):
        if not isinstance(self.id, str) or not self.id.strip():
            raise ValueError("An environment marker requires an id")
        if self.position is not None and (
            not isinstance(self.position, tuple)
            or len(self.position) != 3
            or not all(math.isfinite(v) for v in self.position)
        ):
            raise ValueError("Marker position must be a finite xyz tuple")
        if self.confidence is not None and not 0 <= self.confidence <= 1:
            raise ValueError("Marker confidence must be between zero and one")


@dataclass(frozen=True, kw_only=True)
class TrafficSignal(EnvironmentMarker):
    state: str = "Unknown"
    map_ref: str | None = None

    def __post_init__(self):
        super().__post_init__()
        if self.state not in {"Red", "Yellow", "Green", "Off", "Unknown"}:
            raise ValueError("Unsupported traffic signal state")
        if self.map_ref is not None and (
            not isinstance(self.map_ref, str) or not self.map_ref.strip()
        ):
            raise ValueError("Map reference must be a nonempty string or None")


@dataclass(frozen=True, kw_only=True)
class CrossingOccupancy(EnvironmentMarker):
    occupied: bool | None = None
    map_ref: str | None = None

    def __post_init__(self):
        super().__post_init__()
        if self.occupied is not None and type(self.occupied) is not bool:
            raise ValueError("Crossing occupancy must be true, false or unknown")
        if self.map_ref is not None and (
            not isinstance(self.map_ref, str) or not self.map_ref.strip()
        ):
            raise ValueError("Map reference must be a nonempty string or None")


@dataclass(frozen=True)
class EnvironmentObservation:
    time: float
    markers: tuple[EnvironmentMarker, ...] = ()
    source: str = ""
    frame_id: str = "map"
    status: str = "ok"
    origin: str = "unknown"

    def __post_init__(self):
        if not math.isfinite(self.time) or self.time < 0:
            raise ValueError("Observation time must be finite and nonnegative")
        if not isinstance(self.origin, str) or self.origin not in {
            "sensor",
            "simulator_truth",
            "scenario",
            "unknown",
        }:
            raise ValueError("Unsupported observation origin")
        if not self.frame_id or not isinstance(self.markers, tuple):
            raise ValueError("Observation requires a frame and an immutable marker tuple")
        if not all(isinstance(marker, EnvironmentMarker) for marker in self.markers):
            raise ValueError("Observation contains an invalid marker")
        keys = [(type(marker), marker.id) for marker in self.markers]
        if len(set(keys)) != len(keys):
            raise ValueError("Observation contains duplicate marker ids of the same type")

    def is_fresh(self, now, max_age):
        return math.isfinite(now) and math.isfinite(max_age) and 0 <= now - self.time <= max_age
