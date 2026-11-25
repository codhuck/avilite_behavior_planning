"""Shared perception data contract, independent of simulator APIs."""

from dataclasses import dataclass, field


@dataclass
class RegulatoryObservation:
    time: float
    lights: dict[str, str] = field(default_factory=dict)
    pedestrians: list[tuple[float, float]] = field(default_factory=list)
    source: str = field(default="legacy", kw_only=True)
    origin: str = field(default="unknown", kw_only=True)


def environment_observation(observation):
    """Translate legacy detector output without carrying model/actor handles."""
    from .environment import CrossingOccupancy, EnvironmentObservation, TrafficSignal

    return EnvironmentObservation(
        time=observation.time,
        source=getattr(observation, "source", "legacy"),
        origin=getattr(observation, "origin", "unknown"),
        status=getattr(observation, "diagnostics", {}).get("status", "ok"),
        markers=tuple(
            [
                TrafficSignal(id=f"signal:{key}", map_ref=key, state=value)
                for key, value in observation.lights.items()
            ]
            + [
                CrossingOccupancy(id=f"crossing:{key}", map_ref=key, occupied=value)
                for key, value in getattr(observation, "crossings", {}).items()
            ]
        ),
    )


def regulatory_observation(perception_model):
    """Read the public snapshot; a present but unusable snapshot never falls back to GT."""
    from types import SimpleNamespace

    from .environment import CrossingOccupancy, TrafficSignal

    observation = getattr(perception_model, "environment_observation", None)
    if observation is None:
        return getattr(perception_model, "regulatory_observation", None)
    markers = observation.markers if observation.frame_id == "map" else ()
    lights, crossings = {}, {}
    for marker in markers:
        if isinstance(marker, TrafficSignal) and marker.map_ref is not None:
            previous = lights.get(marker.map_ref, marker.state)
            lights[marker.map_ref] = marker.state if previous == marker.state else "Unknown"
        elif isinstance(marker, CrossingOccupancy) and marker.map_ref is not None:
            previous = crossings.get(marker.map_ref, marker.occupied)
            crossings[marker.map_ref] = marker.occupied if previous == marker.occupied else None
    return SimpleNamespace(
        time=observation.time,
        lights=lights,
        crossings=crossings,
        pedestrians=[],
        source=observation.source,
        origin=observation.origin,
        diagnostics={
            "status": observation.status if observation.frame_id == "map" else "unknown_frame"
        },
    )
