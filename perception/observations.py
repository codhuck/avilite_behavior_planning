"""Shared perception data contract, independent of simulator APIs."""

from dataclasses import dataclass, field


@dataclass
class RegulatoryObservation:
    time: float
    lights: dict[str, str] = field(default_factory=dict)
    pedestrians: list[tuple[float, float]] = field(default_factory=list)


def environment_observation(observation):
    """Translate legacy detector output without carrying model/actor handles."""
    from .environment import CrossingOccupancy, EnvironmentObservation, TrafficSignal

    return EnvironmentObservation(
        time=observation.time,
        source=getattr(observation, "source", "ground_truth"),
        status=getattr(observation, "diagnostics", {}).get("status", "ok"),
        markers=tuple(
            [TrafficSignal(id=key, state=value) for key, value in observation.lights.items()]
            + [
                CrossingOccupancy(id=key, occupied=value)
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
    return SimpleNamespace(
        time=observation.time,
        lights={m.id: m.state for m in markers if isinstance(m, TrafficSignal)},
        crossings={m.id: m.occupied for m in markers if isinstance(m, CrossingOccupancy)},
        pedestrians=[],
        source=observation.source,
        diagnostics={
            "status": observation.status if observation.frame_id == "map" else "unknown_frame"
        },
    )
