"""Shared perception data contract, independent of simulator APIs."""

from dataclasses import dataclass, field


@dataclass
class RegulatoryObservation:
    time: float
    lights: dict[str, str] = field(default_factory=dict)
    pedestrians: list[tuple[float, float]] = field(default_factory=list)
