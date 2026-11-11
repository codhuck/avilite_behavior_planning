"""Extract regulatory elements from an AVLite OpenDRIVE :class:`HDMap`.

AVLite's core parser intentionally keeps the original XML elements on every
road.  This module turns the relevant ``<signal>`` and ``<object>`` entries
into small world-coordinate records without modifying the core map model.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np


class RegulatoryKind(str, Enum):
    STOP_SIGN = "stop_sign"
    CROSSWALK = "crosswalk"
    TRAFFIC_LIGHT = "traffic_light"


@dataclass(frozen=True)
class RegulatoryFeature:
    id: str
    kind: RegulatoryKind
    road_id: str
    s: float
    t: float
    x: float
    y: float
    orientation: str = "none"
    name: str = ""
    dynamic: bool = False
    lane_validity: tuple[tuple[int, int], ...] = ()
    is_reference: bool = False
    stop_line_s: float | None = None
    polygon: tuple[tuple[float, float], ...] = ()


def road_st_to_xy(road, s: float, t: float) -> tuple[float, float]:
    """Convert OpenDRIVE road coordinates ``(s, t)`` to map ``(x, y)``.

    ``road.center_line`` is AVLite's sampled OpenDRIVE reference line.  Positive
    ``t`` points to its left, matching the OpenDRIVE convention.
    """

    points = np.asarray(road.center_line, dtype=float).T
    if points.ndim != 2 or points.shape[0] == 0 or points.shape[1] < 2:
        raise ValueError(f"Road {getattr(road, 'id', '?')} has no usable center line")
    if points.shape[0] == 1:
        return float(points[0, 0]), float(points[0, 1])

    delta = np.diff(points, axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    s_clamped = float(np.clip(s, 0.0, cumulative[-1]))
    segment = min(int(np.searchsorted(cumulative, s_clamped, side="right") - 1), len(delta) - 1)
    segment = max(0, segment)

    # Skip a degenerate sampled segment if one survived map parsing.
    if lengths[segment] <= 1e-9:
        valid = np.flatnonzero(lengths > 1e-9)
        if len(valid) == 0:
            return float(points[0, 0]), float(points[0, 1])
        segment = int(valid[np.argmin(np.abs(cumulative[valid] - s_clamped))])

    alpha = (s_clamped - cumulative[segment]) / lengths[segment]
    reference = points[segment] + alpha * delta[segment]
    tangent = delta[segment] / lengths[segment]
    left_normal = np.array([-tangent[1], tangent[0]])
    xy = reference + float(t) * left_normal
    return float(xy[0]), float(xy[1])


def _signal_kind(element) -> RegulatoryKind | None:
    name = (element.get("name") or "").lower()
    signal_type = (element.get("type") or "").lower()
    if "stop" in name or signal_type == "206":
        return RegulatoryKind.STOP_SIGN
    if (element.get("dynamic") or "").lower() == "yes" or "light" in name:
        return RegulatoryKind.TRAFFIC_LIGHT
    return None


def extract_regulatory_features(hdmap) -> list[RegulatoryFeature]:
    """Return stop signs, crosswalks, and traffic lights from an AVLite HD map."""

    features: list[RegulatoryFeature] = []
    definitions = {}
    for road in getattr(hdmap, "roads", ()):
        xml = getattr(road, "road_element", None)
        if xml is not None:
            definitions.update({e.get("id"): e for e in xml.findall("signals/signal")})
    for road in getattr(hdmap, "roads", ()):  # works with HDMap and small test doubles
        road_element = getattr(road, "road_element", None)
        if road_element is None:
            continue

        for element in road_element.findall("signals/signal") + road_element.findall(
            "signals/signalReference"
        ):
            definition = definitions.get(element.get("id"), element)
            kind = _signal_kind(definition)
            if kind is None:
                continue
            s = float(element.get("s", "0"))
            t = float(element.get("t", "0"))
            x, y = road_st_to_xy(road, s, t)
            lines = [
                float(o.get("s"))
                for o in road_element.findall("objects/object")
                if (o.get("name") or "").lower() == "stopline"
            ]
            line_s = min(lines, key=lambda value: abs(value - s)) if lines else None
            # Do not associate a line at the opposite end of the road.
            if line_s is not None and abs(line_s - s) > 6.0:
                line_s = None
            features.append(
                RegulatoryFeature(
                    id=element.get("id", ""),
                    kind=kind,
                    road_id=str(road.id),
                    s=s,
                    t=t,
                    x=x,
                    y=y,
                    orientation=element.get("orientation", "none"),
                    name=definition.get("name", ""),
                    dynamic=(definition.get("dynamic", "").lower() == "yes"),
                    lane_validity=tuple(
                        (int(v.get("fromLane")), int(v.get("toLane")))
                        for v in element.findall("validity")
                    ),
                    is_reference=element.tag == "signalReference",
                    stop_line_s=line_s,
                )
            )

        for element in road_element.findall("objects/object"):
            object_type = (element.get("type") or "").lower()
            name = (element.get("name") or "").lower()
            if object_type != "crosswalk" and "crosswalk" not in name:
                continue
            s = float(element.get("s", "0"))
            t = float(element.get("t", "0"))
            x, y = road_st_to_xy(road, s, t)
            # Object u/v coordinates rotate by hdg relative to the road tangent.
            a = np.asarray(road_st_to_xy(road, max(0, s - 0.05), 0))
            b = np.asarray(road_st_to_xy(road, s + 0.05, 0))
            angle = np.arctan2(*(b - a)[::-1]) + float(element.get("hdg", "0"))
            rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
            corners = element.findall("outline/cornerLocal") + element.findall(
                "outlines/outline/cornerLocal"
            )
            if corners:
                uv = [(float(c.get("u")), float(c.get("v"))) for c in corners]
            else:
                length, width = float(element.get("length", "0")), float(element.get("width", "0"))
                uv = [
                    (-length / 2, -width / 2),
                    (length / 2, -width / 2),
                    (length / 2, width / 2),
                    (-length / 2, width / 2),
                ]
            polygon = tuple(tuple(np.array([x, y]) + rotation @ p) for p in uv)
            features.append(
                RegulatoryFeature(
                    id=element.get("id", ""),
                    kind=RegulatoryKind.CROSSWALK,
                    road_id=str(road.id),
                    s=s,
                    t=t,
                    x=x,
                    y=y,
                    orientation=element.get("orientation", "none"),
                    name=element.get("name", ""),
                    polygon=polygon,
                )
            )

    return features
