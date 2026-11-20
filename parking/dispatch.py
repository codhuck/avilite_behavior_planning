"""Visibility-aware arbitration with an explicit observation/stop contract.

No simulator API or privileged obstacle list is accepted. Safety checks are
conditional on the supplied free-space estimate, localization and motion bound;
they are not a formal guarantee about unobserved real-world objects.
"""

from dataclasses import dataclass, replace

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon


@dataclass(frozen=True)
class VisibilitySnapshot:
    free: object
    occupied: object
    stamp: float
    sources: tuple = ()
    minimum_support: int = 1
    hazard_support_radius: float = 0.0


@dataclass(frozen=True)
class Maneuver:
    name: str
    kind: str  # park or observe
    parking_type: str
    poses: np.ndarray
    direction: int
    speed: float
    gain: float = 0.0
    rear_offset: float = 0.0


@dataclass(frozen=True)
class Arbitration:
    state: str
    reason: str
    maneuver: Maneuver | None = None
    certificate: object = None


@dataclass(frozen=True)
class MotionCertificate:
    """Audit of a conditional geometric check, not a formal safety proof."""

    observation_stamp: float
    checked_at: float
    sources: tuple
    minimum_support: int
    speed_bound: float
    stopping_distance: float
    clearance: float
    released_length: float
    endpoint: tuple
    hazard_support_radius: float = 0.0


def body(pose, length, width, margin=0):
    x, y, yaw = pose
    corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])
    corners = corners * [length / 2 + margin, width / 2 + margin]
    c, s = np.cos(yaw), np.sin(yaw)
    return Polygon(corners @ np.array([[c, s], [-s, c]]) + [x, y])


def swept_area(poses, length, width, margin):
    """Convex hull between adjacent footprints; callers limit pose spacing."""
    return shapely.union_all(swept_cells(poses, length, width, margin))


def swept_cells(poses, length, width, margin):
    """Vectorized segment footprints, retaining the path order for certification."""
    poses = np.asarray(poses)
    corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * [
        length / 2 + margin,
        width / 2 + margin,
    ]
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    rotations = np.stack((c, s, -s, c), axis=1).reshape(-1, 2, 2)
    coordinates = corners[None, :, :] @ rotations + poses[:, None, :2]
    shapes = shapely.polygons(coordinates)
    return shapely.convex_hull(shapely.union(shapes[:-1], shapes[1:]))


def expected_gain(endpoint, snapshot, target, sensor_range=12, spacing=0.5):
    """Count unknown target samples potentially visible past known obstacles.

    This is a view-ranking heuristic, never evidence for traversability.
    Unknown obstacles may prevent the predicted information from being gained.
    """
    x0, y0, x1, y1 = target.bounds
    samples = []
    for x in np.arange(x0 + spacing / 2, x1, spacing):
        for y in np.arange(y0 + spacing / 2, y1, spacing):
            p = Point(x, y)
            if target.covers(p) and not snapshot.free.covers(p):
                samples.append((x, y))
    origin = np.asarray(endpoint[:2])
    return spacing**2 * sum(
        np.linalg.norm(np.asarray(p) - origin) <= sensor_range
        and not LineString([origin, p]).intersects(snapshot.occupied)
        for p in samples
    )


class VisibilityDispatcher:
    """Select parking first, then useful checked observation, otherwise stop."""

    def __init__(
        self,
        max_age=0.3,
        margin=0.15,
        deceleration=1.0,
        latency=0.1,
        unseen_speed=1.5,
        max_views=8,
        min_gain=0.1,
        allow_prefix=False,
        adaptive_speed=False,
    ):
        values = [max_age, margin, deceleration, latency, unseen_speed, min_gain]
        if not np.isfinite(values).all() or min(values) < 0 or deceleration <= 0 or max_views < 0:
            raise ValueError("Invalid arbitration bounds")
        self.max_age, self.margin = max_age, margin
        self.deceleration, self.latency = deceleration, latency
        self.unseen_speed, self.max_views, self.min_gain = unseen_speed, max_views, min_gain
        self.allow_prefix = allow_prefix
        self.adaptive_speed = adaptive_speed
        self.reset()

    def reset(self):
        self.rejections = {}
        self.views = 0
        self.visited = []
        self.last_stamp = None
        self._prefix_limits = {}

    def complete_view(self, endpoint):
        self.views += 1
        self.visited.append(np.asarray(endpoint[:2], dtype=float))

    def admissible(self, candidate, snapshot, now, pose, velocity, length, width):
        def reject(reason):
            self.rejections[candidate.name] = reason
            return False

        p = np.asarray(candidate.poses)
        if (
            p.ndim != 2
            or p.shape[1] != 3
            or len(p) < 2
            or not np.isfinite(p).all()
            or candidate.direction not in (-1, 1)
            or candidate.kind not in ("park", "observe")
            or not np.isfinite([candidate.speed, candidate.gain]).all()
            or not 0 < candidate.speed <= 0.6
        ):
            return reject("invalid_candidate")
        if np.linalg.norm(p[0, :2] - pose[:2]) > 0.15:
            return reject("start_position_mismatch")
        heading = np.arctan2(np.sin(p[0, 2] - pose[2]), np.cos(p[0, 2] - pose[2]))
        if abs(heading) > np.deg2rad(10):
            return reject("start_heading_mismatch")
        if not np.isfinite(candidate.rear_offset) or candidate.rear_offset < 0:
            return reject("invalid_rear_offset")
        reference = p[:, :2] - candidate.rear_offset * np.column_stack(
            (np.cos(p[:, 2]), np.sin(p[:, 2]))
        )
        lengths = np.linalg.norm(np.diff(reference, axis=0), axis=1)
        if np.max(lengths) > 0.1 or np.min(lengths) < 1e-6:
            return reject("sampling_gap")
        # Reject teleports, lateral slides and infeasible curvature.
        headings = np.arctan2(np.sin(np.diff(p[:, 2])), np.cos(np.diff(p[:, 2])))
        if np.max(np.abs(headings) / lengths) > 0.25:
            return reject("curvature")
        travel = np.diff(reference, axis=0) / lengths[:, None]
        facing = candidate.direction * np.column_stack((np.cos(p[:-1, 2]), np.sin(p[:-1, 2])))
        if np.min(np.sum(travel * facing, axis=1)) < 0.98:
            return reject("nonholonomic_direction")
        speed = max(abs(velocity), candidate.speed)
        stop_time = self.latency + speed / self.deceleration
        stop_distance = speed * self.latency + speed**2 / (2 * self.deceleration)
        if lengths.sum() < stop_distance:
            return reject("insufficient_stopping_distance")
        inflation = self.margin + self.unseen_speed * (now - snapshot.stamp + stop_time)
        cells = swept_cells(p, length, width, inflation)
        current = body(pose, length, width, inflation)
        shapely.prepare(snapshot.free)
        shapely.prepare(snapshot.occupied)
        covered = shapely.covers(snapshot.free, cells)
        clear = ~shapely.intersects(snapshot.occupied, cells)
        failures = np.flatnonzero(~(covered & clear))
        self._prefix_limits[candidate.name] = int(failures[0]) + 1 if len(failures) else len(p)
        if not snapshot.free.covers(current) or current.intersects(snapshot.occupied):
            self._prefix_limits[candidate.name] = 0
            return reject("unobserved_stopping_envelope")
        if not np.all(covered):
            return reject("unobserved_stopping_envelope")
        if not np.all(clear):
            return reject("occupied_stopping_envelope")
        return True

    def select(
        self,
        candidates,
        snapshot,
        now,
        pose,
        velocity,
        length,
        width,
        allowed_types,
        *,
        min_view_utility=None,
    ):
        self.rejections = {}
        if min_view_utility is not None and (
            not np.isfinite(min_view_utility) or min_view_utility <= 0
        ):
            return Arbitration("STOP", "invalid_view_utility_threshold")
        if (
            not np.isfinite([now, snapshot.stamp, velocity, length, width, *pose]).all()
            or min(length, width) <= 0
        ):
            return Arbitration("STOP", "invalid_state")
        if not 0 <= now - snapshot.stamp <= self.max_age:
            return Arbitration("STOP", "stale_or_future_observation")
        if self.last_stamp is not None and snapshot.stamp < self.last_stamp:
            return Arbitration("STOP", "out_of_order_observation")
        self.last_stamp = snapshot.stamp
        if not snapshot.free.is_valid or not snapshot.occupied.is_valid or snapshot.free.is_empty:
            return Arbitration("STOP", "unknown_space")

        def release(state, reason, candidate):
            speed = max(abs(velocity), candidate.speed)
            certificate = MotionCertificate(
                snapshot.stamp,
                now,
                snapshot.sources,
                snapshot.minimum_support,
                speed,
                speed * self.latency + speed**2 / (2 * self.deceleration),
                self.margin
                + self.unseen_speed
                * (now - snapshot.stamp + self.latency + speed / self.deceleration),
                float(np.linalg.norm(np.diff(candidate.poses[:, :2], axis=0), axis=1).sum()),
                tuple(float(v) for v in candidate.poses[-1]),
                snapshot.hazard_support_radius,
            )
            return Arbitration(state, reason, candidate, certificate)

        admissible = []
        if self.adaptive_speed:
            candidates = [
                replace(candidate, name=f"{candidate.name}@{speed:g}", speed=speed)
                for candidate in candidates
                for speed in (
                    [candidate.speed] + [s for s in (0.2, 0.1, 0.05) if s < candidate.speed]
                )
            ]
        for candidate in candidates:
            if candidate.parking_type not in allowed_types:
                continue
            if self.admissible(candidate, snapshot, now, np.asarray(pose), velocity, length, width):
                admissible.append(candidate)
            elif (
                self.allow_prefix
                and candidate.kind == "park"
                and self.rejections.get(candidate.name)
                in ("unobserved_stopping_envelope", "occupied_stopping_envelope")
            ):
                # Use the first invalid segment from the ordered coverage
                # check, then revalidate stopping distance for that prefix.
                limit = self._prefix_limits.get(candidate.name, 0)
                best = None
                if limit >= 2:
                    prefix = replace(candidate, poses=candidate.poses[:limit])
                    if self.admissible(
                        prefix, snapshot, now, np.asarray(pose), velocity, length, width
                    ):
                        best = prefix
                if best is not None:
                    admissible.append(best)
                    self.rejections.pop(candidate.name, None)
        parking = [c for c in admissible if c.kind == "park"]
        if parking:
            if self.adaptive_speed:
                best = max(
                    parking,
                    key=lambda c: min(
                        c.speed, 0.5 * np.linalg.norm(np.diff(c.poses[:, :2], axis=0), axis=1).sum()
                    ),
                )
            else:
                best = min(
                    parking,
                    key=lambda c: np.linalg.norm(np.diff(c.poses[:, :2], axis=0), axis=1).sum(),
                )
            return release("PARK", "observed_maneuver", best)
        if self.views >= self.max_views:
            return Arbitration("STOP", "observation_budget_exhausted")
        views = [
            c
            for c in admissible
            if c.kind == "observe"
            and (
                c.gain >= self.min_gain
                if min_view_utility is None
                else c.gain / (1 + np.linalg.norm(np.diff(c.poses[:, :2], axis=0), axis=1).sum())
                >= min_view_utility
            )
            and all(np.linalg.norm(c.poses[-1, :2] - p) > 0.4 for p in self.visited)
        ]
        if views:
            best = max(
                views,
                key=lambda c: (
                    c.gain / (1 + np.linalg.norm(np.diff(c.poses[:, :2], axis=0), axis=1).sum())
                ),
            )
            return release("OBSERVE", "expected_visibility_gain", best)
        return Arbitration("STOP", "no_admissible_maneuver")
