"""Task-weighted view utility from observed geometry, never motion authorization.

A proposed parking path defines relevance; predicted line of sight is optimistic
and must never be merged into measured free space. No actor labels are accepted.
"""

from dataclasses import dataclass

import numpy as np
import shapely
from shapely.geometry import LineString, Point


@dataclass(frozen=True)
class ViewUtility:
    weighted_gain: float
    revealed_area: float
    unknown_area: float
    samples: int


class TaskVisibility:
    """Cache missing swept-footprint samples and prioritize the near-term path."""

    def __init__(self, snapshot, poses, width, length, spacing=0.4, horizon=8.0, decay=4.0):
        poses = np.asarray(poses, dtype=float)
        if poses.ndim != 2 or poses.shape[1] != 3 or len(poses) < 2 or not np.isfinite(poses).all():
            raise ValueError("Expected a finite path of at least two SE(2) poses")
        if (
            not np.isfinite([width, length, spacing, horizon, decay]).all()
            or min(width, length, spacing, horizon, decay) <= 0
        ):
            raise ValueError("Expected positive finite geometry and sampling bounds")
        distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1))]
        keep = distance <= horizon
        path = poses[keep]
        if len(path) < 2:
            path = poses[:2]
        centerline = LineString(path[:, :2])
        # Footprint union defines relevance only; the execution gate remains separate.
        corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * [
            length / 2 + 0.15,
            width / 2 + 0.15,
        ]
        c, s = np.cos(path[:, 2]), np.sin(path[:, 2])
        rotations = np.stack((c, s, -s, c), axis=1).reshape(-1, 2, 2)
        footprints = shapely.polygons(corners[None, :, :] @ rotations + path[:, None, :2])
        corridor = shapely.union_all(
            shapely.convex_hull(shapely.union(footprints[:-1], footprints[1:]))
        )
        missing = corridor.difference(snapshot.free).difference(snapshot.occupied)
        self.points = np.empty((0, 2))
        self.weights = np.empty(0)
        self.spacing = spacing
        self.occupied = snapshot.occupied
        if missing.is_empty:
            return
        x0, y0, x1, y1 = missing.bounds
        xs, ys = np.meshgrid(
            np.arange(x0 + spacing / 2, x1, spacing), np.arange(y0 + spacing / 2, y1, spacing)
        )
        points = np.column_stack((xs.ravel(), ys.ravel()))
        inside = shapely.intersects_xy(missing, points[:, 0], points[:, 1])
        self.points = points[inside]
        progress = np.array([centerline.project(Point(p)) for p in self.points])
        self.weights = np.exp(-progress / decay)

    def visible(self, pose, sensor_range=12.0):
        if not np.isfinite(pose[:2]).all() or not np.isfinite(sensor_range) or sensor_range <= 0:
            raise ValueError("Invalid viewpoint or sensor range")
        origin = np.asarray(pose[:2])
        return np.array(
            [
                np.linalg.norm(p - origin) <= sensor_range
                and not LineString([origin, p]).intersects(self.occupied)
                for p in self.points
            ],
            dtype=bool,
        )

    def evaluate(self, origin, endpoint, sensor_range=12.0):
        """Predicted *incremental* visibility; already visible samples earn no gain."""
        newly_visible = self.visible(endpoint, sensor_range) & ~self.visible(origin, sensor_range)
        return ViewUtility(
            float(self.weights[newly_visible].sum() * self.spacing**2),
            float(newly_visible.sum() * self.spacing**2),
            float(len(self.points) * self.spacing**2),
            len(self.points),
        )


@dataclass(frozen=True)
class GroundLidar:
    """Calibrated level LiDAR geometry above a surveyed flat ground plane."""

    x: float
    y: float
    height: float
    vertical_angles: tuple
    sensor_range: float

    def __post_init__(self):
        values = [self.x, self.y, self.height, self.sensor_range, *self.vertical_angles]
        if (
            not np.isfinite(values).all()
            or self.height <= 0
            or self.sensor_range <= 0
            or not self.vertical_angles
            or min(self.vertical_angles) <= -90
            or max(self.vertical_angles) >= 90
        ):
            raise ValueError("Invalid level-LiDAR geometry (angles are degrees)")


def beam_visibility(task, pose, sensors, minimum_support=1, hazard_support_radius=2.0):
    """Predict support for ground cells from vertical beam rings and known blockers.

    Full azimuth scans and level calibrated mounts are assumed. Unknown occluders,
    dropout and ground roughness remain unmodelled. Predictions are only a score.
    """
    if not sensors or minimum_support < 1 or hazard_support_radius < 0:
        raise ValueError("Invalid sensor support configuration")
    points = task.points
    counts = np.zeros(len(points), dtype=int)
    c, s = np.cos(pose[2]), np.sin(pose[2])
    for sensor in sensors:
        origin = np.asarray(pose[:2]) + [c * sensor.x - s * sensor.y, s * sensor.x + c * sensor.y]
        radius = np.linalg.norm(points - origin, axis=1)
        angles = np.deg2rad(np.asarray(sensor.vertical_angles))
        angles = angles[angles < -1e-6]
        rings = -sensor.height / np.tan(angles)
        rings = rings[rings <= sensor.sensor_range]
        if not len(rings) or not len(points):
            continue
        hit = np.min(np.abs(radius[:, None] - rings[None, :]), axis=1) <= task.spacing / np.sqrt(2)
        hit &= radius <= sensor.sensor_range
        indices = np.flatnonzero(hit)
        if len(indices):
            rays = shapely.linestrings(
                np.stack((np.broadcast_to(origin, (len(indices), 2)), points[indices]), axis=1)
            )
            counts[indices] += ~shapely.intersects(rays, task.occupied)
    required = np.full(len(points), minimum_support)
    if hazard_support_radius and not task.occupied.is_empty:
        near = shapely.distance(shapely.points(points), task.occupied) <= hazard_support_radius
        required[near] = np.maximum(required[near], 2)
    return counts >= required


def beam_gain(task, origin, endpoint, sensors, minimum_support=1, hazard_support_radius=2.0):
    before = beam_visibility(task, origin, sensors, minimum_support, hazard_support_radius)
    after = beam_visibility(task, endpoint, sensors, minimum_support, hazard_support_radius)
    new = after & ~before
    return float(task.weights[new].sum() * task.spacing**2)
