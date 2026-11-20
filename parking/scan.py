"""Finite-ray ground support prediction for ranking, never observed occupancy.

The model assumes level full-azimuth LiDARs and a surveyed flat ground plane.
Phase averaging expresses sampling sensitivity, not calibrated probability.
"""

from dataclasses import dataclass

import numpy as np
import shapely


@dataclass(frozen=True)
class ScanUtility:
    expected_area: float
    phase_min: float
    phase_max: float


def _pose(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError("Expected a finite SE(2) pose")
    return value


def _blocked_by_body(origin, ends, length, width, bottom, top):
    """Segment/slab intersection with a calibrated conservative ego-body proxy."""
    low = np.array([-length / 2, -width / 2, bottom])
    high = np.array([length / 2, width / 2, top])
    directions = ends - origin
    enter = np.zeros(len(ends))
    leave = np.ones(len(ends))
    for axis in range(3):
        parallel = np.abs(directions[:, axis]) < 1e-12
        outside = (origin[axis] < low[axis]) or (origin[axis] > high[axis])
        leave[parallel & outside] = -1
        moving = ~parallel
        a = (low[axis] - origin[axis]) / directions[moving, axis]
        b = (high[axis] - origin[axis]) / directions[moving, axis]
        enter[moving] = np.maximum(enter[moving], np.minimum(a, b))
        leave[moving] = np.minimum(leave[moving], np.maximum(a, b))
    return (enter <= leave) & (leave > 1e-8)


class ScanVisibility:
    """Rasterize finite ground rays and count each configured device once per cell.

    Only cells missing in the measured task region contribute. A fresh stationary
    scan is predicted explicitly; gain is NOT obtained by subtracting boolean
    visibility masks. Known 2D occupied geometry masks rays conservatively.
    """

    def __init__(
        self,
        task,
        sensors,
        *,
        resolution=0.25,
        azimuth_samples=250,
        phases=4,
        minimum_support=1,
        hazard_support_radius=2.0,
        body_length=3.7,
        body_width=1.8,
        body_bottom=0.15,
        body_top=1.6,
    ):
        if not sensors or len(set(sensors)) != len(sensors):
            raise ValueError("Expected distinct calibrated sensor geometries")
        for value in (azimuth_samples, phases, minimum_support):
            if (
                isinstance(value, bool)
                or not np.isfinite(value)
                or int(value) != value
                or value < 1
            ):
                raise ValueError("Sampling and support counts must be positive integers")
        values = [resolution, hazard_support_radius, body_length, body_width, body_bottom, body_top]
        if (
            not np.isfinite(values).all()
            or min(resolution, body_length, body_width) <= 0
            or hazard_support_radius < 0
            or not 0 <= body_bottom < body_top
        ):
            raise ValueError("Invalid raster or body geometry")
        self.task = task
        self.resolution = resolution
        self.phases = int(phases)
        self.cells = np.floor(task.points / resolution).astype(np.int64)
        self.keys = self.cells.view(np.dtype((np.void, 16))).ravel()
        self.required = np.full(len(self.cells), int(minimum_support))
        if hazard_support_radius and len(self.cells) and not task.occupied.is_empty:
            near = (
                shapely.distance(shapely.points(task.points), task.occupied)
                <= hazard_support_radius
            )
            self.required[near] = np.maximum(self.required[near], 2)
        self.rays = []
        for phase in range(self.phases):
            devices = []
            azimuth = (
                2 * np.pi * (np.arange(azimuth_samples) + phase / self.phases) / azimuth_samples
            )
            for sensor in sensors:
                origin = np.array([sensor.x, sensor.y, sensor.height])
                angles = np.deg2rad(sensor.vertical_angles)
                angles = angles[angles < -1e-6]
                radius = -sensor.height / np.tan(angles)
                radius = radius[np.hypot(radius, sensor.height) <= sensor.sensor_range]
                xy = (
                    radius[:, None, None] * np.column_stack((np.cos(azimuth), np.sin(azimuth)))
                ).reshape(-1, 2)
                ends = np.column_stack((xy + origin[:2], np.zeros(len(xy))))
                blocked = _blocked_by_body(
                    origin, ends, body_length, body_width, body_bottom, body_top
                )
                devices.append((origin[:2], ends[~blocked, :2]))
            self.rays.append(devices)
        self._cache = {}

    def evaluate(self, pose):
        pose = _pose(pose)
        key = tuple(pose)
        if key in self._cache:
            return self._cache[key]
        c, s = np.cos(pose[2]), np.sin(pose[2])
        rotation = np.array([[c, s], [-s, c]])
        scores = []
        # Restrict expensive occlusion tests to rays landing in task raster cells.
        wanted = self.keys
        for devices in self.rays:
            device_cells = []
            for local_origin, local_ends in devices:
                origin = local_origin @ rotation + pose[:2]
                ends = local_ends @ rotation + pose[:2]
                cells = np.floor(ends / self.resolution).astype(np.int64)
                mask = np.isin(cells.view(np.dtype((np.void, 16))).ravel(), wanted)
                ends, cells = ends[mask], cells[mask]
                if len(ends) and not self.task.occupied.is_empty:
                    lines = shapely.linestrings(
                        np.stack((np.broadcast_to(origin, ends.shape), ends), axis=1)
                    )
                    cells = cells[~shapely.intersects(lines, self.task.occupied)]
                device_cells.append(
                    np.unique(np.ascontiguousarray(cells).view(np.dtype((np.void, 16))).ravel())
                )
            keys, counts = np.unique(np.concatenate(device_cells), return_counts=True)
            supported = keys[counts >= 1]
            # Task cells can share an occupancy cell; retain each task weight.
            observed = np.zeros(len(self.keys), dtype=bool)
            for required in np.unique(self.required):
                supported = keys[counts >= required]
                observed |= (self.required == required) & np.isin(self.keys, supported)
            scores.append(float(self.task.weights[observed].sum() * self.task.spacing**2))
        result = ScanUtility(float(np.mean(scores)), float(min(scores)), float(max(scores)))
        self._cache[key] = result
        return result

    def advantage(self, origin, endpoint, length):
        """Expected acquired task area per motion cost minus fresh stationary scan."""
        if not np.isfinite(length) or length < 0:
            raise ValueError("Expected a finite nonnegative motion length")
        return (
            self.evaluate(endpoint).expected_area / (1 + length)
            - self.evaluate(origin).expected_area
        )
