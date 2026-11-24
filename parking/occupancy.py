"""Conservative flat-ground parking occupancy from calibrated LiDAR snapshots.

Only cells with ground returns are observed free. Unobserved cells and cells
with above-ground returns remain blocked. This is a local flat-pad model, not
semantic road segmentation or a guarantee of detecting objects between beams.
"""

from dataclasses import dataclass

import numpy as np
import shapely
from shapely.geometry import GeometryCollection, box
from shapely.ops import unary_union


def ground_cells_geometry(cells, resolution):
    """Merge adjacent cells into row runs without filling any unknown cell."""
    ordered = sorted(cells, key=lambda cell: (cell[1], cell[0]))
    if not ordered:
        return GeometryCollection()
    rectangles = []
    start, row = ordered[0]
    end = start
    for x, y in ordered[1:]:
        if y == row and x == end + 1:
            end = x
        else:
            rectangles.append((start, row, end + 1, row + 1))
            start, end, row = x, x, y
    rectangles.append((start, row, end + 1, row + 1))
    bounds = np.asarray(rectangles, dtype=float) * resolution
    return shapely.union_all(shapely.box(*bounds.T))


@dataclass(frozen=True)
class ParkingOccupancy:
    blocked: object
    free: object
    stamp: float | None
    status: str
    occupied: object = None
    self_footprint: object = None
    sources: tuple = ()

    @property
    def traversable(self):
        """Observed ground plus the space already occupied by the vehicle.

        This exception never expands the vehicle footprint or marks surrounding
        blind cells free. It is conditional on a collision-free initial pose.
        """
        return self.free if self.self_footprint is None else self.free.union(self.self_footprint)

    def permits(self, polygon, clearance=0.15):
        return self.status == "observed" and self.free.covers(polygon.buffer(clearance))


class LidarParkingOccupancy:
    """Stateless occupancy: stale frames never inherit previously clear cells.

    ground_z is a surveyed ground elevation in the localization frame. The
    caller must provide pose at acquisition time, not a later unaligned pose.
    """

    def __init__(
        self,
        resolution=0.25,
        ground_tolerance=0.08,
        max_age=0.3,
        obstacle_padding=0.35,
        surface_link_radius=None,
    ):
        if (
            not np.isfinite([resolution, ground_tolerance, max_age]).all()
            or min(resolution, ground_tolerance, max_age) <= 0
        ):
            raise ValueError("Occupancy parameters must be finite and positive")
        self.resolution = resolution
        self.ground_tolerance = ground_tolerance
        self.max_age = max_age
        if not np.isfinite(obstacle_padding) or obstacle_padding < 0:
            raise ValueError("Obstacle padding must be finite and nonnegative")
        self.obstacle_padding = obstacle_padding
        self.surface_link_radius = (
            obstacle_padding if surface_link_radius is None else surface_link_radius
        )
        if not np.isfinite(self.surface_link_radius) or self.surface_link_radius < obstacle_padding:
            raise ValueError("Surface link radius must be finite and at least obstacle padding")

    def observe(self, lidar, pose, area, now, ground_z):
        empty = GeometryCollection()

        def unknown(status):
            return ParkingOccupancy(area, empty, None, status)

        if area.is_empty or not area.is_valid or area.area <= 0:
            raise ValueError("A valid bounded parking area is required")
        if lidar is None or lidar.points is None or lidar.stamp is None:
            return unknown("missing_lidar")
        if now is None or ground_z is None or not np.isfinite([now, ground_z, lidar.stamp]).all():
            return unknown("invalid_clock_or_ground")
        if not 0 <= now - lidar.stamp <= self.max_age:
            return unknown("stale_or_future_lidar")
        transform = pose.pose_matrix() @ lidar.base_to_sensor
        if not np.isfinite(transform).all():
            return unknown("invalid_pose")
        points = np.asarray(lidar.points)
        if points.ndim != 2 or points.shape[1] < 3 or not len(points):
            return unknown("empty_or_invalid_cloud")
        # Reject malformed frames instead of converting NaNs to observed clear.
        if not np.isfinite(points[:, :3]).all():
            return unknown("nonfinite_cloud")
        points = lidar.to_map(points, pose)
        inside = shapely.intersects_xy(area, points[:, 0], points[:, 1])
        points = points[inside]
        if not len(points):
            return unknown("no_area_returns")
        indices = np.floor(points[:, :2] / self.resolution).astype(np.int64)
        height = points[:, 2] - ground_z
        ground = np.abs(height) <= self.ground_tolerance
        # Drops and any returns above the ground band are hazards, including
        # overhangs. Conservative rejection is preferable to guessed clearance.
        hazards = {tuple(p) for p in indices[~ground]}
        # Deduplicate raster cells in NumPy before constructing Python geometry
        # inputs. Dense LiDAR beams otherwise repeat the same tuple conversion.
        unique_ground = (
            np.unique(np.ascontiguousarray(indices[ground]).view(np.dtype((np.void, 16))).ravel())
            .view(np.int64)
            .reshape(-1, 2)
        )
        clear = {tuple(p) for p in unique_ground} - hazards
        if not clear and not hazards:
            return unknown("no_observed_ground")
        r = self.resolution
        free = ground_cells_geometry(clear, r)
        occupied = GeometryCollection()
        if hazards:
            # Connect nearby surface returns before projecting obstacles. Ground
            # returns underneath a vehicle must not punch holes through its body.
            surfaces = unary_union(
                [
                    box(x * r, y * r, (x + 1) * r, (y + 1) * r).buffer(self.surface_link_radius)
                    for x, y in hazards
                ]
            )
            components = list(surfaces.geoms) if hasattr(surfaces, "geoms") else [surfaces]
            occupied = unary_union(
                [
                    part.convex_hull.buffer(self.obstacle_padding - self.surface_link_radius)
                    for part in components
                ]
            )
            free = free.difference(occupied)
        free = free.intersection(area)
        return ParkingOccupancy(
            area.difference(free), free, float(lidar.stamp), "observed", occupied
        )


class FusedLidarParkingOccupancy(LidarParkingOccupancy):
    """Fuse simultaneous calibrated returns before resolving ground/obstacle conflicts.

    No historical free-space persistence and no privileged obstacle geometry.
    All accepted clouds share the supplied acquisition pose. Older clouds are
    rejected rather than transformed using the wrong pose. The current vehicle
    footprint is an explicit initial-condition exception, not a sensor return.
    """

    multi_sensor = True

    def __init__(
        self,
        *args,
        minimum_support=1,
        hazard_support_radius=0.0,
        self_surface_tolerance=0.03,
        **kwargs,
    ):
        kwargs.setdefault("surface_link_radius", max(0.75, kwargs.get("obstacle_padding", 0.35)))
        super().__init__(*args, **kwargs)
        if (
            isinstance(minimum_support, bool)
            or int(minimum_support) != minimum_support
            or minimum_support < 1
        ):
            raise ValueError("minimum_support must be a positive integer")
        self.minimum_support = int(minimum_support)
        if not np.isfinite(hazard_support_radius) or hazard_support_radius < 0:
            raise ValueError("hazard_support_radius must be finite and nonnegative")
        self.hazard_support_radius = hazard_support_radius
        self.support_histogram = {}
        if not np.isfinite(self_surface_tolerance) or not 0 <= self_surface_tolerance <= 0.1:
            raise ValueError("self_surface_tolerance must be between zero and 0.1 m")
        self.self_surface_tolerance = self_surface_tolerance

    def observe(self, lidar, pose, area, now, ground_z):

        from shapely.geometry import Polygon

        self.support_histogram = {}

        if (
            ground_z is None
            or now is None
            or not np.isfinite(
                [ground_z, now, pose.x, pose.y, pose.z, pose.theta, pose.length, pose.width]
            ).all()
            or min(pose.length, pose.width) <= 0
        ):
            return ParkingOccupancy(area, GeometryCollection(), None, "invalid_clock_or_ground")
        clouds = list(lidar) if isinstance(lidar, (tuple, list)) else [lidar]
        accepted, names, support = [], [], []
        seen = set()
        for index, cloud in enumerate(clouds):
            if cloud is None or cloud.stamp is None or now is None:
                continue
            identity = cloud.sensor_id or cloud.sensor_name or id(cloud)
            if identity in seen:
                continue
            seen.add(identity)
            if not np.isfinite([now, cloud.stamp]).all() or abs(now - cloud.stamp) > 1e-5:
                continue
            if cloud.points is None:
                continue
            points = np.asarray(cloud.points)
            if points.ndim != 2 or points.shape[1] < 3 or not np.isfinite(points).all():
                continue
            local = cloud.to_base(points)
            if not np.isfinite(local).all():
                continue
            # Reject ego-body returns before obstacle projection. No surrounding
            # ground is invented; the exception is applied separately below.
            # A small calibrated tolerance suppresses range-noisy ego surfaces.
            # This is a measurement mask only: the self-footprint exception
            # below remains the exact body, and no free ground is synthesized.
            own = (np.abs(local[:, 0]) <= pose.length / 2 + self.self_surface_tolerance) & (
                np.abs(local[:, 1]) <= pose.width / 2 + self.self_surface_tolerance
            )
            # Preserve actual ground returns under the chassis: their raster
            # cells can straddle the footprint boundary. Mask only ego surfaces.
            ground = np.abs(local[:, 2] + pose.z - ground_z) <= self.ground_tolerance
            own &= ~ground
            accepted.append(local[~own, :3])
            mapped = cloud.to_map(points, pose)
            ground_map = np.abs(mapped[:, 2] - ground_z) <= self.ground_tolerance
            cells = np.unique(
                np.floor(mapped[ground_map, :2] / self.resolution).astype(np.int64), axis=0
            )
            support.append(cells)
            names.append(cloud.sensor_name or str(index))
        if not accepted:
            return ParkingOccupancy(area, GeometryCollection(), None, "missing_synchronized_lidars")
        from avlite.c50_common.c52_world_sensor_datatypes import Lidar

        cells, counts = np.unique(np.vstack(support), axis=0, return_counts=True)
        self.support_histogram = {int(k): int(np.sum(counts == k)) for k in np.unique(counts)}
        combined = Lidar(points=np.vstack(accepted), stamp=now)
        if self.minimum_support > 1 or self.hazard_support_radius > 0:
            # Ground-cell support counts independent devices, not beam count.
            mapped = combined.to_map(combined.points, pose)
            ground = np.abs(mapped[:, 2] - ground_z) <= self.ground_tolerance
            required = np.full(len(cells), self.minimum_support)
            if self.hazard_support_radius > 0 and np.any(~ground):
                from scipy.spatial import cKDTree

                distances, _ = cKDTree(mapped[~ground, :2]).query((cells + 0.5) * self.resolution)
                near = distances <= self.hazard_support_radius + self.resolution / np.sqrt(2)
                required[near] = np.maximum(required[near], 2)
            supported = (
                np.ascontiguousarray(cells[counts >= required])
                .view(np.dtype((np.void, 16)))
                .ravel()
            )
            indices = np.floor(mapped[:, :2] / self.resolution).astype(np.int64)
            keep = ~ground | np.isin(
                np.ascontiguousarray(indices).view(np.dtype((np.void, 16))).ravel(), supported
            )
            combined.points = combined.points[keep]
        result = super().observe(combined, pose, area, now, ground_z)
        if result.status != "observed":
            return result
        own = Polygon(pose.get_bb_corners()).intersection(area)
        # Occupied evidence always takes precedence, including around the body.
        own = own.difference(result.occupied)
        return ParkingOccupancy(
            area.difference(result.free.union(own)),
            result.free,
            result.stamp,
            result.status,
            result.occupied,
            own,
            tuple(names),
        )
