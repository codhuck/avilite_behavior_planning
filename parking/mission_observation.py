"""Parking-area observations shared by mission selection and parking execution."""

import math

from shapely.geometry import GeometryCollection

from .occupancy import ParkingOccupancy


class MissionLidarObservation:
    """Consume the executor's synchronized snapshot without fetching or ticking a world.

    Frame and cloud times must share a clock; the first frame establishes its
    offset from execution time. Later frames retain that offset. Repeated
    frames are never reprojected with a later pose or given a new acquisition
    time. The bridge/localizer must supply the pose for the current snapshot.
    """

    max_age = 0.3

    def __init__(self, provider, area, ground_z):
        self.provider, self.area, self.ground_z = provider, area, ground_z
        self._offset = None
        self._last_frame = None
        self._last_now = None
        self.stamp = None
        self.occupancy = ParkingOccupancy(area, GeometryCollection(), None, "missing_lidar")

    def _unknown(self, reason):
        self.stamp = None
        self.occupancy = ParkingOccupancy(self.area, GeometryCollection(), None, reason)

    def reason(self, now):
        if self.occupancy.status != "observed":
            return self.occupancy.status
        if self.stamp is None or not math.isfinite(now):
            return "invalid_clock"
        if now < self.stamp - 1e-6:
            return "future_lidar"
        if now - self.stamp > self.max_age:
            return "stale_lidar"
        return "observed"

    def observe(self, sensors, pose, now):
        frame_time = getattr(sensors, "stamp", None)
        if (
            frame_time is None
            or not math.isfinite(frame_time)
            or not math.isfinite(now)
            or (self._last_now is not None and now < self._last_now)
        ):
            self._unknown("invalid_or_reversed_clock")
            return
        self._last_now = now
        if self._last_frame is not None and frame_time <= self._last_frame:
            if frame_time < self._last_frame:
                self._unknown("out_of_order_lidar")
            return
        if self._offset is None:
            self._offset = now - frame_time
        stamp = frame_time + self._offset
        if stamp > now + 1e-6 or now - stamp > self.max_age:
            self._unknown("stale_or_future_lidar")
            return
        self._last_frame = frame_time
        cloud = getattr(sensors, "lidar", None)
        if getattr(self.provider, "multi_sensor", False):
            named = getattr(sensors, "lidars", {})
            if named:
                cloud = tuple(named.values())
        elif cloud is not None and (
            cloud.stamp is None
            or not math.isfinite(cloud.stamp)
            or abs(cloud.stamp - frame_time) > 1e-5
        ):
            # There is no pose history here for deskewing an older cloud.
            self._unknown("unsynchronized_lidar")
            return
        self.occupancy = self.provider.observe(cloud, pose, self.area, frame_time, self.ground_z)
        # Subtracting a large sensor-clock offset can leave a tiny positive
        # rounding error; the planner deliberately rejects future observations.
        self.stamp = min(now, stamp) if self.occupancy.status == "observed" else None

    def classify(self, bay, now):
        reason = self.reason(now)
        if reason != "observed":
            return reason
        occupied = self.occupancy.occupied
        if occupied is not None and occupied.intersects(bay.buffer(0.15)):
            return "occupied"
        return "free" if self.occupancy.permits(bay) else "unknown"

    def apply(self, planner, now):
        if self.reason(now) != "observed":
            planner.update_obstacles(None, now)
            return
        blocked = self.occupancy.blocked
        planner.update_obstacles([] if blocked.is_empty else [blocked], self.stamp)
