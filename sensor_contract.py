"""Validate the canonical RGB/LiDAR input at a simulator-independent boundary."""

import numpy as np


def _rigid_mount(value):
    value = np.asarray(value)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        return False
    rotation = value[:3, :3]
    return (
        np.allclose(value[3], [0, 0, 0, 1], atol=1e-5)
        and np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4)
        and np.isclose(np.linalg.det(rotation), 1.0, atol=1e-4)
    )


def validate_regulatory_frame(frame):
    """Return a diagnostic on malformed readings/calibration, otherwise None.

    Missing/repeated/stale readings are checked separately by the detector.
    Sensor mounts map device coordinates into the ego body frame; the camera
    uses optical axes, RGB order and rectified pinhole intrinsics. LiDAR points
    remain in the physical device frame. No actor IDs or simulator SDK needed.
    """
    camera, lidar = frame.camera, frame.lidar
    rgb = camera.rgb
    if (
        not isinstance(rgb, np.ndarray)
        or rgb.dtype != np.uint8
        or rgb.shape != (camera.height, camera.width, 3)
    ):
        return "invalid_image"
    points = lidar.points
    if (
        not isinstance(points, np.ndarray)
        or points.ndim != 2
        or points.shape[1] != 4
        or points.dtype != np.float32
    ):
        return "invalid_lidar"
    intrinsic = np.asarray(camera.intrinsic)
    if (
        intrinsic.shape != (3, 3)
        or not np.isfinite(intrinsic).all()
        or intrinsic[0, 0] <= 0
        or intrinsic[1, 1] <= 0
        or not np.allclose(intrinsic[2], [0, 0, 1])
        or abs(np.linalg.det(intrinsic)) < 1e-9
        or not _rigid_mount(camera.base_to_sensor)
        or not _rigid_mount(lidar.base_to_sensor)
    ):
        return "invalid_calibration"
    return None
