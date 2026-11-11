"""RGB/LiDAR regulatory perception. No CARLA import or actor-state access.

Static map geometry identifies crossings and associates signal heads. Dynamic
states come exclusively from RGB and non-semantic LiDAR. Ego localization is
supplied by the stack (the CARLA demo still uses simulator ego localization).
"""

import json
import logging
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from avlite.c10_perception.c11_perception_model import AgentState
from avlite.c10_perception.c12_perception_strategy import DetectionStrategy
from avlite.c50_common.c51_capabilities import WorldCapability
from shapely import contains_xy
from shapely.geometry import LineString, Point, Polygon

from .observations import RegulatoryObservation
from .regulatory import RegulatoryKind, extract_regulatory_features
from .settings import PluginSettings
from .traffic_light_vision import (
    SignalPerception,
    resolve_model,
)

log = logging.getLogger(__name__)
ROOT = Path(__file__).resolve().parent


@dataclass
class SensorObservation(RegulatoryObservation):
    source: str = "rgb_lidar"
    crossings: dict = field(
        default_factory=dict
    )  # True occupied, False observed clear, None unknown
    detections: list = field(default_factory=list)
    diagnostics: dict = field(default_factory=dict)
    sensor_stamp: float | None = None


@dataclass(frozen=True)
class Detection:
    label: str
    score: float
    box: tuple


class YoloOnnxDetector:
    """YOLOv8 COCO ONNX, OpenCV CPU runtime, bounded thread count."""

    labels = {
        0: "person",
        1: "bicycle",
        2: "car",
        3: "motorcycle",
        5: "bus",
        7: "truck",
        9: "traffic light",
        11: "stop sign",
    }

    def __init__(self, model=None, confidence=0.25):
        import cv2

        cv2.setNumThreads(2)
        path = Path(model) if model else resolve_model("yolo", download=True)
        if not path.is_file():
            raise FileNotFoundError(
                f"Sensor detector model missing: {path}. See SENSOR_PERCEPTION.md"
            )
        self.net = cv2.dnn.readNetFromONNX(str(path))
        self.confidence = confidence

    def __call__(self, rgb):
        import cv2

        h, w = rgb.shape[:2]
        scale = min(640 / w, 640 / h)
        nw, nh = round(w * scale), round(h * scale)
        left, top = (640 - nw) // 2, (640 - nh) // 2
        image = np.full((640, 640, 3), 114, np.uint8)
        image[top : top + nh, left : left + nw] = cv2.resize(rgb, (nw, nh))
        self.net.setInput(cv2.dnn.blobFromImage(image, 1 / 255.0, (640, 640), swapRB=False))
        rows = self.net.forward().squeeze().T
        candidates = []
        for row in rows:
            cls = int(np.argmax(row[4:]))
            score = float(row[4 + cls])
            if cls not in self.labels or score < self.confidence:
                continue
            cx, cy, bw, bh = row[:4]
            x1, y1 = (cx - bw / 2 - left) / scale, (cy - bh / 2 - top) / scale
            x2, y2 = (cx + bw / 2 - left) / scale, (cy + bh / 2 - top) / scale
            box = (
                float(np.clip(x1, 0, w)),
                float(np.clip(y1, 0, h)),
                float(np.clip(x2, 0, w)),
                float(np.clip(y2, 0, h)),
            )
            candidates.append((cls, score, box))
        result = []
        for cls in self.labels:
            group = [c for c in candidates if c[0] == cls]
            if not group:
                continue
            boxes = [[b[0], b[1], b[2] - b[0], b[3] - b[1]] for _, _, b in group]
            for i in np.asarray(
                cv2.dnn.NMSBoxes(boxes, [c[1] for c in group], self.confidence, 0.45)
            ).flatten():
                _, score, box = group[int(i)]
                result.append(Detection(self.labels[cls], score, box))
        return result


def project(points, camera, pose):
    points = np.asarray(points, dtype=float)
    transform = np.linalg.inv(pose @ camera.base_to_sensor)
    optical = points @ transform[:3, :3].T + transform[:3, 3]
    uvw = optical @ camera.intrinsic.T
    uv = uvw[:, :2] / np.maximum(optical[:, 2:3], 1e-6)
    return uv, optical[:, 2]


def ground_footpoint(box, camera, pose, ground_z):
    """Fallback for a visible person's feet when sparse LiDAR misses the body.

    Planar-road assumption, reported as such in diagnostics. Cropped feet and
    near-horizontal rays remain unknown; no guessed depth from box height.
    """
    x1, y1, x2, y2 = box
    if y2 >= camera.height - 2:
        return None
    transform = pose @ camera.base_to_sensor
    ray = transform[:3, :3] @ np.linalg.solve(camera.intrinsic, np.array([(x1 + x2) / 2, y2, 1.0]))
    origin = transform[:3, 3]
    if ray[2] >= -0.02:
        return None
    scale = (ground_z - origin[2]) / ray[2]
    p = origin + scale * ray
    if scale <= 0 or np.linalg.norm(p[:2] - pose[:2, 3]) > 50:
        return None
    return tuple(p[:2])


def lamp_color(rgb):
    """Recognise the illuminated aperture of a vertical three-aspect head.

    CARLA's exposure can desaturate lit lenses. Compare aperture brightness
    as well as colour, excluding the yellow casing and visor. This classifier
    is deliberately limited to the surveyed vertical heads, not arrow lights.
    """

    if rgb.size == 0:
        return "Unknown"
    h, w = rgb.shape[:2]
    if h < 12 or w < 4 or not 1.5 < h / w < 5:
        return "Unknown"
    scores = []
    for low, high in ((0.15, 0.35), (0.42, 0.59), (0.65, 0.83)):
        patch = rgb[
            int(h * low) : max(int(h * low) + 1, int(h * high)),
            int(w * 0.4) : max(int(w * 0.4) + 1, int(w * 0.95)),
        ].astype(float)
        scores.append(float(np.percentile(patch.min(axis=2), 80)))
    best = int(np.argmax(scores))
    ordered = sorted(scores)
    threshold, gap = (100.0, 25.0) if best == 0 else (170.0, 45.0)
    if scores[best] < threshold or ordered[-1] - ordered[-2] < gap:
        return "Unknown"
    # Flat bright boards have no dark inactive apertures.
    if np.percentile(rgb[int(h * 0.15) : int(h * 0.83)].min(axis=2), 20) > 145:
        return "Unknown"
    return ("Red", "Yellow", "Green")[best]


class SensorRegulatoryDetector(DetectionStrategy):
    world_requirements = frozenset({WorldCapability.CAMERA_RGB, WorldCapability.LIDAR_3D})

    def __init__(self, detector=None, geometry=None, signal_classifier=None, signal_backend=None):
        self.detector = detector if detector is not None else YoloOnnxDetector()
        self.signal_backend = signal_backend or PluginSettings.sensor_signal_backend
        if self.signal_backend not in ("carla_aperture", "autoware", "trained_rgb"):
            raise ValueError("Unsupported signal backend")
        self.signal_classifier = signal_classifier
        self.signal_service = None
        if self.signal_backend != "carla_aperture":
            self.signal_service = SignalPerception(
                self.signal_backend,
                model=PluginSettings.sensor_signal_model_path or None,
                classifier=signal_classifier,
                provider=PluginSettings.sensor_signal_provider or None,
            )
            self.signal_classifier = self.signal_service.classifier
        path = Path(
            PluginSettings.sensor_signal_geometry or ROOT / "data/Town10HD_Opt_signals.json"
        )
        self.geometry = (
            geometry
            if geometry is not None
            else (json.loads(path.read_text()) if path.is_file() else {})
        )
        self.reset()

    def reset(self):
        self.last_stamp = None
        self.last_observation = None
        self.last_map = None
        self.features = []
        self.green_streak = {}
        self.clear_since = {}
        self.geometry_matches = False
        self._last_now = None
        self.clock_anchor = None

    def detect(self, perception_model=None, sensors=None, **kwargs):
        pm = perception_model
        if pm is None:
            raise ValueError("SensorRegulatoryDetector requires a perception model")
        now = float(getattr(pm, "simulation_time", 0.0))
        if self._last_now is not None and now < self._last_now:
            self.reset()
        self._last_now = now
        pm.agent_vehicles = []
        pm.static_obstacles = []  # no inherited GT objects
        obs = SensorObservation(now)
        pm.regulatory_observation = obs
        if pm.map is not self.last_map:
            self.features = extract_regulatory_features(pm.map)
            self.last_map = pm.map
            self.green_streak.clear()
            self.clear_since.clear()
            definitions = {
                f.id: np.array([f.x, f.y])
                for f in self.features
                if f.kind == RegulatoryKind.TRAFFIC_LIGHT and not f.is_reference
            }
            survey = self.geometry.get("map_signals", {})
            self.geometry_matches = (
                bool(survey)
                and set(survey) == set(definitions)
                and all(np.linalg.norm(definitions[k] - v) < 0.1 for k, v in survey.items())
            )
        camera = getattr(sensors, "camera", None)
        lidar = getattr(sensors, "lidar", None)
        if (
            camera is None
            or lidar is None
            or camera.rgb is None
            or lidar.points is None
            or camera.stamp is None
            or lidar.stamp is None
            or not np.isfinite(camera.stamp)
            or not np.isfinite(lidar.stamp)
            or abs(camera.stamp - lidar.stamp) > 1e-4
        ):
            self.green_streak.clear()
            self.clear_since.clear()
            obs.diagnostics = {"status": "missing_or_unsynchronised_sensors"}
            return pm
        stamp = float(camera.stamp)
        if self.last_stamp is not None and stamp <= self.last_stamp:
            # Never relabel an old image as a fresh observation or accumulate confirmations.
            self.green_streak.clear()
            self.clear_since.clear()
            obs.diagnostics = {"status": "repeated_or_out_of_order_frame"}
            return pm
        self.last_stamp = stamp
        obs.sensor_stamp = stamp
        if self.clock_anchor is None:
            self.clock_anchor = (stamp, now)
        capture_time = self.clock_anchor[1] + stamp - self.clock_anchor[0]
        if now - capture_time > PluginSettings.observation_max_age or capture_time - now > 0.005:
            self.green_streak.clear()
            self.clear_since.clear()
            obs.diagnostics = {"status": "stale_or_future_frame", "capture_time": capture_time}
            return pm
        obs.time = min(
            now, capture_time
        )  # CARLA float32 tick rounding must not create future observations
        rgb = camera.rgb
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape != (camera.height, camera.width, 3):
            self.green_streak.clear()
            self.clear_since.clear()
            obs.diagnostics = {"status": "invalid_image"}
            return pm
        if np.std(rgb) < 2 or np.percentile(rgb, 95) < 15:
            self.green_streak.clear()
            self.clear_since.clear()
            obs.diagnostics = {"status": "unusable_image"}
            return pm
        pose = pm.ego_vehicle.pose_matrix()  # stack localization, never sensors.base_to_map
        cloud = lidar.to_base(lidar.points)
        finite = np.isfinite(cloud[:, :3]).all(axis=1)
        cloud = cloud[finite]
        if len(cloud) < 100:
            self.green_streak.clear()
            self.clear_since.clear()
            obs.diagnostics = {"status": "insufficient_lidar"}
            return pm
        world = cloud[:, :3] @ pose[:3, :3].T + pose[:3, 3]
        uv, depth = project(world, camera, pose)
        # Ground height from near low LiDAR returns, not a fixed vehicle-origin offset.
        near = np.linalg.norm(cloud[:, :2], axis=1) < 15
        low = near & (cloud[:, 2] > -1.5) & (cloud[:, 2] < 0.25)
        ground_z = float(np.median(world[low, 2])) if np.count_nonzero(low) >= 20 else None
        began = time.monotonic()
        try:
            detections = self.detector(rgb)
        except Exception as exc:
            self.green_streak.clear()
            self.clear_since.clear()
            log.exception("Sensor object detector failed; observations unavailable")
            obs.diagnostics = {"status": "detector_error", "error": str(exc)}
            return pm
        unlocalized_person = False
        for index, d in enumerate(detections):
            x1, y1, x2, y2 = d.box
            item = dict(label=d.label, confidence=d.score, box=list(d.box))
            if d.label in ("person", "car", "bus", "truck", "bicycle", "motorcycle"):
                # Central bbox points avoid most background/road returns.
                hit = (
                    (uv[:, 0] > x1 + 0.15 * (x2 - x1))
                    & (uv[:, 0] < x2 - 0.15 * (x2 - x1))
                    & (uv[:, 1] > y1 + 0.15 * (y2 - y1))
                    & (uv[:, 1] < y2 - 0.1 * (y2 - y1))
                    & (depth > 0)
                    & (cloud[:, 2] > 0.2)
                    & (cloud[:, 2] < 3)
                )
                pts = world[hit]
                if len(pts) >= 3:
                    ranges = depth[hit]
                    nearest = np.percentile(ranges, 20)
                    pts = pts[np.abs(ranges - nearest) < 1.5]
                    if len(pts) < 3:
                        if d.label == "person":
                            xy = (
                                ground_footpoint(d.box, camera, pose, ground_z)
                                if ground_z is not None
                                else None
                            )
                            if xy is None:
                                unlocalized_person = True
                            else:
                                obs.pedestrians.append(xy)
                                item["position"] = list(xy)
                                item["position_source"] = "camera_ground_plane"
                        obs.detections.append(item)
                        continue
                    xy = np.median(pts[:, :2], axis=0)
                    item["position"] = xy.tolist()
                    item["position_source"] = "lidar"
                    if d.label == "person":
                        obs.pedestrians.append(tuple(xy))
                    small = d.label in ("person", "bicycle", "motorcycle")
                    pm.agent_vehicles.append(
                        AgentState(
                            x=float(xy[0]),
                            y=float(xy[1]),
                            length=0.8 if small else 4.5,
                            width=0.8 if small else 2.0,
                            velocity=0.0,
                            agent_id=10000 + index,
                        )
                    )
                elif d.label == "person":
                    xy = (
                        ground_footpoint(d.box, camera, pose, ground_z)
                        if ground_z is not None
                        else None
                    )
                    if xy is None:
                        unlocalized_person = True
                    else:
                        obs.pedestrians.append(xy)
                        item["position"] = list(xy)
                        item["position_source"] = "camera_ground_plane"
            obs.detections.append(item)
        # Static geometry provides only identity and a search ROI; colour is from RGB.
        votes = {}
        rois = []
        known_ids = {f.id for f in self.features if f.kind == RegulatoryKind.TRAFFIC_LIGHT}
        for head in self.geometry.get("heads", []) if self.geometry_matches else []:
            key = head["signal_id"]
            if key not in known_ids:
                continue
            pix, z = project(head["corners"], camera, pose)
            if np.min(z) < 1 or np.max(z) > 65:
                continue
            a = pix.min(axis=0)
            b = pix.max(axis=0)
            x1, y1 = np.floor(a).astype(int)
            x2, y2 = np.ceil(b).astype(int)
            if x1 < 1 or y1 < 1 or x2 >= camera.width - 1 or y2 >= camera.height - 1:
                continue
            if y2 - y1 < 7 or x2 - x1 < 3:
                continue
            # A projected box alone is not evidence: require a visible lamp pattern.
            color = (
                lamp_color(rgb[y1:y2, x1:x2])
                if self.signal_backend == "carla_aperture"
                else "Unknown"
            )
            votes.setdefault(key, []).append(color)
            rois.append(dict(signal_id=key, box=[int(x1), int(y1), int(x2), int(y2)], color=color))
        if self.signal_backend in ("autoware", "trained_rgb"):
            try:
                associated, observed_signals = self.signal_service.recognize(rgb, detections, rois)
            except Exception:
                log.exception("Traffic light classifier failed")
                self.green_streak.clear()
                self.clear_since.clear()
                obs.diagnostics = {"status": "signal_classifier_error"}
                return pm
            obs.detections.extend(
                dict(
                    kind="signal_state",
                    proposal_source=(
                        "hd_map_projection" if self.signal_backend == "trained_rgb" else "yolo"
                    ),
                    **item,
                )
                for item in observed_signals
            )
            votes = {key: [color] for key, color in associated.items()}
        for key, colors in votes.items():
            valid = set(colors) - {"Unknown"}
            color = next(iter(valid)) if len(valid) == 1 else "Unknown"
            # A stop indication is immediate; green needs consecutive independent frames.
            self.green_streak[key] = self.green_streak.get(key, 0) + 1 if color == "Green" else 0
            obs.lights[key] = (
                color if color != "Green" or self.green_streak[key] >= 3 else "Unknown"
            )
        for key in set(self.green_streak) - set(votes):
            self.green_streak[key] = 0
        # Non-semantic LiDAR occupancy also catches missed people/vehicles.
        # Free-space evidence uses low returns; a camera blind spot is not an empty crossing.
        ground = (cloud[:, 2] < 0.25) & (cloud[:, 2] > -1.5)
        angles = np.arctan2(cloud[:, 1], cloud[:, 0])
        ranges = np.linalg.norm(cloud[:, :2], axis=1)
        bins = ((angles + np.pi) / (2 * np.pi) * 720).astype(int) % 720
        reach = np.zeros(720)
        np.maximum.at(reach, bins[ground], ranges[ground])
        high = (cloud[:, 2] > 0.35) & (cloud[:, 2] < 2.5)
        crossing_diagnostics = {}
        for f in self.features:
            if f.kind != RegulatoryKind.CROSSWALK:
                continue
            polygon = Polygon(f.polygon)
            if polygon.is_empty or not polygon.is_valid:
                continue
            if polygon.distance(Point(pm.ego_vehicle.x, pm.ego_vehicle.y)) > 50:
                continue
            occupied = any(
                polygon.buffer(PluginSettings.pedestrian_margin).covers(Point(p))
                for p in obs.pedestrians
            )
            # Evaluate exact polygon for unclassified returns (do not include sidewalk poles).
            count = int(np.count_nonzero(contains_xy(polygon, world[high, 0], world[high, 1])))
            occupied = occupied or count >= 3
            route = getattr(pm, "regulatory_route", None)
            visible_area = (
                polygon.intersection(LineString(route).buffer(6.0))
                if route and len(route) > 1
                else polygon
            )
            if visible_area.is_empty:
                continue
            pieces = list(visible_area.geoms) if hasattr(visible_area, "geoms") else [visible_area]
            visible_area = max(
                (p for p in pieces if p.geom_type == "Polygon"), key=lambda p: p.area, default=None
            )
            if visible_area is None:
                continue
            perimeter = visible_area.exterior
            samples = [
                perimeter.interpolate(t).coords[0]
                for t in np.linspace(0, perimeter.length, 24, endpoint=False)
            ]
            samples.append(visible_area.centroid.coords[0])
            xy = np.asarray(samples)
            rel = (xy - pose[:2, 3]) @ pose[:2, :2]
            aa = np.arctan2(rel[:, 1], rel[:, 0])
            rr = np.linalg.norm(rel, axis=1)
            bb = ((aa + np.pi) / (2 * np.pi) * 720).astype(int) % 720
            # +/- 0.5 degrees accounts for discrete beam sampling, not a broad blind cone.
            coverage = np.maximum.reduce([reach[(bb + d) % 720] for d in (-1, 0, 1)]) >= rr + 0.3
            # A region occupied by our own known body is not an unseen pedestrian area.
            inside_ego = (np.abs(rel[:, 0]) < pm.ego_vehicle.length / 2) & (
                np.abs(rel[:, 1]) < pm.ego_vehicle.width / 2
            )
            coverage |= inside_ego
            visible = bool(np.all(coverage))
            crossing_diagnostics[f.id] = dict(
                coverage=float(np.mean(coverage)), foreground_count=count
            )
            if occupied:
                obs.crossings[f.id] = True
                self.clear_since.pop(f.id, None)
            elif visible and not unlocalized_person:
                since = self.clear_since.setdefault(f.id, now)
                obs.crossings[f.id] = False if now - since >= 0.5 else None
            else:
                obs.crossings[f.id] = None
                self.clear_since.pop(f.id, None)
        obs.diagnostics = dict(
            status="ok",
            inference_ms=round((time.monotonic() - began) * 1000, 1),
            signal_backend=self.signal_backend,
            geometry_matches=self.geometry_matches,
            signal_rois=rois,
            unlocalized_person=unlocalized_person,
            crossings=crossing_diagnostics,
        )
        self.last_observation = obs
        return pm
