"""Camera-only signal recognition; no simulator, map, or localization dependency.

Autoware v4 MobileNetV2 takes RGB, top-left letterbox 224x224 and ImageNet
normalization in byte units. Its ONNX output already contains probabilities.
Static-map association is a separate operation; an unassociated green cannot
authorize movement. This is an experimental adapter, not a validated driving ODD.
"""

from dataclasses import asdict, dataclass

import numpy as np

from .assets import resolve_model

LABELS = (
    "green",
    "left,red",
    "left,red,straight",
    "red",
    "red,right",
    "red,straight",
    "unknown",
    "yellow",
    "red,up_left",
    "red,right,straight",
    "red,up_right",
)


@dataclass(frozen=True)
class SignalState:
    color: str = "Unknown"
    label: str = "unknown"
    confidence: float = 0.0
    reason: str = "unobserved"


class AutowareSignalClassifier:
    def __init__(self, model=None, confidence=0.85, margin=0.25):
        import cv2

        cv2.setNumThreads(2)
        model = model or resolve_model("autoware", download=True)
        self.net = cv2.dnn.readNetFromONNX(str(model))
        self.confidence, self.margin = confidence, margin

    @staticmethod
    def preprocess(rgb):
        import cv2

        h, w = rgb.shape[:2]
        scale = min(224 / w, 224 / h)
        resized = cv2.resize(rgb, (max(1, int(w * scale)), max(1, int(h * scale))))
        image = np.zeros((224, 224, 3), np.float32)
        image[: resized.shape[0], : resized.shape[1]] = resized
        image = (image - np.array([123.675, 116.28, 103.53], np.float32)) / np.array(
            [58.395, 57.12, 57.375], np.float32
        )
        return np.ascontiguousarray(image.transpose(2, 0, 1)[None])

    def decode(self, probabilities):
        p = np.asarray(probabilities).reshape(-1)
        if (
            p.size != len(LABELS)
            or not np.isfinite(p).all()
            or np.any(p < 0)
            or np.any(p > 1)
            or abs(float(p.sum()) - 1) > 0.01
        ):
            return SignalState(reason="invalid_model_output")
        i = int(p.argmax())
        score = float(p[i])
        label = LABELS[i]
        if score < self.confidence or score - float(np.partition(p, -2)[-2]) < self.margin:
            return SignalState(label=label, confidence=score, reason="low_confidence")
        # Arrow interpretation requires maneuver/phase semantics, not just hue.
        if label not in ("red", "yellow", "green"):
            return SignalState(label=label, confidence=score, reason="unsupported_or_unknown")
        return SignalState(label.capitalize(), label, score, "classified")

    def __call__(self, rgb):
        if (
            rgb.ndim != 3
            or rgb.shape[2] != 3
            or rgb.dtype != np.uint8
            or min(rgb.shape[:2]) < 8
            or max(rgb.shape[:2]) < 16
        ):
            return SignalState(reason="invalid_or_small_crop")
        if rgb.std() < 3 or np.percentile(rgb, 95) < 15:
            return SignalState(reason="unusable_exposure")
        self.net.setInput(self.preprocess(rgb))
        return self.decode(self.net.forward())


def recognize_signals(rgb, detections, classifier):
    """Return image observations without assigning road/signal identities."""
    h, w = rgb.shape[:2]
    observations = []
    for detection in detections:
        if detection.label != "traffic light":
            continue
        box = np.asarray(detection.box, dtype=float)
        if box.shape != (4,) or not np.isfinite(box).all():
            continue
        x1, y1 = np.floor(box[:2]).astype(int)
        x2, y2 = np.ceil(box[2:]).astype(int)
        if x1 < 1 or y1 < 1 or x2 >= w - 1 or y2 >= h - 1 or x2 <= x1 or y2 <= y1:
            state = SignalState(reason="clipped_crop")
        else:
            state = classifier(rgb[y1:y2, x1:x2])
        observations.append(
            dict(box=box.tolist(), detector_confidence=float(detection.score), **asdict(state))
        )
    return observations


def associate_signals(observations, projected_heads, min_overlap=0.5):
    """Conservative image-box match to separately surveyed signal identities.

    Refuse any detection overlapping heads of multiple signal IDs. Never select
    the nearest green or infer identity from colour. Unknown observations remain
    unknown, and conflicting heads do not yield permission.
    """
    votes = {}
    for observation in observations:
        a = np.asarray(observation["box"], float)
        matches = set()
        for head in projected_heads:
            b = np.asarray(head["box"], float)
            intersection = np.maximum(0, np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2])).prod()
            area = min(np.maximum(0, a[2:] - a[:2]).prod(), np.maximum(0, b[2:] - b[:2]).prod())
            if area > 0 and intersection / area >= min_overlap:
                matches.add(head["signal_id"])
        if len(matches) == 1:
            votes.setdefault(next(iter(matches)), []).append(observation["color"])
        elif matches:
            for key in matches:
                votes.setdefault(key, []).append("Unknown")
    return {key: colors[0] if len(set(colors)) == 1 else "Unknown" for key, colors in votes.items()}


class TrainedSignalClassifier:
    """Compact learned RGB classifier; weights learned offline, no actor inputs."""

    def __init__(self, model=None, confidence=0.85, margin=0.25):
        path = model or resolve_model("trained_rgb")
        with np.load(path) as weights:
            self.w1 = weights["w1"]
            self.b1 = weights["b1"]
            self.w2 = weights["w2"]
            self.b2 = weights["b2"]
        shapes = (self.w1.shape, self.b1.shape, self.w2.shape, self.b2.shape)
        if shapes != ((3072, 96), (96,), (96, 4), (4,)):
            raise ValueError(f"Incompatible RGB MLP weights: {shapes}")
        if not all(np.isfinite(w).all() for w in (self.w1, self.b1, self.w2, self.b2)):
            raise ValueError("Non-finite RGB MLP weights")
        self.confidence = confidence
        self.margin = margin

    def __call__(self, rgb):
        import cv2

        if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8 or min(rgb.shape[:2]) < 5:
            return SignalState(reason="invalid_or_small_crop")
        if rgb.std() < 3 or np.percentile(rgb, 95) < 15:
            return SignalState(reason="unusable_exposure")
        x = cv2.resize(rgb, (32, 32)).astype(np.float32).reshape(1, -1) / 255.0
        h = np.maximum(0, x @ self.w1 + self.b1)
        z = (h @ self.w2 + self.b2).ravel()
        if not np.isfinite(z).all():
            return SignalState(reason="invalid_model_output")
        z -= z.max()
        p = np.exp(z)
        p /= p.sum()
        i = int(p.argmax())
        score = float(p[i])
        label = ("Red", "Yellow", "Green", "Unknown")[i]
        if score < self.confidence or score - float(np.partition(p, -2)[-2]) < self.margin:
            return SignalState(label=label.lower(), confidence=score, reason="low_confidence")
        return SignalState(color=label, label=label.lower(), confidence=score, reason="trained_rgb")


def fuse_surveyed_heads(observations, heads):
    """Fuse only ROIs with explicit static identity, not inferred nearest identity.

    Unknown is abstention for an unobservable redundant head. Conflicting visible
    colours remain Unknown. Green requires two distinct agreeing head ROIs; this
    deliberately cannot release at a single-head junction yet.
    """
    groups = {}
    for observation, head in zip(observations, heads):
        box = tuple(observation["box"])
        groups.setdefault(head["signal_id"], {})[box] = observation["color"]
    result = {}
    for key, heads_by_box in groups.items():
        colors = [c for c in heads_by_box.values() if c != "Unknown"]
        if not colors or len(set(colors)) != 1:
            result[key] = "Unknown"
        elif colors[0] == "Green" and len(colors) < 2:
            result[key] = "Unknown"
        else:
            result[key] = colors[0]
    return result
