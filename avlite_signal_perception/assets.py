"""Pinned external models, explicit setup, atomic download and checksum checks."""

import hashlib
import os
import tempfile
import urllib.request
from pathlib import Path

ASSETS = {
    "yolo": (
        "yolov8n.onnx",
        "https://huggingface.co/webml/yolov8n/resolve/main/onnx/yolov8n.onnx",
        "190ba5f1e61411a001683e349d6b2cdb0804c0dc67a5e34cd8ff6fd00ee54b4d",
    ),
    "autoware": (
        "traffic_light_mobilenetv2.onnx",
        "https://huggingface.co/AutowareFoundation/traffic_light_classifier/resolve/v4.0/traffic_light_classifier_mobilenetv2_batch_1.onnx",
        "455b71b3b20d3a96aa0e49f32714ba50421f668a2f9b9907c30b1346ac8a3703",
    ),
}


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def resolve_model(name, *, download=False, directory=None):
    if name == "trained_rgb":
        path = Path(__file__).parent / "models/signal_rgb_mlp.npz"
        if not path.is_file():
            raise FileNotFoundError(f"Bundled model missing: {path}")
        return path
    filename, url, expected = ASSETS[name]
    cache = (
        Path(directory)
        if directory
        else Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
        / "avlite_signal_perception"
    )
    # Development checkout may already have verified external models.
    candidates = [cache / filename]
    if directory is None:
        candidates.append(Path(__file__).resolve().parents[1] / "models" / filename)
    for path in candidates:
        if path.is_file():
            if digest(path) == expected:
                return path
            if not download:
                raise ValueError(f"Model checksum mismatch: {path}")
    if not download:
        raise FileNotFoundError(
            f"Model {name} missing. Run python -m avlite_signal_perception setup --models {name}"
        )
    cache.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=filename + ".", suffix=".download", dir=cache)
    os.close(fd)
    try:
        with urllib.request.urlopen(url, timeout=60) as response, open(tmp, "wb") as output:
            for chunk in iter(lambda: response.read(1024 * 1024), b""):
                output.write(chunk)
        if digest(tmp) != expected:
            raise ValueError(f"Model checksum mismatch for {name}; refusing install")
        Path(tmp).replace(cache / filename)
    finally:
        Path(tmp).unlink(missing_ok=True)
    return cache / filename
