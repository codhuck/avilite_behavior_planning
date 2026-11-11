"""Compatibility exports for existing AVLite profiles and offline tools."""

# A source checkout also works before pip installation. No sys.path mutation.
import importlib.util
import sys
from pathlib import Path

try:
    import avlite_signal_perception  # noqa: F401 -- initialize installed package if available
except ModuleNotFoundError as error:
    if error.name != "avlite_signal_perception":
        raise
    folder = Path(__file__).parent / "avlite_signal_perception"
    spec = importlib.util.spec_from_file_location(
        "avlite_signal_perception", folder / "__init__.py", submodule_search_locations=[str(folder)]
    )
    package = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = package
    spec.loader.exec_module(package)
from avlite_signal_perception.assets import resolve_model
from avlite_signal_perception.provider import (
    SignalPerception,
    create_classifier,
    register_classifier,
)
from avlite_signal_perception.vision import (
    LABELS,
    AutowareSignalClassifier,
    SignalState,
    TrainedSignalClassifier,
    associate_signals,
    fuse_surveyed_heads,
    recognize_signals,
)

__all__ = [
    "SignalState",
    "LABELS",
    "AutowareSignalClassifier",
    "TrainedSignalClassifier",
    "recognize_signals",
    "associate_signals",
    "fuse_surveyed_heads",
    "SignalPerception",
    "create_classifier",
    "register_classifier",
    "resolve_model",
]
