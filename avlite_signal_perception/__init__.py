"""Replaceable camera traffic-light perception, independent of AVLite and CARLA."""

from .vision import AutowareSignalClassifier, SignalState, TrainedSignalClassifier

__version__ = "0.1.0"

__all__ = ["SignalState", "AutowareSignalClassifier", "TrainedSignalClassifier"]
