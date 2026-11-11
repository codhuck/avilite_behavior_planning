"""Replaceable visual perception service. Inputs/outputs contain no planner state."""

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .vision import (
    AutowareSignalClassifier,
    SignalState,
    TrainedSignalClassifier,
    associate_signals,
    fuse_surveyed_heads,
    recognize_signals,
)


class SignalClassifier(Protocol):
    def __call__(self, rgb: np.ndarray) -> SignalState: ...


_FACTORIES = {"trained_rgb": TrainedSignalClassifier, "autoware": AutowareSignalClassifier}


def register_classifier(name, factory):
    """Called explicitly by an extension; never imports code from a model file."""
    if not name or name in _FACTORIES:
        raise ValueError(f"Classifier already registered or invalid: {name}")
    if not callable(factory):
        raise TypeError("Factory must be callable")
    _FACTORIES[name] = factory


def create_classifier(name, model=None):
    if name not in _FACTORIES:
        raise ValueError(f"Unknown classifier {name}; available: {sorted(_FACTORIES)}")
    return _FACTORIES[name](model=model)


@dataclass(frozen=True)
class _Candidate:
    box: tuple
    label: str = "traffic light"
    score: float = 1.0


class SignalPerception:
    """ROI proposals supplied by caller; returns states, not throttle/braking."""

    def __init__(self, backend="trained_rgb", model=None, classifier=None, provider=None):
        if backend not in ("trained_rgb", "autoware"):
            raise ValueError(backend)
        self.backend = backend
        self.classifier = (
            classifier if classifier is not None else create_classifier(provider or backend, model)
        )

    def recognize(self, rgb, detections, heads):
        candidates = (
            [_Candidate(tuple(h["box"])) for h in heads]
            if self.backend == "trained_rgb"
            else detections
        )
        observations = recognize_signals(rgb, candidates, self.classifier)
        states = (
            fuse_surveyed_heads(observations, heads)
            if self.backend == "trained_rgb"
            else associate_signals(observations, heads)
        )
        return states, observations
