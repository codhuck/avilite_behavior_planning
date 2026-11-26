"""Versioned scenario inputs. No simulator imports or executable expressions."""

import math
import random
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator


class Model(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, validate_default=True)


class Actor(Model):
    id: str = Field(min_length=1, pattern=r"^[A-Za-z0-9_-]+$")
    kind: Literal["vehicle", "pedestrian"]
    x: float
    y: float
    heading: float = 0.0
    speed: float = Field(default=0, ge=0, le=30)
    start_time: float = Field(default=0, ge=0)
    length: float = Field(default=0.6, gt=0, le=20)
    width: float = Field(default=0.6, gt=0, le=5)


class SignalChange(Model):
    time: float = Field(ge=0)
    state: Literal["Red", "Yellow", "Green", "Off", "Unknown"]


class Signal(Model):
    id: str = Field(min_length=1)
    initial: Literal["Red", "Yellow", "Green", "Off", "Unknown"] = "Red"
    changes: list[SignalChange] = Field(default_factory=list)

    @model_validator(mode="after")
    def ordered(self):
        times = [c.time for c in self.changes]
        if times != sorted(set(times)):
            raise ValueError("Signal changes must have strictly increasing times")
        return self


class Fault(Model):
    start: float = Field(ge=0)
    duration: float = Field(gt=0)
    kind: Literal["dropout", "delay"]
    latency: float = Field(default=0, ge=0, le=10)

    @model_validator(mode="after")
    def delay_value(self):
        if self.kind == "delay" and self.latency <= 0:
            raise ValueError("A delay fault needs positive latency")
        if self.kind == "dropout" and self.latency != 0:
            raise ValueError("Dropout has no latency parameter")
        return self


class Criteria(Model):
    require_completion: bool = True
    max_collisions: int = Field(default=0, ge=0)
    max_red_light_entries: int = Field(default=0, ge=0)
    max_occupied_crossing_entries: int = Field(default=0, ge=0)
    max_offroad_seconds: float = Field(default=0, ge=0)
    max_stopped_seconds: float = Field(default=25, ge=0)


class ScenarioSpec(Model):
    version: Literal[1] = 1
    name: str = Field(min_length=1, pattern=r"^[A-Za-z0-9_-]+$")
    backend: str = "basicsim"
    map: str = Field(min_length=1)
    route: str = Field(min_length=1)
    strategy: Literal["road-rules", "road-tree"] = "road-rules"
    seed: int = Field(default=0, ge=0, le=2**32 - 1)
    dt: float = Field(default=0.05, ge=0.01, le=0.1)
    duration: float = Field(default=65, gt=0, le=600)
    cruise_speed: float = Field(default=3, gt=0, le=10)
    actors: list[Actor] = Field(default_factory=list)
    signals: list[Signal] = Field(default_factory=list)
    faults: list[Fault] = Field(default_factory=list)
    criteria: Criteria = Field(default_factory=Criteria)

    @model_validator(mode="after")
    def consistency(self):
        for values in (self.actors, self.signals):
            ids = [item.id for item in values]
            if len(ids) != len(set(ids)):
                raise ValueError("Duplicate actor or signal IDs")
        if not math.isclose(self.duration / self.dt, round(self.duration / self.dt), abs_tol=1e-7):
            raise ValueError("duration must be a multiple of dt")
        times = [a.start_time for a in self.actors]
        times += [c.time for s in self.signals for c in s.changes]
        if any(t >= self.duration for t in times):
            raise ValueError("An event starts outside the scenario duration")
        faults = sorted(self.faults, key=lambda f: f.start)
        for i, fault in enumerate(faults):
            if fault.start + fault.duration > self.duration:
                raise ValueError("Fault extends outside the scenario duration")
            if i and fault.start < faults[i - 1].start + faults[i - 1].duration:
                raise ValueError("Overlapping observation faults are ambiguous")
        return self


def resolve_parameters(document, seed):
    """Sample in sorted-name order; only entire '$name' values are substituted."""
    document = dict(document)
    parameters = document.pop("parameters", {})
    if not isinstance(parameters, dict):
        raise ValueError("parameters must be a mapping")
    rng, sampled = random.Random(seed), {}
    for name, definition in sorted(parameters.items()):
        if not isinstance(name, str) or not isinstance(definition, dict):
            raise ValueError("Invalid parameter definition")
        if set(definition) == {"uniform"}:
            bounds = definition["uniform"]
            if not isinstance(bounds, list) or len(bounds) != 2:
                raise ValueError("uniform needs [minimum, maximum]")
            low, high = bounds
            if (
                any(type(v) not in (int, float) or not math.isfinite(v) for v in bounds)
                or low > high
            ):
                raise ValueError("Invalid uniform bounds")
            sampled[name] = rng.uniform(low, high)
        elif set(definition) == {"choice"}:
            choices = definition["choice"]
            if not isinstance(choices, list) or not choices:
                raise ValueError("choice needs a nonempty list")
            if any(type(v) not in (str, int, float, bool) for v in choices):
                raise ValueError("choice values must be scalar")
            if any(isinstance(v, float) and not math.isfinite(v) for v in choices):
                raise ValueError("choice numbers must be finite")
            sampled[name] = rng.choice(choices)
        else:
            raise ValueError("Use uniform or choice for each parameter")

    def replace(value):
        if isinstance(value, str) and value.startswith("$"):
            if value[1:] not in sampled:
                raise ValueError(f"Undefined parameter: {value}")
            return sampled[value[1:]]
        if isinstance(value, dict):
            return {key: replace(item) for key, item in value.items()}
        if isinstance(value, list):
            return [replace(item) for item in value]
        return value

    return replace(document), sampled


def load_scenario(path, seed=None, strategy=None):
    path = Path(path).resolve()
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict):
        raise ValueError("Scenario must be a mapping")
    seed = document.get("seed", 0) if seed is None else seed
    if type(seed) is not int or not 0 <= seed < 2**32:
        raise ValueError("seed must be a nonnegative 32-bit integer")
    resolved, sampled = resolve_parameters(document, seed)
    resolved["seed"] = seed
    if strategy is not None:
        resolved["strategy"] = strategy
    spec = ScenarioSpec.model_validate(resolved)
    for key in ("map", "route"):
        asset = (path.parent / getattr(spec, key)).resolve()
        if not asset.is_file():
            raise ValueError(f"Missing {key} asset: {asset}")
        setattr(spec, key, str(asset))
    return spec, sampled
