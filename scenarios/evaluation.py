"""Audit physical traces without consulting the planner's reported success state."""

import math

from shapely.geometry import Polygon

from .timeline import signal_states


def footprint(state):
    c, s = math.cos(state["heading"]), math.sin(state["heading"])
    half_l, half_w = state["length"] / 2, state["width"] / 2
    return Polygon(
        [
            (state["x"] + c * x - s * y, state["y"] + s * x + c * y)
            for x, y in ((half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w))
        ]
    )


class ScenarioAudit:
    """Count contact episodes and front-bumper entries; trace time is simulation time.

    Contact uses relative-motion interpolation within each tick. This is a 2D
    sampled audit, not continuous collision detection or a vehicle safety proof.
    """

    def __init__(self, spec, route, crossings, lights, drivable):
        self.spec, self.route = spec, route
        self.crossings, self.lights, self.drivable = crossings, lights, drivable
        self.previous = None
        self.contacts = set()
        self.crossing_inside = set()
        self.events = []
        self.metrics = dict(
            collisions=0,
            red_light_entries=0,
            occupied_crossing_entries=0,
            offroad_seconds=0.0,
            stopped_seconds=0.0,
            progress_m=0.0,
            minimum_clearance_m=None,
            completed=False,
        )

    def update(self, frame):
        from shapely.geometry import Point

        ego = frame["ego"]
        body = footprint(ego)
        now = frame["time"]
        previous = self.previous
        dt = 0.0 if previous is None else now - previous["time"]
        if dt < 0 or dt > self.spec.dt + 1e-6:
            raise ValueError("Non-monotonic or incomplete scenario trace")
        self.metrics["progress_m"] = float(self.route.project(Point(ego["x"], ego["y"])))
        self.metrics["completed"] = (
            self.metrics["progress_m"] >= self.route.length - 0.8
            and Point(ego["x"], ego["y"]).distance(Point(self.route.coords[-1])) <= 0.8
            and abs(ego["speed"]) <= 0.1
        )
        self.metrics["stopped_seconds"] += dt if abs(ego["speed"]) <= 0.1 else 0
        self.metrics["offroad_seconds"] += dt if not self.drivable.buffer(0.05).covers(body) else 0
        contacts = set()
        old_actors = {a["id"]: a for a in previous["actors"]} if previous else {}
        for actor in frame["actors"]:
            clearance = body.distance(footprint(actor))
            old_ego = previous["ego"] if previous else ego
            old_actor = old_actors.get(actor["id"], actor)
            for fraction in (0.0, 0.25, 0.5, 0.75):
                e, a = dict(ego), dict(actor)
                for key in ("x", "y", "heading"):
                    e[key] = old_ego[key] + fraction * (ego[key] - old_ego[key])
                    a[key] = old_actor[key] + fraction * (actor[key] - old_actor[key])
                clearance = min(clearance, footprint(e).distance(footprint(a)))
            minimum = self.metrics["minimum_clearance_m"]
            self.metrics["minimum_clearance_m"] = (
                clearance if minimum is None else min(minimum, clearance)
            )
            if clearance <= 1e-8:
                contacts.add(actor["id"])
                if actor["id"] not in self.contacts:
                    self._event("collisions", now, actor["id"])
        self.contacts = contacts
        front = self._front(ego)
        if previous:
            old_front = self._front(previous["ego"])
            for identifier, stop_s in self.lights.items():
                if old_front < stop_s <= front:
                    entered = previous["time"] + dt * (stop_s - old_front) / (front - old_front)
                    if signal_states(self.spec, entered)[identifier] != "Green":
                        self._event("red_light_entries", entered, identifier)
        inside = set()
        for identifier, area in self.crossings.items():
            if body.intersects(area):
                inside.add(identifier)
                occupied = any(
                    footprint(a).intersects(area)
                    for a in frame["actors"]
                    if a["kind"] == "pedestrian"
                )
                if identifier not in self.crossing_inside and occupied:
                    self._event("occupied_crossing_entries", now, identifier)
        self.crossing_inside = inside
        self.previous = frame

    def _front(self, ego):
        from shapely.geometry import Point

        return self.route.project(
            Point(
                ego["x"] + ego["length"] / 2 * math.cos(ego["heading"]),
                ego["y"] + ego["length"] / 2 * math.sin(ego["heading"]),
            )
        )

    def _event(self, metric, time, identifier):
        self.metrics[metric] += 1
        self.events.append(dict(kind=metric, time=time, object_id=identifier))

    def result(self):
        criteria = self.spec.criteria
        failures = []
        for metric in (
            "collisions",
            "red_light_entries",
            "occupied_crossing_entries",
            "offroad_seconds",
            "stopped_seconds",
        ):
            limit = getattr(criteria, "max_" + metric)
            if self.metrics[metric] > limit + 1e-8:
                failures.append(f"{metric}: {self.metrics[metric]:.6g} exceeds {limit}")
        if criteria.require_completion and not self.metrics["completed"]:
            failures.append("Route not completed within the time budget")
        return dict(
            passed=not failures, failures=failures, metrics=self.metrics, events=self.events
        )
