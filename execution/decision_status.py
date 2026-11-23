"""Decision summaries for logs and a separate, read-only status window."""

import json
import logging
import math
import os
import tempfile
import time
import uuid
from pathlib import Path

from avlite.c40_execution.c43_task_strategy import TaskStrategy

from ..settings import PluginSettings
from .mission import ParkingMissionTask

log = logging.getLogger(__name__)


def finite_number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (ValueError, TypeError):
        return None


class DecisionStatusTask(TaskStrategy):
    """Publish actual planner decisions without inferring a signal colour or hazard."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.run_id = uuid.uuid4().hex
        self.snapshot = None
        self._key = None
        self._since = 0.0
        self._written = -math.inf
        self._write_error = None

    def execute(self, ex, event=None):
        planner = ex.local_planner
        plan = planner.get_local_plan() if planner is not None else None
        mission = next((t for t in ex.task_runner.tasks if isinstance(t, ParkingMissionTask)), None)
        now = float(ex.elapsed_sim_time)
        state = getattr(plan, "behavior_state", getattr(planner, "state", "UNKNOWN"))
        reason = getattr(
            plan, "behavior_reason", getattr(planner, "reason", "no_decision_reported")
        )
        obj = getattr(plan, "rule_id", None)
        observation = getattr(ex.pm, "regulatory_observation", None)
        stamp = getattr(observation, "time", None)
        source = getattr(observation, "source", "unspecified" if observation else "unavailable")
        if mission is not None and mission.state != "IDLE":
            state, reason, obj = mission.state, mission.reason, mission.space
            observation = mission.parking.observation(ex)
            stamp = getattr(observation, "stamp", None)
            source = (
                "BasicSim demo"
                if PluginSettings.parking_simulator_obstacles
                else "ParkingObservation"
            )
        key = (state, reason, obj)
        changed = key != self._key
        if changed or now < self._since:
            self._since = now
            self._key = key
        stamp = finite_number(stamp)
        age = now - stamp if stamp is not None else None
        self.snapshot = {
            "version": 1,
            "run_id": self.run_id,
            "updated_at": time.time(),
            "simulation_time": finite_number(now),
            "state": str(state),
            "reason": str(reason),
            "object": str(obj) if obj is not None else None,
            "distance_m": finite_number(getattr(plan, "rule_distance", None)),
            "state_duration_s": finite_number(now - self._since),
            "planner": type(planner).__name__,
            "speed_mps": finite_number(ex.pm.ego_vehicle.velocity),
            "observation_source": source,
            "observation_age_s": finite_number(age),
            "motion_holds": list(getattr(ex, "motion_holds", {}).values()),
            "parking_candidates": dict(mission.candidate_results) if mission else {},
        }
        ex.decision_status = self.snapshot
        if changed:
            log.info("Decision %s: %s; object=%s", state, reason, obj)
        filename = PluginSettings.decision_status_file
        if filename and (changed or now - self._written >= 0.25 or now < self._written):
            self._publish(filename)
            self._written = now

    def _publish(self, filename):
        path = Path(filename).expanduser()
        temporary = None
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False) as stream:
                temporary = stream.name
                json.dump(self.snapshot, stream, allow_nan=False)
            os.replace(temporary, path)
            self._write_error = None
        except OSError as exc:
            # Display failure must not abort an otherwise valid control cycle.
            if str(exc) != self._write_error:
                log.warning("Cannot write decision status %s: %s", path, exc)
                self._write_error = str(exc)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
