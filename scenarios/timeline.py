"""Scripted physical state, independent of planner decisions and observation faults."""

import math


def actor_states(spec, time):
    result = []
    for actor in spec.actors:
        elapsed = max(0.0, time - actor.start_time)
        result.append(
            dict(
                id=actor.id,
                kind=actor.kind,
                x=actor.x + math.cos(actor.heading) * actor.speed * elapsed,
                y=actor.y + math.sin(actor.heading) * actor.speed * elapsed,
                heading=actor.heading,
                speed=actor.speed if time >= actor.start_time else 0.0,
                length=actor.length,
                width=actor.width,
            )
        )
    return result


def signal_states(spec, time):
    states = {}
    for signal in spec.signals:
        state = signal.initial
        for change in signal.changes:
            if change.time > time + 1e-9:
                break
            state = change.state
        states[signal.id] = state
    return states


def observation_time(spec, time):
    for fault in spec.faults:
        if fault.start <= time + 1e-9 < fault.start + fault.duration - 1e-9:
            if fault.kind == "dropout":
                return None
            return max(0.0, time - fault.latency)
    return time
