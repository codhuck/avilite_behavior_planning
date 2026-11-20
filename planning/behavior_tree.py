"""Small reactive behavior-tree runtime with explicit tick traces.

Sequences and selectors restart at their first child on every tick. Thus a
new higher-priority condition preempts a previously running branch. Long-lived
state belongs to the planner, never to an implicit composite-node cursor.
"""

from dataclasses import dataclass, field
from enum import Enum, auto


class Status(Enum):
    SUCCESS = auto()
    FAILURE = auto()
    RUNNING = auto()


@dataclass
class Tick:
    values: dict = field(default_factory=dict)
    trace: list = field(default_factory=list)


class Node:
    def __init__(self, name):
        self.name = name

    def tick(self, context):
        result = self.update(context)
        if not isinstance(result, Status):
            raise TypeError(f"{self.name} returned an invalid tree status")
        context.trace.append((self.name, result.name))
        return result

    def update(self, context):
        raise NotImplementedError


class Condition(Node):
    def __init__(self, name, predicate):
        super().__init__(name)
        self.predicate = predicate

    def update(self, context):
        return Status.SUCCESS if self.predicate(context) else Status.FAILURE


class Action(Node):
    def __init__(self, name, action):
        super().__init__(name)
        self.action = action

    def update(self, context):
        return self.action(context)


class Sequence(Node):
    def __init__(self, name, children):
        super().__init__(name)
        self.children = tuple(children)
        if not self.children:
            raise ValueError("A sequence needs children")

    def update(self, context):
        for child in self.children:
            status = child.tick(context)
            if status != Status.SUCCESS:
                return status
        return Status.SUCCESS


class Selector(Sequence):
    def update(self, context):
        for child in self.children:
            status = child.tick(context)
            if status != Status.FAILURE:
                return status
        return Status.FAILURE
