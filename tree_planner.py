"""Reactive rule behavior tree, integrated with AVLite's local planning pipeline."""

from .behavior_tree import Action, Condition, Selector, Sequence, Status, Tick
from .hdmap_behavior import HDMapBehaviorPlanner
from .regulatory import RegulatoryKind


class HDMapBehaviorTreePlanner(HDMapBehaviorPlanner):
    """Tick typed regulatory branches in route order; preserve common stop checks.

    The base adapter handles route projection, observation freshness, braking
    targets and event logging. This tree selects the rule action, whose temporal
    state (STOP dwell, completed rules) is reset together with the route.
    """

    def __init__(self, global_plan, env):
        super().__init__(global_plan, env)
        self.rule_tree = Selector(
            "RegulatoryRule",
            [
                self._branch("StopSign", RegulatoryKind.STOP_SIGN, self._stop_rule),
                self._branch("TrafficLight", RegulatoryKind.TRAFFIC_LIGHT, self._light_rule),
                self._branch("Crosswalk", RegulatoryKind.CROSSWALK, self._crossing_rule),
            ],
        )

    def _branch(self, name, kind, handler):
        def act(context):
            context.values["decision"] = handler(*context.values["args"])
            return Status.SUCCESS

        return Sequence(
            name,
            [
                Condition(name + ".Applicable", lambda c: c.values["args"][0].feature.kind == kind),
                Action(name + ".Evaluate", act),
            ],
        )

    def _evaluate_rule(self, *args):
        context = Tick({"args": args})
        if self.rule_tree.tick(context) != Status.SUCCESS:
            # Unsupported map semantics must not silently grant passage.
            decision = ("PERCEPTION_STOP", "unsupported_regulatory_rule", True)
        else:
            decision = context.values["decision"]
        self.tree_trace.append({"rule_id": args[0].feature.id, "nodes": context.trace})
        return decision

    def plan_behavior(self, plan):
        self.tree_trace = []
        plan = super().plan_behavior(plan)
        plan.behavior_tree_trace = list(self.tree_trace)
        return plan

    def reset(self, wp=0):
        super().reset(wp)
        self.tree_trace = []
