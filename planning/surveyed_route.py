"""Fixed, surveyed approach route for the parking mission example."""

from avlite.c20_planning.c21_planning_model import GlobalPlan
from avlite.c20_planning.c22_global_planning_strategy import GlobalPlannerStrategy
from avlite.c40_execution.c49_settings import ExecutionSettings


class SurveyedRoutePlanner(GlobalPlannerStrategy):
    """Load a supplied route; this planner does not search around obstacles."""

    def __init__(self, map=None, route=None):
        super().__init__(map)
        self.global_plan = (
            route
            if route is not None
            else GlobalPlan.from_file(ExecutionSettings.c40_global_trajectory)
        )
        if len(self.global_plan.path) < 2:
            raise ValueError("A surveyed route needs at least two points")

    def plan(self, perception_model=None, sensors=None):
        return self.global_plan
