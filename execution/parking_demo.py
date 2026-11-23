"""Parked vehicles for the BasicSim mission fixture."""

from avlite.c10_perception.c11_perception_model import AgentState
from avlite.c40_execution.c43_task_strategy import TaskStrategy
from avlite.c40_execution.c46_basic_sim import BasicSim

from ..settings import PluginSettings
from .parking_task import ParkingTask


class ParkingDemoTrafficTask(TaskStrategy):
    """Spawn fixture vehicles before the approach, never during bay selection."""

    def __init__(self):
        self.world = None
        self.agents = []

    def reset(self):
        if self.world is not None:
            self.world.pm.agent_vehicles[:] = [
                a for a in self.world.pm.agent_vehicles if not any(a is b for b in self.agents)
            ]
        self.world, self.agents = None, []

    def before_step(self, ex, *, tasks_enabled=True):
        if not tasks_enabled or self.world is ex.world:
            return
        if not isinstance(ex.world, BasicSim) or not PluginSettings.parking_simulator_obstacles:
            raise ValueError("ParkingDemoTrafficTask is restricted to explicit BasicSim demos")
        self.reset()
        self.world = ex.world
        for fields in ParkingTask().parking_spaces().get("demo_vehicles", []):
            agent = AgentState(**fields)
            self.world.pm.add_agent_vehicle(agent)
            self.agents.append(agent)

    def execute(self, executer, event=None):
        pass
