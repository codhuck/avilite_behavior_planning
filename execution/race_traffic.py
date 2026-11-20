"""Deterministic slower rivals for the visible BasicSim race demonstration."""

import numpy as np
from avlite.c10_perception.c11_perception_model import AgentState, RaceMap
from avlite.c40_execution.c43_task_strategy import TaskStrategy
from avlite.c40_execution.c46_basic_sim import BasicSim


class RaceTrafficTask(TaskStrategy):
    """Two scripted cars following the route at 2 m/s, initially 18/43 m ahead."""

    def __init__(self):
        self.world = None
        self.agents = []
        self.route = None

    def reset(self):
        if self.world is not None:
            self.world.pm.agent_vehicles[:] = [
                a for a in self.world.pm.agent_vehicles if not any(a is own for own in self.agents)
            ]
        self.world = None
        self.agents = []
        self.route = None

    def execute(self, executer, event=None):
        if not isinstance(executer.world, BasicSim) or not isinstance(executer.pm.map, RaceMap):
            return
        route = executer.local_planner.global_plan
        if self.world is not executer.world or self.route is not route:
            self.reset()
            self.world = executer.world
            self.route = route
            self.started = executer.elapsed_sim_time
            ego = executer.pm.ego_vehicle
            self.origin = route.trajectory.convert_xy_to_sd(ego.x, ego.y)[0]
            for _ in range(2):
                agent = AgentState(velocity=2.0)
                self.world.pm.add_agent_vehicle(agent)
                self.agents.append(agent)
        tj = route.trajectory
        elapsed = max(0.0, executer.elapsed_sim_time - self.started)
        for offset, agent in zip((18.0, 43.0), self.agents):
            s = (self.origin + offset + 2.0 * elapsed) % tj.track_end_s
            agent.x, agent.y = tj.convert_sd_to_xy(s, 0.0)
            agent.theta = float(np.interp(s, tj.path_s, np.unwrap(tj.path_heading)))
            agent.velocity = 2.0
