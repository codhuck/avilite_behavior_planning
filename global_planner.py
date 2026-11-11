"""Lane-following routes until a geometric lane-change planner is available."""

import copy

import networkx as nx
from avlite.c20_planning.c24_global_hdmap_planners import HDMapGlobalPlanner


class HDMapLaneFollowingPlanner(HDMapGlobalPlanner):
    """Exclude lateral graph edges rather than draw an abrupt lane jump.

    A longer connected route is intentional. No path is reported if a goal
    requires a lane change; the shared map graph is never modified.
    """

    def __init__(self, map, max_velocity=3.0, wp_to_full_velocity=20):
        route_map = copy.copy(map)
        graph = map.lane_network
        route_map.lane_network = nx.subgraph_view(
            graph, filter_edge=lambda a, b: not graph[a][b].get("lane_change", False)
        )
        super().__init__(route_map, max_velocity, wp_to_full_velocity)
