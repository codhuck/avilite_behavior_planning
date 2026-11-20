"""Evaluator-only staging and continuity checks for scripted CARLA walkers."""

# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import math

    import numpy as np
    from shapely.geometry import LineString

    def crossing_origin(position, yaw, obstacles, clearance=0.4):
        """Choose a five-metre crossing corridor clear of the supplied actor bodies.

        These polygons are used only to construct the experiment, never as planner
        input. Reject an impossible encounter instead of spawning inside a vehicle.
        """
        forward = np.array([math.cos(yaw), math.sin(yaw)])
        side = np.array([-forward[1], forward[0]])
        for distance in (3.5, 2.5, 4.5, 5.5, 6.5):
            origin = np.asarray(position) + distance * forward + 2.5 * side
            corridor = LineString([origin, origin - 5.0 * side]).buffer(clearance)
            if not any(corridor.intersects(body) for body in obstacles):
                return origin, -side
        raise ValueError("No collision-free pedestrian crossing corridor")

    def valid_walker_motion(previous, current, dt, speed):
        """Reject dead-actor sentinel positions and impossible displacement."""
        current = np.asarray(current, dtype=float)
        if current.shape != (3,) or not np.isfinite(current).all():
            return False
        if current[2] < -1 or current[2] > 2:
            return False
        if previous is None:
            return True
        return np.linalg.norm(current[:2] - np.asarray(previous)[:2]) <= max(0.5, 2 * speed * dt)
