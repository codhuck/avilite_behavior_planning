"""Bounded hybrid search for low-speed manoeuvres with direction changes."""

import heapq
import math
import time

import numpy as np
from shapely.geometry import Polygon


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def rollout(pose, direction, curvature, distance=0.6, spacing=0.04):
    travel = np.linspace(0, distance, max(2, int(math.ceil(distance / spacing)) + 1)) * direction
    x, y, yaw = pose
    headings = yaw + travel * curvature
    if abs(curvature) < 1e-8:
        return np.column_stack([x + travel * math.cos(yaw), y + travel * math.sin(yaw), headings])
    return np.column_stack(
        [
            x + (np.sin(headings) - math.sin(yaw)) / curvature,
            y - (np.cos(headings) - math.cos(yaw)) / curvature,
            headings,
        ]
    )


def search(
    start,
    goal,
    length,
    width,
    drivable,
    obstacles,
    final_direction=-1,
    rear_offset=0.0,
    clearance=0.18,
    max_nodes=20000,
    timeout=12.0,
    initial_direction=None,
    position_tolerance=0.18,
    refine_goal=False,
    max_expansions=None,
):
    """Return constant-direction segments or an explicit bounded failure.

    Search nodes are quantized only for deduplication; collision checks and
    returned poses use continuous coordinates and exact circular primitives.
    """
    if max_expansions is not None and (not isinstance(max_expansions, int) or max_expansions <= 0):
        raise ValueError("max_expansions must be a positive integer")
    if timeout is not None and (not math.isfinite(timeout) or timeout < 0):
        raise ValueError("timeout must be nonnegative or None")
    corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * [
        length / 2 + clearance,
        width / 2 + clearance,
    ]

    def body(pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return Polygon(
            corners @ np.array([[c, s], [-s, c]]) + [x + rear_offset * c, y + rear_offset * s]
        )

    def valid(a, b):
        # Convex hull with an additional arc sagitta allowance.
        swept = body(a).union(body(b)).convex_hull.buffer(0.02)
        return drivable.covers(swept) and not any(swept.intersects(o) for o in obstacles)

    grid_resolution = min(0.25, max(0.05, position_tolerance * 1.4))
    cosine, sine = math.cos(start[2]), math.sin(start[2])

    def key(pose, direction):
        # Quantize in the manoeuvre frame: translating/rotating a test scene
        # must not change which physically equivalent nodes are deduplicated.
        dx, dy = pose[0] - start[0], pose[1] - start[1]
        return (
            round((cosine * dx + sine * dy) / grid_resolution),
            round((-sine * dx + cosine * dy) / grid_resolution),
            round(wrap(pose[2] - start[2]) / math.radians(5)),
            direction,
        )

    def heuristic(p):
        return math.hypot(p[0] - goal[0], p[1] - goal[1]) + 2 * abs(wrap(p[2] - goal[2]))

    poses = [tuple(start)]
    parents = [None]
    moves = [None]
    costs = [0.0]
    queue = [(heuristic(start), 0)]
    best = {key(start, final_direction): 0.0}
    began = time.monotonic()
    expansions = 0
    while (
        queue
        and len(poses) < max_nodes
        and (timeout is None or time.monotonic() - began < timeout)
        and (max_expansions is None or expansions < max_expansions)
    ):
        _, i = heapq.heappop(queue)
        p = poses[i]
        prior = moves[i][0] if moves[i] else final_direction
        if costs[i] > best.get(key(p, prior), float("inf")):
            continue
        if (
            # A nearby node is only a shooting-method seed. The fitted path
            # still has to meet the exact endpoint and swept-body checks below.
            math.hypot(p[0] - goal[0], p[1] - goal[1])
            < (0.8 if refine_goal else position_tolerance)
            and abs(wrap(p[2] - goal[2])) < math.radians(15 if refine_goal else 4)
            and prior == final_direction
        ):
            edges = []
            cursor = i
            while parents[cursor] is not None:
                previous = parents[cursor]
                direction, curvature, distance = moves[cursor]
                edges.append((direction, rollout(poses[previous], direction, curvature, distance)))
                cursor = previous
            ordered = list(reversed(edges))
            if refine_goal:
                ordered = refine_endpoint(ordered, goal, valid)
            if ordered:
                segments = []
                for direction, points in ordered:
                    if segments and segments[-1][0] == direction:
                        segments[-1] = (direction, np.vstack([segments[-1][1], points[1:]]))
                    else:
                        segments.append((direction, points))
                return segments, "found"
        expansions += 1
        distance = 0.3 if heuristic(p) < 2 else 0.6
        for direction in (final_direction, -final_direction):
            if i == 0 and initial_direction is not None and direction != initial_direction:
                continue
            for curvature in (0.0, -0.2, -0.1, 0.1, 0.2):
                q = rollout(p, direction, curvature, distance, spacing=distance)[-1]
                score = (
                    costs[i]
                    + distance * (1 + 0.15 * abs(curvature))
                    + (2.5 if direction != prior else 0)
                )
                k = key(q, direction)
                if best.get(k, float("inf")) <= score or not valid(p, q):
                    continue
                best[k] = score
                poses.append(tuple(q))
                parents.append(i)
                moves.append((direction, curvature, distance))
                costs.append(score)
                heapq.heappush(queue, (score + 1.5 * heuristic(q), len(poses) - 1))
    return None, "search_budget_exhausted" if queue else "no_path"


def refine_endpoint(edges, goal, valid):
    """Fit the final primitives to the exact requested pose and recheck collision.

    The outward search's target is the current vehicle pose. Returning a path
    merely near that pose can make a short first reverse segment untrackable.
    Refinement preserves gear choices and curvature limits; failure rejects it.
    """
    from scipy.optimize import least_squares

    if not edges:
        return None
    prefix, tail = edges[:-3], edges[-3:]
    controls = []
    for direction, points in tail:
        distance = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1).sum()
        curvature = wrap(points[-1, 2] - points[0, 2]) / (direction * distance)
        controls.append((direction, curvature, distance))
    while len(controls) < 3:
        direction, curvature, distance = controls.pop()
        controls.extend([(direction, curvature, distance / 2)] * 2)
    start = tail[0][1][0]
    count = len(controls)
    initial = np.array([c[1] for c in controls] + [c[2] for c in controls])
    initial[:count] = np.clip(initial[:count], -0.2, 0.2)

    def endpoint(values, dense=False):
        p = start
        paths = []
        for i, (direction, _, _) in enumerate(controls):
            path = rollout(
                p,
                direction,
                values[i],
                values[count + i],
                spacing=0.04 if dense else values[count + i],
            )
            paths.append((direction, path))
            p = path[-1]
        return p, paths

    def residual(values):
        p, _ = endpoint(values)
        return np.r_[p[:2] - goal[:2], 2 * wrap(p[2] - goal[2]), 0.001 * (values - initial)]

    fit = least_squares(
        residual,
        initial,
        bounds=(
            np.r_[np.full(count, -0.2), np.full(count, 0.02)],
            np.r_[np.full(count, 0.2), np.full(count, 1.5)],
        ),
        max_nfev=80,
    )
    final, paths = endpoint(fit.x, dense=True)
    if np.linalg.norm(final[:2] - goal[:2]) > 0.01 or abs(wrap(final[2] - goal[2])) > math.radians(
        0.5
    ):
        return None
    if any(not valid(a, b) for _, path in paths for a, b in zip(path, path[1:])):
        return None
    return prefix + paths
