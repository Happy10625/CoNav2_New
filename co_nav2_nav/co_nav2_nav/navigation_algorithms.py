"""ROS-independent occupancy-grid algorithms used by the explorer."""

from collections import deque
import math
import numpy as np


FREE = 0
UNKNOWN = -1


def approach_goal_radius(target_clearance, robot_front_extent, goal_margin):
    """Return the base-to-target radius that guarantees the requested clearance."""
    return max(0.0, target_clearance + robot_front_extent - goal_margin)


def target_within_clearance(base_distance, target_clearance, robot_front_extent):
    """Return whether the robot front is within the requested target clearance."""
    return base_distance <= target_clearance + robot_front_extent


def within_radius(x, y, origin_xy, radius):
    """Return whether a world point stays inside a circular test boundary."""
    if origin_xy is None or radius <= 0.0:
        return True
    return math.hypot(x - origin_xy[0], y - origin_xy[1]) <= radius


def scan_yaws(initial_yaw, steps):
    """Return evenly spaced absolute headings for one full in-place scan."""
    if steps <= 0:
        return []
    return [initial_yaw + 2.0 * math.pi * (index + 1) / steps for index in range(steps)]


def world_to_grid(x, y, origin_x, origin_y, resolution):
    return int(math.floor((x - origin_x) / resolution)), int(math.floor((y - origin_y) / resolution))


def grid_to_world(gx, gy, origin_x, origin_y, resolution):
    return (origin_x + (gx + 0.5) * resolution, origin_y + (gy + 0.5) * resolution)


def _neighbors4(x, y, width, height):
    for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
        if 0 <= nx < width and 0 <= ny < height:
            yield nx, ny


def reachable_free(grid, start):
    """Return free cells connected to start, plus their BFS path distance."""
    height, width = grid.shape
    sx, sy = start
    reachable = np.zeros_like(grid, dtype=bool)
    distance = np.full(grid.shape, np.inf, dtype=float)
    if not (0 <= sx < width and 0 <= sy < height) or grid[sy, sx] != FREE:
        return reachable, distance
    queue = deque([(sx, sy)])
    reachable[sy, sx] = True
    distance[sy, sx] = 0.0
    while queue:
        x, y = queue.popleft()
        for nx, ny in _neighbors4(x, y, width, height):
            if not reachable[ny, nx] and grid[ny, nx] == FREE:
                reachable[ny, nx] = True
                distance[ny, nx] = distance[y, x] + 1.0
                queue.append((nx, ny))
    return reachable, distance


def frontier_clusters(grid, robot_cell, min_cells=8):
    """Find reachable free-space boundaries adjacent to unknown map cells."""
    height, width = grid.shape
    reachable, path_distance = reachable_free(grid, robot_cell)
    frontier = np.zeros_like(reachable)
    ys, xs = np.nonzero(reachable)
    for x, y in zip(xs, ys):
        if any(grid[ny, nx] == UNKNOWN for nx, ny in _neighbors4(x, y, width, height)):
            frontier[y, x] = True

    seen = np.zeros_like(frontier)
    clusters = []
    for y, x in zip(*np.nonzero(frontier)):
        if seen[y, x]:
            continue
        queue = deque([(x, y)])
        seen[y, x] = True
        cells = []
        while queue:
            cx, cy = queue.popleft()
            cells.append((cx, cy))
            for nx, ny in _neighbors4(cx, cy, width, height):
                if frontier[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    queue.append((nx, ny))
        if len(cells) >= min_cells:
            # Pick an actual reachable cell near the centroid, not a centroid
            # that can fall inside an obstacle or unknown area.
            mx = sum(p[0] for p in cells) / len(cells)
            my = sum(p[1] for p in cells) / len(cells)
            representative = min(cells, key=lambda p: (p[0] - mx) ** 2 + (p[1] - my) ** 2)
            dist = path_distance[representative[1], representative[0]]
            clusters.append({"cells": cells, "goal": representative, "distance_cells": dist})
    return clusters


def select_frontier(grid, robot_cell, min_cells=8):
    clusters = frontier_clusters(grid, robot_cell, min_cells)
    if not clusters:
        return None, clusters
    # Information gain rewards boundary length; BFS distance approximates path cost.
    best = max(clusters, key=lambda c: len(c["cells"]) - 0.35 * c["distance_cells"])
    return best, clusters


def standoff_candidates(object_xy, robot_xy, radii=(0.8, 1.0, 1.2), samples=24):
    """Return geometric target-facing poses, closest to the robot first.

    Co-Nav2 delegates traversability to its FMM planner after inflating the
    obstacle map.  The ROS 2 integration follows the same separation: this
    function does not reinterpret OccupancyGrid values or reject unknown cells;
    Nav2's planner and costmaps decide whether each pose is reachable.
    """
    ox, oy = object_xy
    candidates = []
    preferred = math.atan2(robot_xy[1] - oy, robot_xy[0] - ox)
    for radius in radii:
        ring = []
        for index in range(samples):
            angle = preferred + 2.0 * math.pi * index / samples
            x, y = ox + radius * math.cos(angle), oy + radius * math.sin(angle)
            yaw = math.atan2(oy - y, ox - x)
            travel = math.hypot(x - robot_xy[0], y - robot_xy[1])
            ring.append((travel, (x, y, yaw, radius)))
        if ring:
            return [item[1] for item in sorted(ring)]
    return candidates
