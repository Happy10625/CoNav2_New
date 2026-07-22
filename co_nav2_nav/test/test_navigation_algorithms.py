import numpy as np

from co_nav2_nav.navigation_algorithms import (scan_yaws, select_frontier,
                                               standoff_candidates,
                                               within_radius)


def test_frontier_is_reachable_and_adjacent_to_unknown():
    grid = np.full((20, 20), -1, dtype=np.int16)
    grid[5:15, 5:15] = 0
    grid[9:11, 9:11] = 100
    selected, clusters = select_frontier(grid, (6, 6), min_cells=3)
    assert selected is not None
    x, y = selected["goal"]
    assert grid[y, x] == 0
    assert len(clusters) > 0


def test_standoff_avoids_occupied_cells():
    grid = np.zeros((100, 100), dtype=np.int16)
    grid[48:53, 48:53] = 100
    candidates = standoff_candidates(
        grid, object_xy=(5.0, 5.0), robot_xy=(3.0, 5.0),
        origin=(0.0, 0.0), resolution=0.1, radii=(1.0,), clearance=0.2)
    assert candidates
    x, y, yaw, radius = candidates[0]
    assert abs(np.hypot(x - 5.0, y - 5.0) - radius) < 1e-6
    assert abs(yaw - np.arctan2(5.0 - y, 5.0 - x)) < 1e-6


def test_scan_yaws_cover_one_turn_in_equal_steps():
    headings = scan_yaws(0.3, 8)
    assert len(headings) == 8
    assert np.isclose(headings[0] - 0.3, np.pi / 4.0)
    assert np.isclose(headings[-1] - 0.3, 2.0 * np.pi)


def test_test_boundary_accepts_inside_and_rejects_outside():
    assert within_radius(2.9, 0.0, (0.0, 0.0), 3.0)
    assert not within_radius(3.1, 0.0, (0.0, 0.0), 3.0)
    assert within_radius(100.0, 100.0, None, 3.0)
