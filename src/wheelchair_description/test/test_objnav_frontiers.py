import numpy as np
import pytest

from objnav_frontiers import Frontier, find_frontiers, pick
from objnav_geometry import Grid


def grid(data, res=0.1):
    return Grid(np.array(data, np.int8), res, 0.0, 0.0)


def half_known():
    d = np.full((40, 40), -1, np.int8)
    d[:, :20] = 0  # x < 2.0 m is known free, the rest is unknown
    return d


def test_one_frontier_along_the_known_edge():
    fs = find_frontiers(grid(half_known()), grid(np.zeros((40, 40))))
    assert len(fs) == 1
    assert fs[0].x == pytest.approx(1.95)
    assert fs[0].y == pytest.approx(2.0, abs=0.1)
    assert fs[0].size == pytest.approx(4.0)


def test_wall_between_free_and_unknown_is_not_a_frontier():
    d = half_known()
    d[:, 19] = 100
    assert find_frontiers(grid(d), grid(np.zeros((40, 40)))) == []


def test_single_unknown_speckle_is_ignored():
    d = np.zeros((40, 40), np.int8)
    d[10, 10] = -1
    assert find_frontiers(grid(d), grid(np.zeros((40, 40)))) == []


def test_goal_moves_to_a_costmap_free_cell():
    cost = np.full((40, 40), 100, np.int8)
    cost[:, 15] = 0  # only the column at x = 1.55 is free
    fs = find_frontiers(grid(half_known()), grid(cost))
    assert fs[0].x == pytest.approx(1.55)


def test_no_goal_when_nothing_free_nearby():
    assert find_frontiers(grid(half_known()), grid(np.full((40, 40), 100))) == []


NEAR, FAR = Frontier(1.0, 0.0, 2.0), Frontier(5.0, 0.0, 2.0)


def test_pick_prefers_close_over_far_of_same_size():
    assert pick([FAR, NEAR], (0.0, 0.0), []) is NEAR


def test_pick_skips_tried_and_frontiers_under_the_chair():
    assert pick([NEAR, FAR], (0.0, 0.0), [(1.1, 0.0)]) is FAR
    assert pick([Frontier(0.2, 0.0, 5.0)], (0.0, 0.0), []) is None


def test_value_can_outweigh_distance():
    def value(x, y):
        return 0.9 if x > 3 else 0.1
    assert pick([FAR, NEAR], (0.0, 0.0), [], value) is FAR


def test_unseen_value_is_neutral():
    def value(x, y):
        return float("nan") if x > 3 else 0.5
    assert pick([FAR, NEAR], (0.0, 0.0), [], value) is NEAR
