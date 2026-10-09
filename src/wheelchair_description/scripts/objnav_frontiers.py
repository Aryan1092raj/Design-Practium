"""Frontiers of the live SLAM map: known free cells next to unknown space.

Approach A picks the frontier that is long and close; approach B also weighs how much the area
around it looks like the target (objnav_value.ValueMap). Pure numpy/scipy, unit-tested in
test/test_objnav_frontiers.py.
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from objnav_geometry import is_free

OCCUPIED = 65  # slam_toolbox publishes 0 free, 100 occupied, -1 unknown
CROSS = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], bool)


@dataclass
class Frontier:
    x: float  # goal point, free in the global costmap
    y: float
    size: float  # frontier length, m


def nearest_free(costmap, x, y, radius):
    steps = int(radius / costmap.res)
    offsets = sorted(((i, j) for i in range(-steps, steps + 1) for j in range(-steps, steps + 1)),
                     key=lambda o: o[0] ** 2 + o[1] ** 2)
    for i, j in offsets:
        gx, gy = x + i * costmap.res, y + j * costmap.res
        if is_free(costmap, gx, gy):
            return gx, gy
    return None


def find_frontiers(grid, costmap, min_size=0.6, search=1.0):
    """min_size 0.6 m keeps doorway frontiers (~0.8 m) and drops one-cell speckles (0.4 m)."""
    d = grid.data
    free = (d >= 0) & (d < OCCUPIED)
    edge = free & ndimage.binary_dilation(d == -1, structure=CROSS)
    labels, n = ndimage.label(edge, structure=np.ones((3, 3), bool))
    out = []
    for i in range(1, n + 1):  # ponytail: one full-grid pass per frontier; fine at 0.05 m up to ~50 x 50 m
        rows, cols = np.nonzero(labels == i)
        size = len(rows) * grid.res
        if size < min_size:
            continue
        k = int(np.argmin((rows - rows.mean()) ** 2 + (cols - cols.mean()) ** 2))
        goal = nearest_free(costmap, *grid.world(rows[k], cols[k]), search)
        if goal is not None:
            out.append(Frontier(goal[0], goal[1], size))
    return out


def pick(frontiers, chair, tried, value=None, tried_radius=1.0):
    """Best frontier by size / (distance + 1); with a value function, also by how much its
    area looks like the target. Frontiers already tried or under the chair are skipped."""
    cands = []
    for f in frontiers:
        if any(math.hypot(f.x - t[0], f.y - t[1]) < tried_radius for t in tried):
            continue
        d = math.hypot(f.x - chair[0], f.y - chair[1])
        if d >= 0.5:
            cands.append((f, f.size / (d + 1.0)))
    if not cands:
        return None
    if value is not None:
        vals = [value(f.x, f.y) for f, _ in cands]
        seen = [v for v in vals if not math.isnan(v)]
        lo, hi = (min(seen), max(seen)) if seen else (0.0, 0.0)
        norm = [0.5 if math.isnan(v) else (v - lo) / (hi - lo) if hi > lo else 1.0 for v in vals]
        cands = [(f, s * (0.2 + 0.8 * w)) for (f, s), w in zip(cands, norm)]
    return max(cands, key=lambda c: c[1])[0]
