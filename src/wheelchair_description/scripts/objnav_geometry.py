"""Geometry for object search: map grids, depth to map points, standoff goals, hit clusters.

No ROS and no models in here, so all of it is unit-tested with plain numpy
(test/test_objnav_geometry.py).
"""
import math
from dataclasses import dataclass

import numpy as np

MIN_DEPTH, MAX_DEPTH = 0.3, 5.0  # m; stereo depth error grows fast past ~5 m
STANDOFF = (1.5, 1.9, 2.4)  # m short of the object; sim places needed >= 1.3 m clearance
FREE_COST = 50  # global costmap cells below this count as free for a goal


@dataclass
class Grid:
    """An OccupancyGrid as numpy: data[row, col], row along y; (ox, oy) is the corner of cell (0, 0)."""
    data: np.ndarray
    res: float
    ox: float
    oy: float

    def cell(self, x, y):
        # floor, not int(): the live SLAM map has cells at negative coordinates
        return math.floor((y - self.oy) / self.res), math.floor((x - self.ox) / self.res)

    def world(self, row, col):
        return self.ox + (col + 0.5) * self.res, self.oy + (row + 0.5) * self.res

    def value(self, x, y, default=-1):
        r, c = self.cell(x, y)
        if 0 <= r < self.data.shape[0] and 0 <= c < self.data.shape[1]:
            return int(self.data[r, c])
        return default


def is_free(costmap, x, y):
    return 0 <= costmap.value(x, y) < FREE_COST


def object_depth(depth, mask):
    """Near-quartile depth of the object's pixels, or None if fewer than 20 are valid.
    The near quartile skips the background seen through and around the object."""
    d = depth[mask]
    d = d[np.isfinite(d) & (d > MIN_DEPTH) & (d < MAX_DEPTH)]
    if d.size < 20:
        return None
    return float(np.percentile(d, 25))


def box_mask(shape, box):
    """Central half of a box: the fallback when the detector gives no mask."""
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    m = np.zeros(shape, bool)
    m[int(y1 + h / 4):int(y2 - h / 4) + 1, int(x1 + w / 4):int(x2 - w / 4) + 1] = True
    return m


def mask_centre(mask):
    ys, xs = np.nonzero(mask)
    return float(xs.mean()), float(ys.mean())


def pixel_to_optical(u, v, z, k):
    """Back-project pixel (u, v) at depth z with CameraInfo.k (3x3, row-major)."""
    return np.array([(u - k[2]) / k[0] * z, (v - k[5]) / k[4] * z, z])


def quat_to_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def standoff(obj, chair, costmap):
    """First free spot short of the object, trying wider angles and then longer distances.
    Returns (x, y, yaw) with yaw facing the object, or None."""
    base = math.atan2(obj[1] - chair[1], obj[0] - chair[0])
    for dist in STANDOFF:
        for dth in (0.0, 0.5, -0.5, 1.0, -1.0):
            a = base + dth
            x, y = obj[0] - dist * math.cos(a), obj[1] - dist * math.sin(a)
            if is_free(costmap, x, y):
                return x, y, a
    return None


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Hit:
    xy: np.ndarray  # object position in the map frame
    view: tuple  # chair (x, y, yaw) when it was seen
    cam: str
    frame: object  # (rgb, box), kept for the VLM check


class Hits:
    """Detector hits grouped by map position. A cluster is ready for the VLM check only once
    two hits from different viewpoints agree, so one bad box in one frame never moves the chair."""

    MAX_HITS = 60  # ponytail: each hit keeps its RGB frame (~1 MB); keep the newest 60

    def __init__(self, radius=0.5, min_baseline=0.3, min_turn=0.3, reject_radius=0.7):
        self.radius, self.min_baseline, self.min_turn = radius, min_baseline, min_turn
        self.reject_radius = reject_radius
        self.hits, self.rejected = [], []

    def add(self, hit):
        if any(np.hypot(*(hit.xy - r)) < self.reject_radius for r in self.rejected):
            return
        self.hits = (self.hits + [hit])[-self.MAX_HITS:]

    def ready(self):
        for h in reversed(self.hits):
            near = [o for o in self.hits if np.hypot(*(o.xy - h.xy)) < self.radius]
            for o in near:
                moved = math.hypot(o.view[0] - h.view[0], o.view[1] - h.view[1]) >= self.min_baseline
                turned = abs(wrap(o.view[2] - h.view[2])) >= self.min_turn
                if moved or turned or o.cam != h.cam:
                    return np.mean([n.xy for n in near], axis=0), h
        return None

    def reject(self, xy):
        xy = np.asarray(xy)
        self.rejected.append(xy)
        self.hits = [h for h in self.hits if np.hypot(*(h.xy - xy)) >= self.reject_radius]
