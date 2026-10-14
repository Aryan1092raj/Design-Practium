"""VLFM-style value map: how much each map cell looks like the target, built from CLIP
similarity scores of the camera views that covered it (Yokoyama et al., ICRA 2024).

Fixed square grid centred on the map origin. ponytail: an 80 x 80 m extent covers a hospital
floor wing; grow it with the SLAM map if a building is bigger. Occlusion is ignored (a view
scores cells behind walls too); add a depth-based FOV cut if frontier choice suffers.
"""
import math

import numpy as np


class ValueMap:
    def __init__(self, size=80.0, res=0.10):
        n = int(round(size / res))
        self.res, self.origin, self.n = res, -size / 2, n
        self.value = np.zeros((n, n), np.float32)
        self.conf = np.zeros((n, n), np.float32)
        centres = self.origin + (np.arange(n) + 0.5) * res
        self.xs, self.ys = np.meshgrid(centres, centres)  # xs[row, col] is the x of col

    def update(self, x, y, yaw, hfov, score, max_range=4.0):
        """Blend one view's score into the cells inside its field of view, trusting cells near
        the optical axis most (confidence cos^2 of the scaled off-axis angle, as in VLFM)."""
        dx, dy = self.xs - x, self.ys - y
        ang = np.arctan2(dy, dx) - yaw
        ang = np.arctan2(np.sin(ang), np.cos(ang))
        view = (np.hypot(dx, dy) < max_range) & (np.abs(ang) < hfov / 2)
        c = np.cos(ang[view] / (hfov / 2) * (math.pi / 2)) ** 2
        old_c, old_v = self.conf[view], self.value[view]
        self.value[view] = (old_v * old_c + score * c) / (old_c + c + 1e-9)
        self.conf[view] = (old_c ** 2 + c ** 2) / (old_c + c + 1e-9)

    def at(self, x, y):
        r = math.floor((y - self.origin) / self.res)
        c = math.floor((x - self.origin) / self.res)
        if not (0 <= r < self.n and 0 <= c < self.n) or self.conf[r, c] == 0:
            return float("nan")
        return float(self.value[r, c])
