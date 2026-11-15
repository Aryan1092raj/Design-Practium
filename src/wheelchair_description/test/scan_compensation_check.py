#!/usr/bin/env python3
"""Offline stationary-return checks through the actual fusion scan callback."""
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as Struct

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'wheelchair_localization'))
from wheelchair_localization.scan_fusion_v9 import CameraOverlay, ScanFusionV9


def check(captured, current, beam=180, increment=1.0):
    ranges = np.full(360, np.inf, dtype=np.float32)
    ranges[beam] = 2.0
    original = ranges.copy()
    overlay = CameraOverlay(min_ranges=ranges, timestamp=time.monotonic(),
                            point_count=1, heading=math.radians(captured))
    published = []
    fixture = Struct(
        _initialized=True, _num_bins=360, _angle_increment=math.radians(increment),
        footprint_filter=None, _rear_crop_mask=None, lidar_pub='lidar', fused_pub='fused',
        _start_time=0, camera_warmup=0, _last_heading=None, _last_heading_time=None,
        _get_heading=lambda: math.radians(current), _cam_names=['front'],
        _overlays={'front': overlay}, max_camera_age=1, _cam_pubs={},
        _total_cam_bins=0, _frame_count=0, _total_latency_us=0,
        _publish=lambda data, scan, pub: published.append((pub, data.copy())))
    scan = Struct(header=Struct(stamp=Struct(sec=1, nanosec=0)), ranges=[np.nan] * 360)
    ScanFusionV9._scan_cb(fixture, scan)
    delta = math.atan2(math.sin(math.radians(current - captured)),
                       math.cos(math.radians(current - captured)))
    # Independent inverse rotation of a stationary point into the new chair frame.
    bearing = math.radians((beam - 180) * increment)
    x, y = math.cos(bearing), math.sin(bearing)
    rotated = math.atan2(-math.sin(delta) * x + math.cos(delta) * y,
                        math.cos(delta) * x + math.sin(delta) * y)
    change = math.atan2(math.sin(rotated - bearing), math.cos(rotated - bearing))
    expected = int(round(beam + math.degrees(change) / increment))
    indices = np.flatnonzero(np.isfinite(published[-1][1]))
    if 0 <= expected < 360:
        assert indices.tolist() == [expected], (captured, current, indices, expected)
    else:
        assert indices.size == 0, 'wrapped bins must be invalidated'
    np.testing.assert_array_equal(original, overlay.min_ranges)  # snapshot remains immutable


if __name__ == '__main__':
    for args in ((0, 10), (0, -10), (0, 0), (179, -171), (-179, 171),
                 (0, 10, 5), (0, -10, 355), (0, 10, 180, -1.0)):
        check(*args)
    print('PASS scan compensation: +/-10 degrees, heading wrap, scan edges, negative increment')
