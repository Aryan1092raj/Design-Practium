#!/usr/bin/env python3
"""Replay saved /scan_filtered captures through WheelchairFootprintFilter.

Offline only: no ROS nodes, no hardware. Run with
  source setup.bash --skip && .venv-voice/bin/python -B src/wheelchair_description/test/self_filter_replay_check.py
Compares the original exclusion zones (first 9 entries) with the current list, counts v30
StopZone points (x in [-0.67, 0.79], |y| <= 0.57, min_points 4) and checks that synthetic
obstacles at the chair's edge are not hidden by the added zones.
"""
import glob
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src' / 'wheelchair_localization'))
from wheelchair_localization.scan_fusion_v9 import WheelchairFootprintFilter

N_ORIGINAL_ZONES = 9
CAPTURES = ['142210', '152511', '154357']
MIN_POINTS = 4


def make_filter(n_zones, geometry, n_bins):
    """Filter using the first n_zones exclusion zones, geometry cached like the node does."""
    flt = WheelchairFootprintFilter()
    flt.exclusion_zones_rad = flt.exclusion_zones_rad[:n_zones]
    angle_min, inc, _, _ = (float(v) for v in geometry)
    scan_angles = angle_min + np.arange(n_bins, dtype=np.float32) * np.float32(inc)
    robot_angles = np.arctan2(-np.sin(scan_angles), -np.cos(scan_angles)).astype(np.float32)
    flt.cache_geometry(n_bins, robot_angles)
    return flt, scan_angles


def clean(ranges):
    out = np.array(ranges, dtype=np.float32)
    np.nan_to_num(out, nan=np.inf, copy=False)
    out[out <= 0.0] = np.inf
    return out


def to_base(ranges, scan_angles, rot, trans):
    ok = np.isfinite(ranges)
    r, a = ranges[ok], scan_angles[ok]
    pts = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=1)
    return pts @ rot.T + trans


def stopzone_count(base_pts):
    x, y = base_pts[:, 0], base_pts[:, 1]
    inside = (x >= -0.67) & (x <= 0.79) & (np.abs(y) <= 0.57)
    return int(inside.sum()), int((inside & (x < 0)).sum())


def load_scans(capture):
    """Yield (file, ranges, geometry, rot, trans) for each /scan_filtered scan."""
    for path in sorted(glob.glob(str(ROOT / 'logs' / f'stopzone_20261112_{capture}' / 'points_*.npz'))):
        d = np.load(path, allow_pickle=True)
        for key in d.keys():
            if key.endswith('_topic') and str(d[key]) == '/scan_filtered':
                k = key[:-len('_topic')]
                yield (Path(path).name, d[f'{k}_ranges'], d[f'{k}_geometry'],
                       d[f'{k}_base_rotation'], d[f'{k}_base_translation'])


results = {}
first_scan = None
for cap in CAPTURES:
    before, after, after_rear = [], [], []
    for name, ranges, geometry, rot, trans in load_scans(cap):
        if first_scan is None:
            first_scan = (ranges, geometry, rot, trans)
        counts = {}
        for label, n_zones in (('before', N_ORIGINAL_ZONES), ('after', 99)):
            flt, ang = make_filter(n_zones, geometry, len(ranges))
            kept = flt.filter_scan(clean(ranges))
            counts[label] = stopzone_count(to_base(kept, ang, rot, trans))
        before.append(counts['before'][0])
        after.append(counts['after'][0])
        after_rear.append(counts['after'][1])
    results[cap] = (before, after, after_rear)
    print(f'capture {cap}: scans={len(after)} max StopZone pts/scan before={max(before)} after={max(after)}'
          f' (after, x<0 only: {max(after_rear)})')

fail = []
for cap in ('152511', '154357'):
    ok = all(c < MIN_POINTS for c in results[cap][1])
    print(f'assert a ({cap}): all scans < {MIN_POINTS} StopZone points -> {"PASS" if ok else "FAIL"}')
    if not ok:
        fail.append(f'a {cap}: max {max(results[cap][1])}')
ok = all(c < MIN_POINTS for c in results['142210'][2])
print(f'assert b (142210): all scans < {MIN_POINTS} rear-half (x<0) StopZone points -> {"PASS" if ok else "FAIL"}')
if not ok:
    fail.append(f'b 142210: max rear-half {max(results["142210"][2])}')

# c. synthetic obstacles: one return in one beam, all others inf
ranges0, geometry, rot, trans = first_scan
angle_min, inc = float(geometry[0]), float(geometry[1])
targets = {
    'rear-left corner (-0.62, 0.52)': (-0.62, 0.52),
    'right side mid (-0.20, -0.55)': (-0.20, -0.55),
    'directly behind (-0.62, 0.0)': (-0.62, 0.0),
    'left side (0.0, 0.55)': (0.0, 0.55),
}
flt_old, ang = make_filter(N_ORIGINAL_ZONES, geometry, len(ranges0))
flt_new, _ = make_filter(99, geometry, len(ranges0))
for label, (bx, by) in targets.items():
    lx, ly, _ = rot.T @ (np.array([bx, by, 0.0]) - trans)
    idx = int(round((np.arctan2(ly, lx) - angle_min) / inc))
    assert 0 <= idx < len(ranges0), f'{label}: beam index {idx} out of scan'
    survives = {}
    for tag, flt in (('original', flt_old), ('new', flt_new)):
        r = np.full(len(ranges0), np.inf, dtype=np.float32)
        r[idx] = np.hypot(lx, ly)
        survives[tag] = bool(np.isfinite(flt.filter_scan(r)[idx]))
    if not survives['original']:
        print(f'assert c {label}: beam {idx} already hidden by ORIGINAL zones/footprint box '
              f'(new: {"survives" if survives["new"] else "hidden"}); not asserted')
    else:
        print(f'assert c {label}: beam {idx} original={survives["original"]} new={survives["new"]} '
              f'-> {"PASS" if survives["new"] else "FAIL"}')
        if not survives['new']:
            fail.append(f'c {label}: hidden by new zones')

assert not fail, f'FAILED: {fail}'
print('ALL PASS')
