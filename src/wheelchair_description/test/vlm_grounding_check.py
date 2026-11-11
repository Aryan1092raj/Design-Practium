#!/usr/bin/env python3
"""Synthetic mixed-depth regressions against the actual VoiceNav.ground method.

Run: .venv-voice/bin/python -B src/wheelchair_description/test/vlm_grounding_check.py
No ROS node, model server, network or motion; no recorded-session fixtures needed.
"""
from pathlib import Path
import math
import sys
import threading
from types import SimpleNamespace as Struct

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import vlm_nav as nav

WIDTH, HEIGHT = 64, 48


def ground(depth, encoding, camera_height=1.34):
    """Forward-facing RGB-D camera at 1.34 m, with a box covering the frame."""
    rgb, distance = nav.Image(), nav.Image()
    for message in (rgb, distance):
        message.width, message.height = WIDTH, HEIGHT
        message.header.stamp.sec = 100
        message.header.frame_id = 'camera_optical'
    rgb.encoding, rgb.step = 'rgb8', WIDTH * 3
    rgb.data = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8).tobytes()
    if encoding == '16UC1':
        # Invalid floating samples correspond to the sensor's zero-depth sentinel.
        pixels = np.round(np.nan_to_num(depth) * 1000).astype(np.uint16)
    else:
        pixels = depth.astype(np.float32)
    distance.encoding, distance.step = encoding, WIDTH * pixels.dtype.itemsize
    distance.data = pixels.tobytes()

    camera_tf = Struct(transform=Struct(
        translation=Struct(x=0., y=0., z=camera_height),
        # Optical right/down/forward -> map right-handed forward/left/up.
        rotation=Struct(x=-.5, y=.5, z=-.5, w=.5)))
    chair_tf = Struct(transform=Struct(translation=Struct(x=0., y=0., z=0.)))
    messages, geometry_calls = [], []

    def lookup(target, source, stamp, *unused):
        assert target == 'map' and source in ('camera_optical', 'base_link')
        return camera_tf if source == 'camera_optical' else chair_tf

    def near_face(chair, point):
        geometry_calls.append(point.copy())
        return point  # Stub map geometry only; depth decoding/projection remain real.

    fixture = Struct(
        frames={'camera': {'rgb': rgb, 'depth': distance,
                           'info': Struct(k=[50., 0., 32., 0., 50., 24., 0., 0., 1.])}},
        cancelled=threading.Event(), tf=Struct(lookup_transform=lookup),
        get_clock=lambda: Struct(now=lambda: nav.Time.from_msg(rgb.header.stamp)),
        get_logger=lambda: Struct(info=messages.append, warn=messages.append),
        vlm=lambda image, target: [0, 0, nav.VLM_W, nav.VLM_H],
        near_face=near_face,
        standoff=lambda point, chair: (float(point[0] - .8), float(point[1]), 0.))
    result = nav.VoiceNav.ground(fixture, 'camera', 'object')
    return result, geometry_calls, messages


def main():
    clean = np.full((HEIGHT, WIDTH), 3.)
    split = np.full_like(clean, 2.)
    split[:, WIDTH // 2:] = 3.5  # Two substantial surfaces in the central sample.
    outliers = clean.copy()
    outliers[20, 20], outliers[32, 40] = .3, 10.
    outliers[21, 21], outliers[22, 22] = np.nan, 0.
    slope = np.broadcast_to(np.linspace(1.5, 4.5, WIDTH), clean.shape).copy()
    for encoding in ('32FC1', '16UC1'):
        baseline, calls, _ = ground(clean, encoding)
        assert baseline is not None and not calls, encoding
        assert math.isclose(baseline[1], 3., abs_tol=1e-6), baseline
        for candidate in (split, split[:, ::-1].copy()):
            result, calls, messages = ground(candidate, encoding)
            assert result is None and not calls, (encoding, result)
            assert any('separated depth surfaces' in message for message in messages)
        result, calls, _ = ground(outliers, encoding)
        assert result is not None and not calls, encoding
        assert np.allclose(result[0], baseline[0]) and math.isclose(result[1], baseline[1]), result
        result, calls, _ = ground(slope, encoding)
        assert result is not None and math.isfinite(result[1]), encoding
        low, calls, _ = ground(clean, encoding, camera_height=.3)
        assert low is not None and len(calls) == 1, encoding
        print(f'PASS {encoding}: clean surface, split surfaces rejected in either order, '
              'sparse outliers retained, continuous slope retained, '
              'elevated depth preserved and low-surface map correction retained')
    print('vlm_grounding_check PASS (synthetic data; no ROS graph or model calls)')


if __name__ == '__main__':
    main()
