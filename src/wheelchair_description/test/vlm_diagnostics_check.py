#!/usr/bin/env python3
"""Offline RGB-D/scan capture and stationary-inspector regressions; no ROS graph."""
import json
import os
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace as Struct
from unittest.mock import patch

import numpy as np

from vlm_grounding_check import ground, nav, WIDTH, HEIGHT
from inspect_object import InspectObject


def bind(fixture, cls, *methods):
    for method in methods:
        setattr(fixture, method, getattr(cls, method).__get__(fixture))


def capture_checks(root):
    holder = []

    def configure(fixture):
        bind(fixture, nav.VoiceNav, 'dump_grounding')
        holder.append(fixture)

    depth = np.full((HEIGHT, WIDTH), 3.)
    baseline, _, _ = ground(depth, '16UC1')
    directory = root / 'new' / 'rgbd'
    with patch.dict(os.environ, VLM_NAV_DUMP=str(directory)):
        captured, _, _ = ground(depth, '16UC1', configure=configure)
        assert captured == baseline
        fixture = holder[-1]
        saved = list(directory.glob('*.npz'))
        assert len(saved) == 1
        with np.load(saved[0], allow_pickle=False) as data:
            assert data['rgb'].shape == (HEIGHT, WIDTH, 3)
            np.testing.assert_allclose(data['depth_metres'], depth)
            np.testing.assert_array_equal(data['box'], [0, 0, nav.VLM_W, nav.VLM_H])
            np.testing.assert_array_equal(data['intrinsics'], fixture.frames['camera']['info'].k)
            np.testing.assert_array_equal(data['rgb_stamp'], [100, 0])
            np.testing.assert_array_equal(data['depth_stamp'], [100, 0])
            np.testing.assert_allclose(data['map_translation'], [0, 0, 1.34])
            np.testing.assert_allclose(data['chair'], [0, 0])
            assert str(data['camera_frame']) == 'camera_optical'
            np.testing.assert_allclose(data['map_rotation'] @ [0, 0, 1], [1, 0, 0])
        for _ in range(35):
            fixture.dump_grounding(fixture.frames['camera'], 'object', [0, 0, 448, 336], depth,
                                   np.eye(3), np.zeros(3), np.zeros(2))
        assert len(list(directory.glob('*.npz'))) == 32

    blocked = root / 'not_a_directory'
    blocked.write_text('preserve this file')
    with patch.dict(os.environ, VLM_NAV_DUMP=str(blocked)):
        result, _, messages = ground(depth, '16UC1', configure=configure)
        assert result == baseline and any('RGB-D dump:' in m for m in messages)
    assert blocked.read_text() == 'preserve this file'

    def missing_tf(fixture):
        configure(fixture)
        fixture.tf.lookup_transform = lambda *args: (_ for _ in ()).throw(RuntimeError('no TF'))

    with patch.dict(os.environ, VLM_NAV_DUMP=str(root / 'missing_tf')):
        result, _, messages = ground(depth, '32FC1', configure=missing_tf)
        assert result is None and holder[-1]._depth_rejected
        with np.load(next((root / 'missing_tf').glob('*.npz')), allow_pickle=False) as data:
            np.testing.assert_allclose(data['depth_metres'], depth)
            assert np.isnan(data['map_rotation']).all()
    print('PASS RGB-D: new directory, metric depth/TF/stamps, 32-snapshot limit, write failure, missing TF')


def failure_checks():
    messages = []
    fixture = Struct(now=lambda: 100., cancelled=threading.Event(), busy=True,
                     say=messages.append, _target_seen=True, _depth_rejected=True,
                     _search_blocked=True, _last_sighting={'old': True})
    bind(fixture, nav.VoiceNav, 'reset_search_state', 'search_failure')
    fixture.reset_search_state()
    assert 'was not found' in fixture.search_failure('chair')
    fixture._search_blocked = True
    assert "couldn't finish looking" in fixture.search_failure('chair')
    fixture._target_seen = fixture._depth_rejected = True
    assert "estimate its distance reliably" in fixture.search_failure('chair')
    fixture._last_sighting = {'object': [2., 1.]}
    assert "free spot next to it" in fixture.search_failure('chair')

    def find(target, deadline):
        assert not fixture._target_seen and not fixture._depth_rejected and not fixture._search_blocked
        assert fixture._last_sighting is None
        return None

    fixture.find = find
    nav.VoiceNav.run(fixture, 'door')
    assert messages == ['Sorry, the door was not found.']
    assert fixture.command_deadline is None and not fixture.busy
    print('PASS status: absent, search blocked, uncertain depth, no approach, reset for next command')


def jpeg_checks(root):
    messages = []
    fixture = Struct(now=lambda: 100., get_logger=lambda: Struct(info=messages.append, warn=messages.append))
    replies = iter(['{"bbox_2d":[200,200,700,850]}', '{"label":"chair"}'])
    fixture.ask = lambda *args: next(replies)
    rgb = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    directory = root / 'new_jpeg'
    with patch.dict(os.environ, VLM_NAV_DUMP=str(directory)):
        assert nav.VoiceNav.vlm(fixture, rgb, 'chair') is not None
        assert len(list(directory.glob('*.jpg'))) == 1
        replies = iter(['{"bbox_2d":[200,200,700,850]}', '{"label":"chair"}'])
        with patch.object(nav.cv2, 'imwrite', return_value=False):
            assert nav.VoiceNav.vlm(fixture, rgb, 'chair') is not None
    assert any('JPEG writer returned failure' in m for m in messages)
    print('PASS verified JPEG: directory creation and failed writer reported without changing detection')


def turn_checks():
    messages, actions, dumps = [], [], []
    fixture = Struct(pose=lambda: (0., 0., 0.), clear=lambda *args: False,
                     get_logger=lambda: Struct(info=messages.append), dump_costmap=dumps.append,
                     spin_client=object())

    def send(client, goal):
        actions.append((client, goal))
        return nav.GoalStatus.STATUS_SUCCEEDED

    fixture.send = send
    assert not nav.VoiceNav.turn(fixture, 1.) and not actions and len(dumps) == 1
    fixture.pose = lambda: None
    assert not nav.VoiceNav.turn(fixture, 1.) and not actions
    fixture.pose = lambda: (0., 0., 0.)
    fixture.clear = lambda *args: True
    assert nav.VoiceNav.turn(fixture, 1.) and len(actions) == 1
    assert actions[0][1].target_yaw == 1. and actions[0][1].time_allowance.sec == 30
    print('PASS search turn: missing pose/clearance prevents action; clear pose retains Nav2 spin')


def inspector_checks(root):
    holder = []

    def configure(fixture):
        holder.append(fixture)
        bind(fixture, nav.VoiceNav, 'reset_search_state', 'ground', 'dump_grounding', 'dump_costmap',
             'coverage', 'clear', 'cost', 'standoff', 'fits')
        bind(fixture, InspectObject, 'inspect', 'on_text', 'send', 'drive', 'turn', 'go_to')
        fixture.reset_search_state()
        fixture.now = lambda: 100.
        fixture.pose = lambda: (0., 0., 0.)
        fixture.costmap = nav.OccupancyGrid()
        fixture.costmap.info.resolution = .5
        fixture.costmap.info.width = fixture.costmap.info.height = 24
        fixture.costmap.info.origin.position.x = fixture.costmap.info.origin.position.y = -6.
        fixture.costmap.data = [0] * (24 * 24)
        fixture._clear_counts = np.zeros((24, 24), np.uint8)
        fixture.diagnostic_scans = {}
        original_lookup = fixture.tf.lookup_transform

        def lookup(target, source, *args):
            if target == 'base_link':
                return Struct(transform=Struct(translation=Struct(x=.47, y=.12, z=.2),
                                                rotation=Struct(x=0., y=0., z=1., w=0.)))
            return original_lookup(target, source, *args)

        fixture.tf.lookup_transform = lookup
        for topic in ('/scan', '/scan_filtered', '/scan_fused'):
            scan = nav.LaserScan()
            scan.header.frame_id = 'laser'
            scan.header.stamp.sec = 100
            scan.angle_min, scan.angle_increment = -1., .5
            scan.range_min, scan.range_max = .2, 10.
            scan.ranges = [1., 2., float('inf')]
            fixture.diagnostic_scans[topic] = scan

    with patch.dict(os.environ, VLM_NAV_DUMP=str(root / 'inspection')):
        _, _, messages = ground(np.full((HEIGHT, WIDTH), 3.), '16UC1', configure=configure)
        fixture = holder[0]
        for command in ('go to the chair', 'stop'):
            fixture.on_text(command)
        for action in ('send', 'drive', 'turn', 'go_to'):
            try:
                getattr(fixture, action)()
            except RuntimeError as exc:
                assert 'cannot submit actions' in str(exc)
            else:
                raise AssertionError(f'{action} was not blocked')
        with patch.object(nav.rclpy, 'ok', return_value=True):
            fixture.inspect('chair')
        captures = list((root / 'inspection').glob('costmap_*.json'))
        assert len(captures) == 3
        for saved in captures:
            assert json.loads(saved.read_text())['sighting']['target'] == 'chair'
            with np.load(str(saved.with_suffix('')) + '_scans.npz', allow_pickle=False) as data:
                assert {str(data[f'scan{i}_topic']) for i in range(3)} == set(fixture.diagnostic_scans)
                for i in range(3):
                    np.testing.assert_array_equal(data[f'scan{i}_ranges'], [1., 2., np.inf])
                    np.testing.assert_array_equal(data[f'scan{i}_stamp'], [100, 0])
                    np.testing.assert_allclose(data[f'scan{i}_base_translation'], [.47, .12, .2])
        assert sum('target distance' in m for m in messages) == 3
        assert any('Inspection complete' in m for m in messages)
    print('PASS inspector: ignored transcripts, all action guards, three looks, costmap and three transformed scans')


def tight_room_checks():
    """A corridor too narrow for the 1.32 m turn circle still gets a footprint-safe standoff."""
    res, w, h = 0.02, 300, 150  # 6 m x 3 m, origin (0, -1.5)

    def standoff(half_width):
        grid = np.full((h, w), 100, np.int16)
        ys = (np.arange(h) * res - 1.5)
        grid[np.abs(ys) <= half_width, :] = 0
        grid[(np.abs(ys) <= 0.15)[:, None] & (np.arange(w) * res >= 3.9)[None, :]] = 100  # the chair
        m = Struct(info=Struct(resolution=res, width=w, height=h,
                               origin=Struct(position=Struct(x=0., y=-1.5))), data=grid.ravel())
        fixture = Struct(costmap=m, pose=lambda: (1.5, 0., 0.), _noise_revision=0, _clear_counts=None)
        bind(fixture, nav.VoiceNav, 'coverage', 'standoff', 'fits', 'clear', 'cost')
        return fixture, fixture.standoff(np.array([3.9, 0.]), np.array([1.5, 0.]))

    fixture, spot = standoff(0.8)
    assert spot is not None and 2.5 <= spot[0] <= 2.95, spot  # >= 1.0 m short of the chair
    assert fixture.fits(spot[:2], spot[2]) and not fixture.clear(spot[:2], spot[2])
    assert fixture.cost((spot[0] + 0.69 + 0.1, spot[1])) == 0  # front corner has its margin
    assert standoff(0.55)[1] is None  # footprint plus margin (1.2 m) does not fit 1.1 m
    print('PASS tight room: footprint-safe standoff where the turn circle fails; none when the footprint does not fit')


def main():
    with tempfile.TemporaryDirectory(prefix='vlm_diagnostics_') as directory:
        root = Path(directory)
        capture_checks(root)
        failure_checks()
        jpeg_checks(root)
        turn_checks()
        inspector_checks(root)
        tight_room_checks()
    print('vlm_diagnostics_check PASS (synthetic inputs; no ROS node, hardware, model or network)')


if __name__ == '__main__':
    main()
