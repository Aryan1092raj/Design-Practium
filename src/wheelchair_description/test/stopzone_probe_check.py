#!/usr/bin/env python3
"""Probe geometry plus actual Nav2 monitor on isolated ROS domain 72.

Only a synthetic scan/TF and /probe_test_cmd velocity input are published.
No wheelchair hardware nodes, controllers or navigation actions are launched.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from datetime import datetime

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
from stopzone_probe import inside_polygon, scan_points


def main():
    polygon = np.array([[.79, .57], [.79, -.57], [-.67, -.57], [-.67, .57]])
    expected = [True, False, False, True, False]
    points = [[0., 0.], [2., 0.], [0., .6], [-.65, -.5], [-.7, 0.]]
    assert inside_polygon(points, polygon).tolist() == expected
    assert not inside_polygon([], polygon).any()
    from sensor_msgs.msg import LaserScan
    message = LaserScan(angle_min=0., angle_increment=float(math.pi),
                        range_min=.1, range_max=10., ranges=[.6, 1.53, float('inf'), 0.])
    transformed, ids = scan_points(message, np.diag([-1., -1., 1.]), np.array([.47, .12, .07]))
    np.testing.assert_allclose(transformed[:, :2], [[-.13, .12], [2., .12]], atol=1e-6)
    assert ids.tolist() == [0, 1]
    assert inside_polygon(transformed[:, :2], polygon).tolist() == [True, False]
    print('PASS geometry: backwards lidar translation, external chair preserved, invalid ranges, polygon', flush=True)

    assert os.environ.get('ROS_DOMAIN_ID') == '72', 'Run this check with ROS_DOMAIN_ID=72'
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.msg import CollisionMonitorState
    from rclpy.node import Node
    from tf2_ros import StaticTransformBroadcaster

    out = ROOT / 'bench_tmp/stopzone_20261112' / ('live_check_' + datetime.now().strftime('%H%M%S_%f'))
    out.mkdir(parents=True, exist_ok=True)
    params = {'collision_monitor': {'ros__parameters': {
        'base_frame_id': 'base_link', 'odom_frame_id': 'odom', 'base_shift_correction': True,
        'transform_tolerance': 1., 'source_timeout': .8,
        'cmd_vel_in_topic': '/probe_test_cmd', 'cmd_vel_out_topic': '/probe_test_safe',
        'enable_stamped_cmd_vel': False, 'state_topic': '/collision_monitor_state',
        'polygons': ['StopZone'], 'StopZone': {'type': 'polygon', 'enabled': True,
            'points': json.dumps(polygon.tolist()), 'action_type': 'stop', 'max_points': 3,
            'visualize': True, 'polygon_pub_topic': '/polygon_stop'},
        'observation_sources': ['scan'], 'scan': {'type': 'scan', 'topic': '/probe_test_scan'}}}}
    config = out / 'params.yaml'
    config.write_text(yaml.safe_dump(params))
    env = dict(os.environ)
    monitor_log = (out / 'monitor.log').open('w')
    probe_log = (out / 'probe.log').open('w')
    monitor = subprocess.Popen(['ros2', 'run', 'nav2_collision_monitor', 'collision_monitor',
        '--ros-args', '--params-file', str(config)], env=env, stdout=monitor_log,
        stderr=subprocess.STDOUT, start_new_session=True)
    probe = None
    rclpy.init()
    node = Node('stopzone_synthetic_fixture')
    states = []
    node.create_subscription(CollisionMonitorState, '/collision_monitor_state',
        lambda m: states.append((time.monotonic(), m.polygon_name, m.action_type)), 10)
    scan_pub = node.create_publisher(LaserScan, '/probe_test_scan', 10)
    cmd_pub = node.create_publisher(Twist, '/probe_test_cmd', 10)
    broadcaster = StaticTransformBroadcaster(node)
    transforms = []
    for parent, child, x, y, z, qz, qw in [('odom', 'base_link', 0., 0., 0., 0., 1.),
                                         ('base_link', 'laser', .47, .12, .07, 1., 0.)]:
        tf = TransformStamped()
        tf.header.stamp = node.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = parent, child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = x, y, z
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        transforms.append(tf)
    broadcaster.sendTransform(transforms)
    client = node.create_client(ChangeState, '/collision_monitor/change_state')
    try:
        assert client.wait_for_service(timeout_sec=8.), 'monitor lifecycle unavailable'
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
            assert future.done() and future.result().success, transition
        probe = subprocess.Popen([sys.executable, str(ROOT / 'scripts/stopzone_probe.py'),
            '--seconds', '9', '--out', str(out / 'capture')], env=env, stdout=probe_log,
            stderr=subprocess.STDOUT, start_new_session=True)
        phases = []
        for name, count, fresh in [('initial_clear', 0, True), ('three_points_clear', 3, True),
                                   ('four_points_stop', 4, True), ('external_obstacle_clear', 0, True),
                                   ('stale_source_stop', 0, False)]:
            start = time.monotonic()
            ros_start = node.get_clock().now().nanoseconds / 1e9
            while time.monotonic() - start < 1.5:
                message = LaserScan()
                message.header.frame_id = 'laser'
                message.header.stamp = node.get_clock().now().to_msg()
                message.angle_min, message.angle_increment = -.05, .02
                message.range_min, message.range_max = .1, 10.
                # Valid distant obstacles remain in every phase; only count
                # near-body returns changes. Never mask the synthetic scan.
                message.ranges = [.6] * count + [3.] * (8-count)
                if fresh:
                    scan_pub.publish(message)
                command = Twist()
                command.linear.x = .1
                cmd_pub.publish(command)
                rclpy.spin_once(node, timeout_sec=.03)
            phases.append({'name': name, 'start': ros_start,
                           'end': node.get_clock().now().nanoseconds / 1e9})
            if name == 'three_points_clear':
                publishers = node.get_publisher_names_and_types_by_node('stopzone_probe', '/')
                assert publishers and all(topic in ('/rosout', '/parameter_events')
                                          for topic, _ in publishers), publishers
        deadline = time.monotonic() + 5.
        while probe.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert probe.poll() == 0, (out / 'probe.log').read_text()
        records = [json.loads(line) for line in (out / 'capture/events.jsonl').read_text().splitlines()]
        config_readback = json.loads((out / 'capture/parameters.json').read_text())
        assert config_readback['effective_min_points'] == 4
        for name, expected_count in [('three_points_clear', 3), ('four_points_stop', 4),
                                     ('external_obstacle_clear', 0)]:
            phase = next(p for p in phases if p['name'] == name)
            rows = [r for r in records if phase['start']+.4 <= r['ros_time'] < phase['end']-.1]
            assert rows, name
            assert all(r['native_points'].get('scan', {}).get('stopzone_points') == expected_count
                       for r in rows), (name, rows)
            assert all(r['scan_projection']['/probe_test_scan'].get('stopzone_points') == expected_count
                       for r in rows), (name, rows)
        assert any(name == 'StopZone' and action == CollisionMonitorState.STOP for _,name,action in states)
        assert any(name == 'invalid source' and action == CollisionMonitorState.STOP for _,name,action in states)
        assert any('stale scan' in r['scan_projection'].get('/probe_test_scan',{}).get('error','')
                   for r in records)
        (out / 'result.json').write_text(json.dumps({'passed': True, 'phases': phases,
            'state_changes': states, 'record_count': len(records)}, indent=2) + '\n')
        print('PASS actual isolated Nav2: 3 points clear, 4 StopZone, distant obstacle clear, stale source stop; '
              'native/probe counts agree; no hardware nodes', flush=True)
    finally:
        for process in (probe, monitor):
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5.)
        monitor_log.close()
        probe_log.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
