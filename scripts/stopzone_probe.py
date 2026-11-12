#!/usr/bin/env python3
"""Read-only StopZone capture: live parameters, scans, TF and Nav2's actual points.

Run with navigation already running and the chair stationary. This observer has
no motion publishers, action clients or parameter setters; it cannot drive the chair.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import PolygonStamped
from nav2_msgs.msg import CollisionMonitorState
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray


def inside_polygon(points, polygon):
    """Nav2 Polygon::isPointInside ray-crossing convention, including its edges."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    result = np.zeros(len(points), dtype=bool)
    x, y = points.T
    previous = polygon[-1]
    for current in polygon:
        xi, yi = previous
        xj, yj = current
        crosses = (y <= yi) == (y > yj)
        if yi != yj:
            intercept = xi + (y - yi) * (xj - xi) / (yj - yi)
            result ^= crosses & (intercept > x)
        previous = current
    return result


def transform_matrix(transform):
    q, t = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return rotation, np.array([t.x, t.y, t.z])


def scan_points(message, rotation, translation):
    ranges = np.asarray(message.ranges, dtype=float)
    # Scan::getData uses the message limits, not pointcloud/range-source settings.
    valid = np.isfinite(ranges) & (ranges >= message.range_min) & (ranges <= message.range_max)
    # The native marker stream below provides the monitor's exact accumulated
    # float-angle points. This vector projection is an independent diagnostic.
    angles = message.angle_min + np.arange(len(ranges)) * message.angle_increment
    points = np.column_stack((ranges[valid] * np.cos(angles[valid]),
                              ranges[valid] * np.sin(angles[valid]), np.zeros(valid.sum())))
    return points @ rotation.T + translation, np.flatnonzero(valid)


class StopZoneProbe(Node):
    def __init__(self, monitor, directory):
        super().__init__('stopzone_probe')
        self.monitor = '/' + monitor.strip('/')
        self.directory = directory
        self.directory.mkdir(parents=True, exist_ok=True)
        self.events = (directory / 'events.jsonl').open('w')
        self.scans = {}
        self.native = None
        self.state = None
        self.polygon = None
        self.samples = 0
        self.native_samples = 0
        self.snapshot_count = 0
        self.last_snapshot = -float('inf')
        self.last_print = -float('inf')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.client = self.create_client(GetParameters, self.monitor + '/get_parameters')

    def parameters(self, names):
        if not self.client.wait_for_service(timeout_sec=5.):
            raise RuntimeError(f'{self.monitor} is not running; start the navigation stack first')
        request = GetParameters.Request(names=names)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.)
        if not future.done() or future.result() is None:
            raise RuntimeError('could not read collision monitor parameters')
        return {name: parameter_value_to_python(value)
                for name, value in zip(names, future.result().values)}

    def configure(self):
        names = ['base_frame_id', 'odom_frame_id', 'base_shift_correction',
                 'source_timeout', 'transform_tolerance', 'observation_sources',
                 'state_topic', 'StopZone.points', 'StopZone.min_points',
                 'StopZone.max_points', 'StopZone.polygon_pub_topic', 'StopZone.enabled']
        self.config = self.parameters(names)
        if self.config['StopZone.enabled'] is False:
            raise RuntimeError('StopZone is disabled; this is not a valid safety check')
        sources = self.config['observation_sources'] or []
        source_names = [f'{s}.{k}' for s in sources
                        for k in ('type', 'topic', 'enabled', 'source_timeout')]
        self.config.update(self.parameters(source_names))
        self.base = self.config['base_frame_id']
        self.odom = self.config['odom_frame_id']
        self.shift = self.config['base_shift_correction']
        maximum = self.config['StopZone.max_points']
        # Jazzy's deprecated max_points overrides min_points by adding one.
        self.threshold = maximum + 1 if maximum is not None else self.config['StopZone.min_points']
        if not self.base or not self.odom or self.threshold is None:
            raise RuntimeError('missing live StopZone geometry/frame/threshold parameters')
        value = self.config['StopZone.points']
        if value:
            self.polygon = np.asarray(json.loads(value) if isinstance(value, str) else value,
                                      dtype=float).reshape(-1, 2)
        self.active_sources = [s for s in sources if self.config.get(f'{s}.enabled') is not False]
        topics = {'/scan', '/scan_filtered', '/scan_fused', '/scan_lidar_only',
                  '/scan_front_camera', '/scan_left_camera', '/scan_right_camera'}
        self.source_topics = {}
        for source in self.active_sources:
            if self.config[f'{source}.type'] == 'scan':
                topic = self.config[f'{source}.topic']
                topics.add(topic)
                self.source_topics[source] = self.resolve_topic_name(topic)
        for topic in sorted(topics):
            topic = self.resolve_topic_name(topic)
            self.create_subscription(LaserScan, topic,
                lambda msg, name=topic: self.scans.__setitem__(name, (msg, time.monotonic())),
                qos_profile_sensor_data)
        state_topic = self.config['state_topic']
        if state_topic:
            self.create_subscription(CollisionMonitorState, state_topic, self.on_state, 10)
        self.create_subscription(MarkerArray, self.monitor + '/collision_points_marker',
                                 self.on_native, 10)
        polygon_topic = self.config['StopZone.polygon_pub_topic']
        if polygon_topic:
            self.create_subscription(PolygonStamped, polygon_topic, self.on_polygon, 10)
        self.config['effective_min_points'] = self.threshold
        (self.directory / 'parameters.json').write_text(json.dumps(self.config, indent=2) + '\n')
        self.create_timer(.2, self.observe)
        self.get_logger().info(f'Observing {self.source_topics}; StopZone triggers at {self.threshold} '
                               'points. No movement commands are published.')

    def on_native(self, message):
        self.native = message, time.monotonic()

    def on_state(self, message):
        self.state = {'polygon_name': message.polygon_name, 'action_type': message.action_type,
                      'received_ros_time': self.get_clock().now().nanoseconds / 1e9}

    def on_polygon(self, message):
        if message.header.frame_id == self.base and message.polygon.points:
            self.polygon = np.array([[p.x, p.y] for p in message.polygon.points])

    def observe(self):
        if self.polygon is None:
            return
        now = self.get_clock().now()
        wall = time.monotonic()
        record = {'ros_time': now.nanoseconds / 1e9, 'state': self.state,
                  'threshold': self.threshold, 'scan_projection': {}, 'native_points': {},
                  'missing_sources': [s for s,t in self.source_topics.items() if t not in self.scans]}
        arrays = {'polygon': self.polygon.copy()}
        counts = {}
        for index, (topic, (message, received)) in enumerate(list(self.scans.items())):
            age = (now - Time.from_msg(message.header.stamp)).nanoseconds / 1e9
            entry = {'stamp': [message.header.stamp.sec, message.header.stamp.nanosec],
                     'frame': message.header.frame_id, 'age_seconds': age,
                     'receipt_age_seconds': wall - received}
            record['scan_projection'][topic] = entry
            prefix = f'scan{index}'
            arrays[prefix + '_topic'] = topic
            arrays[prefix + '_ranges'] = np.asarray(message.ranges, np.float32)
            arrays[prefix + '_geometry'] = [message.angle_min, message.angle_increment,
                                            message.range_min, message.range_max]
            arrays[prefix + '_stamp'] = entry['stamp']
            arrays[prefix + '_frame'] = message.header.frame_id
            timeout = self.config['source_timeout']
            for source, name in self.source_topics.items():
                if topic == name and self.config.get(f'{source}.source_timeout') is not None:
                    timeout = self.config[f'{source}.source_timeout']
            if timeout and age > timeout:
                entry['error'] = 'stale scan; collision monitor rejects this source'
                continue
            try:
                if self.shift:
                    transform = self.tf.lookup_transform_full(self.base, now,
                        message.header.frame_id, Time.from_msg(message.header.stamp), self.odom,
                        timeout=Duration(seconds=.05))
                else:
                    transform = self.tf.lookup_transform(self.base, message.header.frame_id,
                                                          Time(), timeout=Duration(seconds=.05))
                rotation, translation = transform_matrix(transform.transform)
                points, ids = scan_points(message, rotation, translation)
                hit = inside_polygon(points[:, :2], self.polygon)
                entry['stopzone_points'] = int(hit.sum())
                entry['hit_base_xy'] = points[hit, :2].round(5).tolist()
                arrays[prefix + '_base_rotation'] = rotation
                arrays[prefix + '_base_translation'] = translation
                arrays[prefix + '_hit_indices'] = ids[hit]
                counts[topic] = int(hit.sum())
            except Exception as exc:
                entry['error'] = str(exc)
        if self.native is not None:
            message, received = self.native
            record['native_receipt_age_seconds'] = wall - received
            if wall - received <= 1.:
                self.native_samples += 1
                for marker in message.markers:
                    source = marker.ns.removeprefix('collision_points_')
                    if source not in self.active_sources or marker.header.frame_id != self.base:
                        continue
                    points = np.array([[p.x, p.y] for p in marker.points]).reshape(-1, 2)
                    hit = inside_polygon(points, self.polygon)
                    record['native_points'][source] = {'stopzone_points': int(hit.sum()),
                                                        'hit_base_xy': points[hit].tolist()}
                    arrays['native_' + source + '_xy'] = points
        self.events.write(json.dumps(record) + '\n')
        self.events.flush()
        self.samples += 1
        blocked = sum(record['native_points'].get(s, {}).get('stopzone_points', 0)
                      for s in self.active_sources) >= self.threshold
        projected = sum(counts.get(t, 0) for t in self.source_topics.values())
        if wall - self.last_snapshot >= (1. if blocked or projected >= self.threshold else 5.):
            np.savez_compressed(self.directory / f'points_{self.snapshot_count:03d}.npz', **arrays)
            self.snapshot_count += 1
            self.last_snapshot = wall
        if wall - self.last_print >= 2.:
            native_counts = {s: p['stopzone_points'] for s,p in record['native_points'].items()}
            positions = {s: p['hit_base_xy'][:8] for s,p in record['native_points'].items()
                         if p['stopzone_points']}
            self.get_logger().info(f'Scan counts {counts}; actual monitor counts {native_counts}; '
                                   f'trigger positions {positions}; missing sources {record["missing_sources"]}; '
                                   f'last monitor state {self.state}')
            self.last_print = wall

    def close(self):
        self.events.close()
        (self.directory / 'summary.json').write_text(json.dumps({
            'samples': self.samples, 'native_samples': self.native_samples,
            'snapshots': self.snapshot_count, 'effective_min_points': self.threshold,
            'native_note': 'Native points arrive when collision monitor processes velocity messages; '
                           'the observer does not generate velocity messages.'}, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20.)
    parser.add_argument('--monitor', default='/collision_monitor')
    parser.add_argument('--out', type=Path,
                        default=Path('logs') / ('stopzone_' + datetime.now().strftime('%Y%m%d_%H%M%S')))
    args, ros_args = parser.parse_known_args()
    if not 0 < args.seconds <= 120:
        parser.error('--seconds must be greater than 0 and at most 120')
    if (args.out / 'events.jsonl').exists():
        # Retain previous real captures if the user reuses an output directory.
        args.out = args.out / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    rclpy.init(args=ros_args)
    node = StopZoneProbe(args.monitor, args.out)
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node.configure()
        executor.add_node(node)
        deadline = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        node.get_logger().info(f'Capture saved to {args.out.resolve()}')
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        return 2
    finally:
        executor.shutdown()
        if hasattr(node, 'threshold'):
            node.close()
        else:
            node.events.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
