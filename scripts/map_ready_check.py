#!/usr/bin/env python3
"""Require a live SLAM map, TF and two-minute hardware scan rates before mapping."""
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = rclpy.create_node('map_startup_check')
    tf = Buffer()
    listener = TransformListener(tf, node)
    maps = []
    counts = {'/scan': 0, '/scan_fused': 0}
    node.create_subscription(OccupancyGrid, '/map', maps.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for topic in counts:
        def count(msg, topic=topic):
            counts[topic] += 1
        node.create_subscription(LaserScan, topic, count, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if maps and maps[-1].data and all(counts.values()) and 'rviz2' in node.get_node_names():
                try:
                    tf.lookup_transform('map', 'base_link', rclpy.time.Time())
                    break
                except Exception:
                    pass
        else:
            raise RuntimeError('SLAM map, map-to-base TF, RViz or scans unavailable after 180 s')
        counts = dict.fromkeys(counts, 0)
        start = time.monotonic()
        while time.monotonic() - start < 120:
            rclpy.spin_once(node, timeout_sec=.1)
        duration = time.monotonic() - start
        for topic, count in counts.items():
            hz = count / duration
            if (topic == '/scan' and not 9 <= hz <= 11) or (topic == '/scan_fused' and hz < 8):
                raise RuntimeError(f'{topic}: {hz:.2f} Hz; requires about 10 Hz lidar / >=8 Hz fused')
            print(f'PASS {topic}: {hz:.2f} Hz over 120 s', flush=True)
        print('PASS live SLAM map, RViz and map-to-base TF; manual mapping can begin', flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('FAIL mapping readiness: ' + str(exc), flush=True)
        raise SystemExit(1)
