#!/usr/bin/env python3
"""Refine the operator's RViz pose by fitting /scan_lidar_only to /map, then publish /initialpose.

Searches +/-0.6 m and +/-15 deg around the current AMCL pose. Exits 0 when the fitted pose puts
at least --min-fit of the scan points within 10 cm of a map wall, 1 otherwise (pose left as is).
Read-only except for one /initialpose message. Usage: scan_align.py [--min-fit 0.60]
"""
import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose2d(tf, frame):
    t = tf.lookup_transform('map', frame, rclpy.time.Time()).transform
    return t.translation.x, t.translation.y, yaw_of(t.rotation)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--min-fit', type=float, default=0.60)
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('scan_align')
    tf = Buffer()
    TransformListener(tf, node)
    got = {}
    node.create_subscription(LaserScan, '/scan_lidar_only', lambda m: got.__setitem__('scan', m), qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, '/map', lambda m: got.__setitem__('map', m), QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not (
                'scan' in got and 'map' in got and tf.can_transform('map', 'laser', rclpy.time.Time())):
            rclpy.spin_once(node, timeout_sec=0.1)
        if 'scan' not in got or 'map' not in got:
            print('FAIL scan align: /scan_lidar_only, /map or map->laser TF missing after 15 s')
            return 1
        g = got['map']
        r, ox, oy = g.info.resolution, g.info.origin.position.x, g.info.origin.position.y
        occ = np.array(g.data).reshape(g.info.height, g.info.width) >= 65
        dist = distance_transform_edt(~occ) * r
        s = got['scan']
        rng = np.array(s.ranges)
        ang = s.angle_min + np.arange(len(rng)) * s.angle_increment
        ok = np.isfinite(rng) & (rng > s.range_min) & (rng < min(s.range_max, 8.0))
        rng, ang = rng[ok], ang[ok]
        lx, ly, lth = pose2d(tf, 'laser')
        bx, by, bth = pose2d(tf, 'base_link')

        def fit(dx, dy, dth):
            x = lx + dx + rng * np.cos(ang + lth + dth)
            y = ly + dy + rng * np.sin(ang + lth + dth)
            i, j = ((y - oy) / r).astype(int), ((x - ox) / r).astype(int)
            inb = (i >= 0) & (i < occ.shape[0]) & (j >= 0) & (j < occ.shape[1])
            d = np.full(len(x), 9.9)
            d[inb] = dist[i[inb], j[inb]]
            return (d < 0.10).mean()

        f0 = fit(0, 0, 0)
        best = (f0, 0.0, 0.0, 0.0)
        for dth in np.radians(np.arange(-15, 15.1, 1.5)):
            for dx in np.arange(-0.6, 0.61, 0.1):
                for dy in np.arange(-0.6, 0.61, 0.1):
                    f = fit(dx, dy, dth)
                    if f > best[0]:
                        best = (f, dx, dy, dth)
        _, cx, cy, cth = best
        for dth in cth + np.radians(np.arange(-1.5, 1.6, 0.5)):
            for dx in cx + np.arange(-0.1, 0.11, 0.025):
                for dy in cy + np.arange(-0.1, 0.11, 0.025):
                    f = fit(dx, dy, dth)
                    if f > best[0]:
                        best = (f, dx, dy, dth)
        f, dx, dy, dth = best
        # Rotate base_link about the laser by dth, then shift by (dx, dy).
        rx, ry = bx - lx, by - ly
        nx = lx + dx + math.cos(dth) * rx - math.sin(dth) * ry
        ny = ly + dy + math.sin(dth) * rx + math.cos(dth) * ry
        nth = bth + dth
        print(f'Scan fit: operator pose {100 * f0:.0f}% -> ({nx:.2f}, {ny:.2f}, {math.degrees(nth):.1f} deg) '
              f'{100 * f:.0f}% of points within 10 cm of a map wall')
        if f < args.min_fit:
            print(f'FAIL scan align: best fit {100 * f:.0f}% is below {100 * args.min_fit:.0f}%')
            return 1
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = nx, ny
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(nth / 2), math.cos(nth / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.02 ** 2
        cov[35] = math.radians(2) ** 2
        msg.pose.covariance = cov
        for _ in range(10):  # let AMCL's subscription match before the single publish
            rclpy.spin_once(node, timeout_sec=0.1)
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.5)
        print('PASS scan align: published /initialpose')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
