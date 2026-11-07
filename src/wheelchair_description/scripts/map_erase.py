#!/usr/bin/env python3
"""Erase objects that are gone from a saved map, by clicking them in RViz.

Use RViz's "Publish Point" tool: click two opposite corners of the gone object. The rectangle
is painted free in a copy of the map (the source map is never changed), saved after every
rectangle. Ctrl+C when done, then launch nav with map_name:=<output .yaml>.

    ros2 run wheelchair_description map_erase.py maps/real_chair_indoor_2026-11-05_v2.yaml \
        maps/real_chair_indoor_2026-11-05_v3.yaml
"""
import os
import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

FREE = 254


def read_pgm(path):
    with open(path, "rb") as f:
        assert f.readline().strip() == b"P5", "only binary PGM maps"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        maxval = int(f.readline())
        return np.frombuffer(f.read(), np.uint8).reshape(h, w).copy(), maxval


def main():
    if len(sys.argv) != 3 or os.path.abspath(sys.argv[1]) == os.path.abspath(sys.argv[2]):
        sys.exit("usage: map_erase.py <source map .yaml> <new map .yaml> (must differ)")
    src, dst = sys.argv[1], sys.argv[2]
    meta = yaml.safe_load(open(src))
    img, maxval = read_pgm(os.path.join(os.path.dirname(src), meta["image"]))
    res, (ox, oy, _) = meta["resolution"], meta["origin"]
    dst_pgm = os.path.splitext(os.path.basename(dst))[0] + ".pgm"
    meta["image"] = dst_pgm

    rclpy.init()
    node = Node("map_erase")
    corners = []

    def save():
        with open(os.path.join(os.path.dirname(dst), dst_pgm), "wb") as f:
            f.write(b"P5\n%d %d\n%d\n" % (img.shape[1], img.shape[0], maxval) + img.tobytes())
        with open(dst, "w") as f:
            yaml.safe_dump(meta, f, sort_keys=False)

    def on_point(msg):
        corners.append((msg.point.x, msg.point.y))
        if len(corners) < 2:
            print(f"corner 1 at ({msg.point.x:.2f}, {msg.point.y:.2f}); click the opposite corner")
            return
        (x1, y1), (x2, y2) = corners
        corners.clear()
        # PGM row 0 is the top of the map (largest y)
        c1, c2 = sorted(int((x - ox) / res) for x in (x1, x2))
        r1, r2 = sorted(img.shape[0] - 1 - int((y - oy) / res) for y in (y1, y2))
        c1, r1 = max(c1, 0), max(r1, 0)
        img[r1:r2 + 1, c1:c2 + 1] = FREE
        save()
        print(f"erased x [{min(x1, x2):.2f}, {max(x1, x2):.2f}] y [{min(y1, y2):.2f}, {max(y1, y2):.2f}]"
              f" -> saved {dst}")

    node.create_subscription(PointStamped, "/clicked_point", on_point, 10)
    print("click two opposite corners of each gone object with RViz 'Publish Point'")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
