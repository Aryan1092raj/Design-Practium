#!/usr/bin/env python3
"""Save the chair's current AMCL pose as a named place in config/locations.yaml.

Drive the chair to the spot, facing the way it should end up, then:
    ros2 run wheelchair_description save_location.py door
Saving a name again overwrites it. voice_nav.py reads the file when it starts.
"""
import math
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from go_to_location import load_locations

HEADER = "# Named goals on maps/real_chair_indoor_2026-11-05_v2.yaml, written by save_location.py.\n"


def main():
    click = "--click" in sys.argv  # take the spot from RViz "Publish Point" instead of the chair
    args = [a for a in sys.argv[1:] if a != "--click"]
    if len(args) != 1:
        sys.exit("usage: save_location.py <name> [--click]")
    name = args[0].lower()
    rclpy.init()
    node = Node("save_location")
    got = []
    if click:
        node.create_subscription(PointStamped, "/clicked_point", got.append, 10)
        print(f"click the spot for '{name}' in RViz with Publish Point")
        while not got:
            rclpy.spin_once(node, timeout_sec=0.1)
    else:
        node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", got.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for _ in range(100):  # 10 s
            if got:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()
    if not got:
        sys.exit("no /amcl_pose in 10 s: is nav running and localized?")
    if click:
        pt = got[0].point
        entry = {"x": round(pt.x, 2), "y": round(pt.y, 2), "yaw": 0.0}  # nav ignores goal heading
    else:
        p = got[0].pose.pose
        entry = {"x": round(p.position.x, 2), "y": round(p.position.y, 2),
                 "yaw": round(2 * math.atan2(p.orientation.z, p.orientation.w), 2)}
    places = load_locations() or {}
    if "desc" in (places.get(name) or {}):
        entry["desc"] = places[name]["desc"]  # re-saving a spot keeps what the place is for
    places[name] = entry
    path = get_package_share_directory("wheelchair_description") + "/config/locations.yaml"
    with open(path, "w") as f:
        f.write(HEADER + yaml.safe_dump(places, default_flow_style=None, sort_keys=False))
    print(f"saved {name}: {entry}")


if __name__ == "__main__":
    main()
