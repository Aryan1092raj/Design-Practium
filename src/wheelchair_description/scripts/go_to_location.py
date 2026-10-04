#!/usr/bin/env python3
"""Send the wheelchair to a named place from config/locations.yaml.

Usage: ros2 run wheelchair_description go_to_location.py kitchen
"""
import math
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def load_locations():
    path = get_package_share_directory("wheelchair_description") + "/config/locations.yaml"
    with open(path) as f:
        return yaml.safe_load(f)


def go_to(name):
    """Navigate to a saved place. Returns True on success. Voice layer can call this."""
    places = load_locations()
    if name not in places:
        print(f"unknown location '{name}'. valid: {', '.join(places)}")
        return False
    p = places[name]

    node = Node("go_to_location")
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    client.wait_for_server()

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x = float(p["x"])
    goal.pose.pose.position.y = float(p["y"])
    goal.pose.pose.orientation.z = math.sin(p["yaw"] / 2)
    goal.pose.pose.orientation.w = math.cos(p["yaw"] / 2)

    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if not handle.accepted:
        print(f"goal to '{name}' rejected")
        return False
    done = handle.get_result_async()
    rclpy.spin_until_future_complete(node, done)
    ok = done.result().status == 4  # GoalStatus.STATUS_SUCCEEDED
    print(f"{name}: {'SUCCEEDED' if ok else 'FAILED'}")
    node.destroy_node()
    return ok


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(f"usage: go_to_location.py <name>. valid: {', '.join(load_locations())}")
    rclpy.init()
    ok = go_to(sys.argv[1])
    rclpy.shutdown()
    sys.exit(0 if ok else 1)
