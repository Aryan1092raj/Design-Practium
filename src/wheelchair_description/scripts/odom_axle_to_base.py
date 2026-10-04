#!/usr/bin/env python3
"""Sim only: re-reference wheel odometry from the wheel-axle midpoint to base_link.

diff_drive_controller reports the pose of the point midway between the drive wheels.
In the URDF base_link sits 0.32 m ahead of that point, so when the chair turns on the
spot base_link swings around the axle, but the odometry says it stayed put. The EKF,
AMCL and Nav2 then disagree with the real motion by up to 0.6 m per half turn.
This node publishes the same odometry shifted to base_link on /wc_control/odom_base.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

AXLE_TO_BASE = 0.31984  # |x| of the wheel joints in urdf/wheelchair_description.urdf.xacro


class AxleToBase(Node):
    def __init__(self):
        super().__init__("odom_axle_to_base")
        self.pub = self.create_publisher(Odometry, "/wc_control/odom_base", 10)
        self.create_subscription(Odometry, "/wc_control/odom", self.on_odom, 10)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
        m.pose.pose.position.x += AXLE_TO_BASE * math.cos(yaw)
        m.pose.pose.position.y += AXLE_TO_BASE * math.sin(yaw)
        m.twist.twist.linear.y += AXLE_TO_BASE * m.twist.twist.angular.z
        self.pub.publish(m)


if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(AxleToBase())
