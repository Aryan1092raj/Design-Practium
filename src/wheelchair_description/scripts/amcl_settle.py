#!/usr/bin/env python3
"""Request AMCL updates while stopped: 15 calls at 5 Hz after 0.3 s of zero command.

Both wheel bridges publish the final TwistStamped command on /wc_control/cmd_vel.
The service calls are asynchronous; an unavailable service is retried on the next tick.
"""
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Empty


def is_stopped(twist):
    return all(abs(v) < 0.01 for vector in (twist.linear, twist.angular)
               for v in (vector.x, vector.y, vector.z))


class AmclSettle(Node):
    def __init__(self):
        super().__init__("amcl_settle")
        topic = self.declare_parameter("cmd_vel_topic", "/wc_control/cmd_vel").value
        self.stopped_since = None
        self.calls = 0
        self.client = self.create_client(Empty, "/request_nomotion_update")
        self.create_subscription(TwistStamped, topic, self.on_cmd_vel, qos_profile_sensor_data)
        self.create_timer(0.2, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_cmd_vel(self, msg):
        if not is_stopped(msg.twist):
            self.stopped_since = None
            self.calls = 0
        elif self.stopped_since is None:
            self.stopped_since = self.now()

    def tick(self):
        if self.stopped_since is None or self.calls >= 15:
            return
        if self.now() < self.stopped_since + 0.3 or not self.client.service_is_ready():
            return
        self.client.call_async(Empty.Request())
        self.calls += 1


def main():
    rclpy.init()
    node = AmclSettle()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
