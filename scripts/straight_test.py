"""Drive N m straight via Nav2, print distance moved per odom source. Usage: straight_test.py 4.0"""
import math, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

dist = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
rclpy.init()
n = Node('straight_test')
odom = {}
for t in ('/wc_control/odom', '/odometry/filtered'):
    n.create_subscription(Odometry, t, lambda m, t=t: odom.__setitem__(t, (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
tfb = Buffer(); TransformListener(tfb, n)

def snap():
    end = time.time() + 3
    while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
    tr = tfb.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
    q = tr.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return dict(odom, map=(tr.translation.x, tr.translation.y)), yaw, q

before, yaw, q = snap()
g = NavigateToPose.Goal()
g.pose.header.frame_id = 'map'
g.pose.pose.position.x = before['map'][0] + dist * math.cos(yaw)
g.pose.pose.position.y = before['map'][1] + dist * math.sin(yaw)
g.pose.pose.orientation = q
ac = ActionClient(n, NavigateToPose, 'navigate_to_pose'); ac.wait_for_server()
h = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, h)
r = h.result().get_result_async(); rclpy.spin_until_future_complete(n, r)
print('status', r.result().status)
time.sleep(1)
after, _, _ = snap()
for k in before:
    print(f'{k}: {math.dist(before[k], after[k]):.3f} m')
