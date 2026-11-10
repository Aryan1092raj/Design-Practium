"""Drive straight with equal wheel commands via /cmd_vel_nav (smoother + collision monitor stay in the path).
Usage: open_straight.py <metres> <m/s>. Stops at target minus braking distance, at 25 s, or on any error."""
import math, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

dist, speed = float(sys.argv[1]), float(sys.argv[2])
src = sys.argv[3] if len(sys.argv) > 3 else '/odometry/filtered'  # /wc_control/odom for wheels-lifted runs
brake = speed ** 2 / (2 * 0.40)  # smoother max_accel 0.40 m/s^2
rclpy.init()
n = rclpy.create_node('open_straight')
pub = n.create_publisher(Twist, '/cmd_vel_nav', 10)
odom = {}
for t in ('/wc_control/odom', '/odometry/filtered'):
    n.create_subscription(Odometry, t, lambda m, t=t: odom.__setitem__(t, (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
tfb = Buffer(); TransformListener(tfb, n)

def spin(secs):
    end = time.time() + secs
    while time.time() < end: rclpy.spin_once(n, timeout_sec=0.02)

def snap():
    spin(2)
    end = time.time() + 10
    while src not in odom and time.time() < end: spin(0.2)  # DDS discovery can take a few seconds
    if not tfb.can_transform('map', 'base_link', rclpy.time.Time()):
        return dict(odom), 0.0  # wheels-lifted runs: AMCL may not publish map
    tr = tfb.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
    q = tr.rotation
    return dict(odom, map=(tr.translation.x, tr.translation.y)), math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

before, yaw0 = snap()
print(f'T0 {time.time():.3f}')
cmd, start = Twist(), time.time()
cmd.linear.x = speed
try:
    while time.time() - start < 40 and math.dist(before[src], odom[src]) < dist - brake:
        pub.publish(cmd); spin(0.05)
finally:
    stop = Twist()
    for _ in range(40): pub.publish(stop); spin(0.05)
print(f'T1 {time.time():.3f}  drive time {time.time() - start:.1f} s')
after, yaw1 = snap()
for k in before:
    print(f'{k}: {math.dist(before[k], after[k]):.3f} m')
print(f'map yaw change {yaw1 - yaw0:.1f} deg')
