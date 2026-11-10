"""Count /scan and /scan_fused messages for N seconds, print Hz. Usage: scan_rate.py 120"""
import sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 120
rclpy.init()
n = rclpy.create_node('scan_rate')
count = {'/scan': 0, '/scan_fused': 0}
for t in count:
    n.create_subscription(LaserScan, t, lambda m, t=t: count.__setitem__(t, count[t] + 1), qos_profile_sensor_data)
end = time.time() + secs
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
print('  '.join(f'{t} {c / secs:.1f} Hz' for t, c in count.items()))
