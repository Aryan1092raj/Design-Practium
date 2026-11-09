"""Integrate per-wheel distance from a real_nav log's WheelchairInterface lines.

Usage: encoder_ratio.py <log> <t0> <t1>   (ROS epoch seconds)
On a straight drive L/R should be about 1.0 and the encoder yaw near 0.
"""
import math
import re
import sys

WHEEL_RADIUS, WHEEL_SEPARATION = 0.1524, 0.565
LINE = re.compile(r'\[(\d+\.\d+)\] \[WheelchairInterface\]: Wheel velocities - '
                  r'Right: (-?[\d.]+) rad/s.*Left: (-?[\d.]+) rad/s')

log, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
samples = []
for line in open(log, errors='ignore'):
    m = LINE.search(line)
    if m and t0 <= float(m.group(1)) <= t1:
        samples.append(tuple(map(float, m.groups())))

right = left = 0.0
for (ta, r, l), (tb, _, _) in zip(samples, samples[1:]):
    if 0 < tb - ta <= 0.5:  # skip gaps, the same rule zupt_node uses
        right += r * WHEEL_RADIUS * (tb - ta)
        left += l * WHEEL_RADIUS * (tb - ta)

print(f'samples {len(samples)}  right {right:.3f} m  left {left:.3f} m  '
      f'L/R {left / right if right else float("nan"):.2f}  '
      f'encoder yaw {math.degrees((right - left) / WHEEL_SEPARATION):.0f} deg')
