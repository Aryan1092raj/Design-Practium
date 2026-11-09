#!/usr/bin/env python3
"""Offline odometry/encoder regression check; no ROS imports, nodes or sockets.

Run: python3 src/wheelchair_zupt/test/odom_selfcheck.py
Loads the actual pure integrator and callbacks by AST, then feeds fake messages.
Compiles a standard-library-only C++ fixture for the actual parser regex.
"""
import ast
import math
from pathlib import Path
import re
import subprocess
import tempfile
import threading
from types import SimpleNamespace as NS
from typing import Optional

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / 'src/wheelchair_zupt/wheelchair_zupt/zupt_node.py'
tree = ast.parse(SOURCE.read_text())
filter_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ZUPTOdometry')
node_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ZUPTNode')
callbacks = ast.ClassDef(name='Callbacks', bases=[], keywords=[], decorator_list=[], body=[
    n for n in node_class.body if isinstance(n, ast.FunctionDef) and n.name in ('_imu_callback', '_odom_callback')])
module = ast.fix_missing_locations(ast.Module(body=[filter_class, callbacks], type_ignores=[]))
namespace = dict(math=math, Optional=Optional, Imu=object, Odometry=object)
exec(compile(module, str(SOURCE), 'exec'), namespace)
Filter, Callbacks = namespace['ZUPTOdometry'], namespace['Callbacks']
EPOCH = 1794246557 * 1_000_000_000

def stamp(ns):
    return NS(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

def message(ns, x, y=0.0, yaw=0.0, vx=999.0, wz=999.0):
    return NS(header=NS(stamp=stamp(ns)), pose=NS(pose=NS(
        position=NS(x=x, y=y), orientation=NS(x=0.0, y=0.0, z=math.sin(yaw/2), w=math.cos(yaw/2)))),
        twist=NS(twist=NS(linear=NS(x=vx), angular=NS(z=wz))))

def node(theta=0.0, blend=1.0):
    n = Callbacks()
    n.filter = Filter(gyro_blend_alpha=blend)
    n.filter.reset(theta=theta)
    n._lock = threading.Lock()
    n._latest_gyro_z = n._latest_imu_stamp_ns = None
    n._initialized = False
    n.get_parameter = lambda name: NS(value=0.2)
    n.get_logger = lambda: NS(info=lambda text: None)
    n.published = []
    n._publish_state = lambda stamp: n.published.append((n.filter.x, n.filter.y, n.filter.theta))
    return n

def near(a, b, tol=1e-9):
    assert abs(a-b) <= tol, (a, b)

# Feed the actual odom callback a straight stream with the reported pose/twist
# disagreement. Jitter publication times and drop every seventh publication.
n = node(theta=0.63)
ns = EPOCH
n._odom_callback(message(ns, 100.0, -50.0, 0.63))
legacy_distance = 0.0
last_distance = 0.0
last_ns = ns
for i in range(1, 901):
    ns += (10_000_000, 30_000_000, 16_000_000, 24_000_000)[i % 4]
    distance = 3.092 * i / 900
    if i % 7 == 0 and i != 900:
        continue
    dt = (ns-last_ns)*1e-9
    vx = (distance-last_distance)/dt * (4.021/3.092)
    n._imu_callback(NS(header=NS(stamp=stamp(ns)), angular_velocity=NS(z=0.0)))
    n._odom_callback(message(ns, 100.0+distance*math.cos(0.63),
                             -50.0+distance*math.sin(0.63), 0.63, vx=vx))
    legacy_distance += vx*dt
    last_distance, last_ns = distance, ns
actual = math.hypot(n.filter.x, n.filter.y)
near(actual, 3.092)
near(legacy_distance, 4.021)
print(f'PASS straight+jitter+drops: wheel=3.092000 m filtered={actual:.6f} m error={100*abs(actual/3.092-1):.6f}%')
print(f'Legacy twist reintegration on same synthetic stream: {legacy_distance:.6f} m (+{100*(legacy_distance/3.092-1):.3f}% vs wheel)')

# Main regression: the old filter substitutes a nearly straight gyro heading
# for a curved wheel trajectory, inflating ENDPOINT distance without a speed
# scale error. The configured default must preserve wheel heading and endpoint.
launch = (ROOT/'src/wheelchair_bringup/launch/wheelchair_fusion_localization.launch.py').read_text()
assert re.search(r"'gyro_blend_alpha':\s*1\.0", launch)
assert Filter().gyro_blend_alpha == 1.0
f = Filter()
f.update(0.0, 0.0, 0.0, 0.0, EPOCH)
legacy_x = legacy_y = legacy_theta = 0.0
for i in range(1, 501):
    theta = 0.004*i
    f.update(2*math.sin(theta), 2*(1-math.cos(theta)), theta, 0.0,
             EPOCH+i*20_000_000)
    # Original slip branch: omega_odom=0.2, gyro=0, disagreement >0.08,
    # so only 5% encoder heading survives; translation still integrates 0.4 m/s.
    legacy_theta += 0.05*0.2*0.02
    legacy_x += 0.4*0.02*math.cos(legacy_theta)
    legacy_y += 0.4*0.02*math.sin(legacy_theta)
endpoint = 4*math.sin(1.0)
near(f.x, 2*math.sin(2.0))
near(f.y, 2*(1-math.cos(2.0)))
near(f.theta, 2.0)
near(math.hypot(f.x, f.y), endpoint)
print(f'PASS heading/endpoint regression: path=4.000000 m wheel_endpoint={endpoint:.6f} m filtered_endpoint={math.hypot(f.x,f.y):.6f} m; old slip blend endpoint={math.hypot(legacy_x,legacy_y):.6f} m')

# A 10-update rolling mean is sample-weighted, not time-weighted. This model
# illustrates why integrating a mean rate cannot reconstruct wheel increments;
# it is not a measurement of the live controller timing.
mean_rate = ((0.004/0.005) + (0.004/0.015))/2
print(f'Rolling-mean cadence illustration: wheel=0.008000 m mean_twist*0.020={mean_rate*0.020:.6f} m (+33.333%)')

f = Filter(gyro_blend_alpha=0.30)
f.update(50.0, 10.0, 0.0, None, EPOCH)
f.update(51.0, 10.0, 0.0, None, EPOCH+20_000_000)
assert not f.update(900.0, 800.0, 2.0, 1.0, EPOCH+20_000_000)
assert not f.update(700.0, 800.0, 2.0, 1.0, EPOCH+10_000_000)
assert not f.update(float('nan'), 10.0, 0.0, None, EPOCH+30_000_000)
f.update(52.0, 10.0, 0.0, 99.0, EPOCH+1_020_000_000)
near(f.x, 2.0)
near(f.theta, 0.0)
print('PASS duplicate/out-of-order/nonfinite rejection; >0.5 s gap retains displacement and uses wheel heading')

for speed in (-0.2, 0.005):
    f = Filter(gyro_blend_alpha=0.30)
    f.update(0.0, 0.0, 0.0, 0.0, EPOCH)
    for i in range(1, 501):
        f.update(speed*i*0.02, 0.0, 0.0, 0.0, EPOCH+i*20_000_000)
    near(f.x, speed*10)
    near(f.v_x, speed)
print('PASS reverse and 0.005 m/s motion: signed displacement retained below old ZUPT threshold')

f = Filter(gyro_blend_alpha=0.30)
f.reset(theta=3.13)
f.update(4.0, 2.0, 3.13, None, EPOCH)
f.update(4.0, 2.0, -3.13, None, EPOCH+20_000_000)
near(f.theta, -3.13)
near(f.x, 0.0)
near(f.y, 0.0)
print('PASS in-place yaw wrap: no translation')

# Exact agreement on a curved trajectory, including the source pose offset.
f = Filter(gyro_blend_alpha=0.30)
f.update(20.0, 30.0, 0.0, 0.2, EPOCH)
for i in range(1, 201):
    theta = 0.2*i*0.02
    f.update(20+2*math.sin(theta), 30+2*(1-math.cos(theta)), theta,
             0.2, EPOCH+i*20_000_000)
near(f.x, 2*math.sin(0.8))
near(f.y, 2*(1-math.cos(0.8)))
near(f.theta, 0.8)
print('PASS curved trajectory: wheel/gyro agreement preserves endpoint exactly')

f = Filter(gyro_blend_alpha=0.30)
f.update(0.0, 0.0, 0.0, 0.012, EPOCH)
for i in range(1, 301):
    f.update(0.0, 0.0, 0.0, 0.012, EPOCH+i*20_000_000)
near(f.x, 0.0)
near(f.y, 0.0)
near(f.theta, 0.0)
assert 0.011 < f.gyro_bias < 0.012
print('PASS stationary bias adaptation: pose held')

for gyro in (0.079, 0.081):
    f = Filter(gyro_blend_alpha=0.30)
    f.update(0.0, 0.0, 0.0, gyro, EPOCH)
    f.update(0.004, 0.0, 0.0, gyro, EPOCH+20_000_000)
    near(f.omega, 0.7*gyro)
    assert f.is_slip == (gyro > 0.08)
    near(math.hypot(f.x, f.y), 0.004)
print('PASS slip threshold crossing: fixed 30/70 blend, displacement length unchanged')

# Actual callbacks: gyro age validation, missing gyro, and pose-derived omega.
for imu_age in (None, 0, 300_000_000, -20_000_000):
    n = node(blend=0.30)
    n._odom_callback(message(EPOCH, 0.0))
    now = EPOCH+20_000_000
    if imu_age is not None:
        n._imu_callback(NS(header=NS(stamp=stamp(now-imu_age)), angular_velocity=NS(z=0.0)))
    n._odom_callback(message(now, 0.004, yaw=0.002))
    near(n.filter.theta, 0.0006 if imu_age == 0 else 0.002)
print('PASS callback gyro freshness: missing/stale/future gyro falls back to wheel yaw')

# Compile and execute the real C++ std::regex literals (no ROS/hardware link).
cpp = (ROOT/'src/wheelchair_firmware/src/wheelchair_interface.cpp').read_text()
def literal(name):
    return re.search(r'std::regex '+name+r'\(R"\((.*?)\)"\);', cpp).group(1)
fixture = r'''
#include <cassert>
#include <iostream>
#include <regex>
#include <string>
int main() {
  std::regex start(R"PAT(START)PAT"), right(R"PAT(RIGHT)PAT"), left(R"PAT(LEFT)PAT");
  for (auto data : {std::string("rp-0.00,lp0.04,"), std::string("rp1.00,ln-0.00,"),
                   std::string("rn0.30,lp0.25,"), std::string("rp0.00,lp0.00,")}) {
    std::smatch m;
    assert(std::regex_search(data, m, start));
    data = m[1].str();
    assert(std::regex_search(data, right));
    assert(std::regex_search(data, left));
  }
  // The original trim skipped the signed right field and lost that feedback.
  std::regex old_start(R"(([rl][pn]\d+\.?\d*.*))");
  std::string input="rp-0.00,lp0.04,";
  std::smatch m;
  assert(std::regex_search(input, m, old_start));
  assert(m[1].str() == "lp0.04,");
  std::cout << "PASS actual C++ parser regex: negative-zero and signed wheel fields preserved; original truncation reproduced\n";
}
'''
fixture = fixture.replace('START', literal('wheel_match')).replace('RIGHT', literal('right_pattern')).replace('LEFT', literal('left_pattern'))
with tempfile.TemporaryDirectory(prefix='odomfix-', dir='/tmp') as tmp:
    path = Path(tmp)/'parser.cpp'
    path.write_text(fixture)
    binary = Path(tmp)/'parser-check'
    subprocess.run(['g++', '-std=c++17', '-O0', str(path), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print('ALL OFFLINE SELF-CHECKS PASSED', flush=True)
