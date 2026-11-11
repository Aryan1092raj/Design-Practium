#!/usr/bin/env python3
"""Exercise actual ZUPT publication at an offset base frame; no ROS graph/motion."""
from pathlib import Path
import math
import sys
import threading
from types import SimpleNamespace as S

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from wheelchair_zupt.zupt_node import ZUPTNode, ZUPTOdometry


def publish(offset, yaw, vx=.2, omega=.8):
    odometry, transforms = [], []
    f = ZUPTOdometry()
    f.reset(x=2., y=-1., theta=yaw)
    f.v_x, f.omega = vx, omega
    node = S(_lock=threading.Lock(), _initialized=True, filter=f,
             axle_to_base=offset, odom_frame='odom', base_frame='base_link',
             get_parameter=lambda name: S(value=True),
             odom_pub=S(publish=odometry.append),
             tf_broadcaster=S(sendTransform=transforms.append),
             slip_pub=S(publish=lambda m: None), diagnostics_pub=S(publish=lambda m: None))
    ZUPTNode._publish_state(node, S(sec=1, nanosec=0))
    return odometry[0], transforms[0], f


def main():
    d = .31984
    for angle in (0., math.pi/2, math.pi, -math.pi/2, .63):
        odom, tf, f = publish(d, angle)
        p, t = odom.pose.pose.position, tf.transform.translation
        expected = [2+d*math.cos(angle), -1+d*math.sin(angle)]
        assert np.allclose([p.x,p.y], expected)
        assert np.allclose([t.x,t.y], expected)
        assert np.allclose([p.x-d*math.cos(angle),p.y-d*math.sin(angle)], [2.,-1.])
        assert f.x == 2. and f.y == -1., 'published shift must not mutate the axle integrator'
        assert math.isclose(odom.twist.twist.linear.y, d*.8)
        eps=1e-6
        velocity=(d*np.array([math.cos(angle+eps*.8),math.sin(angle+eps*.8)])-
                  d*np.array([math.cos(angle),math.sin(angle)]))/eps
        lateral=-math.sin(angle)*velocity[0]+math.cos(angle)*velocity[1]
        assert math.isclose(lateral,odom.twist.twist.linear.y,abs_tol=1e-6)
        covariance=np.array(odom.pose.covariance).reshape(6,6)
        jacobian=np.eye(6);jacobian[0,5]=-d*math.sin(angle);jacobian[1,5]=d*math.cos(angle)
        expected_cov=jacobian@np.diag([.05,.05,1e6,1e6,1e6,.05])@jacobian.T
        assert np.allclose(covariance,expected_cov)
        assert np.allclose(covariance,covariance.T)
        assert np.linalg.eigvalsh(covariance).min() >= 0
        twist=np.array(odom.twist.covariance).reshape(6,6)
        assert np.allclose(twist,twist.T) and np.linalg.eigvalsh(twist).min() >= 0
        assert math.isclose(twist[1,5],d*.02)
    legacy,_,_=publish(0.,.63)
    assert legacy.pose.pose.position.x==2. and legacy.pose.pose.position.y==-1.
    assert legacy.twist.twist.linear.y==0.
    print('PASS: actual ZUPT output and TF share the offset base pose; axle integrator unchanged;')
    print('      spin lateral velocity, transformed symmetric PSD covariances, legacy zero offset.')


if __name__=='__main__': main()
