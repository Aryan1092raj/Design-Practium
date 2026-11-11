#!/usr/bin/env python3
"""Check the live front RGB, aligned depth and intrinsics used by Mode 3."""
import sys
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener


def main():
    sim='--sim' in sys.argv
    rclpy.init(args=[])
    node=rclpy.create_node('mode3_camera_check', parameter_overrides=[Parameter('use_sim_time',value=sim)])
    frames={}
    tf=Buffer();listener=TransformListener(tf,node)
    for key,kind,topic in [('rgb',Image,'color/image_raw'),('depth',Image,'aligned_depth_to_color/image_raw'),
                           ('info',CameraInfo,'color/camera_info')]:
        node.create_subscription(kind,'/camera/'+topic,lambda m,k=key:frames.__setitem__(k,m),qos_profile_sensor_data)
    deadline=time.monotonic()+15
    reason='front RGB, aligned depth or camera intrinsics are missing'
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if len(frames)!=3:continue
            rgb,depth,info=(frames[k] for k in ['rgb','depth','info'])
            now=node.get_clock().now()
            stamps=[Time.from_msg(m.header.stamp) for m in [rgb,depth]]
            if any(not 0 <= (now-stamp).nanoseconds <= 1e9 for stamp in stamps):
                reason='front RGB/depth is stale or future-dated';continue
            if abs((stamps[0]-stamps[1]).nanoseconds)>1e8:
                reason='front RGB/depth stamps differ by more than 0.10 s';continue
            if rgb.encoding not in ['rgb8','bgr8'] or depth.encoding not in ['16UC1','32FC1']:
                reason=f'unsupported front image encodings {rgb.encoding}/{depth.encoding}';continue
            if (rgb.width<=0 or rgb.height<=0 or (rgb.width,rgb.height)!=(depth.width,depth.height)
                    or (info.width,info.height)!=(rgb.width,rgb.height) or info.k[0]<=0 or info.k[4]<=0):
                reason='front depth is not aligned to RGB dimensions/intrinsics';continue
            try:
                tf.lookup_transform('base_link',depth.header.frame_id,Time())
                values=np.frombuffer(depth.data,np.uint16 if depth.encoding=='16UC1' else np.float32)
                if depth.encoding=='16UC1':values=values.astype(float)/1000.
                if not np.any(np.isfinite(values[::32])&(values[::32]>.2)):
                    reason='front depth has no valid ranges';continue
            except Exception as exc:
                reason='front camera TF/depth: '+str(exc);continue
            print(f'PASS front camera stream: {rgb.width}x{rgb.height} {rgb.encoding}, aligned {depth.encoding}, fresh paired stamps, intrinsics and base TF')
            return
        print('FAIL front camera stream: '+reason)
        raise SystemExit(1)
    finally:
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
