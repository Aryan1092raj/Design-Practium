#!/usr/bin/env python3
"""Read actual Mode 3 localization parameters before enabling voice goals."""
import math
import time

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters


def read(node, name, keys):
    client = node.create_client(GetParameters, name + '/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError(name + ' parameter service unavailable')
        future = client.call_async(GetParameters.Request(names=keys))
        deadline = time.monotonic() + 5.
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not future.done() or future.result() is None:
            raise RuntimeError(name + ' parameter query timed out')
        result = {}
        for key, value in zip(keys, future.result().values):
            field = {ParameterType.PARAMETER_BOOL: 'bool_value',
                     ParameterType.PARAMETER_DOUBLE: 'double_value',
                     ParameterType.PARAMETER_STRING: 'string_value'}.get(value.type)
            if field is None:
                raise RuntimeError(name + ': missing/invalid parameter ' + key)
            result[key] = getattr(value, field)
        if len(result) != len(keys):
            raise RuntimeError(name + ': incomplete parameter reply')
        return result
    finally:
        node.destroy_client(client)


def main():
    rclpy.init()
    node = rclpy.create_node('mode3_localization_check')
    try:
        zupt = read(node, '/zupt_odometry', ['axle_to_base', 'publish_tf', 'gyro_blend_alpha'])
        amcl = read(node, '/amcl', ['robot_model_type'])
        if (not math.isclose(zupt['axle_to_base'], .31984, abs_tol=1e-6)
                or not zupt['publish_tf'] or zupt['gyro_blend_alpha'] != 1.0):
            raise RuntimeError('ZUPT must publish the 0.31984 m shifted base TF with wheel heading: ' + repr(zupt))
        if amcl['robot_model_type'] != 'nav2_amcl::OmniMotionModel':
            raise RuntimeError('AMCL must support base_link lateral swing: ' + repr(amcl))
        print('PASS Mode 3 localization: ZUPT axle offset 0.31984 m, TF enabled, wheel heading; AMCL OmniMotionModel')
    except (RuntimeError, KeyError, TypeError) as exc:
        print('FAIL Mode 3 localization: ' + str(exc))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__': main()
