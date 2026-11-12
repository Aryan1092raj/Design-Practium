#!/usr/bin/env python3
"""Stationary Mode 3 inspection. No microphone, goals, spins or wheel commands."""
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from vlm_nav import CAMERAS, SPIN_CLEAR, SPOT_MARGIN, VoiceNav


class InspectObject(VoiceNav):
    def on_text(self, text):
        self.get_logger().info('Inspection ignores voice/transcript commands.')

    def send(self, *args, **kwargs):
        raise RuntimeError('Inspection cannot submit actions')

    drive = send
    turn = send
    go_to = send

    def inspect(self, target):
        self.get_logger().info(f'Stationary inspection of {target}; no movement actions are submitted.')
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if len(self.frames[CAMERAS[0]]) >= 3 and self.costmap is not None and self.pose() is not None:
                break
            time.sleep(.1)
        else:
            raise RuntimeError('camera, costmap or pose unavailable')
        for attempt in range(3):
            if not rclpy.ok():
                return
            self.reset_search_state()
            hit = self.ground(CAMERAS[0], target)
            if hit:
                goal, distance, bearing = hit
                self.get_logger().info(f'Look {attempt + 1}: target distance {distance:.2f} m; approach={goal}')
                pose = self.pose()
                if pose is not None:
                    self._last_sighting = {'target': target, 'object': [pose[0] + distance * math.cos(bearing),
                                                                      pose[1] + distance * math.sin(bearing)]}
            else:
                self.get_logger().info(f'Look {attempt + 1}: no usable depth estimate; target recognized={self._target_seen}')
            self.dump_costmap(f'stationary inspection of {target}, look {attempt + 1}')
            time.sleep(.2)
        pose = self.pose()
        c = self.coverage(pose)
        if c is not None:
            paths = c['path'][tuple(c['cells'].T)]
            self.get_logger().info(f'{np.count_nonzero(np.isfinite(paths))} reachable search viewpoints; '
                                   f'axle viewpoint clearance {SPIN_CLEAR + SPOT_MARGIN:.2f} m; current spin room={self.clear(pose[:2], pose[2], SPIN_CLEAR)}')
        self.get_logger().info('Inspection complete. No navigation or spin goals were submitted.')


def main():
    # Keep the executor alive until the main thread handles Ctrl+C and cleans up.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = InspectObject()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin)
    worker.start()
    try:
        target = node.declare_parameter('target', 'chair').value
        node.inspect(target)
    except KeyboardInterrupt:
        node.get_logger().info('Inspection stopped; no movement actions were submitted.')
    except RuntimeError as exc:
        node.get_logger().error(f'Inspection failed: {exc}')
        raise SystemExit(1)
    finally:
        executor.shutdown()
        worker.join()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
