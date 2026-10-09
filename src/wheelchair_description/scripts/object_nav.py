#!/usr/bin/env python3
"""Zero-shot object search in an unmapped building: "take me to the sofa" -> find it, drive there.

Needs slam_toolbox (live /map) and Nav2 without AMCL (gazebo_sim.launch.py explore:=true), plus
scripts/objnav_vlm_server.sh. Reuses voice_nav.py for the mic, "stop" and spoken replies; saved
places are switched off, because they belong to a different map.

Per command: announce, then look with the three RGB-D cameras (YOLOE with the spoken word as its
only class) and explore frontiers of the live map until two viewpoints agree on the object.
Qwen3-VL confirms it on the full frame, and the chair drives to a free spot 1.5-2.4 m short of it
and turns to face it. Nav2 does all the driving.

    ros2 run wheelchair_description object_nav.py --ros-args -p use_sim_time:=true \
        -p detector:=$HOME/models/yoloe-26s-seg.pt
    ros2 topic pub --once /voice/transcript std_msgs/String "{data: 'take me to the sofa'}"
"""
import math
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from faster_whisper import WhisperModel
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from objnav_frontiers import find_frontiers, pick
from objnav_geometry import (STANDOFF, Grid, Hit, Hits, box_mask, mask_centre, object_depth,
                             pixel_to_optical, quat_to_matrix, standoff)
from objnav_models import Detector, Verifier
from voice_nav import VoiceNav, listen, wait

CAMERAS = ["camera", "mapping_camera", "right_camera"]  # front, left, right (RealSense names)
SEARCH_TIME = 300.0  # s; the passenger is seated, so a search gives up after 5 minutes
MAX_TRIPS = 20  # frontier goals per command
LOOK_PERIOD = 0.5  # s between looks while a goal runs; also the stop-check period
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def to_rgb(msg):
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    a = a.reshape(msg.height, msg.width, 3)
    return a if msg.encoding == "rgb8" else a[:, :, ::-1]


def to_metres(msg):
    """32FC1 metres (Gazebo) or 16UC1 millimetres (RealSense)."""
    if msg.encoding == "16UC1":
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, -1)[:, :msg.width] / 1000.0
    return np.frombuffer(msg.data, np.float32).reshape(msg.height, -1)[:, :msg.width]


def to_grid(msg):
    i = msg.info
    return Grid(np.array(msg.data, np.int8).reshape(i.height, i.width), i.resolution,
                i.origin.position.x, i.origin.position.y)


def nav_goal(x, y, yaw):
    g = NavigateToPose.Goal()
    g.pose = PoseStamped()
    g.pose.header.frame_id = "map"
    g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
    g.pose.pose.orientation.z = math.sin(yaw / 2)
    g.pose.pose.orientation.w = math.cos(yaw / 2)
    return g


def spin_goal(angle):
    g = Spin.Goal()
    g.target_yaw = float(angle)
    g.time_allowance.sec = 30
    return g


class ObjectNav(VoiceNav):
    def __init__(self):
        super().__init__("object_nav")
        self.places = {}  # saved places belong to a saved map; here the map is built live
        self.detector = Detector(self.declare_parameter("detector", "yoloe-26s-seg.pt").value,
                                 self.declare_parameter("device", "cuda:0").value)
        self.verifier = Verifier(self.declare_parameter(
            "vlm_url", "http://127.0.0.1:8091/v1/chat/completions").value)
        self.frames = {c: {} for c in CAMERAS}
        for cam in CAMERAS:
            for key, typ, topic in (("rgb", Image, "color/image_raw"),
                                    ("depth", Image, "aligned_depth_to_color/image_raw"),
                                    ("info", CameraInfo, "color/camera_info")):
                self.create_subscription(typ, f"/{cam}/{topic}",
                                         lambda m, c=cam, k=key: self.frames[c].__setitem__(k, m),
                                         qos_profile_sensor_data)
        self.map = self.costmap = None
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m), LATCHED)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 lambda m: setattr(self, "costmap", m), LATCHED)
        self.deadline, self.travelled = 0.0, 0.0

    def on_unknown(self, target, names):
        if self.busy:
            self.say_async("I am already moving. Say stop first.")
            return
        self.busy = True
        self.cancelled.clear()
        threading.Thread(target=self.search, args=(target,), daemon=True).start()

    # ---- one command, start to finish ---------------------------------------------------
    def search(self, target):
        start, found = time.monotonic(), False
        self.deadline, self.travelled = start + SEARCH_TIME, 0.0
        try:
            if not self.verifier.ping():
                self.say("My camera check is not running, so I can't search.")
                return
            self.say(f"Looking for the {target}. Say stop to cancel.")
            if self.cancelled.wait(2.0):
                return
            if not self.wait_ready():
                if not self.cancelled.is_set():
                    self.say("My map or cameras are not ready yet. Please ask again in a moment.")
                return
            self.detector.set_target(target)
            hits, tried, trips, boots = Hits(), [], 0, 0
            self.look(hits)
            if hits.ready() is None:  # the cameras cover ~259 deg, so look behind once
                self.run_and_look(self.spin_client, spin_goal(math.pi), hits)
            while not self.cancelled.is_set():
                obj = self.confirmed(hits, target)
                if obj is not None:
                    found = self.approach(target, obj)
                    return
                frontier = None
                if trips < MAX_TRIPS and time.monotonic() < self.deadline:
                    frontier = self.next_frontier(tried)
                    if frontier is None and boots < 2:
                        boots += 1
                        frontier = self.bootstrap(hits, tried)
                        if frontier is None and hits.ready() is not None:
                            continue  # the bootstrap saw the object; check it first
                if frontier is None:
                    if not self.cancelled.is_set():
                        self.say(f"I could not find the {target}.")
                    return
                trips += 1
                chair = self.chair_pose() or (frontier.x, frontier.y, 0.0)
                yaw = math.atan2(frontier.y - chair[1], frontier.x - chair[0])
                if not self.run_and_look(self.nav, nav_goal(frontier.x, frontier.y, yaw), hits):
                    tried.append((frontier.x, frontier.y))  # reached or failed; not if a hit cut it short
        finally:
            self.get_logger().info(f"SEARCH_RESULT target={target} found={found} "
                                   f"time={time.monotonic() - start:.0f}s path={self.travelled:.1f}m")
            self.busy = False

    def bootstrap(self, hits, tried):
        """No frontier yet, as right after start-up when slam_toolbox has only the chair's own
        footprint. Spin in four 90 degree steps (one 180 degree spin can abort near furniture),
        then creep 0.6 m ahead, until the growing map offers a frontier."""
        for _ in range(4):
            if self.run_and_look(self.spin_client, spin_goal(math.pi / 2), hits) \
                    or hits.ready() is not None:
                return None
            f = self.pause_for_map(tried)
            if f is not None:
                return f
        chair = self.chair_pose()
        if chair is not None:
            ahead = nav_goal(chair[0] + 0.6 * math.cos(chair[2]),
                             chair[1] + 0.6 * math.sin(chair[2]), chair[2])
            if self.run_and_look(self.nav, ahead, hits) or hits.ready() is not None:
                return None
            return self.pause_for_map(tried)
        return None

    def pause_for_map(self, tried):
        time.sleep(1.5)  # slam_toolbox publishes /map about once a second
        return None if self.cancelled.is_set() else self.next_frontier(tried)

    def wait_ready(self, timeout=30.0):
        """True once the live map, the costmap, the chair pose and one complete camera exist.
        Right after start-up (sim or chair) a command can arrive before any of them do."""
        end = time.monotonic() + timeout
        while time.monotonic() < end and not self.cancelled.is_set():
            if (self.map is not None and self.costmap is not None
                    and self.chair_pose() is not None
                    and any(len(self.frames[c]) == 3 for c in CAMERAS)):
                return True
            time.sleep(0.5)
        return False

    def confirmed(self, hits, target):
        """Map position of a cluster the VLM agrees is the target, or None."""
        while not self.cancelled.is_set():
            ready = hits.ready()
            if ready is None:
                return None
            centre, newest = ready
            rgb, box = newest.frame
            ok = self.verifier.check(rgb, box, target)
            self.get_logger().info(f"VLM check {target} at ({centre[0]:.2f}, {centre[1]:.2f}): "
                                   f"{'yes' if ok else 'no'}")
            if ok:
                return centre
            hits.reject(centre)
        return None

    def approach(self, target, obj):
        chair = self.chair_pose()
        if chair is None or self.costmap is None:
            self.say(f"I see the {target}, but I lost track of where I am.")
            return False
        dist = math.hypot(obj[0] - chair[0], obj[1] - chair[1])
        if dist < STANDOFF[0] + 0.3:
            self.face(math.atan2(obj[1] - chair[1], obj[0] - chair[0]))
            self.say(f"I am next to the {target}.")
            return True
        goal = standoff(obj, chair[:2], to_grid(self.costmap))
        if goal is None:
            self.say(f"I see the {target}, but I can't find a free spot next to it.")
            return False
        self.say(f"I found the {target}, {dist:.0f} metres away. Going there. Say stop to cancel.")
        if self.cancelled.wait(2.0):
            return False
        ok = self.send(self.nav, nav_goal(*goal)) == GoalStatus.STATUS_SUCCEEDED
        if ok:
            self.face(goal[2])
        if self.cancelled.is_set():
            return False
        self.say(f"Arrived at the {target}." if ok else
                 f"I could not reach the {target}. I may be stuck. Please help me.")
        return ok

    def run_and_look(self, client, goal, hits):
        """Run one Nav2 action while the cameras keep looking. Returns True if it was cut short
        because two viewpoints agreed on the target; stop and the time cap also cancel it."""
        if self.cancelled.is_set() or not client.wait_for_server(timeout_sec=5.0):
            return False
        handle = wait(client.send_goal_async(goal))
        if not handle.accepted:
            return False
        self.goal_handle = handle
        result, interrupted, last = handle.get_result_async(), False, self.chair_pose()
        while not result.done():
            if self.cancelled.is_set() or time.monotonic() > self.deadline:
                handle.cancel_goal_async()
                break
            self.look(hits)
            if hits.ready() is not None:
                handle.cancel_goal_async()
                interrupted = True
                break
            now = self.chair_pose()
            if now and last:
                self.travelled += math.hypot(now[0] - last[0], now[1] - last[1])
            last = now or last
            time.sleep(LOOK_PERIOD)
        wait(result)
        self.goal_handle = None
        return interrupted

    # ---- perception ---------------------------------------------------------------------
    def look(self, hits):
        view = self.chair_pose()
        if view is None:
            return
        for cam in CAMERAS:
            f = dict(self.frames[cam])  # snapshot; callbacks keep replacing entries
            if len(f) < 3:
                continue
            rgb, depth = to_rgb(f["rgb"]), to_metres(f["depth"])
            if rgb.shape[:2] != depth.shape:
                self.get_logger().warn(f"{cam}: aligned depth {depth.shape} != colour "
                                       f"{rgb.shape[:2]}", throttle_duration_sec=10.0)
                continue
            dets = self.detector.detect(rgb)
            if not dets:
                continue
            try:
                tf = self.tf.lookup_transform("map", f["depth"].header.frame_id,
                                              Time.from_msg(f["depth"].header.stamp),
                                              timeout=Duration(seconds=0.1))
            except Exception as e:  # tf2 raises several unrelated exception types
                self.get_logger().warn(f"tf {cam}: {e}", throttle_duration_sec=5.0)
                continue
            q, p = tf.transform.rotation, tf.transform.translation
            r, t = quat_to_matrix(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])
            for box, score, mask in dets:
                m = mask if mask is not None and mask.shape == depth.shape else box_mask(depth.shape, box)
                z = object_depth(depth, m)
                if z is None:
                    continue
                u, v = mask_centre(m)
                xy = (r @ pixel_to_optical(u, v, z, f["info"].k) + t)[:2]
                hits.add(Hit(xy, view, cam, (rgb, box)))
                self.get_logger().info(f"{cam}: box {[round(b) for b in box]} score {score:.2f} "
                                       f"at map ({xy[0]:.2f}, {xy[1]:.2f}), {z:.1f} m")

    def chair_pose(self):
        try:
            tf = self.tf.lookup_transform("map", "base_link", Time())
        except Exception as e:  # tf2 raises several unrelated exception types
            self.get_logger().warn(f"tf: {e}", throttle_duration_sec=5.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, 2 * math.atan2(q.z, q.w)

    def next_frontier(self, tried):
        chair = self.chair_pose()
        if chair is None or self.map is None or self.costmap is None:
            return None
        return pick(find_frontiers(to_grid(self.map), to_grid(self.costmap)), chair[:2], tried)


def main():
    rclpy.init()
    node = ObjectNav()
    stt = WhisperModel("base.en", device="cpu", compute_type="int8")
    threading.Thread(target=listen, args=(node, stt), daemon=True).start()
    node.say_async("Object search ready.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
