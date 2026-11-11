#!/usr/bin/env python3
"""Voice + camera navigation with no saved places: "take me to the table" -> Nav2 goal.

Listens on the laptop mic and turns speech into text with faster-whisper. The target is looked
for in the front RGB-D camera with a VLM (Qwen3-VL 4B Instruct); if it is not in view, the chair turns
in place 60 degrees at a time and looks again. If the full turn misses it, the VLM gets all six
views in one call and picks the one most likely to lead to the target (a doorway toward a
kitchen when asked for a fridge); the chair drives to the reachable spot that shows the most
floor it has not looked at yet, preferring that direction, and scans again, until the floor is
covered or 10 minutes have passed. The bounding box plus the aligned depth give the object's position in the map, and
the goal is a free spot within 1 m in front of it.
Nav2 does all the driving; saying "stop" cancels the goal. Replies are printed and published on
/voice/status (the real chair has no speaker).

Needs nav launched with front_color:=true, src/wheelchair_description/scripts/vlm_server.sh running, and runs inside
.venv-voice:
    ros2 run wheelchair_description vlm_nav.py
Text can also be injected for testing:
    ros2 topic pub --once /voice/transcript std_msgs/String "{data: 'go to the bed'}"
"""
import base64
import json
import math
import os
import re
import threading
import time

import cv2
import numpy as np
import rclpy
import requests
from action_msgs.msg import GoalStatus
from faster_whisper import WhisperModel
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from amcl_settle import is_stopped
from voice_nav import mic_chunks, nav2_gone

VLM_URL = "http://127.0.0.1:8091/v1/chat/completions"  # src/wheelchair_description/scripts/vlm_server.sh (Qwen3-VL 4B Instruct)
# Image size sent to the VLM; about 150 image tokens. Qwen3-VL answers boxes in 0-1000 relative
# coordinates, which vlm() scales to this size.
VLM_W, VLM_H = 448, 336
# Generic household labels for the box check; the rider's target is added when missing, so the
# search stays zero-shot. These are not map labels.
CLASSES = ["bed", "sofa", "chair", "table", "desk", "wardrobe", "cabinet", "shelf", "refrigerator",
           "oven", "sink", "counter", "door", "window", "television", "picture", "piano",
           "dumbbell", "exercise bench", "exercise ball", "plant", "lamp", "bin", "wall", "floor",
           "other"]
# Labels the box check may not tell apart. The check exists to drop boxes on unrelated things
# (portraits, doors), not to split near-synonyms: in m3_z and m3_aa the real wardrobe, seen
# side-on as a plain wooden slab, was labelled "cabinet" 5 times and never reached.
KIN = {"wardrobe": {"cabinet"}, "cabinet": {"wardrobe"}, "table": {"desk"}, "desk": {"table"}}
CAMERAS = ["camera"]  # front D455 only; the side cameras run depth-only to save USB bandwidth
# Chair centre to the object's near face, nearest first. The chair's front is 0.45 m ahead of its
# centre, so with Nav2's 0.15 m goal tolerance the front stops within 1 m of the object. 0.7-0.85
# alone left no free spot in front of the sim sofa (coffee table, 0.65 m inflation).
STANDOFF = (0.8, 1.0, 1.2, 1.3)
# Looks per scan, turning 360/LOOKS degrees between them. Six, so views overlap: with four, an
# object cut by the frame edge got the wrong label (m3_z: the wardrobe, half in view, "cabinet").
LOOKS = 6
# s. A scan plus the drive to it took about 50 s in sim, so 180 s allowed three spots: m3_t and
# m3_u ran out one room short of the bed. The search also ends when no unseen floor is left.
SEARCH_TIME = 600.0
MAX_RANGE = 4.0  # beyond this the depth is not trusted: close in and look again
APPROACHES = 4  # looks per command: the first, plus re-looks after driving toward a far sighting
# The front camera (1.34 m up, 64 degree vertical FOV) sees the floor only beyond ~2.2 m, so a close-in
# drive stops this far from a sighting; closer, a low object (a dumbbell) drops out of the frame.
MIN_SIGHT = 2.5
FREE_COST = 50  # global costmap cells below this count as free for a goal
# The chair turns in place about its drive axle, 0.32 m behind base_link (urdf wheel joints;
# Gazebo poses after four spins all sat within 6 cm of one axle point). Goals keep lethal cells
# TURN_CLEAR from the axle: the padded front corner (0.80, 0.38) from it sweeps 0.89 m, plus
# Nav2's 0.15 m goal tolerance.
AXLE_BACK = 0.32
TURN_CLEAR = 1.04
STOP = re.compile(r"\b(stop|halt|cancel)\b")
GO = re.compile(r"\b(?:go|take me|bring me|drive|move|head|navigate)\b.*?\b(?:to|towards?|near)\s+"
                r"(?:the |a |an |my )?(?P<t>[a-z ]+)")


def wait(future):
    while not future.done():
        time.sleep(0.05)
    return future.result()


def to_rgb(msg):
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    a = a.reshape(msg.height, msg.width, 3)
    return a if msg.encoding == "rgb8" else a[:, :, ::-1]


def to_metres(msg):
    """32FC1 metres (Gazebo) or 16UC1 millimetres (RealSense)."""
    if msg.encoding == "16UC1":
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, -1)[:, :msg.width] / 1000.0
    return np.frombuffer(msg.data, np.float32).reshape(msg.height, -1)[:, :msg.width]


def to_matrix(tf):
    q, t = tf.transform.rotation, tf.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return r, np.array([t.x, t.y, t.z])


def turn_obstacles(grid, res, cleared=None):
    """Only small lethal blobs seen as free in three fresh scans may be discarded.
    Size alone cannot distinguish a one-cell pole from a stale lidar mark."""
    solid = (grid < 0) | (grid == 100)
    if cleared is not None:
        _, labels, stats, _ = cv2.connectedComponentsWithStats((grid == 100).astype(np.uint8), 8)
        small = np.isin(labels, 1 + np.flatnonzero(stats[1:, cv2.CC_STAT_AREA] * res * res <= 0.0016))
        solid &= ~(small & cleared)
    return solid


def obstacle_mask(node, m, grid=None):
    revision = getattr(node, "_noise_revision", 0)
    cached = getattr(node, "_obstacles", None)
    if grid is None and cached is not None and cached[0] is m and cached[1] == revision:
        return cached[2]
    if grid is None:
        grid = np.asarray(m.data, np.int16).reshape(m.info.height, m.info.width)
    counts = getattr(node, "_clear_counts", None)
    solid = turn_obstacles(grid, m.info.resolution, counts >= 3 if counts is not None else None)
    node._obstacles = m, revision, solid
    return solid


class VoiceNav(Node):
    def __init__(self):
        super().__init__("vlm_nav")
        self.frames = {c: {} for c in CAMERAS}
        for cam in CAMERAS:
            for key, typ, topic in (("rgb", Image, "color/image_raw"),
                                    ("depth", Image, "aligned_depth_to_color/image_raw"),
                                    ("info", CameraInfo, "color/camera_info")):
                self.create_subscription(typ, f"/{cam}/{topic}",
                                         lambda m, c=cam, k=key: self.frames[c].__setitem__(k, m),
                                         qos_profile_sensor_data)
        self.costmap = None
        self._clear_counts = None
        self._clear_key = None
        self._clear_stamp = None
        self._noise_revision = 0
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 self.on_costmap,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, "/voice/transcript", lambda m: self.on_text(m.data), 10)
        self.status = self.create_publisher(String, "/voice/status", 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        scan_topic = self.declare_parameter("scan_topic", "/scan" if self.get_parameter(
            "use_sim_time").value else "/scan_fused").value
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.spin_client = ActionClient(self, Spin, "spin")
        self.goal_handle = None
        self.goal_response = None
        self.goal_lock = threading.Lock()
        self.command_deadline = None
        self.stopped_since = None
        cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/wc_control/cmd_vel").value
        self.create_subscription(TwistStamped, cmd_vel_topic, self.on_cmd_vel, qos_profile_sensor_data)
        self.busy = False
        self.cancelled = threading.Event()

    def now(self):
        """Node clock in seconds. Search limits count chair time: Gazebo ran at 0.6x real time in
        m3_t, so a wall-clock 3 minutes gave the chair under 2."""
        return self.get_clock().now().nanoseconds / 1e9

    def on_cmd_vel(self, msg):
        if not is_stopped(msg.twist):
            self.stopped_since = None
        elif self.stopped_since is None:
            self.stopped_since = self.now()

    def wait_localization(self):
        """At least 3.5 s of stopped command on the node clock; repeated reads add no delay."""
        while not self.cancelled.is_set():
            now = self.now()
            if self.command_deadline is not None and now >= self.command_deadline:
                return False
            if self.stopped_since is not None and now - self.stopped_since >= 3.5:
                return True
            self.cancelled.wait(0.05)
        return False

    # ---- speech out / commands in -------------------------------------------------------
    def say(self, text):
        if "can't get closer" in text or "can't find a free spot" in text:
            self.dump_costmap(text)
        self.get_logger().info(text)
        self.status.publish(String(data=text))

    def dump_costmap(self, reason):
        dump, m = os.environ.get("VLM_NAV_DUMP"), self.costmap
        if not dump or m is None:
            return
        try:
            os.makedirs(dump, exist_ok=True)
            stem = os.path.join(dump, f"costmap_{int(self.now())}_{time.time_ns()}")
            np.save(stem + ".npy", np.asarray(m.data, np.int16).reshape(m.info.height, m.info.width))
            if self._clear_counts is not None:
                np.savez_compressed(stem + "_turn.npz", clear_counts=self._clear_counts)
            with open(stem + ".json", "w") as f:
                json.dump({"reason": reason, "time": self.now(), "pose": self.pose(),
                           "resolution": m.info.resolution,
                           "origin": [m.info.origin.position.x, m.info.origin.position.y],
                           "sighting": getattr(self, "_last_sighting", None)}, f)
        except (OSError, ValueError) as e:
            self.get_logger().warn(f"costmap dump: {e}")

    def on_costmap(self, m):
        res, h, w = m.info.resolution, m.info.height, m.info.width
        key = (res, h, w, m.info.origin.position.x, m.info.origin.position.y)
        grid = np.asarray(m.data, np.int16).reshape(h, w)
        if self._clear_key != key:
            self._clear_counts = np.zeros((h, w), np.uint8)
            self._clear_key, self._clear_stamp = key, None
        else:
            previous = np.asarray(self.costmap.data, np.int16).reshape(h, w)
            self._clear_counts[(grid == 100) & (previous != 100)] = 0  # a new pole is unconfirmed
        _, labels, stats, _ = cv2.connectedComponentsWithStats((grid == 100).astype(np.uint8), 8)
        small = np.isin(labels, 1 + np.flatnonzero(stats[1:, cv2.CC_STAT_AREA] * res * res <= 0.0016))
        self._tiny_cells = np.column_stack(np.nonzero(small))
        self.costmap = m

    def on_scan(self, msg):
        """Clear stale tiny marks only where fresh rays pass through, not behind an occluder.
        A return within 10 cm, a beam gap, missing TF or an unseen pole keeps the obstacle."""
        if self.costmap is None or self._clear_counts is None or not len(self._tiny_cells):
            return
        stamp = Time.from_msg(msg.header.stamp)
        if self._clear_stamp is not None and stamp.nanoseconds <= self._clear_stamp:
            return
        if abs(self.get_clock().now().nanoseconds - stamp.nanoseconds) > 1e9:
            return
        try:
            r, t = to_matrix(self.tf.lookup_transform("map", msg.header.frame_id, stamp))
        except Exception:  # no transform means no evidence that a pole's cell is empty
            return
        self._clear_stamp = stamp.nanoseconds
        res, h, w, ox, oy = self._clear_key
        jj, ii = self._tiny_cells.T
        xy = np.column_stack((ox + (ii + 0.5) * res, oy + (jj + 0.5) * res)) - t[:2]
        yaw = math.atan2(r[1, 0], r[0, 0])
        x, y = xy[:, 0] * math.cos(yaw) + xy[:, 1] * math.sin(yaw), -xy[:, 0] * math.sin(yaw) + xy[:, 1] * math.cos(yaw)
        angle, distance = np.arctan2(y, x), np.hypot(x, y)
        ranges = np.asarray(msg.ranges)
        if len(ranges) < 3 or msg.angle_increment <= 0:
            return
        beam = np.rint((angle - msg.angle_min) / msg.angle_increment).astype(int)
        inside = (beam > 0) & (beam < len(ranges) - 1)
        beam = np.clip(beam, 1, len(ranges) - 2)
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges <= msg.range_max)
        ranges = np.where(valid, ranges, 0.0)
        reach = np.minimum.reduce((ranges[beam - 1], ranges[beam], ranges[beam + 1]))
        lateral = distance * np.abs(np.sin(angle - msg.angle_min - beam * msg.angle_increment))
        observed = inside & (lateral <= res / 2) & (distance > msg.range_min) & (reach > 0)
        free = observed & (distance + 0.10 < reach)
        blocked = inside & (reach > 0) & (reach <= distance + 0.10)
        old = self._clear_counts[jj, ii]
        new = np.where(blocked, 0, np.where(free, np.minimum(old + 1, 3), old))
        self._clear_counts[jj, ii] = new
        if np.any((old >= 3) != (new >= 3)):
            self._noise_revision += 1

    def on_text(self, text):
        t = text.lower()
        if STOP.search(t):
            self.cancelled.set()
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            threading.Thread(target=self.say, args=("Stopping.",), daemon=True).start()
            return
        m = GO.search(t)
        if not m:
            return  # ordinary speech near the chair is ignored
        target = re.sub(r"\b(please|now|thanks?)\b", "", m["t"]).strip()
        if not target:
            return
        if self.busy:
            threading.Thread(target=self.say, args=("I am already moving. Say stop first.",),
                             daemon=True).start()
            return
        self.busy = True
        self.cancelled.clear()
        threading.Thread(target=self.run, args=(target,), daemon=True).start()

    # ---- one command, start to finish ---------------------------------------------------
    def run(self, target):
        deadline = time.time() + SEARCH_TIME
        try:
            for attempt in range(APPROACHES):
                # a re-look after driving toward an object already seen is not a search, so the
                # 3-minute limit does not cut it short
                found = self.find(target, deadline if attempt == 0 else time.time() + 60)
                if self.cancelled.is_set():
                    return
                if found is None:
                    self.say(f"Sorry, the {target} was not found.")
                    return
                goal, rng, bearing = found
                if rng <= MAX_RANGE or attempt == APPROACHES - 1:
                    if goal is None:
                        self.say(f"I see the {target}, but I can't find a free spot next to it.")
                    elif rng < STANDOFF[0] + 0.3:
                        self.say(f"I am already next to the {target}.")
                    else:
                        self.drive(target, *goal, final=True, announce=attempt == 0)
                    return
                # Far sightings: in sim, objects seen at 5-7 m were placed metres away from where
                # they are (a fridge outside the kitchen). Close in along the bearing, look again.
                # straight along the bearing when the line is open, else let Nav2 route to the
                # rough estimate (the fridge seen 12 m away from the bedroom, behind walls)
                # shortcut: the Nav2 fallback goes to the standoff, inside MIN_SIGHT, so a floor
                # object seen behind a wall can drop out of view; stop MIN_SIGHT short if that shows up
                step = self.explore_goal(bearing, min(EXPLORE_STEP, rng - MIN_SIGHT)) or goal
                if step is None:
                    self.say(f"I see the {target}, but I can't get closer to it.")
                    return
                if not self.drive(target, *step, final=False, announce=attempt == 0):
                    return  # drive() has said why
        finally:
            self.busy = False

    def drive(self, target, x, y, yaw, final, announce=True):
        if announce:
            self.say(f"Going to the {target}. Say stop to cancel.")
            if self.cancelled.wait(2.0):
                return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        ok = self.send(self.nav, goal) == GoalStatus.STATUS_SUCCEEDED
        if ok and final:
            self.face(yaw)
        if self.cancelled.is_set():
            return False
        if final or not ok:
            self.say(f"Arrived at the {target}." if ok else f"I could not reach the {target}.")
        return ok

    def face(self, yaw):
        """Nav2 ignores the goal heading (yaw_goal_tolerance 6.28 in the chair's params), so turn
        in place to face the target with the behaviour server's Spin."""
        try:
            q = self.tf.lookup_transform("map", "base_link", Time()).transform.rotation
        except Exception as e:  # tf2 raises several unrelated exception types
            self.get_logger().warn(f"tf: {e}")
            return
        turn = yaw - 2 * math.atan2(q.z, q.w)
        turn = math.atan2(math.sin(turn), math.cos(turn))
        if abs(turn) > 0.3 and not self.cancelled.is_set():
            spin = Spin.Goal()
            spin.target_yaw = turn
            spin.time_allowance.sec = 30
            self.send(self.spin_client, spin)

    def send(self, client, goal, deadline=None):
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"action server {client._action_name} not available")
            return None
        handle = wait(client.send_goal_async(goal))
        if not handle.accepted:
            return None
        self.goal_handle = handle
        if self.cancelled.is_set():
            handle.cancel_goal_async()
        result = handle.get_result_async()
        while not result.done() and not nav2_gone(client, handle, self):
            if deadline and time.time() > deadline:  # search time is up: stop exploring
                handle.cancel_goal_async()
                deadline = None
            time.sleep(0.2)
        self.goal_handle = None
        return result.result().status if result.done() else None

    # ---- semantic search ----------------------------------------------------------------
    def find(self, target, deadline):
        """Scan 360 degrees; if the target is not seen, let the VLM pick the view most likely to
        lead to it, drive a few metres that way and scan again, until the deadline."""
        scanned = []  # (x, y) of each full scan, so exploration doesn't drive back to one
        while time.time() < deadline and not self.cancelled.is_set():
            views = []  # (rgb, map yaw) of each look, for the direction question
            for turn in range(LOOKS):
                for cam in CAMERAS:
                    hit = self.ground(cam, target)
                    if hit:
                        self.get_logger().info(f"{target} seen by {cam} at {hit[1]:.1f} m")
                        return hit
                view = self.view()
                if view:
                    views.append(view)
                if turn < LOOKS - 1:
                    if self.cancelled.is_set() or time.time() > deadline:
                        return None
                    self.say(f"Turning to look for the {target}.")
                    if not self.turn(math.pi / 2):
                        return None  # Nav2 gone or spin rejected: rescanning the same view is pointless
            if len(views) < LOOKS or time.time() > deadline or self.cancelled.is_set():
                return None  # the VLM's photo numbers only mean a direction for a full scan
            seen, go = self.choose(views, target)
            if seen:  # the 4-view call saw it although the per-view boxes missed it: look again
                self.turn_to(views[seen - 1][1])
                hit = self.ground(CAMERAS[0], target)
                if hit:
                    return hit
            if not go:
                self.get_logger().info(f"VLM sees no promising direction for '{target}'")
                return None
            pose = self.pose()
            if pose:
                scanned.append(pose[:2])
            # the chosen direction first; if it is blocked within a metre or leads back to a
            # spot already scanned, the other views
            goal = next((g for g in (self.explore_goal(v[1], avoid=scanned) for v in
                                     [views[go - 1]] + views[:go - 1] + views[go:]) if g), None)
            if goal is None or time.time() > deadline:
                return None
            self.say(f"I don't see the {target} here. Looking further that way.")
            g = NavigateToPose.Goal()
            g.pose = PoseStamped()
            g.pose.header.frame_id = "map"
            g.pose.pose.position.x, g.pose.pose.position.y = goal[0], goal[1]
            g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(goal[2] / 2), math.cos(goal[2] / 2)
            if self.send(self.nav, g, deadline) != GoalStatus.STATUS_SUCCEEDED:
                return None
        return None

    def turn(self, angle):
        spin = Spin.Goal()
        spin.target_yaw = angle
        spin.time_allowance.sec = 30
        # only a finished turn counts: an aborted spin left all four "views" on one heading, and
        # the direction the VLM picked from them was not the one it saw
        ok = self.send(self.spin_client, spin) == GoalStatus.STATUS_SUCCEEDED
        time.sleep(1.0)  # fresh frames after the turn
        return ok

    def turn_to(self, yaw):
        pose = self.pose()
        if pose:
            self.turn(math.atan2(math.sin(yaw - pose[2]), math.cos(yaw - pose[2])))

    def pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_link", Time()).transform
        except Exception as e:  # tf2 raises several unrelated exception types
            self.get_logger().warn(f"tf: {e}")
            return None
        return t.translation.x, t.translation.y, 2 * math.atan2(t.rotation.z, t.rotation.w)

    def view(self):
        """The current front image and the chair's map yaw (spins never land on exactly 90)."""
        f, pose = self.frames[CAMERAS[0]], self.pose()
        if "rgb" not in f or pose is None:
            return None
        return cv2.resize(to_rgb(f["rgb"]), (VLM_W, VLM_H)), pose[2]

    def choose(self, views, target):
        """One VLM call with all views: is the target in one, and if not, which leads to it.
        Returns 1-based view numbers, 0 for none."""
        n = len(views)
        text = self.ask([v[0] for v in views],
                        f"These {n} photos were taken from one spot, turning left 90 degrees "
                        f"between photos (photo 1 ahead, photo 2 left, photo 3 behind, photo 4 "
                        f"right). I am looking for a {target}. Is a {target} in any photo? If "
                        f"not, which photo most likely leads to a {target}, for example a "
                        f"doorway or a room where a {target} is usually found? Reply JSON "
                        f'{{"found": <photo number or 0>, "go": <photo number or 0>}}; '
                        f"go is 0 if no photo looks promising.")
        nums = [re.search(rf'"{k}"\s*:\s*(\d+)', text) for k in ("found", "go")]
        seen, go = (int(m[1]) if m and 0 <= int(m[1]) <= n else 0 for m in nums)
        self.get_logger().info(f"VLM 4-view answer for '{target}': {text.strip()!r}")
        return seen, go

    def explore_goal(self, yaw, step=EXPLORE_STEP, avoid=()):
        """A free spot up to step metres along yaw, short of the first blocked cell, so the chair
        heads into the open space the view shows instead of a room behind a wall. Skips spots
        within REVISIT of the points in avoid."""
        pose = self.pose()
        if pose is None:
            return None
        for a in (yaw, yaw + 0.4, yaw - 0.4):
            at = lambda d: (pose[0] + d * math.cos(a), pose[1] + d * math.sin(a))
            k, n = 0, int(step / 0.05 + 1e-6)  # 5 cm samples; integers so the cap is exact
            while k < n and self.free(at((k + 1) * 0.05)):
                k += 1
            d = k * 0.05 - (BACKOFF if k < n else 0.0)
            # the goal itself on a zero-cost cell: goals in the inflation edge drew "collision
            # ahead" and a recovery in sim. It also needs room for the 4-view scan's spins: a
            # cost-0 goal 0.63 m from the dumbbell rack let the corners clip it and wedge the chair
            while d >= 1.0 and not (self.free(at(d), 1) and self.clear(at(d), a)):
                d -= 0.05
            if d >= 1.0 and all(math.dist(at(d), s) > REVISIT for s in avoid):
                return (*at(d), a)
        return None

    def ground(self, cam, target):
        f = self.frames[cam]
        if len(f) < 3:
            return None
        stamp = Time.from_msg(f["depth"].header.stamp)
        now = self.get_clock().now()
        if any((now - Time.from_msg(f[k].header.stamp)).nanoseconds > 1e9 for k in ("rgb", "depth")):
            self.get_logger().warn(f"{cam}: frames older than 1 s, camera stalled?")
            return None  # a stale frame projected from the current pose gives a wrong goal
        box = self.vlm(to_rgb(f["rgb"]), target)
        if box is None:
            return None
        depth = to_metres(f["depth"])
        h, w = depth.shape
        x1, y1, x2, y2 = (box[0] * w / VLM_W, box[1] * h / VLM_H,
                          box[2] * w / VLM_W, box[3] * h / VLM_H)
        try:
            r, t = to_matrix(self.tf.lookup_transform("map", f["depth"].header.frame_id, stamp,
                                                      Duration(seconds=0.5)))
            chair = self.tf.lookup_transform("map", "base_link", Time()).transform.translation
        except Exception as e:  # tf2 raises several unrelated exception types
            self.get_logger().warn(f"tf: {e}")
            return None
        # central half of the box, nearest quartile: skips background seen around the object
        u0, u1 = max(int(x1 + (x2 - x1) / 4), 0), min(int(x2 - (x2 - x1) / 4) + 1, w)
        v0, v1 = max(int(y1 + (y2 - y1) / 4), 0), min(int(y2 - (y2 - y1) / 4) + 1, h)
        patch = depth[v0:v1, u0:u1]
        vv, uu = np.mgrid[v0:v1, u0:u1]
        ok = np.isfinite(patch) & (patch > 0.2)
        if not ok.any():
            return None
        k = f["info"].k
        zs = patch[ok]
        # Drop the floor: the box of a low object (a sofa) includes the floor in front of it,
        # which pulled the estimate 1.5 m short in sim. Floor points are the lowest in the patch.
        height = (np.stack([(uu[ok] - k[2]) / k[0] * zs, (vv[ok] - k[5]) / k[4] * zs, zs], 1)
                  @ r.T + t)[:, 2]
        above = zs[height > height.min() + 0.10]
        z = float(np.percentile(above if above.size else zs, 25))
        u, v = (x1 + x2) / 2, (y1 + y2) / 2
        point = np.array([(u - k[2]) / k[0] * z, (v - k[5]) / k[4] * z, z])  # optical frame
        obj = (r @ point + t)[:2]
        chair = np.array([chair.x, chair.y])
        raw, obj = obj, self.near_face(chair, obj)
        self.get_logger().info(f"{target}: map ({obj[0]:.2f}, {obj[1]:.2f}) (depth ({raw[0]:.2f}, "
                               f"{raw[1]:.2f})), chair at ({chair[0]:.2f}, {chair[1]:.2f})")
        d = obj - chair
        return self.standoff(obj, chair), float(np.hypot(*d)), math.atan2(d[1], d[0])

    def ask(self, imgs, prompt):
        """imgs: one RGB image or a list of them, sent in one message."""
        imgs = imgs if isinstance(imgs, list) else [imgs]
        jpgs = [base64.b64encode(cv2.imencode(".jpg", cv2.cvtColor(i, cv2.COLOR_RGB2BGR))[1]).decode()
                for i in imgs]
        try:
            r = requests.post(VLM_URL, timeout=60, json={
                "temperature": 0, "max_tokens": 120,
                "messages": [{"role": "user", "content": [
                    *({"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + j}}
                      for j in jpgs),
                    {"type": "text", "text": prompt}]}]})
            return r.json()["choices"][0]["message"]["content"]
        except (requests.RequestException, KeyError, IndexError, ValueError) as e:
            self.get_logger().error(f"VLM request failed: {e}")
            return ""

    def vlm(self, rgb, target):
        img = cv2.resize(rgb, (VLM_W, VLM_H))
        text = self.ask(img, f"Is there a {target} in this image? If yes, give its bounding box as "
                             f'JSON {{"bbox_2d": [x1, y1, x2, y2]}}. '
                             f'If there is no {target}, answer {{"bbox_2d": null}}.')
        num = r"\s*(\d+(?:\.\d+)?)\s*"
        m = re.search(rf"\[{num},{num},{num},{num}\]", text)
        if not m:
            return None
        x1, x2 = (min(max(int(float(m[i]) * VLM_W / 1000), 0), VLM_W) for i in (1, 3))
        y1, y2 = (min(max(int(float(m[i]) * VLM_H / 1000), 0), VLM_H) for i in (2, 4))
        if x2 - x1 < 4 or y2 - y1 < 4:
            return None
        # The detector sometimes boxes something when the target is absent; a yes/no on the crop,
        # padded by half the box on each side for context, rejects those. On saved sim views it
        # passed no false box (a bare tight crop passed one and lost two real ones), but it also
        # drops small objects such as a dumbbell, which a closer look after exploring can find.
        # A full-resolution, min-size crop padded black off the frame did worse on the same views:
        # two false boxes at the frame edge and one real dumbbell lost.
        pw, ph = (x2 - x1) // 2, (y2 - y1) // 2
        crop = img[max(y1 - ph, 0):min(y2 + ph, VLM_H), max(x1 - pw, 0):min(x2 + pw, VLM_W)]
        answer = self.ask(crop, f"Is there a {target} in the middle of this image? "
                                f"Answer only yes or no.").strip().lower()
        self.get_logger().info(f"VLM box {[x1, y1, x2, y2]} for '{target}', verify: {answer}")
        return [x1, y1, x2, y2] if answer.startswith("yes") else None

    def standoff(self, obj, chair):
        """Reachable standoff on any side, trying the chair's side first."""
        c = self.coverage(chair)
        if c is None:
            return None
        res, h, w, ox, oy = c["key"]
        base = math.atan2(*(obj - chair)[::-1])
        offsets = np.arange(0, 2 * math.pi, 0.25)
        offsets = np.arctan2(np.sin(offsets), np.cos(offsets))
        offsets = offsets[np.argsort(np.abs(offsets), kind="stable")]
        # The chair's side (the old +/-1 rad sweep) first, nearest distance first; the far side
        # only when that fails, or a closer spot behind the object sends the chair round it.
        near = np.abs(offsets) <= 1.0
        for dth, dist in [(o, d) for side in (offsets[near], offsets[~near])
                          for d in STANDOFF for o in side]:
            a = base + dth
            g = obj - dist * np.array([math.cos(a), math.sin(a)])
            i, j = math.floor((g[0] - ox) / res), math.floor((g[1] - oy) / res)
            # room to turn in place: a cost-30 goal 0.66 m from the sim bed's corner trapped
            # the chair (spin and backup both "Collision Ahead")
            # the object must be in plain view from the spot: a reachable, clear spot on the far
            # side of a partition would report arrival from the next room
            sight = all(self.cost(g + (obj - g) * t) != 100
                        for t in np.arange(0, max(dist - 0.3, 0), res / 2) / dist)
            if (0 <= i < w and 0 <= j < h and np.isfinite(c["path"][j, i]) and sight
                    and self.clear(g, a)):
                return g[0], g[1], a  # yaw a faces the object
        return None

    def near_face(self, chair, obj):
        """The first lethal costmap cell on the line from the chair through the depth estimate,
        from 1.2 m before it to 1.0 m past it. In sim, depth on partial or edge-clipped boxes put
        the bed and sofa estimates inside the object (no free standoff) and the wardrobe 0.7 m
        short of it (the front stopped 1.1 m away). Far sightings and objects the costmap does
        not hold (a dumbbell) keep the depth estimate."""
        d = obj - chair
        rng = float(np.hypot(*d))
        if not 0.3 < rng <= MAX_RANGE or self.costmap is None:
            return obj
        # half-cell samples so a diagonal ray can't step over a lethal cell
        for s in np.arange(max(rng - 1.0, 0.3), rng + 1.0, self.costmap.info.resolution / 2):
            if self.cost(chair + s * d / rng) == 100:  # OccupancyGrid 100 = lethal
                return chair + s * d / rng
        return obj

    def clear(self, p, yaw, r=TURN_CLEAR):
        """No lethal cell (and nothing unknown) within r of the axle of a chair at p facing yaw.
        Lethal only: also rejecting inscribed cells added 0.35 m, so in m3_n no standoff passed
        next to the sofa.
        shortcut: Nav2 ignores goal yaw, so this assumes the chair arrives facing yaw (the drive
        or approach direction); check the real arrival heading if a goal still traps it.
        face() checks the actual arrival pose and heading before the final spin."""
        res = self.costmap.info.resolution
        n = round(r / res)
        cx, cy =p[0] - AXLE_BACK * math.cos(yaw), p[1] - AXLE_BACK * math.sin(yaw)
        m = self.costmap
        solid = obstacle_mask(self, m)
        i0 = math.floor((cx - m.info.origin.position.x) / res)
        j0 = math.floor((cy - m.info.origin.position.y) / res)
        return all(0 <= i0 + i < m.info.width and 0 <= j0 + j < m.info.height and not solid[j0 + j, i0 + i]
                   for i in range(-n, n + 1) for j in range(-n, n + 1) if i * i + j * j <= n * n)

    def cost(self, p):
        """Global costmap value at p: 0-100, or -1 for unknown or off the map."""
        m = self.costmap
        if m is None:
            return -1
        i = math.floor((p[0] - m.info.origin.position.x) / m.info.resolution)
        j = math.floor((p[1] - m.info.origin.position.y) / m.info.resolution)
        if not (0 <= i < m.info.width and 0 <= j < m.info.height):
            return -1
        return m.data[j * m.info.width + i]

    def free(self, p, limit=FREE_COST):
        return 0 <= self.cost(p) < limit


def listen(node, stt):
    """Energy-gated recording from the default ALSA mic; each utterance goes to faster-whisper."""
    hint = "go to the table. take me to the door. stop."
    noise, buf, quiet = None, [], 0
    for chunk in mic_chunks(node):
        a =np.frombuffer(chunk, np.int16).astype(np.float32) / 32768.0
        rms = float(np.sqrt(np.mean(a * a)))
        if not buf:
            noise = rms if noise is None else 0.95 * noise + 0.05 * rms
        loud = rms > max(3.0 * noise, 0.01)  # ponytail: energy gate; Silero VAD if rooms get noisy
        if loud or buf:
            buf.append(a)
            quiet = 0 if loud else quiet + 1
            if quiet > 25 or len(buf) > 300:  # 0.75 s of silence ends it, 9 s max
                audio, buf = np.concatenate(buf), []
                if len(audio) > 0.4 * 16000:
                    segments, _ = stt.transcribe(audio, language="en", beam_size=1,
                                                 vad_filter=True, initial_prompt=hint)
                    text = " ".join(s.text for s in segments).strip()
                    if text:
                        node.get_logger().info(f"heard: {text}")
                        node.on_text(text)


def main():
    rclpy.init()
    node = VoiceNav()
    stt = WhisperModel("base.en", device="cpu", compute_type="int8")
    threading.Thread(target=listen, args=(node, stt), daemon=True).start()
    threading.Thread(target=node.say, args=("Voice navigation ready.",), daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
