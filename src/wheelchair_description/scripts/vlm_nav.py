#!/usr/bin/env python3
"""Voice + camera navigation with no saved places: "take me to the table" -> Nav2 goal.

Listens on the laptop mic and turns speech into text with faster-whisper. The target is looked
for in the front RGB-D camera with a VLM (Qwen2.5-VL 3B); if it is not in view, the chair turns
in place 90 degrees at a time and looks again. The bounding box plus the aligned depth give the
object's position in the map, and the goal is a free spot a short distance in front of it.
Nav2 does all the driving; saying "stop" cancels the goal. Replies are printed and published on
/voice/status (the real chair has no speaker).

Needs nav launched with front_color:=true, scripts/vlm_server.sh running, and runs inside
.venv-voice:
    ros2 run wheelchair_description vlm_nav.py
Text can also be injected for testing:
    ros2 topic pub --once /voice/transcript std_msgs/String "{data: 'go to the bed'}"
"""
import base64
import math
import re
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
import requests
from action_msgs.msg import GoalStatus
from faster_whisper import WhisperModel
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

VLM_URL = "http://127.0.0.1:8091/v1/chat/completions"  # scripts/vlm_server.sh (Qwen2.5-VL 3B)
# Multiples of 28 and within the server's --image-min/max-tokens, so the image is not rescaled
# and Qwen2.5-VL returns boxes in exactly these pixels.
VLM_W, VLM_H = 448, 336
CAMERAS = ["camera"]  # front D455 only; the side cameras run depth-only to save USB bandwidth
STANDOFF = (0.8, 1.1, 1.5)  # metres short of the object's near face; free() rejects inflated cells
LOOKS = 4  # front view plus three 90-degree turns in place
MAX_RANGE = 4.0  # stereo depth error grows with range; farther objects are looked at again on arrival
FREE_COST = 50  # global costmap cells below this count as free for a goal

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
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 lambda m: setattr(self, "costmap", m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, "/voice/transcript", lambda m: self.on_text(m.data), 10)
        self.status = self.create_publisher(String, "/voice/status", 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.spin_client = ActionClient(self, Spin, "spin")
        self.goal_handle = None
        self.busy = False
        self.cancelled = threading.Event()

    # ---- speech out / commands in -------------------------------------------------------
    def say(self, text):
        self.get_logger().info(text)
        self.status.publish(String(data=text))

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
        try:
            for attempt in range(2):
                found = self.find(target)
                if self.cancelled.is_set():
                    return
                if found is None:
                    self.say(f"I can't find the {target}.")
                    return
                goal, rng = found
                if goal is None:
                    self.say(f"I see the {target}, but I can't find a free spot next to it.")
                    return
                if rng < STANDOFF[0] + 0.3:
                    self.say(f"I am already next to the {target}.")
                    return
                final = rng <= MAX_RANGE or attempt == 1
                if not self.drive(target, *goal, final=final, announce=attempt == 0) or final:
                    return
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

    def send(self, client, goal):
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"action server {client._action_name} not available")
            return None
        handle = wait(client.send_goal_async(goal))
        if not handle.accepted:
            return None
        self.goal_handle = handle
        if self.cancelled.is_set():
            handle.cancel_goal_async()
        status = wait(handle.get_result_async()).status
        self.goal_handle = None
        return status

    # ---- semantic search ----------------------------------------------------------------
    def find(self, target):
        """Look with the camera; if nothing, turn 90 degrees in place and look again."""
        for turn in range(LOOKS):
            for cam in CAMERAS:
                hit = self.ground(cam, target)
                if hit:
                    self.get_logger().info(f"{target} seen by {cam} at {hit[1]:.1f} m")
                    return hit
            if turn < LOOKS - 1 and not self.cancelled.is_set():
                self.say(f"Turning to look for the {target}.")
                spin = Spin.Goal()
                spin.target_yaw = math.pi / 2
                spin.time_allowance.sec = 30
                self.send(self.spin_client, spin)
                time.sleep(1.0)  # fresh frames after the turn
            if self.cancelled.is_set():
                return None
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
        # central half of the box, nearest quartile: skips background seen around the object
        u0, u1 = int(x1 + (x2 - x1) / 4), int(x2 - (x2 - x1) / 4) + 1
        v0, v1 = int(y1 + (y2 - y1) / 4), int(y2 - (y2 - y1) / 4) + 1
        patch = depth[max(v0, 0):min(v1, h), max(u0, 0):min(u1, w)]
        d = patch[np.isfinite(patch) & (patch > 0.2)]
        if d.size == 0:
            return None
        z = float(np.percentile(d, 25))
        k = f["info"].k
        u, v = (x1 + x2) / 2, (y1 + y2) / 2
        point = np.array([(u - k[2]) / k[0] * z, (v - k[5]) / k[4] * z, z])  # optical frame
        try:
            r, t = to_matrix(self.tf.lookup_transform("map", f["depth"].header.frame_id, stamp,
                                                      Duration(seconds=0.5)))
            chair = self.tf.lookup_transform("map", "base_link", Time()).transform.translation
        except Exception as e:  # tf2 raises several unrelated exception types
            self.get_logger().warn(f"tf: {e}")
            return None
        obj = (r @ point + t)[:2]
        chair = np.array([chair.x, chair.y])
        self.get_logger().info(f"{target}: map ({obj[0]:.2f}, {obj[1]:.2f})")
        return self.standoff(obj, chair), float(np.hypot(*(obj - chair)))

    def ask(self, img, prompt):
        jpg = base64.b64encode(cv2.imencode(".jpg", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))[1]).decode()
        try:
            r = requests.post(VLM_URL, timeout=60, json={
                "temperature": 0, "max_tokens": 120,
                "messages": [{"role": "user", "content": [
                    {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + jpg}},
                    {"type": "text", "text": prompt}]}]})
            return r.json()["choices"][0]["message"]["content"]
        except (requests.RequestException, KeyError, IndexError, ValueError) as e:
            self.get_logger().error(f"VLM request failed: {e}")
            return ""

    def vlm(self, rgb, target):
        img = cv2.resize(rgb, (VLM_W, VLM_H))
        text = self.ask(img, f"Is there a {target} in this image? If yes, give its bounding box as "
                             f'JSON {{"bbox_2d": [x1, y1, x2, y2]}} in pixel coordinates. '
                             f'If there is no {target}, answer {{"bbox_2d": null}}.')
        num = r"\s*(\d+(?:\.\d+)?)\s*"
        m = re.search(rf"\[{num},{num},{num},{num}\]", text)
        if not m:
            return None
        x1, x2 = (min(max(int(float(m[i])), 0), VLM_W) for i in (1, 3))
        y1, y2 = (min(max(int(float(m[i])), 0), VLM_H) for i in (2, 4))
        if x2 - x1 < 4 or y2 - y1 < 4:
            return None
        # The detector boxes something even when the target is absent; a yes/no on the crop
        # rejects most of those (asked for "bed", it boxed a dumbbell).
        answer = self.ask(img[y1:y2, x1:x2],
                          f"Is this a {target}? Answer only yes or no.").strip().lower()
        self.get_logger().info(f"VLM box {[x1, y1, x2, y2]} for '{target}', verify: {answer}")
        return [x1, y1, x2, y2] if answer.startswith("yes") else None

    def standoff(self, obj, chair):
        """First free spot short of the object, trying wider angles and distances."""
        base = math.atan2(*(obj - chair)[::-1])
        for dist in STANDOFF:
            for dth in (0.0, 0.5, -0.5, 1.0, -1.0):
                a = base + dth
                g = obj - dist * np.array([math.cos(a), math.sin(a)])
                if self.free(g):
                    return g[0], g[1], a  # yaw a faces the object
        return None

    def free(self, p):
        m = self.costmap
        if m is None:
            return False
        i = int((p[0] - m.info.origin.position.x) / m.info.resolution)
        j = int((p[1] - m.info.origin.position.y) / m.info.resolution)
        if not (0 <= i < m.info.width and 0 <= j < m.info.height):
            return False
        return 0 <= m.data[j * m.info.width + i] < FREE_COST


def listen(node, stt):
    """Energy-gated recording from the default ALSA mic; each utterance goes to faster-whisper."""
    proc = subprocess.Popen(["arecord", "-q", "-f", "S16_LE", "-r", "16000", "-c", "1", "-t", "raw"],
                            stdout=subprocess.PIPE)
    hint = "go to the table. take me to the door. stop."
    noise, buf, quiet = None, [], 0
    while rclpy.ok():
        chunk = proc.stdout.read(960)  # 30 ms
        if not chunk:
            break
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
