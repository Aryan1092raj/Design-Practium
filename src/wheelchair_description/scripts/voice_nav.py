#!/usr/bin/env python3
"""Voice navigation on a saved map: "take me to the kitchen" -> Nav2 goal.

Listens on the laptop mic and turns speech into text with faster-whisper. A command like
"go to / take me to <place>" is matched against the named places in config/locations.yaml,
and the place's pose is sent to Nav2 as a NavigateToPose goal. Nav2 does all the driving;
saying "stop" cancels the goal. Replies are spoken with spd-say.

Runs inside .venv-voice, with Nav2 and the saved map already up:
    ros2 run wheelchair_description voice_nav.py
Text can also be injected for testing:
    ros2 topic pub --once /voice/transcript std_msgs/String "{data: 'go to the kitchen'}"
"""
import math
import re
import subprocess
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from faster_whisper import WhisperModel
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from rapidfuzz import fuzz, process
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from go_to_location import load_locations

STOP = re.compile(r"\b(stop|halt|cancel)\b")
GO = re.compile(r"\b(?:go|take me|bring me|drive|move|head|navigate)\b.*?\b(?:to|towards?)\s+"
                r"(?:the |a |an |my )?(?P<t>[a-z ]+)")


def wait(future):
    while not future.done():
        time.sleep(0.05)
    return future.result()


class VoiceNav(Node):
    def __init__(self):
        super().__init__("voice_nav")
        self.places = load_locations()
        self.create_subscription(String, "/voice/transcript", lambda m: self.on_text(m.data), 10)
        self.status = self.create_publisher(String, "/voice/status", 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.spin_client = ActionClient(self, Spin, "spin")
        self.goal_handle = None
        self.busy = False
        self.speaking = False
        self.cancelled = threading.Event()

    def say(self, text):
        self.get_logger().info(text)
        self.status.publish(String(data=text))
        self.speaking = True  # the mic thread drops audio meanwhile, so the chair never hears itself
        try:
            subprocess.run(["spd-say", "-w", text], timeout=20)
        except (OSError, subprocess.TimeoutExpired):
            pass
        time.sleep(0.3)
        self.speaking = False

    def say_async(self, text):
        threading.Thread(target=self.say, args=(text,), daemon=True).start()

    def on_text(self, text):
        t = text.lower()
        if STOP.search(t):
            self.cancelled.set()
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.say_async("Stopping.")
            return
        m = GO.search(t)
        # A bare place name ("kitchen.") counts as a command, because a pause often splits
        # "take me to the" from the place; anything longer without a verb is ignored.
        bare = re.sub(r"[^a-z ]", "", t).strip()
        if not m and len(bare.split()) > 3:
            return  # ordinary speech near the chair is ignored
        target = re.sub(r"\b(please|now|thanks?)\b", "", m["t"] if m else bare).strip()
        names = {k: k.replace("_", " ") for k in self.places}
        match = process.extractOne(target, names, scorer=fuzz.WRatio, score_cutoff=85)
        if not match:
            if m:
                self.say_async(f"I don't know {target}. I know " + ", ".join(names.values()) + ".")
            return
        if self.busy:
            self.say_async("I am already moving. Say stop first.")
            return
        self.busy = True
        self.cancelled.clear()
        threading.Thread(target=self.run, args=(match[0], self.places[match[2]]),
                         daemon=True).start()

    def run(self, name, p):
        try:
            self.say(f"Going to the {name}. Say stop to cancel.")
            if self.cancelled.wait(2.0):
                return
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = "map"
            goal.pose.pose.position.x, goal.pose.pose.position.y = float(p["x"]), float(p["y"])
            goal.pose.pose.orientation.z = math.sin(p["yaw"] / 2)
            goal.pose.pose.orientation.w = math.cos(p["yaw"] / 2)
            ok = self.send(self.nav, goal) == GoalStatus.STATUS_SUCCEEDED
            if ok:
                self.face(p["yaw"])
            if not self.cancelled.is_set():
                self.say(f"Arrived at the {name}." if ok else
                         f"I could not reach the {name}. I may be stuck. Please help me.")
        finally:
            self.busy = False

    def face(self, yaw):
        """Nav2 ignores the goal heading (yaw_goal_tolerance 6.28 in the chair's params), so turn
        in place to the saved heading with the behaviour server's Spin."""
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
            self.get_logger().error("Nav2 action server not available")
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


def listen(node, stt):
    """Energy-gated recording from the default ALSA mic; each utterance goes to faster-whisper."""
    proc = subprocess.Popen(["arecord", "-q", "-f", "S16_LE", "-r", "16000", "-c", "1", "-t", "raw"],
                            stdout=subprocess.PIPE)
    hint = "go to the " + ", ".join(k.replace("_", " ") for k in node.places) + ". stop."
    noise, buf, quiet = None, [], 0
    while rclpy.ok():
        chunk = proc.stdout.read(960)  # 30 ms
        if not chunk:
            break
        if node.speaking:
            buf = []
            continue
        a = np.frombuffer(chunk, np.int16).astype(np.float32) / 32768.0
        rms = float(np.sqrt(np.mean(a * a)))
        if not buf:
            noise = rms if noise is None else 0.95 * noise + 0.05 * rms
        loud = rms > max(3.0 * noise, 0.01)  # ponytail: energy gate; Silero VAD if rooms get noisy
        if loud or buf:
            buf.append(a)
            quiet = 0 if loud else quiet + 1
            if quiet > 40 or len(buf) > 300:  # 1.2 s of silence ends it, 9 s max
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
    node.say_async("Voice navigation ready.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
