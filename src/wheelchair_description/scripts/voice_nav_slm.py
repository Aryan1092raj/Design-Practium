#!/usr/bin/env python3
"""Voice navigation with local SLM orchestration.

This is a new node that keeps the original voice_nav.py untouched.
It follows the same speech recognition + stop handling flow as the current
voice_nav.py, but adds a local SLM layer after the fast fuzzy-match fails.

Flow:
  1. listen for /voice/transcript or transcript text
  2. STOP keyword -> cancel goal
  3. fuzzy match against saved places -> navigate (current behavior)
  4. if no place match or low confidence -> call local SLM
  5. SLM returns intent/target/target_type/confidence
  6. if target_type == place -> try fuzzy-match on target, then navigate
  7. if target_type == object -> publish /voice/object_request (object name)
  8. if low confidence -> ask clarifying question

This allows existing object_nav.py / vlm_nav.py to keep their VLM logic intact.
"""
import math
import re
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from rapidfuzz import fuzz, process
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from go_to_location import load_locations
from slm_local import SLMLocal

STOP = re.compile(r"\b(stop|halt|cancel)\b")
GO = re.compile(r"\b(?:go|take me|bring me|drive|move|head|navigate)\b.*?\b(?:to|towards?)\s+"
                r"(?:the |a |an |my )?(?P<t>[a-z ]+)")


def wait(future):
    while not future.done():
        time.sleep(0.05)
    return future.result()


class VoiceNavSLM(Node):
    def __init__(self, name="voice_nav_slm"):
        super().__init__(name)
        self.places = load_locations()
        self.create_subscription(String, "/voice/transcript", lambda m: self.on_text(m.data), 10)
        self.status = self.create_publisher(String, "/voice/status", 10)
        self.slm_decision_pub = self.create_publisher(String, "/voice/slm_decision", 10)
        self.object_request_pub = self.create_publisher(String, "/voice/object_request", 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.spin_client = ActionClient(self, Spin, "spin")
        self.goal_handle = None
        self.busy = False
        self.cancelled = threading.Event()
        self.slm = SLMLocal()
        self.immediate_cutoff = self.declare_parameter("immediate_cutoff", 85).value
        self.secondary_cutoff = self.declare_parameter("secondary_cutoff", 70).value
        self.auto_conf = self.declare_parameter("slm_auto_conf", 0.7).value
        self.clarify_conf = self.declare_parameter("slm_clarify_conf", 0.4).value

    def say(self, text):
        self.get_logger().info(text)
        self.status.publish(String(data=text))

    def publish_slm_decision(self, parsed):
        self.slm_decision_pub.publish(String(data=str(parsed)))

    def on_text(self, text):
        t = text.lower()
        if STOP.search(t):
            self.cancelled.set()
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.say("Stopping.")
            return

        # First, try direct regex extraction for a simple command.
        m = GO.search(t)
        bare = re.sub(r"[^a-z ]", "", t).strip()

        # A bare place name counts as a command on a short utterance.
        if not m and len(bare.split()) > 3:
            return

        target = re.sub(r"\b(please|now|thanks?)\b", "", (m["t"] if m else bare)).strip()
        names = {k: k.replace("_", " ") for k in self.places}

        # Fast path: direct fuzzy-match against saved places.
        match = process.extractOne(target, names, scorer=fuzz.WRatio, score_cutoff=self.immediate_cutoff)
        if match:
            if self.busy:
                self.say("I am already moving. Say stop first.")
                return
            self.busy = True
            self.cancelled.clear()
            threading.Thread(target=self.run, args=(match[0], self.places[match[2]]), daemon=True).start()
            return

        # Fallback to SLM for harder cases.
        parsed = self.slm.parse_command(text)
        self.publish_slm_decision(parsed)
        if parsed.get("intent") == "stop":
            self.cancelled.set()
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.say("Stopping.")
            return

        if parsed.get("intent") == "go_to":
            target_name = str(parsed.get("target", "")).strip()
            if not target_name:
                self.say("I did not understand that. Please say the destination again.")
                return

            if parsed.get("target_type") == "place":
                # Try a second fuzzy-match against saved places using SLM target.
                match = process.extractOne(target_name, names, scorer=fuzz.WRatio, score_cutoff=self.secondary_cutoff)
                if match:
                    if self.busy:
                        self.say("I am already moving. Say stop first.")
                        return
                    self.busy = True
                    self.cancelled.clear()
                    threading.Thread(target=self.run, args=(match[0], self.places[match[2]]), daemon=True).start()
                    return
                # low confidence -> ask clarifying question instead of blindly moving
                if float(parsed.get("confidence", 0.0)) < self.clarify_conf:
                    self.say(f"I think you mean {target_name}, but I am not sure. Can you say it again more clearly?")
                    return
                # If the SLM still suggests a place but the match failed, let the user clarify.
                self.say(f"I think you mean {target_name}, but I do not have that place saved. Please say another destination.")
                return

            if parsed.get("target_type") == "object":
                # If object target is confident enough, route to object-nav via a dedicated topic.
                if float(parsed.get("confidence", 0.0)) < self.clarify_conf:
                    self.say(f"I think you mean {target_name}, but I am not sure. Can you say the object more clearly?")
                    return
                self.object_request_pub.publish(String(data=target_name))
                self.say(f"Looking for the {target_name}.")
                return

            if float(parsed.get("confidence", 0.0)) < self.clarify_conf:
                self.say("I did not understand that destination. Please say it again more clearly.")
                return

        self.say("I did not understand that instruction.")

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
        try:
            q = self.tf.lookup_transform("map", "base_link", Time()).transform.rotation
        except Exception as e:
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


def main():
    rclpy.init()
    node = VoiceNavSLM()
    node.say("Voice navigation ready.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
