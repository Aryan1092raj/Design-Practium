#!/usr/bin/env python3
"""Voice navigation on a saved map: "take me to the kitchen" -> Nav2 goal.

Listens on the laptop mic and turns speech into text with faster-whisper. A command like
"go to / take me to <place>" is matched against the named places in config/locations.yaml,
or, through each place's optional "desc", against what the rider wants to do there ("take me
where I can print"). The place's pose is sent to Nav2 as a NavigateToPose goal. Nav2 does all
the driving; saying "stop" cancels the goal. Replies are printed in the terminal and published on /voice/status.

Runs inside .venv-voice, with Nav2 and the saved map already up:
    ros2 run wheelchair_description voice_nav.py
In the Gazebo sim, use the small_house places instead:
    ros2 run wheelchair_description voice_nav.py --ros-args -p locations:=locations_sim.yaml
Text can also be injected for testing:
    ros2 topic pub --once /voice/transcript std_msgs/String "{data: 'go to the kitchen'}"
"""
import json
import math
import re
import subprocess
import threading
import time
import urllib.request

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

SLM_MODEL = "qwen3:4b"  # 17/17 on purpose and name phrasing; 0.6b missed every purpose request
SLM_URL = "http://127.0.0.1:11434/api/chat"
STOP = re.compile(r"\b(stop|halt|cancel)\b")
GO = re.compile(r"\b(?:go|take me|bring me|drive|move|head|navigate)\b.*?\b(?:to|towards?)\s+"
                r"(?:the |a |an |my )?(?P<t>[a-z ]+)")


def wait(future):
    while not future.done():
        time.sleep(0.05)
    return future.result()


def nav2_gone(client, handle, node):
    """True if the action server vanished (Nav2 crashed): its result would never come, the
    caller would wait forever, and the chair would ignore every command until a restart."""
    if client.server_is_ready():
        return False
    node.get_logger().error("Nav2 action server vanished mid-goal")
    handle.cancel_goal_async()  # in case it is only a discovery blip, the goal must not run on
    return True


class VoiceNav(Node):
    def __init__(self):
        super().__init__("voice_nav")
        self.places = load_locations(self.declare_parameter("locations", "locations.yaml").value)
        self.create_subscription(String, "/voice/transcript", lambda m: self.on_text(m.data), 10)
        self.status = self.create_publisher(String, "/voice/status", 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.spin_client = ActionClient(self, Spin, "spin")
        self.goal_handle = None
        self.busy = False
        self.cancelled = threading.Event()

    def say(self, text):
        """Replies are printed and published on /voice/status; the real chair has no speaker."""
        self.get_logger().info(text)
        self.status.publish(String(data=text))

    def slm_place(self, text, names):
        """Ask the small language model (Ollama) which place the user wants.

        Returns the place name, "none" when the text is not a go-to command, or None when
        the SLM is unreachable. Called only when no place name matched the text.
        """
        # A schema with an enum, not plain "json": with plain json qwen3:0.6b padded the reply
        # with newlines until num_predict ran out, and the JSON never closed.
        schema = {"type": "object", "required": ["place"], "properties": {
            "place": {"type": "string", "enum": [*names.values(), "none"]}}}
        catalog = "\n".join(f"- {n}: {self.places[k].get('desc', n)}" for k, n in names.items())
        # One worked example built from the last place's desc ("...; cook, eat" -> "I want to cook"),
        # so the model sees a request by purpose, not by name.
        last = list(names)[-1]
        uses = self.places[last].get("desc", "").split(";")[-1].split(",")[0].strip()
        example = [{"role": "user", "content": f"I want to {uses}" if uses else f"take me to the {names[last]}"},
                   {"role": "assistant", "content": json.dumps({"place": names[last]})}]
        body = {"model": SLM_MODEL, "stream": False, "think": False, "format": schema,
                "keep_alive": "30m", "options": {"temperature": 0, "num_predict": 64},  # 24 cut the JSON short
                "messages": [
                    {"role": "system", "content":
                     "You pick where a wheelchair rider wants to go. Places and what is there:\n"
                     + catalog + "\nIf the rider names a place, or says something they want to do "
                     'or find that fits one place, reply JSON {"place": "<that place>"}. If it is '
                     'not a request to go somewhere, reply {"place": "none"}.'},
                    *example,
                    {"role": "user", "content": "what is the weather"},
                    {"role": "assistant", "content": '{"place": "none"}'},
                    {"role": "user", "content": text}]}
        req = urllib.request.Request(SLM_URL, json.dumps(body).encode(),
                                     {"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=15) as r:
                place = str(json.loads(json.loads(r.read())["message"]["content"])["place"]).lower().strip()
        except Exception as e:  # SLM down or bad JSON: fall back to regex
            self.get_logger().warn(f"SLM unavailable ({e}); using regex")
            return None
        return place if place in names.values() else "none"

    def on_text(self, text):
        t = text.lower()
        if STOP.search(t):
            self.cancelled.set()
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.say("Stopping.")
            return
        if not self.places:
            self.say("No places saved. Record them with save_location.py, then restart me.")
            return
        names = {k: k.replace("_", " ") for k in self.places}
        m = GO.search(t)
        # A bare place name ("kitchen.") counts as a command, because a pause often splits
        # "take me to the" from the place.
        bare = re.sub(r"[^a-z ]", "", t).strip()
        target = re.sub(r"\b(please|now|thanks?)\b", "", m["t"] if m else bare).strip()
        # Place names are matched first, without the SLM: qwen3:0.6b got plain names wrong
        # ("take me to the kitchen" -> none). The SLM only resolves what a name match can't,
        # such as "take me where I can cook" through each place's desc.
        match = None
        if m or len(bare.split()) <= 3:
            match = process.extractOne(target, names, scorer=fuzz.WRatio, score_cutoff=85)
        if not match:
            slm = self.slm_place(t, names)  # None when the SLM is down
            if slm and slm != "none":
                match = process.extractOne(slm, names, scorer=fuzz.WRatio, score_cutoff=85)
        if not match:
            if m:
                self.say(f"I don't know {target}. I know " + ", ".join(names.values()) + ".")
            return  # otherwise ordinary speech near the chair is ignored
        if self.busy:
            self.say("I am already moving. Say stop first.")
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
        result = handle.get_result_async()
        while not result.done() and not nav2_gone(client, handle, self):
            time.sleep(0.2)
        self.goal_handle = None
        return result.result().status if result.done() else None


def mic_chunks(node):
    """30 ms chunks from the default ALSA mic. arecord exits when the mic is unplugged or taken
    by another program; it is restarted every 2 s, because a deaf chair can't hear "stop"."""
    warned = False
    while rclpy.ok():
        proc = subprocess.Popen(["arecord", "-q", "-f", "S16_LE", "-r", "16000", "-c", "1",
                                 "-t", "raw"], stdout=subprocess.PIPE)
        while chunk := proc.stdout.read(960):
            warned = False
            yield chunk
        proc.wait()
        if not warned:
            node.say("My microphone stopped working. I keep trying to reconnect it.")
            warned = True
        time.sleep(2.0)


def listen(node, stt):
    """Energy-gated recording from the default ALSA mic; each utterance goes to faster-whisper."""
    hint = "go to the " + ", ".join(k.replace("_", " ") for k in node.places) + ". stop."
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
    node.say("Voice navigation ready.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
