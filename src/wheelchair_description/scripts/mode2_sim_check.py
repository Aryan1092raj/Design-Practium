#!/usr/bin/env python3
"""Play the rider for an already running Mode 2 Gazebo small_house stack.

voice_nav.py must be running with locations:=locations_sim.yaml. This script
publishes purpose-only sentences, observes /voice/status, and stages at
(1.2, 0.0, 0.0) first unless --no-stage is given. Durations use wall time.
The verdict uses voice arrival reports; it does not measure contacts/recoveries.

    ros2 run wheelchair_description mode2_sim_check.py
    python3 mode2_sim_check.py --selftest
    python3 mode2_sim_check.py --sentence "I want to cook something=kitchen"
"""

import argparse
from collections import deque
from dataclasses import dataclass
import math
import re
import sys
import time


SENTENCES = (
    ("take me somewhere I can lie down", "bedroom"),
    ("I want to cook something", "kitchen"),
    ("I'd like to watch some TV", "living_room"),
    ("I need to work out", "gym"),
    ("I want some fresh air", "balcony"),
)
START_TIMEOUT = 15.0
STATUS_PATTERNS = (
    ("going", re.compile(r"Going to the (.+?)\.(?: Say stop to cancel\.)?")),
    ("arrived", re.compile(r"Arrived at the (.+)\.")),
    ("failed", re.compile(
        r"I could not reach the (.+)\. I may be stuck\. Please help me\.")),
)


def place_key(name):
    return "_".join(name.lower().strip().split())


def parse_status(text):
    """Return (event, canonical place), preserving spaces/underscores equally."""
    text = text.strip()
    for event, pattern in STATUS_PATTERNS:
        match = pattern.fullmatch(text)
        if match:
            return event, place_key(match[1])
    if text == "Stopping.":
        return "stopping", None
    if text.startswith("I don't know "):
        return "unknown", None
    return "other", None


def selftest():
    assert parse_status("Going to the living_room. Say stop to cancel.") == (
        "going", "living_room")
    assert parse_status("Arrived at the kitchen.") == ("arrived", "kitchen")
    assert parse_status("Going to the living room. Say stop to cancel.") == (
        "going", "living_room")
    assert parse_status("Arrived at the living room.") == ("arrived", "living_room")
    assert parse_status("I could not reach the gym. I may be stuck. Please help me.") == (
        "failed", "gym")
    assert parse_status("Stopping.") == ("stopping", None)
    assert parse_status("I don't know garage. I know kitchen.") == ("unknown", None)
    assert parse_status("My microphone stopped working.") == ("other", None)
    assert parse_status("Arrived at the kitchen. extra text") == ("other", None)
    print("selftest: PASS")


def sentence_pair(value):
    text, separator, place = value.rpartition("=")
    if not separator or not text.strip() or not place.strip():
        raise argparse.ArgumentTypeError('--sentence must be "text=place"')
    return text.strip(), place_key(place)


def parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--timeout", type=float, default=150.0,
                   help="wall seconds to arrive after resolving a place (default: 150)")
    p.add_argument("--sentence", action="append", type=sentence_pair,
                   help='replace defaults with repeatable "text=place" pairs; '
                        'acceptance requires five different expected places')
    p.add_argument("--no-stage", action="store_true", help="skip the staging goal")
    p.add_argument("--selftest", action="store_true", help="check status parsing without ROS")
    return p


@dataclass
class Record:
    sentence: str
    expected: str
    resolved: str | None = None
    result: str = "no_response"
    seconds: float | None = None


def summarize(rows):
    headers = ("sentence", "expected", "resolved place", "result", "arrival seconds")
    cells = [(row.sentence, row.expected, row.resolved or "-", row.result,
              f"{row.seconds:.2f}" if row.seconds is not None else "-") for row in rows]
    widths = [max(len(header), *(len(row[i]) for row in cells))
              for i, header in enumerate(headers)]
    print("  ".join(header.ljust(width) for header, width in zip(headers, widths)))
    for cell in cells:
        print("  ".join(value.ljust(width) for value, width in zip(cell, widths)))
    arrived = sum(row.result == "arrived" and row.resolved == row.expected for row in rows)
    full_suite = len(rows) == 5 and len({row.expected for row in rows}) == 5
    passed = full_suite and arrived >= 4
    print(f"Mode 2: {'PASS' if passed else 'FAIL'} ({arrived}/{len(rows)} arrived at "
          "the expected place; requires >=4/5 different places)", flush=True)
    return passed


def run_ros(argv):
    # Keep --selftest and --help usable without a ROS environment.
    import rclpy
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.utilities import remove_ros_args
    from std_msgs.msg import String

    p = parser()
    args = p.parse_args(remove_ros_args(argv))
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        p.error("--timeout must be a positive finite number")
    sentences = args.sentence or SENTENCES

    class SimCheck(Node):
        def __init__(self):
            super().__init__("mode2_sim_check", parameter_overrides=[
                rclpy.Parameter("use_sim_time", value=True)])
            self.events = deque()
            self.status_sub = self.create_subscription(
                String, "/voice/status", self.on_status, 10)
            self.transcript = self.create_publisher(String, "/voice/transcript", 10)
            self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
            self.stage_handle = None

        def on_status(self, msg):
            self.events.append((time.monotonic(), msg.data, *parse_status(msg.data)))

        def spin_until(self, predicate, deadline):
            while rclpy.ok() and not predicate() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=min(
                    0.05, max(0.0, deadline - time.monotonic())))
            return predicate()

        def publish(self, text):
            self.events.clear()
            started = time.monotonic()
            self.transcript.publish(String(data=text))
            return started

        def next_event(self, deadline):
            if self.spin_until(lambda: bool(self.events), deadline):
                event = self.events.popleft()
                if event[0] <= deadline:
                    return event
            return None

        def stop(self):
            self.publish("stop")
            deadline = time.monotonic() + 5.0
            while event := self.next_event(deadline):
                if event[2] == "stopping":
                    break
            # "Stopping." acknowledges the transcript, before the worker clears
            # busy. Keep spinning briefly so the next sentence is not rejected.
            self.spin_until(lambda: False, time.monotonic() + 2.0)
            self.events.clear()

        def stage(self):
            # mode1_sim_check.SimCheck/go_to are local to run_ros, so they cannot
            # be imported without changing that file. Use its permitted fallback:
            # the same map pose and the Mode 1 default 120-second stage timeout.
            if not self.spin_until(
                    lambda: self.nav.server_is_ready()
                    and self.get_clock().now().nanoseconds > 0,
                    time.monotonic() + 10.0):
                print("stage: failed (NavigateToPose or /clock not ready)", flush=True)
                return False
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = 1.2
            goal.pose.pose.orientation.w = 1.0  # y=0, yaw=0, as in Mode 1.
            started = time.monotonic()
            deadline = started + 120.0
            send = self.nav.send_goal_async(goal)
            if not self.spin_until(send.done, deadline):
                # Handle late acceptance before exiting, so staging cannot drive
                # on silently after the check reports a timeout.
                if self.spin_until(send.done, time.monotonic() + 5.0):
                    self.stage_handle = send.result()
                else:
                    print("stage: timeout (acceptance unresolved)", flush=True)
                    return False
            else:
                self.stage_handle = send.result()
            if not self.stage_handle.accepted:
                print("stage: failed (goal rejected)", flush=True)
                return False
            result = self.stage_handle.get_result_async()
            completed = self.spin_until(result.done, deadline)
            succeeded = completed and result.result().status == GoalStatus.STATUS_SUCCEEDED
            if not completed:
                self.stage_handle.cancel_goal_async()
                self.spin_until(result.done, time.monotonic() + 5.0)
            print(f"stage: {'succeeded' if succeeded else 'failed'} in "
                  f"{time.monotonic() - started:.1f} s", flush=True)
            self.stage_handle = None
            return succeeded

        def check_sentence(self, sentence, expected):
            row = Record(sentence, expected)
            print(f"Rider: {sentence!r} -> expected {expected}", flush=True)
            started = self.publish(sentence)
            deadline = started + START_TIMEOUT
            while event := self.next_event(deadline):
                _, _, kind, place = event
                if kind == "going":
                    row.resolved = place
                    break
            if row.resolved is None:
                self.stop()  # Cancel a late SLM response before the next trial.
                return row
            if row.resolved != expected:
                row.result = "wrong_place"
                self.stop()
                return row
            deadline = time.monotonic() + args.timeout
            while event := self.next_event(deadline):
                stamp, _, kind, place = event
                if kind == "going" and place != expected:
                    row.resolved, row.result = place, "wrong_place"
                    self.stop()
                    return row
                if kind in ("arrived", "failed") and place == row.resolved:
                    row.result = kind
                    if kind == "arrived":
                        row.seconds = stamp - started
                    # voice_nav publishes the terminal status just before
                    # clearing busy in its worker's finally block.
                    self.spin_until(lambda: False, time.monotonic() + 0.25)
                    return row
            row.result = "timeout"
            self.stop()
            return row

        def negative_check(self):
            started = self.publish("take me to the garage")
            statuses = []
            goal_started = False
            while event := self.next_event(started + START_TIMEOUT):
                _, text, kind, _ = event
                statuses.append(text)
                if kind == "going":
                    goal_started = True
                    self.stop()
                    break
            if not goal_started:
                self.stop()  # Also cancel any response arriving after the window.
            detail = "; ".join(statuses) if statuses else "silence"
            return (f"Negative check (garage): {'FAIL' if goal_started else 'PASS'} "
                    f"(goal started within 15 s: {'yes' if goal_started else 'no'}; "
                    f"status: {detail!r})")

    rclpy.init(args=argv)
    node = SimCheck()
    rows = []
    try:
        if not node.spin_until(lambda: node.transcript.get_subscription_count() > 0,
                               time.monotonic() + 15.0):
            print("Not ready: /voice/transcript needs a subscriber (voice_nav.py).",
                  file=sys.stderr)
            return 2
        if not args.no_stage and not node.stage():
            return 1
        for sentence, expected in sentences:
            rows.append(node.check_sentence(sentence, expected))
        negative = node.negative_check()
        passed = summarize(rows)
        print(negative, flush=True)
        return 0 if passed else 1
    except KeyboardInterrupt:
        if rclpy.ok():
            node.stop()
        return 130
    finally:
        if rclpy.ok():
            if node.stage_handle is not None and node.stage_handle.accepted:
                cancel = node.stage_handle.cancel_goal_async()
                node.spin_until(cancel.done, time.monotonic() + 5.0)
            node.destroy_node()
            rclpy.shutdown()


def main():
    argv = sys.argv[1:]
    if "--selftest" in argv or "--help" in argv or "-h" in argv:
        args = parser().parse_args(argv)
        if args.selftest:
            selftest()
        return 0
    return run_ros(argv)


if __name__ == "__main__":
    sys.exit(main())
