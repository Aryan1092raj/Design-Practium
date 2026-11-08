#!/usr/bin/env python3
"""Check Mode 1 against an already running Gazebo small_house navigation stack.

    ros2 run wheelchair_description mode1_sim_check.py
    ros2 run wheelchair_description mode1_sim_check.py --cmd-topic /cmd_vel
    python3 mode1_sim_check.py --selftest

Use the second command when use_collision_monitor:=false. Both final command
topics carry Twist; gazebo_sim.launch.py stamps it only at /wc_control/cmd_vel.
Speed is abs(forward wheel odometry velocity), including stationary samples in
the mean. Durations/timeouts use wall time; stop intervals use simulation time.
The navigation verdict requires all five distinct places to have been attempted.
Cruise and stop counts are reported separately; stops do not identify obstacles.
A staging goal runs first (--stage, --no-stage) because spawn furniture sits inside the turn radius.
"""

import argparse
from dataclasses import dataclass
import math
import sys
import time


GOALS = ("bedroom", "kitchen", "living_room", "gym", "balcony")
ODOM_TOPIC = "/wc_control/odom"


@dataclass(frozen=True)
class StopState:
    since: float | None = None
    counted: bool = False
    last_time: float | None = None


def count_stop(state, now, linear_x, angular_z, distance, active):
    """Return (new state, increment) without mutating inputs or accessing ROS.

    Each continuous qualifying interval counts once, strictly after 0.3 s.
    Motion, rotation, arrival, inactivity, missing data, or a clock rewind resets
    the interval. Call on commands and timer ticks to observe a held zero command.
    """
    valid = (active and distance is not None and linear_x is not None
             and angular_z is not None
             and all(math.isfinite(v) for v in (now, linear_x, angular_z, distance))
             and distance > 0.5 and abs(linear_x) < 0.01 and abs(angular_z) < 0.01)
    if not valid:
        return StopState(last_time=now), 0
    rewound = state.last_time is not None and now < state.last_time
    since = now if state.since is None or rewound else state.since
    counted = False if rewound else state.counted
    increment = int(not counted and now > since + 0.3)
    return StopState(since, counted or bool(increment), now), increment


def selftest():
    state = StopState()

    def step(now, x=0.0, z=0.0, distance=2.0, active=True):
        nonlocal state
        state, increment = count_stop(state, now, x, z, distance, active)
        return increment

    assert step(0.0) == 0
    assert step(0.3) == 0  # Strict duration threshold.
    assert step(0.301) == 1
    assert step(1.0) == 0  # One count for a sustained stop.
    assert step(1.1, x=0.2) == 0
    assert step(1.2) == 0
    assert step(1.501) == 1  # A second, separate stop.
    for kwargs in ({"x": 0.01}, {"x": -0.01}, {"z": 0.01}, {"z": -0.1},
                   {"distance": 0.5}, {"distance": 0.1}, {"active": False},
                   {"distance": None}, {"x": None}, {"z": None},
                   {"distance": math.nan}, {"x": math.inf}):
        state = StopState()
        assert step(0.0) == 0
        assert step(0.2, **kwargs) == 0
        assert step(0.4) == 0  # Invalid sample broke continuity.
        assert step(0.701) == 1
    state = StopState()
    assert step(0.0, x=-0.009, z=-0.009, distance=0.501) == 0
    assert step(0.301, x=-0.009, z=-0.009, distance=0.501) == 1
    state = StopState()
    assert step(0.0) == 0
    assert step(0.299) == 0
    assert step(0.3, x=0.5) == 0  # Short pause is not a stop.
    assert step(0.4) == 0
    assert step(0.6, z=0.5) == 0  # Turning is not a stop.
    state = StopState()
    assert step(10.0) == 0
    assert step(10.301) == 1
    assert step(1.0) == 0  # Gazebo clock reset starts a new interval.
    assert step(1.301) == 1
    state = StopState()
    assert step(0.0) == 0
    assert step(0.0) == 0  # Paused simulation does not accumulate stop time.
    state = StopState()
    assert step(5.0) == 0
    assert step(5.3) == 0  # Exact boundary at a nonzero start time.
    assert step(5.301) == 1
    print("selftest: PASS")


@dataclass
class GoalRecord:
    name: str
    result: str = "failed"
    duration: float = 0.0
    recoveries: int | None = None
    speed_count: int = 0
    speed_sum: float = 0.0
    max_speed: float = 0.0
    stops: int = 0
    cmd_count: int = 0
    distance_missing: bool = False

    @property
    def mean_speed(self):
        return self.speed_sum / self.speed_count if self.speed_count else math.nan


def parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--goals", nargs="+", choices=GOALS, default=list(GOALS))
    p.add_argument("--timeout", type=float, default=120.0,
                   help="wall seconds per goal (default: 120)")
    p.add_argument("--cmd-topic", default="/cmd_vel_safe",
                   help="final Twist topic; use /cmd_vel without collision monitor")
    p.add_argument("--stage", nargs=3, type=float, metavar=("X", "Y", "YAW"),
                   default=[1.2, 0.0, 0.0],
                   help="map-frame pose for the unmeasured staging goal (default: 1.2 0.0 0.0)")
    p.add_argument("--no-stage", action="store_true", help="skip the staging goal")
    p.add_argument("--selftest", action="store_true", help="check stop logic without ROS")
    return p


def print_row(row):
    maximum = row.max_speed if row.speed_count else math.nan
    recoveries = str(row.recoveries) if row.recoveries is not None else "unknown"
    stops = str(row.stops) if row.cmd_count and not row.distance_missing else "unknown"
    print(f"{row.name:<13} {row.result:<10} {row.duration:>9.2f} {recoveries:>10} "
          f"{maximum:>11.3f} {row.mean_speed:>12.3f} {stops:>7}", flush=True)


def summarize(rows):
    succeeded = sum(row.result == "succeeded" for row in rows)
    recoveries = sum(row.recoveries or 0 for row in rows)
    full_suite = len(rows) == 5 and {row.name for row in rows} == set(GOALS)
    feedback_known = all(row.recoveries is not None for row in rows
                         if row.result == "succeeded")
    passed = full_suite and succeeded >= 4 and recoveries == 0 and feedback_known
    verdict = ("PASS" if passed else "FAIL") if full_suite else "INCOMPLETE"
    recovery_text = str(recoveries) if feedback_known else "unknown"
    print(f"Navigation: {verdict} ({succeeded}/{len(rows)} succeeded, "
          f"{recovery_text} recoveries; acceptance requires >=4/5 and 0 recoveries)")
    sampled = [row.max_speed for row in rows if row.speed_count]
    maximum = max(sampled, default=math.nan)
    cruise = "PASS" if sampled and maximum >= 0.45 else "FAIL"
    print(f"Cruise: {cruise} (max {maximum:.3f} m/s; target >=0.45 m/s)")
    known_stops = all(row.cmd_count and not row.distance_missing for row in rows
                      if row.result != "failed" or row.duration > 0)
    total = sum(row.stops for row in rows)
    suffix = "" if known_stops else " (incomplete command/TF data)"
    print(f"Total mid-path stops: {total}{suffix}")
    return 0 if passed and cruise == "PASS" and known_stops and total == 0 else 1


def run_ros(argv):
    # Keep all ROS and package imports behind --selftest (and --help).
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.utilities import remove_ros_args
    from tf2_ros import Buffer, TransformException, TransformListener

    from go_to_location import load_locations

    p = parser()
    args = p.parse_args(remove_ros_args(argv))
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        p.error("--timeout must be a positive finite number")
    if len(set(args.goals)) != len(args.goals):
        p.error("--goals must contain distinct places")
    places = load_locations("locations_sim.yaml")

    class SimCheck(Node):
        def __init__(self):
            super().__init__("mode1_sim_check", parameter_overrides=[
                rclpy.Parameter("use_sim_time", value=True)])
            self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)
            self.row = None
            self.active = False
            self.target = None
            self.command = None
            self.stop_state = StopState()
            self.odom_seen = False
            self.goal_handle = None
            self.sequence_safe = True
            self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom,
                                     qos_profile_sensor_data)
            self.create_subscription(Twist, args.cmd_topic, self.on_command,
                                     qos_profile_sensor_data)
            self.create_timer(0.05, self.update_stops)

        def distance(self):
            try:
                pos = self.tf.lookup_transform("map", "base_link", Time()).transform.translation
            except TransformException:
                return None
            return math.hypot(pos.x - self.target[0], pos.y - self.target[1])

        def on_odom(self, msg):
            speed = abs(msg.twist.twist.linear.x)
            if not math.isfinite(speed):
                return
            self.odom_seen = True
            if self.active:
                self.row.speed_count += 1
                self.row.speed_sum += speed
                self.row.max_speed = max(self.row.max_speed, speed)

        def on_command(self, msg):
            if self.active:
                self.update_stops()  # Close the previous held command interval.
                self.row.cmd_count += 1
                self.command = (msg.linear.x, msg.angular.z)
                self.update_stops()

        def update_stops(self):
            if not self.active or self.command is None:
                return
            distance = self.distance()
            if distance is None:
                self.row.distance_missing = True
            now = self.get_clock().now().nanoseconds / 1e9
            self.stop_state, increment = count_stop(
                self.stop_state, now, *self.command, distance, self.active)
            self.row.stops += increment

        def feedback(self, msg, row):
            # Late feedback from a completed goal must not affect the next row.
            row.recoveries = max(row.recoveries or 0, msg.feedback.number_of_recoveries)

        def wait(self, future, deadline):
            # Like go_to_location's spin-until-complete, but bounded and keeps
            # odometry, command and TF callbacks running in this single thread.
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
            return future.done()

        def cancel(self, result):
            if self.goal_handle is None:
                return
            self.goal_handle.cancel_goal_async()
            # Never send the next goal until this one has terminated.
            self.sequence_safe = self.wait(result, time.monotonic() + 5.0)
            if not self.sequence_safe:
                self.get_logger().error("Cancellation not confirmed; stopping the check")

        def go_to(self, name, pose=None):
            self.row = GoalRecord(name)
            place = places[name] if pose is None else dict(zip(("x", "y", "yaw"), pose))
            self.target = (float(place["x"]), float(place["y"]))
            self.command = None
            self.stop_state = StopState()
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = self.target
            goal.pose.pose.orientation.z = math.sin(float(place["yaw"]) / 2)
            goal.pose.pose.orientation.w = math.cos(float(place["yaw"]) / 2)
            started = time.monotonic()
            deadline = started + args.timeout
            self.goal_handle = None
            result = None
            try:
                send = self.nav.send_goal_async(
                    goal, feedback_callback=lambda msg, row=self.row: self.feedback(msg, row))
                if not self.wait(send, deadline):
                    # A late acceptance must be canceled, not allowed to overlap
                    # another goal. Give discovery a bounded cancellation window.
                    self.row.result = "timeout"
                    if not self.wait(send, time.monotonic() + 5.0):
                        self.get_logger().error(
                            "Goal acceptance timed out; cancellation cannot be confirmed")
                        self.sequence_safe = False
                        return self.row
                self.goal_handle = send.result()
                if not self.goal_handle.accepted:
                    return self.row
                result = self.goal_handle.get_result_async()
                self.active = True
                if self.row.result == "timeout" or not self.wait(result, deadline):
                    self.row.result = "timeout"
                    self.cancel(result)
                else:
                    status = result.result().status
                    self.row.result = {
                        GoalStatus.STATUS_SUCCEEDED: "succeeded",
                        GoalStatus.STATUS_CANCELED: "canceled",
                    }.get(status, "failed")
            except KeyboardInterrupt:
                self.row.result = "canceled"
                self.sequence_safe = False
                if result is not None:
                    self.cancel(result)
                raise
            except Exception as exc:
                self.get_logger().error(f"Goal to {name} failed: {exc}")
                self.row.result = "failed"
                self.sequence_safe = False
                if result is not None and rclpy.ok():
                    self.cancel(result)
            finally:
                if rclpy.ok():
                    self.update_stops()
                self.active = False
                self.row.duration = time.monotonic() - started
            return self.row

    rclpy.init(args=argv)
    node = SimCheck()
    rows = []
    try:
        node.target = (0.0, 0.0)
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            publishers = node.get_publishers_info_by_topic(args.cmd_topic)
            twist_ready = any(pub.topic_type == "geometry_msgs/msg/Twist" for pub in publishers)
            if (node.nav.server_is_ready() and node.odom_seen and node.distance() is not None
                    and node.get_clock().now().nanoseconds > 0 and twist_ready):
                break
        else:
            print(f"Not ready: require NavigateToPose, {ODOM_TOPIC}, map->base_link, "
                  f"/clock and {args.cmd_topic} (Twist). "
                  "Use --cmd-topic /cmd_vel if collision monitor is disabled.", file=sys.stderr)
            return 2
        print(f"Command: {args.cmd_topic} (Twist); odometry: {ODOM_TOPIC}; "
              "distance: map->base_link TF", flush=True)
        if not args.no_stage:
            stage = node.go_to("stage", args.stage)
            print(f"stage: {stage.result} in {stage.duration:.1f} s", flush=True)
            if stage.result != "succeeded":
                return 1
        print(f"{'goal':<13} {'result':<10} {'seconds':>9} {'recoveries':>10} "
              f"{'max m/s':>11} {'mean m/s':>12} {'stops':>7}", flush=True)
        for name in args.goals:
            row = node.go_to(name)
            rows.append(row)
            print_row(row)
            if not node.sequence_safe:
                break
        return summarize(rows)
    except KeyboardInterrupt:
        if node.row is not None and node.row.name in args.goals and (not rows or rows[-1] is not node.row):
            rows.append(node.row)
            print_row(node.row)
        if rows:
            summarize(rows)
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
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
