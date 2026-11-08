#!/usr/bin/env python3
"""Measure clearance alongside an already running navigation check.

    python3 clearance_monitor.py --selftest
    ros2 run wheelchair_description clearance_monitor.py --duration 180 --out clearance.json \
        --ros-args -p use_sim_time:=true
    ros2 run wheelchair_description clearance_monitor.py --scan-topic /scan_fused \
        --ignore-box "-0.45 -0.30 -0.35 0.35"

Distances use the unpadded physical footprint, scan points projected into
base_link, and occupied map cell CENTRES (not cell edges or contact sensors).
The map search includes centres within 1.5 m of base_link. Ignore boxes affect
only scan points, including their boundaries. No automatic self-hit filtering.
Near-miss events mean clearance < --warn, including hits; hit events mean <= 0.
Each source counts entries once, attributed to the command state at entry.
Moving/stopped labels describe the last Twist, not measured wheel motion;
before a command arrives they are unknown. Duration/status use wall time;
measurement times use scan/TF stamps. Minimum poses use the latest map TF,
whose timestamp is recorded separately. Missing TF never invents an exit.
PASS means zero observed hit events; check coverage in the summary as well.
"""

import argparse
import ast
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time


DEFAULT_PARAMS = "src/wheelchair_navigation/config/nav2_params_3cam_v29.yaml"
BASE_FRAME = "base_link"
POSE_FRAME = "map"
MAP_TOPIC = "/map"
MAP_RADIUS = 1.5
OCCUPIED_THRESHOLD = 65


def parse_footprint(value):
    """Parse Nav2's string of [x, y] pairs into an unpadded polygon, purely."""
    if not isinstance(value, str):
        raise ValueError("footprint must be a string of [x, y] pairs")
    try:
        pairs = ast.literal_eval(value)
        if not isinstance(pairs, (list, tuple)):
            raise ValueError("expected a list of pairs")
        polygon = []
        for pair in pairs:
            if not isinstance(pair, (list, tuple)) or len(pair) != 2:
                raise ValueError("each footprint vertex must have two coordinates")
            x, y = map(float, pair)
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError("footprint coordinates must be finite")
            polygon.append((x, y))
    except (ValueError, TypeError, SyntaxError, OverflowError) as exc:
        raise ValueError(f"invalid footprint: {exc}") from exc
    if len(polygon) > 1 and polygon[0] == polygon[-1]:
        polygon.pop()
    if len(polygon) < 3 or len(set(polygon)) != len(polygon):
        raise ValueError("footprint needs at least three distinct vertices")
    area2 = sum(a[0] * b[1] - b[0] * a[1]
                for a, b in zip(polygon, polygon[1:] + polygon[:1]))
    if abs(area2) <= 1e-12:
        raise ValueError("footprint must enclose a nonzero area")
    return tuple(polygon)


def footprint_from_params(params):
    """Extract only local_costmap's footprint; deliberately ignore padding."""
    section = params["local_costmap"]
    if "local_costmap" in section:
        section = section["local_costmap"]
    return parse_footprint(section["ros__parameters"]["footprint"])


def point_to_polygon_signed_distance(point, polygon):
    """Euclidean distance to a simple polygon: negative inside, zero on edge.

    Ray casting works for either winding and for concave polygons. A 1e-12 m
    edge tolerance absorbs floating-point error when rotating an on-edge point.
    """
    px, py = point
    minimum_squared = math.inf
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        length_squared = dx * dx + dy * dy
        fraction = (((px - ax) * dx + (py - ay) * dy) / length_squared
                    if length_squared else 0.0)
        fraction = max(0.0, min(1.0, fraction))
        squared = (px - ax - fraction * dx) ** 2 + (py - ay - fraction * dy) ** 2
        minimum_squared = min(minimum_squared, squared)
        if (ay > py) != (by > py):
            crossing_x = ax + (py - ay) * dx / dy
            if px < crossing_x:
                inside = not inside
    distance = math.sqrt(minimum_squared)
    if distance <= 1e-12:
        return 0.0
    return -distance if inside else distance


def transform_polygon(polygon, x, y, yaw):
    """Place a base-frame polygon at an arbitrary planar pose, purely."""
    c, s = math.cos(yaw), math.sin(yaw)
    return tuple((x + c * px - s * py, y + s * px + c * py)
                 for px, py in polygon)


def quaternion_yaw(x, y, z, w):
    norm2 = x * x + y * y + z * z + w * w
    if not math.isfinite(norm2) or norm2 <= 0:
        raise ValueError("invalid transform quaternion")
    return math.atan2(2 * (w * z + x * y), norm2 - 2 * (y * y + z * z))


def scan_transform_xy(transform):
    """XY affine transform of lidar-plane points, including lidar roll/pitch."""
    q = transform.rotation
    norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
    if not math.isfinite(norm2) or norm2 <= 0:
        raise ValueError("invalid lidar transform quaternion")
    k = 2.0 / norm2
    return (1 - k * (q.y * q.y + q.z * q.z), k * (q.x * q.y - q.w * q.z),
            k * (q.x * q.y + q.w * q.z), 1 - k * (q.x * q.x + q.z * q.z),
            transform.translation.x, transform.translation.y)


def in_ignore_box(point, boxes):
    x, y = point
    return any(xmin <= x <= xmax and ymin <= y <= ymax
               for xmin, xmax, ymin, ymax in boxes)


@dataclass
class SampleStats:
    checks: int = 0
    points: int = 0
    empty_checks: int = 0
    near_misses: int = 0
    hits: int = 0
    minimum: dict | None = None

    def observe(self, distance, point_count, stamp, elapsed, pose, pose_stamp,
                near_entry, hit_entry):
        self.checks += 1
        self.points += point_count
        self.empty_checks += int(point_count == 0)
        self.near_misses += int(near_entry)
        self.hits += int(hit_entry)
        if math.isfinite(distance) and (self.minimum is None
                                       or distance < self.minimum["clearance_m"]):
            self.minimum = {"clearance_m": distance, "time_sec": stamp,
                            "elapsed_wall_sec": elapsed, "pose": pose,
                            "pose_time_sec": pose_stamp}

    def as_dict(self):
        return {"checks": self.checks, "points": self.points,
                "empty_checks": self.empty_checks, "near_misses": self.near_misses,
                "hits": self.hits, "minimum": self.minimum}


@dataclass
class SourceStats:
    groups: dict = field(default_factory=lambda: {
        name: SampleStats() for name in ("overall", "moving", "stopped", "unknown")})
    near_active: bool = False
    hit_active: bool = False

    def observe(self, distance, point_count, stamp, elapsed, pose, pose_stamp, motion, warn):
        near, hit = distance < warn, distance <= 0.0
        near_entry, hit_entry = near and not self.near_active, hit and not self.hit_active
        self.near_active, self.hit_active = near, hit
        for group in ("overall", motion):
            self.groups[group].observe(distance, point_count, stamp, elapsed, pose,
                                       pose_stamp, near_entry, hit_entry)

    def as_dict(self):
        return {name: stats.as_dict() for name, stats in self.groups.items()}


def parse_ignore_box(value):
    try:
        box = tuple(float(v) for v in value.split())
        if len(box) != 4 or not all(math.isfinite(v) for v in box):
            raise ValueError
        if box[0] > box[1] or box[2] > box[3]:
            raise ValueError
        return box
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            'expected "xmin xmax ymin ymax", finite values with min <= max') from exc


def parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--params", default=DEFAULT_PARAMS, help="Nav2 params YAML (unpadded footprint)")
    p.add_argument("--scan-topic", default="/scan", help="LaserScan; /scan_fused on the real chair")
    p.add_argument("--cmd-topic", default="/cmd_vel_safe", help="Twist for motion labels; empty to disable")
    p.add_argument("--warn", type=float, default=0.05, help="near-miss threshold in metres (default: 0.05)")
    p.add_argument("--ignore-box", type=parse_ignore_box, action="append", default=[],
                   metavar='"XMIN XMAX YMIN YMAX"', help="exclude known chair parts from scan; repeatable")
    p.add_argument("--duration", type=float, default=None, help="wall seconds; default: until stopped")
    p.add_argument("--out", help="optional final JSON path")
    p.add_argument("--selftest", action="store_true", help="assert geometry and events without ROS")
    return p


def selftest():
    polygon = parse_footprint("[[0.45,0.35],[0.45,-0.35],[-0.45,-0.35],[-0.45,0.35]]")
    distance = point_to_polygon_signed_distance
    assert math.isclose(distance((0.65, 0), polygon), 0.20, abs_tol=1e-12)
    assert math.isclose(distance((0.65, 0.55), polygon), math.hypot(0.2, 0.2))
    assert distance((0.45, 0.0), polygon) == 0.0
    assert distance((0.45, 0.35), polygon) == 0.0
    assert math.isclose(distance((0, 0), polygon), -0.35)
    assert math.isclose(distance((0, 0), polygon[::-1]), -0.35)
    # Rotate both the pose and test points, including a non-right-angle rotation.
    for yaw in (math.pi / 2, 0.73):
        rotated = transform_polygon(polygon, 2.0, -1.0, yaw)
        for point, expected in (((0.65, 0.0), 0.2), ((0.45, 0.0), 0.0), ((0, 0), -0.35)):
            placed = transform_polygon((point,), 2.0, -1.0, yaw)[0]
            assert math.isclose(distance(placed, rotated), expected, abs_tol=1e-12)
    concave = parse_footprint("[[0,0],[2,0],[2,1],[1,1],[1,2],[0,2]]")
    assert distance((0.5, 1.5), concave) < 0
    assert distance((1.5, 1.5), concave) > 0
    assert parse_footprint("[[0,0],[1,0],[0,1],[0,0]]") == ((0, 0), (1, 0), (0, 1))
    params = {"local_costmap": {"local_costmap": {"ros__parameters": {
        "footprint": str(list(polygon)), "footprint_padding": 10.0}}}}
    assert footprint_from_params(params) == polygon
    for invalid in ("[]", "[[0,0],[1,1]]", "[[0,0],[1,1],[2,2]]", "[[0,0],[1,0],[0]]",
                    "[[0,0],[1e309,0],[0,1]]", "this is not a polygon", [[0, 0]]):
        try:
            parse_footprint(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted invalid footprint: {invalid}")
    assert in_ignore_box((0, 0), [parse_ignore_box("-1 1 -1 1")])
    assert not in_ignore_box((2, 0), [parse_ignore_box("-1 1 -1 1")])
    source = SourceStats()
    pose = {"x": 1.0, "y": 2.0, "yaw": 0.5}
    sequence = [(0.05, "moving"), (0.04, "moving"), (0.02, "stopped"),
                (0.0, "stopped"), (-0.02, "moving"), (0.01, "moving"),
                (0.10, "stopped"), (-0.01, "moving"), (math.inf, "unknown")]
    for stamp, (clearance, motion) in enumerate(sequence):
        source.observe(clearance, int(math.isfinite(clearance)), float(stamp), float(stamp),
                       pose, float(stamp), motion, 0.05)
    assert source.groups["overall"].near_misses == 2
    assert source.groups["overall"].hits == 2
    assert source.groups["stopped"].hits == 1
    assert source.groups["moving"].hits == 1  # Changing motion inside a hit adds no event.
    assert source.groups["overall"].minimum["clearance_m"] == -0.02
    assert source.groups["overall"].minimum["time_sec"] == 4.0
    assert source.groups["overall"].minimum["pose"] == pose
    assert source.groups["stopped"].minimum["clearance_m"] == 0.0
    assert source.groups["unknown"].minimum is None
    json.dumps(source.as_dict(), allow_nan=False)
    print("selftest: PASS", flush=True)


def format_min(stats):
    minimum = stats["minimum"]
    return f'{minimum["clearance_m"]:.4f} m' if minimum is not None else "none"


def print_summary(summary, out):
    print("\n=== Clearance summary ===", flush=True)
    print(f'Duration: {summary["elapsed_wall_sec"]:.2f} wall s; stop: {summary["stop_reason"]}')
    print("Minima: measurement time in ROS seconds; poses in map (latest TF).")
    print("Near misses: clearance < warn, including hits; events assigned at entry.")
    for source, groups in summary["sources"].items():
        for motion, stats in groups.items():
            minimum = stats["minimum"]
            location = ""
            if minimum is not None:
                pose = minimum["pose"]
                pose_text = (f'({pose["x"]:.3f}, {pose["y"]:.3f}, {pose["yaw"]:.3f})'
                             if pose is not None else "unavailable")
                location = (f'; t={minimum["time_sec"]:.3f}, '
                            f'elapsed={minimum["elapsed_wall_sec"]:.2f}, pose={pose_text}')
            print(f'{source}/{motion}: min={format_min(stats)}, near_misses={stats["near_misses"]}, '
                  f'hits={stats["hits"]}, checks={stats["checks"]}, points={stats["points"]}{location}')
    print("Coverage: " + json.dumps(summary["coverage"], sort_keys=True))
    print("Scope: observed scan points and map cell centres within 1.5 m; review coverage.")
    write_failed = False
    if out:
        try:
            Path(out).write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n",
                                 encoding="utf-8")
            print(f"JSON: {out}")
        except (OSError, ValueError) as exc:
            print(f"Could not write JSON to {out}: {exc}", file=sys.stderr, flush=True)
            write_failed = True
    print(f'Clearance: {summary["verdict"]}', flush=True)
    return write_failed


def run_ros(argv):
    # Geometry, parsing, --help and --selftest need only the standard library.
    import numpy as np
    import yaml
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from rclpy.utilities import remove_ros_args
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformException, TransformListener

    p = parser()
    args = p.parse_args(remove_ros_args(argv))
    if not math.isfinite(args.warn) or args.warn <= 0:
        p.error("--warn must be positive and finite")
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        p.error("--duration must be positive and finite")
    try:
        with Path(args.params).open(encoding="utf-8") as stream:
            footprint = footprint_from_params(yaml.safe_load(stream))
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        p.error(f"cannot read local_costmap footprint from {args.params}: {exc}")

    def stamp_seconds(stamp):
        return float(stamp.sec) + stamp.nanosec / 1e9

    class ClearanceMonitor(Node):
        def __init__(self):
            super().__init__("clearance_monitor")
            self.started = time.monotonic()
            self.started_utc = datetime.now(timezone.utc).isoformat()
            self.sources = {name: SourceStats() for name in ("scan", "map")}
            self.motion = "unknown"
            self.coverage = {"scan_messages": 0, "command_messages": 0, "map_messages": 0,
                             "scan_tf_failures": 0, "map_tf_failures": 0,
                             "scan_pose_missing": 0, "map_pose_missing": 0,
                             "map_not_ready_checks": 0, "invalid_maps": 0,
                             "ignored_scan_points": 0}
            self.lidar_frame = None
            self.lidar_transform = None
            self.scan_frames = set()
            self.printed_closest = False
            self.map_frame = None
            self.map_points = None
            self.map_resolution = None
            self.warn_times = {}
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)
            self.create_subscription(LaserScan, args.scan_topic, self.on_scan, qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid, MAP_TOPIC, self.on_map, QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE))
            if args.cmd_topic:
                self.create_subscription(Twist, args.cmd_topic, self.on_command, qos_profile_sensor_data)
            self.create_timer(0.1, self.check_map)

        def warn(self, key, message):
            now = time.monotonic()
            if now - self.warn_times.get(key, -math.inf) >= 10.0:
                self.warn_times[key] = now
                self.get_logger().warning(message)

        def pose_from_tf(self, tf):
            t, q = tf.transform.translation, tf.transform.rotation
            return {"x": t.x, "y": t.y, "yaw": quaternion_yaw(q.x, q.y, q.z, q.w)}

        def map_pose(self):
            try:
                tf = self.tf.lookup_transform(POSE_FRAME, BASE_FRAME, Time())
                return self.pose_from_tf(tf), stamp_seconds(tf.header.stamp)
            except (TransformException, ValueError):
                return None, None

        def on_command(self, msg):
            values = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.coverage["command_messages"] += 1
            self.motion = ("moving" if any(abs(v) > 0.001 for v in values) else "stopped")
            if not all(math.isfinite(v) for v in values):
                self.motion = "unknown"

        def observe(self, source, distance, points, stamp, pose, pose_stamp):
            if pose is None:
                self.coverage[f"{source}_pose_missing"] += 1
                self.warn("pose", "map -> base_link TF unavailable; minima may have no map pose")
            self.sources[source].observe(distance, points, stamp, time.monotonic() - self.started,
                                         pose, pose_stamp, self.motion, args.warn)

        def on_scan(self, msg):
            self.coverage["scan_messages"] += 1
            frame = msg.header.frame_id.lstrip("/")
            self.scan_frames.add(frame)
            if frame != self.lidar_frame:
                self.lidar_frame, self.lidar_transform = frame, None
            try:
                if not frame:
                    raise ValueError("scan has no frame_id")
                if self.lidar_transform is None:
                    tf = self.tf.lookup_transform(BASE_FRAME, frame, Time())
                    self.lidar_transform = scan_transform_xy(tf.transform)
                    print(f"Scan TF cached: {frame} -> {BASE_FRAME}", flush=True)
            except (TransformException, ValueError) as exc:
                self.coverage["scan_tf_failures"] += 1
                self.warn("scan_tf", f"Cannot transform scan {frame} -> {BASE_FRAME}: {exc}")
                return
            a, b, c, d, tx, ty = self.lidar_transform
            points = []
            for index, r in enumerate(msg.ranges):
                if (not math.isfinite(r) or r < 0.05 or r < msg.range_min or r > msg.range_max):
                    continue
                angle = msg.angle_min + index * msg.angle_increment
                lx, ly = r * math.cos(angle), r * math.sin(angle)
                point = (a * lx + b * ly + tx, c * lx + d * ly + ty)
                clearance = point_to_polygon_signed_distance(point, footprint)
                ignored = in_ignore_box(point, args.ignore_box)
                points.append((clearance, point, ignored))
            if points and not self.printed_closest:
                print("Startup: 5 closest scan points to footprint in base_link (before ignore boxes):")
                for clearance, (x, y), ignored in sorted(points, key=lambda v: v[0])[:5]:
                    print(f"  x={x:.4f}, y={y:.4f}, clearance={clearance:.4f} m"
                          + (" [ignore-box]" if ignored else ""))
                print("", flush=True)
                self.printed_closest = True
            used = [clearance for clearance, _, ignored in points if not ignored]
            self.coverage["ignored_scan_points"] += len(points) - len(used)
            pose, pose_stamp = self.map_pose()
            self.observe("scan", min(used, default=math.inf), len(used),
                         stamp_seconds(msg.header.stamp), pose, pose_stamp)

        def on_map(self, msg):
            self.coverage["map_messages"] += 1
            if self.map_points is not None:
                return  # Static map: cache its occupied centres once.
            info = msg.info
            try:
                if (info.width <= 0 or info.height <= 0 or not math.isfinite(info.resolution)
                        or info.resolution <= 0 or len(msg.data) != info.width * info.height):
                    raise ValueError("invalid grid dimensions/resolution/data length")
                q = info.origin.orientation
                yaw = quaternion_yaw(q.x, q.y, q.z, q.w)
                grid = np.asarray(msg.data).reshape(info.height, info.width)
                rows, cols = np.nonzero(grid >= OCCUPIED_THRESHOLD)
                x = (cols + 0.5) * info.resolution
                y = (rows + 0.5) * info.resolution
                c, s = math.cos(yaw), math.sin(yaw)
                self.map_points = np.column_stack((info.origin.position.x + c * x - s * y,
                                                   info.origin.position.y + s * x + c * y))
            except ValueError as exc:
                self.coverage["invalid_maps"] += 1
                self.warn("map", f"Cannot cache occupied map centres: {exc}")
                return
            self.map_frame = msg.header.frame_id.lstrip("/") or POSE_FRAME
            self.map_resolution = info.resolution
            print(f"Map cached: {len(self.map_points)} occupied cell centres in {self.map_frame}, "
                  f"resolution={info.resolution:.4f} m", flush=True)

        def check_map(self):
            if self.map_points is None:
                self.coverage["map_not_ready_checks"] += 1
                self.warn("map_missing", "Waiting for transient-local /map")
                return
            try:
                tf = self.tf.lookup_transform(self.map_frame, BASE_FRAME, Time())
                grid_pose = self.pose_from_tf(tf)
            except (TransformException, ValueError) as exc:
                self.coverage["map_tf_failures"] += 1
                self.warn("map_tf", f"Cannot transform {self.map_frame} -> {BASE_FRAME}: {exc}")
                return
            x, y = grid_pose["x"], grid_pose["y"]
            # Cheap axis-aligned filter before the circular filter and geometry.
            nearby = self.map_points[(np.abs(self.map_points[:, 0] - x) <= MAP_RADIUS)
                                     & (np.abs(self.map_points[:, 1] - y) <= MAP_RADIUS)]
            nearby = nearby[(nearby[:, 0] - x) ** 2 + (nearby[:, 1] - y) ** 2 <= MAP_RADIUS ** 2]
            polygon = transform_polygon(footprint, x, y, grid_pose["yaw"])
            minimum = min((point_to_polygon_signed_distance(point, polygon) for point in nearby),
                          default=math.inf)
            stamp = stamp_seconds(tf.header.stamp)
            pose, pose_stamp = ((grid_pose, stamp) if self.map_frame == POSE_FRAME else self.map_pose())
            self.observe("map", minimum, len(nearby), stamp, pose, pose_stamp)

        def status(self):
            groups = {name: source.groups["overall"].as_dict() for name, source in self.sources.items()}
            print(f'Clearance status: min scan={format_min(groups["scan"])}, '
                  f'min map={format_min(groups["map"])}, '
                  f'hits scan={groups["scan"]["hits"]} map={groups["map"]["hits"]}', flush=True)

        def summary(self, reason, error):
            hits = sum(source.groups["overall"].hits for source in self.sources.values())
            return {"verdict": "PASS" if hits == 0 else "FAIL", "stop_reason": reason,
                    "runtime_error": error, "started_utc": self.started_utc,
                    "elapsed_wall_sec": time.monotonic() - self.started,
                    "config": {"params": str(Path(args.params).resolve()), "footprint": footprint,
                               "base_frame": BASE_FRAME, "pose_frame": POSE_FRAME,
                               "scan_topic": args.scan_topic, "scan_frames": sorted(self.scan_frames),
                               "map_topic": MAP_TOPIC, "map_frame": self.map_frame,
                               "map_resolution_m": self.map_resolution,
                               "occupied_cells": len(self.map_points) if self.map_points is not None else None,
                               "cmd_topic": args.cmd_topic or None, "warn_m": args.warn,
                               "ignore_boxes": args.ignore_box, "map_radius_m": MAP_RADIUS,
                               "occupancy_threshold": OCCUPIED_THRESHOLD,
                               "near_misses_include_hits": True,
                               "motion_labels": "last commanded Twist; unknown before first command",
                               "event_attribution": "motion state at entry; source continuity across motion changes"},
                    "coverage": dict(self.coverage),
                    "sources": {name: source.as_dict() for name, source in self.sources.items()}}

    stopped = [None]

    def on_signal(signum, _frame):
        stopped[0] = signal.Signals(signum).name

    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    old_handlers = {}
    error = None
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, on_signal)
        node = ClearanceMonitor()
        print(f"Footprint (unpadded, base_link): {footprint}; params: {args.params}")
        print(f"Inputs: scan={args.scan_topic}, map={MAP_TOPIC} (transient_local), "
              f"cmd={args.cmd_topic or 'disabled'} (Twist); pose TF: map -> base_link")
        print(f"Thresholds: warn <{args.warn:.3f} m, hit <=0 m; map centres >=65 within 1.5 m; "
              f"scan ranges >=0.05 m; ignore boxes={args.ignore_box}", flush=True)
        next_status = node.started + 10.0
        while rclpy.ok() and stopped[0] is None:
            now = time.monotonic()
            if args.duration is not None and now - node.started >= args.duration:
                stopped[0] = "duration"
                break
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now >= next_status:
                node.status()
                next_status = now + 10.0
    except (KeyboardInterrupt, ExternalShutdownException):
        stopped[0] = stopped[0] or "ROS shutdown/interrupted"
    except Exception as exc:
        error = str(exc)
        stopped[0] = "runtime error"
        print(f"Clearance monitor error: {exc}", file=sys.stderr, flush=True)
    finally:
        summary = node.summary(stopped[0] or "ROS shutdown", error) if node is not None else None
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    if summary is None:
        return 2
    write_failed = print_summary(summary, args.out)
    return 2 if error or write_failed else int(summary["verdict"] == "FAIL")


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
