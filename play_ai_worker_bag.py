#!/usr/bin/env python3
"""Play a ROS 2 bag with a smooth arm transition at the start of every loop."""

import argparse
import pathlib
import sys
import time

import rclpy
import rosbag2_py
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from trajectory_msgs.msg import JointTrajectory


TRAJECTORY_TYPE = "trajectory_msgs/msg/JointTrajectory"


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Play a ROS 2 bag after moving both arms to the bag's initial "
            "trajectory over a configurable transition time."
        )
    )
    parser.add_argument("bag", help="Bag directory, .mcap, or .db3 path")
    parser.add_argument(
        "--left-topic",
        help="Left arm JointTrajectory topic (default: auto-detect)",
    )
    parser.add_argument(
        "--right-topic",
        help="Right arm JointTrajectory topic (default: auto-detect)",
    )
    parser.add_argument(
        "--transition-time",
        type=float,
        default=5.0,
        help="Initial arm transition duration in seconds (default: 5.0)",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Repeat forever; the transition is applied at every loop",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="Playback rate after the initial transition (default: 1.0)",
    )
    return parser.parse_args()


def storage_id(path):
    suffix = pathlib.Path(path).suffix.lower()
    if suffix == ".mcap":
        return "mcap"
    if suffix == ".db3":
        return "sqlite3"
    # For a bag directory, rosbag2 selects the plugin using metadata.yaml.
    return ""


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id(path)),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def detect_arm_topics(topic_types, left_override, right_override):
    trajectory_topics = [
        name for name, type_name in topic_types.items()
        if type_name == TRAJECTORY_TYPE
    ]

    def detect(side, override):
        if override:
            if topic_types.get(override) != TRAJECTORY_TYPE:
                raise RuntimeError(
                    f"{override!r} is not a {TRAJECTORY_TYPE} topic in the bag"
                )
            return override
        # AI Worker names the arm broadcasters "..._left/..." and
        # "..._right/..." (without the word "arm").  Select by side and
        # explicitly exclude the separate hand/gripper trajectory topics.
        hand_markers = ("hand", "gripper", "finger")
        candidates = [
            name for name in trajectory_topics
            if side in name.lower()
            and not any(marker in name.lower() for marker in hand_markers)
        ]
        if len(candidates) != 1:
            raise RuntimeError(
                f"Could not uniquely detect the {side} arm topic. "
                f"Candidates: {trajectory_topics}. Use --{side}-topic."
            )
        return candidates[0]

    left = detect("left", left_override)
    right = detect("right", right_override)
    if left == right:
        raise RuntimeError("Left and right trajectory topics must be different")
    return left, right


def qos_for_topic(topic):
    if topic == "/tf_static":
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
    # This is compatible with the normal reliable command subscribers and with
    # best-effort sensor subscribers.
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def duration_ns(duration):
    return duration.sec * 1_000_000_000 + duration.nanosec


def set_duration(duration, nanoseconds):
    duration.sec = nanoseconds // 1_000_000_000
    duration.nanosec = nanoseconds % 1_000_000_000


def retime_initial_trajectory(msg, seconds):
    if not msg.points:
        raise RuntimeError("The initial arm trajectory contains no points")
    # A recorded absolute stamp may be stale. Zero means "start now" to a
    # JointTrajectory controller.
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 0
    target_ns = round(seconds * 1_000_000_000)
    old_end_ns = duration_ns(msg.points[-1].time_from_start)
    if old_end_ns > 0:
        for point in msg.points:
            old_ns = duration_ns(point.time_from_start)
            set_duration(
                point.time_from_start,
                round(old_ns * target_ns / old_end_ns),
            )
    else:
        # A zero-duration multi-point trajectory has no meaningful timing.
        # Spread its points uniformly while preserving their order.
        count = len(msg.points)
        for index, point in enumerate(msg.points, start=1):
            set_duration(point.time_from_start, round(target_ns * index / count))
    set_duration(msg.points[-1].time_from_start, target_ns)
    return msg


def spin_sleep(node, seconds):
    deadline = time.monotonic() + max(0.0, seconds)
    while rclpy.ok():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        rclpy.spin_once(node, timeout_sec=min(0.05, remaining))


def inspect_bag(path, left_override, right_override):
    reader = open_reader(path)
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    left_topic, right_topic = detect_arm_topics(
        topic_types, left_override, right_override
    )
    initial = {}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic in (left_topic, right_topic) and topic not in initial:
            initial[topic] = (data, timestamp)
        if len(initial) == 2:
            break
    missing = {left_topic, right_topic} - initial.keys()
    if missing:
        raise RuntimeError(
            "No trajectory message found for: " + ", ".join(sorted(missing))
        )
    return topic_types, left_topic, right_topic, initial


def play_cycle(node, path, initial, topic_types, publishers, left_topic,
               right_topic, transition_time, rate):
    node.get_logger().info(
        f"Moving to bag start pose over {transition_time:.3f} s "
        f"({left_topic}, {right_topic})"
    )
    for topic in (left_topic, right_topic):
        msg = deserialize_message(initial[topic][0], JointTrajectory)
        publishers[topic].publish(retime_initial_trajectory(msg, transition_time))
    spin_sleep(node, transition_time)
    if not rclpy.ok():
        return

    # Do not replay anything preceding the later of the two initial arm
    # commands. Those records belong to bag setup before the safe start pose.
    start_timestamp = max(value[1] for value in initial.values())
    reader = open_reader(path)
    skipped_initial = set()
    bag_zero = None
    wall_zero = None
    while reader.has_next() and rclpy.ok():
        topic, data, timestamp = reader.read_next()
        if timestamp < start_timestamp:
            continue
        if (
            topic in initial
            and topic not in skipped_initial
            and timestamp == initial[topic][1]
        ):
            skipped_initial.add(topic)
            continue
        if bag_zero is None:
            node.get_logger().info("Initial arm transition complete; playing bag")
            bag_zero = timestamp
            wall_zero = time.monotonic()
        target = wall_zero + (timestamp - bag_zero) / 1e9 / rate
        spin_sleep(node, target - time.monotonic())
        if not rclpy.ok():
            return
        msg = deserialize_message(data, get_message(topic_types[topic]))
        publishers[topic].publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)


def main():
    args = parse_args()
    if args.transition_time <= 0:
        raise SystemExit("--transition-time must be greater than zero")
    if args.rate <= 0:
        raise SystemExit("--rate must be greater than zero")

    bag_path = pathlib.Path(args.bag).expanduser().resolve()
    if not bag_path.exists():
        raise SystemExit(f"Bag does not exist: {bag_path}")

    rclpy.init()
    node = rclpy.create_node("ai_worker_smooth_bag_player")
    try:
        topic_types, left_topic, right_topic, initial = inspect_bag(
            bag_path, args.left_topic, args.right_topic
        )
        publishers = {
            topic: node.create_publisher(
                get_message(type_name), topic, qos_for_topic(topic)
            )
            for topic, type_name in topic_types.items()
        }
        node.get_logger().info(
            f"Arm topics: {left_topic}, {right_topic}"
        )
        # Allow DDS endpoint discovery before publishing the one-shot initial
        # commands; otherwise a just-started player can lose them.
        spin_sleep(node, 1.0)
        while rclpy.ok():
            play_cycle(
                node, bag_path, initial, topic_types, publishers, left_topic,
                right_topic, args.transition_time, args.rate,
            )
            if not args.loop:
                break
    except (RuntimeError, KeyboardInterrupt) as error:
        if not isinstance(error, KeyboardInterrupt):
            node.get_logger().error(str(error))
            return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
