"""Run startup checks before starting follower processes."""

import atexit
import errno
import fcntl
from functools import partial
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from trajectory_msgs.msg import JointTrajectory
import yaml


def _load_config(path):
    with open(path, encoding='utf-8') as file:
        return yaml.safe_load(file)['/**']


def gate_follower(leader_files, controller_files, remaps, timeout_sec=2.0):
    """Prevent duplicate launches and check command topics before startup."""
    lock_file = open('/tmp/ffw_follower.lock', 'a')
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as error:
        lock_file.close()
        if error.errno in (errno.EACCES, errno.EAGAIN):
            raise RuntimeError('[follower_initializer] Follower is already running') from error
        raise

    try:
        check_command_topics(leader_files, controller_files, remaps, timeout_sec=timeout_sec)
    except BaseException:
        lock_file.close()
        raise

    # Keep the file in place; deleting it could let another launch take a new lock.
    # ponytail: launch lifetime only; hardware must own the lock to cover orphan nodes.
    atexit.register(lock_file.close)


# -----------------------------------------------------------------------------
# Topic list
#
# Common:
#   /cmd_vel
#   /leader/joint_trajectory_command_broadcaster_{left,right}/joint_trajectory
#   /leader/joystick_controller_{left,right}/joint_trajectory
#   /arm_{l,r}_controller/joint_trajectory
#   /{head,lift}_controller/joint_trajectory
#
# Gripper models:
#   /gripper_{l,r}_controller/joint_trajectory
# HX5 models:
#   /hand_{l,r}_controller/joint_trajectory
#   /leader/joint_trajectory_command_broadcaster_{left,right}_hand/joint_trajectory
# Swerve models:
#   /swerve_steering_initial_position_controller/joint_trajectory
# -----------------------------------------------------------------------------
def check_command_topics(leader_files, controller_files, remaps, timeout_sec=2.0):
    """Check only at startup; later command sources are not blocked."""
    if not 0.0 < timeout_sec < float('inf'):
        raise ValueError('[follower_initializer] timeout_sec must be positive and finite')
    if not leader_files or not controller_files:
        raise ValueError('[follower_initializer] Leader and follower configs are required')

    # Collect command topics sent by the leaders.
    topics = {'/cmd_vel': Twist}
    for path in leader_files:
        config = _load_config(path)
        broadcaster = config['joint_trajectory_command_broadcaster']['ros__parameters']
        joystick = config['joystick_controller']['ros__parameters']
        for side in ('l', 'r'):
            topic = broadcaster[f'dynamixel_{side}_joint_trajectory_topic']
            topics[topic] = JointTrajectory
        sensors = joystick['joystick_sensors']
        if not isinstance(sensors, list) or not sensors:
            raise ValueError(f'[follower_initializer] Missing joystick_sensors in {path}')
        for sensor in sensors:
            topic = joystick[f'{sensor}_joint_trajectory_topic']
            topics[topic] = JointTrajectory

    # Include both original and remapped follower input topics.
    for path in controller_files:
        config = _load_config(path)
        spawn = config['controller_spawn']['ros__parameters']
        controller_names = list(spawn.get('controllers', []))
        for key in ('initial_controller', 'active_controller'):
            if key in spawn:
                controller_names.append(spawn[key])

        for name in controller_names:
            controller = config['controller_manager']['ros__parameters'][name]
            if controller['type'] != 'joint_trajectory_controller/JointTrajectoryController':
                continue
            topics[f'/{name}/joint_trajectory'] = JointTrajectory
            if name in remaps:
                topics[remaps[name]] = JointTrajectory

    # Validate the topics before starting ROS.
    for topic in topics:
        if not isinstance(topic, str) or not topic.startswith('/'):
            raise ValueError(
                '[follower_initializer] Command topics must be absolute ROS topic names')
    if topics['/cmd_vel'] is not Twist:
        raise ValueError('[follower_initializer] A trajectory topic conflicts with /cmd_vel')

    def reject_command(topic, _message=None):
        raise RuntimeError(f'[follower_initializer] Command source detected: {topic}')

    # Watch for publishers and incoming commands before allowing startup.
    context = Context()
    node = None
    try:
        rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node(
            'follower_initializer', context=context, use_global_arguments=False)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        for topic, msg_type in topics.items():
            node.create_subscription(msg_type, topic, partial(reject_command, topic), qos)

        node.get_logger().info(f'Checking {len(topics)} command topics for {timeout_sec:g}s')
        with SingleThreadedExecutor(context=context) as executor:
            executor.add_node(node)
            deadline = monotonic() + timeout_sec
            while True:
                if not context.ok():
                    raise RuntimeError('[follower_initializer] ROS context stopped')
                executor.spin_once(timeout_sec=0.05)
                for topic in topics:
                    if node.count_publishers(topic):
                        reject_command(topic)
                if monotonic() >= deadline:
                    break
        node.get_logger().info('No command sources detected; starting follower')
    finally:
        if node is not None:
            node.destroy_node()
        context.try_shutdown()
