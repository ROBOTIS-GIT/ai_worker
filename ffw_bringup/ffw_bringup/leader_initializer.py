#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Sungho Woo, Woojin Wie, Wonho Yun

import atexit
import errno
import fcntl
import importlib
from math import isfinite
from pathlib import Path
import subprocess
from time import monotonic
import xml.etree.ElementTree as ET

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions


def detect_leader_port(use_mock_hardware=False):
    if use_mock_hardware:
        return '/dev/null'

    # Install pySerial if missing
    # TODO: Remove when pySerial is included in the Docker image
    try:
        importlib.import_module('serial')
    except ModuleNotFoundError as exc:
        if exc.name != 'serial':
            raise
        print('pySerial is missing; installing python3-serial...', flush=True)
        subprocess.run(['apt-get', 'update'], check=True, timeout=120)
        subprocess.run(
            ['apt-get', 'install', '-y', 'python3-serial'], check=True, timeout=120
        )
        importlib.invalidate_caches()
        importlib.import_module('serial')

    candidates = {}
    for path in sorted(Path('/dev/serial/by-id').glob('*')):
        if path.exists() and any(name in path.name.upper() for name in (
            'STM32_VIRTUAL_COMPORT', 'ROBOTIS_AVATAR_CONTROLLER'
        )):
            candidates.setdefault(path.resolve(), path)
    if not candidates:
        raise RuntimeError('[leader_initializer] No leader port found. Connect the leader.')
    if len(candidates) > 1:
        found = ', '.join(str(path) for path in candidates.values())
        raise RuntimeError(
            f'[leader_initializer] Multiple leader ports found ({len(candidates)}). '
            'Connect only one leader.\n'
            f'Detected ports: {found}'
        )
    path = next(iter(candidates.values()))
    if 'STM32_VIRTUAL_COMPORT' in path.name.upper():
        return str(path)

    port = None
    try:
        from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

        port = PortHandler(str(path))
        packet = PacketHandler(2.0)
        if not port.openPort():
            raise RuntimeError('ROBOTIS Avatar Controller: could not open port')
        if not port.setBaudRate(4_000_000):
            raise RuntimeError('ROBOTIS Avatar Controller: could not set baud rate to 4000000')
        data, result, error = packet.readTxRx(port, 200, 10001, 12)
        if result != COMM_SUCCESS:
            raise RuntimeError(f'ROBOTIS Avatar Controller: {packet.getTxRxResult(result)}')
        if error:
            raise RuntimeError(f'ROBOTIS Avatar Controller: {packet.getRxPacketError(error)}')
        if len(data) != 12:
            raise RuntimeError(
                f'ROBOTIS Avatar Controller: expected 12 name bytes, received {len(data)}'
            )
        name = bytes(data).split(b'\x00', 1)[0].decode('ascii')
        if name != 'LEADER_A2':
            raise RuntimeError(f'ROBOTIS Avatar Controller: expected LEADER_A2, received {name!r}')
    except Exception as exc:
        raise RuntimeError(
            f'[leader_initializer] ROBOTIS Avatar Controller identification failed on {path}: {exc}'
        ) from exc
    finally:
        if port is not None and port.is_open:
            port.closePort()
    return str(path)


def read_follower_urdf(node, timeout_sec=2.0):
    """Read names, types, and limits for ros2_control joints"""
    from rclpy.qos import DurabilityPolicy
    from rclpy.wait_for_message import wait_for_message
    from std_msgs.msg import String

    if not 0.0 < timeout_sec < float('inf'):
        raise ValueError('[leader_initializer] timeout_sec must be positive and finite')

    # Receive the last published URDF
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    node.get_logger().info('Waiting for follower URDF on /robot_description...')
    received, message = wait_for_message(
        String, node, '/robot_description', qos_profile=qos, time_to_wait=timeout_sec)
    if not received:
        raise RuntimeError(
            '[leader_initializer] Failed to receive follower URDF on /robot_description '
            f'(timeout: {timeout_sec}s)')
    if not message.data.strip():
        raise RuntimeError('[leader_initializer] Follower robot_description is empty')

    # Parse URDF XML
    root = ET.fromstring(message.data)
    if root.tag != 'robot':
        raise ValueError('[leader_initializer] Follower URDF root must be <robot>')

    # Read joint names under ros2_control
    joint_names = [joint.get('name', '') for joint in root.findall('ros2_control/joint')]
    if not joint_names or not all(joint_names):
        raise ValueError('[leader_initializer] Follower ros2_control joint names are missing')
    if len(set(joint_names)) != len(joint_names):
        raise ValueError('[leader_initializer] Duplicate follower ros2_control joint names')

    joint_types = {}
    joint_limits = {}
    # Extract only ros2_control joints from URDF; skip others
    for joint in root.findall('joint'):
        name = joint.attrib['name']
        if name not in joint_names:
            continue
        joint_type = joint.get('type', '')
        if not joint_type:
            raise ValueError(
                f'[leader_initializer] Follower joint type is missing: {name}')
        if name in joint_types:
            raise ValueError(f'[leader_initializer] Duplicate follower URDF joint: {name}')
        joint_types[name] = joint_type
        limit = joint.find('limit')
        joint_limits[name] = (
            {bound: float(value) for bound, value in limit.attrib.items()}
            if limit is not None else {}
        )

    missing_joints = set(joint_names).difference(joint_types)
    if missing_joints:
        raise ValueError(
            '[leader_initializer] Follower URDF missing joints: '
            f'{", ".join(sorted(missing_joints))}')

    return {
        'follower_urdf': message.data,
        'follower_joint_names': joint_names,
        'follower_joint_types': joint_types,
        'follower_joint_limits': joint_limits,
    }


def read_follower_controller(node, timeout_sec=2.0):
    """Read follower controllers and return left/right end tools"""
    from controller_manager_msgs.srv import ListControllers

    if not 0.0 < timeout_sec < float('inf'):
        raise ValueError('[leader_initializer] timeout_sec must be positive and finite')

    service_name = '/controller_manager/list_controllers'
    client = node.create_client(ListControllers, service_name)

    node.get_logger().info(f'Waiting for follower controllers on {service_name}...')
    deadline = monotonic() + timeout_sec
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(
            f'[leader_initializer] Follower controller service unavailable: {service_name} '
            f'(timeout: {timeout_sec}s)')
    with SingleThreadedExecutor(context=node.context) as executor:
        executor.add_node(node)
        future = client.call_async(ListControllers.Request())
        executor.spin_until_future_complete(
            future, timeout_sec=max(0.0, deadline - monotonic()))
    if not future.done():
        raise RuntimeError(
            f'[leader_initializer] Failed to receive follower controllers from {service_name} '
            f'(timeout: {timeout_sec}s)')

    # Find left and right end tools
    controllers = {controller.name for controller in future.result().controller}
    follower_end_tool = {}
    for side, suffix in (('left', 'l'), ('right', 'r')):
        candidates = [
            tool for tool in ('hand', 'gripper')
            if f'{tool}_{suffix}_controller' in controllers
        ]
        if len(candidates) > 1:
            raise RuntimeError(
                f'[leader_initializer] Cannot determine {side} follower end tool: {candidates}')
        follower_end_tool[side] = candidates[0] if candidates else 'unknown'
        if not candidates:
            node.get_logger().warning(f'Unknown {side} follower end tool')
    return follower_end_tool


def read_follower_joint_states(node, follower_joint_names, timeout_sec=2.0, velocity_threshold=0.1):
    """Read positions when all required joints are present and reported velocities are low"""
    from sensor_msgs.msg import JointState

    if not 0.0 < timeout_sec < float('inf'):
        raise ValueError('[leader_initializer] timeout_sec must be positive and finite')
    if not 0.0 <= velocity_threshold < float('inf'):
        raise ValueError('[leader_initializer] velocity_threshold must be non-negative and finite')
    required_joint_names = set(follower_joint_names)
    if not required_joint_names:
        raise ValueError('[leader_initializer] Follower joint names are missing')

    message = None
    messages = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(JointState, '/joint_states', messages.append, qos)
    node.get_logger().info('Waiting for follower joints to stop on /joint_states...')
    with SingleThreadedExecutor(context=node.context) as executor:
        executor.add_node(node)
        deadline = monotonic() + timeout_sec
        while True:
            remaining = deadline - monotonic()
            if remaining <= 0.0:
                break
            executor.spin_once(timeout_sec=remaining)
            if not messages:
                continue
            message = messages.pop()

            if not message.name or not (
                len(message.name) == len(message.position) == len(message.velocity)
            ):
                raise ValueError(
                    '[leader_initializer] Follower joint states require names, positions, '
                    'and velocities')
            if not all(message.name) or len(set(message.name)) != len(message.name):
                raise ValueError(
                    '[leader_initializer] Follower joint names must be non-empty and unique')
            if not all(isfinite(value) for value in message.position) or not all(
                isfinite(value) for value in message.velocity
            ):
                raise ValueError(
                    '[leader_initializer] Follower joint positions and velocities must be finite')

            # Check all ros2_control joints
            missing_joints = required_joint_names.difference(message.name)
            if missing_joints:
                raise RuntimeError(
                    '[leader_initializer] Follower joint states missing joints: '
                    f'{", ".join(sorted(missing_joints))}')
            if all(abs(velocity) <= velocity_threshold for velocity in message.velocity):
                return dict(zip(message.name, message.position))

    if message is None:
        raise RuntimeError(
            '[leader_initializer] No follower joint states received on /joint_states '
            f'(timeout: {timeout_sec}s)')
    raise RuntimeError(
        '[leader_initializer] Follower is still moving '
        f'(velocity threshold: {velocity_threshold}, timeout: {timeout_sec}s)')


def initialize_leader(use_mock_hardware=False, detect_port=True):
    """Prevent duplicate launches and collect leader startup information."""
    lock_file = open('/tmp/ffw_leader.lock', 'a')
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as error:
        lock_file.close()
        if error.errno in (errno.EACCES, errno.EAGAIN):
            raise RuntimeError('[leader_initializer] Leader is already running') from error
        raise

    try:
        init_info = {}
        if detect_port:
            init_info['port_name'] = detect_leader_port(use_mock_hardware)

        # Share one ROS context and node
        context = Context()
        node = None
        try:
            rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
            node = rclpy.create_node(
                'leader_initializer', context=context, use_global_arguments=False)
            init_info.update(read_follower_urdf(node))
            init_info['follower_end_tool'] = read_follower_controller(node)
            init_info['follower_current_position'] = read_follower_joint_states(
                node, init_info['follower_joint_names'])
        finally:
            if node is not None:
                node.destroy_node()
            context.try_shutdown()
    except BaseException:
        lock_file.close()
        raise

    # Keep the lock file; using a different one allows duplicate launches.
    # Lock ends with launch, even if child nodes remain.
    atexit.register(lock_file.close)
    return init_info
