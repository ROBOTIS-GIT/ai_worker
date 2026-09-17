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

import importlib
from pathlib import Path
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def detect_leader_port():
    candidates = {}
    for path in sorted(Path('/dev/serial/by-id').glob('*')):
        if path.exists() and any(name in path.name.upper() for name in (
            'STM32_VIRTUAL_COMPORT', 'ROBOTIS_AVATAR_CONTROLLER'
        )):
            candidates.setdefault(path.resolve(), path)
    if not candidates:
        raise RuntimeError('No leader port found. Connect the leader.')
    if len(candidates) > 1:
        found = ', '.join(str(path) for path in candidates.values())
        raise RuntimeError(
            f'Multiple leader ports found ({len(candidates)}). Connect only one leader.\n'
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
            f'ROBOTIS Avatar Controller identification failed on {path}: {exc}'
        ) from exc
    finally:
        if port is not None and port.is_open:
            port.closePort()
    return str(path)


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'description_file',
            default_value='ffw_a3.urdf.xacro',
            description='URDF/XACRO file for the robot model.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware mirroring command.',
        ),
        DeclareLaunchArgument(
            'enable_foot_switch',
            default_value='true',
            description='Whether to launch the foot switch node.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    description_file = LaunchConfiguration('description_file')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')

    if IfCondition(use_mock_hardware).evaluate(context):
        port_name = '/dev/null'
    else:
        # Temporary workaround for the missing pySerial dependency.
        # TODO: Remove once pySerial is included in the Docker image.
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
        port_name = detect_leader_port()

    enable_foot_switch = LaunchConfiguration('enable_foot_switch')

    # Robot controllers config file path
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ffw_bringup'),
            'config',
            'ffw_a3',
            'ffw_a3_ai_hardware_controller.yaml',
        ]
    )

    # ros2_control Node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('ffw_description'), 'urdf', 'ffw_a3', description_file]
            ),
            ' ',
            'use_mock_hardware:=', use_mock_hardware,
            ' ',
            'port_name:=', port_name,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_command_broadcaster',
            'trigger_position_controller',
            'joystick_controller',
            'joint_state_broadcaster',
        ],
        parameters=[robot_description],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'frame_prefix': 'leader_'}],
    )

    foot_switch_node = Node(
        package='ffw_bringup',
        executable='foot_switch_node',
        name='foot_switch_node',
        output='both',
        parameters=[{'controller_config_path': robot_controllers}],
        condition=IfCondition(enable_foot_switch),
    )

    gripper_trigger_node = Node(
        package='ffw_joint_trajectory_command_broadcaster',
        executable='gripper_trigger',
        name='gripper_trigger',
        output='both',
        parameters=[{'gripper_threshold': -1.0}],
    )

    # Execute process to publish position command
    position_command_process = ExecuteProcess(
        name='trigger_position_command',
        cmd=[
            'ros2', 'topic', 'pub',
            '-r', '50',
            '-t', '50',
            '-p', '50',
            '/leader/trigger_position_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            'data: [-0.15, -0.15]',
        ],
    )

    # Note: leader_position_controller commands are now continuously published by
    # joint_trajectory_command_broadcaster (mirrors follower poses every cycle).
    # No initial-kick process needed.

    delay_position_command_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[position_command_process],
        )
    )

    # Wrap everything in a namespace 'leader'
    leader_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('leader'),
            control_node,
            robot_controller_spawner,
            robot_state_publisher_node,
            delay_position_command_after_controllers,
            gripper_trigger_node,
        ]
    )

    return [leader_with_namespace, foot_switch_node]
