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

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import yaml


def generate_launch_description():
    controller_config_path = (
        Path(get_package_share_directory('ffw_bringup'))
        / 'config' / 'ffw_a2_leader' / 'ffw_a2_leader_ai_hardware_controller.yaml'
    )
    with controller_config_path.open(encoding='utf-8') as config_file:
        controller_config = yaml.safe_load(config_file)
    trigger_config = controller_config['/**']['trigger_position_controller']['ros__parameters']
    for key in ('left_neutral_position', 'right_neutral_position'):
        value = trigger_config.get(key) if isinstance(trigger_config, dict) else None
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f'{controller_config_path}: {key} must be a finite angle in radians')

    declared_arguments = [
        DeclareLaunchArgument(
            'description_file',
            default_value='ffw_a2_leader.urdf.xacro',
            description='URDF/XACRO file for the robot model.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware mirroring command.',
        ),
        DeclareLaunchArgument(
            'mock_sensor_commands',
            default_value='false',
            description='Expose mock joystick sensor command interfaces for testing.',
        ),
        DeclareLaunchArgument(
            'trigger_left_neutral_position',
            default_value=str(trigger_config['left_neutral_position']),
            description=(
                'Left trigger return position in radians, in the current joint coordinates.'
            ),
        ),
        DeclareLaunchArgument(
            'trigger_right_neutral_position',
            default_value=str(trigger_config['right_neutral_position']),
            description=(
                'Right trigger return position in radians, in the current joint coordinates.'
            ),
        ),
    ]

    description_file = LaunchConfiguration('description_file')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')

    robot_controllers = str(controller_config_path)

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
                [FindPackageShare('ffw_description'), 'urdf', 'ffw_a2_leader', description_file]
            ),
            ' ',
            'use_mock_hardware:=', use_mock_hardware,
            ' ', 'mock_sensor_commands:=', LaunchConfiguration('mock_sensor_commands'),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            '--controller-ros-args',
            '-r /reference/joint_trajectory_command_broadcaster_left/raw_joint_trajectory:='
            '/reference/left/joint',
            '--controller-ros-args',
            '-r /reference/joint_trajectory_command_broadcaster_right/raw_joint_trajectory:='
            '/reference/right/joint',
            '--controller-ros-args',
            '-r /leader/joystick_controller_right/joystick_mode:=/reference/joystick/mode',
            '--controller-ros-args',
            '-r /leader/joystick_controller/tact_trigger:=/reference/joystick/tact',
            'joint_trajectory_command_broadcaster',
            'trigger_position_controller',
            'joystick_controller',
            'joint_state_broadcaster',
        ],
        parameters=[robot_description],
    )

    # Set the return target once after successful controller activation.
    trigger_position_command = ExecuteProcess(
        name='trigger_position_command',
        cmd=[
            'ros2', 'topic', 'pub', '--once', '-w', '1',
            '/reference/trigger_position_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            ['{data: [', LaunchConfiguration('trigger_left_neutral_position'), ', ',
             LaunchConfiguration('trigger_right_neutral_position'), ']}'],
        ],
        output='screen',
    )

    def set_trigger_target_after_spawn(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg='A2 controller activation failed; trigger target was not sent.')]
        return [trigger_position_command]

    trigger_target_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=set_trigger_target_after_spawn,
        )
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'frame_prefix': 'leader_'}],
    )

    # Hardware state and raw references stay separate from public action/state outputs.
    leader_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('reference'),
            trigger_target_handler,
            control_node,
            robot_controller_spawner,
            robot_state_publisher_node,
        ]
    )

    return LaunchDescription(declared_arguments + [leader_with_namespace])
