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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'description_file',
            default_value='ffw_lg2_mini_leader.urdf.xacro',
            description='URDF/XACRO file for the robot model.',
        ),
        DeclareLaunchArgument(
            'launch_foot_switch',
            default_value='true',
            description='Whether to launch the foot switch node.',
        ),
    ]

    description_file = LaunchConfiguration('description_file')
    launch_foot_switch = LaunchConfiguration('launch_foot_switch')

    # Robot controllers config file path
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ffw_bringup'),
            'config',
            'ffw_lg2_mini_leader',
            'ffw_lg2_mini_leader_ai_hardware_controller.yaml',
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
                [FindPackageShare('ffw_description'), 'urdf', 'ffw_lg2_mini_leader', description_file]
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'joint_trajectory_command_broadcaster',
            'trigger_position_controller',
            'joystick_controller',
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
        condition=IfCondition(launch_foot_switch),
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
            'data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
        ],
    )

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
        ]
    )

    # Return combined LaunchDescription
    return LaunchDescription(declared_arguments + [leader_with_namespace, foot_switch_node])
