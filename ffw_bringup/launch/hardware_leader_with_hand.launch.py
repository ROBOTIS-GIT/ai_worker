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
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'description_file',
            default_value='ffw_leader.urdf.xacro',
            description='URDF/XACRO file for the robot model.',
        ),
    ]

    description_file = LaunchConfiguration('description_file')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('ffw_description'), 'urdf', 'leader', description_file
        ]),
    ])
    robot_description = {'robot_description': robot_description_content}

    controller_config = PathJoinSubstitution([
        FindPackageShare('ffw_bringup'),
        'config',
        'leader_with_hand_hardware_controller.yaml',
    ])

    control_node = GroupAction([
        PushRosNamespace('leader'),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controller_config],
            output='screen'
        )
    ])

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/leader/controller_manager'],
        output='screen'
    )

    left_arm_spawner = GroupAction([
        PushRosNamespace('leader_left'),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_left', '-c', '/leader/controller_manager'],
            output='screen'
        )
    ])

    right_arm_spawner = GroupAction([
        PushRosNamespace('leader_right'),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_right', '-c', '/leader/controller_manager'],
            output='screen'
        )
    ])

    left_hand_spawner = GroupAction([
        PushRosNamespace('leader_left_hand'),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_left_hand', '-c', '/leader/controller_manager'],
            output='screen'
        )
    ])

    right_hand_spawner = GroupAction([
        PushRosNamespace('leader_right_hand'),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_right_hand', '-c', '/leader/controller_manager'],
            output='screen'
        )
    ])

    state_publisher = GroupAction([
        PushRosNamespace('leader'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        )
    ])

    return LaunchDescription(declared_arguments + [
        control_node,
        joint_state_spawner,
        left_arm_spawner,
        right_arm_spawner,
        left_hand_spawner,
        right_hand_spawner,
        state_publisher
    ])
