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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'description_file',
            default_value='ffw_lg2_leader.urdf.xacro',
            description='URDF/XACRO file for the robot model.',
        ),
        DeclareLaunchArgument(
            'use_hand_preset',
            default_value='false',
            description='Drive the SH5/BH5 follower hands from the leader gripper with presets.',
        ),
        DeclareLaunchArgument(
            'hand_presets_file',
            default_value='',
            description='Hand preset YAML. Empty uses the ffw_preset_controller default.',
        ),
        DeclareLaunchArgument(
            'hand_initial_preset',
            default_value='',
            description='Initial hand preset. Empty uses `default_preset` from the YAML.',
        ),
    ]

    description_file = LaunchConfiguration('description_file')

    # Robot controllers config file path
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ffw_bringup'),
            'config',
            'ffw_lg2_leader',
            'ffw_lg2_leader_ai_hardware_controller.yaml',
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
                [FindPackageShare('ffw_description'), 'urdf', 'ffw_lg2_leader', description_file]
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            # The '=' form keeps the spawner from parsing '-p' as its own --param-file option.
            [
                '--controller-ros-args=-p exclude_gripper_joints:=',
                LaunchConfiguration('use_hand_preset'),
            ],
            'joint_trajectory_command_broadcaster',
            'spring_actuator_controller_left',
            'spring_actuator_controller_right',
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

    # Wrap everything in a namespace 'leader'
    leader_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('leader'),
            control_node,
            robot_controller_spawner,
            robot_state_publisher_node,
        ]
    )

    hand_preset_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ffw_preset_controller'), 'launch',
                 'hand_preset_controller.launch.py']
            )
        ),
        launch_arguments={
            'params_file': PathJoinSubstitution(
                [FindPackageShare('ffw_bringup'), 'config', 'ffw_lg2_leader',
                 'ffw_lg2_hand_preset_controller.yaml']
            ),
            'presets_file': LaunchConfiguration('hand_presets_file'),
            'initial_preset': LaunchConfiguration('hand_initial_preset'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_hand_preset')),
    )

    # Return combined LaunchDescription
    return LaunchDescription(
        declared_arguments + [leader_with_namespace, hand_preset_controller]
    )
