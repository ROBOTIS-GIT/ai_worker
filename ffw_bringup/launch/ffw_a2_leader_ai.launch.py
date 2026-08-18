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
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
            'start_teleoperation_controller',
            default_value='true',
            description='Start the Cyclo A2 teleoperation controller.',
        ),
    ]

    description_file = LaunchConfiguration('description_file')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')

    start_teleoperation_controller = LaunchConfiguration('start_teleoperation_controller')
    # Robot controllers config file path
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ffw_bringup'),
            'config',
            'ffw_a2_leader',
            'ffw_a2_leader_ai_hardware_controller.yaml',
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
                [FindPackageShare('ffw_description'), 'urdf', 'ffw_a2_leader', description_file]
            ),
            ' ',
            'use_mock_hardware:=', use_mock_hardware,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_command_broadcaster',
            'spring_actuator_controller',
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

    teleoperation_config = PathJoinSubstitution(
        [
            FindPackageShare('ffw_bringup'),
            'config',
            'ffw_a2_leader',
            'ffw_a2_teleoperation.yaml',
        ]
    )
    follower_urdf_path = PathJoinSubstitution(
        [
            FindPackageShare('cyclo_motion_controller_models'),
            'models',
            'ai_worker',
            'ffw_sg2_follower.urdf',
        ]
    )
    follower_srdf_path = PathJoinSubstitution(
        [
            FindPackageShare('cyclo_motion_controller_models'),
            'models',
            'ai_worker',
            'ffw_sg2_follower_default.srdf',
        ]
    )
    teleoperation_node = Node(
        package='cyclo_teleoperation',
        executable='cyclo_teleoperation_node',
        name='cyclo_teleoperation',
        parameters=[
            teleoperation_config,
            {
                'follower_urdf_path': follower_urdf_path,
                'follower_srdf_path': follower_srdf_path,
                'leader_urdf_xml': robot_description_content,
            },
        ],
        output='screen',
        condition=IfCondition(start_teleoperation_controller),
    )
    # Return combined LaunchDescription
    return LaunchDescription(declared_arguments + [leader_with_namespace, teleoperation_node])
