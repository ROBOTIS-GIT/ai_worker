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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'namespace', default_value='a3_left',
            description='Namespace for this A3 controller manager and joint states.',
        ),
        DeclareLaunchArgument(
            'port_name', default_value='/dev/ttyUSB0',
            description='Serial port for this A3 hand.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware', default_value='false',
            description='Use mock hardware instead of connecting to the hand.',
        ),
        DeclareLaunchArgument(
            'mock_sensor_commands', default_value='false',
            description='Enable mock sensor commands.',
        ),
        DeclareLaunchArgument(
            'start_rviz', default_value='false',
            description='Show finger TF frames in RViz.',
        ),
        DeclareLaunchArgument(
            'description_file', default_value='ffw_a3_leader_left.xacro',
            description='A3 wrapper Xacro containing the model and ros2_control system.',
        ),
    ]

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('ffw_description'), 'urdf', 'ffw_a3_leader',
            LaunchConfiguration('description_file'),
        ]),
        # Match robotis_hand_private joint names across URDF, hardware and TF.
        ' prefix:=a3_',
        ' port_name:=', LaunchConfiguration('port_name'),
        ' use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'),
        ' mock_sensor_commands:=', LaunchConfiguration('mock_sensor_commands'),
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
    }
    controller_config = PathJoinSubstitution([
        FindPackageShare('ffw_bringup'), 'config', 'ffw_a3_leader',
        'ffw_a3_left_leader_ai_hardware_controller.yaml',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('ffw_description'), 'rviz', 'ffw_a3_leader_left.rviz',
    ])

    return LaunchDescription(declared_arguments + [
        GroupAction(actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            Node(
                package='robot_state_publisher', executable='robot_state_publisher',
                parameters=[robot_description], output='both',
            ),
            Node(
                package='controller_manager', executable='ros2_control_node',
                parameters=[controller_config],
                output='both',
            ),
            Node(
                package='controller_manager', executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager',
                           'controller_manager'],
                output='both',
            ),
            Node(
                package='rviz2', executable='rviz2',
                arguments=['-d', rviz_config],
                condition=IfCondition(LaunchConfiguration('start_rviz')),
                output='both',
            ),
        ]),
    ])
