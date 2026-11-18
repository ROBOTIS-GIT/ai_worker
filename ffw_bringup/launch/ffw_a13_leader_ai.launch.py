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
            'baud_rate', default_value='2000000', description='Shared bus baud rate.',
        ),
        DeclareLaunchArgument(
            'update_rate', default_value='60', description='Controller update frequency in Hz.',
        ),
        DeclareLaunchArgument(
            'namespace', default_value='leader',
            description='Namespace for this A13 controller manager and joint states.',
        ),
        DeclareLaunchArgument(
            'port_name', default_value='/dev/ttyACM0',
            description='Serial port for the shared A13 bus.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware', default_value='false',
            description='Use mock hardware instead of connecting to the leader.',
        ),
        DeclareLaunchArgument(
            'mock_sensor_commands', default_value='false',
            description='Enable mock sensor commands.',
        ),
        DeclareLaunchArgument(
            'start_rviz', default_value='false',
            description='Show arm and finger TF frames in RViz.',
        ),
        DeclareLaunchArgument(
            'description_file', default_value='ffw_a13_leader.urdf.xacro',
            description='A13 wrapper Xacro containing the model and ros2_control system.',
        ),
    ]

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('ffw_description'), 'urdf', 'ffw_a13_leader',
            LaunchConfiguration('description_file'),
        ]),
        ' port_name:=', LaunchConfiguration('port_name'),
        ' baud_rate:=', LaunchConfiguration('baud_rate'),
        ' use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'),
        ' mock_sensor_commands:=', LaunchConfiguration('mock_sensor_commands'),
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
    }
    controller_config = PathJoinSubstitution([
        FindPackageShare('ffw_bringup'), 'config', 'ffw_a13_leader',
        'ffw_a13_leader_ai_hardware_controller.yaml',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('ffw_description'), 'rviz', 'ffw_a13_leader.rviz',
    ])

    return LaunchDescription(declared_arguments + [
        GroupAction(actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            Node(
                package='robot_state_publisher', executable='robot_state_publisher',
                parameters=[robot_description, {'frame_prefix': 'leader_'}], output='both',
            ),
            Node(
                package='controller_manager', executable='ros2_control_node',
                parameters=[controller_config, {
                    'update_rate': ParameterValue(LaunchConfiguration('update_rate'), value_type=int),
                }],
                output='both',
            ),
            Node(
                package='controller_manager', executable='spawner',
                arguments=['joint_state_broadcaster',
                           'arm_l_state_broadcaster', 'arm_r_state_broadcaster',
                           'hand_l_state_broadcaster', 'hand_r_state_broadcaster',
                           '--controller-manager',
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
