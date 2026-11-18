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
# Author: Woojin Wie, Hyunwoo Nam


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz', default_value='true', description='Open RViz preview.',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='prefix of the joint and link names',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Run joint state publisher gui node',
        ),
        DeclareLaunchArgument(
            'model',
            default_value='ffw_a13_leader',
            description='Robot model name.'
        ),
    ]

    prefix = LaunchConfiguration('prefix')
    use_gui = LaunchConfiguration('use_gui')
    model = LaunchConfiguration('model')

    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('ffw_description'),
            'urdf',
            model,
            'ffw_a13_leader.urdf.xacro',
        ]),
        ' ',
        'prefix:=',
        prefix,
        ' ',
        'use_mock_hardware:=',
        'True',
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ffw_description'),
        'rviz',
        'ffw_a13_leader.rviz',
    ])

    return LaunchDescription(
        declared_arguments + [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': ParameterValue(urdf_file, value_type=str),
                             'frame_prefix': 'leader_'}],
                output='screen',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config_file],
                condition=IfCondition(LaunchConfiguration('start_rviz')),
                output='screen',
            ),
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                condition=IfCondition(use_gui),
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                condition=UnlessCondition(use_gui),
            ),
        ]
    )
