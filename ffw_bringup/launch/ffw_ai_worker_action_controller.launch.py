#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('ffw_bringup'),
        'config',
        'common',
        'ffw_action_controller.yaml',
    ])
    default_follower_urdf = PathJoinSubstitution([
        FindPackageShare('cyclo_motion_controller_models'),
        'models',
        'ai_worker',
        'ffw_sg2_follower.urdf',
    ])
    default_follower_srdf = PathJoinSubstitution([
        FindPackageShare('cyclo_motion_controller_models'),
        'models',
        'ai_worker',
        'ffw_sg2_follower_default.srdf',
    ])

    config = LaunchConfiguration('config')
    follower_urdf_path = LaunchConfiguration('follower_urdf_path')
    follower_srdf_path = LaunchConfiguration('follower_srdf_path')

    declared_arguments = [
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('follower_urdf_path', default_value=default_follower_urdf),
        DeclareLaunchArgument('follower_srdf_path', default_value=default_follower_srdf),
    ]
    action_controller = Node(
        package='cyclo_teleoperation',
        executable='cyclo_action_controller_node',
        name='cyclo_action_controller',
        parameters=[
            config,
            {
                'follower_urdf_path': follower_urdf_path,
                'follower_srdf_path': follower_srdf_path,
            },
        ],
        output='screen',
    )
    return LaunchDescription(declared_arguments + [action_controller])
