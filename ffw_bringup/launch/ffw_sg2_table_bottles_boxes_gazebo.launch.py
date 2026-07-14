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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('model', default_value='ffw_sg2_rev1_follower',
                              description='Robot model name.'),
        DeclareLaunchArgument('robot_x', default_value='-0.05',
                              description='Initial robot x position.'),
        DeclareLaunchArgument('robot_y', default_value='0.0',
                              description='Initial robot y position.'),
        DeclareLaunchArgument('robot_z', default_value='0.05',
                              description='Initial robot z position.'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0',
                              description='Initial robot yaw angle.'),
    ]

    ffw_bringup_path = get_package_share_directory('ffw_bringup')
    table_cube_launch = os.path.join(
        ffw_bringup_path,
        'launch',
        'ffw_sg2_table_cube_gazebo.launch.py'
    )

    table_bottles_boxes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(table_cube_launch),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'world': 'table_bottles_boxes',
            'robot_x': LaunchConfiguration('robot_x'),
            'robot_y': LaunchConfiguration('robot_y'),
            'robot_z': LaunchConfiguration('robot_z'),
            'robot_yaw': LaunchConfiguration('robot_yaw'),
        }.items()
    )

    return LaunchDescription([
        *declared_arguments,
        table_bottles_boxes,
    ])
