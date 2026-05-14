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
#
# Author: Hyungyu Kim

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


VALID_ROBOT_TYPES = ['sg2', 'bg2', 'sh5', 'bh5']


def generate_launch_description():
    pkg_share = get_package_share_directory('ffw_bringup')

    robot_type = LaunchConfiguration('type')

    pack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_share, '/launch/ffw_', robot_type, '_follower_ai.launch.py'
        ]),
        launch_arguments={
            'init_position': 'true',
            'init_position_file': 'pack_position.yaml',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'type',
            description=f'Robot type to launch pack position. One of: {VALID_ROBOT_TYPES}',
        ),
        LogInfo(msg=['Starting FFW ', robot_type, ' packing...']),
        pack_launch,
    ])
