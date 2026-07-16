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
# Authors: Dongyun Kim
#
# F1 leader+follower bringup. Same hardware as bg2 (no mobile base) except the head
# camera is a RealSense D455 instead of a ZED Mini. Reuses the lg2 leader.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_launch_dir = os.path.join(get_package_share_directory('ffw_bringup'), 'launch')
    camera_assignment_mode = LaunchConfiguration('camera_assignment_mode')

    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir,
                                                   'f1_follower_ai.launch.py')),
        launch_arguments={
            'launch_cameras': 'true',
            'init_position': 'true',
            'head_camera_type': 'realsense',
            'camera_assignment_mode': camera_assignment_mode,
        }.items()
    )
    leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir,
                                                   'ffw_lg2_leader_ai.launch.py'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_assignment_mode', default_value='auto',
            choices=['auto', 'manual'],
            description='Assign RealSense cameras automatically or by configured serials.'),
        follower,
        TimerAction(period=30.0, actions=[leader]),
    ])
