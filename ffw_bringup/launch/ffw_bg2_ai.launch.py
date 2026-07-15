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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_launch_dir = os.path.join(get_package_share_directory('ffw_bringup'), 'launch')
    camera_assignment_mode = LaunchConfiguration('camera_assignment_mode')
    camera_left_serial = LaunchConfiguration('camera_left_serial')
    camera_right_serial = LaunchConfiguration('camera_right_serial')

    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir,
                                                   'ffw_bg2_follower_ai.launch.py')),
        launch_arguments={
            'launch_cameras': 'true',
            'init_position': 'true',
            'head_camera_type': 'zed',
            'camera_assignment_mode': camera_assignment_mode,
            'camera_left_serial': camera_left_serial,
            'camera_right_serial': camera_right_serial,
        }.items()
    )
    leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir,
                                                   'ffw_lg2_leader_ai.launch.py'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_assignment_mode', default_value='usb_port',
            choices=['usb_port', 'serial'],
            description='Assign wrist cameras by fixed USB ports or explicit serials.'),
        DeclareLaunchArgument('camera_left_serial', default_value='',
                              description='Left wrist RealSense serial in serial mode.'),
        DeclareLaunchArgument('camera_right_serial', default_value='',
                              description='Right wrist RealSense serial in serial mode.'),
        follower,
        TimerAction(period=30.0, actions=[leader]),
    ])
