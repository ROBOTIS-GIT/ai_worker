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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bringup_launch_dir = os.path.join(get_package_share_directory('ffw_bringup'), 'launch')

    head_camera_type = LaunchConfiguration('head_camera_type')
    camera_assignment_mode = LaunchConfiguration('camera_assignment_mode')
    camera_left_serial = LaunchConfiguration('camera_left_serial')
    camera_right_serial = LaunchConfiguration('camera_right_serial')
    camera_head_serial = LaunchConfiguration('camera_head_serial')

    is_zed_head = PythonExpression(["'", head_camera_type, "' == 'zed'"])
    enable_realsense_head = PythonExpression(
        ["'true' if '", head_camera_type, "' == 'realsense' else 'false'"]
    )

    camera_zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, 'camera_zed.launch.py')),
        launch_arguments={'camera_model': 'zedm'}.items(),
        condition=IfCondition(is_zed_head),
    )

    camera_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'camera_realsense.launch.py')),
        launch_arguments={
            'enable_head_camera': enable_realsense_head,
            'camera_assignment_mode': camera_assignment_mode,
            'camera_left_serial': camera_left_serial,
            'camera_right_serial': camera_right_serial,
            'camera_head_serial': camera_head_serial,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'head_camera_type',
            default_value='zed',
            choices=['zed', 'realsense'],
            description='Head camera type. zed for sg2/bg2/sh5/bh5 (zed mini head + 2 d405 '
                        'wrists). realsense for f2 (d455 head + 2 d405 wrists).'
        ),
        DeclareLaunchArgument(
            'camera_assignment_mode',
            default_value='usb_port',
            choices=['usb_port', 'serial'],
            description='Assign RealSense camera roles by fixed USB ports or explicit serials.'
        ),
        DeclareLaunchArgument('camera_left_serial', default_value='',
                              description='Left wrist RealSense serial in serial mode.'),
        DeclareLaunchArgument('camera_right_serial', default_value='',
                              description='Right wrist RealSense serial in serial mode.'),
        DeclareLaunchArgument('camera_head_serial', default_value='',
                              description='Head RealSense serial in serial mode.'),
        camera_zed,
        TimerAction(period=10.0, actions=[camera_realsense]),
    ])
