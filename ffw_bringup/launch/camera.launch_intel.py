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
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_launch_dir = os.path.join(get_package_share_directory('ffw_bringup'), 'launch')

    # RealSense cameras launch (D455 head, D405 left/right wrist)
    camera_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'camera_realsense.launch.py')),
    )

    # RealSense compressed image relay nodes
    relay_head = Node(
        package='topic_tools',
        executable='relay',
        name='relay_cam_head',
        arguments=[
            '/camera_head/camera_head/color/image_raw/compressed',
            '/robot/camera/cam_head/image_raw/compressed'
        ],
        output='screen'
    )

    relay_left_wrist = Node(
        package='topic_tools',
        executable='relay',
        name='relay_cam_left_wrist',
        arguments=[
            '/camera_left/camera_left/color/image_rect_raw/compressed',
            '/robot/camera/cam_left_wrist/image_raw/compressed'
        ],
        output='screen'
    )

    relay_right_wrist = Node(
        package='topic_tools',
        executable='relay',
        name='relay_cam_right_wrist',
        arguments=[
            '/camera_right/camera_right/color/image_rect_raw/compressed',
            '/robot/camera/cam_right_wrist/image_raw/compressed'
        ],
        output='screen'
    )

    # RealSense camera_info relay nodes
    relay_head_info = Node(
        package='topic_tools',
        executable='relay',
        name='relay_cam_head_info',
        arguments=[
            '/camera_head/camera_head/color/camera_info',
            '/robot/camera/cam_head/image_raw/compressed/camera_info'
        ],
        output='screen'
    )

    relay_left_wrist_info = Node(
        package='topic_tools',
        executable='relay',
        name='relay_cam_left_wrist_info',
        arguments=[
            '/camera_left/camera_left/color/camera_info',
            '/robot/camera/cam_left_wrist/image_raw/compressed/camera_info'
        ],
        output='screen'
    )

    relay_right_wrist_info = Node(
        package='topic_tools',
        executable='relay',
        name='relay_cam_right_wrist_info',
        arguments=[
            '/camera_right/camera_right/color/camera_info',
            '/robot/camera/cam_right_wrist/image_raw/compressed/camera_info'
        ],
        output='screen'
    )

    realsense_relay_nodes = TimerAction(
        period=15.0,
        actions=[relay_head, relay_left_wrist, relay_right_wrist,
                 relay_head_info, relay_left_wrist_info, relay_right_wrist_info]
    )

    return LaunchDescription([
        TimerAction(period=10.0, actions=[camera_realsense]),
        realsense_relay_nodes,
    ])
