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


def generate_launch_description():
    bringup_launch_dir = os.path.join(get_package_share_directory('ffw_bringup'), 'launch')

    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir,
                                                   'ffw_sg2_follower_ai.launch.py')),
        launch_arguments={
            'launch_cameras': 'true',
            'init_position': 'true',
            'head_camera_type': 'zed',
        }.items()
    )
    leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir,
                                                   'ffw_lg2_leader_ai.launch.py'))
    )

    return LaunchDescription([
        follower,
        TimerAction(period=30.0, actions=[leader]),
    ])
