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
from launch.actions import LogInfo, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ffw_bringup_path = os.path.join(
        get_package_share_directory('ffw_bringup')
    )

    left_done_flag = {'value': False}
    right_done_flag = {'value': False}

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ffw_bringup_path, 'launch', 'hardware_leader_with_hand.launch.py')
        )
    )

    calibrator_left = Node(
        package='ffw_bringup',
        executable='hand_calibrator_left',
        name='hand_calibrator_left',
        output='screen',
    )
    calibrator_right = Node(
        package='ffw_bringup',
        executable='hand_calibrator_right',
        name='hand_calibrator_right',
        output='screen',
    )

    run_calibrators = TimerAction(
        period=5.0,
        actions=[calibrator_left, calibrator_right]
    )

    def check_and_shutdown(context, flag_dict, other_flag, shutdown_msg):
        flag_dict['value'] = True
        if other_flag['value']:
            return [LogInfo(msg=shutdown_msg), Shutdown()]
        return []

    left_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=calibrator_left,
            on_exit=lambda context: check_and_shutdown(
                context, left_done_flag, right_done_flag,
                'Both hands calibrated. Shutting down.'
            )
        )
    )
    right_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=calibrator_right,
            on_exit=lambda context: check_and_shutdown(
                context, right_done_flag, left_done_flag,
                'Both hands calibrated. Shutting down.'
            )
        )
    )

    return LaunchDescription([
        hardware_launch,
        run_calibrators,
        left_exit_handler,
        right_exit_handler
    ])
