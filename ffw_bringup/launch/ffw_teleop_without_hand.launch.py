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

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart


def generate_launch_description():
    start_follower = ExecuteProcess(
        cmd=['ros2', 'launch', 'ffw_bringup', 'hardware_follower_teleop_without_hand.launch.py'],
        output='screen'
    )

    init_follower = ExecuteProcess(
        cmd=['ros2', 'run', 'ffw_bringup', 'init_position_for_follower_teleop'],
        output='screen'
    )

    start_leader = ExecuteProcess(
        cmd=['ros2', 'launch', 'ffw_bringup', 'hardware_leader_without_hand.launch.py'],
        output='screen'
    )

    start_keyboard_gui = ExecuteProcess(
        cmd=['ros2', 'run', 'ffw_teleop', 'keyboard_control_standalone.py'],
        shell=True,
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg='Starting Follower Launch'),
        start_follower,

        RegisterEventHandler(
            OnProcessStart(
                target_action=start_follower,
                on_start=[
                    LogInfo(msg='Follower started. Initializing position...'),
                    init_follower
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=init_follower,
                on_exit=[
                    LogInfo(msg='Init complete. Starting Leader Launch...'),
                    start_leader
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessStart(
                target_action=start_leader,
                on_start=[
                    LogInfo(msg='Launching Keyboard GUI...'),
                    start_keyboard_gui
                ]
            )
        ),
    ])
