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

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


UI_DIR = os.path.expanduser('~/ros2_ws/src/ai_worker/ffw_calibration/ui')


def generate_launch_description():
    calibration_service = Node(
        package='ffw_calibration',
        executable='calibration_service',
        name='ffw_calibration_service',
        output='screen',
        parameters=[{
            'calibration_pose_path': os.path.expanduser(
                '~/ros2_ws/src/ai_worker/ffw_calibration/config/ffw_bg2_rev4_follower/calibration_pose.yaml'
            ),
            'homing_offsets_path': os.path.expanduser(
                '~/ros2_ws/src/ai_worker/ffw_calibration/config/ffw_bg2_rev4_follower/homing_offsets.yaml'
            ),
            'joint_states_topic': '/joint_states',
            'stale_timeout_sec': 1.0,
            'default_zero_pose_duration_sec': 10.0,
            'default_effort_duration_sec': 1.0,
            'effort_hz': 50.0,
        }],
    )

    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'fragment_timeout': 600,
            'max_message_size': 100000000,
            'unregister_timeout': 10.0,
            'call_service_timeout': 60.0,
        }],
    )

    ui_dev_server = ExecuteProcess(
        cmd=['npm', 'run', 'dev', '--', '--host', '0.0.0.0'],
        cwd=UI_DIR,
        output='screen',
        sigterm_timeout='5',
    )

    return LaunchDescription([
        calibration_service,
        rosbridge_websocket,
        ui_dev_server,
    ])
