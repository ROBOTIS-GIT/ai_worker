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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    presets_file = LaunchConfiguration('presets_file').perform(context) or os.path.join(
        get_package_share_directory('ffw_preset_controller'), 'config', 'hand_presets.yaml')
    initial_preset = LaunchConfiguration('initial_preset').perform(context)

    parameters = [{'presets_file': presets_file, 'initial_preset': initial_preset}]
    if params_file:
        parameters.insert(0, params_file)

    return [
        Node(
            package='ffw_preset_controller',
            executable='hand_preset_controller',
            name='hand_preset_controller',
            parameters=parameters,
            output='screen',
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Leader-specific parameters (gripper calibration, topics).'),
        DeclareLaunchArgument(
            'presets_file', default_value='',
            description='Hand preset YAML. Empty uses the file installed with this package.'),
        DeclareLaunchArgument(
            'initial_preset', default_value='',
            description='Preset to start with. Empty uses `default_preset` from the YAML.'),
        OpaqueFunction(function=launch_setup),
    ])
