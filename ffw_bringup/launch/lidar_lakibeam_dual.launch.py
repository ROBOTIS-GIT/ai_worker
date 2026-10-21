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

"""Connect two Lakibeam lidars to the AI Worker scan topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Launch the left and right Lakibeam drivers without a scan merger."""
    actions = [
        DeclareLaunchArgument('sensorip0', default_value='192.168.6.3',
                              description='Left lidar IPv4 address'),
        DeclareLaunchArgument('sensorip1', default_value='192.168.6.4',
                              description='Right lidar IPv4 address'),
        DeclareLaunchArgument('output_topic0', default_value='scan_left'),
        DeclareLaunchArgument('output_topic1', default_value='scan_right'),
        DeclareLaunchArgument('frame_id0', default_value='lidar_l_link'),
        DeclareLaunchArgument('frame_id1', default_value='lidar_r_link'),
        DeclareLaunchArgument('inverted', default_value='false'),
        DeclareLaunchArgument('hostip', default_value='0.0.0.0',
                              description='Host IP address to listen on'),
        DeclareLaunchArgument('port0', default_value='"2368"',
                              description='Port for left lidar'),
        DeclareLaunchArgument('port1', default_value='"2369"',
                              description='Port for right lidar'),
        DeclareLaunchArgument('angle_offset', default_value='0'),
        DeclareLaunchArgument('scanfreq', default_value='"30"',
                              description='Scan frequency (10, 20, 25, 30 Hz)'),
        DeclareLaunchArgument('laser_enable', default_value='"true"'),
        DeclareLaunchArgument('scan_range_start', default_value='"45"'),
        DeclareLaunchArgument('scan_range_stop', default_value='"315"'),
    ]
    for index, side in [(0, 'left'), (1, 'right')]:
        actions.append(Node(
            package='lakibeam1',
            executable='lakibeam1_scan_node',
            name=f'richbeam_lidar_node_{side}',
            output='log',
            arguments=['--ros-args', '--log-level', 'ERROR'],
            parameters=[{
                'frame_id': LaunchConfiguration(f'frame_id{index}'),
                'output_topic': LaunchConfiguration(f'output_topic{index}'),
                'sensorip': LaunchConfiguration(f'sensorip{index}'),
                'port': LaunchConfiguration(f'port{index}'),
                'inverted': LaunchConfiguration('inverted'),
                'hostip': LaunchConfiguration('hostip'),
                'angle_offset': LaunchConfiguration('angle_offset'),
                'scanfreq': LaunchConfiguration('scanfreq'),
                'laser_enable': LaunchConfiguration('laser_enable'),
                'scan_range_start': LaunchConfiguration('scan_range_start'),
                'scan_range_stop': LaunchConfiguration('scan_range_stop'),
            }],
        ))
    return LaunchDescription(actions)
