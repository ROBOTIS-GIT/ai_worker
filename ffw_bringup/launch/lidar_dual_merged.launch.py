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

"""Run two selected lidars and publish a merged scan without robot controllers."""

import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def mount_transforms(context):
    """Publish the configured sensor poses relative to the scan target frame."""
    actions = []
    for index, side in [(0, 'left'), (1, 'right')]:
        pose = LaunchConfiguration(f'{side}_pose').perform(context).split()
        if len(pose) != 6 or not all(math.isfinite(float(value)) for value in pose):
            raise ValueError(f'{side}_pose must contain x y z roll pitch yaw (m, rad)')
        arguments = []
        for name, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), pose):
            arguments.extend([f'--{name}', value])
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'lidar_{side}_mount_tf',
            arguments=arguments + [
                '--frame-id', LaunchConfiguration('target_frame'),
                '--child-frame-id', LaunchConfiguration(f'frame_id{index}'),
            ],
        ))
    return actions


def set_default_scan_time(context):
    """Match the scan period to the selected sensor unless explicitly overridden."""
    if LaunchConfiguration('merged_scan_time').perform(context):
        return []
    if LaunchConfiguration('lidar_driver').perform(context) == 'sick_picoscan':
        frequency = 25.0
    else:
        frequency = float(
            LaunchConfiguration('scanfreq', default='30').perform(context).strip('"'))
    if not math.isfinite(frequency) or frequency <= 0:
        raise ValueError('scanfreq must be a positive finite frequency in Hz')
    return [SetLaunchConfiguration('merged_scan_time', str(1.0 / frequency))]


def generate_launch_description():
    """Launch the sensors, their mounting TFs, and the existing scan merger."""
    sensor_launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ffw_bringup'), 'launch', filename,
            ])),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('lidar_driver'), driver)),
        )
        for driver, filename in [
            ('lakibeam', 'lidar_lakibeam_dual.launch.py'),
            ('sick_picoscan', 'lidar_sick_dual.launch.py'),
        ]
    ]
    return LaunchDescription([
        DeclareLaunchArgument('lidar_driver', default_value='lakibeam',
                              choices=['lakibeam', 'sick_picoscan']),
        DeclareLaunchArgument('launch_sensors', default_value='true',
                              choices=['true', 'false'],
                              description='Start drivers; false reuses already running drivers'),
        DeclareLaunchArgument('publish_mount_tf', default_value='true',
                              choices=['true', 'false'],
                              description='Disable if the robot already publishes mounting TFs'),
        DeclareLaunchArgument('output_topic0', default_value='/scan_left'),
        DeclareLaunchArgument('output_topic1', default_value='/scan_right'),
        DeclareLaunchArgument('frame_id0', default_value='lidar_l_link'),
        DeclareLaunchArgument('frame_id1', default_value='lidar_r_link'),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('merged_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('merged_cloud_topic', default_value='/scan_cloud'),
        DeclareLaunchArgument('merged_scan_time', default_value='',
                              description='LaserScan.scan_time (s); empty uses sensor rate'),
        DeclareLaunchArgument('enable_shadow_filter', default_value='false',
                              choices=['true', 'false'],
                              description='Remove isolated points from the merged cloud'),
        DeclareLaunchArgument('enable_average_filter', default_value='false',
                              choices=['true', 'false'],
                              description='Average adjacent input ranges before merging'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Defaults match ffw_description/urdf/ffw_mobile_base/ffw_mobile_base.urdf.xacro.
        DeclareLaunchArgument('left_pose',
                              default_value='-0.111881 0.240 0.34432 0 0 1.57079632679',
                              description='Left sensor x y z roll pitch yaw in target_frame'),
        DeclareLaunchArgument('right_pose',
                              default_value='-0.111881 -0.240 0.34432 0 0 -1.57079632679',
                              description='Right sensor x y z roll pitch yaw in target_frame'),
        OpaqueFunction(function=set_default_scan_time),
        OpaqueFunction(function=mount_transforms,
                       condition=IfCondition(LaunchConfiguration('publish_mount_tf'))),
        GroupAction(
            actions=sensor_launches,
            condition=IfCondition(LaunchConfiguration('launch_sensors')),
        ),
        Node(
            package='dual_laser_merger',
            executable='dual_laser_merger_node',
            name='dual_laser_merger',
            output='screen',
            parameters=[{
                'laser_1_topic': LaunchConfiguration('output_topic0'),
                'laser_2_topic': LaunchConfiguration('output_topic1'),
                'merged_scan_topic': LaunchConfiguration('merged_scan_topic'),
                'merged_cloud_topic': LaunchConfiguration('merged_cloud_topic'),
                'target_frame': LaunchConfiguration('target_frame'),
                'angle_min': -math.pi,
                'angle_max': math.pi,
                'angle_increment': 0.006544985,
                'scan_time': ParameterValue(LaunchConfiguration('merged_scan_time'),
                                            value_type=float),
                'range_min': 0.05,
                'range_max': 20.0,
                'use_inf': True,
                'tolerance': 0.05,
                'queue_size': 10,
                'enable_shadow_filter': ParameterValue(
                    LaunchConfiguration('enable_shadow_filter'), value_type=bool),
                'enable_average_filter': ParameterValue(
                    LaunchConfiguration('enable_average_filter'), value_type=bool),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                               value_type=bool),
            }],
        ),
    ])
