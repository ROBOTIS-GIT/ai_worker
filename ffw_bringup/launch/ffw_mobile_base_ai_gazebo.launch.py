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

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ffw_description_path = get_package_share_directory('ffw_description')
    ffw_bringup_path = get_package_share_directory('ffw_bringup')

    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    model_name = model.perform(context)

    if model_name == 'ffw_sg2_rev1_follower':
        robot_xacro = 'ffw_sg2_follower.urdf.xacro'
    elif model_name == 'ffw_sh5_rev1_follower':
        robot_xacro = 'ffw_sh5_follower.urdf.xacro'
    else:
        raise RuntimeError(
            "Unsupported model. Use 'ffw_sg2_rev1_follower' or "
            "'ffw_sh5_rev1_follower'."
        )

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(ffw_bringup_path, 'worlds'), ':' +
            str(Path(ffw_description_path).parent.resolve())
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[
            ('gz_args', [
                world,
                '.sdf',
                ' -v 1',
                ' -r'
            ])
        ]
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('ffw_description'),
                              'urdf',
                              model,
                              robot_xacro]),
        ' ',
        'model:=', model,
        ' ',
        'use_sim:=true',
    ])

    robot_description = {'robot_description': robot_description_content}

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {
            'use_sim_time': True
        }],
        output='screen'
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.2',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', model_name,
                   '-allow_renaming', 'true',
                   '-use_sim', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    swerve_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_drive_controller'],
        output='screen'
    )

    gz_bridge_params_path = os.path.join(
        ffw_bringup_path,
        'config',
        'common',
        'gz_bridge.yaml'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={gz_bridge_params_path}'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    dual_laser_merger_node = Node(
        package='dual_laser_merger',
        executable='dual_laser_merger_node',
        output='screen',
        parameters=[{
            'laser_1_topic': '/scan_left',
            'laser_2_topic': '/scan_right',
            'merged_scan_topic': '/scan',
            'merged_cloud_topic': '/scan_cloud',
            'target_frame': 'base_link',
            'angle_min': -3.141592654,
            'angle_max': 3.141592654,
            'angle_increment': 0.006544985,
            'scan_time': 0.1,
            'range_min': 0.05,
            'range_max': 20.0,
            'use_inf': True,
            'tolerance': 0.05,
            'queue_size': 10,
            'enable_shadow_filter': True,
            'enable_average_filter': True,
        }, {
            'use_sim_time': True,
        }],
    )

    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[swerve_drive_controller_spawner],
            )
        ),
        bridge,
        dual_laser_merger_node,
        gazebo_resource_path,
        gazebo,
        robot_state_pub_node,
        gz_spawn_entity,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('model', default_value='ffw_sg2_rev1_follower',
                              description='Robot model name.'),
        DeclareLaunchArgument('world', default_value='default',
                              description='Gz sim World'),
    ]

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup),
    ])
