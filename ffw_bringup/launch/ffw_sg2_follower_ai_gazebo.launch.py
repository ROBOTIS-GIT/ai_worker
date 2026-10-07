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
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ffw_bringup.launch_utils import controller_remaps, start_after_success
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('model', default_value='ffw_sg2_rev1_follower',
                              description='Robot model name.'),
        DeclareLaunchArgument('enable_control', default_value='false', choices=['true', 'false'],
                              description='Run the shared teleoperation/model-action node.'),
        DeclareLaunchArgument('initial_source', default_value='model_action',
                              choices=['teleop', 'model_action']),
        DeclareLaunchArgument('world', default_value='default',
                              description='Gz sim World'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                              description='Start Gazebo GUI; false runs the server headlessly.'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'],
                              description='Start RViz.'),
    ]

    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')

    ffw_description_path = os.path.join(
        get_package_share_directory('ffw_description'))

    ffw_bringup_path = os.path.join(
        get_package_share_directory('ffw_bringup'))

    # Set gazebo sim resource path
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
                        ' -r',
                        PythonExpression([
                            "'' if '", LaunchConfiguration('gui'),
                            "' == 'true' else ' -s --headless-rendering'",
                        ]),
                    ])
                ]
             )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('ffw_description'),
                              'urdf',
                              model,
                              'ffw_sg2_follower.urdf.xacro']),
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
                   '-name', model,
                   '-allow_renaming', 'true',
                   '-use_sim', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=controller_remaps() + [
            'arm_l_controller',
            'arm_r_controller',
            'head_controller',
            'lift_controller',
            'swerve_drive_controller',
        ],
        parameters=[robot_description],
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

    rviz_config_file = os.path.join(ffw_description_path, 'rviz', 'ffw_sg2.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Same profiles as the hardware bringup; only the clock uses Gazebo time.
    config_dir = PathJoinSubstitution([FindPackageShare('ffw_bringup'), 'config'])
    teleoperation_configs = [
        PathJoinSubstitution([config_dir, 'ffw_a2_leader', 'ffw_a2_leader_reference.yaml']),
        PathJoinSubstitution([config_dir, model, 'ffw_sg2_teleoperation.yaml']),
        PathJoinSubstitution([config_dir, model, 'ffw_sg2_controller_parameters.yaml']),
        PathJoinSubstitution([config_dir, model, 'ffw_sg2_model_action.yaml']),
    ]
    teleoperation_node = Node(
        package='cyclo_teleoperation',
        executable='cyclo_teleoperation_node',
        name='cyclo_teleoperation',
        parameters=[ParameterFile(path, allow_substs=True) for path in teleoperation_configs] + [{
            'initial_source': LaunchConfiguration('initial_source'),
            'use_sim_time': True,
        }],
        condition=IfCondition(LaunchConfiguration('enable_control')),
        output='screen',
    )

    def after_spawn(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg='Gazebo spawn failed; controllers will not be started.')]
        return [joint_state_broadcaster_spawner]

    def after_joint_state_broadcaster(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg='Joint state broadcaster failed; control will not be started.')]
        return [robot_controller_spawner]

    return LaunchDescription([
        *declared_arguments,
        RegisterEventHandler(OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=start_after_success([robot_controller_spawner], teleoperation_node))),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=after_spawn,
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=joint_state_broadcaster_spawner,
               on_exit=after_joint_state_broadcaster,
            )
        ),
        bridge,
        dual_laser_merger_node,
        gazebo_resource_path,
        gazebo,
        robot_state_pub_node,
        gz_spawn_entity,
        rviz,
    ])
