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
from ffw_bringup.launch_utils import start_after_success
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
        DeclareLaunchArgument('model', default_value='ffw_bg2_rev4_follower',
                              description='Robot model name.'),
        DeclareLaunchArgument('world', default_value='default',
                              description='Gz sim World'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
    ]

    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')

    ffw_description_path = os.path.join(
        get_package_share_directory('ffw_description'))

    ffw_bringup_path = os.path.join(
        get_package_share_directory('ffw_bringup'))

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
                              'ffw_bg2_follower.urdf.xacro']),
        ' ',
        'model:=', model,
        ' ',
        'use_sim:=true',
    ])

    robot_description = {'robot_description': robot_description_content}

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',
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
        arguments=[
            '--controller-ros-args',
            '-r /arm_l_controller/joint_trajectory:='
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
            '--controller-ros-args',
            '-r /arm_r_controller/joint_trajectory:='
            '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
            '--controller-ros-args',
            '-r /head_controller/joint_trajectory:='
            '/leader/joystick_controller_left/joint_trajectory',
            '--controller-ros-args',
            '-r /lift_controller/joint_trajectory:='
            '/leader/joystick_controller_right/joint_trajectory',
            'arm_l_controller',
            'arm_r_controller',
            'head_controller',
            'lift_controller',
        ],
        parameters=[robot_description],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    rviz_config_file = os.path.join(ffw_description_path, 'rviz', 'ffw_bg2.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    config_dir = PathJoinSubstitution([FindPackageShare('ffw_bringup'), 'config'])
    teleoperation_configs = [
        PathJoinSubstitution([config_dir, 'ffw_a2_leader', 'ffw_a2_leader_reference.yaml']),
        PathJoinSubstitution([config_dir, model, 'ffw_bg2_teleoperation.yaml']),
        PathJoinSubstitution([config_dir, model, 'ffw_bg2_controller_parameters.yaml']),
        PathJoinSubstitution([config_dir, model, 'ffw_bg2_model_action.yaml']),
    ]
    teleoperation_node = Node(
        package='cyclo_teleoperation',
        executable='cyclo_teleoperation_node',
        name='cyclo_teleoperation',
        parameters=[ParameterFile(path, allow_substs=True) for path in teleoperation_configs] + [
            {'use_sim_time': True}],
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
        gazebo_resource_path,
        gazebo,
        robot_state_pub_node,
        gz_spawn_entity,
        rviz,
    ])
