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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


CONTROLLER_REMAPS = {
    'hand_l_controller':
        '/leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory',
    'hand_r_controller':
        '/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory',
    'head_controller': '/leader/joystick_controller_left/joint_trajectory',
    'lift_controller': '/leader/joystick_controller_right/joint_trajectory',
}


def load_yaml(path):
    with open(path, encoding='utf-8') as file:
        return yaml.safe_load(file)


def controller_spawn_config(path):
    config = load_yaml(path)
    try:
        return config['/**']['controller_spawn']['ros__parameters']
    except (KeyError, TypeError) as error:
        raise RuntimeError(f'Missing controller_spawn in {path}') from error


def launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context)
    description_share = get_package_share_directory('ffw_description')
    bringup_share = get_package_share_directory('ffw_bringup')
    robots = load_yaml(
        os.path.join(description_share, 'config', 'follower_robots.yaml'))

    if robot not in robots:
        supported = ', '.join(robots)
        raise RuntimeError(f"Unsupported robot '{robot}'. Choose one of: {supported}")

    robot_config = robots[robot]
    body = robot_config['body']
    base = robot_config['base']
    end_tool = robot_config['end_tool']

    controller_root = Path(bringup_share, 'config', 'follower', 'controllers')
    controller_files = [
        controller_root / 'body' / f'{body}.controller.yaml',
        controller_root / 'base' / f'{base}.controller.yaml',
        controller_root / 'end_tool' / f'{end_tool}.controller.yaml',
    ]

    controllers = []
    for controller_file in controller_files:
        if not controller_file.is_file():
            raise RuntimeError(f'Controller config not found: {controller_file}')
        spawn_config = controller_spawn_config(controller_file)
        controllers.extend(spawn_config.get('controllers', []))
        if 'active_controller' in spawn_config:
            controllers.append(spawn_config['active_controller'])

    controllers = [
        controller for controller in controllers
        if controller not in (
            'joint_state_broadcaster',
            'ffw_robot_manager',
            # Gazebo does not provide HX5 pressure state interfaces.
            'pressure_l_broadcaster',
            'pressure_r_broadcaster',
        )
    ]
    controller_arguments = []
    for controller in controllers:
        if controller in CONTROLLER_REMAPS:
            controller_arguments.extend([
                '--controller-ros-args',
                f'-r /{controller}/joint_trajectory:={CONTROLLER_REMAPS[controller]}',
            ])
    controller_arguments.extend(controllers)

    world = LaunchConfiguration('world')
    start_rviz = LaunchConfiguration('start_rviz')
    is_swerve = base.startswith('ffw_swerve')

    resource_paths = [
        os.path.join(bringup_share, 'worlds'),
        str(Path(description_share).parent),
        str(Path(get_package_share_directory('robotis_hand_description')).parent),
        str(Path(get_package_share_directory('ros_gz_sim')).parent),
    ]
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH')
    if existing_resource_path:
        resource_paths.append(existing_resource_path)

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('ffw_description'),
            'urdf', 'follower', 'ffw_follower.urdf.xacro',
        ]),
        ' robot:=', robot,
        ' use_sim:=true',
    ])
    robot_description = {'robot_description': robot_description_content}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [world, '.sdf -v 1 -r'],
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen',
    )
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.2' if is_swerve else '0.0',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
            '-name', robot,
            '-allow_renaming', 'true',
            '-use_sim', 'true',
        ],
        output='screen',
    )
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=controller_arguments,
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            'config_file:=' + os.path.join(
                bringup_share, 'config', 'common', 'gz_bridge.yaml'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    actions = [
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.pathsep.join(resource_paths),
        ),
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_spawner],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[controller_spawner],
        )),
        gazebo,
        bridge,
        robot_state_publisher,
        spawn_entity,
        Node(
            package='ffw_joint_trajectory_command_broadcaster',
            executable='joint_trajectory_splitter',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d', os.path.join(description_share, 'rviz', f'ffw_{robot}.rviz'),
            ],
            parameters=[{'use_sim_time': True}],
            output='log',
            condition=IfCondition(start_rviz),
        ),
    ]

    hand_sides = [side for side in ('l', 'r') if f'hand_{side}_controller' in controllers]
    if hand_sides:
        gripper_to_hand = Node(
            package='ffw_joint_trajectory_command_broadcaster',
            executable='gripper_to_hand',
            name='gripper_to_hand',
            parameters=[{
                'left_enabled': 'l' in hand_sides,
                'right_enabled': 'r' in hand_sides,
                'use_sim_time': True,
            }],
            output='screen',
        )
        actions.insert(0, RegisterEventHandler(OnProcessExit(
            target_action=controller_spawner,
            on_exit=lambda event, context: [gripper_to_hand] if event.returncode == 0 else [],
        )))

    if is_swerve:
        actions.append(Node(
            package='dual_laser_merger',
            executable='dual_laser_merger_node',
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
                'use_sim_time': True,
            }],
            output='screen',
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            description='Follower robot name.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='default',
            description='Gazebo world file name without the .sdf extension',
        ),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
