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
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
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
    executors = []
    controller_switch = {}
    for controller_file in controller_files:
        if not controller_file.is_file():
            raise RuntimeError(f'Controller config not found: {controller_file}')
        spawn_config = controller_spawn_config(controller_file)
        controllers.extend(spawn_config.get('controllers', []))
        executors.extend(spawn_config.get('executors', []))
        for key in ('initial_controller', 'initial_executor', 'active_controller'):
            if key in spawn_config:
                controller_switch[key] = spawn_config[key]

    use_sim = LaunchConfiguration('use_sim')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    port_name = LaunchConfiguration('port_name')
    start_rviz = LaunchConfiguration('start_rviz')
    launch_cameras = LaunchConfiguration('launch_cameras')
    launch_lidar = LaunchConfiguration('launch_lidar')
    init_position = LaunchConfiguration('init_position')
    use_head_eef_tracker = LaunchConfiguration('use_head_eef_tracker')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('ffw_description'),
        'urdf', 'follower', 'ffw_follower.urdf.xacro',
    ])
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', xacro_file,
        ' robot:=', robot,
        ' use_sim:=', use_sim,
        ' use_mock_hardware:=', use_mock_hardware,
        ' mock_sensor_commands:=', mock_sensor_commands,
        ' port_name:=', port_name,
    ])
    robot_description = {'robot_description': robot_description_content}

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            *[str(path) for path in controller_files],
            {'use_sim_time': use_sim},
        ],
        output='both',
        condition=UnlessCondition(use_sim),
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim}],
        output='screen',
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    main_controllers = [
        controller for controller in controllers
        if controller not in ('joint_state_broadcaster', 'ffw_robot_manager')
    ]
    controller_arguments = []
    for controller in main_controllers:
        if controller in CONTROLLER_REMAPS:
            controller_arguments.extend([
                '--controller-ros-args',
                f'-r /{controller}/joint_trajectory:={CONTROLLER_REMAPS[controller]}',
            ])
    controller_arguments.extend(main_controllers)
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=controller_arguments,
        output='screen',
    )

    actions = [
        control_node,
        robot_state_publisher,
        joint_state_spawner,
        controller_spawner,
        # Split /leader/joint_trajectory_command_broadcaster_{left,right}/joint_trajectory
        # into modular arm and gripper controller topics.
        Node(
            package='ffw_bringup',
            executable='joint_trajectory_splitter',
            output='screen',
        ),
    ]

    actions.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ffw_robot_manager'],
        output='screen',
        condition=UnlessCondition(use_sim),
    ))

    initial_positions = os.path.join(
        bringup_share, 'config', 'follower', 'initial_positions.yaml')
    executor_nodes = [
        Node(
            package='ffw_bringup',
            executable='joint_trajectory_executor',
            name=executor,
            parameters=[initial_positions],
            output='screen',
        )
        for executor in executors
    ]

    initial_executor = controller_switch.get('initial_executor')
    if initial_executor:
        swerve_executor = Node(
            package='ffw_bringup',
            executable='joint_trajectory_executor',
            name=initial_executor,
            parameters=[initial_positions],
            output='screen',
        )
        executor_nodes.append(swerve_executor)

        initial_controller = controller_switch['initial_controller']
        active_controller = controller_switch['active_controller']
        actions.extend([
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[initial_controller],
                output='screen',
                condition=IfCondition(init_position),
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[active_controller],
                output='screen',
                condition=UnlessCondition(init_position),
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=swerve_executor,
                    on_exit=[
                        Node(
                            package='controller_manager',
                            executable='unspawner',
                            arguments=[initial_controller],
                            output='screen',
                        ),
                        TimerAction(
                            period=5.0,
                            actions=[Node(
                                package='controller_manager',
                                executable='spawner',
                                arguments=[active_controller],
                                output='screen',
                            )],
                        ),
                    ],
                ),
                condition=IfCondition(init_position),
            ),
        ])

    actions.append(RegisterEventHandler(
        OnProcessExit(
            target_action=controller_spawner,
            on_exit=executor_nodes,
        ),
        condition=IfCondition(init_position),
    ))

    if 'effort_l_controller' in controllers:
        current_command = (
            'data: [' + ', '.join(['300.0'] * 20) + ']'
        )
        for side in ('l', 'r'):
            process = ExecuteProcess(
                name=f'{side}_hand_current_command',
                cmd=[
                    'ros2', 'topic', 'pub', '-r', '50', '-t', '50', '-p', '50',
                    f'/effort_{side}_controller/commands',
                    'std_msgs/msg/Float64MultiArray', current_command,
                ],
                condition=UnlessCondition(use_sim),
            )
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=controller_spawner,
                on_exit=[process],
            )))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(description_share, 'rviz', f'ffw_{robot}.rviz')],
        parameters=[{'use_sim_time': use_sim}],
        output='screen',
        condition=IfCondition(start_rviz),
    )
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=joint_state_spawner,
        on_exit=[rviz],
    )))

    bringup_launch_dir = os.path.join(bringup_share, 'launch')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, 'camera.launch.py')),
        launch_arguments={
            'head_camera_type': robot_config['head_camera_type'],
        }.items(),
        condition=IfCondition(launch_cameras),
    )
    actions.append(GroupAction(
        actions=[
            TimerAction(
                period=20.0,
                actions=[camera_launch],
                condition=IfCondition(init_position),
            ),
            TimerAction(
                period=10.0,
                actions=[camera_launch],
                condition=UnlessCondition(init_position),
            ),
        ],
        condition=UnlessCondition(use_sim),
    ))

    if base.startswith('ffw_swerve'):
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_launch_dir, 'lidar_dual.launch.py')),
        )
        actions.append(GroupAction(
            actions=[
                GroupAction(
                    actions=[
                        TimerAction(
                            period=20.0,
                            actions=[lidar_launch],
                            condition=IfCondition(init_position),
                        ),
                        TimerAction(
                            period=10.0,
                            actions=[lidar_launch],
                            condition=UnlessCondition(init_position),
                        ),
                    ],
                    condition=UnlessCondition(use_sim),
                ),
                Node(
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
                        'use_sim_time': use_sim,
                    }],
                ),
            ],
            condition=IfCondition(launch_lidar),
        ))

    actions.append(Node(
        package='ffw_bringup',
        executable='head_eef_tracker',
        name='head_eef_tracker',
        output='screen',
        condition=IfCondition(use_head_eef_tracker),
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', description='Follower robot profile.'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false'),
        DeclareLaunchArgument('port_name', default_value='/dev/follower'),
        DeclareLaunchArgument('launch_cameras', default_value='true'),
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        DeclareLaunchArgument('init_position', default_value='true'),
        DeclareLaunchArgument('use_head_eef_tracker', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
