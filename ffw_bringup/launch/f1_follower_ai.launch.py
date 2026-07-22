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
# Authors: Dongyun Kim
#
# F1 follower bringup. Hardware is identical to bg2 (no mobile base, arms + lift +
# head only) and reuses bg2 model/URDF/controller configs. The only difference is
# the head camera: f1 uses RealSense D455 instead of the ZED Mini used by bg2.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('start_rviz', default_value='false',
                              description='Whether to execute rviz2'),
        DeclareLaunchArgument('use_sim', default_value='false',
                              description='Start robot in Gazebo simulation.'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false',
                              description='Use mock hardware mirroring command.'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false',
                              description='Enable mock sensor commands.'),
        DeclareLaunchArgument('port_name', default_value='/dev/follower',
                              description='Port name for hardware connection.'),
        DeclareLaunchArgument('launch_cameras', default_value='true',
                              description='Whether to launch cameras.'),
        DeclareLaunchArgument('head_camera_type', default_value='realsense',
                              choices=['zed', 'realsense'],
                              description='Head camera type. realsense for f1 (D455 head), '
                                          'zed for sg2/bg2/sh5/bh5.'),
        DeclareLaunchArgument('camera_assignment_mode', default_value='auto',
                              choices=['auto', 'manual'],
                              description='Use automatic assignment or the manual camera YAML.'),
        DeclareLaunchArgument('init_position', default_value='true',
                              description='Whether to launch the init_position node.'),
        DeclareLaunchArgument('model', default_value='ffw_f1_follower',
                              description='Robot model name.'),
        DeclareLaunchArgument('use_head_eef_tracker', default_value='false',
                              description='Whether to launch the head EEF tracker node.'),
        DeclareLaunchArgument(
            'init_position_file',
            default_value='ffw_f1_follower_initial_positions.yaml',
            description='Initial position file.'),
        DeclareLaunchArgument(
            'ros2_control_type',
            default_value='ffw_f1_follower',
            description='Type of ros2_control',
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    port_name = LaunchConfiguration('port_name')
    launch_cameras = LaunchConfiguration('launch_cameras')
    head_camera_type = LaunchConfiguration('head_camera_type')
    camera_assignment_mode = LaunchConfiguration('camera_assignment_mode')
    init_position = LaunchConfiguration('init_position')
    model = LaunchConfiguration('model')
    use_head_eef_tracker = LaunchConfiguration('use_head_eef_tracker')
    init_position_file = LaunchConfiguration('init_position_file')
    ros2_control_type = LaunchConfiguration('ros2_control_type')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('ffw_description'),
                              'urdf',
                              model,
                              'ffw_f1_follower.urdf.xacro']),
        ' ',
        'use_sim:=', use_sim,
        ' ',
        'use_mock_hardware:=', use_mock_hardware,
        ' ',
        'mock_sensor_commands:=', mock_sensor_commands,
        ' ',
        'port_name:=', port_name,
        ' ',
        'model:=', model,
        ' ',
        'init_position_file:=', init_position_file,
        ' ',
        'ros2_control_type:=', ros2_control_type,
    ])

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('ffw_bringup'), 'config', model,
        'ffw_f1_follower_ai_hardware_controller.yaml'
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ffw_description'), 'rviz', 'ffw_f1.rviz'
    ])

    robot_description = {'robot_description': robot_description_content}

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_manager_config],
        output='both',
        condition=UnlessCondition(use_sim),
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim}],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(start_rviz)
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

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node]
        )
    )

    trajectory_params_file = PathJoinSubstitution([
        FindPackageShare('ffw_bringup'),
        'config',
        model,
        init_position_file,
    ])

    joint_trajectory_executor_left = Node(
        package='ffw_bringup',
        executable='joint_trajectory_executor',
        name='arm_l_joint_trajectory_executor',
        parameters=[trajectory_params_file],
        output='screen',
    )
    joint_trajectory_executor_right = Node(
        package='ffw_bringup',
        executable='joint_trajectory_executor',
        name='arm_r_joint_trajectory_executor',
        parameters=[trajectory_params_file],
        output='screen',
    )
    joint_trajectory_executor_head = Node(
        package='ffw_bringup',
        executable='joint_trajectory_executor',
        name='head_joint_trajectory_executor',
        parameters=[trajectory_params_file],
        output='screen',
    )
    joint_trajectory_executor_lift = Node(
        package='ffw_bringup',
        executable='joint_trajectory_executor',
        name='lift_joint_trajectory_executor',
        parameters=[trajectory_params_file],
        output='screen',
    )

    init_position_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                joint_trajectory_executor_left,
                joint_trajectory_executor_right,
                joint_trajectory_executor_head,
                joint_trajectory_executor_lift
            ]
        ),
        condition=IfCondition(init_position)
    )

    # Camera launch include — f1 default is realsense head camera
    bringup_launch_dir = PathJoinSubstitution([FindPackageShare('ffw_bringup'), 'launch'])
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_launch_dir,
                                                            'camera.launch.py'])),
        launch_arguments={
            'head_camera_type': head_camera_type,
            'camera_assignment_mode': camera_assignment_mode,
        }.items(),
        condition=IfCondition(launch_cameras)
    )

    # Camera timers with conditional delay based on init_position
    camera_timer_20s = TimerAction(period=20.0, actions=[camera_launch],
                                   condition=IfCondition(init_position))
    camera_timer_10s = TimerAction(period=10.0, actions=[camera_launch],
                                   condition=UnlessCondition(init_position))

    # Head EEF Tracker node
    head_eef_tracker_node = Node(
        package='ffw_bringup',
        executable='head_eef_tracker',
        name='head_eef_tracker',
        output='screen',
        condition=IfCondition(use_head_eef_tracker),
    )

    return LaunchDescription(
        declared_arguments + [
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            robot_controller_spawner,
            init_position_event_handler,
            camera_timer_20s,
            camera_timer_10s,
            head_eef_tracker_node,
        ]
    )
