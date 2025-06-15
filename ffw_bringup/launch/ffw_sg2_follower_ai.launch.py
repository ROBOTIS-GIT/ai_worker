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
        DeclareLaunchArgument('start_rviz', default_value='true',
                              description='Whether to execute rviz2'),
        DeclareLaunchArgument('use_sim', default_value='false',
                              description='Start robot in Gazebo simulation.'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                              description='Use fake hardware mirroring command.'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false',
                              description='Enable fake sensor commands.'),
        DeclareLaunchArgument('port_name', default_value='/dev/follower',
                              description='Port name for hardware connection.'),
        DeclareLaunchArgument('launch_cameras', default_value='true',
                              description='Whether to launch cameras.'),
        DeclareLaunchArgument('init_position', default_value='true',
                              description='Whether to launch the init_position node.'),
        DeclareLaunchArgument('model', default_value='ffw_sg2_rev1_follower',
                              description='Robot model name.'),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    port_name = LaunchConfiguration('port_name')
    launch_cameras = LaunchConfiguration('launch_cameras')
    init_position = LaunchConfiguration('init_position')
    model = LaunchConfiguration('model')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('ffw_description'),
                              'urdf',
                              model,
                              'ffw_sg2_follower.urdf.xacro']),
        ' ',
        'use_sim:=', use_sim,
        ' ',
        'use_fake_hardware:=', use_fake_hardware,
        ' ',
        'fake_sensor_commands:=', fake_sensor_commands,
        ' ',
        'port_name:=', port_name,
        ' ',
        'model:=', model,
    ])

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('ffw_bringup'), 'config', model,
        'ffw_sg2_follower_ai_hardware_controller.yaml'
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ffw_description'), 'rviz', 'ffw.rviz'
    ])

    robot_description = {'robot_description': robot_description_content}

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_manager_config],
        output='both',
        condition=UnlessCondition(use_sim),
        remappings=[
            ('/arm_l_controller/joint_trajectory',
             '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory'),
            ('/arm_r_controller/joint_trajectory',
             '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory'),
            ('/head_controller/joint_trajectory',
             '/leader/joystick_controller_left/joint_trajectory'),
            ('/lift_controller/joint_trajectory',
             '/leader/joystick_controller_right/joint_trajectory')
        ]
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
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_l_controller',
            'arm_r_controller',
            'head_controller',
            'lift_controller'
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
        'ffw_sg2_follower_initial_positions.yaml',
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

    # Camera launch include
    bringup_launch_dir = PathJoinSubstitution([FindPackageShare('ffw_bringup'), 'launch'])
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_launch_dir,
                                                            'camera.launch.py'])),
        condition=IfCondition(launch_cameras)
    )

    # Camera timers with conditional delay based on init_position
    camera_timer_20s = TimerAction(period=20.0, actions=[camera_launch],
                                   condition=IfCondition(init_position))
    camera_timer_10s = TimerAction(period=10.0, actions=[camera_launch],
                                   condition=UnlessCondition(init_position))

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
        ]
    )
