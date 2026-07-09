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
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
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
        DeclareLaunchArgument('launch_lidar', default_value='true',
                              description='Whether to launch lidar.'),
        DeclareLaunchArgument('init_position', default_value='true',
                              description='Whether to launch the init_position node.'),
        DeclareLaunchArgument('model', default_value='ffw_sg2_rev1_follower',
                              description='Robot model name.'),
        DeclareLaunchArgument('use_head_eef_tracker', default_value='false',
                              description='Whether to launch the head EEF tracker node.'),
        DeclareLaunchArgument(
            'init_position_file',
            default_value='ffw_sg2_follower_initial_positions.yaml',
            description='Initial position file.'),
        DeclareLaunchArgument(
            'ros2_control_type',
            default_value='ffw_sg2_follower',
            description='Type of ros2_control',
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    port_name = LaunchConfiguration('port_name')
    launch_cameras = LaunchConfiguration('launch_cameras')
    launch_lidar = LaunchConfiguration('launch_lidar')
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
                              'ffw_sg2_follower.urdf.xacro']),
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
        'ffw_sg2_follower_ai_hardware_controller.yaml'
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ffw_description'), 'rviz', 'ffw_sg2.rviz'
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
            'ffw_robot_manager'
        ],
        parameters=[robot_description],
    )

    # Separate spawner for swerve_steering_initial_position_controller
    swerve_steering_initial_position_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_steering_initial_position_controller'],
        parameters=[robot_description],
        condition=IfCondition(init_position),
    )

    # Unspawner for swerve_steering_initial_position_controller
    swerve_steering_initial_position_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=['swerve_steering_initial_position_controller'],
        output='screen',
    )

    # Spawner for swerve_drive_controller
    swerve_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_drive_controller'],
        parameters=[robot_description],
        output='screen',
    )

    # Direct spawner for swerve_drive_controller when init_position is false
    swerve_drive_spawner_direct = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_drive_controller'],
        parameters=[robot_description],
        output='screen',
        condition=UnlessCondition(init_position),
    )

    # Add a TimerAction to delay swerve_drive_spawner by 5 seconds after unspawning
    swerve_drive_spawner_delayed = TimerAction(
        period=5.0,
        actions=[swerve_drive_spawner],
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
    joint_trajectory_executor_swerve_steering = Node(
        package='ffw_bringup',
        executable='joint_trajectory_executor',
        name='swerve_steering_joint_trajectory_executor',
        parameters=[trajectory_params_file],
        output='screen',
        condition=IfCondition(init_position),
    )

    init_position_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                joint_trajectory_executor_left,
                joint_trajectory_executor_right,
                joint_trajectory_executor_head,
                joint_trajectory_executor_lift,
                joint_trajectory_executor_swerve_steering
            ]
        ),
        condition=IfCondition(init_position)
    )

    # Event handler to unspawn swerve_steering_initial_position_controller and
    # spawn swerve_drive_controller
    # when joint_trajectory_executor_swerve_steering is done
    swerve_controller_switch_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_executor_swerve_steering,
            on_exit=[
                swerve_steering_initial_position_unspawner,
                swerve_drive_spawner_delayed
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

    # Lidar launch include
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_launch_dir,
                                                            'lidar_dual.launch.py'])),
        condition=IfCondition(launch_lidar)
    )

    # Lidar timers with conditional delay based on init_position
    lidar_timer_20s = TimerAction(period=20.0, actions=[lidar_launch],
                                  condition=IfCondition(init_position))
    lidar_timer_10s = TimerAction(period=10.0, actions=[lidar_launch],
                                  condition=UnlessCondition(init_position))

    # Head EEF Tracker node
    head_eef_tracker_node = Node(
        package='ffw_bringup',
        executable='head_eef_tracker',
        name='head_eef_tracker',
        output='screen',
        condition=IfCondition(use_head_eef_tracker),
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
            'use_sim_time': use_sim,
        }],
    )

    return LaunchDescription(
        declared_arguments + [
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            robot_controller_spawner,
            swerve_steering_initial_position_spawner,
            swerve_drive_spawner_direct,
            init_position_event_handler,
            swerve_controller_switch_event_handler,
            camera_timer_20s,
            camera_timer_10s,
            lidar_timer_20s,
            lidar_timer_10s,
            head_eef_tracker_node,
            dual_laser_merger_node,
        ]
    )
