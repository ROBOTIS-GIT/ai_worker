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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('model', default_value='ffw_sg2_rev1_follower',
                              description='Robot model name.'),
        DeclareLaunchArgument('world', default_value='table_cube',
                              description='Gz sim World'),
        DeclareLaunchArgument('robot_x', default_value='-0.05',
                              description='Initial robot x position.'),
        DeclareLaunchArgument('robot_y', default_value='0.0',
                              description='Initial robot y position.'),
        DeclareLaunchArgument('robot_z', default_value='0.05',
                              description='Initial robot z position.'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0',
                              description='Initial robot yaw angle.'),
    ]

    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')

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
                              'ffw_sg2_follower_table_camera.urdf.xacro']),
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
                   '-x', robot_x,
                   '-y', robot_y,
                   '-z', robot_z,
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', robot_yaw,
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
            'swerve_drive_controller',
        ],
        parameters=[robot_description],
    )

    gz_bridge_params_path = os.path.join(
        ffw_bringup_path,
        'config',
        'ffw_sg2_rev1_follower',
        'table_gz_bridge.yaml'
    )

    table_initial_positions_config = os.path.join(
        ffw_bringup_path,
        'config',
        'ffw_sg2_rev1_follower',
        'table_initial_positions.yaml'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={gz_bridge_params_path}'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    zedm_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='zedm_rgb_image_bridge',
        output='screen',
        arguments=['/zedm/rgbd_camera/image'],
        parameters=[{'use_sim_time': True}],
    )

    zedm_depth_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='zedm_depth_image_bridge',
        output='screen',
        arguments=['/zedm/rgbd_camera/depth_image'],
        parameters=[{'use_sim_time': True}],
    )

    table_initial_left_arm = Node(
        package='ffw_bringup',
        executable='joint_trajectory_executor',
        name='arm_l_joint_trajectory_executor',
        output='screen',
        parameters=[table_initial_positions_config],
    )

    table_initial_right_arm = Node(
        package='ffw_bringup',
        executable='joint_trajectory_executor',
        name='arm_r_joint_trajectory_executor',
        output='screen',
        parameters=[table_initial_positions_config],
    )

    table_initial_head = Node(
        package='ffw_bringup',
        executable='joint_trajectory_executor',
        name='head_joint_trajectory_executor',
        output='screen',
        parameters=[table_initial_positions_config],
    )

    table_initial_lift = Node(
        package='ffw_bringup',
        executable='joint_trajectory_executor',
        name='lift_joint_trajectory_executor',
        output='screen',
        parameters=[table_initial_positions_config],
    )

    table_initial_pose_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                table_initial_left_arm,
                table_initial_right_arm,
                table_initial_head,
                table_initial_lift,
            ]
        )
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
    )

    return LaunchDescription([
        *declared_arguments,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=joint_state_broadcaster_spawner,
               on_exit=[robot_controller_spawner],
            )
        ),
        table_initial_pose_event_handler,
        bridge,
        zedm_image_bridge,
        zedm_depth_image_bridge,
        dual_laser_merger_node,
        gazebo_resource_path,
        gazebo,
        robot_state_pub_node,
        gz_spawn_entity,
        rviz,
    ])
