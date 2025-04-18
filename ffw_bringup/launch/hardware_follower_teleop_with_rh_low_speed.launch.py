#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
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
# Author: Wonho Yoon, Sungho Woo

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
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
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    port_name = LaunchConfiguration('port_name')

    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('ffw_description'),
                              'urdf',
                              'follower',
                              'ffw_follower_with_rh_low_speed.urdf.xacro']),
        ' ',
        'use_sim:=', use_sim,
        ' ',
        'use_fake_hardware:=', use_fake_hardware,
        ' ',
        'fake_sensor_commands:=', fake_sensor_commands,
        ' ',
        'port_name:=', port_name,
    ])

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('ffw_bringup'), 'config', 'follower_with_rh_hardware_controller.yaml'
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ffw_description'), 'rviz', 'ffw.rviz'
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': urdf_file}, controller_manager_config],
        output='both',
        condition=UnlessCondition(use_sim),
        remappings=[
            ('/arm_l_controller/joint_trajectory',
             '/leader/joint_trajectory_left/joint_trajectory'),
            ('/arm_r_controller/joint_trajectory',
             '/leader/joint_trajectory_right/joint_trajectory')
        ]
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_file, 'use_sim_time': use_sim}],
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

    arm_l_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_l_controller'],
        output='screen'
    )

    arm_r_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_r_controller'],
        output='screen'
    )

    neck_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['neck_controller'],
        output='screen'
    )

    body_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['body_controller'],
        output='screen'
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node]
        )
    )

    delay_arm_l_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_l_controller_spawner]
        )
    )

    delay_arm_r_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_r_controller_spawner]
        )
    )

    delay_neck_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[neck_controller_spawner]
        )
    )

    delay_body_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[body_controller_spawner]
        )
    )

    return LaunchDescription(
        declared_arguments + [
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_arm_l_controller_spawner_after_joint_state_broadcaster_spawner,
            delay_arm_r_controller_spawner_after_joint_state_broadcaster_spawner,
            delay_body_controller_spawner_after_joint_state_broadcaster_spawner,
            delay_neck_controller_spawner_after_joint_state_broadcaster_spawner,
        ]
    )
