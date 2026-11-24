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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'port_name', default_value='/dev/serial/by-role/leader_a1',
            description='A13 shared bus; use /dev/ttyACM0 for laptop testing.',
        ),
        DeclareLaunchArgument(
            'style', default_value='default', choices=['default', 'advanced'],
            description='Leader control style; A13 uses the pedal instead of physical triggers.',
        ),
        DeclareLaunchArgument('launch_pedal', default_value='true'),
        DeclareLaunchArgument('launch_hands', default_value='true',
                              description='Start A3 landmark publishing and HX5 retargeting.'),
        DeclareLaunchArgument('hand_side', default_value='both',
                              choices=['left', 'right', 'both']),
        DeclareLaunchArgument(
            'pedal_port_name',
            default_value=(
                '/dev/serial/by-id/'
                'usb-ROBOTIS_OpenRB-150_2DFEE81F503059384C2E3120FF050A3D-if00'),
            description='Pedal serial device; must be different from the A13 shared bus.',
        ),
        DeclareLaunchArgument('pedal_baud_rate', default_value='1000000'),
        DeclareLaunchArgument('pedal_long_press_seconds', default_value='1.0'),
        DeclareLaunchArgument('pedal_desired_position', default_value='0.5'),
        DeclareLaunchArgument('pedal_pressed_center', default_value='0.0'),
        DeclareLaunchArgument('pedal_pressed_window', default_value='0.1'),
        DeclareLaunchArgument(
            'leader_input_file', default_value=PathJoinSubstitution([
                FindPackageShare('ffw_bringup'), 'config', 'ffw_a13_leader',
                'ffw_a13_leader_input.yaml']),
        ),
        DeclareLaunchArgument(
            'update_rate', default_value='60', description='Controller update frequency in Hz.',
        ),
        DeclareLaunchArgument(
            'namespace', default_value='leader',
            description='Namespace for this A13 controller manager and joint states.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware', default_value='false',
            description='Use mock hardware instead of connecting to the leader.',
        ),
        DeclareLaunchArgument(
            'mock_sensor_commands', default_value='false',
            description='Enable mock sensor commands.',
        ),
        DeclareLaunchArgument(
            'start_rviz', default_value='false',
            description='Show arm and finger TF frames in RViz.',
        ),
        DeclareLaunchArgument(
            'description_file', default_value='ffw_a13_leader.urdf.xacro',
            description='A13 wrapper Xacro containing the model and ros2_control system.',
        ),
    ]

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('ffw_description'), 'urdf', 'ffw_a13_leader',
            LaunchConfiguration('description_file'),
        ]),
        ' port_name:=', LaunchConfiguration('port_name'),
        ' use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'),
        ' mock_sensor_commands:=', LaunchConfiguration('mock_sensor_commands'),
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
    }
    controller_config = PathJoinSubstitution([
        FindPackageShare('ffw_bringup'), 'config', 'ffw_a13_leader',
        'ffw_a13_leader_ai_hardware_controller.yaml',
    ])
    leader_input = ParameterFile(LaunchConfiguration('leader_input_file'), allow_substs=True)
    rviz_config = PathJoinSubstitution([
        FindPackageShare('ffw_description'), 'rviz', 'ffw_a13_leader.rviz',
    ])

    return LaunchDescription(declared_arguments + [
        GroupAction(actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            Node(
                package='robot_state_publisher', executable='robot_state_publisher',
                parameters=[robot_description, {'frame_prefix': 'leader_'}], output='both',
            ),
            Node(
                package='controller_manager', executable='ros2_control_node',
                parameters=[controller_config, leader_input, {
                    'update_rate': ParameterValue(
                        LaunchConfiguration('update_rate'), value_type=int),
                }],
                output='both',
            ),
            Node(
                package='controller_manager', executable='spawner',
                arguments=['joint_state_broadcaster',
                           'arm_l_state_broadcaster', 'arm_r_state_broadcaster',
                           'hand_l_state_broadcaster', 'hand_r_state_broadcaster',
                           'joint_trajectory_command_broadcaster',
                           ['--controller-ros-args=-p style:=', LaunchConfiguration('style')],
                           '--controller-manager',
                           'controller_manager',
                           '--controller-ros-args',
                           '-r hand_l_state_broadcaster:~/joint_states:=/a3_left/joint_states '
                           '-r hand_r_state_broadcaster:~/joint_states:=/a3_right/joint_states'],
                output='both',
            ),
            Node(
                package='rviz2', executable='rviz2',
                arguments=['-d', rviz_config],
                condition=IfCondition(LaunchConfiguration('start_rviz')),
                output='both',
            ),
        ]),
        GroupAction(condition=IfCondition(LaunchConfiguration('launch_hands')), actions=[
            Node(package='cyclo_motion_controller_ros_py', executable='a3_hand_joints',
                 parameters=[{'hand_side': LaunchConfiguration('hand_side')}], output='screen'),
            Node(package='cyclo_motion_controller_ros_py', executable='hx5_retargeting',
                 parameters=[{'side': LaunchConfiguration('hand_side'), 'use_pedal': True}],
                 output='screen'),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ffw_teleop'), 'launch', 'pedal_hardware.launch.py'])),
            condition=IfCondition(LaunchConfiguration('launch_pedal')),
            launch_arguments={
                'namespace': 'pedal',
                'port_name': LaunchConfiguration('pedal_port_name'),
                'baud_rate': LaunchConfiguration('pedal_baud_rate'),
                'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
                'long_press_seconds': LaunchConfiguration('pedal_long_press_seconds'),
                'desired_position': LaunchConfiguration('pedal_desired_position'),
                'pressed_center': LaunchConfiguration('pedal_pressed_center'),
                'pressed_window': LaunchConfiguration('pedal_pressed_window'),
            }.items(),
        ),
    ])
