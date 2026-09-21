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

from ffw_bringup.leader_initializer import initialize_leader

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'model',
            default_value='ffw_lg2_leader',
            description='Robot model name.',
        ),
        DeclareLaunchArgument(
            'description_file',
            default_value='ffw_lg2_leader.urdf.xacro',
            description='URDF/XACRO file for the robot model.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware mirroring command.',
        ),
        DeclareLaunchArgument(
            'use_foot_switch',
            default_value='true',
            description='Whether to launch the foot switch node.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    model = LaunchConfiguration('model')
    description_file = LaunchConfiguration('description_file')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    use_foot_switch = LaunchConfiguration('use_foot_switch')

    init_info = initialize_leader(detect_port=False)

    # Robot controllers config file path
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('ffw_bringup'),
        'config',
        model,
        'ffw_lg2_leader_ai_hardware_controller.yaml',
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('ffw_description'),
            'urdf',
            model,
            description_file,
        ]),
        ' ',
        'use_mock_hardware:=', use_mock_hardware,
    ])

    robot_description = {'robot_description': robot_description_content}

    # ros2_control Node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_command_broadcaster',
            'spring_actuator_controller_left',
            'spring_actuator_controller_right',
            'joystick_controller',
            'joint_state_broadcaster',
        ],
        parameters=[
            robot_description,
            {
                'follower_current_position': init_info['follower_current_position'],
                'follower_end_tool': init_info['follower_end_tool'],
                'follower_joint_limits': init_info['follower_joint_limits'],
            },
        ],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'frame_prefix': 'leader_'}],
    )

    foot_switch_node = Node(
        package='ffw_bringup',
        executable='foot_switch_node',
        name='foot_switch_node',
        output='both',
        parameters=[{'controller_config_path': robot_controllers}],
        condition=IfCondition(use_foot_switch),
    )

    gripper_to_hand_node = Node(
        package='ffw_joint_trajectory_command_broadcaster',
        executable='gripper_to_hand',
        name='gripper_to_hand',
        parameters=[{'follower_end_tool': init_info['follower_end_tool']}],
        output='screen',
        condition=IfCondition(str('hand' in init_info['follower_end_tool'].values())),
    )

    # Wrap everything in a namespace 'leader'
    leader_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('leader'),
            control_node,
            robot_controller_spawner,
            robot_state_publisher_node,
            gripper_to_hand_node,
        ]
    )

    return [leader_with_namespace, foot_switch_node]
