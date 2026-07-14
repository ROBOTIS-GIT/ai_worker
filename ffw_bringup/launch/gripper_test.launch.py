#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('open_position', default_value='0.25'),
        DeclareLaunchArgument('closed_position', default_value='0.55'),
        DeclareLaunchArgument('duration', default_value='1.5'),
        DeclareLaunchArgument('settle_time', default_value='1.0'),
        DeclareLaunchArgument('cycles', default_value='2'),
        DeclareLaunchArgument(
            'joint_trajectory_topic',
            default_value='/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
        ),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
    ]

    gripper_test = Node(
        package='ffw_bringup',
        executable='gripper_test',
        name='gripper_test',
        output='screen',
        parameters=[{
            'open_position': LaunchConfiguration('open_position'),
            'closed_position': LaunchConfiguration('closed_position'),
            'duration': LaunchConfiguration('duration'),
            'settle_time': LaunchConfiguration('settle_time'),
            'cycles': LaunchConfiguration('cycles'),
            'joint_trajectory_topic': LaunchConfiguration('joint_trajectory_topic'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
        }],
    )

    return LaunchDescription(declared_arguments + [gripper_test])
