#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('joint_name', default_value='gripper_l_joint1'),
        DeclareLaunchArgument(
            'joint_trajectory_topic',
            default_value='/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
        ),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('open_position', default_value='0.0'),
        DeclareLaunchArgument('closed_position', default_value='1.1'),
        DeclareLaunchArgument(
            'command_rate_hz',
            default_value='300.0',
            description='Rate to keep re-asserting the target so it stays competitive '
                        'with the MoveL controller streaming to the same topic.',
        ),
        DeclareLaunchArgument('point_lead_time', default_value='0.1'),
    ]

    node = Node(
        package='ffw_bringup',
        executable='left_gripper_controller',
        name='left_gripper_controller',
        output='screen',
        parameters=[{
            'joint_name': LaunchConfiguration('joint_name'),
            'joint_trajectory_topic': LaunchConfiguration('joint_trajectory_topic'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'open_position': LaunchConfiguration('open_position'),
            'closed_position': LaunchConfiguration('closed_position'),
            'command_rate_hz': LaunchConfiguration('command_rate_hz'),
            'point_lead_time': LaunchConfiguration('point_lead_time'),
        }],
    )

    return LaunchDescription(arguments + [node])
