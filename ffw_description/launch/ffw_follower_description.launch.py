#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    use_gui = LaunchConfiguration('use_gui')

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('ffw_description'),
            'urdf',
            'follower',
            'ffw_follower.urdf.xacro',
        ]),
        ' robot:=',
        robot,
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('ffw_description'),
        'rviz',
        ['ffw_', robot, '.rviz'],
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            description='Follower robot: bg2, sg2, bh5, sh5, f1, or f2',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Run joint_state_publisher_gui',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),
    ])
