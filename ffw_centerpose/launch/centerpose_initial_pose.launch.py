#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    initial_positions = PathJoinSubstitution([
        FindPackageShare('ffw_centerpose'),
        'config',
        'ffw_sg2_rev1_follower',
        'centerpose_initial_positions.yaml',
    ])

    arm_l = Node(
        package='ffw_centerpose',
        executable='joint_trajectory_topic_executor',
        name='arm_l_joint_trajectory_executor',
        parameters=[initial_positions],
        output='screen',
    )

    arm_r = Node(
        package='ffw_centerpose',
        executable='joint_trajectory_topic_executor',
        name='arm_r_joint_trajectory_executor',
        parameters=[initial_positions],
        output='screen',
    )

    head = Node(
        package='ffw_centerpose',
        executable='joint_trajectory_topic_executor',
        name='head_joint_trajectory_executor',
        parameters=[initial_positions],
        output='screen',
    )

    lift = Node(
        package='ffw_centerpose',
        executable='joint_trajectory_topic_executor',
        name='lift_joint_trajectory_executor',
        parameters=[initial_positions],
        output='screen',
    )

    return LaunchDescription([arm_l, arm_r, head, lift])
