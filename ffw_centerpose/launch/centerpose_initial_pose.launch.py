#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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
# Author: Seongjin Jeong

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
