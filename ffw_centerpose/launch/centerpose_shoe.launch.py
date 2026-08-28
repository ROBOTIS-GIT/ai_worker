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

from ffw_centerpose.pick_place_base import load_camera_topics
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Measured startup/grasp/yaw calibration + place slot positions are read directly
    # from config/ffw_sg2_rev1_follower/centerpose_shoe_calibration.yaml by the node itself.
    camera = load_camera_topics()

    arguments = [
        # --- Detection / vision input -----------------------------------------
        DeclareLaunchArgument('detections_topic', default_value='/centerpose/detections'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value=camera['camera_info']
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value=camera['depth'],
            description='Real metric depth used for x/y only; z always stays fixed.',
        ),
        DeclareLaunchArgument('depth_window', default_value='5'),
        DeclareLaunchArgument(
            'detection_timeout',
            default_value='10.0',
            description='CenterPose inference rate is low/bursty; a fresh detection can '
                        'lag several seconds behind capture time.',
        ),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('projection_frame', default_value=''),

        # --- Safety --------------------------------------------------------------
        DeclareLaunchArgument(
            'max_grasp_x',
            default_value='0.9',
            description='Safety bound: ~/execute refuses to move at all if any '
                        'captured pose x exceeds this. Wider than '
                        'centerpose_box/centerpose_bottle (0.7) since shoes are worked on '
                        'further out (measured x=0.87).',
        ),

        # --- Motion execution toggle ----------------------------------------------
        DeclareLaunchArgument('execute_motion', default_value='false'),
        DeclareLaunchArgument('movel_topic', default_value='/l_goal_move'),

        # --- Approach / insertion --------------------------------------------------
        DeclareLaunchArgument('pregrasp_distance', default_value='0.1'),
    ]

    node = Node(
        package='ffw_centerpose',
        executable='centerpose_shoe',
        name='centerpose_shoe',
        output='screen',
        parameters=[
            {
                'detections_topic': LaunchConfiguration('detections_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'depth_window': LaunchConfiguration('depth_window'),
                'detection_timeout': LaunchConfiguration('detection_timeout'),
                'target_frame': LaunchConfiguration('target_frame'),
                'projection_frame': LaunchConfiguration('projection_frame'),

                'max_grasp_x': LaunchConfiguration('max_grasp_x'),

                'execute_motion': LaunchConfiguration('execute_motion'),
                'movel_topic': LaunchConfiguration('movel_topic'),

                'pregrasp_distance': LaunchConfiguration('pregrasp_distance'),
            },
        ],
    )

    return LaunchDescription(arguments + [node])
