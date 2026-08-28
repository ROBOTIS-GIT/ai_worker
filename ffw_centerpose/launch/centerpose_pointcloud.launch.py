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
    camera = load_camera_topics()

    arguments = [
        # --- Crop ------------------------------------------------------------
        DeclareLaunchArgument(
            'input_topic', default_value=camera['point_cloud']
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/centerpose/cloud_cropped',
            description='Same namespace as marker_topic/table_plane_topic so all '
                        'three show up grouped together in RViz.',
        ),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'x_max',
            default_value='1.2',
            description='Keep points with 0 <= x <= x_max in target_frame (forward '
                        'reach from the robot).',
        ),
        DeclareLaunchArgument(
            'y_extent',
            default_value='1.0',
            description='Keep points with -y_extent <= y <= y_extent in target_frame '
                        '(left/right reach); 1.0 keeps a 2m-wide band. 0 (or '
                        'negative) disables this bound -- left/right is left '
                        'uncropped.',
        ),
        DeclareLaunchArgument(
            'z_min',
            default_value='-10.0',
            description='Height bound, effectively disabled by default.',
        ),
        DeclareLaunchArgument('z_max', default_value='10.0'),

        # --- Table plane -----------------------------------------------------
        DeclareLaunchArgument(
            'table_plane_topic', default_value='/centerpose/table_plane'
        ),
        DeclareLaunchArgument(
            'color_topic',
            default_value=camera['image_compressed'],
            description='Color image reprojected onto the real table plane using '
                        'the camera pose (not per-pixel depth) to texture the '
                        'raster above.',
        ),
        DeclareLaunchArgument(
            'freeze_table_plane',
            default_value='false',
            description='Once a table plane raster is successfully captured, keep '
                        'republishing that exact snapshot (position + color) '
                        'forever instead of refitting/retexturing every frame -- '
                        'freezes the display to whatever the camera saw the moment '
                        'it was first captured.',
        ),

        # --- Bbox markers ------------------------------------------------------
        DeclareLaunchArgument('detections_topic', default_value='/centerpose/detections'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value=camera['camera_info']
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value=camera['depth'],
            description='Real metric depth used to correct CenterPose position, '
                        'which otherwise comes from an assumed canonical object '
                        'size.',
        ),
        DeclareLaunchArgument('marker_topic', default_value='/centerpose/bbox_markers'),
    ]

    node = Node(
        package='ffw_centerpose',
        executable='centerpose_pointcloud',
        name='centerpose_pointcloud',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'target_frame': LaunchConfiguration('target_frame'),
            'x_max': LaunchConfiguration('x_max'),
            'y_extent': LaunchConfiguration('y_extent'),
            'z_min': LaunchConfiguration('z_min'),
            'z_max': LaunchConfiguration('z_max'),

            'table_plane_topic': LaunchConfiguration('table_plane_topic'),
            'color_topic': LaunchConfiguration('color_topic'),
            'freeze_table_plane': LaunchConfiguration('freeze_table_plane'),

            'detections_topic': LaunchConfiguration('detections_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'marker_topic': LaunchConfiguration('marker_topic'),
        }],
    )

    return LaunchDescription(arguments + [node])
