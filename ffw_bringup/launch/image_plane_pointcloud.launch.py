#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'input_topic',
            default_value='/zedm/zed_node/left/image_rect_color/compressed',
            description='Compressed transport -- raw Image never arrives over this '
                        'robot\'s network link, only compressed/compressedDepth do.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/zedm/zed_node/left/camera_info'
        ),
        DeclareLaunchArgument('output_topic', default_value='/camera/image_plane'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='',
            description='Empty -> reuse the incoming image header frame_id.',
        ),
        DeclareLaunchArgument('pixel_step', default_value='4'),
        DeclareLaunchArgument('pixel_size', default_value='0.002'),
        DeclareLaunchArgument(
            'use_depth_for_distance',
            default_value='true',
            description='Take the plane distance from the real depth camera '
                        '(median of valid readings) instead of a fixed number.',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/zedm/zed_node/depth/depth_registered/compressedDepth',
        ),
        DeclareLaunchArgument('plane_distance', default_value='1.0'),
    ]

    node = Node(
        package='ffw_bringup',
        executable='image_plane_pointcloud',
        name='image_plane_pointcloud',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'pixel_step': LaunchConfiguration('pixel_step'),
            'pixel_size': LaunchConfiguration('pixel_size'),
            'use_depth_for_distance': LaunchConfiguration('use_depth_for_distance'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'plane_distance': LaunchConfiguration('plane_distance'),
        }],
    )

    return LaunchDescription(arguments + [node])
