#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'color_topic',
            default_value='/zedm/zed_node/left/image_rect_color/compressed',
            description='Compressed transport -- raw Image never arrives over this '
                        'robot\'s network link, only compressed/compressedDepth do.',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/zedm/zed_node/depth/depth_registered/compressedDepth',
        ),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/zedm/zed_node/left/camera_info'
        ),
        DeclareLaunchArgument('output_topic', default_value='/camera/depth_pointcloud'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='',
            description='Empty -> reuse the incoming depth image header frame_id.',
        ),
        DeclareLaunchArgument('pixel_step', default_value='2'),
        DeclareLaunchArgument(
            'min_depth', default_value='0.0', description='0 disables the lower bound.'
        ),
        DeclareLaunchArgument(
            'max_depth', default_value='0.0', description='0 disables the upper bound.'
        ),
    ]

    node = Node(
        package='ffw_bringup',
        executable='depth_to_pointcloud',
        name='depth_to_pointcloud',
        output='screen',
        parameters=[{
            'color_topic': LaunchConfiguration('color_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'pixel_step': LaunchConfiguration('pixel_step'),
            'min_depth': LaunchConfiguration('min_depth'),
            'max_depth': LaunchConfiguration('max_depth'),
        }],
    )

    return LaunchDescription(arguments + [node])
