#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'color_topic', default_value='/zedm/zed_node/left/image_rect_color'
        ),
        DeclareLaunchArgument(
            'depth_topic', default_value='/zedm/zed_node/depth/depth_registered'
        ),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/zedm/zed_node/left/camera_info'
        ),
        DeclareLaunchArgument('output_topic', default_value='/camera/table_plane'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='',
            description='Empty -> reuse the incoming depth image header frame_id.',
        ),
        DeclareLaunchArgument('pixel_step', default_value='4'),
        DeclareLaunchArgument('min_depth', default_value='0.15'),
        DeclareLaunchArgument('max_depth', default_value='2.0'),
        DeclareLaunchArgument('plane_distance_threshold', default_value='0.01'),
        DeclareLaunchArgument('ransac_iterations', default_value='150'),
        DeclareLaunchArgument('flatten_to_plane', default_value='true'),
        DeclareLaunchArgument('min_inliers', default_value='200'),
    ]

    node = Node(
        package='ffw_bringup',
        executable='table_plane_pointcloud',
        name='table_plane_pointcloud',
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
            'plane_distance_threshold': LaunchConfiguration('plane_distance_threshold'),
            'ransac_iterations': LaunchConfiguration('ransac_iterations'),
            'flatten_to_plane': LaunchConfiguration('flatten_to_plane'),
            'min_inliers': LaunchConfiguration('min_inliers'),
        }],
    )

    return LaunchDescription(arguments + [node])
