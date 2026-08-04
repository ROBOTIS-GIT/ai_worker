#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        # --- Crop ------------------------------------------------------------
        DeclareLaunchArgument(
            'input_topic', default_value='/zedm/zed_node/point_cloud/cloud_registered'
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
            'publish_table_plane',
            default_value='true',
            description='RANSAC-fit the dominant flat surface (the table) among '
                        'the cropped points, then republish a color-image-textured '
                        'raster on that real plane/height -- not a point-cloud '
                        'reconstruction, so it does not flicker with depth noise.',
        ),
        DeclareLaunchArgument(
            'table_plane_topic', default_value='/centerpose/table_plane'
        ),
        DeclareLaunchArgument('table_plane_distance_threshold', default_value='0.01'),
        DeclareLaunchArgument('table_plane_ransac_iterations', default_value='150'),
        DeclareLaunchArgument('table_plane_min_inliers', default_value='200'),
        DeclareLaunchArgument('table_plane_max_ransac_points', default_value='20000'),
        DeclareLaunchArgument(
            'table_plane_normal_z_min',
            default_value='0.8',
            description='Reject the fitted plane if tilted more than ~37 degrees '
                        'off horizontal -- guards against locking onto a wall or '
                        'the robot itself instead of the table.',
        ),
        DeclareLaunchArgument(
            'table_plane_smoothing_alpha',
            default_value='0.15',
            description='Blend each new plane fit (pose + footprint) into a '
                        'running estimate; higher = faster to track, jitterier.',
        ),
        DeclareLaunchArgument(
            'table_plane_grid_resolution',
            default_value='0.004',
            description='Output raster cell size (m) on the real table plane -- '
                        'closer to an image resolution knob than a point density '
                        'one, since the raster is redrawn on the smoothed plane '
                        'each frame, not built from individual noisy points.',
        ),
        DeclareLaunchArgument(
            'color_topic',
            default_value='/zedm/zed_node/left/image_rect_color/compressed',
            description='Color image reprojected onto the real table plane using '
                        'the camera pose (not per-pixel depth) to texture the '
                        'raster above.',
        ),
        DeclareLaunchArgument(
            'below_table_margin',
            default_value='0.02',
            description='Points more than this far below the smoothed table '
                        'height are dropped from the main cropped cloud.',
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

        # --- Wall (bent up from the table's far edge) --------------------------
        DeclareLaunchArgument(
            'enable_wall',
            default_value='true',
            description='Add a second raster standing straight up from the '
                        'table\'s far edge (farthest from the robot along '
                        'base_link +X), textured the same way as the table, so '
                        'the color image beyond the table footprint doesn\'t just '
                        'get cut off.',
        ),
        DeclareLaunchArgument(
            'wall_min_height',
            default_value='0.02',
            description='The wall\'s height is solved geometrically each frame so '
                        'its own projection reaches the top row of the image -- '
                        'stood up tall enough to cover everything above the '
                        'table the camera can see. Clamped to '
                        '[wall_min_height, wall_max_height] as a safety bound.',
        ),
        DeclareLaunchArgument('wall_max_height', default_value='2.0'),

        # --- Bbox markers ------------------------------------------------------
        DeclareLaunchArgument('detections_topic', default_value='/centerpose/detections'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/zedm/zed_node/left/camera_info'
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/zedm/zed_node/depth/depth_registered',
            description='Real metric depth used to correct CenterPose position, '
                        'which otherwise comes from an assumed canonical object '
                        'size.',
        ),
        DeclareLaunchArgument('depth_window', default_value='5'),
        DeclareLaunchArgument('marker_topic', default_value='/centerpose/bbox_markers'),
        DeclareLaunchArgument(
            'marker_color_rgba',
            default_value='[1.0, 0.3, 0.0, 0.60]',
            description='Translucent cube color as [r, g, b, a], each 0-1.',
        ),
        DeclareLaunchArgument(
            'bbox_size_scale',
            default_value='0.2',
            description='Flat correction for CenterPose bbox.size, measured against '
                        'the real object (~1/5 scale).',
        ),
        DeclareLaunchArgument(
            'marker_lifetime',
            default_value='2.0',
            description='Seconds before an unrefreshed marker auto-expires in RViz. '
                        'Needs to comfortably outlast the gap between CenterPose\'s '
                        'own (slow/bursty) detections -- too short makes the marker '
                        'flicker on/off between detections.',
        ),
        DeclareLaunchArgument(
            'marker_hold_timeout',
            default_value='1.0',
            description='Seconds to keep redrawing a marker at its last known '
                        'position/size after its detection stops refreshing, before '
                        'actually deleting it. Bridges single dropped/failed frames '
                        'so the marker holds steady instead of flickering.',
        ),
        DeclareLaunchArgument(
            'show_marker_axes',
            default_value='true',
            description='Draw a red/green/blue XYZ axis triad at each box center '
                        'showing its detected orientation.',
        ),
        DeclareLaunchArgument(
            'marker_axis_length',
            default_value='0.15',
            description='Length in meters of each axis line in the orientation triad.',
        ),
        DeclareLaunchArgument(
            'marker_axis_width',
            default_value='0.006',
            description='Line thickness in meters of the orientation triad.',
        ),
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

            'publish_table_plane': LaunchConfiguration('publish_table_plane'),
            'table_plane_topic': LaunchConfiguration('table_plane_topic'),
            'table_plane_distance_threshold': LaunchConfiguration(
                'table_plane_distance_threshold'
            ),
            'table_plane_ransac_iterations': LaunchConfiguration(
                'table_plane_ransac_iterations'
            ),
            'table_plane_min_inliers': LaunchConfiguration('table_plane_min_inliers'),
            'table_plane_max_ransac_points': LaunchConfiguration(
                'table_plane_max_ransac_points'
            ),
            'table_plane_normal_z_min': LaunchConfiguration('table_plane_normal_z_min'),
            'table_plane_smoothing_alpha': LaunchConfiguration('table_plane_smoothing_alpha'),
            'table_plane_grid_resolution': LaunchConfiguration('table_plane_grid_resolution'),
            'color_topic': LaunchConfiguration('color_topic'),
            'below_table_margin': LaunchConfiguration('below_table_margin'),
            'freeze_table_plane': LaunchConfiguration('freeze_table_plane'),

            'enable_wall': LaunchConfiguration('enable_wall'),
            'wall_min_height': LaunchConfiguration('wall_min_height'),
            'wall_max_height': LaunchConfiguration('wall_max_height'),

            'detections_topic': LaunchConfiguration('detections_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'depth_window': LaunchConfiguration('depth_window'),
            'marker_topic': LaunchConfiguration('marker_topic'),
            'marker_color_rgba': LaunchConfiguration('marker_color_rgba'),
            'bbox_size_scale': LaunchConfiguration('bbox_size_scale'),
            'marker_lifetime': LaunchConfiguration('marker_lifetime'),
            'marker_hold_timeout': LaunchConfiguration('marker_hold_timeout'),
            'show_marker_axes': LaunchConfiguration('show_marker_axes'),
            'marker_axis_length': LaunchConfiguration('marker_axis_length'),
            'marker_axis_width': LaunchConfiguration('marker_axis_width'),
        }],
    )

    return LaunchDescription(arguments + [node])
