#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('detections_topic', default_value='/yolo/detections'),
        DeclareLaunchArgument('depth_topic', default_value='/zedm/rgbd_camera/depth_image'),
        DeclareLaunchArgument('camera_info_topic', default_value='/zedm/rgbd_camera/camera_info'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('projection_frame', default_value='zedm_left_camera_optical_frame'),
        DeclareLaunchArgument('class_names', default_value='cup,bottle'),
        DeclareLaunchArgument('min_score', default_value='0.25'),
        DeclareLaunchArgument('depth_window', default_value='7'),
        DeclareLaunchArgument('execute_motion', default_value='false'),
        DeclareLaunchArgument('movel_topic', default_value='/l_goal_move'),
        DeclareLaunchArgument('right_movel_topic', default_value='/r_goal_move'),
        DeclareLaunchArgument('movel_duration', default_value='4.0'),
        DeclareLaunchArgument('movel_subscriber_timeout', default_value='2.0'),
        DeclareLaunchArgument('settle_time', default_value='0.5'),
        DeclareLaunchArgument('safe_z', default_value='0.98'),
        DeclareLaunchArgument('grasp_z_offset', default_value='-0.05'),
        DeclareLaunchArgument('grasp_position_offset', default_value='[0.05, 0.0, 0.0]'),
        DeclareLaunchArgument('place_position', default_value='[0.55, 0.50, 0.84]'),
        DeclareLaunchArgument('tool_orientation_rpy', default_value='[0.0, -1.5707963267948966, 0.0]'),
        DeclareLaunchArgument(
            'arm_joint_trajectory_topic',
            default_value='/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
        ),
        DeclareLaunchArgument('joint_trajectory_subscriber_timeout', default_value='2.0'),
        DeclareLaunchArgument('gripper_joint', default_value='gripper_l_joint1'),
        DeclareLaunchArgument('gripper_open_position', default_value='0.0'),
        DeclareLaunchArgument('gripper_closed_position', default_value='1.0'),
        DeclareLaunchArgument('gripper_duration', default_value='1.5'),
        DeclareLaunchArgument('gripper_settle_time', default_value='0.5'),
        DeclareLaunchArgument(
            'left_home_pose',
            default_value='[0.11689475923776627, 0.24582697451114655, 0.9756827354431152, -1.906610336277481e-08, -0.716576099395752, -2.2137101041153073e-08, 0.6975088715553284]',
        ),
        DeclareLaunchArgument(
            'right_home_pose',
            default_value='[0.11689475923776627, -0.24582697451114655, 0.9756827354431152, -1.906610336277481e-08, -0.716576099395752, -2.2137101041153073e-08, 0.6975088715553284]',
        ),
        DeclareLaunchArgument('home_duration', default_value='3.0'),
        DeclareLaunchArgument('home_settle_time', default_value='1.0'),
        DeclareLaunchArgument('return_home_after_place', default_value='true'),
    ]

    vision_pick_place = Node(
        package='ffw_bringup',
        executable='vision_pick_place',
        name='vision_pick_place',
        output='screen',
        parameters=[{
            'detections_topic': LaunchConfiguration('detections_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'target_frame': LaunchConfiguration('target_frame'),
            'projection_frame': LaunchConfiguration('projection_frame'),
            'class_names': LaunchConfiguration('class_names'),
            'min_score': LaunchConfiguration('min_score'),
            'depth_window': LaunchConfiguration('depth_window'),
            'execute_motion': LaunchConfiguration('execute_motion'),
            'movel_topic': LaunchConfiguration('movel_topic'),
            'right_movel_topic': LaunchConfiguration('right_movel_topic'),
            'movel_duration': LaunchConfiguration('movel_duration'),
            'movel_subscriber_timeout': LaunchConfiguration('movel_subscriber_timeout'),
            'settle_time': LaunchConfiguration('settle_time'),
            'safe_z': LaunchConfiguration('safe_z'),
            'grasp_z_offset': LaunchConfiguration('grasp_z_offset'),
            'grasp_position_offset': LaunchConfiguration('grasp_position_offset'),
            'place_position': LaunchConfiguration('place_position'),
            'tool_orientation_rpy': LaunchConfiguration('tool_orientation_rpy'),
            'arm_joint_trajectory_topic': LaunchConfiguration('arm_joint_trajectory_topic'),
            'joint_trajectory_subscriber_timeout': LaunchConfiguration(
                'joint_trajectory_subscriber_timeout'
            ),
            'gripper_joint': LaunchConfiguration('gripper_joint'),
            'gripper_open_position': LaunchConfiguration('gripper_open_position'),
            'gripper_closed_position': LaunchConfiguration('gripper_closed_position'),
            'gripper_duration': LaunchConfiguration('gripper_duration'),
            'gripper_settle_time': LaunchConfiguration('gripper_settle_time'),
            'left_home_pose': LaunchConfiguration('left_home_pose'),
            'right_home_pose': LaunchConfiguration('right_home_pose'),
            'home_duration': LaunchConfiguration('home_duration'),
            'home_settle_time': LaunchConfiguration('home_settle_time'),
            'return_home_after_place': LaunchConfiguration('return_home_after_place'),
        }],
    )

    return LaunchDescription(declared_arguments + [vision_pick_place])
