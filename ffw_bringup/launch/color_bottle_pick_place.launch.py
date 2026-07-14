#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('detections_topic', default_value='/yolo/detections'),
        DeclareLaunchArgument('input_image_topic', default_value='/zedm/rgbd_camera/image'),
        DeclareLaunchArgument('depth_topic', default_value='/zedm/rgbd_camera/depth_image'),
        DeclareLaunchArgument('camera_info_topic', default_value='/zedm/rgbd_camera/camera_info'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('projection_frame', default_value='zedm_left_camera_optical_frame'),
        DeclareLaunchArgument('class_names', default_value='red,blue,green'),
        DeclareLaunchArgument('min_score', default_value='0.15'),
        DeclareLaunchArgument('depth_window', default_value='7'),
        DeclareLaunchArgument('execute_motion', default_value='false'),
        DeclareLaunchArgument('movel_topic', default_value='/l_goal_move'),
        DeclareLaunchArgument('right_movel_topic', default_value='/r_goal_move'),
        DeclareLaunchArgument('movel_duration', default_value='1.5'),
        DeclareLaunchArgument('movel_subscriber_timeout', default_value='2.0'),
        DeclareLaunchArgument('settle_time', default_value='0.1'),
        DeclareLaunchArgument('safe_z', default_value='0.98'),
        DeclareLaunchArgument('grasp_z_offset', default_value='-0.05'),
        DeclareLaunchArgument('grasp_position_offset', default_value='[0.05, 0.0, 0.0]'),
        DeclareLaunchArgument('tool_orientation_rpy', default_value='[0.0, -1.5707963267948966, 0.0]'),
        DeclareLaunchArgument('color_order', default_value='red,blue,green'),
        DeclareLaunchArgument('red_place_position', default_value='[0.90, 0.35, 0.84]'),
        DeclareLaunchArgument('blue_place_position', default_value='[0.90, 0.0, 0.84]'),
        DeclareLaunchArgument('green_place_position', default_value='[0.90, -0.35, 0.84]'),
        DeclareLaunchArgument('arm_center_deadband_ratio', default_value='0.15'),
        DeclareLaunchArgument(
            'arm_joint_trajectory_topic',
            default_value='/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
        ),
        DeclareLaunchArgument(
            'right_arm_joint_trajectory_topic',
            default_value='/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
        ),
        DeclareLaunchArgument('joint_trajectory_subscriber_timeout', default_value='2.0'),
        DeclareLaunchArgument('gripper_joint', default_value='gripper_l_joint1'),
        DeclareLaunchArgument('right_gripper_joint', default_value='gripper_r_joint1'),
        DeclareLaunchArgument('gripper_open_position', default_value='0.0'),
        DeclareLaunchArgument('gripper_closed_position', default_value='0.6'),
        DeclareLaunchArgument('gripper_duration', default_value='0.6'),
        DeclareLaunchArgument('gripper_settle_time', default_value='0.1'),
        DeclareLaunchArgument(
            'left_home_pose',
            default_value='[0.11689475923776627, 0.24582697451114655, 0.9756827354431152, -1.906610336277481e-08, -0.716576099395752, -2.2137101041153073e-08, 0.6975088715553284]',
        ),
        DeclareLaunchArgument(
            'right_home_pose',
            default_value='[0.11689475923776627, -0.24582697451114655, 0.9756827354431152, -1.906610336277481e-08, -0.716576099395752, -2.2137101041153073e-08, 0.6975088715553284]',
        ),
        DeclareLaunchArgument(
            'right_home_joint_names',
            default_value='["arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4", "arm_r_joint5", "arm_r_joint6", "arm_r_joint7", "gripper_r_joint1"]',
        ),
        DeclareLaunchArgument('home_duration', default_value='3.0'),
        DeclareLaunchArgument('home_settle_time', default_value='1.0'),
        DeclareLaunchArgument('return_home_between_places', default_value='true'),
        DeclareLaunchArgument('return_home_after_place', default_value='true'),
    ]

    color_bottle_pick_place = Node(
        package='ffw_bringup',
        executable='color_bottle_pick_place',
        name='color_bottle_pick_place',
        output='screen',
        parameters=[{
            'detections_topic': LaunchConfiguration('detections_topic'),
            'input_image_topic': LaunchConfiguration('input_image_topic'),
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
            'tool_orientation_rpy': LaunchConfiguration('tool_orientation_rpy'),
            'color_order': LaunchConfiguration('color_order'),
            'red_place_position': LaunchConfiguration('red_place_position'),
            'blue_place_position': LaunchConfiguration('blue_place_position'),
            'green_place_position': LaunchConfiguration('green_place_position'),
            'arm_center_deadband_ratio': LaunchConfiguration('arm_center_deadband_ratio'),
            'arm_joint_trajectory_topic': LaunchConfiguration('arm_joint_trajectory_topic'),
            'right_arm_joint_trajectory_topic': LaunchConfiguration(
                'right_arm_joint_trajectory_topic'
            ),
            'joint_trajectory_subscriber_timeout': LaunchConfiguration(
                'joint_trajectory_subscriber_timeout'
            ),
            'gripper_joint': LaunchConfiguration('gripper_joint'),
            'right_gripper_joint': LaunchConfiguration('right_gripper_joint'),
            'gripper_open_position': LaunchConfiguration('gripper_open_position'),
            'gripper_closed_position': LaunchConfiguration('gripper_closed_position'),
            'gripper_duration': LaunchConfiguration('gripper_duration'),
            'gripper_settle_time': LaunchConfiguration('gripper_settle_time'),
            'left_home_pose': LaunchConfiguration('left_home_pose'),
            'right_home_pose': LaunchConfiguration('right_home_pose'),
            'right_home_joint_names': LaunchConfiguration('right_home_joint_names'),
            'home_duration': LaunchConfiguration('home_duration'),
            'home_settle_time': LaunchConfiguration('home_settle_time'),
            'return_home_between_places': LaunchConfiguration('return_home_between_places'),
            'return_home_after_place': LaunchConfiguration('return_home_after_place'),
        }],
    )

    return LaunchDescription(declared_arguments + [color_bottle_pick_place])
