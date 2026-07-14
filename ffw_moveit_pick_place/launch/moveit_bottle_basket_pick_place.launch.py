#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('detections_topic', default_value='/yolo/detections'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera_head/camera_head/aligned_depth_to_color/image_raw',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera_head/camera_head/aligned_depth_to_color/camera_info',
        ),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('projection_frame', default_value=''),
        DeclareLaunchArgument('bottle_class_name', default_value='bottle'),
        DeclareLaunchArgument('basket_class_name', default_value='basket'),
        DeclareLaunchArgument('min_score', default_value='0.15'),
        DeclareLaunchArgument('depth_window', default_value='7'),
        DeclareLaunchArgument('execute_motion', default_value='false'),
        DeclareLaunchArgument('arm_center_deadband_ratio', default_value='0.15'),
        DeclareLaunchArgument('safe_z', default_value='0.98'),
        DeclareLaunchArgument('above_z_offset', default_value='0.04'),
        DeclareLaunchArgument('grasp_z_offset', default_value='-0.15'),
        DeclareLaunchArgument('grasp_position_offset', default_value='[0.05, 0.0, 0.0]'),
        DeclareLaunchArgument('basket_place_z_offset', default_value='-0.05'),
        DeclareLaunchArgument('tool_orientation_rpy', default_value='[0.0, -1.5707963267948966, 0.0]'),
        DeclareLaunchArgument('use_cartesian_path', default_value='true'),
        DeclareLaunchArgument('cartesian_eef_step', default_value='0.01'),
        DeclareLaunchArgument('cartesian_min_fraction', default_value='0.90'),
        DeclareLaunchArgument('max_velocity_scaling', default_value='0.35'),
        DeclareLaunchArgument('max_acceleration_scaling', default_value='0.35'),
        DeclareLaunchArgument(
            'arm_joint_trajectory_topic',
            default_value='/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
        ),
        DeclareLaunchArgument(
            'right_arm_joint_trajectory_topic',
            default_value='/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
        ),
        DeclareLaunchArgument('gripper_joint', default_value='gripper_l_joint1'),
        DeclareLaunchArgument('right_gripper_joint', default_value='gripper_r_joint1'),
        DeclareLaunchArgument('gripper_open_position', default_value='0.25'),
        DeclareLaunchArgument('gripper_closed_position', default_value='0.55'),
        DeclareLaunchArgument('gripper_duration', default_value='1.0'),
        DeclareLaunchArgument('gripper_settle_time', default_value='0.1'),
        DeclareLaunchArgument(
            'home_joint_positions',
            default_value='[1.1383335868601994, 0.16853415484402948, 0.08995359213960642, -1.6951566286377817, 0.27226960562479596, -0.9465500356997781, -0.11196998211609252, 0.0]',
        ),
        DeclareLaunchArgument(
            'right_home_joint_positions',
            default_value='[1.1433190244208276, -0.13817811315876127, -0.04756538864936023, -1.7302943760602871, -0.11737349872306227, -0.925337957617297, 0.19234304058867288, 0.0015339807878856412]',
        ),
        DeclareLaunchArgument('return_home_after_place', default_value='true'),
        DeclareLaunchArgument('start_moveit', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('use_sim', default_value='false'),
    ]

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ffw_moveit_config'),
            '/launch/moveit.launch.py',
        ]),
        condition=IfCondition(LaunchConfiguration('start_moveit')),
        launch_arguments={
            'start_rviz': LaunchConfiguration('start_rviz'),
            'use_sim': LaunchConfiguration('use_sim'),
        }.items(),
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name='ffw', package_name='ffw_moveit_config')
        .robot_description_semantic(Path('config') / 'ffw.srdf')
        .to_moveit_configs()
    )

    node = Node(
        package='ffw_moveit_pick_place',
        executable='moveit_bottle_basket_pick_place',
        name='moveit_bottle_basket_pick_place',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
            'detections_topic': LaunchConfiguration('detections_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'target_frame': LaunchConfiguration('target_frame'),
            'projection_frame': LaunchConfiguration('projection_frame'),
            'bottle_class_name': LaunchConfiguration('bottle_class_name'),
            'basket_class_name': LaunchConfiguration('basket_class_name'),
            'min_score': LaunchConfiguration('min_score'),
            'depth_window': LaunchConfiguration('depth_window'),
            'execute_motion': LaunchConfiguration('execute_motion'),
            'arm_center_deadband_ratio': LaunchConfiguration('arm_center_deadband_ratio'),
            'safe_z': LaunchConfiguration('safe_z'),
            'above_z_offset': LaunchConfiguration('above_z_offset'),
            'grasp_z_offset': LaunchConfiguration('grasp_z_offset'),
            'grasp_position_offset': LaunchConfiguration('grasp_position_offset'),
            'basket_place_z_offset': LaunchConfiguration('basket_place_z_offset'),
            'tool_orientation_rpy': LaunchConfiguration('tool_orientation_rpy'),
            'use_cartesian_path': LaunchConfiguration('use_cartesian_path'),
            'cartesian_eef_step': LaunchConfiguration('cartesian_eef_step'),
            'cartesian_min_fraction': LaunchConfiguration('cartesian_min_fraction'),
            'max_velocity_scaling': LaunchConfiguration('max_velocity_scaling'),
            'max_acceleration_scaling': LaunchConfiguration('max_acceleration_scaling'),
            'arm_joint_trajectory_topic': LaunchConfiguration('arm_joint_trajectory_topic'),
            'right_arm_joint_trajectory_topic': LaunchConfiguration('right_arm_joint_trajectory_topic'),
            'gripper_joint': LaunchConfiguration('gripper_joint'),
            'right_gripper_joint': LaunchConfiguration('right_gripper_joint'),
            'gripper_open_position': LaunchConfiguration('gripper_open_position'),
            'gripper_closed_position': LaunchConfiguration('gripper_closed_position'),
            'gripper_duration': LaunchConfiguration('gripper_duration'),
            'gripper_settle_time': LaunchConfiguration('gripper_settle_time'),
            'home_joint_positions': LaunchConfiguration('home_joint_positions'),
            'right_home_joint_positions': LaunchConfiguration('right_home_joint_positions'),
            'return_home_after_place': LaunchConfiguration('return_home_after_place'),
            'use_sim_time': LaunchConfiguration('use_sim'),
            },
        ],
    )

    return LaunchDescription(declared_arguments + [moveit_launch, node])
