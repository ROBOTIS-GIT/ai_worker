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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Measured startup/grasp/yaw calibration + place slot positions -- see
    # config/ffw_sg2_rev1_follower/centerpose_shoe_calibration.yaml.
    calibration_params_file = PathJoinSubstitution([
        FindPackageShare('ffw_centerpose'),
        'config',
        'ffw_sg2_rev1_follower',
        'centerpose_shoe_calibration.yaml',
    ])

    arguments = [
        # --- Detection / vision input -----------------------------------------
        DeclareLaunchArgument('detections_topic', default_value='/centerpose/detections'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/zedm/zed_node/left/camera_info'
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/zedm/zed_node/depth/depth_registered',
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
                        'captured pose x exceeds this. Wider than centerpose_box/centerpose_bottle '
                        '(0.7) since shoes are worked on further out (measured x=0.87).',
        ),

        # --- Motion execution toggle ----------------------------------------------
        DeclareLaunchArgument('execute_motion', default_value='false'),
        DeclareLaunchArgument('movel_topic', default_value='/l_goal_move'),
        DeclareLaunchArgument('movel_duration', default_value='10.0'),
        DeclareLaunchArgument('eef_link', default_value='end_effector_l_link'),

        # --- Startup + start pose timing ---
        DeclareLaunchArgument(
            'startup_duration',
            default_value='10.0',
            description='Same as start_duration -- very slow, like the start pose '
                        'move.',
        ),
        DeclareLaunchArgument(
            'start_duration',
            default_value='10.0',
            description='This move rotates the arm almost all the way around, so it '
                        'defaults very slow -- used at startup (startup_pose -> '
                        'start_pose).',
        ),
        DeclareLaunchArgument(
            'return_duration',
            default_value='6.0',
            description='Used only for the final return to start pose after the '
                        'whole queue is placed -- roughly 1.5x faster than '
                        'start_duration (10.0).',
        ),

        # --- Approach / insertion --------------------------------------------------
        DeclareLaunchArgument('pregrasp_distance', default_value='0.1'),
        DeclareLaunchArgument('pregrasp_duration', default_value='4.0'),
        DeclareLaunchArgument('insertion_duration', default_value='2.0'),
        DeclareLaunchArgument('movel_subscriber_timeout', default_value='2.0'),
        DeclareLaunchArgument('settle_time', default_value='0.5'),

        # --- Shoe yaw -> gripper yaw clamp (measured reference/scale values are
        # in centerpose_shoe_calibration.yaml) ---
        DeclareLaunchArgument('yaw_clamp_min_deg', default_value='-90.0'),
        DeclareLaunchArgument('yaw_clamp_max_deg', default_value='90.0'),

        # --- Place sequence timing ---
        DeclareLaunchArgument('place_hover_duration', default_value='4.0'),
        DeclareLaunchArgument('place_lower_duration', default_value='2.0'),
        DeclareLaunchArgument('place_duration', default_value='1.0'),

        # --- Arm / gripper -----------------------------------------------------
        DeclareLaunchArgument(
            'left_arm_joint_trajectory_topic',
            default_value='/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
        ),
        DeclareLaunchArgument('left_gripper_joint', default_value='gripper_l_joint1'),
        DeclareLaunchArgument(
            'left_arm_joint_names',
            default_value='[arm_l_joint1, arm_l_joint2, arm_l_joint3, arm_l_joint4, '
                           'arm_l_joint5, arm_l_joint6, arm_l_joint7, gripper_l_joint1]',
        ),
        DeclareLaunchArgument('gripper_open_position', default_value='0.0'),
        DeclareLaunchArgument(
            'gripper_closed_position',
            default_value='1.1',
            description='Higher than centerpose_bottle.py\'s 0.7 -- shoes need a firmer '
                        'grip.',
        ),
        DeclareLaunchArgument(
            'gripper_release_position',
            default_value='0.7',
            description='Only used when releasing a shoe into a place slot -- '
                        'opening all the way (gripper_open_position) knocks into the '
                        'neighboring already-placed shoe.',
        ),
        DeclareLaunchArgument('gripper_duration', default_value='1.0'),
        DeclareLaunchArgument('gripper_settle_time', default_value='0.2'),
        DeclareLaunchArgument('command_rate_hz', default_value='300.0'),
        DeclareLaunchArgument('lift_height', default_value='0.1'),
        DeclareLaunchArgument('lift_duration', default_value='2.0'),
    ]

    node = Node(
        package='ffw_centerpose',
        executable='centerpose_shoe',
        name='centerpose_shoe',
        output='screen',
        parameters=[
            calibration_params_file,
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
                'movel_duration': LaunchConfiguration('movel_duration'),
                'eef_link': LaunchConfiguration('eef_link'),

                'startup_duration': LaunchConfiguration('startup_duration'),
                'start_duration': LaunchConfiguration('start_duration'),
                'return_duration': LaunchConfiguration('return_duration'),

                'pregrasp_distance': LaunchConfiguration('pregrasp_distance'),
                'pregrasp_duration': LaunchConfiguration('pregrasp_duration'),
                'insertion_duration': LaunchConfiguration('insertion_duration'),
                'movel_subscriber_timeout': LaunchConfiguration('movel_subscriber_timeout'),
                'settle_time': LaunchConfiguration('settle_time'),

                'yaw_clamp_min_deg': LaunchConfiguration('yaw_clamp_min_deg'),
                'yaw_clamp_max_deg': LaunchConfiguration('yaw_clamp_max_deg'),

                'place_hover_duration': LaunchConfiguration('place_hover_duration'),
                'place_lower_duration': LaunchConfiguration('place_lower_duration'),
                'place_duration': LaunchConfiguration('place_duration'),

                'left_arm_joint_trajectory_topic': LaunchConfiguration(
                    'left_arm_joint_trajectory_topic'
                ),
                'left_gripper_joint': LaunchConfiguration('left_gripper_joint'),
                'left_arm_joint_names': LaunchConfiguration('left_arm_joint_names'),
                'gripper_open_position': LaunchConfiguration('gripper_open_position'),
                'gripper_closed_position': LaunchConfiguration('gripper_closed_position'),
                'gripper_release_position': LaunchConfiguration('gripper_release_position'),
                'gripper_duration': LaunchConfiguration('gripper_duration'),
                'gripper_settle_time': LaunchConfiguration('gripper_settle_time'),
                'command_rate_hz': LaunchConfiguration('command_rate_hz'),
                'lift_height': LaunchConfiguration('lift_height'),
                'lift_duration': LaunchConfiguration('lift_duration'),
            },
        ],
    )

    return LaunchDescription(arguments + [node])
