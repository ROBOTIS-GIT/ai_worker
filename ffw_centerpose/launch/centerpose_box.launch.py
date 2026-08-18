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
    # Measured grasp/yaw calibration + place/home positions -- see
    # config/ffw_sg2_rev1_follower/centerpose_box_calibration.yaml.
    calibration_params_file = PathJoinSubstitution([
        FindPackageShare('ffw_centerpose'),
        'config',
        'ffw_sg2_rev1_follower',
        'centerpose_box_calibration.yaml',
    ])

    arguments = [
        # --- Detection / vision input -----------------------------------------
        DeclareLaunchArgument('detections_topic', default_value='/centerpose/detections'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/zed/zed_node/left/camera_info'
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/zed/zed_node/depth/depth_registered',
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
            default_value='0.7',
            description='Safety bound: ~/execute refuses to move at all if any '
                        'captured pose x exceeds this -- a target this far forward is '
                        'outside the real workspace and almost always means depth was '
                        'misread.',
        ),

        # --- Motion execution toggle ----------------------------------------------
        DeclareLaunchArgument('execute_motion', default_value='false'),
        DeclareLaunchArgument('movel_topic', default_value='/l_goal_move'),
        DeclareLaunchArgument('eef_link', default_value='end_effector_l_link'),

        # --- Pick sequence timing: approach -> insert -> gripper close -> lift ---
        DeclareLaunchArgument('movel_duration', default_value='10.0'),
        DeclareLaunchArgument(
            'pregrasp_distance',
            default_value='0.11',
            description='Back off along the gripper approach axis to this pregrasp '
                        'point before inserting straight in, so the gripper body does '
                        'not clip the box on approach.',
        ),
        DeclareLaunchArgument('pregrasp_duration', default_value='4.0'),
        DeclareLaunchArgument('insertion_duration', default_value='3.0'),
        DeclareLaunchArgument('settle_time', default_value='0.5'),
        DeclareLaunchArgument(
            'lift_height',
            default_value='0.1',
            description='Meters to lift straight up after closing the gripper.',
        ),
        DeclareLaunchArgument('lift_duration', default_value='2.0'),

        # --- Place sequence timing (after lifting) -> release -> retreat -> reclose -> push -> home ---
        DeclareLaunchArgument('place_hover_duration', default_value='4.0'),
        DeclareLaunchArgument('place_duration', default_value='6.0'),
        DeclareLaunchArgument(
            'place_release_gripper_position',
            default_value='0.9',
            description='At place_retreat, the gripper recloses to this narrower '
                        'position -- not fully open, but not gripping -- just enough '
                        'to clear the box\'s sides while pushing back in.',
        ),
        DeclareLaunchArgument('place_retreat_duration', default_value='2.0'),
        DeclareLaunchArgument('place_push_duration', default_value='2.0'),
        DeclareLaunchArgument(
            'return_to_initial',
            default_value='true',
            description='Return to the initial pose after pushing the box into '
                        'place, then open the gripper fully.',
        ),
        DeclareLaunchArgument('home_duration', default_value='6.0'),

        # --- Gripper ---------------------------------------------------------------
        DeclareLaunchArgument('gripper_duration', default_value='1.0'),
        DeclareLaunchArgument('gripper_settle_time', default_value='0.2'),
        DeclareLaunchArgument(
            'command_rate_hz',
            default_value='300.0',
            description='Rate to keep re-asserting gripper/return-to-initial targets so '
                        'they stay competitive with the MoveL/MoveJ controller streaming '
                        'to the same topic.',
        ),
    ]

    node = Node(
        package='ffw_centerpose',
        executable='centerpose_box',
        name='centerpose_box',
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
                'eef_link': LaunchConfiguration('eef_link'),

                'movel_duration': LaunchConfiguration('movel_duration'),
                'pregrasp_distance': LaunchConfiguration('pregrasp_distance'),
                'pregrasp_duration': LaunchConfiguration('pregrasp_duration'),
                'insertion_duration': LaunchConfiguration('insertion_duration'),
                'settle_time': LaunchConfiguration('settle_time'),
                'lift_height': LaunchConfiguration('lift_height'),
                'lift_duration': LaunchConfiguration('lift_duration'),

                'place_hover_duration': LaunchConfiguration('place_hover_duration'),
                'place_duration': LaunchConfiguration('place_duration'),
                'place_release_gripper_position': LaunchConfiguration(
                    'place_release_gripper_position'
                ),
                'place_retreat_duration': LaunchConfiguration('place_retreat_duration'),
                'place_push_duration': LaunchConfiguration('place_push_duration'),
                'return_to_initial': LaunchConfiguration('return_to_initial'),
                'home_duration': LaunchConfiguration('home_duration'),

                'gripper_duration': LaunchConfiguration('gripper_duration'),
                'gripper_settle_time': LaunchConfiguration('gripper_settle_time'),
                'command_rate_hz': LaunchConfiguration('command_rate_hz'),
            },
        ],
    )

    return LaunchDescription(arguments + [node])
