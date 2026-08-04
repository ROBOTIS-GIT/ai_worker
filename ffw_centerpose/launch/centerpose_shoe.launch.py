#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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

        # --- Startup + start pose (arm rotates a lot, so move slowly) ------------
        # Run ONCE at node startup (execute_motion=true), not as part of each pick:
        # startup_pose (slow) -> start_pose (very slow). ~/execute assumes the arm
        # is already at start_pose and skips straight to opening the gripper.
        DeclareLaunchArgument(
            'startup_position_xyz',
            default_value='[0.17088773846626282, 0.48100942373275757, 0.9724668264389038]',
            description='First waypoint on node startup, moved to slowly -- measured '
                        'via `ros2 topic echo --once /l_goal_pose`.',
        ),
        DeclareLaunchArgument(
            'startup_orientation_xyzw',
            default_value='[-0.06572848558425903, -0.6882247924804688, '
                           '-0.06251275539398193, 0.7198045253753662]',
        ),
        DeclareLaunchArgument(
            'startup_duration',
            default_value='10.0',
            description='Same as start_duration -- very slow, like the start pose '
                        'move.',
        ),
        DeclareLaunchArgument(
            'start_position_xyz',
            default_value='[0.23394590616226196, 0.3340926766395569, 0.9377147555351257]',
            description='Pose the arm settles at after startup, and returns to after '
                        'placing -- measured via `ros2 topic echo --once /l_goal_pose`.',
        ),
        DeclareLaunchArgument(
            'start_orientation_xyzw',
            default_value='[-0.002594258636236191, 0.008345617912709713, '
                           '-0.00505678029730916, 0.9999490976333618]',
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
        DeclareLaunchArgument(
            'insertion_overshoot_distance',
            default_value='0.1438221335411271',
            description='How far to lower from the hover height (fixed_grasp_z) to '
                        'the actual grab height, along the approach axis (roughly '
                        'straight down). Measured: fixed_grasp_z - min_grasp_z.',
        ),
        DeclareLaunchArgument('movel_subscriber_timeout', default_value='2.0'),
        DeclareLaunchArgument('settle_time', default_value='0.5'),
        DeclareLaunchArgument(
            'fixed_grasp_z',
            default_value='0.9377147555351257',
            description='Hover height before lowering to grab -- always fixed, same '
                        'height as start_position_xyz\'s z.',
        ),
        DeclareLaunchArgument(
            'min_grasp_z',
            default_value='0.7938926219940186',
            description='Measured hard floor -- the arm must NEVER descend below '
                        'this when grabbing a shoe. insert_pose.z is clamped to this '
                        'in code, not just relied on via arithmetic.',
        ),

        # --- Shoe hole offset from detected center ----------------------------------
        DeclareLaunchArgument(
            'grasp_position_offset',
            default_value='[-0.03, 0.0, 0.0]',
            description='x pulled 3cm toward the robot (-X, 1cm then +2cm more).',
        ),
        DeclareLaunchArgument(
            'grasp_position_y_slope',
            default_value='-6.944444444444444e-05',
            description='centerpose_box와 같은 방식의 픽셀 위치 기반 y 보정 기울기.',
        ),
        DeclareLaunchArgument('grasp_position_y_reference_pixel', default_value='288.0'),
        DeclareLaunchArgument(
            'second_shoe_grasp_y_offset',
            default_value='-0.02',
            description='Extra right (-Y) pull applied only to the second-captured '
                        'shoe (queue index 1), separate from the position-based '
                        'correction above.',
        ),
        DeclareLaunchArgument(
            'shoe_depth_center_offset',
            default_value='0.0',
            description='TODO: depth correction from the shoe\'s visible surface to '
                        'its actual grab point. Placeholder until measured.',
        ),
        DeclareLaunchArgument(
            'tool_orientation_offset_xyzw', default_value='[0.0, 0.0, 0.0, 1.0]'
        ),

        # --- Shoe yaw -> gripper yaw calibration ------------------------------------
        DeclareLaunchArgument(
            'shoe_yaw_reference_orientation_xyzw',
            default_value='[0.04132861537025957, -0.3549227920668959, '
                           '0.9339735266150585, -0.003899846823498184]',
            description='CenterPose bbox.center.orientation captured with the shoe '
                        'facing the robot (shoe_yaw=0 reference).',
        ),
        DeclareLaunchArgument(
            'shoe_yaw_axis_xyz',
            default_value='[0.13999845344725156, -0.9872268603133045, '
                           '0.07604971603046785]',
            description='Calibrated rotation axis (camera frame) the shoe actually '
                        'turns about, derived from the reference vs. a clockwise-'
                        'turned sample.',
        ),
        DeclareLaunchArgument('shoe_yaw_flip_threshold_deg', default_value='90.0'),
        DeclareLaunchArgument(
            'grasp_fixed_roll',
            default_value='-0.005273412980347409',
            description='Fixed gripper roll (rad); only yaw follows the shoe yaw '
                        '(unlike centerpose_box.py, which grasps from the side and rolls '
                        'instead -- shoes are grasped from directly above).',
        ),
        DeclareLaunchArgument('grasp_fixed_pitch', default_value='0.01666491788848952'),
        DeclareLaunchArgument(
            'grasp_yaw_from_shoe_yaw_scale',
            default_value='-1.016908340135941',
            description='Linear fit slope: gripper yaw (rad) = scale * shoe_yaw (rad) '
                        '+ offset. Least-squares fit from 3 measured points (shoe_yaw '
                        '0/+35.75/-69.38 deg), residuals ~2-6 deg; validated against a '
                        '4th independent sample with ~4.4 deg error.',
        ),
        DeclareLaunchArgument('grasp_yaw_offset', default_value='-0.12016636998914268'),
        DeclareLaunchArgument('yaw_clamp_min_deg', default_value='-90.0'),
        DeclareLaunchArgument('yaw_clamp_max_deg', default_value='90.0'),

        # --- Place sequence (2 slots, same pattern as centerpose_bottle.py) ----------------
        DeclareLaunchArgument(
            'place_slot_1_position_xyz',
            default_value='[0.47151222825050354, 0.42120254039764404, 0.9456136226654053]',
            description='Hover pose above the first drop slot (leftmost-captured '
                        'shoe goes here). Measured via '
                        '`ros2 topic echo --once /l_goal_pose`.',
        ),
        DeclareLaunchArgument(
            'place_slot_1_orientation_xyzw',
            default_value='[-0.0023092320188879967, 0.008429071865975857, '
                           '0.028931625187397003, 0.9995430707931519]',
        ),
        DeclareLaunchArgument(
            'place_slot_2_position_xyz',
            default_value='[0.45977485179901123, 0.22981253385543823, 0.9466336965560913]',
            description='y shifted from slot 1 (right) so the two placed shoes do '
                        'not touch.',
        ),
        DeclareLaunchArgument(
            'place_slot_2_orientation_xyzw',
            default_value='[-0.0023092320188879967, 0.008429071865975857, '
                           '0.028931625187397003, 0.9995430707931519]',
        ),
        DeclareLaunchArgument('place_hover_duration', default_value='4.0'),
        DeclareLaunchArgument(
            'place_lower_distance',
            default_value='0.1338221335411271',
            description='How far to lower from a place slot\'s hover pose to '
                        'actually release the shoe -- insertion_overshoot_distance '
                        'minus 1cm (release 1cm higher).',
        ),
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
        parameters=[{
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

            'startup_position_xyz': LaunchConfiguration('startup_position_xyz'),
            'startup_orientation_xyzw': LaunchConfiguration('startup_orientation_xyzw'),
            'startup_duration': LaunchConfiguration('startup_duration'),
            'start_position_xyz': LaunchConfiguration('start_position_xyz'),
            'start_orientation_xyzw': LaunchConfiguration('start_orientation_xyzw'),
            'start_duration': LaunchConfiguration('start_duration'),
            'return_duration': LaunchConfiguration('return_duration'),

            'pregrasp_distance': LaunchConfiguration('pregrasp_distance'),
            'pregrasp_duration': LaunchConfiguration('pregrasp_duration'),
            'insertion_duration': LaunchConfiguration('insertion_duration'),
            'insertion_overshoot_distance': LaunchConfiguration(
                'insertion_overshoot_distance'
            ),
            'movel_subscriber_timeout': LaunchConfiguration('movel_subscriber_timeout'),
            'settle_time': LaunchConfiguration('settle_time'),
            'fixed_grasp_z': LaunchConfiguration('fixed_grasp_z'),
            'min_grasp_z': LaunchConfiguration('min_grasp_z'),

            'grasp_position_offset': LaunchConfiguration('grasp_position_offset'),
            'grasp_position_y_slope': LaunchConfiguration('grasp_position_y_slope'),
            'grasp_position_y_reference_pixel': LaunchConfiguration(
                'grasp_position_y_reference_pixel'
            ),
            'second_shoe_grasp_y_offset': LaunchConfiguration('second_shoe_grasp_y_offset'),
            'shoe_depth_center_offset': LaunchConfiguration('shoe_depth_center_offset'),
            'tool_orientation_offset_xyzw': LaunchConfiguration(
                'tool_orientation_offset_xyzw'
            ),

            'shoe_yaw_reference_orientation_xyzw': LaunchConfiguration(
                'shoe_yaw_reference_orientation_xyzw'
            ),
            'shoe_yaw_axis_xyz': LaunchConfiguration('shoe_yaw_axis_xyz'),
            'shoe_yaw_flip_threshold_deg': LaunchConfiguration(
                'shoe_yaw_flip_threshold_deg'
            ),
            'grasp_fixed_roll': LaunchConfiguration('grasp_fixed_roll'),
            'grasp_fixed_pitch': LaunchConfiguration('grasp_fixed_pitch'),
            'grasp_yaw_from_shoe_yaw_scale': LaunchConfiguration(
                'grasp_yaw_from_shoe_yaw_scale'
            ),
            'grasp_yaw_offset': LaunchConfiguration('grasp_yaw_offset'),
            'yaw_clamp_min_deg': LaunchConfiguration('yaw_clamp_min_deg'),
            'yaw_clamp_max_deg': LaunchConfiguration('yaw_clamp_max_deg'),

            'place_slot_1_position_xyz': LaunchConfiguration('place_slot_1_position_xyz'),
            'place_slot_1_orientation_xyzw': LaunchConfiguration(
                'place_slot_1_orientation_xyzw'
            ),
            'place_slot_2_position_xyz': LaunchConfiguration('place_slot_2_position_xyz'),
            'place_slot_2_orientation_xyzw': LaunchConfiguration(
                'place_slot_2_orientation_xyzw'
            ),
            'place_hover_duration': LaunchConfiguration('place_hover_duration'),
            'place_lower_distance': LaunchConfiguration('place_lower_distance'),
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
        }],
    )

    return LaunchDescription(arguments + [node])
