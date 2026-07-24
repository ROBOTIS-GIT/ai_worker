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
            default_value='0.7',
            description='Safety bound: ~/execute refuses to move at all if any '
                        'captured pose x exceeds this -- a target this far forward is '
                        'outside the real workspace and almost always means depth was '
                        'misread (e.g. a background hit instead of the bottle).',
        ),

        # --- Arm selection (which arm picks which detection) ---------------------
        DeclareLaunchArgument(
            'arm',
            default_value='left',
            description='Tie-break arm used only when the bottle pixel falls inside '
                        'the center deadband; the arm is otherwise chosen per '
                        'detection from which side of the image it falls on.',
        ),
        DeclareLaunchArgument(
            'arm_center_deadband_ratio',
            default_value='0.1',
            description='Fraction of image width around the center where the side '
                        'is ambiguous; detections inside this band use "arm".',
        ),
        DeclareLaunchArgument('eef_link', default_value='end_effector_l_link'),
        DeclareLaunchArgument('right_eef_link', default_value='end_effector_r_link'),

        # --- Motion execution toggle ----------------------------------------------
        DeclareLaunchArgument('execute_motion', default_value='false'),

        # --- Per-arm grasp position/orientation calibration -----------------------
        DeclareLaunchArgument(
            'fixed_grasp_z',
            default_value='0.8241714239120483',
            description='Left-arm grasp height (base_link Z). Pinned to the EEF height '
                        'measured in the old (lower-lift) bottle_ready pose, since the '
                        'table this grasps from has not moved -- only the robot body '
                        'lift did (see home_position_xyz). Negative: fall back to '
                        'dynamically holding current EEF Z at capture time instead.',
        ),
        DeclareLaunchArgument(
            'right_fixed_grasp_z',
            default_value='0.812108039855957',
            description='Right-arm equivalent of fixed_grasp_z, same reasoning.',
        ),
        DeclareLaunchArgument(
            'grasp_position_offset',
            default_value='[0.01, -0.04, 0.0]',
            description='Calibrated from measured error: ~2cm too far forward (+X).',
        ),
        DeclareLaunchArgument(
            'right_grasp_position_offset',
            default_value='[0.01, -0.02, 0.0]',
            description='UNCALIBRATED placeholder copied from the left arm; re-measure '
                        'for the right gripper before trusting it.',
        ),
        DeclareLaunchArgument(
            'tool_orientation_offset_xyzw',
            default_value='[0.0, 0.0, 0.0, 1.0]',
            description='CenterPose object frame to gripper tool-frame rotation.',
        ),
        DeclareLaunchArgument(
            'right_tool_orientation_offset_xyzw',
            default_value='[0.0, 0.0, 0.0, 1.0]',
        ),
        DeclareLaunchArgument(
            'grasp_fixed_pitch',
            default_value='-1.5256446590458033',
            description='Fixed gripper pitch (rad); only roll follows the object yaw.',
        ),
        DeclareLaunchArgument(
            'right_grasp_fixed_pitch',
            default_value='-1.5176096004888053',
            description='Fit from 3 measured /r_goal_pose targets (see '
                        'right_grasp_roll_from_yaw_scale); pitch held ~constant '
                        '(-86.95 deg) across all 3, same as the left rig.',
        ),
        DeclareLaunchArgument(
            'grasp_fixed_yaw',
            default_value='0.010559241974565694',
            description='Fixed gripper yaw (rad); only roll follows the object yaw.',
        ),
        DeclareLaunchArgument(
            'right_grasp_fixed_yaw',
            default_value='0.17740065731475654',
            description='Fit from the same 3 right-arm samples; yaw held ~constant '
                        '(10.16 deg) across all 3.',
        ),
        DeclareLaunchArgument('grasp_roll_from_yaw_scale', default_value='0.6622504892367908'),
        DeclareLaunchArgument(
            'right_grasp_roll_from_yaw_scale',
            default_value='0.5103027930797728',
            description='Linear fit (roll vs. object yaw, radians) from 3 measured '
                        '/r_goal_pose targets: object yaw -66.8/-80.7/-159.2 deg -> '
                        'roll 1.0/-11.3/-48.1 deg.',
        ),
        DeclareLaunchArgument('grasp_roll_offset', default_value='0.9278879706509177'),
        DeclareLaunchArgument(
            'right_grasp_roll_offset',
            default_value='0.5713675639841987',
            description='Intercept of the right-arm roll-vs-yaw fit above (radians).',
        ),
        DeclareLaunchArgument(
            'bottle_yaw_zero_offset_deg',
            default_value='-80.9',
            description='Calibrated so a bottle facing the robot front reads ~0 deg. '
                        'Re-measure and update if the camera mount or model changes. '
                        'Shared by both arms -- describes the camera/object relationship.',
        ),
        DeclareLaunchArgument(
            'roll_clamp_min_deg',
            default_value='-45.0',
            description='Gripper roll (deg) is clamped to [roll_clamp_min_deg, '
                        'roll_clamp_max_deg] -- the window the linear yaw fit was '
                        'validated in. Past either bound the gripper holds at that '
                        'bound instead of following the raw fit far outside where '
                        'it was measured.',
        ),
        DeclareLaunchArgument('roll_clamp_max_deg', default_value='45.0'),
        DeclareLaunchArgument(
            'right_roll_clamp_min_deg',
            default_value='-65.0',
            description='Right rig measured roll window is -48.1 to 1.0 deg; this '
                        'clamp window is centered around that range.',
        ),
        DeclareLaunchArgument('right_roll_clamp_max_deg', default_value='25.0'),

        # --- Pick sequence timing: approach -> insert -> gripper close -> lift ---
        DeclareLaunchArgument('movel_duration', default_value='5.0'),
        DeclareLaunchArgument(
            'pregrasp_distance',
            default_value='0.08',
            description='Back off along the gripper approach axis to this pregrasp '
                        'point before inserting straight in, so the gripper body does '
                        'not clip the bottle on approach.',
        ),
        DeclareLaunchArgument('pregrasp_duration', default_value='5.0'),
        DeclareLaunchArgument('insertion_duration', default_value='1.0'),
        DeclareLaunchArgument('settle_time', default_value='0.3'),
        DeclareLaunchArgument(
            'lift_height',
            default_value='0.1',
            description='Meters to lift straight up after closing the gripper.',
        ),
        DeclareLaunchArgument('lift_duration', default_value='1.0'),

        # --- Retract + return-to-initial-pose sequence (after lifting) -----------
        DeclareLaunchArgument(
            'return_to_initial',
            default_value='true',
            description='Lift then retract toward the robot body after grasping.',
        ),
        DeclareLaunchArgument(
            'retract_distance',
            default_value='0.2',
            description='Meters to move further left (base_link +Y, away from the body '
                        'midline) after lifting, before heading to the initial pose -- '
                        'avoids crossing the body centerline, which was flipping the '
                        'elbow to a different IK configuration.',
        ),
        DeclareLaunchArgument(
            'right_retract_distance',
            default_value='0.17',
            description='Same as retract_distance but for the right arm (moves -Y); '
                        'was retracting too far out from the body, brought 3cm closer in.',
        ),
        DeclareLaunchArgument(
            'retract_position_xyz',
            default_value='[0.0]',
            description='Sentinel (not a valid [x, y, z]): derive the retract point '
                        'from the grasp pose as usual (retract_distance/retract_sign). '
                        'A real [x, y, z]: use this fixed base_link point instead.',
        ),
        DeclareLaunchArgument(
            'right_retract_position_xyz',
            default_value='[0.35064399242401123, -0.21548181772232056, 1.0577376640319824]',
            description='Measured via /r_goal_pose mid-retract on the real right arm '
                        '(original bottle_ready lift height), then Z shifted by the '
                        '+0.15005m lift raise (see home_position_xyz) since lift_joint '
                        'is a pure vertical prismatic joint at the arm base -- x/y and '
                        'orientation are unaffected by lift height. Re-measure via '
                        '/r_goal_pose mid-retract if this drifts.',
        ),
        DeclareLaunchArgument('retract_duration', default_value='2.0'),
        DeclareLaunchArgument(
            'home_position_xyz',
            default_value='[0.13451801240444183, 0.2999741733074188, 0.9742214239120483]',
            description='Originally measured via tf2_echo base_link end_effector_l_link '
                        'with the left arm in its bottle_ready initial pose; Z shifted by '
                        '+0.15005m for the lift raise from -0.31497 to -0.16492 (lift_joint '
                        'is a pure vertical prismatic joint at the arm base, so x/y and '
                        'orientation are unaffected). Re-measure with tf2_echo if this '
                        'drifts.',
        ),
        DeclareLaunchArgument(
            'right_home_position_xyz',
            default_value='[0.14263390004634857, -0.24826078116893768, 0.962158039855957]',
            description='Originally measured via /r_goal_pose with the right arm in its '
                        'own bottle_ready initial pose; Z shifted by the same +0.15005m '
                        'lift raise as home_position_xyz. Re-measure via /r_goal_pose if '
                        'this drifts.',
        ),
        DeclareLaunchArgument(
            'home_orientation_xyzw',
            default_value='[-0.0657237321138382, -0.6881383657455444, '
                           '-0.06250208616256714, 0.7198885083198547]',
        ),
        DeclareLaunchArgument(
            'right_home_orientation_xyzw',
            default_value='[0.0784875676035881, -0.6821072101593018, '
                           '0.08384530246257782, 0.7221768498420715]',
            description='Measured via /r_goal_pose with the right arm in its own '
                        'bottle_ready initial pose.',
        ),
        DeclareLaunchArgument('home_duration', default_value='4.0'),

        # --- Gripper ---------------------------------------------------------------
        DeclareLaunchArgument('gripper_duration', default_value='1.0'),
        DeclareLaunchArgument('gripper_settle_time', default_value='0.2'),
        DeclareLaunchArgument(
            'command_rate_hz',
            default_value='400.0',
            description='Rate to keep re-asserting gripper/return-to-initial targets so '
                        'they stay competitive with the MoveL/MoveJ controller streaming '
                        'to the same topic.',
        ),
    ]

    node = Node(
        package='ffw_bringup',
        executable='centerpose_bottle_pick',
        name='centerpose_bottle_pick',
        output='screen',
        parameters=[{
            # Detection / vision input
            'detections_topic': LaunchConfiguration('detections_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'depth_window': LaunchConfiguration('depth_window'),
            'detection_timeout': LaunchConfiguration('detection_timeout'),
            'target_frame': LaunchConfiguration('target_frame'),
            'projection_frame': LaunchConfiguration('projection_frame'),

            # Safety
            'max_grasp_x': LaunchConfiguration('max_grasp_x'),

            # Arm selection
            'arm': LaunchConfiguration('arm'),
            'arm_center_deadband_ratio': LaunchConfiguration('arm_center_deadband_ratio'),
            'eef_link': LaunchConfiguration('eef_link'),
            'right_eef_link': LaunchConfiguration('right_eef_link'),

            # Motion execution toggle
            'execute_motion': LaunchConfiguration('execute_motion'),

            # Per-arm grasp position/orientation calibration
            'fixed_grasp_z': LaunchConfiguration('fixed_grasp_z'),
            'right_fixed_grasp_z': LaunchConfiguration('right_fixed_grasp_z'),
            'grasp_position_offset': LaunchConfiguration('grasp_position_offset'),
            'right_grasp_position_offset': LaunchConfiguration('right_grasp_position_offset'),
            'tool_orientation_offset_xyzw': LaunchConfiguration('tool_orientation_offset_xyzw'),
            'right_tool_orientation_offset_xyzw': LaunchConfiguration(
                'right_tool_orientation_offset_xyzw'
            ),
            'grasp_fixed_pitch': LaunchConfiguration('grasp_fixed_pitch'),
            'right_grasp_fixed_pitch': LaunchConfiguration('right_grasp_fixed_pitch'),
            'grasp_fixed_yaw': LaunchConfiguration('grasp_fixed_yaw'),
            'right_grasp_fixed_yaw': LaunchConfiguration('right_grasp_fixed_yaw'),
            'grasp_roll_from_yaw_scale': LaunchConfiguration('grasp_roll_from_yaw_scale'),
            'right_grasp_roll_from_yaw_scale': LaunchConfiguration(
                'right_grasp_roll_from_yaw_scale'
            ),
            'grasp_roll_offset': LaunchConfiguration('grasp_roll_offset'),
            'right_grasp_roll_offset': LaunchConfiguration('right_grasp_roll_offset'),
            'bottle_yaw_zero_offset_deg': LaunchConfiguration('bottle_yaw_zero_offset_deg'),
            'roll_clamp_min_deg': LaunchConfiguration('roll_clamp_min_deg'),
            'roll_clamp_max_deg': LaunchConfiguration('roll_clamp_max_deg'),
            'right_roll_clamp_min_deg': LaunchConfiguration('right_roll_clamp_min_deg'),
            'right_roll_clamp_max_deg': LaunchConfiguration('right_roll_clamp_max_deg'),

            # Pick sequence timing: approach -> insert -> gripper close -> lift
            'movel_duration': LaunchConfiguration('movel_duration'),
            'pregrasp_distance': LaunchConfiguration('pregrasp_distance'),
            'pregrasp_duration': LaunchConfiguration('pregrasp_duration'),
            'insertion_duration': LaunchConfiguration('insertion_duration'),
            'settle_time': LaunchConfiguration('settle_time'),
            'lift_height': LaunchConfiguration('lift_height'),
            'lift_duration': LaunchConfiguration('lift_duration'),

            # Retract + return-to-initial-pose sequence (after lifting)
            'return_to_initial': LaunchConfiguration('return_to_initial'),
            'retract_distance': LaunchConfiguration('retract_distance'),
            'right_retract_distance': LaunchConfiguration('right_retract_distance'),
            'retract_position_xyz': LaunchConfiguration('retract_position_xyz'),
            'right_retract_position_xyz': LaunchConfiguration('right_retract_position_xyz'),
            'retract_duration': LaunchConfiguration('retract_duration'),
            'home_position_xyz': LaunchConfiguration('home_position_xyz'),
            'right_home_position_xyz': LaunchConfiguration('right_home_position_xyz'),
            'home_orientation_xyzw': LaunchConfiguration('home_orientation_xyzw'),
            'right_home_orientation_xyzw': LaunchConfiguration('right_home_orientation_xyzw'),
            'home_duration': LaunchConfiguration('home_duration'),

            # Gripper
            'gripper_duration': LaunchConfiguration('gripper_duration'),
            'gripper_settle_time': LaunchConfiguration('gripper_settle_time'),
            'command_rate_hz': LaunchConfiguration('command_rate_hz'),
        }],
    )

    return LaunchDescription(arguments + [node])
