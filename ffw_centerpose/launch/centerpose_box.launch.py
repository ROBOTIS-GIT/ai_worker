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
                        'misread (e.g. a background hit instead of the box).',
        ),

        # --- Motion execution toggle ----------------------------------------------
        DeclareLaunchArgument('execute_motion', default_value='false'),
        DeclareLaunchArgument('movel_topic', default_value='/l_goal_move'),
        DeclareLaunchArgument('eef_link', default_value='end_effector_l_link'),

        # --- Grasp position/orientation ---------------------------------------
        DeclareLaunchArgument(
            'fixed_grasp_z',
            default_value='0.8241714239120483',
            description='Left-arm grasp height (base_link Z), same fixed height as '
                        'centerpose_bottle -- measured via tf2_echo base_link '
                        'end_effector_l_link in the bottle_ready initial pose. '
                        'Negative: fall back to dynamically holding current EEF Z at '
                        'capture time instead.',
        ),
        DeclareLaunchArgument(
            'grasp_position_offset',
            default_value='[0.01, -0.02, 0.0]',
            description='[x, y, z] offset, both x and y valid AT '
                        'grasp_position_y_reference_pixel ("left") -- see '
                        'grasp_position_x_slope/grasp_position_y_slope below for the '
                        'pixel-dependent part. y=-0.02 (2cm right, was 0.0) on '
                        '2026-07-30 -- grasp at "left" was landing 2cm too far left. '
                        'x=+0.01 (1cm forward, was 0.0) same day.',
        ),
        DeclareLaunchArgument(
            'grasp_position_y_slope',
            default_value='0.0',
            description='A constant y offset only ever matches one screen position -- '
                        'real-robot testing repeatedly adjusted this as the box needs '
                        'progressively more +Y (left) correction from "left" toward '
                        '"center" (or less, depending on the current sign): 0.0 at '
                        'pixel_u=288 ("left"), 0.0 at pixel_u=576 ("center") too -- '
                        'combined with grasp_position_offset[1]=-0.02, gives -0.02 '
                        '(2cm right) at both. Adjusted repeatedly on 2026-07-30 '
                        '(+0.03, then +0.01, then -0.01, -0.02, -0.03, -0.05, -0.07, '
                        '-0.01, 0.0, +0.01, 0.0, +0.02, 0.0, settled at 0.0) -- '
                        'centered grasp kept overshooting left/right each time. '
                        'Applied as: y_offset = grasp_position_offset[1] + '
                        'grasp_position_y_slope * (pixel_u - '
                        'grasp_position_y_reference_pixel).',
        ),
        DeclareLaunchArgument(
            'grasp_position_y_reference_pixel',
            default_value='288.0',
            description='Pixel u at which grasp_position_offset[1] is exactly correct '
                        '(no slope correction applied there).',
        ),
        DeclareLaunchArgument(
            'grasp_position_x_slope',
            default_value='-3.472222222222222e-05',
            description='Same idea as grasp_position_y_slope, but for forward/'
                        'backward. Applied as: x_offset = grasp_position_offset[0] + '
                        'grasp_position_x_slope * (pixel_u - '
                        'grasp_position_y_reference_pixel). Added 2026-07-30 at '
                        '+0.02 (2cm more forward than grasp_position_offset[0] at '
                        '"center"), settled at a value that cancels '
                        'grasp_position_offset[0] out to 0.0 (no correction) at '
                        '"center".',
        ),
        DeclareLaunchArgument(
            'box_depth_center_offset',
            default_value='0.08',
            description='Depth reads the box\'s visible front face, not its center -- '
                        'the box has real thickness, so the center sits this much '
                        'further along the same camera ray. Added directly to the '
                        'sampled depth (not applied to x/y/z separately afterwards), '
                        'since x/y also scale with depth in the pinhole '
                        'back-projection.',
        ),
        DeclareLaunchArgument(
            'tool_orientation_offset_xyzw',
            default_value='[0.0, 0.0, 0.0, 1.0]',
            description='CenterPose object frame to gripper tool-frame rotation.',
        ),
        DeclareLaunchArgument(
            'box_yaw_reference_orientation_xyzw',
            default_value='[-0.6272754451461677, 0.2145960107465087, '
                           '-0.6681762105760168, 0.33766050954862303]',
            description='CenterPose bbox.center.orientation captured with the box '
                        'facing the robot (box_yaw=0 reference). object_yaw is measured '
                        'as the signed angle from THIS orientation via quaternion '
                        'axis-angle, not a plain per-quaternion Euler yaw extraction -- '
                        'the latter was tried and found not to correlate with the '
                        'needed roll (the box\'s true rotation axis isn\'t the camera\'s '
                        'Z axis).',
        ),
        DeclareLaunchArgument(
            'box_yaw_axis_xyz',
            default_value='[0.15517196976367656, -0.9264099543820705, '
                           '0.3430543050618564]',
            description='Calibrated rotation axis (camera frame) the box actually '
                        'turns about, derived from the reference above vs. a second '
                        'measured ~32.5 deg-turned sample. Used only to sign the angle '
                        'from box_yaw_reference_orientation_xyzw.',
        ),
        DeclareLaunchArgument(
            'box_yaw_flip_threshold_deg',
            default_value='90.0',
            description='CenterPose intermittently reports this box front/back-flipped '
                        '180 deg about its own local Y axis (confirmed visually against '
                        'the detector\'s rendered axes) -- this does not change the '
                        'reported bbox size, since a 180 deg turn about Y only flips '
                        'local X/Z. When the raw (unsigned) angle from the reference '
                        'exceeds this threshold, the flip is assumed and corrected '
                        'before recomputing. A real box turn should stay well under '
                        'this (see the roll_clamp window below).',
        ),
        DeclareLaunchArgument(
            'grasp_fixed_pitch',
            default_value='-1.5016251715681637',
            description='Fixed gripper pitch (rad); only roll follows the box yaw. '
                        'Taken directly from the reference sample\'s own Euler '
                        'decomposition (-86.04 deg).',
        ),
        DeclareLaunchArgument(
            'grasp_fixed_yaw',
            default_value='-0.052366725449585025',
            description='Fixed gripper yaw (rad); only roll follows the box yaw. Taken '
                        'directly from the reference sample\'s own Euler decomposition '
                        '(-3.00 deg).',
        ),
        DeclareLaunchArgument(
            'grasp_roll_from_yaw_scale',
            default_value='-0.9731714138014503',
            description='Linear fit slope: gripper roll (rad) = scale * box_yaw (rad) '
                        '+ offset, where box_yaw is the signed angle from '
                        'box_yaw_reference_orientation_xyzw (see centerpose_box.py '
                        '_signed_box_yaw). Least-squares fit from 3 measured points: '
                        'box yaw 0/+32.52/-58.51 deg -> roll -0.33/-23.54/+54.82 deg '
                        '(residuals ~1-3 deg). The -58.51 deg point arrived as a raw '
                        '~124 deg detection and needed the 180 deg flip correction '
                        'above before use. Steepened twice on 2026-07-30 (was '
                        '-0.870624840566106, then -0.9218981271837782): real-robot '
                        'testing found counter-clockwise boxes (negative box_yaw) '
                        'needed ~3 deg more roll than the fit predicted at the -58.51 '
                        'deg sample, then another ~3 deg on top of that -- box_yaw=0 '
                        'is untouched, but clockwise (positive box_yaw) grasps roll '
                        'proportionally more too since the scale is shared across '
                        'both signs.',
        ),
        DeclareLaunchArgument(
            'grasp_roll_offset',
            default_value='0.04845972067574276',
            description='Intercept of the roll-vs-yaw linear fit above (rad) -- the '
                        'fitted roll at box_yaw=0 (box facing the robot).',
        ),
        DeclareLaunchArgument(
            'box_yaw_zero_offset_deg',
            default_value='0.0',
            description='Display-only: reported box yaw = object_yaw - this offset. '
                        'Tune so a box facing the robot front reads ~0 deg.',
        ),
        DeclareLaunchArgument(
            'roll_clamp_min_deg',
            default_value='-60.0',
            description='Gripper roll (deg) is clamped to [roll_clamp_min_deg, '
                        'roll_clamp_max_deg]. Widened to +/-60 deg (was -32/+38) -- '
                        'real-robot testing confirmed the gripper can safely go this '
                        'far. Past either bound the gripper holds at that bound '
                        'instead of following the raw fit further.',
        ),
        DeclareLaunchArgument('roll_clamp_max_deg', default_value='60.0'),

        # --- Pick sequence timing: approach -> insert -> gripper close -> lift ---
        DeclareLaunchArgument('movel_duration', default_value='10.0'),
        DeclareLaunchArgument(
            'pregrasp_distance',
            default_value='0.11',
            description='Back off along the gripper approach axis to this pregrasp '
                        'point before inserting straight in, so the gripper body does '
                        'not clip the box on approach. 2026-07-30: raising this '
                        '(0.13 -> 0.15) moved the pregrasp pose forward on the real '
                        'robot instead of back; lowered below the original value '
                        '(0.13 -> 0.11) to get an actual 2cm-further-back pregrasp.',
        ),
        DeclareLaunchArgument('pregrasp_duration', default_value='4.0'),
        DeclareLaunchArgument('insertion_duration', default_value='3.0'),
        DeclareLaunchArgument(
            'insertion_overshoot_distance',
            default_value='-0.04',
            description='Detected surface position is an estimate; push this much '
                        'further along the approach axis than the raw detection before '
                        'closing the gripper, so it actually makes contact instead of '
                        'stopping right at the estimate. Pulled back 2cm, 1cm, 2cm, '
                        '2cm, then pushed 2cm back in (was 0.01) -- was not deep '
                        'enough.',
        ),
        DeclareLaunchArgument('settle_time', default_value='0.5'),
        DeclareLaunchArgument(
            'lift_height',
            default_value='0.1',
            description='Meters to lift straight up after closing the gripper.',
        ),
        DeclareLaunchArgument('lift_duration', default_value='2.0'),

        # --- Place sequence (after lifting) -> release -> retreat -> reclose -> push -> home -> open ---
        DeclareLaunchArgument(
            'place_hover_position_xyz',
            default_value='[0.4098847508430481, 0.3597005009651184, 0.9550905227661133]',
            description='Hover pose above the drop location, moved to right after '
                        'lifting. Measured via `ros2 topic echo --once /l_goal_pose` '
                        'during a manual demo with the arm holding a box (2026-07-30).',
        ),
        DeclareLaunchArgument(
            'place_hover_orientation_xyzw',
            default_value='[-0.007772511336952448, -0.692754328250885, '
                           '-0.007550723850727081, 0.7210924029350281]',
        ),
        DeclareLaunchArgument('place_hover_duration', default_value='4.0'),
        DeclareLaunchArgument(
            'place_position_xyz',
            default_value='[0.6282518005371094, 0.3551318049430847, 0.8419253444671631]',
            description='Actual release pose, moved to slowly from place_hover. Same '
                        'measurement method. Z lowered 2cm, then another 2cm (was '
                        '0.8819253444671631, then 0.8619253444671631) on 2026-07-30, '
                        'real-robot testing wanted the box placed lower. X pushed 1cm '
                        'further forward (was 0.6182518005371094) same day, wanted '
                        'the box placed 1cm further out.',
        ),
        DeclareLaunchArgument(
            'place_orientation_xyzw',
            default_value='[-0.007772511336952448, -0.692754328250885, '
                           '-0.007550723850727081, 0.7210924029350281]',
        ),
        DeclareLaunchArgument('place_duration', default_value='6.0'),
        DeclareLaunchArgument(
            'place_retreat_position_xyz',
            default_value='[0.5029386687278748, 0.3570454716682434, 0.847183346748352]',
            description='Backed-off pose moved to right after releasing (gripper still '
                        'fully open), before reclosing and pushing the box the rest of '
                        'the way in. Pulled 2cm further toward the robot (base_link '
                        '-X, was 0.5229386687278748) on 2026-07-30.',
        ),
        DeclareLaunchArgument(
            'place_retreat_orientation_xyzw',
            default_value='[-0.00785414595156908, -0.6926011443138123, '
                           '-0.007563420571386814, 0.721238374710083]',
        ),
        DeclareLaunchArgument('place_retreat_duration', default_value='2.0'),
        DeclareLaunchArgument(
            'place_release_gripper_position',
            default_value='0.9',
            description='At place_retreat, the gripper recloses to this narrower '
                        'position -- not fully open, but not gripping -- just enough '
                        'to clear the box\'s sides while pushing back in.',
        ),
        DeclareLaunchArgument(
            'place_push_position_xyz',
            default_value='[0.6187779307365417, 0.3549533486366272, 0.8432992696762085]',
            description='Pushed-back-in pose moved to from place_retreat, to shove the '
                        'released box the rest of the way into place.',
        ),
        DeclareLaunchArgument(
            'place_push_orientation_xyzw',
            default_value='[-0.00785414595156908, -0.6926011443138123, '
                           '-0.007563420571386814, 0.721238374710083]',
        ),
        DeclareLaunchArgument('place_push_duration', default_value='2.0'),
        DeclareLaunchArgument(
            'return_to_initial',
            default_value='true',
            description='Return to the initial pose after pushing the box into '
                        'place, then open the gripper fully.',
        ),
        DeclareLaunchArgument(
            'home_position_xyz',
            default_value='[0.13451801240444183, 0.2999741733074188, 0.9742214239120483]',
            description='Measured via tf2_echo base_link end_effector_l_link with the '
                        'left arm in its bottle_ready initial pose.',
        ),
        DeclareLaunchArgument(
            'home_orientation_xyzw',
            default_value='[-0.0657237321138382, -0.6881383657455444, '
                           '-0.06250208616256714, 0.7198885083198547]',
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
            'eef_link': LaunchConfiguration('eef_link'),

            'fixed_grasp_z': LaunchConfiguration('fixed_grasp_z'),
            'grasp_position_offset': LaunchConfiguration('grasp_position_offset'),
            'grasp_position_y_slope': LaunchConfiguration('grasp_position_y_slope'),
            'grasp_position_y_reference_pixel': LaunchConfiguration(
                'grasp_position_y_reference_pixel'
            ),
            'grasp_position_x_slope': LaunchConfiguration('grasp_position_x_slope'),
            'box_depth_center_offset': LaunchConfiguration('box_depth_center_offset'),
            'tool_orientation_offset_xyzw': LaunchConfiguration('tool_orientation_offset_xyzw'),
            'box_yaw_reference_orientation_xyzw': LaunchConfiguration(
                'box_yaw_reference_orientation_xyzw'
            ),
            'box_yaw_axis_xyz': LaunchConfiguration('box_yaw_axis_xyz'),
            'box_yaw_flip_threshold_deg': LaunchConfiguration('box_yaw_flip_threshold_deg'),
            'grasp_fixed_pitch': LaunchConfiguration('grasp_fixed_pitch'),
            'grasp_fixed_yaw': LaunchConfiguration('grasp_fixed_yaw'),
            'grasp_roll_from_yaw_scale': LaunchConfiguration('grasp_roll_from_yaw_scale'),
            'grasp_roll_offset': LaunchConfiguration('grasp_roll_offset'),
            'box_yaw_zero_offset_deg': LaunchConfiguration('box_yaw_zero_offset_deg'),
            'roll_clamp_min_deg': LaunchConfiguration('roll_clamp_min_deg'),
            'roll_clamp_max_deg': LaunchConfiguration('roll_clamp_max_deg'),

            'movel_duration': LaunchConfiguration('movel_duration'),
            'pregrasp_distance': LaunchConfiguration('pregrasp_distance'),
            'pregrasp_duration': LaunchConfiguration('pregrasp_duration'),
            'insertion_duration': LaunchConfiguration('insertion_duration'),
            'insertion_overshoot_distance': LaunchConfiguration('insertion_overshoot_distance'),
            'settle_time': LaunchConfiguration('settle_time'),
            'lift_height': LaunchConfiguration('lift_height'),
            'lift_duration': LaunchConfiguration('lift_duration'),

            'place_hover_position_xyz': LaunchConfiguration('place_hover_position_xyz'),
            'place_hover_orientation_xyzw': LaunchConfiguration('place_hover_orientation_xyzw'),
            'place_hover_duration': LaunchConfiguration('place_hover_duration'),
            'place_position_xyz': LaunchConfiguration('place_position_xyz'),
            'place_orientation_xyzw': LaunchConfiguration('place_orientation_xyzw'),
            'place_duration': LaunchConfiguration('place_duration'),
            'place_release_gripper_position': LaunchConfiguration(
                'place_release_gripper_position'
            ),
            'place_retreat_position_xyz': LaunchConfiguration('place_retreat_position_xyz'),
            'place_retreat_orientation_xyzw': LaunchConfiguration(
                'place_retreat_orientation_xyzw'
            ),
            'place_retreat_duration': LaunchConfiguration('place_retreat_duration'),
            'place_push_position_xyz': LaunchConfiguration('place_push_position_xyz'),
            'place_push_orientation_xyzw': LaunchConfiguration('place_push_orientation_xyzw'),
            'place_push_duration': LaunchConfiguration('place_push_duration'),
            'return_to_initial': LaunchConfiguration('return_to_initial'),
            'home_position_xyz': LaunchConfiguration('home_position_xyz'),
            'home_orientation_xyzw': LaunchConfiguration('home_orientation_xyzw'),
            'home_duration': LaunchConfiguration('home_duration'),

            'gripper_duration': LaunchConfiguration('gripper_duration'),
            'gripper_settle_time': LaunchConfiguration('gripper_settle_time'),
            'command_rate_hz': LaunchConfiguration('command_rate_hz'),
        }],
    )

    return LaunchDescription(arguments + [node])
