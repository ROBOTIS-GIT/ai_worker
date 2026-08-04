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

        # --- Motion execution toggle ----------------------------------------------
        DeclareLaunchArgument('execute_motion', default_value='false'),
        DeclareLaunchArgument('movel_topic', default_value='/l_goal_move'),
        DeclareLaunchArgument('eef_link', default_value='end_effector_l_link'),

        # --- Grasp position/orientation ---------------------------------------
        DeclareLaunchArgument(
            'fixed_grasp_z',
            default_value='0.8241714239120483',
            description='Left-arm grasp height (base_link Z), pinned to the EEF height '
                        'measured in the bottle_ready initial pose. Negative: fall back '
                        'to dynamically holding current EEF Z at capture time instead.',
        ),
        DeclareLaunchArgument(
            'grasp_position_offset',
            default_value='[0.01, -0.04, 0.0]',
            description='Calibrated from measured grasp error on the real robot.',
        ),
        DeclareLaunchArgument(
            'grasp_orientation_xyzw',
            default_value='[-0.0657237321138382, -0.6881383657455444, '
                           '-0.06250208616256714, 0.7198885083198547]',
            description='Fixed gripper orientation used for every step (pregrasp, '
                        'grasp, lift, box drop) -- measured via tf2_echo base_link '
                        'end_effector_l_link with the left arm in its bottle_ready '
                        'initial pose. The gripper never rolls to match the bottle.',
        ),

        # --- Pick sequence timing: approach -> insert -> gripper close -> lift ---
        DeclareLaunchArgument('movel_duration', default_value='5.0'),
        DeclareLaunchArgument(
            'pregrasp_distance',
            default_value='0.08',
            description='Back off along the fixed approach axis to this pregrasp '
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

        # --- Box drop-off + return-to-initial-pose sequence -----------------------
        # Two physical slots, filled left-to-right by capture order (leftmost
        # detected bottle -> slot 1, next -> slot 2); with only one bottle captured,
        # slot 1 is used. Each slot hovers at its own position/orientation, then
        # lowers by box_place_z_offset to release, then returns to the same hover
        # pose -- slot 2 needs the wrist rotated to fit, unlike the fixed
        # grasp_orientation_xyzw used while picking. All measured via `ros2 topic
        # echo --once /l_goal_pose` with the arm holding a bottle above each slot.
        DeclareLaunchArgument(
            'box_slot_1_position_xyz',
            default_value='[0.6468760967254639, 0.36483344435691833, 0.9507306218147278]',
        ),
        DeclareLaunchArgument(
            'box_slot_1_orientation_xyzw',
            default_value='[-0.06588234007358551, -0.68890780210495, '
                           '-0.06271106004714966, 0.7191194891929626]',
        ),
        DeclareLaunchArgument(
            'box_slot_2_position_xyz',
            default_value='[0.6272304654121399, 0.2589269280433655, 0.9516366720199585]',
        ),
        DeclareLaunchArgument(
            'box_slot_2_orientation_xyzw',
            default_value='[0.13866588473320007, -0.6788055896759033, '
                           '0.13322767615318298, 0.7086924910545349]',
        ),
        DeclareLaunchArgument('box_duration', default_value='5.0'),
        DeclareLaunchArgument(
            'box_place_z_offset',
            default_value='0.1093128323554992',
            description='Straight-down distance from a slot hover pose to its '
                        'release pose (both slots share this -- only the hover pose '
                        'was measured per slot).',
        ),
        DeclareLaunchArgument('box_place_duration', default_value='2.0'),
        DeclareLaunchArgument(
            'return_to_initial',
            default_value='true',
            description='Return to the initial pose after releasing and raising back '
                        'above the box.',
        ),
        DeclareLaunchArgument(
            'home_position_xyz',
            default_value='[0.13451801240444183, 0.2999741733074188, 0.9742214239120483]',
            description='Measured via tf2_echo base_link end_effector_l_link with the '
                        'left arm in its bottle_ready initial pose.',
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
        package='ffw_centerpose',
        executable='centerpose_bottle',
        name='centerpose_bottle',
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
            'grasp_orientation_xyzw': LaunchConfiguration('grasp_orientation_xyzw'),

            'movel_duration': LaunchConfiguration('movel_duration'),
            'pregrasp_distance': LaunchConfiguration('pregrasp_distance'),
            'pregrasp_duration': LaunchConfiguration('pregrasp_duration'),
            'insertion_duration': LaunchConfiguration('insertion_duration'),
            'settle_time': LaunchConfiguration('settle_time'),
            'lift_height': LaunchConfiguration('lift_height'),
            'lift_duration': LaunchConfiguration('lift_duration'),

            'box_slot_1_position_xyz': LaunchConfiguration('box_slot_1_position_xyz'),
            'box_slot_1_orientation_xyzw': LaunchConfiguration('box_slot_1_orientation_xyzw'),
            'box_slot_2_position_xyz': LaunchConfiguration('box_slot_2_position_xyz'),
            'box_slot_2_orientation_xyzw': LaunchConfiguration('box_slot_2_orientation_xyzw'),
            'box_duration': LaunchConfiguration('box_duration'),
            'box_place_z_offset': LaunchConfiguration('box_place_z_offset'),
            'box_place_duration': LaunchConfiguration('box_place_duration'),
            'return_to_initial': LaunchConfiguration('return_to_initial'),
            'home_position_xyz': LaunchConfiguration('home_position_xyz'),
            'home_duration': LaunchConfiguration('home_duration'),

            'gripper_duration': LaunchConfiguration('gripper_duration'),
            'gripper_settle_time': LaunchConfiguration('gripper_settle_time'),
            'command_rate_hz': LaunchConfiguration('command_rate_hz'),
        }],
    )

    return LaunchDescription(arguments + [node])
