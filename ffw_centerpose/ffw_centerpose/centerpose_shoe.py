#!/usr/bin/env python3

import math
import threading

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

from ffw_centerpose.pick_place_base import PickPlaceNodeBase


class CenterposeShoe(PickPlaceNodeBase):
    # DRAFT: uncalibrated placeholders below.
    _OBJECT_LABEL_PLURAL = 'shoe(s)'

    def __init__(self):
        super().__init__('centerpose_shoe')

        # --- Parameters ---
        self.declare_parameter('detections_topic', '/centerpose/detections')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('depth_topic', '/zedm/zed_node/depth/depth_registered')
        self.declare_parameter('depth_window', 5)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('projection_frame', '')
        self.declare_parameter('detection_timeout', 10.0)
        self.declare_parameter('max_grasp_x', 0.9)
        self.declare_parameter('execute_motion', False)
        self.declare_parameter('movel_topic', '/l_goal_move')
        self.declare_parameter('movel_duration', 10.0)

        # --- Startup sequence ---
        self.declare_parameter(
            'startup_position_xyz',
            [0.17088773846626282, 0.48100942373275757, 0.9724668264389038],
        )
        self.declare_parameter(
            'startup_orientation_xyzw',
            [-0.06572848558425903, -0.6882247924804688, -0.06251275539398193,
             0.7198045253753662],
        )
        self.declare_parameter('startup_duration', 10.0)
        self.declare_parameter(
            'start_position_xyz',
            [0.23394590616226196, 0.3340926766395569, 0.9377147555351257],
        )
        self.declare_parameter(
            'start_orientation_xyzw',
            [-0.002594258636236191, 0.008345617912709713, -0.00505678029730916,
             0.9999490976333618],
        )
        self.declare_parameter('start_duration', 10.0)
        self.declare_parameter('return_duration', 6.0)

        # --- Approach / insertion ---
        self.declare_parameter('pregrasp_distance', 0.1)
        self.declare_parameter('pregrasp_duration', 4.0)
        self.declare_parameter('insertion_duration', 2.0)
        self.declare_parameter('insertion_overshoot_distance', 0.1438221335411271)
        self.declare_parameter('movel_subscriber_timeout', 2.0)
        self.declare_parameter('settle_time', 0.5)
        self.declare_parameter('eef_link', 'end_effector_l_link')
        self.declare_parameter('fixed_grasp_z', 0.9377147555351257)
        self.declare_parameter('min_grasp_z', 0.7938926219940186)

        # --- Shoe hole offset ---
        self.declare_parameter('grasp_position_offset', [-0.03, 0.0, 0.0])
        self.declare_parameter('grasp_position_y_slope', -6.944444444444444e-05)
        self.declare_parameter('grasp_position_y_reference_pixel', 288.0)
        self.declare_parameter('second_shoe_grasp_y_offset', -0.02)
        self.declare_parameter('shoe_depth_center_offset', 0.0)
        self.declare_parameter('tool_orientation_offset_xyzw', [0.0, 0.0, 0.0, 1.0])

        # --- Shoe yaw -> gripper yaw calibration ---
        self.declare_parameter(
            'shoe_yaw_reference_orientation_xyzw',
            [0.04132861537025957, -0.3549227920668959, 0.9339735266150585,
             -0.003899846823498184],
        )
        self.declare_parameter(
            'shoe_yaw_axis_xyz', [0.13999845344725156, -0.9872268603133045, 0.07604971603046785]
        )
        self.declare_parameter('shoe_yaw_flip_threshold_deg', 90.0)
        self.declare_parameter('grasp_fixed_roll', -0.005273412980347409)
        self.declare_parameter('grasp_fixed_pitch', 0.01666491788848952)
        self.declare_parameter('grasp_yaw_from_shoe_yaw_scale', -1.016908340135941)
        self.declare_parameter('grasp_yaw_offset', -0.12016636998914268)
        self.declare_parameter('yaw_clamp_min_deg', -90.0)
        self.declare_parameter('yaw_clamp_max_deg', 90.0)

        self.declare_parameter(
            'left_arm_joint_trajectory_topic',
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
        )
        self.declare_parameter('left_gripper_joint', 'gripper_l_joint1')
        self.declare_parameter(
            'left_arm_joint_names',
            [
                'arm_l_joint1',
                'arm_l_joint2',
                'arm_l_joint3',
                'arm_l_joint4',
                'arm_l_joint5',
                'arm_l_joint6',
                'arm_l_joint7',
                'gripper_l_joint1',
            ],
        )
        self.declare_parameter('gripper_open_position', 0.0)
        self.declare_parameter('gripper_closed_position', 1.1)
        self.declare_parameter('gripper_release_position', 0.7)
        self.declare_parameter('gripper_duration', 1.0)
        self.declare_parameter('gripper_settle_time', 0.2)
        self.declare_parameter('command_rate_hz', 300.0)
        self.declare_parameter('lift_height', 0.1)
        self.declare_parameter('lift_duration', 2.0)

        # --- Place sequence ---
        self.declare_parameter(
            'place_slot_1_position_xyz',
            [0.47151222825050354, 0.42120254039764404, 0.9456136226654053],
        )
        self.declare_parameter(
            'place_slot_1_orientation_xyzw',
            [-0.0023092320188879967, 0.008429071865975857, 0.028931625187397003,
             0.9995430707931519],
        )
        self.declare_parameter(
            'place_slot_2_position_xyz',
            [0.45977485179901123, 0.22981253385543823, 0.9466336965560913],
        )
        self.declare_parameter(
            'place_slot_2_orientation_xyzw',
            [-0.0023092320188879967, 0.008429071865975857, 0.028931625187397003,
             0.9995430707931519],
        )
        self.declare_parameter('place_hover_duration', 4.0)
        self.declare_parameter('place_lower_distance', 0.1338221335411271)
        self.declare_parameter('place_lower_duration', 2.0)
        self.declare_parameter('place_duration', 1.0)

        # --- Read parameters ---
        self.detections_topic = self.get_parameter('detections_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.depth_window = int(self.get_parameter('depth_window').value)
        self.target_frame = self.get_parameter('target_frame').value
        self.projection_frame = self.get_parameter('projection_frame').value
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        self.max_grasp_x = float(self.get_parameter('max_grasp_x').value)
        self.execute_motion = self._bool_parameter('execute_motion')
        self.movel_topic = self.get_parameter('movel_topic').value
        self.movel_duration = float(self.get_parameter('movel_duration').value)

        self.startup_position_xyz = self._list_parameter('startup_position_xyz')
        if len(self.startup_position_xyz) != 3:
            raise ValueError('startup_position_xyz must contain [x, y, z]')
        self.startup_orientation_xyzw = self._list_parameter('startup_orientation_xyzw')
        if len(self.startup_orientation_xyzw) != 4:
            raise ValueError('startup_orientation_xyzw must contain [x, y, z, w]')
        self.startup_duration = float(self.get_parameter('startup_duration').value)

        self.start_position_xyz = self._list_parameter('start_position_xyz')
        if len(self.start_position_xyz) != 3:
            raise ValueError('start_position_xyz must contain [x, y, z]')
        self.start_orientation_xyzw = self._list_parameter('start_orientation_xyzw')
        if len(self.start_orientation_xyzw) != 4:
            raise ValueError('start_orientation_xyzw must contain [x, y, z, w]')
        self.start_duration = float(self.get_parameter('start_duration').value)
        self.return_duration = float(self.get_parameter('return_duration').value)

        self.pregrasp_distance = float(self.get_parameter('pregrasp_distance').value)
        self.pregrasp_duration = float(self.get_parameter('pregrasp_duration').value)
        self.insertion_duration = float(self.get_parameter('insertion_duration').value)
        self.insertion_overshoot_distance = float(
            self.get_parameter('insertion_overshoot_distance').value
        )
        self.movel_subscriber_timeout = float(
            self.get_parameter('movel_subscriber_timeout').value
        )
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.eef_link = str(self.get_parameter('eef_link').value)
        self.fixed_grasp_z = float(self.get_parameter('fixed_grasp_z').value)
        self.min_grasp_z = float(self.get_parameter('min_grasp_z').value)

        grasp_position_offset = self._list_parameter('grasp_position_offset')
        if len(grasp_position_offset) != 3:
            raise ValueError('grasp_position_offset must contain [x, y, z]')
        self.grasp_position_offset = grasp_position_offset
        self.grasp_position_y_slope = float(
            self.get_parameter('grasp_position_y_slope').value
        )
        self.grasp_position_y_reference_pixel = float(
            self.get_parameter('grasp_position_y_reference_pixel').value
        )
        self.second_shoe_grasp_y_offset = float(
            self.get_parameter('second_shoe_grasp_y_offset').value
        )
        self.shoe_depth_center_offset = float(
            self.get_parameter('shoe_depth_center_offset').value
        )

        tool_orientation_offset = np.asarray(
            self._list_parameter('tool_orientation_offset_xyzw'), dtype=np.float64
        )
        if tool_orientation_offset.shape != (4,):
            raise ValueError('tool_orientation_offset_xyzw must contain [x, y, z, w]')
        tool_orientation_norm = np.linalg.norm(tool_orientation_offset)
        if tool_orientation_norm < 1e-9:
            raise ValueError('tool_orientation_offset_xyzw must not be zero')
        self.tool_orientation_offset = tool_orientation_offset / tool_orientation_norm

        shoe_yaw_reference_orientation = np.asarray(
            self._list_parameter('shoe_yaw_reference_orientation_xyzw'), dtype=np.float64
        )
        if shoe_yaw_reference_orientation.shape != (4,):
            raise ValueError('shoe_yaw_reference_orientation_xyzw must contain [x, y, z, w]')
        reference_norm = np.linalg.norm(shoe_yaw_reference_orientation)
        if reference_norm < 1e-9:
            raise ValueError('shoe_yaw_reference_orientation_xyzw must not be zero')
        self.shoe_yaw_reference_orientation = shoe_yaw_reference_orientation / reference_norm

        shoe_yaw_axis = np.asarray(self._list_parameter('shoe_yaw_axis_xyz'), dtype=np.float64)
        if shoe_yaw_axis.shape != (3,):
            raise ValueError('shoe_yaw_axis_xyz must contain [x, y, z]')
        shoe_yaw_axis_norm = np.linalg.norm(shoe_yaw_axis)
        if shoe_yaw_axis_norm < 1e-9:
            raise ValueError('shoe_yaw_axis_xyz must not be zero')
        self.shoe_yaw_axis = shoe_yaw_axis / shoe_yaw_axis_norm

        self.shoe_yaw_flip_threshold_deg = float(
            self.get_parameter('shoe_yaw_flip_threshold_deg').value
        )
        self.grasp_fixed_roll = float(self.get_parameter('grasp_fixed_roll').value)
        self.grasp_fixed_pitch = float(self.get_parameter('grasp_fixed_pitch').value)
        self.grasp_yaw_from_shoe_yaw_scale = float(
            self.get_parameter('grasp_yaw_from_shoe_yaw_scale').value
        )
        self.grasp_yaw_offset = float(self.get_parameter('grasp_yaw_offset').value)
        self.yaw_clamp_min_deg = float(self.get_parameter('yaw_clamp_min_deg').value)
        self.yaw_clamp_max_deg = float(self.get_parameter('yaw_clamp_max_deg').value)

        self.left_arm_joint_trajectory_topic = self.get_parameter(
            'left_arm_joint_trajectory_topic'
        ).value
        self.left_gripper_joint = self.get_parameter('left_gripper_joint').value
        self.left_arm_joint_names = [
            str(name) for name in self.get_parameter('left_arm_joint_names').value
        ]
        if self.left_gripper_joint not in self.left_arm_joint_names:
            raise ValueError('left_gripper_joint must be included in left_arm_joint_names')

        self.gripper_open_position = float(self.get_parameter('gripper_open_position').value)
        self.gripper_closed_position = float(
            self.get_parameter('gripper_closed_position').value
        )
        self.gripper_release_position = float(
            self.get_parameter('gripper_release_position').value
        )
        self.gripper_duration = float(self.get_parameter('gripper_duration').value)
        self.gripper_settle_time = float(self.get_parameter('gripper_settle_time').value)
        self.command_rate_hz = float(self.get_parameter('command_rate_hz').value)
        self.lift_height = float(self.get_parameter('lift_height').value)
        self.lift_duration = float(self.get_parameter('lift_duration').value)

        self.place_slots = [self._build_place_slot(1), self._build_place_slot(2)]
        self.place_hover_duration = float(self.get_parameter('place_hover_duration').value)
        self.place_lower_distance = float(self.get_parameter('place_lower_distance').value)
        self.place_lower_duration = float(self.get_parameter('place_lower_duration').value)
        self.place_duration = float(self.get_parameter('place_duration').value)

        self._setup_common()

        self.get_logger().warn(
            'CenterposeShoe is a DRAFT: grasp_position_offset (shoe hole x/y offset) and '
            'place_lower_distance (how far to lower before releasing) are not '
            'independently measured yet.'
        )
        self.get_logger().info('CenterposeShoe (left arm) ready')
        self.get_logger().info(f'  MoveL: {self.movel_topic}')
        self.get_logger().info(f'  detections: {self.detections_topic}')
        self.get_logger().info(f'  camera info: {self.camera_info_topic}')
        self.get_logger().info(f'  depth (x/y only): {self.depth_topic}')
        for index, slot in enumerate(self.place_slots, start=1):
            self.get_logger().info(f'  place slot {index} hover: {slot["position"]}')

        if self.execute_motion:
            threading.Thread(target=self._startup_sequence, daemon=True).start()

    def _startup_sequence(self):
        startup_pose = PoseStamped()
        startup_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(
            startup_pose, self.startup_position_xyz, self.startup_orientation_xyzw
        )
        start_pose = PoseStamped()
        start_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(start_pose, self.start_position_xyz, self.start_orientation_xyzw)

        with self.execution_lock:
            self.execution_step = 'startup: move to startup pose (very slow)'
            self.get_logger().info('Executing startup: move to startup pose (very slow)')
            if not self._move_l(startup_pose, duration=self.startup_duration):
                self.get_logger().error('Startup sequence stopped at startup pose')
                self.execution_step = 'idle'
                return

            self.execution_step = 'startup: move to start pose (very slow)'
            self.get_logger().info('Executing startup: move to start pose (very slow)')
            if not self._move_l(start_pose, duration=self.start_duration):
                self.get_logger().error('Startup sequence stopped at start pose')
                self.execution_step = 'idle'
                return

            self.get_logger().info('Startup sequence finished; holding start pose')
            self.execution_step = 'idle'

    def _build_place_slot(self, index):
        position = self._list_parameter(f'place_slot_{index}_position_xyz')
        if len(position) != 3:
            raise ValueError(f'place_slot_{index}_position_xyz must contain [x, y, z]')

        orientation = np.asarray(
            self._list_parameter(f'place_slot_{index}_orientation_xyzw'), dtype=np.float64
        )
        if orientation.shape != (4,):
            raise ValueError(f'place_slot_{index}_orientation_xyzw must contain [x, y, z, w]')
        norm = np.linalg.norm(orientation)
        if norm < 1e-9:
            raise ValueError(f'place_slot_{index}_orientation_xyzw must not be zero')

        return {'position': position, 'orientation': orientation / norm}

    # --- Detection -> pose conversion ---
    def _process_single_detection(
        self, detection, camera_info, depth_image, depth_msg,
        camera_transform, fixed_z, log, index
    ):
        center = detection.bbox.center.position
        if not all(math.isfinite(value) for value in (center.x, center.y, center.z)):
            return None

        projected = self._project_to_pixel(camera_info, center, log)
        if projected is None:
            return None
        u, v = projected

        real_point_cam = self._real_camera_point(
            camera_info, u, v, depth_image, depth_msg, log
        )
        if real_point_cam is None:
            return None

        transformed = self._transform_centerpose(
            camera_transform, real_point_cam, detection.bbox.center.orientation
        )
        if transformed is None:
            return None
        point, orientation, object_yaw, gripper_yaw = transformed

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.target_frame
        y_offset = self.grasp_position_offset[1] + self.grasp_position_y_slope * (
            u - self.grasp_position_y_reference_pixel
        )
        pose.pose.position.x = float(point[0] + self.grasp_position_offset[0])
        pose.pose.position.y = float(point[1] + y_offset)
        pose.pose.position.z = fixed_z
        pose.pose.orientation = self._quaternion_message(orientation)

        self.object_pose_pub.publish(pose)
        self.grasp_pose_pub.publish(pose)

        if log:
            self.get_logger().info(
                f'[{index}] pixel=({u:.0f}, {v:.0f})/{camera_info.width}, '
                f'real camera xyz=({real_point_cam[0]:.3f}, {real_point_cam[1]:.3f}, '
                f'{real_point_cam[2]:.3f}), fixed z={fixed_z:.3f}, target=('
                f'{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, '
                f'{pose.pose.position.z:.3f}), '
                f'shoe_yaw={math.degrees(object_yaw):.1f} deg -> '
                f'gripper_yaw={math.degrees(gripper_yaw):.1f} deg'
            )
        return {'pose': pose, 'pixel_u': u}

    def _real_camera_point(self, camera_info, u, v, depth_image, depth_msg, log):
        # TODO: shoe_depth_center_offset unmeasured.
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        u_px = int(round(u))
        v_px = int(round(v))

        depth = self._sample_depth(depth_image, depth_msg, u_px, v_px)
        if depth is None:
            if log:
                self.get_logger().warn(
                    f'Invalid depth around pixel ({u_px}, {v_px})', throttle_duration_sec=2.0
                )
            return None

        depth += self.shoe_depth_center_offset

        return np.array(
            [(u - cx) * depth / fx, (v - cy) * depth / fy, depth], dtype=np.float64
        )

    def _execute_queue(self, queue):
        with self.execution_lock:
            try:
                for index, item in enumerate(queue):
                    if self.cancel_event.is_set():
                        return
                    label = f'shoe {index + 1}/{len(queue)}'
                    slot = self.place_slots[min(index, len(self.place_slots) - 1)]
                    grasp_pose = item['pose']
                    if index == 1:
                        grasp_pose.pose.position.y += self.second_shoe_grasp_y_offset
                    if not self._pick_and_place(grasp_pose, slot, label):
                        if self.cancel_event.is_set():
                            return
                        self.get_logger().error(f'{label} failed; continuing with next shoe')
                        continue
                self.get_logger().info(f'Finished picking {len(queue)} shoe(s)')

                if not self.cancel_event.is_set():
                    start_pose = PoseStamped()
                    start_pose.header.frame_id = self.target_frame
                    self._set_pose_from_arrays(
                        start_pose, self.start_position_xyz, self.start_orientation_xyzw
                    )
                    self.execution_step = 'return to start pose (1.5x speed)'
                    self.get_logger().info('Executing return to start pose (1.5x speed)')
                    if not self._move_l(start_pose, duration=self.return_duration):
                        self.get_logger().error('Stopped returning to start pose')
            finally:
                self.execution_step = 'idle'

    # --- Pick/place motion ---
    def _pick_and_place(self, grasp_pose, slot, label='shoe'):
        grasp_q = np.array([
            grasp_pose.pose.orientation.x,
            grasp_pose.pose.orientation.y,
            grasp_pose.pose.orientation.z,
            grasp_pose.pose.orientation.w,
        ], dtype=np.float64)
        approach_dir = self._rotate_vector(np.array([0.0, 0.0, -1.0]), grasp_q)

        insert_pose = self._copy_pose(grasp_pose)
        insert_pose.pose.position.x += approach_dir[0] * self.insertion_overshoot_distance
        insert_pose.pose.position.y += approach_dir[1] * self.insertion_overshoot_distance
        insert_pose.pose.position.z += approach_dir[2] * self.insertion_overshoot_distance
        insert_pose.pose.position.z = max(insert_pose.pose.position.z, self.min_grasp_z)

        pregrasp_pose = self._copy_pose(insert_pose)
        pregrasp_pose.pose.position.x -= approach_dir[0] * self.pregrasp_distance
        pregrasp_pose.pose.position.y -= approach_dir[1] * self.pregrasp_distance
        pregrasp_pose.pose.position.z -= approach_dir[2] * self.pregrasp_distance

        lift_pose = self._copy_pose(insert_pose)
        lift_pose.pose.position.z += self.lift_height

        place_hover_pose = PoseStamped()
        place_hover_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(place_hover_pose, slot['position'], slot['orientation'])

        place_lower_pose = self._copy_pose(place_hover_pose)
        place_lower_pose.pose.position.z -= self.place_lower_distance

        steps = [
            ('open gripper', lambda: self._move_gripper(self.gripper_open_position)),
            ('move to pregrasp', lambda: self._move_l(pregrasp_pose, duration=self.pregrasp_duration)),
            ('insert to shoe', lambda: self._move_l(insert_pose, duration=self.insertion_duration)),
            ('close gripper', lambda: self._move_gripper(self.gripper_closed_position)),
            ('lift shoe', lambda: self._move_l(lift_pose, duration=self.lift_duration)),
            ('move to place hover', lambda: self._move_l(place_hover_pose, duration=self.place_hover_duration)),
            ('lower into place', lambda: self._move_l(place_lower_pose, duration=self.place_lower_duration)),
            ('release shoe', lambda: self._move_gripper(self.gripper_release_position)),
            ('back to place hover', lambda: self._move_l(place_hover_pose, duration=self.place_duration)),
        ]

        for name, command in steps:
            if self.cancel_event.is_set():
                return False
            self.execution_step = f'{label}: {name}'
            self.get_logger().info(f'Executing {label} {name}')
            if not command():
                self.get_logger().error(f'Stopped at {label} {name}')
                return False
        self.get_logger().info(f'{label} finished; holding pose')
        return True

    def _transform_centerpose(self, camera_transform, position_cam, orientation):
        transform_q, t = camera_transform
        object_q = np.array(
            [orientation.x, orientation.y, orientation.z, orientation.w], dtype=np.float64
        )
        object_norm = np.linalg.norm(object_q)
        if object_norm < 1e-9:
            self.get_logger().warn('CenterPose orientation contains a zero quaternion')
            return None
        object_q /= object_norm

        point = self._rotate_vector(np.asarray(position_cam, dtype=np.float64), transform_q) + t

        object_yaw, raw_angle = self._signed_shoe_yaw(object_q)
        if math.degrees(raw_angle) > self.shoe_yaw_flip_threshold_deg:
            object_q = self._quaternion_multiply(object_q, np.array([0.0, 1.0, 0.0, 0.0]))
            object_yaw, _ = self._signed_shoe_yaw(object_q)

        raw_yaw_deg = math.degrees(
            self.grasp_yaw_from_shoe_yaw_scale * object_yaw + self.grasp_yaw_offset
        )
        clamped_yaw_deg = min(
            max(raw_yaw_deg, self.yaw_clamp_min_deg), self.yaw_clamp_max_deg
        )
        gripper_yaw = self._wrap_angle(math.radians(clamped_yaw_deg))
        base_q = self._quaternion_from_euler(
            self.grasp_fixed_roll, self.grasp_fixed_pitch, gripper_yaw
        )
        target_q = self._quaternion_multiply(base_q, self.tool_orientation_offset)
        return point, target_q, object_yaw, gripper_yaw

    def _signed_shoe_yaw(self, object_q):
        relative_q = self._quaternion_multiply(
            self._quaternion_conjugate(self.shoe_yaw_reference_orientation), object_q
        )
        axis, angle = self._quaternion_axis_angle(relative_q)
        sign = 1.0 if np.dot(axis, self.shoe_yaw_axis) >= 0.0 else -1.0
        return sign * angle, angle

    # --- Quaternion/angle helpers (same as centerpose_box.py) ---
    @staticmethod
    def _quaternion_multiply(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return np.array(
            [
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz,
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _quaternion_conjugate(q):
        x, y, z, w = q
        return np.array([-x, -y, -z, w], dtype=np.float64)

    @staticmethod
    def _quaternion_axis_angle(q):
        q = q / np.linalg.norm(q)
        x, y, z, w = q
        if w < 0.0:
            x, y, z, w = -x, -y, -z, -w
        angle = 2.0 * math.acos(max(-1.0, min(1.0, w)))
        s = math.sqrt(max(0.0, 1.0 - w * w))
        axis = np.zeros(3) if s < 1e-8 else np.array([x, y, z]) / s
        return axis, angle

    @staticmethod
    def _wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _quaternion_from_euler(roll, pitch, yaw):
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        return np.array(
            [
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy,
            ],
            dtype=np.float64,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CenterposeShoe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
