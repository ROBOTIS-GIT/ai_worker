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

import math

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

from ffw_centerpose.pick_place_base import PickPlaceNodeBase


class CenterposeBox(PickPlaceNodeBase):
    _OBJECT_LABEL_PLURAL = 'box(es)'

    _LOCAL_Y_180_FLIP = np.array([0.0, 1.0, 0.0, 0.0])

    def __init__(self):
        super().__init__('centerpose_box')

        # --- Parameters ---
        self.declare_parameter('detections_topic', '/centerpose/detections')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('depth_window', 5)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('projection_frame', '')
        self.declare_parameter('detection_timeout', 10.0)
        self.declare_parameter('max_grasp_x', 0.7)
        self.declare_parameter('execute_motion', False)
        self.declare_parameter('movel_topic', '/l_goal_move')
        self.declare_parameter('movel_duration', 10.0)
        self.declare_parameter('pregrasp_distance', 0.11)
        self.declare_parameter('pregrasp_duration', 4.0)
        self.declare_parameter('insertion_duration', 3.0)
        # Real values for this group come from centerpose_box_calibration.yaml.
        self.declare_parameter('insertion_overshoot_distance', 0.0)
        self.declare_parameter('movel_subscriber_timeout', 2.0)
        self.declare_parameter('settle_time', 0.5)
        self.declare_parameter('eef_link', 'end_effector_l_link')
        self.declare_parameter('fixed_grasp_z', 0.0)
        # --- Grasp position/yaw calibration ---
        self.declare_parameter('grasp_position_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('grasp_position_y_slope', 0.0)
        self.declare_parameter('grasp_position_y_reference_pixel', 0.0)
        self.declare_parameter('grasp_position_x_slope', 0.0)
        self.declare_parameter('box_depth_center_offset', 0.0)
        self.declare_parameter('tool_orientation_offset_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('box_yaw_reference_orientation_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('box_yaw_axis_xyz', [0.0, 0.0, 1.0])
        self.declare_parameter('box_yaw_flip_threshold_deg', 90.0)
        self.declare_parameter('grasp_fixed_pitch', 0.0)
        self.declare_parameter('grasp_fixed_yaw', 0.0)
        self.declare_parameter('grasp_roll_from_yaw_scale', 0.0)
        self.declare_parameter('grasp_roll_offset', 0.0)
        self.declare_parameter('box_yaw_zero_offset_deg', 0.0)
        self.declare_parameter('roll_clamp_min_deg', -60.0)
        self.declare_parameter('roll_clamp_max_deg', 60.0)
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
        self.declare_parameter('gripper_closed_position', 0.57)
        self.declare_parameter('gripper_duration', 1.0)
        self.declare_parameter('gripper_settle_time', 0.2)
        self.declare_parameter('command_rate_hz', 300.0)
        self.declare_parameter('lift_height', 0.1)
        self.declare_parameter('lift_duration', 2.0)
        # --- Place sequence (values from centerpose_box_calibration.yaml) ---
        self.declare_parameter('place_hover_position_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('place_hover_orientation_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('place_hover_duration', 4.0)
        self.declare_parameter('place_position_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('place_orientation_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('place_duration', 6.0)
        self.declare_parameter('place_release_gripper_position', 0.9)
        self.declare_parameter('place_retreat_position_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('place_retreat_orientation_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('place_retreat_duration', 2.0)
        self.declare_parameter('place_push_position_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('place_push_orientation_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('place_push_duration', 2.0)
        self.declare_parameter('return_to_initial', True)
        self.declare_parameter('home_position_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('home_orientation_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('home_duration', 6.0)

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
        self.pregrasp_distance = float(self.get_parameter('pregrasp_distance').value)
        self.insertion_overshoot_distance = float(
            self.get_parameter('insertion_overshoot_distance').value
        )
        self.pregrasp_duration = float(self.get_parameter('pregrasp_duration').value)
        self.insertion_duration = float(self.get_parameter('insertion_duration').value)
        self.movel_subscriber_timeout = float(
            self.get_parameter('movel_subscriber_timeout').value
        )
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.eef_link = str(self.get_parameter('eef_link').value)
        self.fixed_grasp_z = float(self.get_parameter('fixed_grasp_z').value)

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
        self.grasp_position_x_slope = float(
            self.get_parameter('grasp_position_x_slope').value
        )
        self.box_depth_center_offset = float(
            self.get_parameter('box_depth_center_offset').value
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

        box_yaw_reference_orientation = np.asarray(
            self._list_parameter('box_yaw_reference_orientation_xyzw'), dtype=np.float64
        )
        if box_yaw_reference_orientation.shape != (4,):
            raise ValueError('box_yaw_reference_orientation_xyzw must contain [x, y, z, w]')
        reference_norm = np.linalg.norm(box_yaw_reference_orientation)
        if reference_norm < 1e-9:
            raise ValueError('box_yaw_reference_orientation_xyzw must not be zero')
        self.box_yaw_reference_orientation = box_yaw_reference_orientation / reference_norm

        box_yaw_axis = np.asarray(self._list_parameter('box_yaw_axis_xyz'), dtype=np.float64)
        if box_yaw_axis.shape != (3,):
            raise ValueError('box_yaw_axis_xyz must contain [x, y, z]')
        box_yaw_axis_norm = np.linalg.norm(box_yaw_axis)
        if box_yaw_axis_norm < 1e-9:
            raise ValueError('box_yaw_axis_xyz must not be zero')
        self.box_yaw_axis = box_yaw_axis / box_yaw_axis_norm

        self.box_yaw_flip_threshold_deg = float(
            self.get_parameter('box_yaw_flip_threshold_deg').value
        )

        self.grasp_fixed_pitch = float(self.get_parameter('grasp_fixed_pitch').value)
        self.grasp_fixed_yaw = float(self.get_parameter('grasp_fixed_yaw').value)
        self.grasp_roll_from_yaw_scale = float(
            self.get_parameter('grasp_roll_from_yaw_scale').value
        )
        self.grasp_roll_offset = float(self.get_parameter('grasp_roll_offset').value)
        self.box_yaw_zero_offset_deg = float(
            self.get_parameter('box_yaw_zero_offset_deg').value
        )
        self.roll_clamp_min_deg = float(self.get_parameter('roll_clamp_min_deg').value)
        self.roll_clamp_max_deg = float(self.get_parameter('roll_clamp_max_deg').value)

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
        self.gripper_duration = float(self.get_parameter('gripper_duration').value)
        self.gripper_settle_time = float(self.get_parameter('gripper_settle_time').value)
        self.command_rate_hz = float(self.get_parameter('command_rate_hz').value)
        self.lift_height = float(self.get_parameter('lift_height').value)
        self.lift_duration = float(self.get_parameter('lift_duration').value)

        self.place_hover_position_xyz = self._list_parameter('place_hover_position_xyz')
        if len(self.place_hover_position_xyz) != 3:
            raise ValueError('place_hover_position_xyz must contain [x, y, z]')
        self.place_hover_orientation_xyzw = self._list_parameter(
            'place_hover_orientation_xyzw'
        )
        if len(self.place_hover_orientation_xyzw) != 4:
            raise ValueError('place_hover_orientation_xyzw must contain [x, y, z, w]')
        self.place_hover_duration = float(self.get_parameter('place_hover_duration').value)

        self.place_position_xyz = self._list_parameter('place_position_xyz')
        if len(self.place_position_xyz) != 3:
            raise ValueError('place_position_xyz must contain [x, y, z]')
        self.place_orientation_xyzw = self._list_parameter('place_orientation_xyzw')
        if len(self.place_orientation_xyzw) != 4:
            raise ValueError('place_orientation_xyzw must contain [x, y, z, w]')
        self.place_duration = float(self.get_parameter('place_duration').value)

        self.place_release_gripper_position = float(
            self.get_parameter('place_release_gripper_position').value
        )

        self.place_retreat_position_xyz = self._list_parameter('place_retreat_position_xyz')
        if len(self.place_retreat_position_xyz) != 3:
            raise ValueError('place_retreat_position_xyz must contain [x, y, z]')
        self.place_retreat_orientation_xyzw = self._list_parameter(
            'place_retreat_orientation_xyzw'
        )
        if len(self.place_retreat_orientation_xyzw) != 4:
            raise ValueError('place_retreat_orientation_xyzw must contain [x, y, z, w]')
        self.place_retreat_duration = float(self.get_parameter('place_retreat_duration').value)

        self.place_push_position_xyz = self._list_parameter('place_push_position_xyz')
        if len(self.place_push_position_xyz) != 3:
            raise ValueError('place_push_position_xyz must contain [x, y, z]')
        self.place_push_orientation_xyzw = self._list_parameter('place_push_orientation_xyzw')
        if len(self.place_push_orientation_xyzw) != 4:
            raise ValueError('place_push_orientation_xyzw must contain [x, y, z, w]')
        self.place_push_duration = float(self.get_parameter('place_push_duration').value)

        self.return_to_initial = self._bool_parameter('return_to_initial')

        home_position_xyz = self._list_parameter('home_position_xyz')
        if len(home_position_xyz) != 3:
            raise ValueError('home_position_xyz must contain [x, y, z]')
        self.home_position_xyz = home_position_xyz
        home_orientation_xyzw = self._list_parameter('home_orientation_xyzw')
        if len(home_orientation_xyzw) != 4:
            raise ValueError('home_orientation_xyzw must contain [x, y, z, w]')
        self.home_orientation_xyzw = home_orientation_xyzw
        self.home_duration = float(self.get_parameter('home_duration').value)

        self._setup_common()

        self.get_logger().info('CenterposeBox (left arm, roll follows box yaw) ready')
        self.get_logger().info(f'  MoveL: {self.movel_topic}')
        self.get_logger().info(f'  detections: {self.detections_topic}')
        self.get_logger().info(f'  camera info: {self.camera_info_topic}')
        self.get_logger().info(f'  depth (x/y only): {self.depth_topic}')

    def _format_capture_line(self, item):
        # Override to also show box yaw (base class only shows position).
        p = item['pose'].pose.position
        return f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), yaw={item["box_yaw_deg"]:.1f} deg'

    # --- Detection -> pose conversion ---
    def _process_single_detection(
        self, detection, camera_info, depth_image, depth_msg,
        camera_transform, fixed_z, log, index
    ):
        # Convert one CenterPose detection into a grasp pose in base_link.
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
        point, orientation, object_yaw, roll = transformed

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.target_frame
        # grasp_position_y_reference_pixel is the single shared reference pixel for
        # both slopes, despite the name.
        x_offset = self.grasp_position_offset[0] + self.grasp_position_x_slope * (
            u - self.grasp_position_y_reference_pixel
        )
        y_offset = self.grasp_position_offset[1] + self.grasp_position_y_slope * (
            u - self.grasp_position_y_reference_pixel
        )
        pose.pose.position.x = float(point[0] + x_offset)
        pose.pose.position.y = float(point[1] + y_offset)
        pose.pose.position.z = fixed_z + self.grasp_position_offset[2]
        pose.pose.orientation = self._quaternion_message(orientation)

        self.object_pose_pub.publish(pose)
        self.grasp_pose_pub.publish(pose)

        box_yaw_deg = self._wrap_degrees(
            math.degrees(object_yaw) - self.box_yaw_zero_offset_deg
        )

        if log:
            self.get_logger().info(
                f'[{index}] pixel=({u:.0f}, {v:.0f})/{camera_info.width}, '
                f'real camera xyz=({real_point_cam[0]:.3f}, {real_point_cam[1]:.3f}, '
                f'{real_point_cam[2]:.3f}), fixed z={fixed_z:.3f}, target=('
                f'{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, '
                f'{pose.pose.position.z:.3f}), '
                f'box_yaw={box_yaw_deg:.1f} deg (0=robot front) -> '
                f'roll={math.degrees(roll):.1f} deg'
            )
        return {'pose': pose, 'pixel_u': u, 'box_yaw_deg': box_yaw_deg}

    def _real_camera_point(self, camera_info, u, v, depth_image, depth_msg, log):
        # Back-project pixel (u, v) + sampled depth into a camera-frame 3D point.
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

        depth += self.box_depth_center_offset

        return np.array(
            [(u - cx) * depth / fx, (v - cy) * depth / fy, depth], dtype=np.float64
        )

    def _execute_queue(self, queue):
        # Pick and place every captured box in order.
        with self.execution_lock:
            try:
                for index, item in enumerate(queue):
                    if self.cancel_event.is_set():
                        return
                    label = f'box {index + 1}/{len(queue)}'
                    if not self._pick_and_place(item['pose'], label):
                        if self.cancel_event.is_set():
                            return
                        self.get_logger().error(f'{label} failed; continuing with next box')
                        continue
                self.get_logger().info(f'Finished picking {len(queue)} box(es)')
            finally:
                self.execution_step = 'idle'

    # --- Pick/place motion ---
    def _pick_and_place(self, grasp_pose, label='box'):
        # Run the full pick (insert->close->lift) then place (hover->release->retreat->push) sequence.
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

        pregrasp_pose = self._copy_pose(insert_pose)
        pregrasp_pose.pose.position.x -= approach_dir[0] * self.pregrasp_distance
        pregrasp_pose.pose.position.y -= approach_dir[1] * self.pregrasp_distance
        pregrasp_pose.pose.position.z -= approach_dir[2] * self.pregrasp_distance

        lift_pose = self._copy_pose(insert_pose)
        lift_pose.pose.position.z += self.lift_height

        place_hover_pose = PoseStamped()
        place_hover_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(
            place_hover_pose, self.place_hover_position_xyz, self.place_hover_orientation_xyzw
        )

        place_pose = PoseStamped()
        place_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(
            place_pose, self.place_position_xyz, self.place_orientation_xyzw
        )

        place_retreat_pose = PoseStamped()
        place_retreat_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(
            place_retreat_pose, self.place_retreat_position_xyz,
            self.place_retreat_orientation_xyzw
        )

        place_push_pose = PoseStamped()
        place_push_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(
            place_push_pose, self.place_push_position_xyz, self.place_push_orientation_xyzw
        )

        home_pose = PoseStamped()
        home_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(home_pose, self.home_position_xyz, self.home_orientation_xyzw)

        steps = [
            ('open gripper', lambda: self._move_gripper(self.gripper_open_position)),
            ('move to pregrasp', lambda: self._move_l(pregrasp_pose, duration=self.pregrasp_duration)),
            ('insert to box', lambda: self._move_l(insert_pose, duration=self.insertion_duration)),
            ('close gripper', lambda: self._move_gripper(self.gripper_closed_position)),
            ('lift box', lambda: self._move_l(lift_pose, duration=self.lift_duration)),
            ('move to place hover', lambda: self._move_l(place_hover_pose, duration=self.place_hover_duration)),
            ('move to place', lambda: self._move_l(place_pose, duration=self.place_duration)),
            ('release box', lambda: self._move_gripper(self.gripper_open_position)),
            ('move to place retreat', lambda: self._move_l(place_retreat_pose, duration=self.place_retreat_duration)),
            ('reclose gripper for push', lambda: self._move_gripper(self.place_release_gripper_position)),
            ('push box into place', lambda: self._move_l(place_push_pose, duration=self.place_push_duration)),
        ]
        if self.return_to_initial:
            steps.append(
                ('return to initial pose', lambda: self._move_l(home_pose, duration=self.home_duration))
            )
            steps.append(
                ('open gripper', lambda: self._move_gripper(self.gripper_open_position))
            )

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
        # Convert a camera-frame point+orientation into a base_link grasp pose
        # (gripper roll follows the box's yaw; pitch/yaw stay fixed).
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

        object_yaw, raw_angle = self._signed_box_yaw(object_q)
        if math.degrees(raw_angle) > self.box_yaw_flip_threshold_deg:
            object_q = self._quaternion_multiply(object_q, self._LOCAL_Y_180_FLIP)
            object_yaw, _ = self._signed_box_yaw(object_q)

        raw_roll_deg = math.degrees(
            self.grasp_roll_from_yaw_scale * object_yaw + self.grasp_roll_offset
        )
        clamped_roll_deg = min(
            max(raw_roll_deg, self.roll_clamp_min_deg), self.roll_clamp_max_deg
        )
        roll = self._wrap_angle(math.radians(clamped_roll_deg))
        base_q = self._quaternion_from_euler(roll, self.grasp_fixed_pitch, self.grasp_fixed_yaw)
        target_q = self._quaternion_multiply(base_q, self.tool_orientation_offset)
        return point, target_q, object_yaw, roll

    def _signed_box_yaw(self, object_q):
        # Signed angle of the box's orientation relative to the calibrated reference.
        relative_q = self._quaternion_multiply(
            self._quaternion_conjugate(self.box_yaw_reference_orientation), object_q
        )
        axis, angle = self._quaternion_axis_angle(relative_q)
        sign = 1.0 if np.dot(axis, self.box_yaw_axis) >= 0.0 else -1.0
        return sign * angle, angle

    @staticmethod
    def _wrap_degrees(degrees):
        # Normalize an angle in degrees to [-180, 180].
        return (degrees + 180.0) % 360.0 - 180.0


def main(args=None):
    rclpy.init(args=args)
    node = CenterposeBox()
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
