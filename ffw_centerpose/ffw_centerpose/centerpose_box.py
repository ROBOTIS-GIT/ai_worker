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

from ffw_centerpose.pick_place_base import load_camera_topics, PickPlaceNodeBase


class CenterposeBox(PickPlaceNodeBase):
    _OBJECT_LABEL_PLURAL = 'box(es)'

    _LOCAL_Y_180_FLIP = np.array([0.0, 1.0, 0.0, 0.0])

    def __init__(self):
        super().__init__('centerpose_box')

        camera = load_camera_topics()

        # --- Parameters ---
        self.declare_parameter('detections_topic', '/centerpose/detections')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('depth_topic', camera['depth'])
        self.declare_parameter('depth_window', 5)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('projection_frame', '')
        self.declare_parameter('detection_timeout', 10.0)
        self.declare_parameter('max_grasp_x', 0.7)
        self.declare_parameter('execute_motion', False)
        self.declare_parameter('movel_topic', '/l_goal_move')
        self.declare_parameter('pregrasp_distance', 0.11)
        self.declare_parameter(
            'left_arm_joint_trajectory_topic',
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
        )
        self.declare_parameter('return_to_initial', True)

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
        self.movel_duration = 10.0
        self.pregrasp_distance = float(self.get_parameter('pregrasp_distance').value)
        self.pregrasp_duration = 4.0
        self.insertion_duration = 3.0
        self.left_arm_joint_trajectory_topic = self.get_parameter(
            'left_arm_joint_trajectory_topic'
        ).value
        self.return_to_initial = self._bool_parameter('return_to_initial')

        # --- Fixed gripper/motion calibration ---
        self.gripper_open_position = 0.0
        self.gripper_closed_position = 0.57
        self.lift_height = 0.1
        self.place_release_gripper_position = 0.9
        self.place_hover_duration = 4.0
        self.place_duration = 6.0
        self.place_retreat_duration = 2.0
        self.place_push_duration = 2.0
        self.home_duration = 6.0

        # --- Measured calibration ---
        calib = self._load_calibration('centerpose_box_calibration.yaml')

        # Grasp position offset/slope
        self.fixed_grasp_z = float(calib['fixed_grasp_z'])
        self.insertion_overshoot_distance = float(calib['insertion_overshoot_distance'])
        self.grasp_position_offset = calib['grasp_position_offset']
        self.grasp_position_y_slope = float(calib['grasp_position_y_slope'])
        self.grasp_position_y_reference_pixel = float(calib['grasp_position_y_reference_pixel'])
        self.grasp_position_x_slope = float(calib['grasp_position_x_slope'])
        self.box_depth_center_offset = float(calib['box_depth_center_offset'])
        self._depth_center_offset = self.box_depth_center_offset

        # Grasp orientation / yaw calibration
        self.tool_orientation_offset = self._normalized(calib['tool_orientation_offset_xyzw'])
        self.box_yaw_reference_orientation = self._normalized(
            calib['box_yaw_reference_orientation_xyzw']
        )
        self.box_yaw_axis = self._normalized(calib['box_yaw_axis_xyz'])
        self.box_yaw_flip_threshold_deg = float(calib['box_yaw_flip_threshold_deg'])
        self.grasp_fixed_pitch = float(calib['grasp_fixed_pitch'])
        self.grasp_fixed_yaw = float(calib['grasp_fixed_yaw'])
        self.grasp_roll_from_yaw_scale = float(calib['grasp_roll_from_yaw_scale'])
        self.grasp_roll_offset = float(calib['grasp_roll_offset'])
        self.box_yaw_zero_offset_deg = float(calib['box_yaw_zero_offset_deg'])
        self.roll_clamp_min_deg = float(calib['roll_clamp_min_deg'])
        self.roll_clamp_max_deg = float(calib['roll_clamp_max_deg'])

        # Place / home poses
        self.place_hover_position_xyz = calib['place_hover_position_xyz']
        self.place_hover_orientation_xyzw = calib['place_hover_orientation_xyzw']
        self.place_position_xyz = calib['place_position_xyz']
        self.place_orientation_xyzw = calib['place_orientation_xyzw']
        self.place_retreat_position_xyz = calib['place_retreat_position_xyz']
        self.place_retreat_orientation_xyzw = calib['place_retreat_orientation_xyzw']
        self.place_push_position_xyz = calib['place_push_position_xyz']
        self.place_push_orientation_xyzw = calib['place_push_orientation_xyzw']
        self.home_position_xyz = calib['home_position_xyz']
        self.home_orientation_xyzw = calib['home_orientation_xyzw']

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
