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
import threading

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

from ffw_centerpose.pick_place_base import load_camera_topics, PickPlaceNodeBase


class CenterposeShoe(PickPlaceNodeBase):
    # DRAFT: uncalibrated placeholders below.
    _OBJECT_LABEL_PLURAL = 'shoe(s)'
    _LOCAL_Y_180_FLIP = np.array([0.0, 1.0, 0.0, 0.0])

    def __init__(self):
        super().__init__('centerpose_shoe')

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
        self.declare_parameter('max_grasp_x', 0.9)
        self.declare_parameter('execute_motion', False)
        self.declare_parameter('movel_topic', '/l_goal_move')
        self.declare_parameter('pregrasp_distance', 0.1)
        self.declare_parameter(
            'left_arm_joint_trajectory_topic',
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
        )

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
        self.pregrasp_distance = float(self.get_parameter('pregrasp_distance').value)
        self.left_arm_joint_trajectory_topic = self.get_parameter(
            'left_arm_joint_trajectory_topic'
        ).value

        # --- Fixed gripper/motion calibration ---
        self.movel_duration = 10.0
        self.startup_duration = 10.0
        self.start_duration = 10.0
        self.return_duration = 6.0
        self.pregrasp_duration = 4.0
        self.insertion_duration = 2.0
        self.gripper_open_position = 0.0
        self.gripper_closed_position = 1.1
        self.gripper_release_position = 0.7
        self.lift_height = 0.1
        self.place_hover_duration = 4.0
        self.place_lower_duration = 2.0
        self.place_duration = 1.0
        self.yaw_clamp_min_deg = -90.0
        self.yaw_clamp_max_deg = 90.0

        # --- Measured calibration ---
        calib = self._load_calibration('centerpose_shoe_calibration.yaml')

        # Startup / start poses
        self.startup_position_xyz = calib['startup_position_xyz']
        self.startup_orientation_xyzw = calib['startup_orientation_xyzw']
        self.start_position_xyz = calib['start_position_xyz']
        self.start_orientation_xyzw = calib['start_orientation_xyzw']

        # Grasp position offset/slope
        self.insertion_overshoot_distance = float(calib['insertion_overshoot_distance'])
        self.fixed_grasp_z = float(calib['fixed_grasp_z'])
        self.min_grasp_z = float(calib['min_grasp_z'])
        self.grasp_position_offset = calib['grasp_position_offset']
        self.grasp_position_y_slope = float(calib['grasp_position_y_slope'])
        self.grasp_position_y_reference_pixel = float(calib['grasp_position_y_reference_pixel'])
        self.second_shoe_grasp_y_offset = float(calib['second_shoe_grasp_y_offset'])
        # TODO: shoe_depth_center_offset unmeasured.
        self.shoe_depth_center_offset = float(calib['shoe_depth_center_offset'])
        self._depth_center_offset = self.shoe_depth_center_offset

        # Grasp orientation / yaw calibration
        self.tool_orientation_offset = self._normalized(calib['tool_orientation_offset_xyzw'])
        self.shoe_yaw_reference_orientation = self._normalized(
            calib['shoe_yaw_reference_orientation_xyzw']
        )
        self.shoe_yaw_axis = self._normalized(calib['shoe_yaw_axis_xyz'])
        self.shoe_yaw_flip_threshold_deg = float(calib['shoe_yaw_flip_threshold_deg'])
        self.grasp_fixed_roll = float(calib['grasp_fixed_roll'])
        self.grasp_fixed_pitch = float(calib['grasp_fixed_pitch'])
        self.grasp_yaw_from_shoe_yaw_scale = float(calib['grasp_yaw_from_shoe_yaw_scale'])
        self.grasp_yaw_offset = float(calib['grasp_yaw_offset'])

        # Place poses
        self.place_slots = [
            {
                'position': calib[f'place_slot_{index}_position_xyz'],
                'orientation': self._normalized(calib[f'place_slot_{index}_orientation_xyzw']),
            }
            for index in (1, 2)
        ]
        self.place_lower_distance = float(calib['place_lower_distance'])

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
        # Move the arm through startup_pose -> start_pose once, at node startup.
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
        point, orientation, object_yaw, gripper_yaw = transformed

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.target_frame
        y_offset = self.grasp_position_offset[1] + self.grasp_position_y_slope * (
            u - self.grasp_position_y_reference_pixel
        )
        pose.pose.position.x = float(point[0] + self.grasp_position_offset[0])
        pose.pose.position.y = float(point[1] + y_offset)
        pose.pose.position.z = fixed_z + self.grasp_position_offset[2]
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

    def _execute_queue(self, queue):
        # Pick and place every captured shoe in order, then return to start pose.
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
        # Run the full pick (insert->close->lift) then place (hover->lower->release) sequence.
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
        # Convert a camera-frame point+orientation into a base_link grasp pose
        # (gripper yaw follows the shoe's yaw; roll/pitch stay fixed).
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
            object_q = self._quaternion_multiply(object_q, self._LOCAL_Y_180_FLIP)
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
        # Signed angle of the shoe's orientation relative to the calibrated reference.
        relative_q = self._quaternion_multiply(
            self._quaternion_conjugate(self.shoe_yaw_reference_orientation), object_q
        )
        axis, angle = self._quaternion_axis_angle(relative_q)
        sign = 1.0 if np.dot(axis, self.shoe_yaw_axis) >= 0.0 else -1.0
        return sign * angle, angle


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
