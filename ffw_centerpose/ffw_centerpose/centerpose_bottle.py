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


class CenterposeBottle(PickPlaceNodeBase):
    _OBJECT_LABEL_PLURAL = 'bottle(s)'

    def __init__(self):
        super().__init__('centerpose_bottle')

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
        self.declare_parameter('pregrasp_distance', 0.08)
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
        self.movel_duration = 5.0
        self.pregrasp_distance = float(self.get_parameter('pregrasp_distance').value)
        self.pregrasp_duration = 5.0
        self.insertion_duration = 1.0
        self.left_arm_joint_trajectory_topic = self.get_parameter(
            'left_arm_joint_trajectory_topic'
        ).value
        self.return_to_initial = self._bool_parameter('return_to_initial')

        # --- Fixed gripper/motion calibration ---
        self.gripper_open_position = 0.0
        self.gripper_closed_position = 0.7
        self.lift_height = 0.1
        self.box_duration = 5.0
        self.box_place_duration = 2.0
        self.home_duration = 4.0

        # --- Measured calibration ---
        calib = self._load_calibration('centerpose_bottle_calibration.yaml')
        self.fixed_grasp_z = float(calib['fixed_grasp_z'])
        self.grasp_position_offset = calib['grasp_position_offset']
        self.grasp_orientation = self._normalized(calib['grasp_orientation_xyzw'])
        self.approach_dir = self._rotate_vector(
            np.array([0.0, 0.0, -1.0]), self.grasp_orientation
        )
        self.box_slots = [
            {
                'position': calib[f'box_slot_{index}_position_xyz'],
                'orientation': self._normalized(calib[f'box_slot_{index}_orientation_xyzw']),
            }
            for index in (1, 2)
        ]
        self.box_place_z_offset = float(calib['box_place_z_offset'])
        self.home_position_xyz = calib['home_position_xyz']

        self._setup_common()

        self.get_logger().info('CenterposeBottle (left arm, fixed orientation) ready')
        self.get_logger().info(f'  MoveL: {self.movel_topic}')
        self.get_logger().info(f'  detections: {self.detections_topic}')
        self.get_logger().info(f'  camera info: {self.camera_info_topic}')
        self.get_logger().info(f'  depth (x/y only): {self.depth_topic}')
        for index, slot in enumerate(self.box_slots, start=1):
            self.get_logger().info(f'  box slot {index} hover: {slot["position"]}')

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

        transform_q, t = camera_transform
        point = self._rotate_vector(np.asarray(real_point_cam, dtype=np.float64), transform_q) + t

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.target_frame
        pose.pose.position.x = float(point[0] + self.grasp_position_offset[0])
        pose.pose.position.y = float(point[1] + self.grasp_position_offset[1])
        pose.pose.position.z = fixed_z + self.grasp_position_offset[2]
        pose.pose.orientation = self._quaternion_message(self.grasp_orientation)

        self.object_pose_pub.publish(pose)
        self.grasp_pose_pub.publish(pose)

        if log:
            self.get_logger().info(
                f'[{index}] pixel=({u:.0f}, {v:.0f})/{camera_info.width}, '
                f'real camera xyz=({real_point_cam[0]:.3f}, {real_point_cam[1]:.3f}, '
                f'{real_point_cam[2]:.3f}), fixed z={fixed_z:.3f}, target=('
                f'{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, '
                f'{pose.pose.position.z:.3f})'
            )
        return {'pose': pose, 'pixel_u': u}

    def _execute_queue(self, queue):
        # Pick and place every captured bottle in order.
        with self.execution_lock:
            try:
                for index, item in enumerate(queue):
                    if self.cancel_event.is_set():
                        return
                    label = f'bottle {index + 1}/{len(queue)}'
                    slot = self.box_slots[min(index, len(self.box_slots) - 1)]
                    if not self._pick_and_place(item['pose'], slot, label):
                        if self.cancel_event.is_set():
                            return
                        self.get_logger().error(f'{label} failed; continuing with next bottle')
                        continue
                self.get_logger().info(f'Finished picking {len(queue)} bottle(s)')
            finally:
                self.execution_step = 'idle'

    # --- Pick/place motion ---
    def _pick_and_place(self, grasp_pose, slot, label='bottle'):
        # Run the full pick (insert->close->lift) then place (hover->lower->release) sequence.
        pregrasp_pose = self._copy_pose(grasp_pose)
        pregrasp_pose.pose.position.x -= self.approach_dir[0] * self.pregrasp_distance
        pregrasp_pose.pose.position.y -= self.approach_dir[1] * self.pregrasp_distance
        pregrasp_pose.pose.position.z -= self.approach_dir[2] * self.pregrasp_distance

        lift_pose = self._copy_pose(grasp_pose)
        lift_pose.pose.position.z += self.lift_height

        box_pose = PoseStamped()
        box_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(box_pose, slot['position'], slot['orientation'])

        box_place_pose = self._copy_pose(box_pose)
        box_place_pose.pose.position.z -= self.box_place_z_offset

        home_pose = PoseStamped()
        home_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(home_pose, self.home_position_xyz, self.grasp_orientation)

        steps = [
            ('open gripper', lambda: self._move_gripper(self.gripper_open_position)),
            ('move to pregrasp', lambda: self._move_l(pregrasp_pose, duration=self.pregrasp_duration)),
            ('insert to bottle', lambda: self._move_l(grasp_pose, duration=self.insertion_duration)),
            ('close gripper', lambda: self._move_gripper(self.gripper_closed_position)),
            ('lift bottle', lambda: self._move_l(lift_pose, duration=self.lift_duration)),
            ('move above box', lambda: self._move_l(box_pose, duration=self.box_duration)),
            ('lower into box', lambda: self._move_l(box_place_pose, duration=self.box_place_duration)),
            ('release in box', lambda: self._move_gripper(self.gripper_open_position)),
            ('raise back above box', lambda: self._move_l(box_pose, duration=self.box_place_duration)),
        ]
        if self.return_to_initial:
            steps.append(
                ('return to initial pose', lambda: self._move_l(home_pose, duration=self.home_duration))
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


def main(args=None):
    rclpy.init(args=args)
    node = CenterposeBottle()
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
