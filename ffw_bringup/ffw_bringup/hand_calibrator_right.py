#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
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
# Authors: Sungho Woo, Woojin Wie, Wonho Yun

import os
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import yaml


class HandCalibrator(Node):

    def __init__(self):
        super().__init__('hand_calibrator_right')
        self.sub = self.create_subscription(
            JointTrajectory,
            '/leader/joint_trajectory_right_hand/joint_trajectory',
            self.callback, 10)

        self.target_order = [
            'right_little_1_joint',
            'right_ring_1_joint',
            'right_middle_1_joint',
            'right_index_1_joint',
            'right_thumb_2_joint',
            'right_thumb_1_joint',
        ]

        self.joint_min = None
        self.joint_max = None
        self.sample_count = 0
        self.max_samples = 1000
        self.done = False

        install_path = get_package_share_directory('ffw_bringup')
        src_bringup_path = install_path.replace(
            '/install/ffw_bringup/share/ffw_bringup', '/src/ai_worker/ffw_bringup')
        self.output_file = os.path.join(src_bringup_path, 'config', 'hand_joint_range_right.yaml')
        time.sleep(0.5)
        self.get_logger().info('Starting right hand calibration... (Move your hand!)')

    def callback(self, msg: JointTrajectory):
        if self.done or not msg.points:
            return

        joint_dict = dict(zip(msg.joint_names, msg.points[0].positions))
        ordered_pos = [joint_dict.get(name, 0.0) for name in self.target_order]

        if self.joint_min is None:
            self.joint_min = list(ordered_pos)
            self.joint_max = list(ordered_pos)
        else:
            self.joint_min = [min(a, b) for a, b in zip(self.joint_min, ordered_pos)]
            self.joint_max = [max(a, b) for a, b in zip(self.joint_max, ordered_pos)]

        self.sample_count += 1
        if self.sample_count >= self.max_samples:
            self.save()
            self.done = True
            self.get_logger().info('Right hand sampling complete. Shutting down the node.')

    def save(self):
        data = {'min': self.joint_min, 'max': self.joint_max}
        with open(self.output_file, 'w') as f:
            yaml.dump(data, f)
        self.get_logger().info(f'Calibration complete! Saved to: {self.output_file}')


def main(args=None):
    rclpy.init(args=args)
    node = HandCalibrator()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
