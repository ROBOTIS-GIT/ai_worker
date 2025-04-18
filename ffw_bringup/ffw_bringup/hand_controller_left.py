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

from ament_index_python.packages import get_package_share_directory
from ffw_hand_library.library import InspireHand
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from trajectory_msgs.msg import JointTrajectory
import yaml


class LeaderFollowerHand(Node):

    def __init__(self):
        super().__init__('leader_follower_left_hand')

        self.declare_parameter('serial_port', '/dev/left_hand')
        self.declare_parameter('hand_id', 2)
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.hand_id = self.get_parameter('hand_id').get_parameter_value().integer_value

        self.hand = InspireHand(self.serial_port, self.hand_id)
        self.get_logger().info(f'InspireHand connected: {self.serial_port}, ID {self.hand_id}')

        self.sub = self.create_subscription(
            JointTrajectory,
            '/leader/joint_trajectory_left_hand/joint_trajectory',
            self.leader_callback, 10)

        self.hand_pub = self.create_publisher(Int32MultiArray, '/follower/left_hand_angles', 10)

        self.joint_range = self.load_joint_range()
        self.joint_map = {
            'left_little_1_joint': 0,
            'left_ring_1_joint': 1,
            'left_middle_1_joint': 2,
            'left_index_1_joint': 3,
            'left_thumb_2_joint': 4,
            'left_thumb_1_joint': 5,
        }

    def get_src_config_path(self, filename):
        install_path = get_package_share_directory('ffw_bringup')
        src_bringup_path = install_path.replace(
            '/install/ffw_bringup/share/ffw_bringup',
            '/src/ai_worker/ffw_bringup'
        )
        return os.path.join(src_bringup_path, 'config', filename)

    def load_joint_range(self):
        self.output_file = self.get_src_config_path('hand_joint_range_left.yaml')

        if not os.path.exists(self.output_file):
            self.get_logger().warn('joint_range YAML file not found! Using default values')
            return {'min': [0.0] * 6, 'max': [6.28] * 6}

        with open(self.output_file, 'r') as f:
            data = yaml.safe_load(f)
        self.get_logger().info('Loaded joint_range.yaml')
        return data

    def scale(self, val, index):
        rad_min = self.joint_range['min'][index]
        rad_max = self.joint_range['max'][index]

        if rad_max - rad_min == 0:
            self.get_logger().warn(
                f'Index {index} has min and max equal: {rad_min}. Returning default value 0')
            return 0

        val = max(min(val, rad_max), rad_min)
        norm = (val - rad_min) / (rad_max - rad_min)
        scaled = int(norm * 1000)
        return 1000 - scaled

    def leader_callback(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn('Trajectory message has no points')
            return

        joint_names = msg.joint_names
        positions = msg.points[0].positions
        name_to_position = dict(zip(joint_names, positions))

        scaled = [0] * 6
        for joint_name, target_index in self.joint_map.items():
            val = name_to_position.get(joint_name, 0.0)
            scaled[target_index] = self.scale(val, target_index)

        self.hand.setangle(*scaled)

        angles = self.hand.get_actangle()
        self.hand_pub.publish(Int32MultiArray(data=angles))

    def destroy_node(self):
        self.hand.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollowerHand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
