#!/usr/bin/env python3
"""
Directly enables torque and sets Goal Position via DynamixelHardware services.
For ffw_lg2_mini_leader (right arm: dxl1~7, XL330, 4096 counts/rev).

Usage: python3 test_initial_pose_publisher.py
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from dynamixel_interfaces.srv import SetDataToDxl
import math

# XL330: 4096 counts per revolution
RAD_TO_RAW = 4096.0 / (2.0 * math.pi)

# Motor ID -> transmission factor (from transmission_to_joint_matrix diagonal)
# joint_rad = factor * motor_rad, so motor_rad = joint_rad / factor
MOTORS = {
    # motor_id: (joint_name, transmission_factor)
    1: ('arm_r_joint1', 1.0),
    2: ('arm_r_joint2', 1.0),
    3: ('arm_r_joint3', 1.0),
    4: ('arm_r_joint4', 1.0),
    5: ('arm_r_joint5', 1.0),
    6: ('arm_r_joint6', 2.0),
    7: ('arm_r_joint7', 1.0),
}

# Target joint positions in radians
TARGET = [0.75, 0.0, 0.0, -2.3, 0.0, 0.0, 0.0]

SVC_NS = '/leader/dynamixel_hardware_interface'


class TestDirectPose(Node):
    def __init__(self):
        super().__init__('test_direct_pose')

        self.torque_cli = self.create_client(SetBool, f'{SVC_NS}/set_dxl_torque')
        self.data_cli = self.create_client(SetDataToDxl, f'{SVC_NS}/set_dxl_data')

        self.get_logger().info('Waiting for services...')
        self.torque_cli.wait_for_service(timeout_sec=5.0)
        self.data_cli.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('Services ready.')

        self.enable_torque()
        self.set_goal_positions()

    def enable_torque(self):
        req = SetBool.Request()
        req.data = True
        future = self.torque_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.get_logger().info(f'Torque enable: {future.result()}')

    def set_goal_positions(self):
        for i, (motor_id, (joint_name, factor)) in enumerate(MOTORS.items()):
            joint_rad = TARGET[i]
            motor_rad = joint_rad / factor
            raw = int(motor_rad * RAD_TO_RAW)

            req = SetDataToDxl.Request()
            req.id = motor_id
            req.item_name = 'Goal Position'
            req.item_data = raw

            future = self.data_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            self.get_logger().info(
                f'ID={motor_id} ({joint_name}): {joint_rad:.2f} rad -> raw={raw}')


def main():
    rclpy.init()
    node = TestDirectPose()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
