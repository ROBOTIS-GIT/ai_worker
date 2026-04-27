#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

TOPIC = '/robot/arm_right_follower/joint_states'
JOINT_NAME = 'arm_r_joint6'
# THRESHOLD = -0.7
# THRESHOLD = 3.5
THRESHOLD = 30.0
TORQUE_SERVICE = '/dynamixel_hardware_interface/set_dxl_torque'
WAITING_TIME = 10


class FinishMonitor(Node):

    def __init__(self):
        super().__init__('finish_monitor')
        self.armed = False
        self.triggered = False
        self.cached_idx = -1

        self.torque_client = self.create_client(SetBool, TORQUE_SERVICE)
        if not self.torque_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f'{TORQUE_SERVICE} not available yet; will still try when triggered'
            )

        self.subscription = self.create_subscription(
            JointState,
            TOPIC,
            self.joint_state_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'finish_monitor started, watching {JOINT_NAME} on {TOPIC} '
            f'(must rise above {THRESHOLD} first, then drop below to trigger)'
        )

    def joint_state_callback(self, msg):
        if self.triggered:
            return
        if not msg.velocity:
            return

        max_abs_vel = 0.0
        max_name = ''
        for i, v in enumerate(msg.velocity):
            av = abs(v)
            if av > max_abs_vel:
                max_abs_vel = av
                max_name = msg.name[i] if i < len(msg.name) else f'idx{i}'

        if not self.armed:
            if max_abs_vel > THRESHOLD:
                self.armed = True
                self.get_logger().info(
                    f'{max_name}={max_abs_vel:.4f} > {THRESHOLD}, armed'
                )
            return

        if max_abs_vel < THRESHOLD:
            self.triggered = True
            self.get_logger().info(
                f'all joints below {THRESHOLD} (max={max_abs_vel:.4f} @ {max_name}), '
                f'disabling follower torque'
            )
            self.disable_torque()

    def disable_torque(self):
        req = SetBool.Request()
        req.data = False
        future = self.torque_client.call_async(req)
        future.add_done_callback(self.torque_done_callback)

    def torque_done_callback(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'set_dxl_torque call failed: {exc}')
        else:
            if result is None:
                self.get_logger().error('set_dxl_torque returned no result')
            elif result.success:
                self.get_logger().info(f'follower torque disabled: {result.message}')
            else:
                self.get_logger().error(
                    f'set_dxl_torque rejected: {result.message}'
                )
        # Exit the process so the launch's OnProcessExit handler shuts down everything
        rclpy.shutdown()


def main(args=None):
    time.sleep(WAITING_TIME)
    rclpy.init(args=args)
    node = FinishMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
