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
# Authors: Wonho Yun

import tkinter as tk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class KeyboardController(Node):

    def __init__(self):
        super().__init__('keyboard_joint_controller')

        self.controllers = {
            'arm_l': {
                'joints': [
                    'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
                    'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7', 'l_rh_r1_joint'
                ],
                'positions': [0.0] * 8,
                'labels': [None] * 8,
                'limits': [(-3.14, 3.14)] * 8,
                'position_step': [0.1] * 8,
                'publisher': self.create_publisher(
                    JointTrajectory, '/leader/joint_trajectory_left/joint_trajectory', 10)
            },
            'arm_r': {
                'joints': [
                    'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
                    'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7', 'r_rh_r1_joint'
                ],
                'positions': [0.0] * 8,
                'labels': [None] * 8,
                'limits': [(-3.14, 3.14)] * 8,
                'position_step': [0.1] * 8,
                'publisher': self.create_publisher(
                    JointTrajectory, '/leader/joint_trajectory_right/joint_trajectory', 10)
            },
            'neck': {
                'joints': ['neck_joint1', 'neck_joint2'],
                'positions': [0.0] * 2,
                'labels': [None] * 2,
                'limits': [(-1.0, 1.0)] * 2,
                'position_step': [0.1] * 2,
                'publisher': self.create_publisher(
                    JointTrajectory, '/neck_controller/joint_trajectory', 10)
            },
            'body': {
                'joints': ['linear_joint'],
                'positions': [0.0],
                'labels': [None],
                'limits': [(-1.0, 0.0)],
                'position_step': [0.1],
                'publisher': self.create_publisher(
                    JointTrajectory, '/body_controller/joint_trajectory', 10)
            }
        }

        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.joint_received = False
        self.running = True

        self.root = tk.Tk()
        self.root.title('Joint Controller GUI')
        self.hold_buttons = set()
        self.build_gui()
        self.root.after(100, self.process_held_buttons)

    def joint_state_callback(self, msg):
        for ctrl_key, ctrl in self.controllers.items():
            for i, joint in enumerate(ctrl['joints']):
                if joint in msg.name:
                    # if joint == 'arm_r_joint7' or joint == 'arm_l_joint7':
                    #     continue
                    idx = msg.name.index(joint)
                    ctrl['positions'][i] = msg.position[idx]
                    if ctrl['labels'][i]:
                        value = msg.position[idx]
                        ctrl['labels'][i].config(text=f'{value:.2f}')
        self.joint_received = True

    def send_command(self, ctrl_key):
        ctrl = self.controllers[ctrl_key]
        msg = JointTrajectory()
        msg.joint_names = ctrl['joints']
        point = JointTrajectoryPoint()
        point.positions = ctrl['positions']
        point.time_from_start.sec = 0
        msg.points.append(point)
        ctrl['publisher'].publish(msg)
        self.get_logger().info(f"{ctrl_key} command: {ctrl['positions']}")

    def change_joint(self, ctrl_key, joint_index, direction):
        ctrl = self.controllers[ctrl_key]
        min_limit, max_limit = ctrl['limits'][joint_index]
        delta = direction * ctrl['position_step'][joint_index]
        before = ctrl['positions'][joint_index]
        new_pos = before + delta
        clamped_pos = max(min(new_pos, max_limit), min_limit)
        ctrl['positions'][joint_index] = clamped_pos

        if ctrl['labels'][joint_index]:
            ctrl['labels'][joint_index].config(text=f'{clamped_pos:.2f}')

        joint_name = ctrl['joints'][joint_index]
        self.get_logger().info(
            f'Joint [{ctrl_key}/{joint_name}]: {before:.3f} → {clamped_pos:.3f} (delta={delta:.3f}'
        )
        self.send_command(ctrl_key)

    def press_and_hold(self, ctrl_key, joint_index, direction):
        self.hold_buttons.add((ctrl_key, joint_index, direction))

    def release_button(self, ctrl_key, joint_index, direction):
        self.hold_buttons.discard((ctrl_key, joint_index, direction))

    def process_held_buttons(self):
        for ctrl_key, joint_index, direction in list(self.hold_buttons):
            self.change_joint(ctrl_key, joint_index, direction)
        self.root.after(100, self.process_held_buttons)

    def build_gui(self):
        row = 0
        for ctrl_key, ctrl in self.controllers.items():
            tk.Label(
                self.root,
                text=ctrl_key.upper(),
                font=('Arial', 12, 'bold')).grid(row=row, column=0, columnspan=4)
            row += 1
            for i, joint in enumerate(ctrl['joints']):
                tk.Label(self.root, text=joint).grid(row=row, column=0)
                btn_minus = tk.Button(self.root, text='-', width=3)
                btn_plus = tk.Button(self.root, text='+', width=3)
                label = tk.Label(self.root, text=f"{ctrl['positions'][i]:.2f}", width=6)
                ctrl['labels'][i] = label
                btn_minus.grid(row=row, column=1)
                btn_plus.grid(row=row, column=2)
                label.grid(row=row, column=3)
                btn_minus.bind(
                    '<ButtonPress-1>', lambda e, c=ctrl_key, j=i: self.press_and_hold(c, j, -1))
                btn_minus.bind(
                    '<ButtonRelease-1>', lambda e, c=ctrl_key, j=i: self.release_button(c, j, -1))
                btn_plus.bind(
                    '<ButtonPress-1>', lambda e, c=ctrl_key, j=i: self.press_and_hold(c, j, +1))
                btn_plus.bind(
                    '<ButtonRelease-1>', lambda e, c=ctrl_key, j=i: self.release_button(c, j, +1))
                row += 1

    def run(self):
        while not self.joint_received and rclpy.ok() and self.running:
            self.get_logger().info('Waiting for /joint_states...')
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info('GUI control ready. Close the window to exit.')
        try:
            while rclpy.ok() and self.running:
                rclpy.spin_once(self, timeout_sec=0.01)
                self.root.update()
        except tk.TclError:
            self.running = False


def main():
    rclpy.init()
    node = KeyboardController()
    try:
        node.run()
    except KeyboardInterrupt:
        node.running = False
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
