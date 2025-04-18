#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
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
# Author: Sungho Woo

import sys

from control_msgs.action import FollowJointTrajectory
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveToHome(Node):

    def __init__(self):
        super().__init__('move_to_home')

        # Declare parameter for including hand joints
        self.declare_parameter('include_hand', True)
        self.include_hand = self.get_parameter('include_hand').value

        # Define joint groups and their controllers
        arm_l_joints = ['arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3', 'arm_l_joint4',
                        'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7']
        arm_r_joints = ['arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3', 'arm_r_joint4',
                        'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7']

        if self.include_hand:
            arm_l_joints.append('l_rh_r1_joint')
            arm_r_joints.append('r_rh_r1_joint')

        self.joint_groups = {
            'arm_l_controller': arm_l_joints,
            'arm_r_controller': arm_r_joints,
            'neck_controller': ['neck_joint1', 'neck_joint2'],
            'body_controller': ['linear_joint']
        }

        # Initialize target positions
        self.target_positions = {
            'arm_l_joint1': 0.0, 'arm_l_joint2': 0.0, 'arm_l_joint3': 0.0,
            'arm_l_joint4': 0.0, 'arm_l_joint5': 0.0, 'arm_l_joint6': 0.0,
            'arm_l_joint7': 0.0, 'arm_r_joint1': 0.0, 'arm_r_joint2': 0.0,
            'arm_r_joint3': 0.0, 'arm_r_joint4': 0.0, 'arm_r_joint5': 0.0,
            'arm_r_joint6': 0.0, 'arm_r_joint7': 0.0, 'neck_joint1': 0.0,
            'neck_joint2': 0.0, 'linear_joint': 0.0
        }

        if self.include_hand:
            self.target_positions.update({
                'l_rh_r1_joint': 0.0,
                'r_rh_r1_joint': 0.0
            })

        # Create action clients for each controller
        self.action_clients = {}
        for controller in self.joint_groups.keys():
            self.action_clients[controller] = ActionClient(
                self,
                FollowJointTrajectory,
                f'/{controller}/follow_joint_trajectory'
            )

        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        self.current_positions = {}
        self.current_velocities = {}
        self.epsilon = 0.05  # Error threshold for motion completion
        self.num_points = 100  # Number of points for smooth trajectory
        self.duration = 5.0  # Duration for movement
        self.goal_handles = {}
        self.last_status_time = 0.0
        self.status_interval = 1.0  # Log status every second
        self.current_step = 0  # 0: move to initial positions, 1: move to final positions
        self.reached_target = False

        # Wait for all action servers
        for controller, client in self.action_clients.items():
            self.get_logger().info(f'Waiting for {controller} action server...')
            client.wait_for_server()
            self.get_logger().info(f'{controller} action server available')

    def create_smooth_trajectory(self, controller, start_pos, end_pos):
        traj = JointTrajectory()
        traj.joint_names = self.joint_groups[controller]

        times = np.linspace(0, self.duration, self.num_points)

        for i in range(self.num_points):
            point = JointTrajectoryPoint()
            t = times[i]

            # Quintic polynomial coefficients
            t_norm = t / self.duration
            t_norm2 = t_norm * t_norm
            t_norm3 = t_norm2 * t_norm
            t_norm4 = t_norm3 * t_norm
            t_norm5 = t_norm4 * t_norm

            # Quintic polynomial coefficients for position
            pos_coeff = 10 * t_norm3 - 15 * t_norm4 + 6 * t_norm5

            # Velocity coefficients (derivative of position)
            vel_coeff = (
                30 * t_norm2 - 60 * t_norm3 + 30 * t_norm4) / self.duration

            # Acceleration coefficients (derivative of velocity)
            acc_coeff = (
                60 * t_norm - 180 * t_norm2 + 120 * t_norm3) / (self.duration * self.duration)

            positions = []
            velocities = []
            accelerations = []

            for j in range(len(traj.joint_names)):
                joint_name = traj.joint_names[j]
                pos = start_pos[joint_name] + (
                    end_pos[joint_name] - start_pos[joint_name]) * pos_coeff
                vel = (end_pos[joint_name] - start_pos[joint_name]) * vel_coeff
                acc = (end_pos[joint_name] - start_pos[joint_name]) * acc_coeff

                positions.append(pos)
                velocities.append(vel)
                accelerations.append(acc)

            point.positions = positions
            point.velocities = velocities
            point.accelerations = accelerations
            point.time_from_start.sec = int(times[i])
            point.time_from_start.nanosec = int((times[i] % 1) * 1e9)

            traj.points.append(point)

        return traj

    def check_step_completion(self):
        for joint_name, target_pos in self.target_positions.items():
            if abs(self.current_positions.get(joint_name, 0) - target_pos) >= self.epsilon:
                return False
        return True

    def feedback_callback(self, feedback_msg, controller):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback from {controller}: {feedback.actual.positions}')

    def goal_response_callback(self, future, controller):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Goal rejected for {controller} :(')
            return

        self.get_logger().info(f'Goal accepted for {controller} :)')
        self.goal_handles[controller] = goal_handle

    def joint_state_callback(self, msg):
        # Update current positions and velocities
        for i, joint_name in enumerate(msg.name):
            self.current_positions[joint_name] = msg.position[i]
            self.current_velocities[joint_name] = msg.velocity[i]

        # Check if we need to send new goals
        if all(handle is None for handle in self.goal_handles.values()):
            if self.current_step < 2:  # We have two steps: initial and final positions
                for controller, joint_names in self.joint_groups.items():
                    if (controller not in self.goal_handles or
                            self.goal_handles[controller] is None):
                        self.get_logger().info(
                            f'Moving {controller} to step {self.current_step} positions'
                        )

                        goal_msg = FollowJointTrajectory.Goal()
                        goal_msg.trajectory = self.create_smooth_trajectory(
                            controller,
                            self.current_positions,
                            self.target_positions
                        )

                        goal_msg.path_tolerance = []
                        goal_msg.goal_tolerance = []
                        goal_msg.goal_time_tolerance.sec = 0
                        goal_msg.goal_time_tolerance.nanosec = 0

                        self.get_logger().info(f'Sending goal to {controller}...')
                        self._send_goal_future = self.action_clients[controller].send_goal_async(
                            goal_msg,
                            feedback_callback=lambda msg, c=controller: self.feedback_callback(
                                msg, c
                            )
                        )
                        self._send_goal_future.add_done_callback(
                            lambda future, c=controller: self.goal_response_callback(future, c)
                        )
            else:
                self.get_logger().info('All steps completed!')
                self.shutdown_node()
                return

        # Check if current step has reached its target
        if self.check_step_completion():
            if not self.reached_target:
                self.reached_target = True
                self.get_logger().info(f'🎯 Step {self.current_step} completed!')
                self.goal_handles = {k: None for k in self.goal_handles}
                self.current_step += 1
                self.reached_target = False

        # Log status periodically
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_status_time >= self.status_interval:
            self.last_status_time = current_time
            self.get_logger().info(f'Current positions: {self.current_positions}')
            self.get_logger().info(f'Target positions: {self.target_positions}')
            self.get_logger().info(f'Current step: {self.current_step}')

    def shutdown_node(self):
        for handle in self.goal_handles.values():
            if handle:
                handle.cancel_goal_async()
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = MoveToHome()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
