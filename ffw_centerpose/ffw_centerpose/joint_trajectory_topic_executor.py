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

import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class JointTrajectoryTopicExecutor(Node):
    """관절 그룹 하나를 미리 정해둔 자세(step)들로 순서대로 부드럽게 이동시키는 노드.

    ffw_bringup의 joint_trajectory_executor와 달리 FollowJointTrajectory 액션
    서버가 필요 없음 -- 그냥 토픽에 궤적을 직접 publish한다 (leader의
    joint_trajectory_command_broadcaster처럼 액션 서버가 없는 경우를 위함).
    """

    # 관절 이름/목표 자세(step)/토픽 등 파라미터를 읽고, 퍼블리셔/구독을 연결.
    def __init__(self):
        super().__init__('joint_trajectory_topic_executor')

        self.declare_parameter('joint_names', [''])
        self.declare_parameter('step_names', [''])
        self.declare_parameter('duration', 10.0)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('velocity_tolerance', 0.01)
        self.declare_parameter('trajectory_topic', '')
        self.declare_parameter('joint_states_topic', '/joint_states')

        self.joint_names = (
            self.get_parameter('joint_names').get_parameter_value().string_array_value
        )
        self.step_names = (
            self.get_parameter('step_names').get_parameter_value().string_array_value
        )
        self.duration = self.get_parameter('duration').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.velocity_tolerance = self.get_parameter('velocity_tolerance').value
        self.trajectory_topic = self.get_parameter('trajectory_topic').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value

        if not self.joint_names:
            self.get_logger().error('Missing required parameter: joint_names')
            sys.exit(1)
        if not self.step_names:
            self.get_logger().error('Missing required parameter: step_names')
            sys.exit(1)
        if not self.trajectory_topic:
            self.get_logger().error('Missing required parameter: trajectory_topic')
            sys.exit(1)

        self.positions_list = []
        for step_name in self.step_names:
            self.declare_parameter(step_name, [0.0] * len(self.joint_names))
            step_positions = (
                self.get_parameter(step_name).get_parameter_value().double_array_value
            )
            self.positions_list.append(step_positions)

        if not self.positions_list:
            self.get_logger().error('No valid step positions found')
            sys.exit(1)

        for i, pos in enumerate(self.positions_list):
            if len(pos) != len(self.joint_names):
                self.get_logger().error(
                    f'Position array {i} has incorrect length. '
                    f'Expected {len(self.joint_names)}, got {len(pos)}'
                )
                sys.exit(1)

        self.trajectory_pub = self.create_publisher(
            JointTrajectory, self.trajectory_topic, 10
        )
        self.subscription = self.create_subscription(
            JointState, self.joint_states_topic, self.joint_state_callback, 10
        )

        self.current_positions = None
        self.current_velocities = None
        self.reached_target = False
        self.num_points = 100  # Number of points for smooth trajectory
        self.goal_sent = False
        self.current_step = 0

        self.get_logger().info(f'Using trajectory topic: {self.trajectory_topic}')
        self.get_logger().info(f'Using joint states topic: {self.joint_states_topic}')

    # 지금 진행 중인 step(current_step)의 목표 관절 위치 배열을 반환.
    def get_step_target_positions(self):
        return self.positions_list[self.current_step]

    # 현재 위치/속도가 목표 위치와 tolerance 안으로 들어왔는지(=도착했는지) 판단.
    def check_step_completion(self):
        target_positions = self.get_step_target_positions()
        positions_ok = all(
            abs(curr - target) < self.position_tolerance
            for curr, target in zip(self.current_positions, target_positions)
        )
        velocities_ok = all(
            abs(vel) < self.velocity_tolerance for vel in self.current_velocities
        )
        return positions_ok and velocities_ok

    # /joint_states 수신마다: 현재 위치 갱신 -> (아직 안 보냈으면) 목표 궤적 publish
    # -> 도착했으면 다음 step으로 넘어가고, 마지막 step까지 다 끝나면 노드 종료.
    def joint_state_callback(self, msg):
        if set(self.joint_names).issubset(set(msg.name)):
            self.current_positions = [
                msg.position[msg.name.index(j)] for j in self.joint_names
            ]
            self.current_velocities = [
                msg.velocity[msg.name.index(j)] for j in self.joint_names
            ]

            if not self.goal_sent:
                if self.current_step < len(self.positions_list):
                    target_positions = self.get_step_target_positions()
                    self.get_logger().info(
                        f'Moving to step {self.current_step} target positions'
                    )

                    trajectory = self.create_smooth_trajectory(
                        self.current_positions, target_positions
                    )
                    self.goal_sent = True
                    self.trajectory_pub.publish(trajectory)

            if self.check_step_completion():
                if not self.reached_target:
                    self.reached_target = True
                    self.get_logger().info(f'Step {self.current_step} completed!')
                    self.goal_sent = False
                    self.current_step += 1
                    self.reached_target = False

                    if self.current_step >= len(self.positions_list):
                        self.get_logger().info('All steps completed!')
                        self.shutdown_node()
                        return

    # 모든 step이 끝났을 때 노드를 스스로 종료.
    def shutdown_node(self):
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    # 현재 위치(start_pos)에서 목표 위치(end_pos)까지 5차 다항식(quintic)으로
    # 부드럽게(가속/감속 있게) 이동하는 궤적(위치/속도/가속도 점들)을 생성.
    def create_smooth_trajectory(self, start_pos, end_pos):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        times = np.linspace(0, self.duration, self.num_points)

        for i in range(self.num_points):
            point = JointTrajectoryPoint()
            t = times[i]

            # 5차 다항식(quintic) 계수
            t_norm = t / self.duration
            t_norm2 = t_norm * t_norm
            t_norm3 = t_norm2 * t_norm
            t_norm4 = t_norm3 * t_norm
            t_norm5 = t_norm4 * t_norm

            pos_coeff = 10 * t_norm3 - 15 * t_norm4 + 6 * t_norm5
            vel_coeff = (30 * t_norm2 - 60 * t_norm3 + 30 * t_norm4) / self.duration
            acc_coeff = (60 * t_norm - 180 * t_norm2 + 120 * t_norm3) / (
                self.duration * self.duration
            )

            positions = []
            velocities = []
            accelerations = []

            for j in range(len(self.joint_names)):
                pos = start_pos[j] + (end_pos[j] - start_pos[j]) * pos_coeff
                vel = (end_pos[j] - start_pos[j]) * vel_coeff
                acc = (end_pos[j] - start_pos[j]) * acc_coeff

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


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryTopicExecutor()  # 노드 실행 진입점 (ros2 run/launch에서 호출됨)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
