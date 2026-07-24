#!/usr/bin/env python3

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class LeftGripperController(Node):
    """Open/close or set an arbitrary position for the left gripper joint."""

    def __init__(self):
        super().__init__('left_gripper_controller')

        self.declare_parameter('joint_name', 'gripper_l_joint1')
        self.declare_parameter(
            'joint_trajectory_topic',
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
        )
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('open_position', 0.0)
        self.declare_parameter('closed_position', 1.1)
        self.declare_parameter('subscriber_timeout', 2.0)
        # The MoveL controller (cyclo_control) streams arm-only trajectory points to this
        # same topic at ~100 Hz to hold its pose, even at rest. Because arm_l_controller
        # allows partial joint goals, each of those messages freezes the gripper joint at
        # its current position, so a single one-shot command gets pre-empted almost
        # immediately. Re-asserting the target at a comparable rate keeps it competitive.
        self.declare_parameter('command_rate_hz', 300.0)
        self.declare_parameter('point_lead_time', 0.1)

        self.joint_name = self.get_parameter('joint_name').value
        self.joint_trajectory_topic = self.get_parameter('joint_trajectory_topic').value
        self.open_position = float(self.get_parameter('open_position').value)
        self.closed_position = float(self.get_parameter('closed_position').value)
        self.subscriber_timeout = float(self.get_parameter('subscriber_timeout').value)
        self.command_rate_hz = float(self.get_parameter('command_rate_hz').value)
        self.point_lead_time = float(self.get_parameter('point_lead_time').value)

        self.position_min = min(self.open_position, self.closed_position)
        self.position_max = max(self.open_position, self.closed_position)
        self.current_position = None
        self.target_position = None

        self.trajectory_pub = self.create_publisher(
            JointTrajectory, self.joint_trajectory_topic, 10
        )
        self.state_pub = self.create_publisher(Float64, '~/state', 10)

        self.create_subscription(
            JointState,
            self.get_parameter('joint_states_topic').value,
            self._joint_state_callback,
            10,
        )
        self.create_subscription(
            Float64, '~/position_command', self._position_command_callback, 10
        )

        self.create_service(Trigger, '~/open', self._open_callback)
        self.create_service(Trigger, '~/close', self._close_callback)

        self.create_timer(1.0 / self.command_rate_hz, self._stream_callback)

        self.get_logger().info('Left gripper controller ready')
        self.get_logger().info(f'  joint: {self.joint_name}')
        self.get_logger().info(f'  trajectory topic: {self.joint_trajectory_topic}')
        self.get_logger().info(
            f'  open={self.open_position:.3f}, closed={self.closed_position:.3f}'
        )

    def _joint_state_callback(self, msg):
        if self.joint_name in msg.name:
            self.current_position = float(msg.position[msg.name.index(self.joint_name)])
            self.state_pub.publish(Float64(data=self.current_position))

    def _position_command_callback(self, msg):
        self._send_target(float(msg.data))

    def _open_callback(self, _request, response):
        return self._handle_trigger(self.open_position, response)

    def _close_callback(self, _request, response):
        return self._handle_trigger(self.closed_position, response)

    def _handle_trigger(self, target, response):
        if self._send_target(target):
            response.success = True
            response.message = f'{self.joint_name} -> {target:.3f}'
        else:
            response.success = False
            response.message = f'no subscriber on {self.joint_trajectory_topic}'
        return response

    def _send_target(self, target):
        clamped = min(self.position_max, max(self.position_min, target))
        if clamped != target:
            self.get_logger().warn(
                f'Requested position {target:.3f} clamped to {clamped:.3f}'
            )

        if not self._wait_for_subscriber():
            self.get_logger().error(f'No subscriber on {self.joint_trajectory_topic}')
            return False

        self.target_position = clamped
        self.get_logger().info(f'{self.joint_name} -> {clamped:.3f}')
        return True

    def _stream_callback(self):
        if self.target_position is None:
            return
        self._publish_point(self.target_position)

    def _publish_point(self, position):
        trajectory = JointTrajectory()
        trajectory.joint_names = [self.joint_name]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = int(self.point_lead_time)
        point.time_from_start.nanosec = int(self.point_lead_time % 1.0 * 1e9)
        trajectory.points = [point]
        self.trajectory_pub.publish(trajectory)

    def _wait_for_subscriber(self):
        deadline = time.monotonic() + self.subscriber_timeout
        while time.monotonic() < deadline:
            if self.trajectory_pub.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        return self.trajectory_pub.get_subscription_count() > 0


def main(args=None):
    rclpy.init(args=args)
    node = LeftGripperController()
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
