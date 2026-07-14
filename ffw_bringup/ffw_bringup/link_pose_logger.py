#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException


class LinkPoseLogger(Node):
    """Log a link pose from TF as position and quaternion."""

    def __init__(self):
        super().__init__('link_pose_logger')

        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'zedm_camera_link')
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('publish_topic', 'link_pose')

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.rate = float(self.get_parameter('rate').value)
        self.publish_topic = self.get_parameter('publish_topic').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, self.publish_topic, 10)

        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.get_logger().info(
            f'Logging TF pose: {self.target_frame} -> {self.source_frame} '
            f'at {self.rate:.2f} Hz'
        )

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Could not transform {self.target_frame} -> '
                f'{self.source_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = translation.x
        pose.pose.position.y = translation.y
        pose.pose.position.z = translation.z
        pose.pose.orientation = rotation
        self.pose_pub.publish(pose)

        self.get_logger().info(
            f'{self.source_frame} in {self.target_frame}: '
            f'pos=({translation.x:.6f}, {translation.y:.6f}, {translation.z:.6f}), '
            f'quat=({rotation.x:.6f}, {rotation.y:.6f}, '
            f'{rotation.z:.6f}, {rotation.w:.6f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LinkPoseLogger()
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
