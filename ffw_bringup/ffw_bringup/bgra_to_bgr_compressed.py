#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class BgraToBgrCompressed(Node):
    """Convert a BGRA8 (or RGBA8/RGB8) Image topic into a raw BGR8 Image topic."""

    def __init__(self):
        super().__init__('bgra_to_bgr_compressed')

        self.declare_parameter('input_topic', '/zed/zed_node/rgb/color/rect/image')
        self.declare_parameter('output_topic', '/image')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.bridge = CvBridge()

        self.pub = self.create_publisher(Image, self.output_topic, 10)
        self.sub = self.create_subscription(
            Image, self.input_topic, self.image_callback, 10)

        self.get_logger().info(f'Converting {self.input_topic} -> {self.output_topic} (bgr8)')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}', throttle_duration_sec=2.0)
            return

        if msg.encoding.lower() in ('bgra8', 'rgba8'):
            code = cv2.COLOR_BGRA2BGR if msg.encoding.lower() == 'bgra8' else cv2.COLOR_RGBA2BGR
            bgr = cv2.cvtColor(frame, code)
        elif msg.encoding.lower() == 'rgb8':
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            bgr = frame

        out = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        out.header = msg.header
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BgraToBgrCompressed()
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
