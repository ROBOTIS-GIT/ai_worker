#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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
# Author: Seongjin Jeong

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CenterposeCamera(Node):
    def __init__(self):
        super().__init__('centerpose_camera')

        self.declare_parameter('input_topic', '/zedm/zed_node/left/image_rect_color')
        self.declare_parameter('output_topic', '/image')
        self.declare_parameter('camera_info_input_topic', '/zedm/zed_node/left/camera_info')
        self.declare_parameter('camera_info_output_topic', '/camera_info')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.camera_info_input_topic = self.get_parameter('camera_info_input_topic').value
        self.camera_info_output_topic = self.get_parameter('camera_info_output_topic').value

        self.bridge = CvBridge()

        self.pub = self.create_publisher(Image, self.output_topic, 10)
        self.sub = self.create_subscription(
            Image, self.input_topic, self.image_callback, 10)

        self.camera_info_pub = self.create_publisher(
            CameraInfo, self.camera_info_output_topic, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_input_topic, self.camera_info_callback, 10)

        self.get_logger().info(f'Converting {self.input_topic} -> {self.output_topic} (bgr8)')
        self.get_logger().info(
            f'Relaying {self.camera_info_input_topic} -> {self.camera_info_output_topic}')

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

    def camera_info_callback(self, msg):
        self.camera_info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CenterposeCamera()
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
