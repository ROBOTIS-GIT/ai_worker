#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CenterposeCamera(Node):
    """ZED 카메라의 BGRA8(또는 RGBA8/RGB8) 이미지를 BGR8로 변환해서 재발행하고,
    CameraInfo도 이름만 바꿔서 같이 릴레이하는 노드.
    (topic_tools relay를 따로 안 띄워도 되게 하나로 합침)
    """

    # BGRA(ZED 카메라 원본) -> BGR 이미지 토픽 이름들 + CameraInfo 릴레이 토픽 이름들
    # 파라미터로 선언하고 퍼블리셔/구독을 연결.
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

        # BGRA -> BGR 변환 이미지 발행/구독
        self.pub = self.create_publisher(Image, self.output_topic, 10)
        self.sub = self.create_subscription(
            Image, self.input_topic, self.image_callback, 10)

        # CameraInfo는 변환 없이 이름만 바꿔서 그대로 릴레이
        self.camera_info_pub = self.create_publisher(
            CameraInfo, self.camera_info_output_topic, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_input_topic, self.camera_info_callback, 10)

        self.get_logger().info(f'Converting {self.input_topic} -> {self.output_topic} (bgr8)')
        self.get_logger().info(
            f'Relaying {self.camera_info_input_topic} -> {self.camera_info_output_topic}')

    # 들어온 이미지의 인코딩(bgra8/rgba8/rgb8)에 맞춰 BGR8로 변환해서 재발행.
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

    # 받은 CameraInfo를 변환 없이 그대로 다시 발행 (토픽 이름만 바뀜).
    def camera_info_callback(self, msg):
        self.camera_info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CenterposeCamera()  # 노드 실행 진입점 (ros2 run에서 호출됨)
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
