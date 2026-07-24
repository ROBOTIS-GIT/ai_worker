#!/usr/bin/env python3

import struct

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2, PointField


class DepthToPointCloud(Node):
    """Reconstruct a real, true-scale colored PointCloud2 straight from a
    depth image and its CameraInfo -- every point is the camera's own
    measured depth at that pixel (no fixed distance, no plane fitting), so
    sizes/positions match the real world and line up via the depth image's
    own (real, TF-connected) frame.

    Subscribes to the *compressed* color/depth transports rather than raw
    Image topics: over this robot's zenoh network link, small messages
    (CameraInfo) get through fine but raw Image/depth payloads never arrive
    at all -- only the compressed variants make it across. compressedDepth
    isn't a format cv_bridge understands (it's compressed_depth_image_transport's
    own 12-byte header + quantized-inverse-depth PNG), so it's decoded here
    directly; verified against a live depth frame to produce sane meter values.
    """

    # compressed_depth_image_transport's ConfigHeader: int32 format (0 = INV_DEPTH),
    # then two float32 quantization params (depthQuantA, depthQuantB).
    _DEPTH_HEADER_STRUCT = struct.Struct('<iff')

    def __init__(self):
        super().__init__('depth_to_pointcloud')

        self.declare_parameter(
            'color_topic', '/zedm/zed_node/left/image_rect_color/compressed'
        )
        self.declare_parameter(
            'depth_topic', '/zedm/zed_node/depth/depth_registered/compressedDepth'
        )
        self.declare_parameter('camera_info_topic', '/zedm/zed_node/left/camera_info')
        self.declare_parameter('output_topic', '/camera/depth_pointcloud')
        self.declare_parameter('frame_id', '')

        # Sample every Nth pixel in each axis to keep publishing/RViz cheap.
        self.declare_parameter('pixel_step', 2)
        # Drop points outside this depth range (meters); 0 disables the bound.
        self.declare_parameter('min_depth', 0.0)
        self.declare_parameter('max_depth', 0.0)

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id_override = self.get_parameter('frame_id').value
        self.pixel_step = max(1, int(self.get_parameter('pixel_step').value))
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)

        self.latest_camera_info = None
        self.latest_color = None

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(PointCloud2, self.output_topic, sensor_qos)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_callback, sensor_qos
        )
        self.create_subscription(
            CompressedImage, self.color_topic, self._color_callback, sensor_qos
        )
        self.create_subscription(
            CompressedImage, self.depth_topic, self._depth_callback, sensor_qos
        )

        self.get_logger().info(
            f'Publishing real-scale depth points from {self.depth_topic} '
            f'(+{self.camera_info_topic}, colored from {self.color_topic}) -> '
            f'{self.output_topic}'
        )

    def _camera_info_callback(self, msg):
        self.latest_camera_info = msg

    def _color_callback(self, msg):
        buffer = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        decoded = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if decoded is None:
            self.get_logger().warn('Failed to decode compressed color image', throttle_duration_sec=2.0)
            return
        self.latest_color = decoded

    def _decode_compressed_depth(self, msg):
        data = bytes(msg.data)
        header_size = self._DEPTH_HEADER_STRUCT.size
        if len(data) <= header_size:
            return None

        fmt, quant_a, quant_b = self._DEPTH_HEADER_STRUCT.unpack(data[:header_size])
        buffer = np.frombuffer(data[header_size:], dtype=np.uint8)
        decoded = cv2.imdecode(buffer, cv2.IMREAD_UNCHANGED)
        if decoded is None:
            return None

        if fmt == 0:  # INV_DEPTH: quantized inverse depth, dequantize to meters
            depth = np.zeros(decoded.shape, dtype=np.float32)
            valid = decoded != 0
            depth[valid] = quant_a / (decoded[valid].astype(np.float32) - quant_b)
            depth[~valid] = np.nan
            return depth
        # Raw 16-bit depth in millimeters, no quantization applied.
        return decoded.astype(np.float32) * 0.001

    def _depth_callback(self, msg):
        camera_info = self.latest_camera_info
        if camera_info is None or camera_info.k[0] <= 0.0 or camera_info.k[4] <= 0.0:
            self.get_logger().warn(
                'Waiting for camera_info before reconstructing points', throttle_duration_sec=2.0
            )
            return

        depth = self._decode_compressed_depth(msg)
        if depth is None:
            self.get_logger().warn('Failed to decode compressedDepth image', throttle_duration_sec=2.0)
            return

        step = self.pixel_step
        depth_sampled = depth[::step, ::step]
        height_s, width_s = depth_sampled.shape[:2]
        if height_s == 0 or width_s == 0:
            return

        valid = np.isfinite(depth_sampled) & (depth_sampled > 0.0)
        if self.min_depth > 0.0:
            valid &= depth_sampled >= self.min_depth
        if self.max_depth > 0.0:
            valid &= depth_sampled <= self.max_depth
        if not np.any(valid):
            return

        orig_height, orig_width = depth.shape[:2]
        scale_x = orig_width / camera_info.width if camera_info.width else 1.0
        scale_y = orig_height / camera_info.height if camera_info.height else 1.0
        fx = camera_info.k[0] * scale_x
        fy = camera_info.k[4] * scale_y
        cx = camera_info.k[2] * scale_x
        cy = camera_info.k[5] * scale_y

        rows, cols = np.meshgrid(
            np.arange(height_s, dtype=np.float32) * step,
            np.arange(width_s, dtype=np.float32) * step,
            indexing='ij',
        )

        z = depth_sampled
        x = (cols - cx) * z / fx
        y = (rows - cy) * z / fy

        points = np.stack([x[valid], y[valid], z[valid]], axis=1)
        colors = self._sample_colors(rows[valid], cols[valid], orig_width, orig_height)

        self._publish(msg, points, colors)

    def _sample_colors(self, rows, cols, orig_width, orig_height):
        color = self.latest_color
        n = rows.shape[0]
        if color is None:
            return np.full((n, 3), 200, dtype=np.uint32)

        color_height, color_width = color.shape[:2]
        col_idx = np.clip(
            (cols * (color_width / orig_width)).astype(np.int32), 0, color_width - 1
        )
        row_idx = np.clip(
            (rows * (color_height / orig_height)).astype(np.int32), 0, color_height - 1
        )
        bgr = color[row_idx, col_idx]
        return bgr[:, ::-1].astype(np.uint32)  # BGR -> RGB order for packing below

    def _publish(self, source_msg, points, colors_rgb):
        n = points.shape[0]
        if n == 0:
            return

        r = colors_rgb[:, 0]
        g = colors_rgb[:, 1]
        b = colors_rgb[:, 2]
        rgb_uint32 = (r << 16) | (g << 8) | b
        rgb_float = rgb_uint32.astype(np.uint32).view(np.float32)

        data = np.zeros((n, 4), dtype=np.float32)
        data[:, 0] = points[:, 0]
        data[:, 1] = points[:, 1]
        data[:, 2] = points[:, 2]
        data[:, 3] = rgb_float

        cloud = PointCloud2()
        cloud.header = source_msg.header
        if self.frame_id_override:
            cloud.header.frame_id = self.frame_id_override
        cloud.height = 1
        cloud.width = n
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * n
        cloud.is_dense = True
        cloud.data = data.tobytes()

        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloud()
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
