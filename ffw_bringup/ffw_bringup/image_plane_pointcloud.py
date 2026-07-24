#!/usr/bin/env python3

import struct

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2, PointField


class ImagePlanePointCloud(Node):
    """Re-publish a camera Image as a flat, colored PointCloud2 plane.

    Every point comes from an image pixel (not from per-pixel depth) and is
    placed at one constant depth in front of the camera -- it's the camera
    feed floating as a flat "screen" in 3D space, not a real reconstruction.
    Real intrinsics (fx, fy, cx, cy) from CameraInfo unproject each pixel so
    the plane's size/shape at that depth matches the camera's true field of
    view. That one constant depth itself is taken from the robot's real
    depth camera (median of the valid readings) by default, so the plane
    sits at the scene's actual real-world distance instead of a made-up
    number; set use_depth_for_distance:=false to use a fixed plane_distance
    instead.

    Subscribes to the *compressed* color/depth transports rather than raw
    Image topics: over this robot's zenoh network link, small messages
    (CameraInfo) get through fine but raw Image/depth payloads never arrive
    at all -- only the compressed variants make it across.
    """

    # compressed_depth_image_transport's ConfigHeader: int32 format (0 = INV_DEPTH),
    # then two float32 quantization params (depthQuantA, depthQuantB).
    _DEPTH_HEADER_STRUCT = struct.Struct('<iff')

    def __init__(self):
        super().__init__('image_plane_pointcloud')

        self.declare_parameter(
            'input_topic', '/zedm/zed_node/left/image_rect_color/compressed'
        )
        self.declare_parameter('camera_info_topic', '/zedm/zed_node/left/camera_info')
        self.declare_parameter('output_topic', '/camera/image_plane')
        # Empty -> reuse the incoming image's own frame_id (usually the camera
        # optical frame), so the plane appears wherever that frame's TF puts it.
        self.declare_parameter('frame_id', '')
        # Sample every Nth pixel in each axis. A full-resolution cloud (step=1)
        # is far more points than RViz needs and than the network needs to carry.
        self.declare_parameter('pixel_step', 4)
        # Spacing between neighboring sampled pixels, in meters. Only used as a
        # fallback when no CameraInfo has been received yet.
        self.declare_parameter('pixel_size', 0.002)
        # Use the real depth camera's median valid reading as the plane's
        # distance, refreshed every depth frame, instead of a fixed number.
        self.declare_parameter('use_depth_for_distance', True)
        self.declare_parameter(
            'depth_topic', '/zedm/zed_node/depth/depth_registered/compressedDepth'
        )
        # Constant depth (meters, along the optical frame's +Z/forward axis) at
        # which the whole plane is placed. Used as-is when
        # use_depth_for_distance is false, and as a startup fallback (before
        # any depth frame has arrived) when it's true.
        self.declare_parameter('plane_distance', 1.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id_override = self.get_parameter('frame_id').value
        self.pixel_step = max(1, int(self.get_parameter('pixel_step').value))
        self.pixel_size = float(self.get_parameter('pixel_size').value)
        self.use_depth_for_distance = bool(self.get_parameter('use_depth_for_distance').value)
        self.depth_topic = self.get_parameter('depth_topic').value
        self.plane_distance = float(self.get_parameter('plane_distance').value)

        self.latest_camera_info = None

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(PointCloud2, self.output_topic, sensor_qos)
        self.sub = self.create_subscription(
            CompressedImage, self.input_topic, self._image_callback, sensor_qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_callback, sensor_qos
        )
        if self.use_depth_for_distance:
            self.create_subscription(
                CompressedImage, self.depth_topic, self._depth_callback, sensor_qos
            )

        self.get_logger().info(
            f'Publishing {self.input_topic} -> {self.output_topic} as a flat '
            f'PointCloud2 plane (step={self.pixel_step}); using real intrinsics from '
            f'{self.camera_info_topic} once received, else pixel_size={self.pixel_size} m; '
            + (
                f'plane distance from real depth on {self.depth_topic} (median), '
                f'starting at {self.plane_distance} m until the first depth frame arrives'
                if self.use_depth_for_distance
                else f'fixed plane distance={self.plane_distance} m'
            )
        )

    def _camera_info_callback(self, msg):
        self.latest_camera_info = msg

    def _depth_callback(self, msg):
        data = bytes(msg.data)
        header_size = self._DEPTH_HEADER_STRUCT.size
        if len(data) <= header_size:
            return
        fmt, quant_a, quant_b = self._DEPTH_HEADER_STRUCT.unpack(data[:header_size])
        buffer = np.frombuffer(data[header_size:], dtype=np.uint8)
        decoded = cv2.imdecode(buffer, cv2.IMREAD_UNCHANGED)
        if decoded is None:
            return

        valid = decoded != 0
        if not np.any(valid):
            return
        if fmt == 0:  # INV_DEPTH: quantized inverse depth, dequantize to meters
            depth_valid = quant_a / (decoded[valid].astype(np.float32) - quant_b)
        else:  # Raw 16-bit depth in millimeters
            depth_valid = decoded[valid].astype(np.float32) * 0.001

        finite = np.isfinite(depth_valid) & (depth_valid > 0.0)
        if not np.any(finite):
            return
        self.plane_distance = float(np.median(depth_valid[finite]))

    def _image_callback(self, msg):
        buffer = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        bgr = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().warn('Failed to decode compressed color image', throttle_duration_sec=2.0)
            return

        step = self.pixel_step
        sampled = bgr[::step, ::step]
        height, width = sampled.shape[:2]
        if height == 0 or width == 0:
            return

        rows, cols = np.meshgrid(
            np.arange(height, dtype=np.float32), np.arange(width, dtype=np.float32), indexing='ij'
        )

        camera_info = self.latest_camera_info
        if camera_info is not None and camera_info.k[0] > 0.0 and camera_info.k[4] > 0.0:
            # Scale intrinsics in case CameraInfo's resolution differs from the
            # image actually received (e.g. a downscaled publish).
            orig_height, orig_width = bgr.shape[:2]
            scale_x = orig_width / camera_info.width if camera_info.width else 1.0
            scale_y = orig_height / camera_info.height if camera_info.height else 1.0
            fx = camera_info.k[0] * scale_x
            fy = camera_info.k[4] * scale_y
            cx = camera_info.k[2] * scale_x
            cy = camera_info.k[5] * scale_y

            # Real pixel coordinates in the original (pre-downsample) image.
            orig_cols = cols * step
            orig_rows = rows * step

            depth = self.plane_distance
            x = (orig_cols - cx) * depth / fx
            y = (orig_rows - cy) * depth / fy
        else:
            center_col = (width - 1) * 0.5
            center_row = (height - 1) * 0.5
            # Optical-frame convention: +X right, +Y down, +Z forward.
            x = (cols - center_col) * self.pixel_size
            y = (rows - center_row) * self.pixel_size

        z = np.full_like(x, self.plane_distance, dtype=np.float32)

        b = sampled[:, :, 0].astype(np.uint32)
        g = sampled[:, :, 1].astype(np.uint32)
        r = sampled[:, :, 2].astype(np.uint32)
        rgb_uint32 = (r << 16) | (g << 8) | b
        rgb_float = rgb_uint32.view(np.float32)

        points = np.zeros((height, width, 4), dtype=np.float32)
        points[:, :, 0] = x
        points[:, :, 1] = y
        points[:, :, 2] = z
        points[:, :, 3] = rgb_float

        cloud = PointCloud2()
        cloud.header = msg.header
        if self.frame_id_override:
            cloud.header.frame_id = self.frame_id_override
        cloud.height = height
        cloud.width = width
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * width
        cloud.is_dense = True
        cloud.data = points.tobytes()

        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePlanePointCloud()
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
