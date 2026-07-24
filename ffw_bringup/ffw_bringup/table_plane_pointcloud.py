#!/usr/bin/env python3

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class TablePlanePointCloud(Node):
    """Reconstruct real 3D points from a depth image and keep only the
    dominant flat surface in view (e.g. a table), fit by RANSAC.

    Unlike a synthetic fixed-distance plane, every point here comes from the
    depth camera's real per-pixel depth plus its real intrinsics, so the
    cloud is published in the depth image's own (real, TF-connected) frame
    and lines up with the robot's actual geometry/eye height.
    """

    def __init__(self):
        super().__init__('table_plane_pointcloud')

        self.declare_parameter('color_topic', '/zedm/zed_node/left/image_rect_color')
        self.declare_parameter('depth_topic', '/zedm/zed_node/depth/depth_registered')
        self.declare_parameter('camera_info_topic', '/zedm/zed_node/left/camera_info')
        self.declare_parameter('output_topic', '/camera/table_plane')
        self.declare_parameter('frame_id', '')

        # Sample every Nth pixel in each axis to keep RANSAC and publishing cheap.
        self.declare_parameter('pixel_step', 4)
        # Only consider points in this depth range as plane candidates (meters);
        # narrows the search to roughly where the table is expected to be and
        # keeps far background / near robot parts out of the fit.
        self.declare_parameter('min_depth', 0.15)
        self.declare_parameter('max_depth', 2.0)
        # Max distance from the fitted plane (meters) to count as "on the table".
        self.declare_parameter('plane_distance_threshold', 0.01)
        self.declare_parameter('ransac_iterations', 150)
        # Snap inlier points exactly onto the fitted plane so the result renders
        # as a perfectly flat sheet instead of the depth sensor's natural noise.
        self.declare_parameter('flatten_to_plane', True)
        self.declare_parameter('min_inliers', 200)

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id_override = self.get_parameter('frame_id').value
        self.pixel_step = max(1, int(self.get_parameter('pixel_step').value))
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.plane_distance_threshold = float(
            self.get_parameter('plane_distance_threshold').value
        )
        self.ransac_iterations = max(1, int(self.get_parameter('ransac_iterations').value))
        self.flatten_to_plane = bool(self.get_parameter('flatten_to_plane').value)
        self.min_inliers = max(3, int(self.get_parameter('min_inliers').value))

        self.bridge = CvBridge()
        self.latest_camera_info = None
        self.latest_color = None
        self.rng = np.random.default_rng()

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
        self.create_subscription(Image, self.color_topic, self._color_callback, sensor_qos)
        self.create_subscription(Image, self.depth_topic, self._depth_callback, sensor_qos)

        self.get_logger().info(
            f'Fitting table plane from {self.depth_topic} (+{self.camera_info_topic}) -> '
            f'{self.output_topic}; depth range [{self.min_depth}, {self.max_depth}] m, '
            f'inlier threshold {self.plane_distance_threshold} m'
        )

    def _camera_info_callback(self, msg):
        self.latest_camera_info = msg

    def _color_callback(self, msg):
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warn(f'color cv_bridge conversion failed: {exc}', throttle_duration_sec=2.0)

    def _depth_callback(self, msg):
        camera_info = self.latest_camera_info
        if camera_info is None or camera_info.k[0] <= 0.0 or camera_info.k[4] <= 0.0:
            self.get_logger().warn(
                'Waiting for camera_info before reconstructing points', throttle_duration_sec=2.0
            )
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as exc:
            self.get_logger().warn(f'depth cv_bridge conversion failed: {exc}', throttle_duration_sec=2.0)
            return

        depth = np.asarray(depth_image, dtype=np.float32)
        if msg.encoding == '16UC1':
            depth = depth * 0.001

        step = self.pixel_step
        depth_sampled = depth[::step, ::step]
        height_s, width_s = depth_sampled.shape[:2]
        if height_s == 0 or width_s == 0:
            return

        valid = np.isfinite(depth_sampled) & (depth_sampled >= self.min_depth) & \
            (depth_sampled <= self.max_depth)
        if int(valid.sum()) < self.min_inliers:
            self.get_logger().warn(
                'Not enough valid depth points in range to fit a plane',
                throttle_duration_sec=2.0,
            )
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

        plane = self._fit_plane_ransac(points)
        if plane is None:
            self.get_logger().warn('Plane fit failed (not enough inliers)', throttle_duration_sec=2.0)
            return
        normal, centroid, inlier_mask = plane

        table_points = points[inlier_mask]
        if self.flatten_to_plane:
            offsets = (table_points - centroid) @ normal
            table_points = table_points - offsets[:, None] * normal[None, :]

        colors = self._sample_colors(
            rows[valid][inlier_mask], cols[valid][inlier_mask], orig_width, orig_height
        )

        self._publish(msg, table_points, colors)

    def _fit_plane_ransac(self, points):
        n_points = points.shape[0]
        if n_points < self.min_inliers:
            return None

        best_inlier_count = -1
        best_normal = None
        best_point = None

        sample_count = min(n_points, 20000)
        if sample_count < n_points:
            sample_indices = self.rng.choice(n_points, size=sample_count, replace=False)
            sample_points = points[sample_indices]
        else:
            sample_points = points

        for _ in range(self.ransac_iterations):
            idx = self.rng.choice(sample_points.shape[0], size=3, replace=False)
            p0, p1, p2 = sample_points[idx]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = np.linalg.norm(normal)
            if norm < 1e-9:
                continue
            normal = normal / norm

            distances = np.abs((sample_points - p0) @ normal)
            inlier_count = int((distances < self.plane_distance_threshold).sum())
            if inlier_count > best_inlier_count:
                best_inlier_count = inlier_count
                best_normal = normal
                best_point = p0

        if best_normal is None or best_inlier_count < self.min_inliers:
            return None

        # Refine with a least-squares fit over all inliers (from the full point
        # set, not just the RANSAC sampling subset) for a cleaner plane.
        distances_all = np.abs((points - best_point) @ best_normal)
        inlier_mask = distances_all < self.plane_distance_threshold
        if int(inlier_mask.sum()) < self.min_inliers:
            return None

        inlier_points = points[inlier_mask]
        centroid = inlier_points.mean(axis=0)
        centered = inlier_points - centroid
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        refined_normal = vt[-1]
        if refined_normal @ best_normal < 0.0:
            refined_normal = -refined_normal

        distances_refined = np.abs((points - centroid) @ refined_normal)
        inlier_mask = distances_refined < self.plane_distance_threshold
        if int(inlier_mask.sum()) < self.min_inliers:
            return None

        return refined_normal, centroid, inlier_mask

    def _sample_colors(self, rows, cols, orig_width, orig_height):
        color = self.latest_color
        n = rows.shape[0]
        if color is None:
            gray = np.full((n, 3), 200, dtype=np.uint32)
            return gray

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
    node = TablePlanePointCloud()
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
