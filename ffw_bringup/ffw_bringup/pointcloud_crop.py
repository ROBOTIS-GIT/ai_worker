#!/usr/bin/env python3

import math
import threading

from builtin_interfaces.msg import Duration
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField
import tf2_ros
from tf2_ros import TransformException
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

# PointField.datatype -> numpy scalar type, for building a structured dtype that
# mirrors the incoming cloud's own field layout (including any padding, via
# point_step as the record itemsize) so every field (rgb, intensity, ...) survives
# the crop untouched -- only which points survive changes.
_DATATYPE_TO_NUMPY = {
    PointField.INT8: np.int8,
    PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


class PointCloudCrop(Node):
    """Workspace point cloud tooling: crop, flatten the table, and mark detections.

    Three things bundled into one node so there's a single thing to launch:

    1. Crop: republish the ZED registered cloud with only points inside an
       axis-aligned box in target_frame kept (base_link: +X forward, +Y left) --
       crops to "in front of the robot" regardless of which way the camera looks.
       Points below the (smoothed) table height minus below_table_margin are
       dropped too, once a table plane has been found.
    2. Table plane: RANSAC-fit the dominant flat surface among the cropped points
       (same idea as table_plane_pointcloud.py) to find the table's real pose and
       footprint, smoothed across frames (table_plane_smoothing_alpha) since a
       raw per-frame fit wobbles even for a static table. Unlike a point-cloud
       reconstruction (each point placed from its own noisy stereo depth), the
       output is a regular raster built directly on that real, fixed plane
       (table_plane_grid_resolution apart) and colored by reprojecting the color
       image onto it -- like image_plane_pointcloud.py's flat image, except
       grounded to the table's real position/orientation instead of floating in
       front of the camera and following its viewing angle. Depth is only used to
       measure where/how big that plane is, never to place individual points.
    3. Bbox markers: render CenterPose's detections as RViz cubes, corrected the
       same way centerpose_bottle_pick.py corrects its grasp target -- CenterPose's
       own position/size assume a canonical object size and can be off several
       times over, so position is recovered via camera_info+depth reprojection and
       size via a flat measured scale factor.
    """

    def __init__(self):
        super().__init__('pointcloud_crop')

        # --- Crop -------------------------------------------------------------
        self.declare_parameter('input_topic', '/zedm/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('output_topic', '/zedm/zed_node/point_cloud/cloud_cropped')
        self.declare_parameter('target_frame', 'base_link')
        # Forward reach: keep points with 0 <= x <= x_max (base_link +X is forward).
        self.declare_parameter('x_max', 1.2)
        # Left/right reach: keep points with -y_extent <= y <= y_extent (base_link
        # +Y is left), i.e. y_extent=1.0 keeps a 2m-wide band. 0 (or negative)
        # disables this bound entirely -- left/right is left uncropped.
        self.declare_parameter('y_extent', 1.0)
        # Height bounds default wide open (effectively disabled) since it wasn't
        # asked for; narrow these if the ceiling/floor need cropping too.
        self.declare_parameter('z_min', -10.0)
        self.declare_parameter('z_max', 10.0)

        # --- Table plane --------------------------------------------------------
        self.declare_parameter('publish_table_plane', True)
        self.declare_parameter('table_plane_topic', '/zedm/zed_node/point_cloud/table_plane')
        # Max distance from the fitted plane (meters) to count as "on the table".
        self.declare_parameter('table_plane_distance_threshold', 0.01)
        self.declare_parameter('table_plane_ransac_iterations', 150)
        self.declare_parameter('table_plane_min_inliers', 200)
        # RANSAC samples from at most this many of the cropped points, to keep the
        # fit cheap regardless of how dense the input cloud is.
        self.declare_parameter('table_plane_max_ransac_points', 20000)
        # Reject the fitted plane if it's tilted more than ~37 degrees off
        # horizontal (base_link Z) -- guards against locking onto a wall, the
        # robot's own arm, or some other non-table flat surface in the crop box.
        self.declare_parameter('table_plane_normal_z_min', 0.8)
        # Per-frame RANSAC fits wobble a little even for a static table (stereo
        # depth noise), which made a naive per-point reconstruction flicker like a
        # raw depth cloud instead of looking like a stable image. Blend each new
        # fit (pose and footprint both) into a running estimate (higher alpha =
        # faster to track, jitterier).
        self.declare_parameter('table_plane_smoothing_alpha', 0.15)
        # Cell size (meters) of the output raster -- the table plane is redrawn as
        # a regular grid on the smoothed, real plane (not one point per noisy
        # source point), so this is closer to "image resolution" than a point
        # density knob.
        self.declare_parameter('table_plane_grid_resolution', 0.004)
        # Color source for the raster: the color image is reprojected onto the
        # real table plane using the camera's own pose, not per-pixel depth.
        self.declare_parameter('color_topic', '/zedm/zed_node/left/image_rect_color/compressed')
        # Points more than this far below the smoothed table height are dropped
        # from the main cropped cloud (legs, floor, robot base clutter under the
        # table). A small margin (not 0) keeps the table's own noisy points, which
        # scatter slightly below the fit, from being eaten too.
        self.declare_parameter('below_table_margin', 0.02)

        # --- Bbox markers --------------------------------------------------------
        self.declare_parameter('detections_topic', '/centerpose/detections')
        self.declare_parameter('camera_info_topic', '/zedm/zed_node/left/camera_info')
        self.declare_parameter('depth_topic', '/zedm/zed_node/depth/depth_registered')
        self.declare_parameter('depth_window', 5)
        self.declare_parameter('marker_topic', '/centerpose/bbox_markers')
        self.declare_parameter('marker_color_rgba', [1.0, 0.3, 0.0, 0.35])
        # CenterPose's own bbox.size is measured ~5x too big against the real object
        # (e.g. a 0.06x0.06x0.20m bottle reported as ~0.38x1.00x0.37m); fixed ratio,
        # not distance-dependent, so it's applied as a flat scale.
        self.declare_parameter('bbox_size_scale', 0.2)
        # Markers auto-expire this long after being published, so stale boxes don't
        # linger in RViz if the detections stream stops or drops a frame.
        self.declare_parameter('marker_lifetime', 0.5)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.x_max = float(self.get_parameter('x_max').value)
        y_extent = float(self.get_parameter('y_extent').value)
        if y_extent > 0.0:
            self.y_min = -y_extent
            self.y_max = y_extent
        else:
            self.y_min = -math.inf
            self.y_max = math.inf
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)

        self.publish_table_plane = bool(self.get_parameter('publish_table_plane').value)
        self.table_plane_topic = self.get_parameter('table_plane_topic').value
        self.table_plane_distance_threshold = float(
            self.get_parameter('table_plane_distance_threshold').value
        )
        self.table_plane_ransac_iterations = max(
            1, int(self.get_parameter('table_plane_ransac_iterations').value)
        )
        self.table_plane_min_inliers = max(
            3, int(self.get_parameter('table_plane_min_inliers').value)
        )
        self.table_plane_max_ransac_points = max(
            3, int(self.get_parameter('table_plane_max_ransac_points').value)
        )
        self.table_plane_normal_z_min = float(
            self.get_parameter('table_plane_normal_z_min').value
        )
        self.table_plane_smoothing_alpha = float(
            self.get_parameter('table_plane_smoothing_alpha').value
        )
        self.table_plane_grid_resolution = float(
            self.get_parameter('table_plane_grid_resolution').value
        )
        self.color_topic = self.get_parameter('color_topic').value
        self.below_table_margin = float(self.get_parameter('below_table_margin').value)
        self.rng = np.random.default_rng()
        self._table_centroid_ema = None
        self._table_normal_ema = None
        self._table_extent_ema = None
        self._table_z_ema = None

        self.detections_topic = self.get_parameter('detections_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.depth_window = int(self.get_parameter('depth_window').value)
        self.marker_topic = self.get_parameter('marker_topic').value
        marker_color_rgba = [float(v) for v in self.get_parameter('marker_color_rgba').value]
        if len(marker_color_rgba) != 4:
            raise ValueError('marker_color_rgba must contain [r, g, b, a]')
        self.marker_color_rgba = marker_color_rgba
        self.bbox_size_scale = float(self.get_parameter('bbox_size_scale').value)
        self.marker_lifetime = float(self.get_parameter('marker_lifetime').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.data_lock = threading.Lock()
        self.latest_camera_info = None
        self.latest_depth_image = None
        self.latest_depth_msg = None
        self.previous_detection_count = 0

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(PointCloud2, self.output_topic, sensor_qos)
        if self.publish_table_plane:
            self.table_plane_pub = self.create_publisher(
                PointCloud2, self.table_plane_topic, sensor_qos
            )
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.create_subscription(
            PointCloud2, self.input_topic, self._cloud_callback, sensor_qos
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_callback, sensor_qos
        )
        self.create_subscription(Image, self.depth_topic, self._depth_callback, sensor_qos)
        self.create_subscription(
            Detection3DArray, self.detections_topic, self._detections_callback, sensor_qos
        )
        if self.publish_table_plane:
            self.create_subscription(
                CompressedImage, self.color_topic, self._color_callback, sensor_qos
            )

        self.get_logger().info(
            f'Cropping {self.input_topic} -> {self.output_topic} to {self.target_frame} '
            f'box: x in [0, {self.x_max}], y in [{self.y_min}, {self.y_max}], '
            f'z in [{self.z_min}, {self.z_max}]'
        )
        if self.publish_table_plane:
            self.get_logger().info(f'  table plane -> {self.table_plane_topic}')
        self.get_logger().info(
            f'  bbox markers: {self.detections_topic} -> {self.marker_topic} '
            f'(corrected with {self.camera_info_topic}, {self.depth_topic})'
        )

    # --- Point cloud crop + table plane ---------------------------------------

    def _cloud_callback(self, msg):
        if msg.width * msg.height == 0:
            return

        field_dtype = self._build_dtype(msg)
        if field_dtype is None:
            return

        points = np.frombuffer(msg.data, dtype=field_dtype, count=msg.width * msg.height)

        xyz = np.column_stack(
            [points['x'], points['y'], points['z']]
        ).astype(np.float64)
        finite = np.isfinite(xyz).all(axis=1)
        if not np.any(finite):
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, rclpy.time.Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot transform {msg.header.frame_id} -> {self.target_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        rotation = self._quaternion_to_matrix(np.array([q.x, q.y, q.z, q.w], dtype=np.float64))
        translation = np.array([t.x, t.y, t.z], dtype=np.float64)

        target_xyz = xyz[finite] @ rotation.T + translation
        in_box = (
            (target_xyz[:, 0] >= 0.0) & (target_xyz[:, 0] <= self.x_max) &
            (target_xyz[:, 1] >= self.y_min) & (target_xyz[:, 1] <= self.y_max) &
            (target_xyz[:, 2] >= self.z_min) & (target_xyz[:, 2] <= self.z_max)
        )

        kept = points[finite][in_box]
        kept_target_xyz = target_xyz[in_box]

        if self.publish_table_plane:
            self._update_table_plane_state(kept_target_xyz)

        if self._table_z_ema is not None:
            floor = self._table_z_ema - self.below_table_margin
            above_floor = kept_target_xyz[:, 2] >= floor
            kept = kept[above_floor]

        self._publish(msg, kept)

    def _update_table_plane_state(self, kept_target_xyz):
        """Fit+smooth the table's pose (centroid/normal) and footprint (in-plane
        extent) from the cropped points. Only the plane's geometry comes from
        here -- the actual table_plane cloud is textured from the color image in
        _color_callback, using this state."""
        plane = self._fit_plane_ransac(kept_target_xyz)
        if plane is None:
            return
        normal, centroid, inlier_mask = plane
        if abs(normal[2]) < self.table_plane_normal_z_min:
            return

        alpha = self.table_plane_smoothing_alpha
        if self._table_centroid_ema is None:
            self._table_centroid_ema = centroid.copy()
            self._table_normal_ema = normal.copy()
        else:
            # Keep the normal on the same hemisphere as the running estimate
            # before blending -- RANSAC/SVD can flip its sign frame to frame.
            if self._table_normal_ema @ normal < 0.0:
                normal = -normal
            self._table_centroid_ema = (1.0 - alpha) * self._table_centroid_ema + alpha * centroid
            blended_normal = (1.0 - alpha) * self._table_normal_ema + alpha * normal
            norm = np.linalg.norm(blended_normal)
            if norm > 1e-9:
                self._table_normal_ema = blended_normal / norm
        self._table_z_ema = float(self._table_centroid_ema[2])

        u_hat, v_hat = self._plane_basis(self._table_normal_ema)
        rel = kept_target_xyz[inlier_mask] - self._table_centroid_ema
        pu = rel @ u_hat
        pv = rel @ v_hat
        extent = np.array([pu.min(), pu.max(), pv.min(), pv.max()])
        if self._table_extent_ema is None:
            self._table_extent_ema = extent
        else:
            self._table_extent_ema = (1.0 - alpha) * self._table_extent_ema + alpha * extent

    def _color_callback(self, msg):
        if self._table_centroid_ema is None or self._table_extent_ema is None:
            self.get_logger().warn(
                'Table plane not found yet; cannot texture-map it', throttle_duration_sec=2.0
            )
            return

        camera_info = self.latest_camera_info
        if camera_info is None or camera_info.k[0] <= 0.0 or camera_info.k[4] <= 0.0:
            return

        buffer = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        bgr = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().warn(
                'Failed to decode compressed color image', throttle_duration_sec=2.0
            )
            return

        camera_frame = msg.header.frame_id or camera_info.header.frame_id
        try:
            # Inverse of the crop's lookup (camera <- target instead of target <-
            # camera): gives points-in-target -> points-in-camera directly.
            transform = self.tf_buffer.lookup_transform(
                camera_frame, self.target_frame, rclpy.time.Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot transform {self.target_frame} -> {camera_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        rotation = self._quaternion_to_matrix(np.array([q.x, q.y, q.z, q.w], dtype=np.float64))
        translation = np.array([t.x, t.y, t.z], dtype=np.float64)

        centroid = self._table_centroid_ema
        normal = self._table_normal_ema
        u_hat, v_hat = self._plane_basis(normal)
        pu_min, pu_max, pv_min, pv_max = self._table_extent_ema
        res = self.table_plane_grid_resolution
        pu_vals = np.arange(pu_min, pu_max, res)
        pv_vals = np.arange(pv_min, pv_max, res)
        if pu_vals.size == 0 or pv_vals.size == 0:
            return
        grid_pu, grid_pv = np.meshgrid(pu_vals, pv_vals, indexing='ij')
        grid_pu = grid_pu.reshape(-1)
        grid_pv = grid_pv.reshape(-1)

        # Build the raster directly on the real, smoothed table plane -- every
        # grid point's position is pure geometry (centroid/basis), never derived
        # from a noisy per-pixel depth reading.
        positions_target = (
            centroid[None, :] + grid_pu[:, None] * u_hat[None, :] + grid_pv[:, None] * v_hat[None, :]
        )

        # Reproject into the camera to look up which pixel colors each grid
        # point -- this is the only place depth/vision touches color, and it's
        # a lookup, not a placement.
        positions_cam = positions_target @ rotation.T + translation
        z_cam = positions_cam[:, 2]
        in_front = z_cam > 1e-3

        orig_height, orig_width = bgr.shape[:2]
        scale_x = orig_width / camera_info.width if camera_info.width else 1.0
        scale_y = orig_height / camera_info.height if camera_info.height else 1.0
        fx = camera_info.k[0] * scale_x
        fy = camera_info.k[4] * scale_y
        cx = camera_info.k[2] * scale_x
        cy = camera_info.k[5] * scale_y

        u_px = np.zeros_like(z_cam)
        v_px = np.zeros_like(z_cam)
        u_px[in_front] = fx * positions_cam[in_front, 0] / z_cam[in_front] + cx
        v_px[in_front] = fy * positions_cam[in_front, 1] / z_cam[in_front] + cy

        u_idx = np.round(u_px).astype(np.int64)
        v_idx = np.round(v_px).astype(np.int64)
        in_image = (
            in_front & (u_idx >= 0) & (u_idx < orig_width) & (v_idx >= 0) & (v_idx < orig_height)
        )
        if not np.any(in_image):
            return

        sel_positions = positions_target[in_image]
        sel_bgr = bgr[v_idx[in_image], u_idx[in_image]]
        r = sel_bgr[:, 2].astype(np.uint32)
        g = sel_bgr[:, 1].astype(np.uint32)
        b = sel_bgr[:, 0].astype(np.uint32)
        rgb_float = ((r << 16) | (g << 8) | b).view(np.float32)

        n = sel_positions.shape[0]
        data = np.zeros((n, 4), dtype=np.float32)
        data[:, 0:3] = sel_positions
        data[:, 3] = rgb_float

        cloud = PointCloud2()
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = self.target_frame
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
        self.table_plane_pub.publish(cloud)

    @staticmethod
    def _plane_basis(normal):
        reference = (
            np.array([0.0, 0.0, 1.0]) if abs(normal[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        )
        u_hat = reference - (reference @ normal) * normal
        u_hat /= np.linalg.norm(u_hat)
        v_hat = np.cross(normal, u_hat)
        return u_hat, v_hat

    def _fit_plane_ransac(self, points):
        n_points = points.shape[0]
        if n_points < self.table_plane_min_inliers:
            return None

        sample_count = min(n_points, self.table_plane_max_ransac_points)
        if sample_count < n_points:
            sample_indices = self.rng.choice(n_points, size=sample_count, replace=False)
            sample_points = points[sample_indices]
        else:
            sample_points = points

        best_inlier_count = -1
        best_normal = None
        best_point = None

        for _ in range(self.table_plane_ransac_iterations):
            idx = self.rng.choice(sample_points.shape[0], size=3, replace=False)
            p0, p1, p2 = sample_points[idx]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = np.linalg.norm(normal)
            if norm < 1e-9:
                continue
            normal = normal / norm

            distances = np.abs((sample_points - p0) @ normal)
            inlier_count = int((distances < self.table_plane_distance_threshold).sum())
            if inlier_count > best_inlier_count:
                best_inlier_count = inlier_count
                best_normal = normal
                best_point = p0

        if best_normal is None or best_inlier_count < self.table_plane_min_inliers:
            return None

        # Refine with a least-squares fit over all inliers (from the full point
        # set, not just the RANSAC sampling subset) for a cleaner plane.
        distances_all = np.abs((points - best_point) @ best_normal)
        inlier_mask = distances_all < self.table_plane_distance_threshold
        if int(inlier_mask.sum()) < self.table_plane_min_inliers:
            return None

        inlier_points = points[inlier_mask]
        centroid = inlier_points.mean(axis=0)
        centered = inlier_points - centroid
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        refined_normal = vt[-1]
        if refined_normal @ best_normal < 0.0:
            refined_normal = -refined_normal

        distances_refined = np.abs((points - centroid) @ refined_normal)
        inlier_mask = distances_refined < self.table_plane_distance_threshold
        if int(inlier_mask.sum()) < self.table_plane_min_inliers:
            return None

        return refined_normal, centroid, inlier_mask

    def _build_dtype(self, msg):
        names = []
        formats = []
        offsets = []
        for field in msg.fields:
            numpy_type = _DATATYPE_TO_NUMPY.get(field.datatype)
            if numpy_type is None:
                self.get_logger().warn(
                    f'Unsupported PointField datatype {field.datatype} for '
                    f'"{field.name}"; skipping cloud',
                    throttle_duration_sec=2.0,
                )
                return None
            names.append(field.name)
            formats.append(numpy_type)
            offsets.append(field.offset)

        if 'x' not in names or 'y' not in names or 'z' not in names:
            self.get_logger().warn(
                'Point cloud has no x/y/z fields; cannot crop', throttle_duration_sec=2.0
            )
            return None

        return np.dtype({
            'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': msg.point_step,
        })

    def _publish(self, source_msg, points):
        cloud = PointCloud2()
        cloud.header = source_msg.header
        cloud.height = 1
        cloud.width = points.shape[0]
        cloud.fields = source_msg.fields
        cloud.is_bigendian = source_msg.is_bigendian
        cloud.point_step = source_msg.point_step
        cloud.row_step = source_msg.point_step * points.shape[0]
        cloud.is_dense = True
        cloud.data = points.tobytes()
        self.pub.publish(cloud)

    @staticmethod
    def _quaternion_to_matrix(q):
        x, y, z, w = q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float64)

    # --- Bbox markers -----------------------------------------------------------

    def _camera_info_callback(self, msg):
        with self.data_lock:
            self.latest_camera_info = msg

    def _depth_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as exc:
            self.get_logger().warn(
                f'Failed to convert depth image: {exc}', throttle_duration_sec=2.0
            )
            return
        with self.data_lock:
            self.latest_depth_image = image
            self.latest_depth_msg = msg

    def _detections_callback(self, msg):
        with self.data_lock:
            camera_info = self.latest_camera_info
            depth_image = self.latest_depth_image
            depth_msg = self.latest_depth_msg

        if camera_info is None or depth_image is None or depth_msg is None:
            self.get_logger().warn(
                'No camera info / depth image yet; cannot correct bbox position/size',
                throttle_duration_sec=2.0,
            )
            return

        markers = MarkerArray()
        lifetime = self._duration(self.marker_lifetime)
        published = 0

        for detection in msg.detections:
            corrected = self._correct_bbox(detection, camera_info, depth_image, depth_msg)
            if corrected is None:
                continue
            position, size = corrected

            frame_id = detection.header.frame_id or msg.header.frame_id
            stamp = (
                detection.header.stamp
                if (detection.header.stamp.sec or detection.header.stamp.nanosec)
                else msg.header.stamp
            )

            box = Marker()
            box.header.frame_id = frame_id
            box.header.stamp = stamp
            box.ns = 'centerpose_bbox'
            box.id = published
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(position[0])
            box.pose.position.y = float(position[1])
            box.pose.position.z = float(position[2])
            box.pose.orientation = detection.bbox.center.orientation
            # A zero-size cube renders invisibly and can also upset RViz; floor it.
            box.scale.x = max(float(size[0]), 1e-3)
            box.scale.y = max(float(size[1]), 1e-3)
            box.scale.z = max(float(size[2]), 1e-3)
            box.color.r, box.color.g, box.color.b, box.color.a = self.marker_color_rgba
            box.lifetime = lifetime
            markers.markers.append(box)

            published += 1

        # A previous message may have produced more markers than this one; explicitly
        # delete the now-unused ids so their boxes don't linger past marker_lifetime.
        for stale_id in range(published, self.previous_detection_count):
            delete_marker = Marker()
            delete_marker.header.frame_id = msg.header.frame_id
            delete_marker.ns = 'centerpose_bbox'
            delete_marker.id = stale_id
            delete_marker.action = Marker.DELETE
            markers.markers.append(delete_marker)
        self.previous_detection_count = published

        self.marker_pub.publish(markers)

    def _correct_bbox(self, detection, camera_info, depth_image, depth_msg):
        center = detection.bbox.center.position
        if not all(math.isfinite(value) for value in (center.x, center.y, center.z)):
            return None
        if abs(center.z) < 1e-6:
            return None

        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        # CenterPose's (x, y, z) has the wrong scale but the right bearing -- reproject
        # that bearing to a pixel and read real depth there instead of trusting
        # CenterPose's own z magnitude (see centerpose_bottle_pick.py).
        u = fx * (center.x / center.z) + cx
        v = fy * (center.y / center.z) + cy

        real_depth = self._sample_depth(depth_image, depth_msg, int(round(u)), int(round(v)))
        if real_depth is None:
            return None

        position = np.array(
            [(u - cx) * real_depth / fx, (v - cy) * real_depth / fy, real_depth],
            dtype=np.float64,
        )

        # CenterPose's reported size is off by a fixed ratio (bbox_size_scale),
        # measured against the real object -- not distance-dependent like the
        # position depth error corrected above.
        size = np.array(
            [detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z],
            dtype=np.float64,
        ) * self.bbox_size_scale

        return position, size

    def _sample_depth(self, depth_image, depth_msg, u, v):
        height, width = depth_image.shape[:2]
        half_window = max(0, self.depth_window // 2)
        u_min = max(0, u - half_window)
        u_max = min(width, u + half_window + 1)
        v_min = max(0, v - half_window)
        v_max = min(height, v + half_window + 1)
        if u_max <= u_min or v_max <= v_min:
            return None

        patch = np.asarray(depth_image[v_min:v_max, u_min:u_max], dtype=np.float32)
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if valid.size == 0:
            return None

        depth = float(np.median(valid))
        if depth_msg.encoding == '16UC1':
            depth *= 0.001
        return depth

    @staticmethod
    def _duration(seconds):
        duration = Duration()
        duration.sec = int(seconds)
        duration.nanosec = int((seconds % 1.0) * 1e9)
        return duration


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudCrop()
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
