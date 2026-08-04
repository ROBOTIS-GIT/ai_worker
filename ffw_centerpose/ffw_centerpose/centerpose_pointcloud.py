#!/usr/bin/env python3

import math
import threading

from builtin_interfaces.msg import Duration
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField
from std_msgs.msg import ColorRGBA
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


class CenterposePointcloud(Node):
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
       measure where/how big that plane is, never to place individual points. A
       second raster (enable_wall) bends up from the table's far edge (farthest
       from the robot along base_link +X) along the table normal, textured the
       same way, so the color image content beyond the table's own footprint
       doesn't just get cut off -- it stands up like a wall instead. Its height
       isn't known from the table fit (RANSAC only ever sees the flat table), so
       it's measured each frame from one real depth reading straight up in the
       image from the edge (see _build_wall_points), smoothed the same way as the
       table geometry.
    3. Bbox markers: render CenterPose's detections as RViz cubes, corrected the
       same way the pick nodes correct their grasp targets -- CenterPose's own
       position/size assume a canonical object size and can be off several times
       over, so position is recovered via camera_info+depth reprojection and size
       via a flat measured scale factor.
    """

    def __init__(self):
        super().__init__('centerpose_pointcloud')

        # --- Crop -------------------------------------------------------------
        self.declare_parameter('input_topic', '/zedm/zed_node/point_cloud/cloud_registered')
        # marker_topic(/centerpose/bbox_markers)과 같은 네임스페이스에 둬서 RViz에서
        # bbox 마커/크롭 클라우드/테이블 평면이 한 카테고리로 묶여 보이게 함.
        self.declare_parameter('output_topic', '/centerpose/cloud_cropped')
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
        self.declare_parameter('table_plane_topic', '/centerpose/table_plane')
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
        # Once a table plane raster is successfully built, keep republishing that
        # exact snapshot (position + color) forever instead of refitting/retexturing
        # every frame -- freezes the display to whatever the camera saw the moment
        # it was first captured.
        self.declare_parameter('freeze_table_plane', False)

        # --- Wall (bent up from the table's far edge) -----------------------------
        # The table raster above only covers the table's own flat footprint. This
        # adds a second raster standing straight up (along the table normal) from
        # the table's far edge (the edge farthest from the robot along base_link
        # +X), textured the same way, so the color image content beyond the table
        # edge doesn't just get cut off.
        self.declare_parameter('enable_wall', True)
        # The wall's height isn't known from the table fit (RANSAC only ever sees
        # the flat table) -- it's solved geometrically each frame so the wall's
        # own projection reaches the top row of the image, i.e. it's stood up
        # tall enough to cover everything above the table the camera can see.
        # Clamped to this range as a safety bound (e.g. if the wall stands
        # near edge-on to the camera, the solve can blow up).
        self.declare_parameter('wall_min_height', 0.02)
        self.declare_parameter('wall_max_height', 2.0)

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
        # linger in RViz if the detections stream stops or drops a frame. CenterPose
        # itself publishes slowly/burstily (see detection_timeout elsewhere), so this
        # needs to comfortably outlast the gap between its detections -- too short
        # (e.g. the old 0.5s default) makes the marker flicker on/off between
        # detections instead of just disappearing when the stream actually stops.
        self.declare_parameter('marker_lifetime', 2.0)
        # CenterPose's detections stream is bursty and sometimes skips an id for a
        # single frame (e.g. one failed depth sample) even though the object is still
        # there -- treating that as "gone" and deleting the marker right away made it
        # flicker off and back on every gap. Instead, keep redrawing each marker's
        # last known position/size for up to this long after its last real update,
        # and only delete it once it's actually been missing longer than this.
        self.declare_parameter('marker_hold_timeout', 1.0)
        # Draws a red/green/blue XYZ axis triad at each box's center (its detected
        # orientation), the same way RViz's own TF axes look -- makes the object's
        # pose, not just its position, visible at a glance.
        self.declare_parameter('show_marker_axes', True)
        self.declare_parameter('marker_axis_length', 0.15)
        self.declare_parameter('marker_axis_width', 0.006)

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
        self.freeze_table_plane = bool(self.get_parameter('freeze_table_plane').value)
        self._frozen_table_cloud = None
        self.enable_wall = bool(self.get_parameter('enable_wall').value)
        self.wall_min_height = float(self.get_parameter('wall_min_height').value)
        self.wall_max_height = float(self.get_parameter('wall_max_height').value)
        self.rng = np.random.default_rng()
        self._table_centroid_ema = None
        self._table_normal_ema = None
        self._table_extent_ema = None
        self._table_z_ema = None
        self._wall_height_ema = None

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
        self.marker_hold_timeout = float(self.get_parameter('marker_hold_timeout').value)
        self.show_marker_axes = bool(self.get_parameter('show_marker_axes').value)
        self.marker_axis_length = float(self.get_parameter('marker_axis_length').value)
        self.marker_axis_width = float(self.get_parameter('marker_axis_width').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.data_lock = threading.Lock()
        self.latest_camera_info = None
        self.latest_depth_image = None
        self.latest_depth_msg = None
        self.last_marker_data = {}

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
    # (포인트클라우드 크롭 + 테이블 평면 감지/텍스처링)

    def _lookup_rotation_translation(self, target_frame, source_frame):
        """target_frame <- source_frame TF를 조회해 (회전행렬, 평행이동벡터)로 반환.
        조회 실패 시 경고를 남기고 None을 반환 (호출부에서 그 프레임 처리를 건너뜀)."""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot transform {source_frame} -> {target_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        rotation = self._quaternion_to_matrix(np.array([q.x, q.y, q.z, q.w], dtype=np.float64))
        translation = np.array([t.x, t.y, t.z], dtype=np.float64)
        return rotation, translation

    # 원본 포인트클라우드를 target_frame 박스로 크롭 + 테이블 아래 점 제거 + 재발행.
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

        rotation_translation = self._lookup_rotation_translation(
            self.target_frame, msg.header.frame_id
        )
        if rotation_translation is None:
            return
        rotation, translation = rotation_translation

        target_xyz = xyz[finite] @ rotation.T + translation
        in_box = (
            (target_xyz[:, 0] >= 0.0) & (target_xyz[:, 0] <= self.x_max) &
            (target_xyz[:, 1] >= self.y_min) & (target_xyz[:, 1] <= self.y_max) &
            (target_xyz[:, 2] >= self.z_min) & (target_xyz[:, 2] <= self.z_max)
        )

        kept = points[finite][in_box]
        kept_target_xyz = target_xyz[in_box]

        if self.publish_table_plane and not (self.freeze_table_plane and self._frozen_table_cloud is not None):
            self._update_table_plane_state(kept_target_xyz)

        if self._table_z_ema is not None:
            floor = self._table_z_ema - self.below_table_margin
            above_floor = kept_target_xyz[:, 2] >= floor
            kept = kept[above_floor]

        self._publish(msg, kept)

    # RANSAC으로 테이블 평면을 찾아서 위치/법선/가로세로 범위를 EMA로 부드럽게 갱신.
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

    # 테이블 평면을 실제 격자(raster)로 만들고 컬러 이미지를 재투영해 입혀서 발행.
    # freeze_table_plane=true면 처음 한 번만 계산하고 그 뒤로는 그대로 재발행.
    def _color_callback(self, msg):
        if self.freeze_table_plane and self._frozen_table_cloud is not None:
            # Already captured one snapshot -- keep republishing it untouched so
            # the display stays frozen instead of following the live camera feed.
            self._frozen_table_cloud.header.stamp = msg.header.stamp
            self.table_plane_pub.publish(self._frozen_table_cloud)
            return

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
        # Inverse of the crop's lookup (camera <- target instead of target <-
        # camera): gives points-in-target -> points-in-camera directly.
        rotation_translation = self._lookup_rotation_translation(
            camera_frame, self.target_frame
        )
        if rotation_translation is None:
            return
        rotation, translation = rotation_translation

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
        table_positions_target = (
            centroid[None, :] + grid_pu[:, None] * u_hat[None, :] + grid_pv[:, None] * v_hat[None, :]
        )

        intrinsics = self._scaled_intrinsics(camera_info, bgr.shape)

        # Reproject into the camera to look up which pixel colors each grid
        # point -- this is the only place depth/vision touches color, and it's
        # a lookup, not a placement.
        sel_positions, rgb_float = self._reproject_and_sample_color(
            table_positions_target, rotation, translation, intrinsics, bgr
        )
        if sel_positions is None:
            return
        all_positions = [sel_positions]
        all_rgb = [rgb_float]

        if self.enable_wall:
            wall_positions_target = self._build_wall_points(
                centroid, normal, u_hat, v_hat, pu_min, pu_max, pv_min, pv_max,
                rotation, translation, intrinsics,
            )
            if wall_positions_target is not None:
                wall_sel_positions, wall_rgb_float = self._reproject_and_sample_color(
                    wall_positions_target, rotation, translation, intrinsics, bgr
                )
                if wall_sel_positions is not None:
                    all_positions.append(wall_sel_positions)
                    all_rgb.append(wall_rgb_float)

        n = sum(p.shape[0] for p in all_positions)
        data = np.zeros((n, 4), dtype=np.float32)
        data[:, 0:3] = np.concatenate(all_positions, axis=0)
        data[:, 3] = np.concatenate(all_rgb, axis=0)

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

        if self.freeze_table_plane and self._frozen_table_cloud is None:
            self._frozen_table_cloud = cloud
            self.get_logger().info(
                'Table plane captured; freezing further updates (freeze_table_plane=true)'
            )

    @staticmethod
    def _scaled_intrinsics(camera_info, image_shape):
        """fx, fy, cx, cy rescaled to the actually-received image size, in case it
        differs from CameraInfo's own resolution (e.g. a downscaled publish)."""
        orig_height, orig_width = image_shape[:2]
        scale_x = orig_width / camera_info.width if camera_info.width else 1.0
        scale_y = orig_height / camera_info.height if camera_info.height else 1.0
        return (
            camera_info.k[0] * scale_x,
            camera_info.k[4] * scale_y,
            camera_info.k[2] * scale_x,
            camera_info.k[5] * scale_y,
        )

    @staticmethod
    def _reproject_and_sample_color(positions_target, rotation, translation, intrinsics, bgr):
        """Project target-frame points into the camera and look up their pixel
        color -- the only place depth/vision touches these positions is this
        lookup; the positions themselves come from plane geometry, not per-pixel
        depth, so the result doesn't flicker like a raw depth reconstruction."""
        fx, fy, cx, cy = intrinsics
        orig_height, orig_width = bgr.shape[:2]

        positions_cam = positions_target @ rotation.T + translation
        z_cam = positions_cam[:, 2]
        in_front = z_cam > 1e-3

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
            return None, None

        sel_positions = positions_target[in_image]
        sel_bgr = bgr[v_idx[in_image], u_idx[in_image]]
        r = sel_bgr[:, 2].astype(np.uint32)
        g = sel_bgr[:, 1].astype(np.uint32)
        b = sel_bgr[:, 0].astype(np.uint32)
        rgb_float = ((r << 16) | (g << 8) | b).view(np.float32)
        return sel_positions, rgb_float

    # 테이블 먼 쪽 끝에서 위로 서는 "벽" 격자 좌표 생성 (테이블 밖으로 나간
    # 이미지 내용이 잘리지 않도록).
    def _build_wall_points(
        self, centroid, normal, u_hat, v_hat, pu_min, pu_max, pv_min, pv_max,
        rotation, translation, intrinsics,
    ):
        """Grid of points standing straight up (along the table normal) from the
        table's far edge -- the edge farthest from the robot along base_link +X --
        so the color image content beyond the table doesn't just get cut off.

        The wall's height isn't known from the table fit (RANSAC only ever sees
        the flat table). A real depth reading straight up from the edge was tried
        first, but the sensor frequently has no valid return that far up/off to
        the side, making the wall come out too short. Instead, solve
        geometrically for the height at which the wall's own projection reaches
        the top row of the image (v=0) -- i.e. stand the wall up until it covers
        everything above the table that the camera can see, not a measured or
        assumed height.
        """
        # Normal may point either up or down depending on how RANSAC/SVD happened
        # to orient it this session -- force it up so "along the normal" from the
        # table means up, not down through the floor.
        wall_normal = normal if normal[2] >= 0.0 else -normal

        # Which in-plane axis (u or v) points away from the robot (base_link +X)
        # decides which one the wall stands up from; the other stays the wall's
        # width.
        u_dot = float(u_hat[0])
        v_dot = float(v_hat[0])
        if abs(u_dot) >= abs(v_dot):
            far_value = pu_max if u_dot >= 0.0 else pu_min
            primary_hat = u_hat
            width_min, width_max, width_hat = pv_min, pv_max, v_hat
        else:
            far_value = pv_max if v_dot >= 0.0 else pv_min
            primary_hat = v_hat
            width_min, width_max, width_hat = pu_min, pu_max, u_hat

        edge_base = centroid + far_value * primary_hat
        edge_center = edge_base + 0.5 * (width_min + width_max) * width_hat

        fx, fy, cx, cy = intrinsics
        edge_cam = edge_center @ rotation.T + translation
        if edge_cam[2] <= 1e-3:
            return None

        # Along the line edge_cam + H*normal_cam, v_px(H) = fy*y/z + cy. Solve
        # v_px(H) = 0 directly (top row) instead of scanning/sampling.
        normal_cam = wall_normal @ rotation.T
        denom = fy * normal_cam[1] + cy * normal_cam[2]
        if abs(denom) < 1e-9:
            return None
        wall_height = -(fy * edge_cam[1] + cy * edge_cam[2]) / denom
        wall_height = min(max(wall_height, self.wall_min_height), self.wall_max_height)

        alpha = self.table_plane_smoothing_alpha
        if self._wall_height_ema is None:
            self._wall_height_ema = wall_height
        else:
            self._wall_height_ema = (1.0 - alpha) * self._wall_height_ema + alpha * wall_height
        wall_height = self._wall_height_ema

        res = self.table_plane_grid_resolution
        width_vals = np.arange(width_min, width_max, res)
        height_vals = np.arange(0.0, wall_height, res)
        if width_vals.size == 0 or height_vals.size == 0:
            return None
        grid_w, grid_h = np.meshgrid(width_vals, height_vals, indexing='ij')
        grid_w = grid_w.reshape(-1)
        grid_h = grid_h.reshape(-1)

        return (
            edge_base[None, :] + grid_w[:, None] * width_hat[None, :]
            + grid_h[:, None] * wall_normal[None, :]
        )

    @staticmethod
    def _plane_basis(normal):
        reference = (
            np.array([0.0, 0.0, 1.0]) if abs(normal[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        )
        u_hat = reference - (reference @ normal) * normal
        u_hat /= np.linalg.norm(u_hat)
        v_hat = np.cross(normal, u_hat)
        return u_hat, v_hat

    # RANSAC으로 점들 중 가장 우세한 평면(테이블)을 찾아 법선/중심/inlier를 반환.
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
    # (CenterPose 검출을 RViz용 큐브 마커로 변환)

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
        now = self.get_clock().now()

        # id = 이 검출이 msg.detections에서 원래 몇 번째였는지(고정 슬롯) -- 이전엔
        # "지금까지 보정 성공한 개수"를 id로 썼는데, 그러면 앞쪽 검출 하나가 이번
        # 프레임에만 depth 보정 실패해도 뒤쪽 검출들의 id가 전부 하나씩 밀려서,
        # RViz에서 그 마커가 사라졌다 다른 id로 다시 생기는 것처럼 보여
        # 깜빡이는 원인이 됐었다.
        for index, detection in enumerate(msg.detections):
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

            self.last_marker_data[index] = {
                'frame_id': frame_id,
                'stamp': stamp,
                'position': position,
                'size': size,
                'orientation': detection.bbox.center.orientation,
                'last_seen': now,
            }

        # Redraw every id we've seen recently -- not just ones this particular message
        # refreshed -- so a single dropped/failed frame just keeps showing the last
        # known box instead of flickering it away. Only actually delete an id once
        # it's been stale longer than marker_hold_timeout.
        stale_ids = []
        for marker_id, data in self.last_marker_data.items():
            age = (now - data['last_seen']).nanoseconds * 1e-9
            if age > self.marker_hold_timeout:
                stale_ids.append(marker_id)
                continue

            box = Marker()
            box.header.frame_id = data['frame_id']
            box.header.stamp = data['stamp']
            box.ns = 'centerpose_bbox'
            box.id = marker_id
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(data['position'][0])
            box.pose.position.y = float(data['position'][1])
            box.pose.position.z = float(data['position'][2])
            box.pose.orientation = data['orientation']
            # A zero-size cube renders invisibly and can also upset RViz; floor it.
            box.scale.x = max(float(data['size'][0]), 1e-3)
            box.scale.y = max(float(data['size'][1]), 1e-3)
            box.scale.z = max(float(data['size'][2]), 1e-3)
            box.color.r, box.color.g, box.color.b, box.color.a = self.marker_color_rgba
            box.lifetime = lifetime
            markers.markers.append(box)

            if self.show_marker_axes:
                axes = Marker()
                axes.header.frame_id = data['frame_id']
                axes.header.stamp = data['stamp']
                axes.ns = 'centerpose_bbox_axes'
                axes.id = marker_id
                axes.type = Marker.LINE_LIST
                axes.action = Marker.ADD
                axes.pose.position.x = float(data['position'][0])
                axes.pose.position.y = float(data['position'][1])
                axes.pose.position.z = float(data['position'][2])
                axes.pose.orientation = data['orientation']
                axes.scale.x = self.marker_axis_width
                length = self.marker_axis_length
                origin = Point(x=0.0, y=0.0, z=0.0)
                axes.points = [
                    origin, Point(x=length, y=0.0, z=0.0),
                    origin, Point(x=0.0, y=length, z=0.0),
                    origin, Point(x=0.0, y=0.0, z=length),
                ]
                red = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                green = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                blue = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
                axes.colors = [red, red, green, green, blue, blue]
                axes.lifetime = lifetime
                markers.markers.append(axes)

        for marker_id in stale_ids:
            del self.last_marker_data[marker_id]
            delete_marker = Marker()
            delete_marker.header.frame_id = msg.header.frame_id
            delete_marker.ns = 'centerpose_bbox'
            delete_marker.id = marker_id
            delete_marker.action = Marker.DELETE
            markers.markers.append(delete_marker)

            delete_axes = Marker()
            delete_axes.header.frame_id = msg.header.frame_id
            delete_axes.ns = 'centerpose_bbox_axes'
            delete_axes.id = marker_id
            delete_axes.action = Marker.DELETE
            markers.markers.append(delete_axes)

        self.marker_pub.publish(markers)

    # CenterPose가 준 bbox 위치/크기는 부정확해서, 실제 depth로 위치를 다시 잡고
    # 크기는 측정된 비율(bbox_size_scale)로 보정.
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
        # CenterPose's own z magnitude.
        u = fx * (center.x / center.z) + cx
        v = fy * (center.y / center.z) + cy

        real_depth = self._sample_depth(depth_image, depth_msg, int(round(u)), int(round(v)))
        if real_depth is None:
            return None

        # CenterPose's reported size is off by a fixed ratio (bbox_size_scale),
        # measured against the real object -- not distance-dependent like the
        # position depth error corrected above.
        size = np.array(
            [detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z],
            dtype=np.float64,
        ) * self.bbox_size_scale

        # `real_depth` is the depth sensor's reading at the object's near (visible)
        # surface, not its volumetric center -- push the position back along the
        # same camera ray by half the corrected object size so the marker lands on
        # the center instead of floating toward the camera, off the true surface.
        center_depth = real_depth + 0.5 * float(size.mean())

        position = np.array(
            [(u - cx) * center_depth / fx, (v - cy) * center_depth / fy, center_depth],
            dtype=np.float64,
        )

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
    node = CenterposePointcloud()
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
