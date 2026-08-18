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
    def __init__(self):
        super().__init__('centerpose_pointcloud')

        # --- Crop ---
        self.declare_parameter('input_topic', '/zed/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('output_topic', '/centerpose/cloud_cropped')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('x_max', 1.2)
        self.declare_parameter('y_extent', 1.0)
        self.declare_parameter('z_min', -10.0)
        self.declare_parameter('z_max', 10.0)

        # --- Table plane ---
        self.declare_parameter('publish_table_plane', True)
        self.declare_parameter('table_plane_topic', '/centerpose/table_plane')
        self.declare_parameter('table_plane_distance_threshold', 0.01)
        self.declare_parameter('table_plane_ransac_iterations', 150)
        self.declare_parameter('table_plane_min_inliers', 200)
        self.declare_parameter('table_plane_max_ransac_points', 20000)
        self.declare_parameter('table_plane_normal_z_min', 0.8)
        self.declare_parameter('table_plane_smoothing_alpha', 0.15)
        self.declare_parameter('table_plane_grid_resolution', 0.004)
        self.declare_parameter('color_topic', '/zed/zed_node/left/image_rect_color/compressed')
        self.declare_parameter('below_table_margin', 0.02)
        self.declare_parameter('freeze_table_plane', False)

        # --- Wall ---
        self.declare_parameter('enable_wall', True)
        self.declare_parameter('wall_min_height', 0.02)
        self.declare_parameter('wall_max_height', 2.0)

        # --- Bbox markers ---
        self.declare_parameter('detections_topic', '/centerpose/detections')
        self.declare_parameter('camera_info_topic', '/zed/zed_node/left/camera_info')
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('depth_window', 5)
        self.declare_parameter('marker_topic', '/centerpose/bbox_markers')
        self.declare_parameter('marker_color_rgba', [1.0, 0.3, 0.0, 0.35])
        self.declare_parameter('bbox_size_scale', 0.2)
        self.declare_parameter('marker_lifetime', 2.0)
        self.declare_parameter('marker_hold_timeout', 1.0)
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

    # --- Point cloud crop + table plane detection/texturing ---

    def _lookup_rotation_translation(self, target_frame, source_frame):
        # Look up target_frame <- source_frame TF as a (rotation matrix, translation) pair.
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

    def _cloud_callback(self, msg):
        # Crop the raw cloud to the target_frame box, drop below-table points, republish.
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

    def _update_table_plane_state(self, kept_target_xyz):
        # Fit the table plane via RANSAC and smooth its pose/footprint over frames (EMA).
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
            # Normal sign alignment.
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
        # Rasterize the table plane, texture it from the color image, and publish.
        if self.freeze_table_plane and self._frozen_table_cloud is not None:
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

        table_positions_target = (
            centroid[None, :] + grid_pu[:, None] * u_hat[None, :] + grid_pv[:, None] * v_hat[None, :]
        )

        intrinsics = self._scaled_intrinsics(camera_info, bgr.shape)

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
        # Rescale fx/fy/cx/cy to the actually-received image size.
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
        # Project target-frame points into the camera and sample their pixel color.
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

    def _build_wall_points(
        self, centroid, normal, u_hat, v_hat, pu_min, pu_max, pv_min, pv_max,
        rotation, translation, intrinsics,
    ):
        # Build a grid standing up from the table's far edge, tall enough to fill the frame.
        wall_normal = normal if normal[2] >= 0.0 else -normal

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

        # Wall height solve.
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
        # Build two in-plane axes (u_hat, v_hat) perpendicular to a plane normal.
        reference = (
            np.array([0.0, 0.0, 1.0]) if abs(normal[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        )
        u_hat = reference - (reference @ normal) * normal
        u_hat /= np.linalg.norm(u_hat)
        v_hat = np.cross(normal, u_hat)
        return u_hat, v_hat

    def _fit_plane_ransac(self, points):
        # Find the dominant plane (the table) via RANSAC; return normal, centroid, inlier mask.
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
        # Build a numpy dtype mirroring the source PointCloud2's field layout.
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
        # Publish the cropped point array as a PointCloud2 with the source's field layout.
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
        # Convert a [x, y, z, w] quaternion into a 3x3 rotation matrix.
        x, y, z, w = q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float64)

    # --- Bbox markers ---

    def _camera_info_callback(self, msg):
        # Keep only the latest CameraInfo, used to correct marker position/size.
        with self.data_lock:
            self.latest_camera_info = msg

    def _depth_callback(self, msg):
        # Convert and keep only the latest depth image, used to correct marker position/size.
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
        # Correct each detection's position/size and (re)draw it as an RViz bbox marker.
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

    def _correct_bbox(self, detection, camera_info, depth_image, depth_msg):
        # Re-derive bbox position from real depth and rescale size by bbox_size_scale.
        center = detection.bbox.center.position
        if not all(math.isfinite(value) for value in (center.x, center.y, center.z)):
            return None
        if abs(center.z) < 1e-6:
            return None

        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        # Reproject and resample real depth.
        u = fx * (center.x / center.z) + cx
        v = fy * (center.y / center.z) + cy

        real_depth = self._sample_depth(depth_image, depth_msg, int(round(u)), int(round(v)))
        if real_depth is None:
            return None

        size = np.array(
            [detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z],
            dtype=np.float64,
        ) * self.bbox_size_scale

        center_depth = real_depth + 0.5 * float(size.mean())

        position = np.array(
            [(u - cx) * center_depth / fx, (v - cy) * center_depth / fy, center_depth],
            dtype=np.float64,
        )

        return position, size

    def _sample_depth(self, depth_image, depth_msg, u, v):
        # Median depth in a small window around pixel (u, v).
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
        # Convert a float number of seconds into a ROS Duration.
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
