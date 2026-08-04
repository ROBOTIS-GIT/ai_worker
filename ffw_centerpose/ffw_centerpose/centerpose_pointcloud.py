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

# PointField.datatype -> numpy 타입 매핑. 원본 클라우드의 필드 구성을 그대로
# 살려서(패딩 포함) 크롭해도 rgb/intensity 등 모든 필드가 그대로 유지되게 함.
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
    """작업 공간 포인트클라우드 도구: 크롭 + 테이블 평면 시각화 + 검출 마커.

    하나의 노드에 세 기능을 합쳐놓음:
    1. 크롭: target_frame 기준 박스 안(로봇 앞쪽)의 점만 남기고, 테이블
       평면을 찾은 뒤엔 그 아래 점도 제거.
    2. 테이블 평면: 크롭된 점들로 RANSAC 평면 피팅 후 여러 프레임에 걸쳐
       부드럽게(smoothing), 실제 depth 대신 카메라 이미지로 색을 입힌
       평평한 격자로 다시 그림. enable_wall이면 테이블 먼 쪽 가장자리에
       벽면도 하나 더 세워서 화면이 잘려 보이지 않게 함.
    3. Bbox 마커: CenterPose 검출을 RViz 큐브로 표시하되, pick 노드들과
       같은 방식으로 위치/크기를 보정 (CenterPose 원본 값은 부정확함).
    """

    # 파라미터 선언/읽기, 퍼블리셔/구독 설정까지 전부 이 안에서 처리.
    def __init__(self):
        super().__init__('centerpose_pointcloud')

        # --- 크롭 ---
        self.declare_parameter('input_topic', '/zedm/zed_node/point_cloud/cloud_registered')
        # marker_topic(/centerpose/bbox_markers)과 같은 네임스페이스에 둬서 RViz에서
        # bbox 마커/크롭 클라우드/테이블 평면이 한 카테고리로 묶여 보이게 함.
        self.declare_parameter('output_topic', '/centerpose/cloud_cropped')
        self.declare_parameter('target_frame', 'base_link')
        # 전방 범위: 0 <= x <= x_max인 점만 남김 (base_link +X가 전방).
        self.declare_parameter('x_max', 1.2)
        # 좌우 범위: -y_extent <= y <= y_extent (base_link +Y가 왼쪽). 0 이하면
        # 좌우는 크롭 안 함.
        self.declare_parameter('y_extent', 1.0)
        # 높이 범위는 기본적으로 거의 무제한 -- 천장/바닥도 잘라야 하면 좁히기.
        self.declare_parameter('z_min', -10.0)
        self.declare_parameter('z_max', 10.0)

        # --- 테이블 평면 ---
        self.declare_parameter('publish_table_plane', True)
        self.declare_parameter('table_plane_topic', '/centerpose/table_plane')
        # 피팅된 평면과의 거리(m)가 이 안이면 "테이블 위"로 간주.
        self.declare_parameter('table_plane_distance_threshold', 0.01)
        self.declare_parameter('table_plane_ransac_iterations', 150)
        self.declare_parameter('table_plane_min_inliers', 200)
        # RANSAC이 크롭된 점들 중 최대 이만큼만 샘플링 (입력이 조밀해도 계산량 고정).
        self.declare_parameter('table_plane_max_ransac_points', 20000)
        # 피팅된 평면이 수평(base_link Z)에서 이 이상 기울어져 있으면 버림 --
        # 벽이나 로봇 팔 같은 테이블 아닌 평면에 잘못 걸리는 걸 방지.
        self.declare_parameter('table_plane_normal_z_min', 0.8)
        # 프레임마다 RANSAC 결과가 약간씩 흔들려서, 매번 새로 그리면 화면이
        # 깜빡거림 -- 이전 추정치와 섞어서(EMA) 부드럽게 함 (클수록 빠르게 추적,
        # 대신 더 흔들림).
        self.declare_parameter('table_plane_smoothing_alpha', 0.15)
        # 출력 격자의 셀 크기(m) -- 노이즈 있는 원본 점 하나하나가 아니라, 부드럽게
        # 다듬은 평면 위에 일정 간격 격자로 다시 그리므로 "이미지 해상도"에 가까움.
        self.declare_parameter('table_plane_grid_resolution', 0.004)
        # 격자의 색은 depth가 아니라 컬러 이미지를 카메라 자세로 실제 평면에
        # 재투영해서 입힘.
        self.declare_parameter('color_topic', '/zedm/zed_node/left/image_rect_color/compressed')
        # 다듬어진 테이블 높이보다 이만큼 이상 아래인 점은 제거 (다리/바닥/로봇
        # 몸체 등). 0이 아니라 약간 여유를 둬서 테이블 자체의 노이즈 점까지
        # 같이 잘리는 걸 방지.
        self.declare_parameter('below_table_margin', 0.02)
        # 한 번 테이블 격자가 만들어지면, 그 뒤로는 매 프레임 다시 계산하지 않고
        # 그 스냅샷(위치+색)을 계속 그대로 재발행 -- 화면을 고정시킴.
        self.declare_parameter('freeze_table_plane', False)

        # --- 벽면 (테이블 먼 쪽 가장자리에서 위로 세움) ---
        # 테이블 래스터는 테이블 자기 영역만 덮으므로, 그 너머 화면이 그냥 잘려
        # 보이지 않게 같은 방식으로 텍스처 입힌 벽면을 하나 더 세움.
        self.declare_parameter('enable_wall', True)
        # 벽 높이는 테이블 피팅만으론 알 수 없어서 매 프레임 기하학적으로 계산
        # (화면 맨 위까지 닿도록). 안전을 위해 이 범위로 clamp.
        self.declare_parameter('wall_min_height', 0.02)
        self.declare_parameter('wall_max_height', 2.0)

        # --- Bbox 마커 ---
        self.declare_parameter('detections_topic', '/centerpose/detections')
        self.declare_parameter('camera_info_topic', '/zedm/zed_node/left/camera_info')
        self.declare_parameter('depth_topic', '/zedm/zed_node/depth/depth_registered')
        self.declare_parameter('depth_window', 5)
        self.declare_parameter('marker_topic', '/centerpose/bbox_markers')
        self.declare_parameter('marker_color_rgba', [1.0, 0.3, 0.0, 0.35])
        # CenterPose의 bbox.size는 실제보다 약 5배 크게 나와서, 거리와 무관한
        # 고정 비율로 이렇게 축소해서 씀.
        self.declare_parameter('bbox_size_scale', 0.2)
        # 마커는 이 시간 뒤 자동 소멸 -- CenterPose 발행 주기가 들쭉날쭉해서 너무
        # 짧으면 검출 사이사이에 마커가 깜빡거림.
        self.declare_parameter('marker_lifetime', 2.0)
        # CenterPose가 한 프레임만 검출을 놓쳐도 물체는 그대로 있는 경우가 많아서,
        # 바로 지우지 않고 이 시간까지는 마지막 위치로 계속 그려줌 (깜빡임 방지).
        self.declare_parameter('marker_hold_timeout', 1.0)
        # 각 박스 중심에 RGB 축(XYZ)을 그려서 위치뿐 아니라 방향도 한눈에 보이게 함.
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

    # --- 포인트클라우드 크롭 + 테이블 평면 감지/텍스처링 ---

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
            # RANSAC/SVD가 프레임마다 법선 부호를 뒤집을 수 있어서, 섞기 전에
            # 기존 추정치와 같은 방향으로 맞춤.
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
            # 이미 스냅샷을 찍었으면 그대로 재발행만 (실시간 카메라를 안 따라감).
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
        # 크롭에서 쓰는 조회의 역방향 (camera<-target) -- target 좌표를 바로
        # camera 좌표로 바꿔줌.
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

        # 격자 점 위치는 순수 기하 계산(centroid/basis)으로만 정해짐 -- 노이즈
        # 있는 depth를 직접 쓰지 않음.
        table_positions_target = (
            centroid[None, :] + grid_pu[:, None] * u_hat[None, :] + grid_pv[:, None] * v_hat[None, :]
        )

        intrinsics = self._scaled_intrinsics(camera_info, bgr.shape)

        # 각 격자점을 카메라에 재투영해서 색만 조회 (위치 결정에는 안 쓰임).
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

    # camera_info의 해상도와 실제 받은 이미지 해상도가 다를 때 fx/fy/cx/cy를 비례 조정.
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

    # 평면 위 3D 점들을 카메라에 재투영해서 그 픽셀의 색을 읽어옴 (색 입히기 전용).
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
        """테이블 먼 쪽 가장자리에서 법선 방향으로 곧게 세운 격자.

        벽 높이는 테이블 피팅만으론 알 수 없음. 가장자리에서 실측 depth를
        읽는 방법도 시도했으나 그쪽 방향엔 유효한 depth가 없는 경우가 많아서
        벽이 너무 짧아짐 -- 대신 벽의 투영이 화면 맨 위(v=0)에 닿는 높이를
        기하학적으로 직접 풀어서 구함.
        """
        # 이번 세션에 RANSAC/SVD가 법선을 위/아래 어느 쪽으로 정했을지 몰라서,
        # 항상 위쪽을 향하도록 강제.
        wall_normal = normal if normal[2] >= 0.0 else -normal

        # 로봇 반대쪽(base_link +X)을 가리키는 축(u 또는 v)이 벽이 서는 방향이고,
        # 나머지 축은 벽의 폭이 됨.
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

        # edge_cam + H*normal_cam 선 위에서 v_px(H) = fy*y/z + cy. v_px(H)=0(맨
        # 위 행)이 되는 H를 스캔 없이 바로 수식으로 풀이.
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

    # 평면의 법선(normal) 하나로부터, 그 평면 위에서 쓸 두 축(u_hat, v_hat)을 만듦.
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

        # 샘플링 서브셋이 아니라 전체 inlier로 최소자승 재피팅해서 더 정밀한 평면 구함.
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

    # 원본 PointCloud2 메시지의 필드 구성(x/y/z/rgb 등)을 그대로 반영한 numpy dtype 생성.
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

    # 크롭된 포인트 배열을 원본과 같은 필드 구성의 PointCloud2로 만들어 발행.
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

    # 쿼터니언을 3x3 회전행렬로 변환.
    @staticmethod
    def _quaternion_to_matrix(q):
        x, y, z, w = q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float64)

    # --- Bbox 마커 (CenterPose 검출을 RViz 큐브 마커로 변환) ---
    # (CenterPose 검출을 RViz용 큐브 마커로 변환)

    # 마커 보정용 CameraInfo는 최신 것만 저장.
    def _camera_info_callback(self, msg):
        with self.data_lock:
            self.latest_camera_info = msg

    # 마커 보정용 depth 이미지를 변환해서 최신 것만 저장.
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

    # CenterPose 검출이 올 때마다 위치/크기를 보정해서 bbox 마커로 다시 그림.
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

        # 이번 메시지에서 갱신된 것뿐 아니라 최근에 본 id는 전부 다시 그림 -- 한
        # 프레임 놓쳐도 마지막 위치를 계속 보여줘서 깜빡임을 막음. marker_hold_timeout
        # 보다 오래 안 보이면 그때 진짜로 삭제.
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
            # 크기가 0인 큐브는 안 보이고 RViz도 이상해질 수 있어 최소값을 보장.
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

        # CenterPose의 (x,y,z)는 스케일은 틀려도 방향(bearing)은 맞음 -- 그 방향을
        # 픽셀로 투영해서 실제 depth를 다시 읽음 (CenterPose의 z값은 안 믿음).
        u = fx * (center.x / center.z) + cx
        v = fy * (center.y / center.z) + cy

        real_depth = self._sample_depth(depth_image, depth_msg, int(round(u)), int(round(v)))
        if real_depth is None:
            return None

        # CenterPose가 보고하는 크기는 고정 비율(bbox_size_scale)만큼 틀림 --
        # 위의 위치 depth 오차와 달리 거리에 따라 달라지지 않음.
        size = np.array(
            [detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z],
            dtype=np.float64,
        ) * self.bbox_size_scale

        # real_depth는 물체 앞면(보이는 면)까지의 거리라 중심이 아님 -- 보정된
        # 크기의 절반만큼 카메라 반대 방향으로 밀어서 중심 위치를 추정.
        center_depth = real_depth + 0.5 * float(size.mean())

        position = np.array(
            [(u - cx) * center_depth / fx, (v - cy) * center_depth / fy, center_depth],
            dtype=np.float64,
        )

        return position, size

    # 픽셀 (u, v) 주변 depth_window 크기 영역에서 유효한 depth 값들의 중앙값을 샘플링.
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

    # 초(float)를 ROS Duration(sec, nanosec)으로 변환.
    @staticmethod
    def _duration(seconds):
        duration = Duration()
        duration.sec = int(seconds)
        duration.nanosec = int((seconds % 1.0) * 1e9)
        return duration


def main(args=None):
    rclpy.init(args=args)
    node = CenterposePointcloud()  # 노드 실행 진입점 (ros2 run/launch에서 호출됨)
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
