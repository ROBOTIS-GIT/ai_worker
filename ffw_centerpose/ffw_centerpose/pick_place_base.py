#!/usr/bin/env python3

"""centerpose_bottle / centerpose_box 등 pick-and-place 노드들이 공통으로 쓰는 로직 모음.

카메라/깊이 이미지 구독, capture/execute/cancel 서비스, MoveL·그리퍼 명령 스트리밍,
TF 조회, 쿼터니언/포즈 유틸 등 -- 각 노드마다 값(파라미터)만 다르고 로직 자체는
동일했던 부분을 여기 기반 클래스로 모아 중복을 없앴다. 검출 하나를 실제 목표
자세로 바꾸는 계산(_process_single_detection)과 실제로 팔을 움직이는 순서
(_pick_and_place 등)은 노드마다 다르므로 여기서 다루지 않고 서브클래스가 채운다.
"""

import ast
import threading
import time

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from robotis_interfaces.msg import MoveL
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros
from tf2_ros import TransformException
from vision_msgs.msg import Detection3DArray


class PickPlaceNodeBase(Node):
    """capture -> execute 흐름을 갖는 pick-and-place 노드의 공통 기반 클래스.

    서브클래스는 __init__에서 파라미터를 선언/저장한 뒤 self._setup_common()을
    호출해 TF/락/퍼블리셔/구독/서비스를 한 번에 초기화한다.
    """

    # 서브클래스가 오버라이드: capture/execute 응답 메시지에 쓰이는 복수형 표현
    # (예: 'bottle(s)', 'box(es)')
    _OBJECT_LABEL_PLURAL = 'object(s)'

    def _setup_common(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.data_lock = threading.Lock()
        self.joint_lock = threading.Lock()
        self.execution_lock = threading.Lock()
        self.cancel_event = threading.Event()

        self.bridge = CvBridge()
        self.latest_camera_info = None
        self.latest_depth_image = None
        self.latest_depth_msg = None
        self.latest_detection_msg = None
        self.latest_detection_time = None
        self.captured_queue = []
        self.current_joint_positions = {}
        self.execution_step = 'idle'

        self.movel_pub = self.create_publisher(MoveL, self.movel_topic, 10)
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, self.left_arm_joint_trajectory_topic, 10
        )
        self.object_pose_pub = self.create_publisher(PoseStamped, '~/object_pose', 10)
        self.grasp_pose_pub = self.create_publisher(PoseStamped, '~/grasp_pose', 10)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Detection3DArray, self.detections_topic, self._detections_callback, sensor_qos
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_callback, sensor_qos
        )
        self.create_subscription(Image, self.depth_topic, self._depth_callback, sensor_qos)
        self.create_subscription(
            JointState,
            self.get_parameter('joint_states_topic').value,
            self._joint_state_callback,
            10,
        )

        self.create_service(Trigger, '~/capture', self._capture_callback)
        self.create_service(Trigger, '~/execute', self._execute_callback)
        self.create_service(Trigger, '~/cancel', self._cancel_callback)

    # --- 구독 콜백: 최신 값만 저장해두고 실제 처리는 capture 시점에 ---
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
            self.latest_detection_msg = msg
            self.latest_detection_time = self.get_clock().now()

    def _joint_state_callback(self, msg):
        with self.joint_lock:
            for name, position in zip(msg.name, msg.position):
                self.current_joint_positions[name] = float(position)

    # --- 서비스 콜백 ---
    def _capture_callback(self, _request, response):
        results = self._process_detections(log=True)
        if not results:
            response.success = False
            response.message = 'no fresh CenterPose detections, camera info, or TF'
            return response

        self.captured_queue = results
        lines = [self._format_capture_line(item) for item in results]
        response.success = True
        response.message = (
            f'captured {len(results)} {self._OBJECT_LABEL_PLURAL}: ' + '; '.join(lines)
        )
        return response

    def _format_capture_line(self, item):
        """~/capture 응답에 각 검출을 요약하는 한 줄. 필요하면 서브클래스가 오버라이드."""
        p = item['pose'].pose.position
        return f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f})'

    def _execute_callback(self, _request, response):
        if not self.execute_motion:
            response.success = False
            response.message = 'set execute_motion:=true to enable robot motion'
            return response
        if not self.captured_queue:
            response.success = False
            response.message = 'call ~/capture while objects are detected first'
            return response
        if self.execution_lock.locked():
            response.success = False
            response.message = f'already executing: {self.execution_step}'
            return response

        unreachable = [
            item for item in self.captured_queue
            if item['pose'].pose.position.x >= self.max_grasp_x
        ]
        if unreachable:
            details = ', '.join(f'x={item["pose"].pose.position.x:.3f}' for item in unreachable)
            response.success = False
            response.message = (
                f'refusing to move: captured x >= {self.max_grasp_x:.2f}m '
                f'(likely bad depth): {details}. Re-capture and check depth.'
            )
            return response

        self.cancel_event.clear()
        queue = self.captured_queue
        self.captured_queue = []
        thread = threading.Thread(target=self._execute_queue, args=(queue,), daemon=True)
        thread.start()

        response.success = True
        response.message = (
            f'started picking {len(queue)} {self._OBJECT_LABEL_PLURAL} one at a time'
        )
        return response

    def _cancel_callback(self, _request, response):
        self.cancel_event.set()
        response.success = True
        response.message = 'cancel requested' if self.execution_lock.locked() else 'idle'
        return response

    def _process_detections(self, log):
        with self.data_lock:
            detections = self.latest_detection_msg
            detection_time = self.latest_detection_time
            camera_info = self.latest_camera_info
            depth_image = self.latest_depth_image
            depth_msg = self.latest_depth_msg

        if detections is None or detection_time is None:
            return []

        age = (self.get_clock().now() - detection_time).nanoseconds * 1e-9
        if age > self.detection_timeout:
            if log:
                self.get_logger().warn(f'CenterPose detection is stale ({age:.2f} s)')
            return []

        if not detections.detections or camera_info is None:
            return []

        if depth_image is None or depth_msg is None:
            if log:
                self.get_logger().warn('No depth image received yet', throttle_duration_sec=2.0)
            return []

        camera_frame = self.projection_frame or camera_info.header.frame_id
        camera_transform = self._lookup_camera_transform(camera_frame, log)
        if camera_transform is None:
            return []

        fixed_z = self._current_eef_z()
        if fixed_z is None:
            return []

        results = []
        for index, detection in enumerate(detections.detections):
            result = self._process_single_detection(
                detection, camera_info, depth_image, depth_msg,
                camera_transform, fixed_z, log, index
            )
            if result is not None:
                results.append(result)
        # 검출기 자체 출력 순서와 무관하게, 화면 왼쪽에 있는 것부터 순서대로
        # (슬롯 채우기/한 번에 하나씩 집기 순서를 안정적으로 만들기 위함).
        results.sort(key=lambda item: item['pixel_u'])
        return results

    # --- 카메라/깊이 기하 계산 ---
    def _project_to_pixel(self, camera_info, position, log):
        if abs(position.z) < 1e-6:
            if log:
                self.get_logger().warn(
                    'CenterPose z is ~0; cannot project to a pixel', throttle_duration_sec=2.0
                )
            return None

        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]
        u = fx * (position.x / position.z) + cx
        v = fy * (position.y / position.z) + cy
        return u, v

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

    def _lookup_camera_transform(self, camera_frame, log):
        """base_link <- camera_frame을 검출 배치당 한 번만 조회 (Detection3DArray
        안의 모든 검출이 같은 camera_info를 공유하므로)."""
        if not camera_frame:
            self.get_logger().warn(
                'CameraInfo frame_id is empty; set projection_frame explicitly',
                throttle_duration_sec=2.0,
            )
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, camera_frame, rclpy.time.Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot transform {camera_frame} -> {self.target_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        transform_q = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
        transform_norm = np.linalg.norm(transform_q)
        if transform_norm < 1e-9:
            if log:
                self.get_logger().warn('Camera TF contains a zero quaternion')
            return None
        transform_q /= transform_norm
        return transform_q, np.array([t.x, t.y, t.z], dtype=np.float64)

    def _current_eef_z(self):
        if self.fixed_grasp_z >= 0.0:
            return self.fixed_grasp_z

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, self.eef_link, rclpy.time.Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot get fixed grasp height from {self.eef_link}: {exc}',
                throttle_duration_sec=2.0,
            )
            return None
        return float(transform.transform.translation.z)

    # --- MoveL / 그리퍼 명령 ---
    def _move_l(self, pose, duration=None):
        if not self._wait_for_subscriber(self.movel_pub, self.movel_topic):
            return False

        duration = self.movel_duration if duration is None else duration
        msg = MoveL()
        msg.pose = pose
        msg.time_from_start = self._duration(duration)
        self.movel_pub.publish(msg)
        return self._cancelable_sleep(duration + self.settle_time)

    def _move_gripper(self, target):
        deadline = time.monotonic() + 2.0
        positions = None
        while time.monotonic() < deadline and positions is None:
            with self.joint_lock:
                current = [
                    self.current_joint_positions.get(name) for name in self.left_arm_joint_names
                ]
            if all(value is not None for value in current):
                positions = [float(value) for value in current]
            else:
                time.sleep(0.05)

        if positions is None:
            missing = []
            with self.joint_lock:
                for name in self.left_arm_joint_names:
                    if name not in self.current_joint_positions:
                        missing.append(name)
            self.get_logger().error('Missing joint states: ' + ', '.join(missing))
            return False

        if not self._wait_for_subscriber(
            self.trajectory_pub, self.left_arm_joint_trajectory_topic
        ):
            return False

        positions[self.left_arm_joint_names.index(self.left_gripper_joint)] = float(target)

        self.get_logger().info(
            f'Gripper target {self.left_gripper_joint}={float(target):.3f} on '
            f'{self.left_arm_joint_trajectory_topic}'
        )
        return self._stream_trajectory(
            self.trajectory_pub,
            self.left_arm_joint_names,
            lambda _elapsed: positions,
            self.command_rate_hz,
            self.gripper_duration + self.gripper_settle_time,
        )

    def _stream_trajectory(self, publisher, joint_names, position_fn, rate_hz, total_duration):
        """position_fn(elapsed_seconds)을 rate_hz로 total_duration 동안 계속 재전송.

        MoveL/MoveJ 컨트롤러가 같은 토픽에 ~100Hz로 계속 자기 자세를 스트리밍하기
        때문에, 한 번만 보내는 명령은 거의 즉시 덮어써진다 -- 그래서 여기서 목표를
        계속 재전송한다.
        """
        period = 1.0 / rate_hz
        point_time = max(period * 2.0, 0.05)
        point_duration = self._duration(point_time)
        start_time = time.monotonic()
        deadline = start_time + total_duration
        while time.monotonic() < deadline:
            if self.cancel_event.is_set():
                return False
            elapsed = time.monotonic() - start_time
            trajectory = JointTrajectory()
            trajectory.joint_names = joint_names
            point = JointTrajectoryPoint()
            point.positions = position_fn(elapsed)
            point.time_from_start = point_duration
            trajectory.points = [point]
            publisher.publish(trajectory)
            time.sleep(period)
        return True

    def _wait_for_subscriber(self, publisher, topic):
        deadline = time.monotonic() + self.movel_subscriber_timeout
        while time.monotonic() < deadline:
            if self.cancel_event.is_set():
                return False
            if publisher.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        self.get_logger().error(f'No subscriber on {topic}')
        return False

    def _cancelable_sleep(self, seconds):
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            if self.cancel_event.is_set():
                return False
            time.sleep(max(0.0, min(0.05, deadline - time.monotonic())))
        return True

    # --- 파라미터 읽기 유틸 ---
    def _list_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return list(ast.literal_eval(value))
        return list(value)

    def _bool_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    # --- 쿼터니언 / 포즈 유틸 ---
    @staticmethod
    def _quaternion_message(q):
        msg = PoseStamped().pose.orientation
        norm = np.linalg.norm(q)
        msg.x, msg.y, msg.z, msg.w = [float(value / norm) for value in q]
        return msg

    @staticmethod
    def _rotate_vector(vector, q):
        q_vector = q[:3]
        uv = np.cross(q_vector, vector)
        uuv = np.cross(q_vector, uv)
        return vector + 2.0 * (q[3] * uv + uuv)

    @staticmethod
    def _copy_pose(source):
        pose = PoseStamped()
        pose.header = source.header
        pose.pose.position.x = source.pose.position.x
        pose.pose.position.y = source.pose.position.y
        pose.pose.position.z = source.pose.position.z
        pose.pose.orientation = source.pose.orientation
        return pose

    @staticmethod
    def _set_pose_from_arrays(pose, xyz, xyzw):
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        pose.pose.orientation.x = float(xyzw[0])
        pose.pose.orientation.y = float(xyzw[1])
        pose.pose.orientation.z = float(xyzw[2])
        pose.pose.orientation.w = float(xyzw[3])

    @staticmethod
    def _duration(seconds):
        duration = MoveL().time_from_start
        duration.sec = int(seconds)
        duration.nanosec = int(seconds % 1.0 * 1000000000.0)
        return duration
