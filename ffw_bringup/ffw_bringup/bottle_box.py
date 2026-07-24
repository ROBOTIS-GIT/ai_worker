#!/usr/bin/env python3

import ast
import math
import threading
import time

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from robotis_interfaces.msg import MoveL
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros
from tf2_ros import TransformException
from vision_msgs.msg import Detection3DArray


class BottleBox(Node):
    """Grab a CenterPose-detected bottle with the left arm and drop it in a fixed box.

    Unlike centerpose_bottle_pick, the gripper orientation never follows the
    detected object's yaw -- it stays at the single fixed orientation measured in
    the arm's bottle_ready initial pose for every step (pregrasp, grasp, lift, and
    the box drop), only the target x/y/z position changes. Left arm only.
    """

    def __init__(self):
        super().__init__('bottle_box')

        self.declare_parameter('detections_topic', '/centerpose/detections')
        self.declare_parameter('camera_info_topic', '/camera_info')
        # As in centerpose_bottle_pick: CenterPose's own depth is unreliable, so x/y
        # are recovered by projecting CenterPose's direction to a pixel and reading
        # real metric depth from this image at that pixel. z always stays fixed (see
        # fixed_grasp_z) and never comes from either source.
        self.declare_parameter('depth_topic', '/zedm/zed_node/depth/depth_registered')
        self.declare_parameter('depth_window', 5)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('projection_frame', '')
        self.declare_parameter('detection_timeout', 10.0)
        # Safety bound: a target this far forward is outside the real workspace and
        # almost always means depth was misread. ~/execute refuses to move at all if
        # any captured pose exceeds this.
        self.declare_parameter('max_grasp_x', 0.7)
        self.declare_parameter('execute_motion', False)
        self.declare_parameter('movel_topic', '/l_goal_move')
        self.declare_parameter('movel_duration', 5.0)
        # Approaching straight to the final grasp pose lets the gripper body clip the
        # bottle on the way in. Instead, back off along the gripper's own approach axis
        # (fixed, since orientation is fixed) to a pregrasp pose clear of the object,
        # then move straight in along that same axis to grasp.
        self.declare_parameter('pregrasp_distance', 0.08)
        self.declare_parameter('pregrasp_duration', 2.0)
        self.declare_parameter('insertion_duration', 1.0)
        self.declare_parameter('movel_subscriber_timeout', 2.0)
        self.declare_parameter('settle_time', 0.5)
        self.declare_parameter('eef_link', 'end_effector_l_link')
        # CenterPose height is not trusted: z always stays at this fixed height, never
        # the vision-measured object height. Negative: fall back to dynamically
        # holding the current EEF base-frame Z at capture time instead.
        self.declare_parameter('fixed_grasp_z', 0.8241714239120483)
        # Calibrated from measured error (see centerpose_bottle_pick.py).
        self.declare_parameter('grasp_position_offset', [0.01, -0.04, 0.0])
        # Fixed gripper orientation used for every step (pregrasp/grasp/lift/box) --
        # measured via tf2_echo base_link end_effector_l_link with the left arm in its
        # bottle_ready initial pose. The gripper never rolls to match the bottle.
        self.declare_parameter(
            'grasp_orientation_xyzw',
            [-0.0657237321138382, -0.6881383657455444, -0.06250208616256714, 0.7198885083198547],
        )
        self.declare_parameter(
            'left_arm_joint_trajectory_topic',
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
        )
        self.declare_parameter('left_gripper_joint', 'gripper_l_joint1')
        self.declare_parameter(
            'left_arm_joint_names',
            [
                'arm_l_joint1',
                'arm_l_joint2',
                'arm_l_joint3',
                'arm_l_joint4',
                'arm_l_joint5',
                'arm_l_joint6',
                'arm_l_joint7',
                'gripper_l_joint1',
            ],
        )
        self.declare_parameter('gripper_open_position', 0.0)
        self.declare_parameter('gripper_closed_position', 0.7)
        self.declare_parameter('gripper_duration', 0.5)
        self.declare_parameter('gripper_settle_time', 0.2)
        # The MoveL controller (cyclo_control) streams arm-only trajectory points to
        # this same topic continuously to hold its pose, so a single one-shot gripper
        # command gets pre-empted almost immediately -- the target is re-asserted at
        # this rate instead (see centerpose_bottle_pick.py).
        self.declare_parameter('command_rate_hz', 300.0)
        self.declare_parameter('lift_height', 0.1)
        self.declare_parameter('lift_duration', 1.0)
        # Where each bottle gets dropped off: first hover above its box slot, then
        # move straight down by box_place_z_offset to the release height (so the
        # bottle isn't dragged sideways into the box wall on the way down), release,
        # then straight back up to the same hover pose. There are two slots, filled
        # left-to-right by capture order (leftmost detection -> slot 1, next -> slot
        # 2); with only one bottle captured, slot 1 is used, matching the original
        # single-box behavior. Each slot has its own orientation -- slot 2 needs the
        # wrist rotated to fit its spot, unlike the fixed grasp_orientation_xyzw used
        # for picking. All measured via `ros2 topic echo --once /l_goal_pose` with
        # the arm holding a bottle above/at each slot.
        self.declare_parameter(
            'box_slot_1_position_xyz',
            [0.6468760967254639, 0.36483344435691833, 0.9507306218147278],
        )
        self.declare_parameter(
            'box_slot_1_orientation_xyzw',
            [-0.06588234007358551, -0.68890780210495, -0.06271106004714966, 0.7191194891929626],
        )
        self.declare_parameter(
            'box_slot_2_position_xyz',
            [0.6272304654121399, 0.2589269280433655, 0.9516366720199585],
        )
        self.declare_parameter(
            'box_slot_2_orientation_xyzw',
            [0.13866588473320007, -0.6788055896759033, 0.13322767615318298, 0.7086924910545349],
        )
        self.declare_parameter('box_duration', 1.5)
        # Both slots' hover -> place poses were measured 0.1093128323554992m apart in
        # Z; reused for both since only the hover pose was measured per slot.
        self.declare_parameter('box_place_z_offset', 0.1093128323554992)
        self.declare_parameter('box_place_duration', 1.0)
        self.declare_parameter('return_to_initial', True)
        # Measured via tf2_echo base_link end_effector_l_link with the left arm in its
        # bottle_ready initial pose (same pose grasp_orientation_xyzw was measured from).
        self.declare_parameter(
            'home_position_xyz',
            [0.13451801240444183, 0.2999741733074188, 0.9742214239120483],
        )
        self.declare_parameter('home_duration', 3.0)

        self.detections_topic = self.get_parameter('detections_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.depth_window = int(self.get_parameter('depth_window').value)
        self.target_frame = self.get_parameter('target_frame').value
        self.projection_frame = self.get_parameter('projection_frame').value
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        self.max_grasp_x = float(self.get_parameter('max_grasp_x').value)
        self.execute_motion = self._bool_parameter('execute_motion')
        self.movel_topic = self.get_parameter('movel_topic').value
        self.movel_duration = float(self.get_parameter('movel_duration').value)
        self.pregrasp_distance = float(self.get_parameter('pregrasp_distance').value)
        self.pregrasp_duration = float(self.get_parameter('pregrasp_duration').value)
        self.insertion_duration = float(self.get_parameter('insertion_duration').value)
        self.movel_subscriber_timeout = float(
            self.get_parameter('movel_subscriber_timeout').value
        )
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.eef_link = str(self.get_parameter('eef_link').value)
        self.fixed_grasp_z = float(self.get_parameter('fixed_grasp_z').value)

        grasp_position_offset = self._list_parameter('grasp_position_offset')
        if len(grasp_position_offset) != 3:
            raise ValueError('grasp_position_offset must contain [x, y, z]')
        self.grasp_position_offset = grasp_position_offset

        grasp_orientation = np.asarray(
            self._list_parameter('grasp_orientation_xyzw'), dtype=np.float64
        )
        if grasp_orientation.shape != (4,):
            raise ValueError('grasp_orientation_xyzw must contain [x, y, z, w]')
        orientation_norm = np.linalg.norm(grasp_orientation)
        if orientation_norm < 1e-9:
            raise ValueError('grasp_orientation_xyzw must not be zero')
        self.grasp_orientation = grasp_orientation / orientation_norm
        # Measured from the calibrated grasp orientation: local -Z rotates to
        # ~base_link +X (toward the bottle), so that's the direction of travel when
        # inserting; back off along the opposite of it to get the pregrasp point.
        # Fixed once here since the orientation itself never changes per detection.
        self.approach_dir = self._rotate_vector(
            np.array([0.0, 0.0, -1.0]), self.grasp_orientation
        )

        self.left_arm_joint_trajectory_topic = self.get_parameter(
            'left_arm_joint_trajectory_topic'
        ).value
        self.left_gripper_joint = self.get_parameter('left_gripper_joint').value
        self.left_arm_joint_names = [
            str(name) for name in self.get_parameter('left_arm_joint_names').value
        ]
        if self.left_gripper_joint not in self.left_arm_joint_names:
            raise ValueError('left_gripper_joint must be included in left_arm_joint_names')

        self.gripper_open_position = float(self.get_parameter('gripper_open_position').value)
        self.gripper_closed_position = float(
            self.get_parameter('gripper_closed_position').value
        )
        self.gripper_duration = float(self.get_parameter('gripper_duration').value)
        self.gripper_settle_time = float(self.get_parameter('gripper_settle_time').value)
        self.command_rate_hz = float(self.get_parameter('command_rate_hz').value)
        self.lift_height = float(self.get_parameter('lift_height').value)
        self.lift_duration = float(self.get_parameter('lift_duration').value)

        self.box_slots = [self._build_box_slot(1), self._build_box_slot(2)]
        self.box_duration = float(self.get_parameter('box_duration').value)
        self.box_place_z_offset = float(self.get_parameter('box_place_z_offset').value)
        self.box_place_duration = float(self.get_parameter('box_place_duration').value)

        self.return_to_initial = self._bool_parameter('return_to_initial')
        home_position_xyz = self._list_parameter('home_position_xyz')
        if len(home_position_xyz) != 3:
            raise ValueError('home_position_xyz must contain [x, y, z]')
        self.home_position_xyz = home_position_xyz
        self.home_duration = float(self.get_parameter('home_duration').value)

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
        self.create_subscription(
            Image, self.depth_topic, self._depth_callback, sensor_qos
        )
        self.create_subscription(
            JointState,
            self.get_parameter('joint_states_topic').value,
            self._joint_state_callback,
            10,
        )

        self.create_service(Trigger, '~/capture', self._capture_callback)
        self.create_service(Trigger, '~/execute', self._execute_callback)
        self.create_service(Trigger, '~/cancel', self._cancel_callback)

        self.get_logger().info('BottleBox (left arm, fixed orientation) ready')
        self.get_logger().info(f'  MoveL: {self.movel_topic}')
        self.get_logger().info(f'  detections: {self.detections_topic}')
        self.get_logger().info(f'  camera info: {self.camera_info_topic}')
        self.get_logger().info(f'  depth (x/y only): {self.depth_topic}')
        for index, slot in enumerate(self.box_slots, start=1):
            self.get_logger().info(f'  box slot {index} hover: {slot["position"]}')

    def _build_box_slot(self, index):
        position = self._list_parameter(f'box_slot_{index}_position_xyz')
        if len(position) != 3:
            raise ValueError(f'box_slot_{index}_position_xyz must contain [x, y, z]')

        orientation = np.asarray(
            self._list_parameter(f'box_slot_{index}_orientation_xyzw'), dtype=np.float64
        )
        if orientation.shape != (4,):
            raise ValueError(f'box_slot_{index}_orientation_xyzw must contain [x, y, z, w]')
        norm = np.linalg.norm(orientation)
        if norm < 1e-9:
            raise ValueError(f'box_slot_{index}_orientation_xyzw must not be zero')

        return {'position': position, 'orientation': orientation / norm}

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

    def _capture_callback(self, _request, response):
        results = self._process_detections(log=True)
        if not results:
            response.success = False
            response.message = 'no fresh CenterPose detections, camera info, or TF'
            return response

        self.captured_queue = results
        lines = []
        for item in results:
            p = item['pose'].pose.position
            lines.append(f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
        response.success = True
        response.message = f'captured {len(results)} bottle(s): ' + '; '.join(lines)
        return response

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
        response.message = f'started picking {len(queue)} bottle(s) one at a time'
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
        # Leftmost-in-image first, so box slots fill left-to-right in a stable order
        # regardless of the detector's own output order.
        results.sort(key=lambda item: item['pixel_u'])
        return results

    def _process_single_detection(
        self, detection, camera_info, depth_image, depth_msg,
        camera_transform, fixed_z, log, index
    ):
        center = detection.bbox.center.position
        if not all(math.isfinite(value) for value in (center.x, center.y, center.z)):
            return None

        projected = self._project_to_pixel(camera_info, center, log)
        if projected is None:
            return None
        u, v = projected

        real_point_cam = self._real_camera_point(
            camera_info, u, v, depth_image, depth_msg, log
        )
        if real_point_cam is None:
            return None

        transform_q, t = camera_transform
        point = self._rotate_vector(np.asarray(real_point_cam, dtype=np.float64), transform_q) + t

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.target_frame
        pose.pose.position.x = float(point[0] + self.grasp_position_offset[0])
        pose.pose.position.y = float(point[1] + self.grasp_position_offset[1])
        pose.pose.position.z = fixed_z
        pose.pose.orientation = self._quaternion_message(self.grasp_orientation)

        self.object_pose_pub.publish(pose)
        self.grasp_pose_pub.publish(pose)

        if log:
            self.get_logger().info(
                f'[{index}] pixel=({u:.0f}, {v:.0f})/{camera_info.width}, '
                f'real camera xyz=({real_point_cam[0]:.3f}, {real_point_cam[1]:.3f}, '
                f'{real_point_cam[2]:.3f}), fixed z={fixed_z:.3f}, target=('
                f'{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, '
                f'{pose.pose.position.z:.3f})'
            )
        return {'pose': pose, 'pixel_u': u}

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

    def _real_camera_point(self, camera_info, u, v, depth_image, depth_msg, log):
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        u_px = int(round(u))
        v_px = int(round(v))

        depth = self._sample_depth(depth_image, depth_msg, u_px, v_px)
        if depth is None:
            if log:
                self.get_logger().warn(
                    f'Invalid depth around pixel ({u_px}, {v_px})', throttle_duration_sec=2.0
                )
            return None

        return np.array(
            [(u - cx) * depth / fx, (v - cy) * depth / fy, depth], dtype=np.float64
        )

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

    def _execute_queue(self, queue):
        with self.execution_lock:
            try:
                for index, item in enumerate(queue):
                    if self.cancel_event.is_set():
                        return
                    label = f'bottle {index + 1}/{len(queue)}'
                    # Extra bottles beyond the number of physical slots reuse the
                    # last slot -- there's nowhere else defined to put them.
                    slot = self.box_slots[min(index, len(self.box_slots) - 1)]
                    if not self._pick_and_place(item['pose'], slot, label):
                        if self.cancel_event.is_set():
                            return
                        self.get_logger().error(f'{label} failed; continuing with next bottle')
                        continue
                self.get_logger().info(f'Finished picking {len(queue)} bottle(s)')
            finally:
                self.execution_step = 'idle'

    def _pick_and_place(self, grasp_pose, slot, label='bottle'):
        pregrasp_pose = self._copy_pose(grasp_pose)
        pregrasp_pose.pose.position.x -= self.approach_dir[0] * self.pregrasp_distance
        pregrasp_pose.pose.position.y -= self.approach_dir[1] * self.pregrasp_distance
        pregrasp_pose.pose.position.z -= self.approach_dir[2] * self.pregrasp_distance

        lift_pose = self._copy_pose(grasp_pose)
        lift_pose.pose.position.z += self.lift_height

        box_pose = PoseStamped()
        box_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(box_pose, slot['position'], slot['orientation'])

        box_place_pose = self._copy_pose(box_pose)
        box_place_pose.pose.position.z -= self.box_place_z_offset

        home_pose = PoseStamped()
        home_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(home_pose, self.home_position_xyz, self.grasp_orientation)

        steps = [
            ('open gripper', lambda: self._move_gripper(self.gripper_open_position)),
            ('move to pregrasp', lambda: self._move_l(pregrasp_pose, duration=self.pregrasp_duration)),
            ('insert to bottle', lambda: self._move_l(grasp_pose, duration=self.insertion_duration)),
            ('close gripper', lambda: self._move_gripper(self.gripper_closed_position)),
            ('lift bottle', lambda: self._move_l(lift_pose, duration=self.lift_duration)),
            ('move above box', lambda: self._move_l(box_pose, duration=self.box_duration)),
            ('lower into box', lambda: self._move_l(box_place_pose, duration=self.box_place_duration)),
            ('release in box', lambda: self._move_gripper(self.gripper_open_position)),
            # Retreat straight back up to the hover pose before heading home, rather
            # than pulling out low and close to the box -- less risk of clipping it.
            ('raise back above box', lambda: self._move_l(box_pose, duration=self.box_place_duration)),
        ]
        if self.return_to_initial:
            steps.append(
                ('return to initial pose', lambda: self._move_l(home_pose, duration=self.home_duration))
            )

        for name, command in steps:
            if self.cancel_event.is_set():
                return False
            self.execution_step = f'{label}: {name}'
            self.get_logger().info(f'Executing {label} {name}')
            if not command():
                self.get_logger().error(f'Stopped at {label} {name}')
                return False
        self.get_logger().info(f'{label} finished; holding pose')
        return True

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
        """Re-publish position_fn(elapsed_seconds) at rate_hz for total_duration.

        A single one-shot trajectory message gets pre-empted almost immediately by the
        MoveL/MoveJ controller's own ~100 Hz stream to the same topic, so the target is
        continuously re-asserted here instead of published once.
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

    def _lookup_camera_transform(self, camera_frame, log):
        """Resolve base_link <- camera_frame once per detection batch (it's the same
        for every detection in a Detection3DArray, since they share one camera_info)."""
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
            time.sleep(min(0.05, deadline - time.monotonic()))
        return True

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


def main(args=None):
    rclpy.init(args=args)
    node = BottleBox()
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
