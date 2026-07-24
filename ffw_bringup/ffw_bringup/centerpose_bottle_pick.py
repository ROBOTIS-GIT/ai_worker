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


class CenterPoseBottlePick(Node):
    """Drive an open gripper directly to a CenterPose 3D bottle pose.

    The arm is chosen per detection from which side of the image the bottle falls
    on (left side -> left arm, right side -> right arm), not fixed at startup.
    """

    def __init__(self):
        super().__init__('centerpose_bottle_pick')

        self.declare_parameter('detections_topic', '/centerpose/detections')
        self.declare_parameter('camera_info_topic', '/camera_info')
        # CenterPose's own depth (position magnitude) is unreliable -- it's derived from
        # an assumed canonical object size, not a real measurement, and can be off by
        # several times. x/y are instead recovered by projecting CenterPose's direction
        # (which stays valid even when its scale is wrong) to a pixel and reading real
        # metric depth from this image at that pixel. z always stays fixed (see
        # fixed_grasp_z / _current_eef_z) and never comes from either source.
        self.declare_parameter('depth_topic', '/zedm/zed_node/depth/depth_registered')
        self.declare_parameter('depth_window', 5)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('projection_frame', '')
        # Used only as the tie-break arm when the bottle's pixel falls inside the
        # center deadband (see arm_center_deadband_ratio) and cannot be confidently
        # assigned to either side.
        self.declare_parameter('arm', 'left')
        # Fraction of image width around the center where the side is ambiguous;
        # detections inside this band fall back to "arm".
        self.declare_parameter('arm_center_deadband_ratio', 0.1)
        self.declare_parameter('detection_timeout', 10.0)
        # Safety bound: a target this far forward is outside the real workspace and
        # almost always means depth was misread (e.g. a background hit instead of the
        # bottle -- see the 3.5m target seen with a bad depth reading). ~/execute
        # refuses to move at all if any captured pose exceeds this.
        self.declare_parameter('max_grasp_x', 0.7)
        self.declare_parameter('execute_motion', False)
        self.declare_parameter('movel_topic', '/l_goal_move')
        self.declare_parameter('right_movel_topic', '/r_goal_move')
        self.declare_parameter('movel_duration', 6.0)
        # Approaching straight to the final grasp pose lets the gripper body clip the
        # bottle on the way in. Instead, back off along the gripper's own approach axis
        # (local +Z, rotated into base_link by the grasp orientation) to a pregrasp pose
        # clear of the object, then move straight in along that same axis to grasp.
        self.declare_parameter('pregrasp_distance', 0.08)
        self.declare_parameter('pregrasp_duration', 5.0)
        self.declare_parameter('insertion_duration', 2.0)
        self.declare_parameter('movel_subscriber_timeout', 2.0)
        self.declare_parameter('settle_time', 0.5)
        self.declare_parameter('eef_link', 'end_effector_l_link')
        self.declare_parameter('right_eef_link', 'end_effector_r_link')
        # CenterPose height is not trusted: z always stays at this fixed height, never
        # the vision-measured object height. Negative: fall back to the EEF height
        # captured from the default/ready posture at capture time (dynamic). Pinned to
        # a literal instead once the ready posture itself moves (e.g. lift height
        # changes) but the table it grasps from does not -- see home_position_xyz.
        self.declare_parameter('fixed_grasp_z', -1.0)
        self.declare_parameter('right_fixed_grasp_z', -1.0)
        # Calibrated from measured error (after fixing the zedm TF root cause): reached
        # ~1cm too far forward (+X) and ~5cm too far left (+Y) versus the real bottle
        # position. (-0.10 on Y overshot to the right, so this is back down from there.)
        # right_* values below are placeholders mirrored from the left calibration --
        # they have NOT been separately measured for the right gripper and must be
        # re-tuned on the real robot before relying on them.
        self.declare_parameter('grasp_position_offset', [-0.02, 0.0, 0.0])
        self.declare_parameter('right_grasp_position_offset', [-0.02, 0.0, 0.0])
        self.declare_parameter('tool_orientation_offset_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('right_tool_orientation_offset_xyzw', [0.0, 0.0, 0.0, 1.0])
        # Grasp orientation keeps pitch/yaw fixed and only rolls the gripper to match
        # the object's yaw. Calibrated from two measured /l_gripper_pose targets: bottle
        # facing the robot front (raw yaw -80.9 deg) and bottle at -51.1 deg display yaw
        # (raw yaw -132.0 deg) -> pitch/yaw stayed ~constant across both, only roll moved.
        self.declare_parameter('grasp_fixed_pitch', -1.5256446590458033)
        # Right-gripper equivalents, fit from 3 measured /r_goal_pose targets at object
        # (raw base-link) yaws of -66.8/-80.7/-159.2 deg -> roll 1.0/-11.3/-48.1 deg;
        # pitch/yaw held constant across all 3 (-86.95/10.16 deg), same as the left rig.
        self.declare_parameter('right_grasp_fixed_pitch', -1.5176096004888053)
        self.declare_parameter('grasp_fixed_yaw', 0.010559241974565694)
        self.declare_parameter('right_grasp_fixed_yaw', 0.17740065731475654)
        self.declare_parameter('grasp_roll_from_yaw_scale', 0.6622504892367908)
        self.declare_parameter('right_grasp_roll_from_yaw_scale', 0.5103027930797728)
        self.declare_parameter('grasp_roll_offset', 0.9278879706509177)
        self.declare_parameter('right_grasp_roll_offset', 0.5713675639841987)
        # Display-only: reported bottle yaw = object_yaw - this offset, wrapped to
        # (-180, 180]. Tune so a bottle facing the robot's front (base_link +X) reads 0.
        # Shared across arms -- it describes the camera/object relationship, not the arm.
        self.declare_parameter('bottle_yaw_zero_offset_deg', -80.9)
        # grasp_roll_from_yaw_scale/offset is a linear fit only validated across a
        # narrow measured window (yaw 0 to -49 deg -> roll ~0 to -33 deg); extrapolated
        # far outside that, the raw roll swings into mechanically nonsense angles. The
        # gripper's roll is clamped to this known-good [min, max] range instead --
        # values outside it saturate at the nearer bound rather than following the
        # raw extrapolation.
        self.declare_parameter('roll_clamp_min_deg', -45.0)
        self.declare_parameter('roll_clamp_max_deg', 45.0)
        # Right rig's measured roll window is -48.1 to 1.0 deg; centered similarly.
        self.declare_parameter('right_roll_clamp_min_deg', -65.0)
        self.declare_parameter('right_roll_clamp_max_deg', 25.0)
        self.declare_parameter(
            'left_arm_joint_trajectory_topic',
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
        )
        self.declare_parameter(
            'right_arm_joint_trajectory_topic',
            '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
        )
        self.declare_parameter('left_gripper_joint', 'gripper_l_joint1')
        self.declare_parameter('right_gripper_joint', 'gripper_r_joint1')
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
        self.declare_parameter(
            'right_arm_joint_names',
            [
                'arm_r_joint1',
                'arm_r_joint2',
                'arm_r_joint3',
                'arm_r_joint4',
                'arm_r_joint5',
                'arm_r_joint6',
                'arm_r_joint7',
                'gripper_r_joint1',
            ],
        )
        self.declare_parameter('gripper_open_position', 0.0)
        self.declare_parameter('gripper_closed_position', 0.6)
        self.declare_parameter('gripper_duration', 1.0)
        self.declare_parameter('gripper_settle_time', 0.2)
        # The MoveL/MoveJ controllers (cyclo_control) stream arm-only trajectory points to
        # this same topic continuously to hold their pose. Since arm_l/r_controller allow
        # partial joint goals, a single one-shot command (gripper or full-arm return) gets
        # pre-empted almost immediately, so the target is re-asserted at this rate instead.
        self.declare_parameter('command_rate_hz', 300.0)
        # While the MoveL controller (cyclo_control) owns the arm, it actively holds the
        # last commanded Cartesian goal in a real-time control loop -- it isn't a "last
        # publisher wins" race, so direct joint-trajectory commands (like the old
        # joint-space "return to initial pose") get fought and overridden forever. Any
        # post-grasp motion has to go through MoveL goals (_move_l) instead, hence the
        # retract-toward-body step below rather than a joint-space homing move.
        self.declare_parameter('return_to_initial', True)
        self.declare_parameter('lift_height', 0.1)
        self.declare_parameter('lift_duration', 2.0)
        self.declare_parameter('retract_distance', 0.2)
        # Right arm was retracting too far out from the body; bring it 3cm closer in.
        self.declare_parameter('right_retract_distance', 0.17)
        # Sentinel [0.0] (not a valid [x, y, z]): derive the retract point from the grasp
        # pose (lift_pose + retract_sign * retract_distance), as usual. A real [x, y, z]:
        # use this fixed absolute base_link point instead, overriding retract_distance/
        # retract_sign for that arm. Right arm's default here was measured via
        # /r_goal_pose mid-retract on the real robot. (Can't default to [] -- ROS2 can't
        # infer an array element type from an empty list and errors on the launch
        # override; a length-1 list still infers as a float array.)
        self.declare_parameter('retract_position_xyz', [0.0])
        self.declare_parameter(
            'right_retract_position_xyz',
            [0.35064399242401123, -0.21548181772232056, 0.9076876640319824],
        )
        self.declare_parameter('retract_duration', 3.0)
        # Measured via `ros2 run tf2_ros tf2_echo base_link end_effector_l_link` with the
        # left arm in its bottle_ready initial pose.
        self.declare_parameter(
            'home_position_xyz',
            [0.13451801240444183, 0.2999741733074188, 0.8241714239120483],
        )
        # Measured from /r_goal_pose with the right arm in its own bottle_ready pose.
        self.declare_parameter(
            'right_home_position_xyz',
            [0.14263390004634857, -0.24826078116893768, 0.812108039855957],
        )
        self.declare_parameter(
            'home_orientation_xyzw',
            [-0.0657237321138382, -0.6881383657455444, -0.06250208616256714, 0.7198885083198547],
        )
        self.declare_parameter(
            'right_home_orientation_xyzw',
            [0.0784875676035881, -0.6821072101593018, 0.08384530246257782, 0.7221768498420715],
        )
        self.declare_parameter('home_duration', 4.0)

        self.detections_topic = self.get_parameter('detections_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.depth_window = int(self.get_parameter('depth_window').value)
        self.target_frame = self.get_parameter('target_frame').value
        self.projection_frame = self.get_parameter('projection_frame').value
        self.default_arm = str(self.get_parameter('arm').value).lower()
        if self.default_arm not in ('left', 'right'):
            raise ValueError('arm must be "left" or "right"')
        self.arm_center_deadband_ratio = float(
            self.get_parameter('arm_center_deadband_ratio').value
        )
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        self.max_grasp_x = float(self.get_parameter('max_grasp_x').value)
        self.execute_motion = self._bool_parameter('execute_motion')
        self.movel_duration = float(self.get_parameter('movel_duration').value)
        self.pregrasp_distance = float(self.get_parameter('pregrasp_distance').value)
        self.pregrasp_duration = float(self.get_parameter('pregrasp_duration').value)
        self.insertion_duration = float(self.get_parameter('insertion_duration').value)
        self.movel_subscriber_timeout = float(
            self.get_parameter('movel_subscriber_timeout').value
        )
        self.settle_time = float(self.get_parameter('settle_time').value)

        self.gripper_open_position = float(self.get_parameter('gripper_open_position').value)
        self.gripper_closed_position = float(
            self.get_parameter('gripper_closed_position').value
        )
        self.gripper_duration = float(self.get_parameter('gripper_duration').value)
        self.gripper_settle_time = float(self.get_parameter('gripper_settle_time').value)
        self.command_rate_hz = float(self.get_parameter('command_rate_hz').value)
        self.return_to_initial = self._bool_parameter('return_to_initial')
        self.lift_height = float(self.get_parameter('lift_height').value)
        self.lift_duration = float(self.get_parameter('lift_duration').value)
        self.retract_duration = float(self.get_parameter('retract_duration').value)
        self.home_duration = float(self.get_parameter('home_duration').value)
        self.bottle_yaw_zero_offset_deg = float(
            self.get_parameter('bottle_yaw_zero_offset_deg').value
        )

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

        self.arm_params = {
            'left': self._build_arm_config('left'),
            'right': self._build_arm_config('right'),
        }

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

        self.get_logger().info('CenterPose direct bottle grasp ready')
        self.get_logger().info(
            f'  left MoveL: {self.arm_params["left"]["movel_topic"]}, '
            f'right MoveL: {self.arm_params["right"]["movel_topic"]}'
        )
        self.get_logger().info(f'  detections: {self.detections_topic}')
        self.get_logger().info(f'  camera info: {self.camera_info_topic}')
        self.get_logger().info(f'  depth (x/y only): {self.depth_topic}')
        self.get_logger().info(
            f'  arm chosen per detection by image side (deadband '
            f'{self.arm_center_deadband_ratio:.2f}); default/tie-break: {self.default_arm}'
        )

    def _build_arm_config(self, side):
        is_right = side == 'right'
        prefix = 'right_' if is_right else ''

        grasp_position_offset = self._list_parameter(f'{prefix}grasp_position_offset')
        if len(grasp_position_offset) != 3:
            raise ValueError(f'{prefix}grasp_position_offset must contain [x, y, z]')

        tool_orientation_offset = np.asarray(
            self._list_parameter(f'{prefix}tool_orientation_offset_xyzw'), dtype=np.float64
        )
        if tool_orientation_offset.shape != (4,):
            raise ValueError(f'{prefix}tool_orientation_offset_xyzw must contain [x, y, z, w]')
        orientation_norm = np.linalg.norm(tool_orientation_offset)
        if orientation_norm < 1e-9:
            raise ValueError(f'{prefix}tool_orientation_offset_xyzw must not be zero')
        tool_orientation_offset = tool_orientation_offset / orientation_norm

        home_position_xyz = self._list_parameter(f'{prefix}home_position_xyz')
        home_orientation_xyzw = self._list_parameter(f'{prefix}home_orientation_xyzw')
        if len(home_position_xyz) != 3:
            raise ValueError(f'{prefix}home_position_xyz must contain [x, y, z]')
        if len(home_orientation_xyzw) != 4:
            raise ValueError(f'{prefix}home_orientation_xyzw must contain [x, y, z, w]')

        retract_position_xyz = self._list_parameter(f'{prefix}retract_position_xyz')
        if len(retract_position_xyz) <= 1:
            retract_position_xyz = None
        elif len(retract_position_xyz) != 3:
            raise ValueError(f'{prefix}retract_position_xyz must be unset or [x, y, z]')

        movel_topic = self.get_parameter(
            'right_movel_topic' if is_right else 'movel_topic'
        ).value
        trajectory_topic = self.get_parameter(
            'right_arm_joint_trajectory_topic' if is_right else 'left_arm_joint_trajectory_topic'
        ).value
        gripper_joint = self.get_parameter(
            'right_gripper_joint' if is_right else 'left_gripper_joint'
        ).value
        arm_joint_names = [
            str(name)
            for name in self.get_parameter(
                'right_arm_joint_names' if is_right else 'left_arm_joint_names'
            ).value
        ]
        if gripper_joint not in arm_joint_names:
            raise ValueError(
                f'{gripper_joint} must be included in '
                f'{"right_arm_joint_names" if is_right else "left_arm_joint_names"}'
            )

        return {
            'side': side,
            'eef_link': str(self.get_parameter(f'{prefix}eef_link').value),
            'fixed_grasp_z': float(self.get_parameter(f'{prefix}fixed_grasp_z').value),
            'grasp_position_offset': grasp_position_offset,
            'tool_orientation_offset': tool_orientation_offset,
            'grasp_fixed_pitch': float(self.get_parameter(f'{prefix}grasp_fixed_pitch').value),
            'grasp_fixed_yaw': float(self.get_parameter(f'{prefix}grasp_fixed_yaw').value),
            'grasp_roll_from_yaw_scale': float(
                self.get_parameter(f'{prefix}grasp_roll_from_yaw_scale').value
            ),
            'grasp_roll_offset': float(self.get_parameter(f'{prefix}grasp_roll_offset').value),
            'roll_clamp_min_deg': float(self.get_parameter(f'{prefix}roll_clamp_min_deg').value),
            'roll_clamp_max_deg': float(self.get_parameter(f'{prefix}roll_clamp_max_deg').value),
            'home_position_xyz': home_position_xyz,
            'home_orientation_xyzw': home_orientation_xyzw,
            # Retract moves further out to the side away from the body midline (see
            # comment in _pick_and_lift); that's +Y for the left arm, -Y for the right.
            'retract_sign': -1.0 if is_right else 1.0,
            'retract_distance': float(self.get_parameter(f'{prefix}retract_distance').value),
            'retract_position_xyz': retract_position_xyz,
            'movel_topic': movel_topic,
            'trajectory_topic': trajectory_topic,
            'gripper_joint': gripper_joint,
            'arm_joint_names': arm_joint_names,
            'movel_pub': self.create_publisher(MoveL, movel_topic, 10),
            'trajectory_pub': self.create_publisher(JointTrajectory, trajectory_topic, 10),
        }

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
            lines.append(
                f'{item["arm"]} @ ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), '
                f'yaw={item["bottle_yaw_deg"]:.1f} deg'
            )
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
            details = ', '.join(
                f'{item["arm"]} x={item["pose"].pose.position.x:.3f}' for item in unreachable
            )
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

    def _select_arm(self, pixel_x, image_width):
        if image_width <= 0:
            return self.default_arm
        image_center_x = image_width * 0.5
        deadband_half_width = image_width * self.arm_center_deadband_ratio * 0.5
        if pixel_x > image_center_x + deadband_half_width:
            return 'right'
        if pixel_x < image_center_x - deadband_half_width:
            return 'left'
        return self.default_arm

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

        # Shared by every detection in this batch (same camera_info/TF at this instant),
        # so resolve them once here instead of once per detection.
        camera_frame = self.projection_frame or camera_info.header.frame_id
        camera_transform = self._lookup_camera_transform(camera_frame, log)
        if camera_transform is None:
            return []
        eef_z_cache = {}

        results = []
        for index, detection in enumerate(detections.detections):
            result = self._process_single_detection(
                detection, camera_info, depth_image, depth_msg,
                camera_transform, eef_z_cache, log, index
            )
            if result is not None:
                results.append(result)
        return results

    def _process_single_detection(
        self, detection, camera_info, depth_image, depth_msg,
        camera_transform, eef_z_cache, log, index
    ):
        center = detection.bbox.center.position
        if not all(math.isfinite(value) for value in (center.x, center.y, center.z)):
            return None

        projected = self._project_to_pixel(camera_info, center, log)
        if projected is None:
            return None
        u, v = projected

        arm = self._select_arm(u, camera_info.width)
        arm_cfg = self.arm_params[arm]

        if arm not in eef_z_cache:
            eef_z_cache[arm] = self._current_eef_z(arm_cfg)
        fixed_z = eef_z_cache[arm]
        if fixed_z is None:
            return None

        real_point_cam = self._real_camera_point(
            camera_info, u, v, depth_image, depth_msg, log
        )
        if real_point_cam is None:
            return None

        transformed = self._transform_centerpose(
            camera_transform, real_point_cam, detection.bbox.center.orientation, arm_cfg
        )
        if transformed is None:
            return None
        point, orientation, object_yaw, roll = transformed

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.target_frame
        pose.pose.position.x = float(point[0] + arm_cfg['grasp_position_offset'][0])
        pose.pose.position.y = float(point[1] + arm_cfg['grasp_position_offset'][1])
        pose.pose.position.z = fixed_z
        pose.pose.orientation = self._quaternion_message(orientation)

        self.object_pose_pub.publish(pose)
        self.grasp_pose_pub.publish(pose)

        # Bottle heading relative to the robot: 0 deg = facing base_link +X (robot front).
        bottle_yaw_deg = self._wrap_degrees(
            math.degrees(object_yaw) - self.bottle_yaw_zero_offset_deg
        )

        if log:
            self.get_logger().info(
                f'[{index}] arm={arm} (pixel_x={u:.0f}/{camera_info.width}), '
                f'camera xyz=({center.x:.3f}, {center.y:.3f}, {center.z:.3f}), '
                f'real camera xyz=({real_point_cam[0]:.3f}, {real_point_cam[1]:.3f}, '
                f'{real_point_cam[2]:.3f}), fixed z={fixed_z:.3f}, target=('
                f'{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, '
                f'{pose.pose.position.z:.3f}), '
                f'bottle_yaw={bottle_yaw_deg:.1f} deg (0=robot front) -> '
                f'roll={math.degrees(roll):.1f} deg'
            )
        return {'pose': pose, 'arm': arm, 'bottle_yaw_deg': bottle_yaw_deg}

    def _project_to_pixel(self, camera_info, position, log):
        # CenterPose's (x,y,z) has the wrong scale but the right bearing: x/z and y/z
        # are the same ratios a correctly-scaled detection would have, since they come
        # from where the object appears in the image, not from the (unreliable) size
        # assumption used to infer z. Re-project that bearing to a pixel and look up
        # real depth there instead of trusting CenterPose's own z magnitude.
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
                    if not self._pick_and_lift(item['pose'], item['arm'], label):
                        if self.cancel_event.is_set():
                            return
                        self.get_logger().error(f'{label} failed; continuing with next bottle')
                        continue
                self.get_logger().info(f'Finished picking {len(queue)} bottle(s)')
            finally:
                self.execution_step = 'idle'

    def _pick_and_lift(self, grasp_pose, arm, label='bottle'):
        arm_cfg = self.arm_params[arm]
        grasp_q = np.array([
            grasp_pose.pose.orientation.x,
            grasp_pose.pose.orientation.y,
            grasp_pose.pose.orientation.z,
            grasp_pose.pose.orientation.w,
        ], dtype=np.float64)
        # Measured from the calibrated grasp orientation: local -Z rotates to
        # ~base_link +X (toward the bottle), so that's the direction of travel when
        # inserting; back off along the opposite of it to get the pregrasp point.
        approach_dir = self._rotate_vector(np.array([0.0, 0.0, -1.0]), grasp_q)

        pregrasp_pose = self._copy_pose(grasp_pose)
        pregrasp_pose.pose.position.x -= approach_dir[0] * self.pregrasp_distance
        pregrasp_pose.pose.position.y -= approach_dir[1] * self.pregrasp_distance
        pregrasp_pose.pose.position.z -= approach_dir[2] * self.pregrasp_distance

        lift_pose = self._copy_pose(grasp_pose)
        lift_pose.pose.position.z += self.lift_height

        home_pose = PoseStamped()
        home_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(
            home_pose, arm_cfg['home_position_xyz'], arm_cfg['home_orientation_xyzw']
        )

        # Move further out to the side away from the body midline rather than
        # pulling straight back through it -- crossing near the body centerline is
        # what was forcing the IK solver to flip the elbow to a very different
        # configuration on the way home. That side is +Y for the left arm, -Y for
        # the right (retract_sign). Orientation switches to the home orientation
        # here already, instead of carrying the grasp orientation all the way back.
        retract_pose = self._copy_pose(lift_pose)
        if arm_cfg['retract_position_xyz'] is not None:
            self._set_pose_from_arrays(retract_pose, arm_cfg['retract_position_xyz'])
        else:
            retract_pose.pose.position.y += arm_cfg['retract_sign'] * arm_cfg['retract_distance']
        retract_pose.pose.orientation = home_pose.pose.orientation

        steps = [
            (
                'open gripper',
                lambda: self._move_gripper(arm_cfg, self.gripper_open_position),
            ),
            (
                'move to pregrasp',
                lambda: self._move_l(arm_cfg, pregrasp_pose, duration=self.pregrasp_duration),
            ),
            (
                'insert to bottle',
                lambda: self._move_l(arm_cfg, grasp_pose, duration=self.insertion_duration),
            ),
            (
                'close gripper',
                lambda: self._move_gripper(arm_cfg, self.gripper_closed_position),
            ),
            (
                'lift bottle',
                lambda: self._move_l(arm_cfg, lift_pose, duration=self.lift_duration),
            ),
        ]
        if self.return_to_initial:
            steps.append((
                'retract toward body',
                lambda: self._move_l(arm_cfg, retract_pose, duration=self.retract_duration),
            ))
            steps.append((
                'return to initial pose',
                lambda: self._move_l(arm_cfg, home_pose, duration=self.home_duration),
            ))

        for name, command in steps:
            if self.cancel_event.is_set():
                return False
            self.execution_step = f'{label} ({arm}): {name}'
            self.get_logger().info(f'Executing {label} {arm} arm {name}')
            if not command():
                self.get_logger().error(f'Stopped at {label} {arm} arm {name}')
                return False
        self.get_logger().info(f'{label} finished with {arm} arm; holding pose')
        return True

    def _move_l(self, arm_cfg, pose, duration=None):
        if not self._wait_for_subscriber(arm_cfg['movel_pub'], arm_cfg['movel_topic']):
            return False

        duration = self.movel_duration if duration is None else duration
        msg = MoveL()
        msg.pose = pose
        msg.time_from_start = self._duration(duration)
        arm_cfg['movel_pub'].publish(msg)
        return self._cancelable_sleep(duration + self.settle_time)

    def _move_gripper(self, arm_cfg, target):
        arm_joint_names = arm_cfg['arm_joint_names']
        deadline = time.monotonic() + 2.0
        positions = None
        while time.monotonic() < deadline and positions is None:
            with self.joint_lock:
                current = [
                    self.current_joint_positions.get(name) for name in arm_joint_names
                ]
            if all(value is not None for value in current):
                positions = [float(value) for value in current]
            else:
                time.sleep(0.05)

        if positions is None:
            missing = []
            with self.joint_lock:
                for name in arm_joint_names:
                    if name not in self.current_joint_positions:
                        missing.append(name)
            self.get_logger().error('Missing joint states: ' + ', '.join(missing))
            return False

        if not self._wait_for_subscriber(arm_cfg['trajectory_pub'], arm_cfg['trajectory_topic']):
            return False

        gripper_joint = arm_cfg['gripper_joint']
        positions[arm_joint_names.index(gripper_joint)] = float(target)

        self.get_logger().info(
            f'Gripper target {gripper_joint}={float(target):.3f} on '
            f'{arm_cfg["trajectory_topic"]}'
        )
        return self._stream_trajectory(
            arm_cfg['trajectory_pub'],
            arm_joint_names,
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

    def _transform_centerpose(self, camera_transform, position_cam, orientation, arm_cfg):
        transform_q, t = camera_transform
        object_q = np.array(
            [orientation.x, orientation.y, orientation.z, orientation.w], dtype=np.float64
        )
        object_norm = np.linalg.norm(object_q)
        if object_norm < 1e-9:
            self.get_logger().warn('CenterPose orientation contains a zero quaternion')
            return None
        object_q /= object_norm

        point = self._rotate_vector(np.asarray(position_cam, dtype=np.float64), transform_q) + t

        # Only the gripper roll follows the object; pitch/yaw stay at the fixed
        # approach orientation (calibrated from matching gripper/object photos).
        object_yaw = self._yaw_from_quaternion(
            self._quaternion_multiply(transform_q, object_q)
        )
        raw_roll_deg = math.degrees(
            arm_cfg['grasp_roll_from_yaw_scale'] * object_yaw + arm_cfg['grasp_roll_offset']
        )
        # The linear fit is only validated across a narrow measured window; far outside
        # it, the raw roll swings into mechanically nonsense angles. Clamp to the
        # known-good [min, max] range instead of trusting far extrapolation -- past
        # either bound, the gripper just holds at that bound rather than rotating
        # further.
        clamped_roll_deg = min(
            max(raw_roll_deg, arm_cfg['roll_clamp_min_deg']), arm_cfg['roll_clamp_max_deg']
        )
        roll = self._wrap_angle(math.radians(clamped_roll_deg))
        base_q = self._quaternion_from_euler(
            roll, arm_cfg['grasp_fixed_pitch'], arm_cfg['grasp_fixed_yaw']
        )
        target_q = self._quaternion_multiply(base_q, arm_cfg['tool_orientation_offset'])
        return point, target_q, object_yaw, roll

    def _current_eef_z(self, arm_cfg):
        if arm_cfg['fixed_grasp_z'] >= 0.0:
            return arm_cfg['fixed_grasp_z']

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, arm_cfg['eef_link'], rclpy.time.Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot get fixed grasp height from {arm_cfg["eef_link"]}: {exc}',
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
    def _quaternion_multiply(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return np.array(
            [
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz,
            ],
            dtype=np.float64,
        )

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
    def _yaw_from_quaternion(q):
        x, y, z, w = q
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _wrap_degrees(degrees):
        return (degrees + 180.0) % 360.0 - 180.0

    @staticmethod
    def _quaternion_from_euler(roll, pitch, yaw):
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        return np.array(
            [
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy,
            ],
            dtype=np.float64,
        )

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
    def _set_pose_from_arrays(pose, xyz, xyzw=None):
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        if xyzw is not None:
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
    node = CenterPoseBottlePick()
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
