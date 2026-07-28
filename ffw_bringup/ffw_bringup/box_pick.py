#!/usr/bin/env python3

import math

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

from ffw_bringup.pick_place_base import PickPlaceNodeBase


class BoxPick(PickPlaceNodeBase):
    """Grab a CenterPose-detected box with the left arm and place it at a fixed drop
    location, rolling the grasp to match the box's yaw.

    Common capture/execute/MoveL/gripper/TF plumbing lives in PickPlaceNodeBase
    (shared with bottle_box.py). Like bottle_box: z always stays at a fixed height
    (fixed_grasp_z) and the arm approaches by backing off to a pregrasp pose along
    the gripper's own approach axis, then moving straight in to grasp (pregrasp ->
    insert -> close -> lift). Unlike bottle_box (whose orientation is fixed for
    every step): only the gripper roll follows the detected box's yaw, same as
    centerpose_bottle_pick -- pitch/yaw stay fixed, and roll =
    grasp_roll_from_yaw_scale * box_yaw + grasp_roll_offset (clamped), fit from a
    few measured (box yaw, gripper roll) pairs. After lifting, the arm moves
    through a fixed sequence -- place hover -> place -> release -> back to place
    hover -> (optionally) home -- all at fixed, pre-measured poses (unlike the
    grasp pose, none of these track the detection). Left arm only.
    """

    _OBJECT_LABEL_PLURAL = 'box(es)'

    # Corrects CenterPose's intermittent 180 deg front/back mislabel -- see
    # box_yaw_flip_threshold_deg.
    _LOCAL_Y_180_FLIP = np.array([0.0, 1.0, 0.0, 0.0])

    def __init__(self):
        super().__init__('box_pick')

        # --- 파라미터 선언 (기본값들) ---
        self.declare_parameter('detections_topic', '/centerpose/detections')
        self.declare_parameter('camera_info_topic', '/camera_info')
        # As in bottle_box: CenterPose's own depth is unreliable, so x/y are
        # recovered by projecting CenterPose's direction to a pixel and reading
        # real metric depth from this image at that pixel. z always stays fixed
        # (see fixed_grasp_z) and never comes from either source.
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
        self.declare_parameter('movel_duration', 10.0)
        # Approaching straight to the final grasp pose lets the gripper body clip the
        # box on the way in. Instead, back off along the gripper's own approach axis
        # (local -Z, rotated by the per-detection grasp orientation -- this varies
        # with roll, unlike bottle_box's single fixed axis) to a pregrasp pose clear
        # of the object, then move straight in along that same axis to grasp.
        self.declare_parameter('pregrasp_distance', 0.13)
        self.declare_parameter('pregrasp_duration', 4.0)
        self.declare_parameter('insertion_duration', 2.0)
        # Detected surface position is an estimate; push this much further along the
        # approach axis than the raw detection before closing the gripper, so the
        # gripper actually makes contact instead of stopping right at the estimate.
        # Pulled back 2cm, 1cm, 2cm, 2cm, then pushed 2cm back in (was 0.01) --
        # was not deep enough.
        self.declare_parameter('insertion_overshoot_distance', -0.04)
        self.declare_parameter('movel_subscriber_timeout', 2.0)
        self.declare_parameter('settle_time', 0.5)
        self.declare_parameter('eef_link', 'end_effector_l_link')
        # Box height is not trusted: z always stays at this fixed height, same value
        # as bottle_box's fixed_grasp_z (measured via tf2_echo base_link
        # end_effector_l_link with the left arm in its bottle_ready initial pose).
        # Negative: fall back to dynamically holding the current EEF base-frame Z
        # at capture time instead.
        self.declare_parameter('fixed_grasp_z', 0.8241714239120483)
        # y shifted so the gripper grabs left of the detected box center. Valid AT
        # grasp_position_y_reference_pixel -- see that param and
        # grasp_position_y_slope for why a flat constant wasn't enough on its own.
        # Re-measured 2026-07-28: 0.0 (no correction) at pixel_u=288 ("left" position).
        self.declare_parameter('grasp_position_offset', [0.0, 0.0, 0.0])
        # A constant y offset only ever matches one screen position -- real-robot
        # testing showed the box needs progressively more +Y (left) correction as its
        # detected pixel moves from the left side of frame toward center -- most
        # likely a small yaw error in the camera->base_link mounting TF that shows up
        # as a bearing-dependent lateral error. Applied as: y_offset =
        # grasp_position_offset[1] + grasp_position_y_slope * (pixel_u -
        # grasp_position_y_reference_pixel). Re-measured 2026-07-28: 0.0 (was -0.02)
        # at pixel_u=288 ("left"), +0.03 (unchanged) at pixel_u=576 ("center").
        self.declare_parameter('grasp_position_y_slope', 0.00010416666666666666)
        self.declare_parameter('grasp_position_y_reference_pixel', 288.0)
        # Depth reads the box's visible front face, not its center -- the box has real
        # thickness, so the center sits this much further along the same camera ray
        # (added directly to the sampled depth in _real_camera_point, since x/y are
        # also derived from depth via the pinhole back-projection and need to scale
        # with it too, not just z).
        self.declare_parameter('box_depth_center_offset', 0.08)
        self.declare_parameter('tool_orientation_offset_xyzw', [0.0, 0.0, 0.0, 1.0])
        # object_yaw is NOT a plain per-quaternion Euler extraction (that was tried and
        # broke down: CenterPose's raw camera-frame yaw isn't linearly related to the
        # needed roll, since the true box rotation axis isn't aligned with the camera's
        # Z axis). Instead, object_yaw is the SIGNED angle of the detected box
        # orientation relative to this fixed reference orientation (a "box facing the
        # robot" detection captured once), measured via quaternion axis-angle, which is
        # invariant to the (unknown) camera->base_link mounting rotation. The sign
        # comes from comparing the rotation axis against box_yaw_axis_xyz below.
        # Recalibrated 2026-07-28 from 3 fresh (box detection, gripper l_goal_pose)
        # samples: box facing the robot (reference, box_yaw=0), ~32.5 deg clockwise,
        # and ~58.5 deg counter-clockwise (this last one arrived as a raw ~124 deg
        # detection -- over box_yaw_flip_threshold_deg below -- so the 180 deg
        # front/back flip correction was applied before use, same as the runtime
        # logic does). grasp_fixed_pitch/yaw were taken directly from the reference
        # sample's own Euler decomposition (see below); for the other 2 samples, roll
        # was solved numerically (holding pitch/yaw fixed at the reference values,
        # searching for the roll that best reproduces the measured l_goal_pose
        # quaternion) rather than reusing their own independent Euler decomposition,
        # since that decomposition is unstable this close to pitch=-90 deg (gimbal
        # lock) and gave implausible multi-tens-of-degrees swings in the "yaw"
        # component alone. grasp_roll_from_yaw_scale/offset are then a least-squares
        # fit of roll vs. signed box_yaw over all 3 points (residuals ~1-3 deg):
        # box_yaw 0/+32.52/-58.51 deg -> roll -0.33/-23.54/+54.82 deg.
        self.declare_parameter(
            'box_yaw_reference_orientation_xyzw',
            [-0.6272754451461677, 0.2145960107465087, -0.6681762105760168, 0.33766050954862303],
        )
        # Calibrated rotation axis (camera frame) that the box actually turns about,
        # derived from the reference vs. the +32.5 deg sample above -- NOT the
        # camera's Z axis (extracting yaw naively about Z is what broke).
        self.declare_parameter(
            'box_yaw_axis_xyz',
            [0.15517196976367656, -0.9264099543820705, 0.3430543050618564],
        )
        # CenterPose intermittently reports a box detection rotated 180 deg about the
        # box's own local Y axis (a front/back face mix-up -- confirmed visually: the
        # detector's red local-X axis comes out pointing the opposite way). This does
        # not change the reported bbox size (a 180 deg turn about Y only flips local
        # X/Z), which is why it isn't visible in the size fields. When the raw signed
        # angle from the reference exceeds this threshold (a real box turn should stay
        # within roll_clamp's validated window, well under this), the 180 deg flip is
        # assumed and corrected before recomputing the angle.
        self.declare_parameter('box_yaw_flip_threshold_deg', 90.0)
        # Grasp orientation keeps pitch/yaw fixed and only rolls the gripper to match
        # the box's yaw. grasp_fixed_pitch/yaw are exactly the reference sample's own
        # (roll, pitch, yaw) Euler decomposition (box_yaw=0 there by construction), so
        # the reference point is reproduced exactly; the other 2 points are a
        # best-fit approximation (see recalibration note above).
        self.declare_parameter('grasp_fixed_pitch', -1.5016251715681637)
        self.declare_parameter('grasp_fixed_yaw', -0.052366725449585025)
        self.declare_parameter('grasp_roll_from_yaw_scale', -0.870624840566106)
        self.declare_parameter('grasp_roll_offset', 0.04845972067574276)
        # Display-only: reported box yaw = object_yaw - this offset, wrapped to
        # (-180, 180]. Tune so a box facing the robot's front (base_link +X) reads 0.
        self.declare_parameter('box_yaw_zero_offset_deg', 0.0)
        # grasp_roll_from_yaw_scale/offset is a linear fit only validated across the
        # practical box-yaw window (+/-40-45 deg); extrapolated far outside that, the
        # raw roll swings into mechanically nonsense angles. The gripper's roll is
        # clamped to this known-good [min, max] range instead -- values outside it
        # saturate at the nearer bound rather than following the raw extrapolation.
        # Widened to +/-60 deg (was -32/+38) -- real-robot testing confirmed the
        # gripper can safely go this far.
        self.declare_parameter('roll_clamp_min_deg', -60.0)
        self.declare_parameter('roll_clamp_max_deg', 60.0)
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
        # Box and gripper are nearly the same size, so much less closing travel is
        # needed than for a bottle.
        self.declare_parameter('gripper_closed_position', 0.57)
        self.declare_parameter('gripper_duration', 1.0)
        self.declare_parameter('gripper_settle_time', 0.2)
        # The MoveL controller (cyclo_control) streams arm-only trajectory points to
        # this same topic continuously to hold its pose, so a single one-shot gripper
        # command gets pre-empted almost immediately -- the target is re-asserted at
        # this rate instead (see PickPlaceNodeBase._stream_trajectory).
        self.declare_parameter('command_rate_hz', 300.0)
        self.declare_parameter('lift_height', 0.1)
        self.declare_parameter('lift_duration', 2.0)
        # After lifting, move to a hover pose above the drop location, then straight
        # down/in to the actual place pose, release, then back to the hover pose and
        # (if return_to_initial) head home. Both measured via
        # `ros2 topic echo --once /l_goal_pose` with the arm holding a box at each
        # point.
        self.declare_parameter(
            'place_hover_position_xyz',
            [0.32607388496398926, 0.13636542856693268, 0.8921283483505249],
        )
        self.declare_parameter(
            'place_hover_orientation_xyzw',
            [-0.019953999668359756, -0.6819889545440674, -0.020979225635528564,
             0.7307890057563782],
        )
        self.declare_parameter('place_hover_duration', 4.0)
        self.declare_parameter(
            'place_position_xyz',
            [0.6525217294692993, 0.11738879978656769, 0.8008310198783875],
        )
        self.declare_parameter(
            'place_orientation_xyzw',
            [-0.019953999668359756, -0.6819889545440674, -0.020979225635528564,
             0.7307890057563782],
        )
        self.declare_parameter('place_duration', 6.0)
        self.declare_parameter('return_to_initial', True)
        # Measured via tf2_echo base_link end_effector_l_link with the left arm in its
        # bottle_ready initial pose (same pose bottle_box's fixed values come from).
        self.declare_parameter(
            'home_position_xyz',
            [0.13451801240444183, 0.2999741733074188, 0.9742214239120483],
        )
        self.declare_parameter(
            'home_orientation_xyzw',
            [-0.0657237321138382, -0.6881383657455444, -0.06250208616256714, 0.7198885083198547],
        )
        self.declare_parameter('home_duration', 6.0)

        # --- 위에서 선언한 파라미터들을 실제 self.xxx 값으로 읽어들임 ---
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
        self.insertion_overshoot_distance = float(
            self.get_parameter('insertion_overshoot_distance').value
        )
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
        self.grasp_position_y_slope = float(
            self.get_parameter('grasp_position_y_slope').value
        )
        self.grasp_position_y_reference_pixel = float(
            self.get_parameter('grasp_position_y_reference_pixel').value
        )
        self.box_depth_center_offset = float(
            self.get_parameter('box_depth_center_offset').value
        )

        tool_orientation_offset = np.asarray(
            self._list_parameter('tool_orientation_offset_xyzw'), dtype=np.float64
        )
        if tool_orientation_offset.shape != (4,):
            raise ValueError('tool_orientation_offset_xyzw must contain [x, y, z, w]')
        tool_orientation_norm = np.linalg.norm(tool_orientation_offset)
        if tool_orientation_norm < 1e-9:
            raise ValueError('tool_orientation_offset_xyzw must not be zero')
        self.tool_orientation_offset = tool_orientation_offset / tool_orientation_norm

        box_yaw_reference_orientation = np.asarray(
            self._list_parameter('box_yaw_reference_orientation_xyzw'), dtype=np.float64
        )
        if box_yaw_reference_orientation.shape != (4,):
            raise ValueError('box_yaw_reference_orientation_xyzw must contain [x, y, z, w]')
        reference_norm = np.linalg.norm(box_yaw_reference_orientation)
        if reference_norm < 1e-9:
            raise ValueError('box_yaw_reference_orientation_xyzw must not be zero')
        self.box_yaw_reference_orientation = box_yaw_reference_orientation / reference_norm

        box_yaw_axis = np.asarray(self._list_parameter('box_yaw_axis_xyz'), dtype=np.float64)
        if box_yaw_axis.shape != (3,):
            raise ValueError('box_yaw_axis_xyz must contain [x, y, z]')
        box_yaw_axis_norm = np.linalg.norm(box_yaw_axis)
        if box_yaw_axis_norm < 1e-9:
            raise ValueError('box_yaw_axis_xyz must not be zero')
        self.box_yaw_axis = box_yaw_axis / box_yaw_axis_norm

        self.box_yaw_flip_threshold_deg = float(
            self.get_parameter('box_yaw_flip_threshold_deg').value
        )

        self.grasp_fixed_pitch = float(self.get_parameter('grasp_fixed_pitch').value)
        self.grasp_fixed_yaw = float(self.get_parameter('grasp_fixed_yaw').value)
        self.grasp_roll_from_yaw_scale = float(
            self.get_parameter('grasp_roll_from_yaw_scale').value
        )
        self.grasp_roll_offset = float(self.get_parameter('grasp_roll_offset').value)
        self.box_yaw_zero_offset_deg = float(
            self.get_parameter('box_yaw_zero_offset_deg').value
        )
        self.roll_clamp_min_deg = float(self.get_parameter('roll_clamp_min_deg').value)
        self.roll_clamp_max_deg = float(self.get_parameter('roll_clamp_max_deg').value)

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

        self.place_hover_position_xyz = self._list_parameter('place_hover_position_xyz')
        if len(self.place_hover_position_xyz) != 3:
            raise ValueError('place_hover_position_xyz must contain [x, y, z]')
        self.place_hover_orientation_xyzw = self._list_parameter(
            'place_hover_orientation_xyzw'
        )
        if len(self.place_hover_orientation_xyzw) != 4:
            raise ValueError('place_hover_orientation_xyzw must contain [x, y, z, w]')
        self.place_hover_duration = float(self.get_parameter('place_hover_duration').value)

        self.place_position_xyz = self._list_parameter('place_position_xyz')
        if len(self.place_position_xyz) != 3:
            raise ValueError('place_position_xyz must contain [x, y, z]')
        self.place_orientation_xyzw = self._list_parameter('place_orientation_xyzw')
        if len(self.place_orientation_xyzw) != 4:
            raise ValueError('place_orientation_xyzw must contain [x, y, z, w]')
        self.place_duration = float(self.get_parameter('place_duration').value)

        self.return_to_initial = self._bool_parameter('return_to_initial')

        home_position_xyz = self._list_parameter('home_position_xyz')
        if len(home_position_xyz) != 3:
            raise ValueError('home_position_xyz must contain [x, y, z]')
        self.home_position_xyz = home_position_xyz
        home_orientation_xyzw = self._list_parameter('home_orientation_xyzw')
        if len(home_orientation_xyzw) != 4:
            raise ValueError('home_orientation_xyzw must contain [x, y, z, w]')
        self.home_orientation_xyzw = home_orientation_xyzw
        self.home_duration = float(self.get_parameter('home_duration').value)

        # TF/락/퍼블리셔/구독/서비스 등 공통 초기화는 PickPlaceNodeBase에 위임
        self._setup_common()

        self.get_logger().info('BoxPick (left arm, roll follows box yaw) ready')
        self.get_logger().info(f'  MoveL: {self.movel_topic}')
        self.get_logger().info(f'  detections: {self.detections_topic}')
        self.get_logger().info(f'  camera info: {self.camera_info_topic}')
        self.get_logger().info(f'  depth (x/y only): {self.depth_topic}')

    # ~/capture 응답에 박스 yaw까지 같이 보여주도록 오버라이드 (bottle_box는 위치만).
    def _format_capture_line(self, item):
        p = item['pose'].pose.position
        return f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), yaw={item["box_yaw_deg"]:.1f} deg'

    # 검출 하나(픽셀 좌표 + depth + 박스 orientation)를 실제 목표 그립 자세로 변환.
    # bottle_box와 달리 방향(roll)이 박스 yaw를 따라가는 게 핵심 차이
    # (_transform_centerpose에서 계산).
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

        transformed = self._transform_centerpose(
            camera_transform, real_point_cam, detection.bbox.center.orientation
        )
        if transformed is None:
            return None
        point, orientation, object_yaw, roll = transformed

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.target_frame
        y_offset = self.grasp_position_offset[1] + self.grasp_position_y_slope * (
            u - self.grasp_position_y_reference_pixel
        )
        pose.pose.position.x = float(point[0] + self.grasp_position_offset[0])
        pose.pose.position.y = float(point[1] + y_offset)
        pose.pose.position.z = fixed_z
        pose.pose.orientation = self._quaternion_message(orientation)

        self.object_pose_pub.publish(pose)
        self.grasp_pose_pub.publish(pose)

        # object_yaw is the signed angle from box_yaw_reference_orientation_xyzw (see
        # _transform_centerpose); box_yaw_zero_offset_deg is display-only, tuned so a
        # chosen reference box heading reads ~0 here.
        box_yaw_deg = self._wrap_degrees(
            math.degrees(object_yaw) - self.box_yaw_zero_offset_deg
        )

        if log:
            self.get_logger().info(
                f'[{index}] pixel=({u:.0f}, {v:.0f})/{camera_info.width}, '
                f'real camera xyz=({real_point_cam[0]:.3f}, {real_point_cam[1]:.3f}, '
                f'{real_point_cam[2]:.3f}), fixed z={fixed_z:.3f}, target=('
                f'{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, '
                f'{pose.pose.position.z:.3f}), '
                f'box_yaw={box_yaw_deg:.1f} deg (0=robot front) -> '
                f'roll={math.degrees(roll):.1f} deg'
            )
        return {'pose': pose, 'pixel_u': u, 'box_yaw_deg': box_yaw_deg}

    # 픽셀(u,v) + 그 지점의 실제 depth로 카메라 좌표계에서의 3D 점(x,y,z)을 역투영.
    # bottle_box와 달리 표면-중심 보정(box_depth_center_offset)이 추가로 들어감.
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

        # The depth sensor only reaches the box's visible front face, not its center
        # -- the box has real thickness, so the center sits this much further along
        # the same camera ray. x/y must be re-derived from the corrected depth (not
        # just z bumped afterwards), since both scale with depth in this back
        # -projection -- using the wrong depth here is exactly what was throwing x/y
        # off too.
        depth += self.box_depth_center_offset

        return np.array(
            [(u - cx) * depth / fx, (v - cy) * depth / fy, depth], dtype=np.float64
        )

    # ~/execute로 캡처해둔 박스들을 한 번에 하나씩 순서대로 집어서 놓는다.
    def _execute_queue(self, queue):
        with self.execution_lock:
            try:
                for index, item in enumerate(queue):
                    if self.cancel_event.is_set():
                        return
                    label = f'box {index + 1}/{len(queue)}'
                    if not self._pick_and_place(item['pose'], label):
                        if self.cancel_event.is_set():
                            return
                        self.get_logger().error(f'{label} failed; continuing with next box')
                        continue
                self.get_logger().info(f'Finished picking {len(queue)} box(es)')
            finally:
                self.execution_step = 'idle'

    # 박스 하나를 실제로 집어서(pregrasp->insert->close->lift) 지정 위치에
    # 놓고(hover->내려놓기->release->다시 hover) 오는 전체 동작 시퀀스.
    def _pick_and_place(self, grasp_pose, label='box'):
        grasp_q = np.array([
            grasp_pose.pose.orientation.x,
            grasp_pose.pose.orientation.y,
            grasp_pose.pose.orientation.z,
            grasp_pose.pose.orientation.w,
        ], dtype=np.float64)
        # Measured from the calibrated grasp orientation family: local -Z rotates to
        # ~base_link +X (toward the object), so that's the direction of travel when
        # inserting; back off along the opposite of it to get the pregrasp point.
        # Recomputed per grasp (unlike bottle_box's single fixed axis) since roll --
        # and therefore this axis -- varies with the box's yaw.
        approach_dir = self._rotate_vector(np.array([0.0, 0.0, -1.0]), grasp_q)

        # Push 1cm further in than the detected grasp point before closing -- the
        # detected surface is an estimate, so a bit of extra insertion depth ensures
        # real contact instead of just barely reaching it.
        insert_pose = self._copy_pose(grasp_pose)
        insert_pose.pose.position.x += approach_dir[0] * self.insertion_overshoot_distance
        insert_pose.pose.position.y += approach_dir[1] * self.insertion_overshoot_distance
        insert_pose.pose.position.z += approach_dir[2] * self.insertion_overshoot_distance

        pregrasp_pose = self._copy_pose(insert_pose)
        pregrasp_pose.pose.position.x -= approach_dir[0] * self.pregrasp_distance
        pregrasp_pose.pose.position.y -= approach_dir[1] * self.pregrasp_distance
        pregrasp_pose.pose.position.z -= approach_dir[2] * self.pregrasp_distance

        lift_pose = self._copy_pose(insert_pose)
        lift_pose.pose.position.z += self.lift_height

        place_hover_pose = PoseStamped()
        place_hover_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(
            place_hover_pose, self.place_hover_position_xyz, self.place_hover_orientation_xyzw
        )

        place_pose = PoseStamped()
        place_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(
            place_pose, self.place_position_xyz, self.place_orientation_xyzw
        )

        home_pose = PoseStamped()
        home_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(home_pose, self.home_position_xyz, self.home_orientation_xyzw)

        steps = [
            ('open gripper', lambda: self._move_gripper(self.gripper_open_position)),
            ('move to pregrasp', lambda: self._move_l(pregrasp_pose, duration=self.pregrasp_duration)),
            ('insert to box', lambda: self._move_l(insert_pose, duration=self.insertion_duration)),
            ('close gripper', lambda: self._move_gripper(self.gripper_closed_position)),
            ('lift box', lambda: self._move_l(lift_pose, duration=self.lift_duration)),
            ('move to place hover', lambda: self._move_l(place_hover_pose, duration=self.place_hover_duration)),
            ('move to place', lambda: self._move_l(place_pose, duration=self.place_duration)),
            ('release box', lambda: self._move_gripper(self.gripper_open_position)),
            ('back to place hover', lambda: self._move_l(place_hover_pose, duration=self.place_hover_duration)),
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

    # 카메라 좌표계의 점+박스 orientation을 base_link 기준 (위치, 목표 그리퍼
    # orientation, 박스 yaw, roll)로 변환. pitch/yaw는 고정, roll만 박스 yaw에
    # 맞춰 선형식(grasp_roll_from_yaw_scale/offset)으로 계산.
    def _transform_centerpose(self, camera_transform, position_cam, orientation):
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
        # approach orientation. Unlike centerpose_bottle_pick.py's plain per-quaternion
        # Euler yaw extraction (tried here first and found to not correlate with the
        # needed roll -- the box's true rotation axis isn't the camera's Z axis),
        # object_yaw is the signed angle of object_q relative to the fixed
        # box_yaw_reference_orientation_xyzw, measured via quaternion axis-angle (frame
        # -invariant, so it doesn't need a live camera->base_link TF either).
        object_yaw, raw_angle = self._signed_box_yaw(object_q)
        if math.degrees(raw_angle) > self.box_yaw_flip_threshold_deg:
            # CenterPose intermittently reports this box front/back-flipped 180 deg
            # about its own local Y axis (confirmed visually against the detector's
            # rendered axes) -- correct and recompute.
            object_q = self._quaternion_multiply(object_q, self._LOCAL_Y_180_FLIP)
            object_yaw, _ = self._signed_box_yaw(object_q)

        raw_roll_deg = math.degrees(
            self.grasp_roll_from_yaw_scale * object_yaw + self.grasp_roll_offset
        )
        # The linear fit is only validated across a narrow measured window; far
        # outside it, the raw roll swings into mechanically nonsense angles. Clamp
        # to the known-good [min, max] range instead of trusting far extrapolation --
        # past either bound, the gripper just holds at that bound.
        clamped_roll_deg = min(
            max(raw_roll_deg, self.roll_clamp_min_deg), self.roll_clamp_max_deg
        )
        roll = self._wrap_angle(math.radians(clamped_roll_deg))
        base_q = self._quaternion_from_euler(roll, self.grasp_fixed_pitch, self.grasp_fixed_yaw)
        target_q = self._quaternion_multiply(base_q, self.tool_orientation_offset)
        return point, target_q, object_yaw, roll

    def _signed_box_yaw(self, object_q):
        """Signed angle (rad) of object_q relative to box_yaw_reference_orientation.

        Returns (signed_angle, unsigned_angle) -- the unsigned angle is always in
        [0, pi] and is what box_yaw_flip_threshold_deg is compared against, since a
        180 deg front/back mislabel always shows up as an unusually large angle here
        regardless of which way the box actually turned.
        """
        relative_q = self._quaternion_multiply(
            self._quaternion_conjugate(self.box_yaw_reference_orientation), object_q
        )
        axis, angle = self._quaternion_axis_angle(relative_q)
        sign = 1.0 if np.dot(axis, self.box_yaw_axis) >= 0.0 else -1.0
        return sign * angle, angle

    # --- 아래는 전부 순수 쿼터니언/각도 계산용 정적 헬퍼 (box_pick 전용) ---
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
    def _quaternion_conjugate(q):
        x, y, z, w = q
        return np.array([-x, -y, -z, w], dtype=np.float64)

    @staticmethod
    def _quaternion_axis_angle(q):
        """Angle in [0, pi] and its unit rotation axis, forcing w >= 0 for uniqueness."""
        q = q / np.linalg.norm(q)
        x, y, z, w = q
        if w < 0.0:
            x, y, z, w = -x, -y, -z, -w
        angle = 2.0 * math.acos(max(-1.0, min(1.0, w)))
        s = math.sqrt(max(0.0, 1.0 - w * w))
        axis = np.zeros(3) if s < 1e-8 else np.array([x, y, z]) / s
        return axis, angle

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


def main(args=None):
    rclpy.init(args=args)
    node = BoxPick()
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
