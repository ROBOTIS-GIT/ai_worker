#!/usr/bin/env python3

import math

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

from ffw_centerpose.pick_place_base import PickPlaceNodeBase


class CenterposeBox(PickPlaceNodeBase):
    """CenterPose로 검출한 박스를 왼팔로 집어서 정해진 위치에 놓는 노드.

    centerpose_bottle과 달리 그리퍼 roll이 박스의 yaw를 따라가고(pitch/yaw는 고정),
    놓을 때도 release -> 후퇴 -> 밀어넣기까지 여러 단계를 거친다. 공통 로직은
    PickPlaceNodeBase 참고.
    """

    _OBJECT_LABEL_PLURAL = 'box(es)'

    # CenterPose가 가끔 앞뒤 180도 뒤집어 인식하는 걸 보정 (box_yaw_flip_threshold_deg 참고).
    _LOCAL_Y_180_FLIP = np.array([0.0, 1.0, 0.0, 0.0])

    # 파라미터 선언/읽기 후 공통 초기화(_setup_common) 호출까지 한 번에 처리.
    def __init__(self):
        super().__init__('centerpose_box')

        # --- 파라미터 선언 (기본값들) ---
        self.declare_parameter('detections_topic', '/centerpose/detections')
        self.declare_parameter('camera_info_topic', '/camera_info')
        # CenterPose depth는 부정확해서 x/y는 픽셀 투영 + 실측 depth로 구하고,
        # z는 항상 고정값(fixed_grasp_z)을 씀.
        self.declare_parameter('depth_topic', '/zedm/zed_node/depth/depth_registered')
        self.declare_parameter('depth_window', 5)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('projection_frame', '')
        self.declare_parameter('detection_timeout', 10.0)
        # 안전장치: 이 값보다 x가 크면 depth 오독으로 보고 ~/execute가 동작을 거부.
        self.declare_parameter('max_grasp_x', 0.7)
        self.declare_parameter('execute_motion', False)
        self.declare_parameter('movel_topic', '/l_goal_move')
        self.declare_parameter('movel_duration', 10.0)
        # 바로 직진해서 잡으면 그리퍼가 박스에 부딪힐 수 있어서, 진입축(그리퍼별
        # roll에 따라 매번 달라짐)을 따라 이만큼 뒤로 뺀 pregrasp를 거쳐서 들어감.
        self.declare_parameter('pregrasp_distance', 0.11)
        self.declare_parameter('pregrasp_duration', 4.0)
        self.declare_parameter('insertion_duration', 3.0)
        # 감지된 표면 위치는 추정값이라, 실제로 접촉하도록 이만큼 더 밀고 들어간 뒤
        # 그리퍼를 닫음.
        self.declare_parameter('insertion_overshoot_distance', -0.04)
        self.declare_parameter('movel_subscriber_timeout', 2.0)
        self.declare_parameter('settle_time', 0.5)
        self.declare_parameter('eef_link', 'end_effector_l_link')
        # 박스 높이도 안 믿고 z는 항상 이 고정값 사용 (centerpose_bottle과 동일).
        # 음수면 캡처 시점의 현재 엔드이펙터 z를 대신 사용.
        self.declare_parameter('fixed_grasp_z', 0.8241714239120483)
        # [x, y, z] 위치 보정값. grasp_position_y_reference_pixel 위치에서 정확하고,
        # 다른 픽셀 위치에서는 grasp_position_x/y_slope로 추가 보정됨.
        self.declare_parameter('grasp_position_offset', [0.01, -0.02, 0.0])
        # 화면 위치에 따라 y 보정량이 달라져서(카메라 장착 각도 오차로 추정) 픽셀
        # 위치 기반으로 추가 보정하는 기울기. y_offset = grasp_position_offset[1]
        # + grasp_position_y_slope * (pixel_u - grasp_position_y_reference_pixel)
        self.declare_parameter('grasp_position_y_slope', 0.0)
        self.declare_parameter('grasp_position_y_reference_pixel', 288.0)
        # grasp_position_y_slope와 같은 방식의 전후(x) 방향 보정 기울기.
        self.declare_parameter('grasp_position_x_slope', -3.472222222222222e-05)
        # depth는 박스 앞면을 읽는데 실제 중심은 그보다 더 안쪽이라, 이만큼 depth에
        # 더해서 중심 위치를 추정.
        self.declare_parameter('box_depth_center_offset', 0.08)
        self.declare_parameter('tool_orientation_offset_xyzw', [0.0, 0.0, 0.0, 1.0])
        # object_yaw는 단순 오일러 yaw가 아니라, 박스가 로봇 정면을 보는 기준 자세
        # 대비 쿼터니언 axis-angle로 구한 signed 각도 (카메라 장착 회전에 영향
        # 안 받음). 실측 3개 지점(0/+32.5/-58.5도)으로 roll과의 선형관계를 피팅.
        self.declare_parameter(
            'box_yaw_reference_orientation_xyzw',
            [-0.6272754451461677, 0.2145960107465087, -0.6681762105760168, 0.33766050954862303],
        )
        # 박스가 실제로 회전하는 축 (카메라 Z축이 아님 -- 실측으로 구한 값).
        self.declare_parameter(
            'box_yaw_axis_xyz',
            [0.15517196976367656, -0.9264099543820705, 0.3430543050618564],
        )
        # CenterPose가 가끔 박스를 앞뒤 180도 뒤집어 인식함 (크기값엔 안 나타남).
        # 기준 자세와의 각도차가 이 값을 넘으면 뒤집힌 걸로 보고 보정함.
        self.declare_parameter('box_yaw_flip_threshold_deg', 90.0)
        # pitch/yaw는 고정하고 roll만 박스 yaw를 따라감. 기준 샘플의 오일러각을
        # 그대로 사용.
        self.declare_parameter('grasp_fixed_pitch', -1.5016251715681637)
        self.declare_parameter('grasp_fixed_yaw', -0.052366725449585025)
        # roll = scale * box_yaw + offset의 선형 피팅 기울기 (실측 기반).
        self.declare_parameter('grasp_roll_from_yaw_scale', -0.9731714138014503)
        self.declare_parameter('grasp_roll_offset', 0.04845972067574276)
        # 표시용: 박스가 로봇 정면을 볼 때 0도로 보이게 하는 오프셋.
        self.declare_parameter('box_yaw_zero_offset_deg', 0.0)
        # 위 선형 피팅은 실측한 각도 범위 안에서만 유효해서, 그 밖은 이 범위로 clamp.
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
        # 박스는 그리퍼와 크기가 비슷해서 병보다 덜 오므려도 됨.
        self.declare_parameter('gripper_closed_position', 0.57)
        self.declare_parameter('gripper_duration', 1.0)
        self.declare_parameter('gripper_settle_time', 0.2)
        # MoveL 컨트롤러가 같은 토픽에 계속 스트리밍하므로, 그리퍼 명령도 이 주기로
        # 계속 재전송해야 밀려나지 않음.
        self.declare_parameter('command_rate_hz', 300.0)
        self.declare_parameter('lift_height', 0.1)
        self.declare_parameter('lift_duration', 2.0)
        # 들어올린 후: hover 위치로 이동 -> 내려놓기 -> release -> 살짝 오므려서
        # 후퇴 -> 다시 밀어넣기 -> (설정 시) 원위치. 전부 실제 로봇에서 측정한 값.
        self.declare_parameter(
            'place_hover_position_xyz',
            [0.4098847508430481, 0.3597005009651184, 0.9550905227661133],
        )
        self.declare_parameter(
            'place_hover_orientation_xyzw',
            [-0.007772511336952448, -0.692754328250885, -0.007550723850727081,
             0.7210924029350281],
        )
        self.declare_parameter('place_hover_duration', 4.0)
        # 실제로 놓는 위치 (hover에서 천천히 내려감).
        self.declare_parameter(
            'place_position_xyz',
            [0.6282518005371094, 0.3551318049430847, 0.8419253444671631],
        )
        self.declare_parameter(
            'place_orientation_xyzw',
            [-0.007772511336952448, -0.692754328250885, -0.007550723850727081,
             0.7210924029350281],
        )
        self.declare_parameter('place_duration', 6.0)
        # 완전히 열지는 않고, 박스 옆면만 스칠 정도로 살짝만 오므림.
        self.declare_parameter('place_release_gripper_position', 0.9)
        # release 직후 후퇴하는 위치.
        self.declare_parameter(
            'place_retreat_position_xyz',
            [0.5029386687278748, 0.3570454716682434, 0.847183346748352],
        )
        self.declare_parameter(
            'place_retreat_orientation_xyzw',
            [-0.00785414595156908, -0.6926011443138123, -0.007563420571386814,
             0.721238374710083],
        )
        self.declare_parameter('place_retreat_duration', 2.0)
        self.declare_parameter(
            'place_push_position_xyz',
            [0.6187779307365417, 0.3549533486366272, 0.8432992696762085],
        )
        self.declare_parameter(
            'place_push_orientation_xyzw',
            [-0.00785414595156908, -0.6926011443138123, -0.007563420571386814,
             0.721238374710083],
        )
        self.declare_parameter('place_push_duration', 2.0)
        self.declare_parameter('return_to_initial', True)
        # bottle_ready 초기 자세에서 tf2_echo로 측정한 원위치.
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
        self.grasp_position_x_slope = float(
            self.get_parameter('grasp_position_x_slope').value
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

        self.place_release_gripper_position = float(
            self.get_parameter('place_release_gripper_position').value
        )

        self.place_retreat_position_xyz = self._list_parameter('place_retreat_position_xyz')
        if len(self.place_retreat_position_xyz) != 3:
            raise ValueError('place_retreat_position_xyz must contain [x, y, z]')
        self.place_retreat_orientation_xyzw = self._list_parameter(
            'place_retreat_orientation_xyzw'
        )
        if len(self.place_retreat_orientation_xyzw) != 4:
            raise ValueError('place_retreat_orientation_xyzw must contain [x, y, z, w]')
        self.place_retreat_duration = float(self.get_parameter('place_retreat_duration').value)

        self.place_push_position_xyz = self._list_parameter('place_push_position_xyz')
        if len(self.place_push_position_xyz) != 3:
            raise ValueError('place_push_position_xyz must contain [x, y, z]')
        self.place_push_orientation_xyzw = self._list_parameter('place_push_orientation_xyzw')
        if len(self.place_push_orientation_xyzw) != 4:
            raise ValueError('place_push_orientation_xyzw must contain [x, y, z, w]')
        self.place_push_duration = float(self.get_parameter('place_push_duration').value)

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

        self.get_logger().info('CenterposeBox (left arm, roll follows box yaw) ready')
        self.get_logger().info(f'  MoveL: {self.movel_topic}')
        self.get_logger().info(f'  detections: {self.detections_topic}')
        self.get_logger().info(f'  camera info: {self.camera_info_topic}')
        self.get_logger().info(f'  depth (x/y only): {self.depth_topic}')

    # ~/capture 응답에 박스 yaw까지 같이 보여주도록 오버라이드 (centerpose_bottle는 위치만).
    def _format_capture_line(self, item):
        p = item['pose'].pose.position
        return f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), yaw={item["box_yaw_deg"]:.1f} deg'

    # 검출 하나(픽셀 좌표 + depth + 박스 orientation)를 실제 목표 그립 자세로 변환.
    # centerpose_bottle와 달리 방향(roll)이 박스 yaw를 따라가는 게 핵심 차이
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
        x_offset = self.grasp_position_offset[0] + self.grasp_position_x_slope * (
            u - self.grasp_position_y_reference_pixel
        )
        y_offset = self.grasp_position_offset[1] + self.grasp_position_y_slope * (
            u - self.grasp_position_y_reference_pixel
        )
        pose.pose.position.x = float(point[0] + x_offset)
        pose.pose.position.y = float(point[1] + y_offset)
        pose.pose.position.z = fixed_z
        pose.pose.orientation = self._quaternion_message(orientation)

        self.object_pose_pub.publish(pose)
        self.grasp_pose_pub.publish(pose)

        # object_yaw는 box_yaw_reference_orientation_xyzw 기준 signed 각도
        # (_transform_centerpose 참고); box_yaw_zero_offset_deg는 표시용 오프셋.
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
    # centerpose_bottle와 달리 표면-중심 보정(box_depth_center_offset)이 추가로 들어감.
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

        # depth는 박스 앞면까지 거리라 중심까지는 더 멀어서 이만큼 보정. x/y도
        # depth로부터 역투영되므로 z만 따로 고치면 안 되고 depth 자체를 고쳐야 함.
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
    # 놓고(hover->내려놓기->release->retreat(open)->reclose->push->home->open) 오는
    # 전체 동작 시퀀스.
    def _pick_and_place(self, grasp_pose, label='box'):
        grasp_q = np.array([
            grasp_pose.pose.orientation.x,
            grasp_pose.pose.orientation.y,
            grasp_pose.pose.orientation.z,
            grasp_pose.pose.orientation.w,
        ], dtype=np.float64)
        # grasp 방향 기준 진입축(local -Z가 대략 물체 쪽) -- roll이 박스 yaw마다
        # 달라지므로 centerpose_bottle과 달리 매번 다시 계산.
        approach_dir = self._rotate_vector(np.array([0.0, 0.0, -1.0]), grasp_q)

        # 감지된 표면은 추정값이라, 실제로 접촉하도록 조금 더 밀고 들어간 뒤 닫음.
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

        place_retreat_pose = PoseStamped()
        place_retreat_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(
            place_retreat_pose, self.place_retreat_position_xyz,
            self.place_retreat_orientation_xyzw
        )

        place_push_pose = PoseStamped()
        place_push_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(
            place_push_pose, self.place_push_position_xyz, self.place_push_orientation_xyzw
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
            ('move to place retreat', lambda: self._move_l(place_retreat_pose, duration=self.place_retreat_duration)),
            ('reclose gripper for push', lambda: self._move_gripper(self.place_release_gripper_position)),
            ('push box into place', lambda: self._move_l(place_push_pose, duration=self.place_push_duration)),
        ]
        if self.return_to_initial:
            steps.append(
                ('return to initial pose', lambda: self._move_l(home_pose, duration=self.home_duration))
            )
            steps.append(
                ('open gripper', lambda: self._move_gripper(self.gripper_open_position))
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

        # roll만 박스 방향을 따라가고 pitch/yaw는 고정. object_yaw는 기준 자세
        # 대비 signed 각도 (쿼터니언 axis-angle 방식, TF 오차에 영향 안 받음).
        object_yaw, raw_angle = self._signed_box_yaw(object_q)
        if math.degrees(raw_angle) > self.box_yaw_flip_threshold_deg:
            # CenterPose가 가끔 앞뒤를 180도 뒤집어 인식하는 경우 -- 보정 후 재계산.
            object_q = self._quaternion_multiply(object_q, self._LOCAL_Y_180_FLIP)
            object_yaw, _ = self._signed_box_yaw(object_q)

        raw_roll_deg = math.degrees(
            self.grasp_roll_from_yaw_scale * object_yaw + self.grasp_roll_offset
        )
        # 선형 피팅은 실측 범위 안에서만 유효해서, 그 밖은 이 범위로 clamp.
        clamped_roll_deg = min(
            max(raw_roll_deg, self.roll_clamp_min_deg), self.roll_clamp_max_deg
        )
        roll = self._wrap_angle(math.radians(clamped_roll_deg))
        base_q = self._quaternion_from_euler(roll, self.grasp_fixed_pitch, self.grasp_fixed_yaw)
        target_q = self._quaternion_multiply(base_q, self.tool_orientation_offset)
        return point, target_q, object_yaw, roll

    # box_yaw_reference_orientation 대비 object_q가 얼마나(signed) 돌아가 있는지 계산.
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

    # --- 아래는 전부 순수 쿼터니언/각도 계산용 정적 헬퍼 (centerpose_box 전용) ---
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

    # 쿼터니언의 역회전(켤레) 계산.
    @staticmethod
    def _quaternion_conjugate(q):
        x, y, z, w = q
        return np.array([-x, -y, -z, w], dtype=np.float64)

    # 쿼터니언을 "회전축 + 회전각"으로 분해.
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

    # 각도(라디안)를 -pi ~ +pi 범위로 정규화.
    @staticmethod
    def _wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # 각도(도)를 -180 ~ +180 범위로 정규화.
    @staticmethod
    def _wrap_degrees(degrees):
        return (degrees + 180.0) % 360.0 - 180.0

    # roll/pitch/yaw(오일러각)를 쿼터니언으로 변환.
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
    node = CenterposeBox()  # 노드 실행 진입점 (ros2 run/launch에서 호출됨)
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
