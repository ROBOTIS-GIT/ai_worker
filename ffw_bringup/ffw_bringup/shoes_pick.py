#!/usr/bin/env python3

import math
import threading

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

from ffw_bringup.pick_place_base import PickPlaceNodeBase


class ShoesPick(PickPlaceNodeBase):
    """[초안/DRAFT] Grab a CenterPose-detected shoe with the left arm.

    box_pick.py와 핵심적으로 다른 점: 그리퍼가 신발을 위에서 아래로 내려다보며
    잡는 자세라서, 신발이 회전하면 그리퍼는 roll이 아니라 **yaw**가 따라가야
    한다 (box_pick은 옆에서 잡으므로 roll이 따라감). 실측 3개 포인트(정면 기준,
    시계 방향, 반시계 방향)로 확인: roll을 신발 yaw에 맞춰 봤더니 오차가 46도
    가까이 났지만, yaw를 맞추니 오차가 1도 미만이었다. grasp_fixed_roll/pitch는
    고정, grasp_yaw_from_shoe_yaw_scale/offset으로 yaw만 계산한다.

    높이(z)는 두 단계로 나뉜다: 잡기 전(hover) 높이는 항상 fixed_grasp_z로
    고정(시작 자세와 같은 높이), 실제로 잡을 때는 그보다 insertion_overshoot_distance
    만큼 더 내려가는데, 이때 min_grasp_z보다 절대 더 내려가지 않도록 코드에서
    직접 clamp한다 (실측된 "여기보다 아래로 내려가면 안 됨" 안전선).

    잡은 후에는 bottle_box.py와 같은 2슬롯 방식으로 놓는다 (캡처 순서대로
    왼쪽부터 슬롯1, 슬롯2 -- 슬롯보다 신발이 많으면 마지막 슬롯 재사용):
    슬롯 hover -> place_lower_distance만큼 내려서 놓기 -> release -> 다시 hover.
    신발과 신발 사이에는 시작 자세로 돌아가지 않고, hover에서 바로 다음 신발의
    pregrasp로 이어간다 -- 시작 자세 복귀(왕복이라 start_duration만큼 느리게)는
    큐에 담긴 신발을 전부 처리한 뒤 한 번만(_execute_queue).

    아직 캘리브레이션 전인 부분 (TODO로 표시):
    - grasp_position_offset: 신발 구멍(잡는 지점) 위치가 검출 center로부터
      x/y로 얼마나 떨어져 있는지 -- 아직 [0, 0, 0]. 실기로 ~/capture 결과와
      비교해서 채울 예정.
    - place_lower_distance: 슬롯 hover에서 실제로 놓는 높이까지 내려가는 거리 --
      아직 따로 측정 안 해서 잡을 때 하강 거리를 그대로 재사용 중.

    시작 자세로 가는 큰 회전은 이제 pick 시퀀스 자체에 들어있지 않다 -- 노드가
    시작될 때(execute_motion=true) 딱 한 번, startup_pose -> start_pose 둘 다
    아주 느린 속도(startup_duration, start_duration)로 왼팔을 이동시키고
    (_startup_sequence),
    이후 ~/execute는 이미 start_pose에 있다고 가정하고 바로 그리퍼를 여는 것부터
    시작한다. 다만 놓은 뒤 복귀는 여전히 pick 시퀀스 끝에서 start_duration으로
    (왕복 이동도 똑같이 큰 회전이라). 왼팔만 사용.
    """

    _OBJECT_LABEL_PLURAL = 'shoe(s)'

    def __init__(self):
        super().__init__('shoes_pick')

        # --- 파라미터 선언 (기본값들) ---
        self.declare_parameter('detections_topic', '/centerpose/detections')
        self.declare_parameter('camera_info_topic', '/camera_info')
        # CenterPose의 depth는 부정확 -- x/y는 CenterPose 방향(픽셀)만 빌리고
        # 실제 metric depth는 이 토픽에서 읽는다. z는 fixed_grasp_z로 항상 고정.
        self.declare_parameter('depth_topic', '/zedm/zed_node/depth/depth_registered')
        self.declare_parameter('depth_window', 5)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('projection_frame', '')
        self.declare_parameter('detection_timeout', 10.0)
        # 안전장치: 이보다 먼 x는 실제 작업 범위 밖 -- depth 오독일 가능성이 높아
        # ~/execute가 아예 움직이지 않고 거부한다. box_pick/bottle_box(0.7)보다
        # 넓게 잡음 -- 신발 작업 범위가 더 멀리까지 나감 (실측 x=0.87 확인됨).
        self.declare_parameter('max_grasp_x', 0.9)
        self.declare_parameter('execute_motion', False)
        self.declare_parameter('movel_topic', '/l_goal_move')
        self.declare_parameter('movel_duration', 10.0)

        # --- 노드 시작 시 한 번만: 왼팔을 startup_pose -> start_pose 순서로 아주
        # 천천히 이동 (execute_motion=true일 때만, __init__ 끝에서 백그라운드
        # 스레드로 실행 -- see _startup_sequence). ~/execute의 pick 시퀀스에는
        # 더 이상 포함되지 않는다.
        self.declare_parameter(
            'startup_position_xyz',
            [0.17088773846626282, 0.48100942373275757, 0.9724668264389038],
        )
        self.declare_parameter(
            'startup_orientation_xyzw',
            [-0.06572848558425903, -0.6882247924804688, -0.06251275539398193,
             0.7198045253753662],
        )
        # start_duration과 동일하게 아주 천천히.
        self.declare_parameter('startup_duration', 10.0)

        # --- 신발잡기 시작 자세 (실측값, 팔이 크게 돌아가므로 아주 느리게) ---
        # `ros2 topic echo --once /l_goal_pose`로 측정한 값. 노드 시작 시
        # startup_pose 다음으로 이동하고, 놓은 뒤 복귀 지점으로도 쓰인다.
        self.declare_parameter(
            'start_position_xyz',
            [0.23394590616226196, 0.3340926766395569, 0.9377147555351257],
        )
        self.declare_parameter(
            'start_orientation_xyzw',
            [-0.002594258636236191, 0.008345617912709713, -0.00505678029730916,
             0.9999490976333618],
        )
        # 팔이 거의 완전히 돌아가는 동작이라 기본값을 크게 잡음 -- 실기 테스트하며 조정.
        self.declare_parameter('start_duration', 10.0)
        # 큐에 담긴 신발을 전부 처리한 뒤 마지막으로 시작 자세로 돌아올 때만 쓰는
        # 속도 -- start_duration(10초)보다 대략 1.5배 빠르게, 깔끔하게 6초로.
        self.declare_parameter('return_duration', 6.0)

        # --- 접근/삽입 ---
        self.declare_parameter('pregrasp_distance', 0.1)
        self.declare_parameter('pregrasp_duration', 4.0)
        self.declare_parameter('insertion_duration', 2.0)
        # 잡기 전(hover, fixed_grasp_z)에서 실제로 잡는 높이(min_grasp_z)까지
        # 접근축(대략 수직 아래) 방향으로 내려가는 거리 -- 둘 다 실측값이라 그
        # 차이로 계산됨: 0.9377147555351257(시작 자세 높이) - 0.7938926219940186
        # (실측된 "이 이하로 내려가면 안 되는" 한계) = 0.1438221335411271.
        self.declare_parameter('insertion_overshoot_distance', 0.1438221335411271)
        self.declare_parameter('movel_subscriber_timeout', 2.0)
        self.declare_parameter('settle_time', 0.5)
        self.declare_parameter('eef_link', 'end_effector_l_link')
        # 잡기 전(hover) 높이는 항상 이 값으로 고정 -- 시작 자세(start_position_xyz)와
        # 같은 높이를 그대로 재사용 (실측: `ros2 topic echo --once /l_goal_pose`).
        self.declare_parameter('fixed_grasp_z', 0.9377147555351257)
        # 실제로 신발을 잡을 때(내려간 자세) 절대 이 아래로 내려가면 안 되는 높이
        # -- 실측값. _pick_and_place에서 insert_pose.z를 이 값 아래로 못 내려가게
        # 직접 clamp한다 (근사 계산에 기대지 않고 안전하게).
        self.declare_parameter('min_grasp_z', 0.7938926219940186)

        # --- 신발 구멍(잡는 지점) x/y 오프셋 ---
        # x: 로봇 몸쪽(-X)으로 3cm 당겨서 잡음 (1cm -> 추가로 2cm 더).
        self.declare_parameter('grasp_position_offset', [-0.03, 0.0, 0.0])
        # box_pick.py와 동일한 위치 기반 y 보정: 화면 중앙에 가까워질수록 왼쪽으로
        # 더 잡아야 함. grasp_position_offset[1]은 grasp_position_y_reference_pixel
        # (u=288, 화면 왼쪽 부근)에서 정확히 맞는 값이고, 여기서 화면 중앙(u=576)
        # 방향으로 갈수록 이 기울기만큼 더해짐. 실측: 중앙에서 2cm 더 왼쪽 필요 --
        # 처음에 부호를 반대로 넣어서(+) 오른쪽으로 잡혔던 걸 확인 후 뒤집음(-).
        self.declare_parameter('grasp_position_y_slope', -6.944444444444444e-05)
        self.declare_parameter('grasp_position_y_reference_pixel', 288.0)
        # 캡처 순서상 두 번째 신발(항상 오른쪽에 있던 신발)만 잡을 때 추가로
        # 오른쪽(-Y)으로 더 당겨 잡음 -- 화면 위치 기반 보정과는 별개로, 캡처
        # 순번(큐 index==1)에만 적용.
        self.declare_parameter('second_shoe_grasp_y_offset', -0.02)
        self.declare_parameter('shoe_depth_center_offset', 0.0)
        self.declare_parameter('tool_orientation_offset_xyzw', [0.0, 0.0, 0.0, 1.0])

        # --- 신발 각도(yaw) -> 그리퍼 yaw 캘리브레이션 ---
        # box_pick.py와 달리 그리퍼가 신발을 위에서 내려다보며 잡으므로, 신발이
        # 회전하면 그리퍼는 roll이 아니라 yaw가 따라가야 한다 (실측 확인: roll로
        # 맞추면 오차 46도, yaw로 맞추면 오차 1도 미만). object_yaw = 이 reference
        # orientation 대비 신발 orientation의 signed 각도(쿼터니언 axis-angle),
        # gripper_yaw = scale*object_yaw + offset. roll/pitch는 고정.
        # 실측 3점(기준/시계 35.75도/반시계 -69.38도) 최소자승 피팅, 잔차 2~6도,
        # 별도 4번째 샘플로 검증 시 오차 약 4.4도.
        self.declare_parameter(
            'shoe_yaw_reference_orientation_xyzw',
            [0.04132861537025957, -0.3549227920668959, 0.9339735266150585,
             -0.003899846823498184],
        )
        self.declare_parameter(
            'shoe_yaw_axis_xyz', [0.13999845344725156, -0.9872268603133045, 0.07604971603046785]
        )
        self.declare_parameter('shoe_yaw_flip_threshold_deg', 90.0)
        self.declare_parameter('grasp_fixed_roll', -0.005273412980347409)
        self.declare_parameter('grasp_fixed_pitch', 0.01666491788848952)
        self.declare_parameter('grasp_yaw_from_shoe_yaw_scale', -1.016908340135941)
        self.declare_parameter('grasp_yaw_offset', -0.12016636998914268)
        self.declare_parameter('yaw_clamp_min_deg', -90.0)
        self.declare_parameter('yaw_clamp_max_deg', 90.0)

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
        # bottle_box.py는 0.7이지만 신발은 더 세게 쥐어야 해서 늘림 (0.9 -> 1.1).
        self.declare_parameter('gripper_closed_position', 1.1)
        # 신발을 놓을 때(release)만 쓰는 값 -- gripper_open_position(완전히 열기)
        # 만큼 벌리면 옆 슬롯에 이미 놓인 신발을 건드려서, 놓을 때는 이만큼만 벌림.
        self.declare_parameter('gripper_release_position', 0.7)
        self.declare_parameter('gripper_duration', 1.0)
        self.declare_parameter('gripper_settle_time', 0.2)
        self.declare_parameter('command_rate_hz', 300.0)
        self.declare_parameter('lift_height', 0.1)
        self.declare_parameter('lift_duration', 2.0)

        # --- 놓는 위치 (bottle_box.py와 동일 패턴: 슬롯 2개, 캡처 순서대로 왼쪽부터
        # 채움 -- 왼쪽 검출 -> 슬롯1, 다음 -> 슬롯2). 둘 다 hover(공중) 자세이고,
        # 실제로 놓을 땐 place_lower_distance만큼 더 내려간 뒤 놓는다.
        # `ros2 topic echo --once /l_goal_pose`로 hover 상태에서 실측.
        self.declare_parameter(
            'place_slot_1_position_xyz',
            [0.47151222825050354, 0.42120254039764404, 0.9456136226654053],
        )
        self.declare_parameter(
            'place_slot_1_orientation_xyzw',
            [-0.0023092320188879967, 0.008429071865975857, 0.028931625187397003,
             0.9995430707931519],
        )
        # y를 슬롯1에서 더 멀어지는(오른쪽) 방향으로 4cm 이동 (0.279813 -> 0.239813)
        # -- 두 신발이 서로 안 닿게 간격을 다시 벌림.
        self.declare_parameter(
            'place_slot_2_position_xyz',
            [0.45977485179901123, 0.23981253385543824, 0.9466336965560913],
        )
        self.declare_parameter(
            'place_slot_2_orientation_xyzw',
            [-0.0023092320188879967, 0.008429071865975857, 0.028931625187397003,
             0.9995430707931519],
        )
        self.declare_parameter('place_hover_duration', 4.0)
        # TODO: hover에서 실제로 놓는 높이까지 얼마나 내려가는지 아직 정확히 측정
        # 안 됨 -- 잡을 때의 하강 거리(insertion_overshoot_distance)에서 1cm 덜
        # 내려가게(더 높은 곳에서 놓게) 뺌.
        self.declare_parameter('place_lower_distance', 0.1338221335411271)
        self.declare_parameter('place_lower_duration', 2.0)
        self.declare_parameter('place_duration', 1.0)

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

        self.startup_position_xyz = self._list_parameter('startup_position_xyz')
        if len(self.startup_position_xyz) != 3:
            raise ValueError('startup_position_xyz must contain [x, y, z]')
        self.startup_orientation_xyzw = self._list_parameter('startup_orientation_xyzw')
        if len(self.startup_orientation_xyzw) != 4:
            raise ValueError('startup_orientation_xyzw must contain [x, y, z, w]')
        self.startup_duration = float(self.get_parameter('startup_duration').value)

        self.start_position_xyz = self._list_parameter('start_position_xyz')
        if len(self.start_position_xyz) != 3:
            raise ValueError('start_position_xyz must contain [x, y, z]')
        self.start_orientation_xyzw = self._list_parameter('start_orientation_xyzw')
        if len(self.start_orientation_xyzw) != 4:
            raise ValueError('start_orientation_xyzw must contain [x, y, z, w]')
        self.start_duration = float(self.get_parameter('start_duration').value)
        self.return_duration = float(self.get_parameter('return_duration').value)

        self.pregrasp_distance = float(self.get_parameter('pregrasp_distance').value)
        self.pregrasp_duration = float(self.get_parameter('pregrasp_duration').value)
        self.insertion_duration = float(self.get_parameter('insertion_duration').value)
        self.insertion_overshoot_distance = float(
            self.get_parameter('insertion_overshoot_distance').value
        )
        self.movel_subscriber_timeout = float(
            self.get_parameter('movel_subscriber_timeout').value
        )
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.eef_link = str(self.get_parameter('eef_link').value)
        self.fixed_grasp_z = float(self.get_parameter('fixed_grasp_z').value)
        self.min_grasp_z = float(self.get_parameter('min_grasp_z').value)

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
        self.second_shoe_grasp_y_offset = float(
            self.get_parameter('second_shoe_grasp_y_offset').value
        )
        self.shoe_depth_center_offset = float(
            self.get_parameter('shoe_depth_center_offset').value
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

        shoe_yaw_reference_orientation = np.asarray(
            self._list_parameter('shoe_yaw_reference_orientation_xyzw'), dtype=np.float64
        )
        if shoe_yaw_reference_orientation.shape != (4,):
            raise ValueError('shoe_yaw_reference_orientation_xyzw must contain [x, y, z, w]')
        reference_norm = np.linalg.norm(shoe_yaw_reference_orientation)
        if reference_norm < 1e-9:
            raise ValueError('shoe_yaw_reference_orientation_xyzw must not be zero')
        self.shoe_yaw_reference_orientation = shoe_yaw_reference_orientation / reference_norm

        shoe_yaw_axis = np.asarray(self._list_parameter('shoe_yaw_axis_xyz'), dtype=np.float64)
        if shoe_yaw_axis.shape != (3,):
            raise ValueError('shoe_yaw_axis_xyz must contain [x, y, z]')
        shoe_yaw_axis_norm = np.linalg.norm(shoe_yaw_axis)
        if shoe_yaw_axis_norm < 1e-9:
            raise ValueError('shoe_yaw_axis_xyz must not be zero')
        self.shoe_yaw_axis = shoe_yaw_axis / shoe_yaw_axis_norm

        self.shoe_yaw_flip_threshold_deg = float(
            self.get_parameter('shoe_yaw_flip_threshold_deg').value
        )
        self.grasp_fixed_roll = float(self.get_parameter('grasp_fixed_roll').value)
        self.grasp_fixed_pitch = float(self.get_parameter('grasp_fixed_pitch').value)
        self.grasp_yaw_from_shoe_yaw_scale = float(
            self.get_parameter('grasp_yaw_from_shoe_yaw_scale').value
        )
        self.grasp_yaw_offset = float(self.get_parameter('grasp_yaw_offset').value)
        self.yaw_clamp_min_deg = float(self.get_parameter('yaw_clamp_min_deg').value)
        self.yaw_clamp_max_deg = float(self.get_parameter('yaw_clamp_max_deg').value)

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
        self.gripper_release_position = float(
            self.get_parameter('gripper_release_position').value
        )
        self.gripper_duration = float(self.get_parameter('gripper_duration').value)
        self.gripper_settle_time = float(self.get_parameter('gripper_settle_time').value)
        self.command_rate_hz = float(self.get_parameter('command_rate_hz').value)
        self.lift_height = float(self.get_parameter('lift_height').value)
        self.lift_duration = float(self.get_parameter('lift_duration').value)

        self.place_slots = [self._build_place_slot(1), self._build_place_slot(2)]
        self.place_hover_duration = float(self.get_parameter('place_hover_duration').value)
        self.place_lower_distance = float(self.get_parameter('place_lower_distance').value)
        self.place_lower_duration = float(self.get_parameter('place_lower_duration').value)
        self.place_duration = float(self.get_parameter('place_duration').value)

        # TF/락/퍼블리셔/구독/서비스 등 공통 초기화는 PickPlaceNodeBase에 위임
        self._setup_common()

        self.get_logger().warn(
            'ShoesPick is a DRAFT: grasp_position_offset (shoe hole x/y offset) and '
            'place_lower_distance (how far to lower before releasing) are not '
            'independently measured yet.'
        )
        self.get_logger().info('ShoesPick (left arm) ready')
        self.get_logger().info(f'  MoveL: {self.movel_topic}')
        self.get_logger().info(f'  detections: {self.detections_topic}')
        self.get_logger().info(f'  camera info: {self.camera_info_topic}')
        self.get_logger().info(f'  depth (x/y only): {self.depth_topic}')
        for index, slot in enumerate(self.place_slots, start=1):
            self.get_logger().info(f'  place slot {index} hover: {slot["position"]}')

        # 노드 시작 시 한 번만: startup_pose -> start_pose 둘 다 아주 천천히
        # 왼팔을 이동. execute_motion=true일 때만 실행하고, 블로킹 이동이라
        # 백그라운드 스레드로 돌려서 노드 초기화/spin을 막지 않게 한다.
        if self.execute_motion:
            threading.Thread(target=self._startup_sequence, daemon=True).start()

    # 노드가 막 시작됐을 때 왼팔을 안전하게 신발 시작 자세로 데려가는 시퀀스.
    # ~/execute의 pick 시퀀스에는 더 이상 이 이동이 포함되지 않는다.
    def _startup_sequence(self):
        startup_pose = PoseStamped()
        startup_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(
            startup_pose, self.startup_position_xyz, self.startup_orientation_xyzw
        )
        start_pose = PoseStamped()
        start_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(start_pose, self.start_position_xyz, self.start_orientation_xyzw)

        with self.execution_lock:
            self.execution_step = 'startup: move to startup pose (very slow)'
            self.get_logger().info('Executing startup: move to startup pose (very slow)')
            if not self._move_l(startup_pose, duration=self.startup_duration):
                self.get_logger().error('Startup sequence stopped at startup pose')
                self.execution_step = 'idle'
                return

            self.execution_step = 'startup: move to start pose (very slow)'
            self.get_logger().info('Executing startup: move to start pose (very slow)')
            if not self._move_l(start_pose, duration=self.start_duration):
                self.get_logger().error('Startup sequence stopped at start pose')
                self.execution_step = 'idle'
                return

            self.get_logger().info('Startup sequence finished; holding start pose')
            self.execution_step = 'idle'

    # place_slot_{index}_position_xyz / orientation_xyzw 파라미터를 읽어서
    # {'position', 'orientation'} 딕셔너리 하나로 묶어준다 (슬롯 1, 2 각각 호출).
    def _build_place_slot(self, index):
        position = self._list_parameter(f'place_slot_{index}_position_xyz')
        if len(position) != 3:
            raise ValueError(f'place_slot_{index}_position_xyz must contain [x, y, z]')

        orientation = np.asarray(
            self._list_parameter(f'place_slot_{index}_orientation_xyzw'), dtype=np.float64
        )
        if orientation.shape != (4,):
            raise ValueError(f'place_slot_{index}_orientation_xyzw must contain [x, y, z, w]')
        norm = np.linalg.norm(orientation)
        if norm < 1e-9:
            raise ValueError(f'place_slot_{index}_orientation_xyzw must not be zero')

        return {'position': position, 'orientation': orientation / norm}

    # 검출 하나(픽셀 좌표 + depth + 신발 orientation)를 실제 목표 그립 자세로 변환.
    # box_pick.py와 달리 그리퍼 yaw만 신발 yaw를 따라가고 roll/pitch는 고정
    # (위에서 내려다보며 잡는 자세라서).
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
        point, orientation, object_yaw, gripper_yaw = transformed

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.target_frame
        # box_pick.py와 동일한 위치 기반 y 보정: 화면 중앙에 가까워질수록 왼쪽으로
        # 더 잡음 (grasp_position_y_reference_pixel에서 정확히
        # grasp_position_offset[1], 거기서 벗어난 만큼 slope로 보정).
        y_offset = self.grasp_position_offset[1] + self.grasp_position_y_slope * (
            u - self.grasp_position_y_reference_pixel
        )
        pose.pose.position.x = float(point[0] + self.grasp_position_offset[0])
        pose.pose.position.y = float(point[1] + y_offset)
        pose.pose.position.z = fixed_z
        pose.pose.orientation = self._quaternion_message(orientation)

        self.object_pose_pub.publish(pose)
        self.grasp_pose_pub.publish(pose)

        if log:
            self.get_logger().info(
                f'[{index}] pixel=({u:.0f}, {v:.0f})/{camera_info.width}, '
                f'real camera xyz=({real_point_cam[0]:.3f}, {real_point_cam[1]:.3f}, '
                f'{real_point_cam[2]:.3f}), fixed z={fixed_z:.3f}, target=('
                f'{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, '
                f'{pose.pose.position.z:.3f}), '
                f'shoe_yaw={math.degrees(object_yaw):.1f} deg -> '
                f'gripper_yaw={math.degrees(gripper_yaw):.1f} deg'
            )
        return {'pose': pose, 'pixel_u': u}

    # 픽셀(u,v) + 그 지점의 실제 depth로 카메라 좌표계에서의 3D 점(x,y,z)을 역투영.
    # TODO: shoe_depth_center_offset(신발 표면->실제 잡는 지점 깊이 보정)은 아직 0.
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

        depth += self.shoe_depth_center_offset

        return np.array(
            [(u - cx) * depth / fx, (v - cy) * depth / fy, depth], dtype=np.float64
        )

    # ~/execute로 캡처해둔 신발들을 한 번에 하나씩 순서대로 집어서 놓는다.
    # bottle_box.py와 동일: 슬롯 수보다 신발이 많으면 남는 신발은 마지막 슬롯 재사용.
    # 신발과 신발 사이에는 시작 자세로 돌아가지 않고, 놓고 나서 바로 다음 신발의
    # pregrasp로 향한다 -- 시작 자세 복귀는 큐 전체가 끝난 뒤 한 번만.
    def _execute_queue(self, queue):
        with self.execution_lock:
            try:
                for index, item in enumerate(queue):
                    if self.cancel_event.is_set():
                        return
                    label = f'shoe {index + 1}/{len(queue)}'
                    slot = self.place_slots[min(index, len(self.place_slots) - 1)]
                    grasp_pose = item['pose']
                    if index == 1:
                        # 두 번째로 캡처된(오른쪽) 신발만 추가로 오른쪽으로 더 당겨 잡음.
                        grasp_pose.pose.position.y += self.second_shoe_grasp_y_offset
                    if not self._pick_and_place(grasp_pose, slot, label):
                        if self.cancel_event.is_set():
                            return
                        self.get_logger().error(f'{label} failed; continuing with next shoe')
                        continue
                self.get_logger().info(f'Finished picking {len(queue)} shoe(s)')

                if not self.cancel_event.is_set():
                    start_pose = PoseStamped()
                    start_pose.header.frame_id = self.target_frame
                    self._set_pose_from_arrays(
                        start_pose, self.start_position_xyz, self.start_orientation_xyzw
                    )
                    self.execution_step = 'return to start pose (1.5x speed)'
                    self.get_logger().info('Executing return to start pose (1.5x speed)')
                    if not self._move_l(start_pose, duration=self.return_duration):
                        self.get_logger().error('Stopped returning to start pose')
            finally:
                self.execution_step = 'idle'

    # 신발 하나를 집어서(pregrasp -> insert -> close -> lift) 지정된 슬롯에
    # 놓는(hover -> 내려서 놓기 -> release -> 다시 hover) 동작 시퀀스. 팔은 이미
    # start_pose 근처에 있다고 가정 (첫 신발은 _startup_sequence, 다음 신발부터는
    # 이전 신발을 놓고 hover에 있는 상태 그대로 이어서 진행).
    def _pick_and_place(self, grasp_pose, slot, label='shoe'):
        grasp_q = np.array([
            grasp_pose.pose.orientation.x,
            grasp_pose.pose.orientation.y,
            grasp_pose.pose.orientation.z,
            grasp_pose.pose.orientation.w,
        ], dtype=np.float64)
        # box_pick.py와 동일: local -Z가 접근 방향(신발은 대략 수직 아래), 검출마다
        # 방향이 조금씩 다르므로 매번 재계산.
        approach_dir = self._rotate_vector(np.array([0.0, 0.0, -1.0]), grasp_q)

        insert_pose = self._copy_pose(grasp_pose)
        insert_pose.pose.position.x += approach_dir[0] * self.insertion_overshoot_distance
        insert_pose.pose.position.y += approach_dir[1] * self.insertion_overshoot_distance
        insert_pose.pose.position.z += approach_dir[2] * self.insertion_overshoot_distance
        # 안전장치: 근사 계산에 기대지 않고, 절대로 min_grasp_z보다 아래로 내려가지
        # 않도록 직접 clamp (실측된 "여기보다 아래로 내려가면 안 됨" 한계).
        insert_pose.pose.position.z = max(insert_pose.pose.position.z, self.min_grasp_z)

        pregrasp_pose = self._copy_pose(insert_pose)
        pregrasp_pose.pose.position.x -= approach_dir[0] * self.pregrasp_distance
        pregrasp_pose.pose.position.y -= approach_dir[1] * self.pregrasp_distance
        pregrasp_pose.pose.position.z -= approach_dir[2] * self.pregrasp_distance

        lift_pose = self._copy_pose(insert_pose)
        lift_pose.pose.position.z += self.lift_height

        place_hover_pose = PoseStamped()
        place_hover_pose.header.frame_id = self.target_frame
        self._set_pose_from_arrays(place_hover_pose, slot['position'], slot['orientation'])

        place_lower_pose = self._copy_pose(place_hover_pose)
        place_lower_pose.pose.position.z -= self.place_lower_distance

        steps = [
            # startup_pose -> start_pose 이동은 이제 노드 시작 시 한 번만
            # (_startup_sequence)이라 여기서는 안 함 -- 팔은 이미 start_pose에 있다고
            # 가정.
            ('open gripper', lambda: self._move_gripper(self.gripper_open_position)),
            ('move to pregrasp', lambda: self._move_l(pregrasp_pose, duration=self.pregrasp_duration)),
            ('insert to shoe', lambda: self._move_l(insert_pose, duration=self.insertion_duration)),
            ('close gripper', lambda: self._move_gripper(self.gripper_closed_position)),
            ('lift shoe', lambda: self._move_l(lift_pose, duration=self.lift_duration)),
            ('move to place hover', lambda: self._move_l(place_hover_pose, duration=self.place_hover_duration)),
            ('lower into place', lambda: self._move_l(place_lower_pose, duration=self.place_lower_duration)),
            ('release shoe', lambda: self._move_gripper(self.gripper_release_position)),
            # 다음 신발이 있으면 여기서 바로 그 pregrasp로 이어감 (시작 자세를
            # 거치지 않음) -- _execute_queue 참고.
            ('back to place hover', lambda: self._move_l(place_hover_pose, duration=self.place_duration)),
        ]

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

    # 카메라 좌표계의 점 + 신발 orientation을 base_link 기준 (위치, 목표 그리퍼
    # orientation, 신발 yaw, 그리퍼 yaw)로 변환. box_pick.py의 _transform_centerpose와
    # 같은 구조지만, 신발은 위에서 내려다보며 잡으므로 roll이 아니라 그리퍼 yaw가
    # 신발 yaw를 따라간다 (roll/pitch는 고정).
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

        object_yaw, raw_angle = self._signed_shoe_yaw(object_q)
        if math.degrees(raw_angle) > self.shoe_yaw_flip_threshold_deg:
            object_q = self._quaternion_multiply(object_q, np.array([0.0, 1.0, 0.0, 0.0]))
            object_yaw, _ = self._signed_shoe_yaw(object_q)

        raw_yaw_deg = math.degrees(
            self.grasp_yaw_from_shoe_yaw_scale * object_yaw + self.grasp_yaw_offset
        )
        clamped_yaw_deg = min(
            max(raw_yaw_deg, self.yaw_clamp_min_deg), self.yaw_clamp_max_deg
        )
        gripper_yaw = self._wrap_angle(math.radians(clamped_yaw_deg))
        base_q = self._quaternion_from_euler(
            self.grasp_fixed_roll, self.grasp_fixed_pitch, gripper_yaw
        )
        target_q = self._quaternion_multiply(base_q, self.tool_orientation_offset)
        return point, target_q, object_yaw, gripper_yaw

    def _signed_shoe_yaw(self, object_q):
        """shoe_yaw_reference_orientation 대비 object_q의 signed 각도(rad).

        (signed_angle, unsigned_angle) 반환 -- box_pick.py의 _signed_box_yaw와 동일.
        """
        relative_q = self._quaternion_multiply(
            self._quaternion_conjugate(self.shoe_yaw_reference_orientation), object_q
        )
        axis, angle = self._quaternion_axis_angle(relative_q)
        sign = 1.0 if np.dot(axis, self.shoe_yaw_axis) >= 0.0 else -1.0
        return sign * angle, angle

    # --- 아래는 전부 순수 쿼터니언/각도 계산용 정적 헬퍼 (box_pick.py와 동일) ---
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
    node = ShoesPick()
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
