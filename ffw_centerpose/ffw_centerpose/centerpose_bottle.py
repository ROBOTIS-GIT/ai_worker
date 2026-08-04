#!/usr/bin/env python3

import math

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

from ffw_centerpose.pick_place_base import PickPlaceNodeBase


class CenterposeBottle(PickPlaceNodeBase):
    """CenterPose로 검출한 병을 왼팔로 집어서 정해진 박스에 놓는 노드.

    그리퍼 방향은 항상 고정(bottle_ready 자세 기준)이고, x/y/z 위치만 검출을
    따라간다. 공통 로직은 PickPlaceNodeBase 참고.
    """

    _OBJECT_LABEL_PLURAL = 'bottle(s)'

    # 파라미터 선언/읽기 후 공통 초기화(_setup_common) 호출까지 한 번에 처리.
    def __init__(self):
        super().__init__('centerpose_bottle')

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
        self.declare_parameter('movel_duration', 5.0)
        # 직진으로 바로 잡으면 그리퍼가 병에 부딪힐 수 있어서, 진입축을 따라
        # 이만큼 뒤로 뺀 pregrasp를 거쳐서 들어감.
        self.declare_parameter('pregrasp_distance', 0.08)
        self.declare_parameter('pregrasp_duration', 2.0)
        self.declare_parameter('insertion_duration', 1.0)
        self.declare_parameter('movel_subscriber_timeout', 2.0)
        self.declare_parameter('settle_time', 0.5)
        self.declare_parameter('eef_link', 'end_effector_l_link')
        # 병 높이도 안 믿고 z는 항상 이 고정값 사용. 음수면 캡처 시점의 현재
        # 엔드이펙터 z를 대신 사용.
        self.declare_parameter('fixed_grasp_z', 0.8241714239120483)
        # 실제 로봇에서 측정한 grasp 오차로 보정한 값.
        self.declare_parameter('grasp_position_offset', [0.01, -0.04, 0.0])
        # 모든 단계에서 고정으로 쓰는 그리퍼 방향 (bottle_ready 자세에서 tf2_echo로
        # 측정). 병 방향엔 안 따라감.
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
        # MoveL 컨트롤러가 같은 토픽에 계속 스트리밍하므로, 그리퍼 명령도 이 주기로
        # 계속 재전송해야 밀려나지 않음.
        self.declare_parameter('command_rate_hz', 300.0)
        self.declare_parameter('lift_height', 0.1)
        self.declare_parameter('lift_duration', 1.0)
        # 놓는 위치: hover -> box_place_z_offset만큼 내려서 release -> 다시 hover.
        # 슬롯 2개, 캡처 순서(왼쪽부터)로 채움 -- 슬롯 1개만 쓸 땐 슬롯1만 사용.
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
        # hover에서 놓는 높이까지 내려가는 z 거리 (두 슬롯 공통값).
        self.declare_parameter('box_place_z_offset', 0.1093128323554992)
        self.declare_parameter('box_place_duration', 1.0)
        self.declare_parameter('return_to_initial', True)
        # bottle_ready 초기 자세에서 tf2_echo로 측정한 원위치.
        self.declare_parameter(
            'home_position_xyz',
            [0.13451801240444183, 0.2999741733074188, 0.9742214239120483],
        )
        self.declare_parameter('home_duration', 3.0)

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
        # grasp_orientation 기준 진입 방향(local -Z가 대략 병 쪽을 향함) -- 방향이
        # 고정이라 한 번만 계산해둠.
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

        # TF/락/퍼블리셔/구독/서비스 등 공통 초기화는 PickPlaceNodeBase에 위임
        self._setup_common()

        self.get_logger().info('CenterposeBottle (left arm, fixed orientation) ready')
        self.get_logger().info(f'  MoveL: {self.movel_topic}')
        self.get_logger().info(f'  detections: {self.detections_topic}')
        self.get_logger().info(f'  camera info: {self.camera_info_topic}')
        self.get_logger().info(f'  depth (x/y only): {self.depth_topic}')
        for index, slot in enumerate(self.box_slots, start=1):
            self.get_logger().info(f'  box slot {index} hover: {slot["position"]}')

    # box_slot_{index}_position_xyz / orientation_xyzw 파라미터를 읽어서
    # {'position', 'orientation'} 딕셔너리 하나로 묶어준다 (슬롯 1, 2 각각 호출).
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

    # 검출 하나(픽셀 좌표 + depth)를 실제 목표 그립 자세(PoseStamped)로 변환.
    # 방향은 항상 grasp_orientation 고정값 -- 병 방향에 안 따라감(이게 centerpose_box와의
    # 핵심 차이).
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

    # 픽셀(u,v) + 그 지점의 실제 depth로 카메라 좌표계에서의 3D 점(x,y,z)을 역투영.
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

    # ~/execute로 캡처해둔 병들을 한 번에 하나씩 순서대로 집어서 박스 슬롯에 넣는다.
    def _execute_queue(self, queue):
        with self.execution_lock:
            try:
                for index, item in enumerate(queue):
                    if self.cancel_event.is_set():
                        return
                    label = f'bottle {index + 1}/{len(queue)}'
                    # 슬롯 개수보다 병이 많으면 마지막 슬롯을 재사용.
                    slot = self.box_slots[min(index, len(self.box_slots) - 1)]
                    if not self._pick_and_place(item['pose'], slot, label):
                        if self.cancel_event.is_set():
                            return
                        self.get_logger().error(f'{label} failed; continuing with next bottle')
                        continue
                self.get_logger().info(f'Finished picking {len(queue)} bottle(s)')
            finally:
                self.execution_step = 'idle'

    # 병 하나를 실제로 집어서(pregrasp->insert->close->lift) 지정된 박스 슬롯에
    # 놓고(hover->내려놓기->release->다시 hover) 오는 전체 동작 시퀀스.
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
            # 박스에 가깝게 낮은 채로 빼지 않고, hover 높이로 먼저 올라간 뒤 집으로
            # 이동 (박스에 부딪힐 위험을 줄임).
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


def main(args=None):
    rclpy.init(args=args)
    node = CenterposeBottle()  # 노드 실행 진입점 (ros2 run/launch에서 호출됨)
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
