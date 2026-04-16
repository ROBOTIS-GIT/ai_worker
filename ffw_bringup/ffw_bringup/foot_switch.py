#!/usr/bin/env python3

import os
import struct
import time

import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from builtin_interfaces.msg import Duration
from std_msgs.msg import UInt8
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

DEVICE = "/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd"

EV_KEY = 0x01
INPUT_EVENT_FORMAT = "llHHi"
INPUT_EVENT_SIZE = struct.calcsize(INPUT_EVENT_FORMAT)

KEY_LEFT = 30
KEY_MIDDLE = 48
KEY_RIGHT = 46

# 같은 버튼에서 너무 짧은 시간 안에 다시 들어온 이벤트는 무시
DEBOUNCE_SEC = 0.5
DEBOUNCE_ENABLED = True


class FootSwitchReader(Node):
    def __init__(self, device: str):
        super().__init__("foot_switch_node")
        self.device = device
        self.fd = None

        # (버튼, value)별 마지막 처리 시각
        self.last_event_time = {}

        # Middle 페달이 눌려 있는 동안 deadzone set을 한 번만 발행하기 위한 플래그
        self.already_middle_pub = False

        # Parameter client for joystick_controller deadzone
        self.deadzone_param_client = self.create_client(
            SetParameters,
            '/leader/joystick_controller/set_parameters'
        )

        # enable publishers (0=disable, 1=enable, 2=toggle)
        self.left_enable_pub = self.create_publisher(
            UInt8, "/leader/left_enable", 1)
        self.right_enable_pub = self.create_publisher(
            UInt8, "/leader/right_enable", 1)

        # trajectory publishers
        self.left_traj_pub = self.create_publisher(
            JointTrajectory,
            "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
            10,
        )
        self.right_traj_pub = self.create_publisher(
            JointTrajectory,
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
            10,
        )

        # joint names — load from controller config yaml
        self.declare_parameter('controller_config_path', '')
        self.left_joint_names = []
        self.right_joint_names = []
        self._load_joint_names_from_yaml()

        # Saved postures
        self.left_position = [0.5, 0.32, 0.0, -2.05, 0.25, -0.0, -1.0, 0.0]
        self.right_position = [0.5, -0.32, -0.0, -2.05, -0.25, -0.0, 1.0, 0.0]

        self.traj_duration_sec = 3.0

    def open_device(self):
        try:
            # self.fd = os.open(DEVICE, os.O_RDONLY | os.O_NONBLOCK)
            self.fd = os.open(self.device, os.O_RDONLY)
            print(f"Opened device: {self.device}")
        except PermissionError:
            print(f"Permission denied: {self.device}")
            raise
        except FileNotFoundError:
            print(f"Device not found: {self.device}")
            raise

    def close_device(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
            print("Device closed")

    def get_key_name(self, event_code: int) -> str:
        if event_code == KEY_LEFT:
            return "left"
        if event_code == KEY_MIDDLE:
            return "middle"
        if event_code == KEY_RIGHT:
            return "right"
        return f"unknown({event_code})"

    def is_debounced(self, event_code: int, event_value: int) -> bool:
        if not DEBOUNCE_ENABLED:
            return True

        key = (event_code, event_value)
        now = time.monotonic()
        last_time = self.last_event_time.get(key, 0.0)

        if now - last_time < DEBOUNCE_SEC:
            return False

        self.last_event_time[key] = now
        return True

    def _load_joint_names_from_yaml(self):
        yaml_path = self.get_parameter('controller_config_path').value
        if not yaml_path or not os.path.isfile(yaml_path):
            # Fallback: find config from package share directory
            try:
                pkg_path = get_package_share_directory('ffw_bringup')
                yaml_path = os.path.join(
                    pkg_path, 'config', 'ffw_lg2_mini_leader',
                    'ffw_lg2_mini_leader_ai_hardware_controller.yaml')
                print(f"controller_config_path not set, using default: {yaml_path}")
            except Exception:
                pass
        if not yaml_path or not os.path.isfile(yaml_path):
            raise RuntimeError(
                f"controller_config_path is empty or file not found: '{yaml_path}'"
            )

        with open(yaml_path, 'r') as f:
            lines = f.readlines()

        def extract_list(key: str):
            result = []
            in_block = False
            indent = None
            for line in lines:
                stripped = line.rstrip('\n')
                if not in_block:
                    if stripped.lstrip().startswith(f'{key}:'):
                        indent = len(stripped) - len(stripped.lstrip())
                        in_block = True
                    continue
                cur_indent = len(stripped) - len(stripped.lstrip())
                item = stripped.lstrip()
                if item.startswith('- '):
                    result.append(item[2:].strip())
                elif stripped.strip() and cur_indent <= indent:
                    break
            return result

        self.left_joint_names = extract_list('left_joints')
        self.right_joint_names = extract_list('right_joints')

        if not self.left_joint_names or not self.right_joint_names:
            raise RuntimeError(
                f"Failed to parse left_joints/right_joints from {yaml_path}"
            )

        print(f"Loaded left_joints: {self.left_joint_names}")
        print(f"Loaded right_joints: {self.right_joint_names}")

    def _set_deadzone(self, value: float):
        req = SetParameters.Request()
        param = Parameter()
        param.name = 'deadzone'
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=value,
        )
        req.parameters = [param]
        self.deadzone_param_client.call_async(req)
        print(f"deadzone : {value}")

    def _make_trajectory(self, joint_names, positions):
        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions

        sec = int(self.traj_duration_sec)
        nanosec = int((self.traj_duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        traj.points.append(point)
        return traj

    def handle_left(self, event_value: int):
        if event_value == 1:
            msg = UInt8()
            msg.data = 0
            self.left_enable_pub.publish(msg)

            # Give broadcaster time to process the disable before publishing trajectory
            time.sleep(0.1)

            traj = self._make_trajectory(self.left_joint_names, self.left_position)
            self.left_traj_pub.publish(traj)
            print("((Publish) left arm trajectory")

    def handle_middle(self, event_value: int):
        # 2 (repeat) 동안만 deadzone을 0.05로, 0 (release) 시 1.0로 복귀
        if event_value == 2:
            if not self.already_middle_pub:
                self._set_deadzone(0.05)
                self.already_middle_pub = True
        elif event_value == 0:
            if self.already_middle_pub:
                self._set_deadzone(1.0)
                self.already_middle_pub = False

    def handle_right(self, event_value: int):
        if event_value == 1:
            msg = UInt8()
            msg.data = 0
            self.right_enable_pub.publish(msg)

            # Give broadcaster time to process the disable before publishing trajectory
            time.sleep(0.1)

            traj = self._make_trajectory(self.right_joint_names, self.right_position)
            self.right_traj_pub.publish(traj)
            print("((Publish) right arm trajectory")

    def process_key_event(self, event_code: int, event_value: int):

        # left / middle / right 외 입력 무시
        if event_code not in (KEY_LEFT, KEY_MIDDLE, KEY_RIGHT):
            return

        if not self.is_debounced(event_code, event_value):
            return

        key_name = self.get_key_name(event_code)
        print(f"input detected -> key: {key_name}, value: {event_value}")

        if event_code == KEY_LEFT:
            self.handle_left(event_value)
        elif event_code == KEY_MIDDLE:
            self.handle_middle(event_value)
        elif event_code == KEY_RIGHT:
            self.handle_right(event_value)

    def run(self):
        if self.fd is None:
            raise RuntimeError("Device is not opened")

        print("Start reading foot switch (blocking mode)...")

        while True:
            data = os.read(self.fd, INPUT_EVENT_SIZE)

            if len(data) != INPUT_EVENT_SIZE:
                print(f"Incomplete input event: {len(data)} bytes")
                continue

            _, _, event_type, event_code, event_value = struct.unpack(
                INPUT_EVENT_FORMAT, data
            )

            if event_type != EV_KEY:
                continue

            self.process_key_event(event_code, event_value)


def main(args=None):
    rclpy.init(args=args)
    reader = FootSwitchReader(DEVICE)

    try:
        reader.open_device()
        reader.run()
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        reader.close_device()
        reader.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()