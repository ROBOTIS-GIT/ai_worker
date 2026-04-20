#!/usr/bin/env python3

import os
import struct
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import UInt8

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

# enable 값: left/right 페달 누르면 이 값을 enable 토픽에 발행
# broadcaster는 이 값에 맞는 save pose로 보간
SAVE_POSE_ID = 4


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

        # enable publishers (0=disable, 1=enable, 2=toggle, 3+=save pose N)
        self.left_enable_pub = self.create_publisher(
            UInt8, "/leader/left_command", 1)
        self.right_enable_pub = self.create_publisher(
            UInt8, "/leader/right_command", 1)

    def open_device(self):
        try:
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

    def handle_left(self, event_value: int):
        if event_value == 1:
            msg = UInt8()
            msg.data = SAVE_POSE_ID
            self.left_enable_pub.publish(msg)
            print(f"(Publish) left_enable = {SAVE_POSE_ID}")

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
            msg.data = SAVE_POSE_ID
            self.right_enable_pub.publish(msg)
            print(f"(Publish) right_enable = {SAVE_POSE_ID}")

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
