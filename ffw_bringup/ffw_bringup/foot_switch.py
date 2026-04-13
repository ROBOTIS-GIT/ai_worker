#!/usr/bin/env python3

import os
import struct
import time

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEVICE = "/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd"

EV_KEY = 0x01
INPUT_EVENT_FORMAT = "llHHi"
INPUT_EVENT_SIZE = struct.calcsize(INPUT_EVENT_FORMAT)

KEY_LEFT = 30
KEY_MIDDLE = 48
KEY_RIGHT = 46

# Middle pedal press duration threshold.
# duration < threshold -> "right" (record/save toggle)
# duration >= threshold -> "left" (discard/cancel path)
MIDDLE_LONG_PRESS_SEC = 2.0

# Ignore rapid duplicate middle-button events caused by switch bounce.
MIDDLE_DEBOUNCE_SEC = 0.05


class FootSwitchTrajectoryNode(Node):
    def __init__(self):
        super().__init__("foot_switch_trajectory_node")

        # enable false publisher
        self.left_enable_pub = self.create_publisher(Bool, "/leader/left_enable", 10)
        self.right_enable_pub = self.create_publisher(Bool, "/leader/right_enable", 10)

        # trajectory publisher
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

        # ai_server trigger publisher (reuse existing joystick trigger path)
        self.tact_trigger_pub = self.create_publisher(
            String,
            "/leader/joystick_controller/tact_trigger",
            10,
        )

        # joint names
        self.left_joint_names = [
            "arm_l_joint1",
            "arm_l_joint2",
            "arm_l_joint3",
            "arm_l_joint4",
            "arm_l_joint5",
            "arm_l_joint6",
            "arm_l_joint7",
            "gripper_l_joint1",
        ]

        self.right_joint_names = [
            "arm_r_joint1",
            "arm_r_joint2",
            "arm_r_joint3",
            "arm_r_joint4",
            "arm_r_joint5",
            "arm_r_joint6",
            "arm_r_joint7",
            "gripper_r_joint1",
        ]

        # Saved postures for left/right pedal trajectory behavior
        self.left_position = [0.75, 0.0, 0.0, -2.3, 0.0, 0.5, 0.0, 0.0]
        self.right_position = [0.75, 0.0, 0.0, -2.3, 0.0, -0.5, 0.0, 0.0]

        self.traj_duration_sec = 5.0

        # Middle pedal state for short/long press detection
        self.middle_pressed = False
        self.middle_press_time = 0.0
        self.middle_last_event_time = 0.0

        # Open foot switch device in non-blocking mode for timer callback safety
        try:
            self.fd = os.open(DEVICE, os.O_RDONLY | os.O_NONBLOCK)
            self.get_logger().info(f"Opened device: {DEVICE}")
        except PermissionError:
            self.get_logger().error(
                f"Permission denied: {DEVICE}\n"
                f"Try: sudo chmod 666 {DEVICE}\n"
                f"or run with sudo / udev rule."
            )
            raise
        except FileNotFoundError:
            self.get_logger().error(f"Device not found: {DEVICE}")
            raise

        # Polling timer
        self.timer = self.create_timer(0.01, self.read_foot_switch)

    def publish_disable(self):
        msg = Bool()
        msg.data = False
        self.left_enable_pub.publish(msg)
        self.right_enable_pub.publish(msg)
        self.get_logger().info("Published false to /leader/left_enable and /leader/right_enable")

    def make_trajectory(self, joint_names, positions):
        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions

        sec = int(self.traj_duration_sec)
        nanosec = int((self.traj_duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        traj.points.append(point)
        return traj

    def publish_target(self, positions, label, side):
        # Publish disable first
        self.publish_disable()

        if side == "left":
            traj = self.make_trajectory(self.left_joint_names, positions)
            self.left_traj_pub.publish(traj)
            self.get_logger().info(
                f"{label} pedal pressed -> published left-arm trajectory"
            )
        elif side == "right":
            traj = self.make_trajectory(self.right_joint_names, positions)
            self.right_traj_pub.publish(traj)
            self.get_logger().info(
                f"{label} pedal pressed -> published right-arm trajectory"
            )
        else:
            self.get_logger().error(f"Unknown side '{side}' for pedal label '{label}'")

    def publish_tact_trigger(self, trigger: str):
        msg = String()
        msg.data = trigger
        self.tact_trigger_pub.publish(msg)
        self.get_logger().info(
            f"Middle pedal -> published tact trigger '{trigger}' to /leader/joystick_controller/tact_trigger"
        )

    def handle_middle_press(self):
        # Ignore duplicate press events while already pressed
        now = time.monotonic()

        if self.middle_pressed:
            return

        if now - self.middle_last_event_time < MIDDLE_DEBOUNCE_SEC:
            self.get_logger().debug("Ignoring bounced middle press")
            return

        self.middle_pressed = True
        self.middle_press_time = now
        self.middle_last_event_time = now

    def handle_middle_release(self):
        # Atomically check and clear to prevent duplicate processing
        now = time.monotonic()

        if not self.middle_pressed:
            return

        self.middle_pressed = False

        if now - self.middle_last_event_time < MIDDLE_DEBOUNCE_SEC:
            self.get_logger().debug("Ignoring bounced middle release")
            return

        self.middle_last_event_time = now

        duration = now - self.middle_press_time

        if duration >= MIDDLE_LONG_PRESS_SEC:
            # Long press -> left (discard/cancel path in ai_server)
            self.publish_tact_trigger("left")
            self.get_logger().info(
                f"Middle long press ({duration:.3f}s) -> left"
            )
        else:
            # Short press -> right (record/save toggle in ai_server)
            self.publish_tact_trigger("right")
            self.get_logger().info(
                f"Middle short press ({duration:.3f}s) -> right"
            )

    def handle_key_event(self, event_code: int, event_value: int):
        # event_value: 0=release, 1=press, 2=repeat
        if event_code == KEY_MIDDLE:
            if event_value == 1:
                self.handle_middle_press()
            elif event_value == 0:
                self.handle_middle_release()
            # ignore repeat(2)
            return

        # Keep existing left/right behavior unchanged: act on key press only
        if event_value != 1:
            return

        if event_code == KEY_LEFT:
            self.publish_target(self.left_position, "left", side="left")
        elif event_code == KEY_RIGHT:
            self.publish_target(self.right_position, "right", side="right")

    def read_foot_switch(self):
        try:
            # Drain all available input events from non-blocking fd.
            while True:
                data = os.read(self.fd, INPUT_EVENT_SIZE)
                if len(data) != INPUT_EVENT_SIZE:
                    break

                _, _, event_type, event_code, event_value = struct.unpack(
                    INPUT_EVENT_FORMAT, data
                )

                if event_type != EV_KEY:
                    continue

                self.handle_key_event(event_code, event_value)

        except BlockingIOError:
            # No new input event available in non-blocking mode.
            return
        except Exception as e:
            self.get_logger().error(f"Error while reading foot switch: {e}")

    def destroy_node(self):
        try:
            if hasattr(self, "fd"):
                os.close(self.fd)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FootSwitchTrajectoryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
