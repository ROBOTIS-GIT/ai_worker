# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import time

from ffw_teleop.pedal import PedalHold
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from robotis_interfaces.srv import SetTeleoperation
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool


class PedalInputNode(Node):
    """Drive the pedal and toggle teleoperation through existing public services."""

    def __init__(self):
        super().__init__('pedal_input')
        self.joint_name = self.declare_parameter('joint_name', 'pedal_joint').value
        desired_position = self.declare_parameter('desired_position', 0.5).value
        if not math.isfinite(desired_position):
            raise ValueError('desired_position must be finite')
        self.hold = PedalHold(
            self.declare_parameter('pressed_center', 0.0).value,
            self.declare_parameter('pressed_window', 0.1).value,
            self.declare_parameter('long_press_seconds', 1.0).value,
            self.declare_parameter('communication_timeout_seconds', 1.0).value)
        self.request_timeout = self.declare_parameter('request_timeout_seconds', 10.0).value
        self.transition_timeout = self.declare_parameter('transition_timeout_seconds', 60.0).value
        if any(not math.isfinite(value) or value <= 0.0 for value in (
                self.request_timeout, self.transition_timeout)):
            raise ValueError('Service and transition timeouts must be finite and positive')
        self.source_client = self.create_client(
            SetBool, self.declare_parameter('set_source_service', '/set_source').value)
        self.enabled_client = self.create_client(
            SetTeleoperation, self.declare_parameter('set_enabled_service', '/set_enabled').value)
        self.source = None
        self.source_seen = 0.0
        self.phase = None
        self.pending = None
        self.starting = False
        self.stop_requested = False
        self.transition_seen = False
        self.next_enable = 0.0
        self.enable_rejection_logged = False
        source_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.source_subscription = self.create_subscription(
            String, self.declare_parameter('source_topic', '/source').value,
            self.on_source, source_qos)
        self.commands_pub = self.create_publisher(
            Float64MultiArray, 'position_controller/commands', 1)
        self.command = Float64MultiArray(data=[desired_position])
        self.subscription = self.create_subscription(
            JointState, 'joint_states', self.on_joint_states, qos_profile_sensor_data)
        self.timer = self.create_timer(
            0.1, self.on_timer, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'Hold pedal for {self.hold.duration:.1f}s to start/stop both-arm teleop; '
            'release it first. A leader bringup must also be running.')

    def on_joint_states(self, message):
        """Accept only a complete, unambiguous pedal position sample."""
        if (len(message.name) != len(message.position) or
                message.name.count(self.joint_name) != 1):
            self.hold.reset()
            return
        position = message.position[message.name.index(self.joint_name)]
        if self.hold.update(position, time.monotonic()):
            self.request_toggle()

    def on_source(self, message):
        """Track source heartbeat and cancel a start that another request superseded."""
        self.source = message.data
        self.source_seen = time.monotonic()
        if self.phase is not None and self.starting:
            if self.source in ('switching', 'teleop'):
                self.transition_seen = True
            elif self.source != 'model_action' or self.transition_seen:
                self.finish_transition('start cancelled: control source changed', warning=True)

    def request_toggle(self):
        """Start one transaction from the current source, never from a local toggle flag."""
        now = time.monotonic()
        if self.phase is not None:
            if self.starting and not self.stop_requested:
                self.stop_requested = True
                self.get_logger().info(
                    'Pedal start cancelled; waiting for source readiness to return external')
            else:
                self.get_logger().warning('Pedal press ignored: stop is already in progress')
            return
        if now - self.source_seen > 0.5 or self.source not in ('teleop', 'model_action'):
            self.get_logger().warning('Pedal request ignored: control source is not ready')
            return
        self.starting = self.source != 'teleop'
        if self.starting and not (
                self.source_client.service_is_ready() and self.enabled_client.service_is_ready()):
            self.get_logger().warning(
                'Pedal start not sent: source or arm-enable service is unavailable')
            return
        self.transition_seen = False
        self.stop_requested = False
        self.enable_rejection_logged = False
        self.next_enable = now
        self.transition_deadline = now + self.transition_timeout
        self.phase = 'acquire_source' if self.starting else 'disable_arms'
        self.get_logger().info(f'Pedal {"start" if self.starting else "stop"} requested')

    def finish_transition(self, message, warning=False):
        """Discard local futures and never continue a cancelled transaction."""
        if self.pending is not None:
            self.pending.cancel()
            self.pending = None
        self.phase = None
        if warning:
            self.get_logger().warning('Pedal ' + message)
        else:
            self.get_logger().info('Pedal ' + message)

    def advance_transition(self, now):
        """Advance asynchronous service calls without blocking pedal or source callbacks."""
        if self.phase is None:
            return
        if now >= self.transition_deadline or (self.starting and now - self.source_seen > 0.5):
            self.finish_transition('transition cancelled: timeout or source heartbeat lost', True)
            return
        if self.pending is not None:
            if not self.pending.done():
                if now >= self.request_deadline:
                    if self.phase == 'disable_arms':
                        self.pending.cancel()
                        self.pending = None
                        self.phase = 'release_source'
                        self.get_logger().warning(
                            'Pedal arm-disable response timed out; requesting external source')
                    else:
                        self.finish_transition(
                            'service response timed out; request cancelled', True)
                return
            future, self.pending = self.pending, None
            try:
                response = future.result()
            except Exception as error:
                if self.phase != 'disable_arms':
                    self.finish_transition(f'service request failed: {error}', True)
                    return
                self.get_logger().warning(f'Pedal arm-disable failed: {error}')
                self.phase = 'release_source'
            else:
                if self.phase in ('acquire_source', 'release_source'):
                    if not response.success:
                        self.finish_transition(f'source change rejected: {response.message}', True)
                        return
                    self.phase = 'wait_teleop' if self.starting else 'wait_external'
                elif self.phase == 'enable_arms':
                    if response.accepted and not self.stop_requested:
                        self.finish_transition('both-arm teleop enable accepted; source: teleop')
                        return
                    if not response.accepted and not self.enable_rejection_logged:
                        self.get_logger().warning(
                            f'Pedal waiting for arm enable: {response.message}')
                        self.enable_rejection_logged = True
                    self.next_enable = now + 0.1
                    self.phase = 'wait_teleop'
                else:
                    if not response.accepted:
                        self.get_logger().warning(
                            f'Pedal arm-disable rejected: {response.message}; releasing source')
                    self.phase = 'release_source'
        if self.phase == 'wait_external':
            if self.source == 'model_action' and now - self.source_seen <= 0.5:
                self.finish_transition('stop completed; source: external')
            return
        if self.phase == 'wait_teleop':
            if self.source != 'teleop' or now < self.next_enable:
                return
            if self.stop_requested:
                self.starting = False
                self.phase = 'disable_arms'
            else:
                self.phase = 'enable_arms'
        if self.phase in ('acquire_source', 'release_source'):
            client = self.source_client
            request = SetBool.Request(data=self.starting)
        else:
            client = self.enabled_client
            request = SetTeleoperation.Request(
                target_arm='both', enabled=self.phase == 'enable_arms')
        if not client.service_is_ready():
            if self.phase == 'disable_arms':
                self.phase = 'release_source'
                self.get_logger().warning('Pedal arm-enable service unavailable; releasing source')
            else:
                self.finish_transition(f'service unavailable: {client.srv_name}', True)
            return
        try:
            self.pending = client.call_async(request)
            self.request_deadline = now + self.request_timeout
        except Exception as error:
            if self.phase == 'disable_arms':
                self.phase = 'release_source'
                self.get_logger().warning(f'Pedal arm-disable failed: {error}; releasing source')
            else:
                self.finish_transition(f'service request failed: {error}', True)

    def on_timer(self):
        """Maintain the pedal spring and advance the nonblocking handoff using wall time."""
        self.commands_pub.publish(self.command)
        if self.hold.check_timeout(time.monotonic()):
            self.get_logger().warning(
                'Pedal feedback lost; pending press cleared. Release after reconnecting.')
        self.advance_transition(time.monotonic())


def main(args=None):
    """Run the standalone pedal input node."""
    rclpy.init(args=args)
    node = None
    try:
        node = PedalInputNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
