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

import time

from ffw_teleop.pedal_input_node import PedalInputNode
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from robotis_interfaces.srv import SetTeleoperation
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool


class ControlPeer:
    """Provide actual ROS services with controllable preparation and rejection."""

    def __init__(self):
        self.node = rclpy.create_node('pedal_control_peer')
        self.source = 'model_action'
        self.send_source = True
        self.preparing = False
        self.accept_source = True
        self.accept_enabled = True
        self.enable_rejections = 0
        self.requests = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.source_pub = self.node.create_publisher(String, '/source', qos)
        self.pedal_pub = self.node.create_publisher(JointState, 'joint_states', 1)
        self.source_service = self.node.create_service(SetBool, '/set_source', self.set_source)
        self.enabled_service = self.node.create_service(
            SetTeleoperation, '/set_enabled', self.set_enabled)

    def set_source(self, request, response):
        self.requests.append(('source', request.data))
        response.success = self.accept_source
        response.message = 'accepted' if self.accept_source else 'test rejection'
        if response.success:
            self.source = 'switching' if self.preparing else (
                'teleop' if request.data else 'model_action')
        return response

    def set_enabled(self, request, response):
        assert request.target_arm == 'both'
        self.requests.append(('enabled', request.enabled))
        response.accepted = self.accept_enabled and self.enable_rejections == 0
        response.message = 'accepted' if response.accepted else 'not ready'
        self.enable_rejections = max(0, self.enable_rejections - 1)
        return response


@pytest.fixture
def control():
    rclpy.init(args=['--ros-args', '-r', 'joint_states:=/pedal_test/joint_states',
                     '-r', '/source:=/pedal_test/source',
                     '-r', '/set_source:=/pedal_test/set_source',
                     '-r', '/set_enabled:=/pedal_test/set_enabled',
                     '-r', 'position_controller/commands:=/pedal_test/commands',
                     '-p', 'long_press_seconds:=0.03',
                     '-p', 'request_timeout_seconds:=0.25',
                     '-p', 'transition_timeout_seconds:=1.5'])
    node = PedalInputNode()
    peer = ControlPeer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer.node)

    def pump(seconds, position=0.5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if peer.send_source:
                peer.source_pub.publish(String(data=peer.source))
            peer.pedal_pub.publish(JointState(name=['pedal_joint'], position=[position]))
            executor.spin_once(timeout_sec=0.005)

    pump(0.3)
    try:
        yield node, peer, executor, pump
    finally:
        executor.shutdown()
        node.destroy_node()
        peer.node.destroy_node()
        rclpy.shutdown()


def test_start_and_stop_use_existing_services_in_order(control):
    node, peer, executor, pump = control
    assert peer.requests == []
    pump(0.5, 0.0)
    assert peer.requests == [('source', True), ('enabled', True)]
    pump(0.1)
    pump(0.5, 0.0)
    assert peer.requests == [('source', True), ('enabled', True),
                             ('enabled', False), ('source', False)]
    assert peer.source == 'model_action'


def test_accepted_start_waits_for_preparation(control):
    node, peer, executor, pump = control
    peer.preparing = True
    pump(0.6, 0.0)
    assert peer.requests == [('source', True)]
    peer.source = 'teleop'
    pump(0.3, 0.0)
    assert peer.requests == [('source', True), ('enabled', True)]


def test_source_rejection_does_not_enable_and_can_retry(control):
    node, peer, executor, pump = control
    peer.accept_source = False
    pump(0.4, 0.0)
    assert peer.requests == [('source', True)]
    peer.accept_source = True
    pump(0.1)
    pump(0.5, 0.0)
    assert peer.requests == [('source', True), ('source', True), ('enabled', True)]


def test_enable_retries_until_leader_has_observed_source(control):
    node, peer, executor, pump = control
    peer.enable_rejections = 2
    pump(0.8, 0.0)
    assert peer.requests == [('source', True), ('enabled', True),
                             ('enabled', True), ('enabled', True)]


def test_enable_rejection_has_a_bounded_retry_window(control):
    node, peer, executor, pump = control
    peer.accept_enabled = False
    pump(1.8, 0.0)
    count = len(peer.requests)
    assert count > 1
    pump(0.4, 0.0)
    assert len(peer.requests) == count


def test_interrupted_preparation_does_not_replay_enable(control):
    node, peer, executor, pump = control
    peer.preparing = True
    pump(0.4, 0.0)
    assert peer.requests == [('source', True)]
    peer.source = 'model_action'
    pump(0.2, 0.0)
    peer.source = 'teleop'
    pump(0.4, 0.0)
    assert peer.requests == [('source', True)]


def test_source_heartbeat_loss_cancels_pending_enable(control):
    node, peer, executor, pump = control
    peer.preparing = True
    pump(0.4, 0.0)
    peer.send_source = False
    pump(0.7, 0.0)
    peer.source = 'teleop'
    peer.send_source = True
    pump(0.4, 0.0)
    assert peer.requests == [('source', True)]


def test_second_hold_during_preparation_cancels_start_and_releases_source(control):
    node, peer, executor, pump = control
    peer.preparing = True
    pump(0.4, 0.0)
    pump(0.1)
    pump(0.3, 0.0)
    assert peer.requests == [('source', True)]
    peer.preparing = False
    peer.source = 'teleop'
    pump(0.4, 0.0)
    assert peer.requests == [('source', True), ('enabled', False), ('source', False)]
    peer.source = 'teleop'
    pump(0.3, 0.0)
    assert ('enabled', True) not in peer.requests


def test_stop_releases_source_even_if_disable_is_rejected(control):
    node, peer, executor, pump = control
    peer.source = 'teleop'
    peer.accept_enabled = False
    pump(0.2)
    pump(0.6, 0.0)
    assert peer.requests == [('enabled', False), ('source', False)]


def test_missing_source_service_does_not_enable(control):
    node, peer, executor, pump = control
    peer.node.destroy_service(peer.source_service)
    pump(0.6)
    pump(0.5, 0.0)
    assert peer.requests == []


def test_late_service_response_cannot_replay_enable(control):
    node, peer, executor, pump = control
    executor.remove_node(peer.node)
    pump(0.6, 0.0)
    executor.add_node(peer.node)
    pump(0.4, 0.0)
    assert ('enabled', True) not in peer.requests
