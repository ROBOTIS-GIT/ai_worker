#!/usr/bin/env python3
#
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

"""Drive the SH5/BH5 20 DOF hands from the leader gripper using grasp/release presets."""

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from robotis_interfaces.srv import SetPreset
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml


FINGERS = ('thumb', 'index', 'middle', 'ring', 'little')
JOINTS_PER_FINGER = 4
NUM_HAND_JOINTS = len(FINGERS) * JOINTS_PER_FINGER
SIDES = ('left', 'right')


class PresetFileError(Exception):
    pass


@dataclass(frozen=True)
class HandPose:
    release: Tuple[float, ...]
    grasp: Tuple[float, ...]

    def at(self, ratio: float) -> List[float]:
        return [r + (g - r) * ratio for r, g in zip(self.release, self.grasp)]

    def blend(self, other: 'HandPose', alpha: float) -> 'HandPose':
        """Return the pose that is `alpha` of the way from self to other."""
        return HandPose(
            tuple(a + (b - a) * alpha for a, b in zip(self.release, other.release)),
            tuple(a + (b - a) * alpha for a, b in zip(self.grasp, other.grasp)),
        )


def _parse_pose(value, where: str) -> Tuple[float, ...]:
    if not isinstance(value, dict):
        raise PresetFileError(f'{where}: expected a mapping of fingers {list(FINGERS)}')
    unknown = set(value) - set(FINGERS)
    if unknown:
        raise PresetFileError(f'{where}: unknown fingers {sorted(unknown)}')
    positions = []
    for finger in FINGERS:
        joints = value.get(finger)
        if not isinstance(joints, list) or len(joints) != JOINTS_PER_FINGER:
            raise PresetFileError(
                f'{where}.{finger}: expected a list of {JOINTS_PER_FINGER} joint positions')
        for joint in joints:
            if isinstance(joint, bool) or not isinstance(joint, (int, float)) or \
                    not math.isfinite(joint):
                raise PresetFileError(f'{where}.{finger}: {joint!r} is not a finite number')
            positions.append(float(joint))
    return tuple(positions)


def _mirror(pose: Tuple[float, ...], mirrored_joints: List[int]) -> Tuple[float, ...]:
    return tuple(
        -value if (index + 1) in mirrored_joints else value
        for index, value in enumerate(pose))


def load_presets(path: str) -> Tuple[str, Dict[str, Dict[str, HandPose]], Dict[int, str]]:
    """Load named hand poses and their stable numeric SetPreset identifiers."""
    try:
        with open(path, encoding='utf-8') as preset_file:
            data = yaml.safe_load(preset_file)
    except (OSError, yaml.YAMLError) as error:
        raise PresetFileError(f'failed to read {path}: {error}') from error

    if not isinstance(data, dict):
        raise PresetFileError(f'{path}: top level must be a mapping')

    mirrored_joints = data.get('mirrored_joints', [])
    if not isinstance(mirrored_joints, list) or not all(
            type(j) is int and 1 <= j <= NUM_HAND_JOINTS for j in mirrored_joints):
        raise PresetFileError(f'mirrored_joints must be joint numbers in 1..{NUM_HAND_JOINTS}')

    raw_presets = data.get('presets')
    if not isinstance(raw_presets, dict) or not raw_presets:
        raise PresetFileError(f'{path}: `presets` must be a non-empty mapping')

    presets = {}
    preset_ids = {}
    for name, preset in raw_presets.items():
        name = str(name)
        if not isinstance(preset, dict):
            raise PresetFileError(f'presets.{name}: expected a mapping')
        preset_id = preset.get('id')
        if isinstance(preset_id, bool) or not isinstance(preset_id, int) or \
                not 1 <= preset_id <= 65535:
            raise PresetFileError(f'presets.{name}.id: expected an integer in 1..65535')
        if preset_id in preset_ids:
            raise PresetFileError(
                f'presets.{name}.id: duplicate preset ID {preset_id} '
                f'for `{preset_ids[preset_id]}`')
        release = _parse_pose(preset.get('release'), f'presets.{name}.release')
        grasp = _parse_pose(preset.get('grasp'), f'presets.{name}.grasp')
        left_release = (
            _parse_pose(preset['left_release'], f'presets.{name}.left_release')
            if 'left_release' in preset else _mirror(release, mirrored_joints))
        left_grasp = (
            _parse_pose(preset['left_grasp'], f'presets.{name}.left_grasp')
            if 'left_grasp' in preset else _mirror(grasp, mirrored_joints))
        presets[name] = {
            'left': HandPose(left_release, left_grasp),
            'right': HandPose(release, grasp),
        }
        preset_ids[preset_id] = name

    default_preset = str(data.get('default_preset', next(iter(presets))))
    if default_preset not in presets:
        raise PresetFileError(f'default_preset `{default_preset}` is not a defined preset')
    return default_preset, presets, preset_ids


class HandSide:

    def __init__(self, name: str, joint_names: List[str], gripper_joint: str,
                 gripper_release: float, gripper_grasp: float, publisher):
        self.name = name
        self.joint_names = joint_names
        self.gripper_joint = gripper_joint
        self.gripper_release = gripper_release
        self.gripper_grasp = gripper_grasp
        self.publisher = publisher
        self.preset_name = ''
        self.pose: Optional[HandPose] = None
        self.previous_pose: Optional[HandPose] = None
        self.switch_start: Optional[Time] = None
        self.gripper_position: Optional[float] = None
        self.gripper_stamp: Optional[Time] = None

    def grasp_ratio(self) -> float:
        ratio = (self.gripper_position - self.gripper_release) / \
            (self.gripper_grasp - self.gripper_release)
        return min(max(ratio, 0.0), 1.0)

    def switch_alpha(self, now: Time, duration: float) -> float:
        if self.switch_start is None or self.previous_pose is None or duration <= 0.0:
            return 1.0
        return min(max((now - self.switch_start).nanoseconds * 1e-9 / duration, 0.0), 1.0)

    def effective_pose(self, now: Time, duration: float) -> HandPose:
        alpha = self.switch_alpha(now, duration)
        if alpha >= 1.0:
            return self.pose
        return self.previous_pose.blend(self.pose, alpha)

    def set_pose(self, preset_name: str, pose: HandPose, now: Time, duration: float):
        if self.pose is not None:
            self.previous_pose = self.effective_pose(now, duration)
            self.switch_start = now
        self.preset_name = preset_name
        self.pose = pose


class HandPresetController(Node):

    def __init__(self):
        super().__init__('hand_preset_controller')

        self.declare_parameter('presets_file', '')
        self.declare_parameter('initial_preset', '')
        self.declare_parameter('enabled_arms', 'both')
        self.declare_parameter('leader_joint_states_topic', '/leader/joint_states')
        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('clamp_to_joint_limits', True)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('time_from_start', 0.0)
        self.declare_parameter('preset_switch_duration', 1.0)
        self.declare_parameter('leader_state_timeout', 0.5)

        side_defaults = {
            'left': ('gripper_l_joint1', 'finger_l_joint'),
            'right': ('gripper_r_joint1', 'finger_r_joint'),
        }
        for side, (gripper_joint, joint_prefix) in side_defaults.items():
            self.declare_parameter(f'{side}.gripper_joint', gripper_joint)
            self.declare_parameter(f'{side}.gripper_release_position', 0.0)
            self.declare_parameter(f'{side}.gripper_grasp_position', 1.0)
            self.declare_parameter(f'{side}.hand_joint_prefix', joint_prefix)
            self.declare_parameter(
                f'{side}.command_topic',
                f'/leader/joint_trajectory_command_broadcaster_{side}_hand/joint_trajectory')

        self.presets_file = self.get_parameter('presets_file').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.time_from_start = float(self.get_parameter('time_from_start').value)
        self.switch_duration = float(self.get_parameter('preset_switch_duration').value)
        self.leader_state_timeout = float(self.get_parameter('leader_state_timeout').value)
        self.clamp_to_joint_limits = bool(self.get_parameter('clamp_to_joint_limits').value)

        if not self.presets_file:
            raise ValueError('`presets_file` parameter is required')
        if self.publish_rate <= 0.0:
            raise ValueError('`publish_rate` must be positive')
        if self.time_from_start < 0.0 or self.switch_duration < 0.0 or \
                self.leader_state_timeout <= 0.0:
            raise ValueError(
                '`time_from_start` and `preset_switch_duration` must be >= 0 and '
                '`leader_state_timeout` must be > 0')

        enabled_arms = self.get_parameter('enabled_arms').value
        if enabled_arms == 'both':
            enabled_sides = list(SIDES)
        elif enabled_arms in SIDES:
            enabled_sides = [enabled_arms]
        else:
            raise ValueError('`enabled_arms` must be "left", "right" or "both"')

        self.sides: Dict[str, HandSide] = {}
        for side in enabled_sides:
            release = float(self.get_parameter(f'{side}.gripper_release_position').value)
            grasp = float(self.get_parameter(f'{side}.gripper_grasp_position').value)
            if not math.isfinite(release) or not math.isfinite(grasp) or release == grasp:
                raise ValueError(
                    f'{side}: gripper release and grasp positions must be finite and different')
            prefix = self.get_parameter(f'{side}.hand_joint_prefix').value
            self.sides[side] = HandSide(
                name=side,
                joint_names=[f'{prefix}{i}' for i in range(1, NUM_HAND_JOINTS + 1)],
                gripper_joint=self.get_parameter(f'{side}.gripper_joint').value,
                gripper_release=release,
                gripper_grasp=grasp,
                publisher=self.create_publisher(
                    JointTrajectory, self.get_parameter(f'{side}.command_topic').value, 10),
            )

        self.default_preset, self.presets, self.preset_ids = load_presets(self.presets_file)
        initial_preset = self.get_parameter('initial_preset').value or self.default_preset
        if initial_preset not in self.presets:
            raise ValueError(
                f'initial_preset `{initial_preset}` is not one of {list(self.presets)}')
        now = self.get_clock().now()
        for side in self.sides.values():
            side.set_pose(initial_preset, self.presets[initial_preset][side.name],
                          now, self.switch_duration)

        self.joint_limits: Dict[str, Tuple[float, float]] = {}
        self.warned_missing_limits = False
        if self.clamp_to_joint_limits:
            latched_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(
                String, self.get_parameter('robot_description_topic').value,
                self.robot_description_callback, latched_qos)

        self.create_subscription(
            JointState, self.get_parameter('leader_joint_states_topic').value,
            self.leader_joint_states_callback, 10)

        self.preset_transition_id = 0
        self.create_service(SetPreset, '~/set_preset', self.set_preset_callback)
        self.create_service(Trigger, '~/reload_presets', self.reload_presets_callback)

        self.create_timer(1.0 / self.publish_rate, self.publish_commands)

        self.get_logger().info(
            f'Hand preset controller ready: hands={list(self.sides)}, '
            f'preset=`{initial_preset}`, preset IDs={self.preset_ids}, file={self.presets_file}')

    def robot_description_callback(self, msg: String):
        try:
            root = ET.fromstring(msg.data)
        except ET.ParseError as error:
            self.get_logger().error(f'Failed to parse robot description: {error}')
            return
        hand_joints = {name for side in self.sides.values() for name in side.joint_names}
        limits = {}
        for joint in root.iter('joint'):
            name = joint.get('name')
            limit = joint.find('limit')
            if name in hand_joints and limit is not None and \
                    joint.get('type') in ('revolute', 'prismatic'):
                limits[name] = (float(limit.get('lower', '-inf')),
                                float(limit.get('upper', 'inf')))
        self.joint_limits = limits
        missing = sorted(hand_joints - set(limits))
        if missing:
            self.get_logger().warning(
                f'No joint limits in robot description for {len(missing)} hand joints; '
                f'those joints will not be clamped (e.g. {missing[0]})')
        else:
            self.get_logger().info('Hand joint limits loaded from robot description')

    def leader_joint_states_callback(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            return
        now = self.get_clock().now()
        positions = dict(zip(msg.name, msg.position))
        for side in self.sides.values():
            position = positions.get(side.gripper_joint)
            if position is not None and math.isfinite(position):
                side.gripper_position = position
                side.gripper_stamp = now

    def publish_commands(self):
        now = self.get_clock().now()
        if self.clamp_to_joint_limits and not self.joint_limits and \
                not self.warned_missing_limits:
            self.warned_missing_limits = True
            self.get_logger().warning(
                'Follower robot description not received yet; '
                'hand commands are not clamped to joint limits')

        for side in self.sides.values():
            if side.gripper_stamp is None or \
                    (now - side.gripper_stamp).nanoseconds * 1e-9 > self.leader_state_timeout:
                self.get_logger().warning(
                    f'{side.name} hand paused: no recent `{side.gripper_joint}` state '
                    'from the leader', throttle_duration_sec=2.0)
                continue

            positions = side.effective_pose(now, self.switch_duration).at(side.grasp_ratio())
            for index, joint_name in enumerate(side.joint_names):
                limit = self.joint_limits.get(joint_name)
                if limit is not None:
                    positions[index] = min(max(positions[index], limit[0]), limit[1])

            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(seconds=self.time_from_start).to_msg()
            trajectory = JointTrajectory()
            trajectory.joint_names = side.joint_names
            trajectory.points = [point]
            side.publisher.publish(trajectory)

    def set_preset_callback(self, request, response):
        target_arm = request.target_arm or SetPreset.Request.TARGET_BOTH
        if target_arm == SetPreset.Request.TARGET_BOTH:
            targets = list(self.sides.values())
        elif target_arm in self.sides:
            targets = [self.sides[target_arm]]
        else:
            response.accepted = False
            response.message = (
                f'Invalid target_arm `{request.target_arm}`; enabled hands: {list(self.sides)}')
            return response

        selections = {
            'left': request.left_preset_id,
            'right': request.right_preset_id,
        }
        missing = [
            f'{side.name}={selections[side.name]}' for side in targets
            if selections[side.name] not in self.preset_ids
        ]
        if missing:
            response.accepted = False
            response.message = (
                f'Unknown hand preset ID(s): {", ".join(missing)}; '
                f'available: {sorted(self.preset_ids)}')
            return response
        now = self.get_clock().now()
        for side in targets:
            preset_name = self.preset_ids[selections[side.name]]
            side.set_pose(preset_name, self.presets[preset_name][side.name],
                          now, self.switch_duration)
        self.preset_transition_id += 1
        response.accepted = True
        response.transition_id = self.preset_transition_id
        response.message = (
            'Hand preset IDs selected: ' +
            ', '.join(f'{side.name}={selections[side.name]}' for side in targets))
        self.get_logger().info(response.message)
        return response

    def reload_presets_callback(self, _request, response):
        try:
            default_preset, presets, preset_ids = load_presets(self.presets_file)
        except PresetFileError as error:
            response.success = False
            response.message = f'Reload failed, keeping previous presets: {error}'
            self.get_logger().error(response.message)
            return response

        self.default_preset = default_preset
        self.presets = presets
        self.preset_ids = preset_ids
        now = self.get_clock().now()
        for side in self.sides.values():
            preset_name = side.preset_name
            if preset_name not in presets:
                self.get_logger().warning(
                    f'{side.name} preset `{preset_name}` was removed; '
                    f'switching to `{default_preset}`')
                preset_name = default_preset
            side.set_pose(preset_name, presets[preset_name][side.name],
                          now, self.switch_duration)
        response.success = True
        response.message = f'Reloaded {len(presets)} presets: {list(presets)}'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HandPresetController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
