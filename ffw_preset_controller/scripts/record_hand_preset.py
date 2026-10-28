#!/usr/bin/env python3
# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Record one complete JointState hand pose into the shared preset YAML."""

import argparse
import copy
import math
import os
from pathlib import Path
import stat
import sys
import tempfile
import time

from hand_preset_controller import FINGERS, load_presets, NUM_HAND_JOINTS, PresetFileError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import yaml


def extract_pose(message, side='auto', prefix=''):
    """Find exactly one complete hand, independent of JointState array ordering."""
    if len(message.name) != len(message.position):
        raise ValueError('JointState name and position arrays have different lengths')
    positions = dict(zip(message.name, message.position))
    candidates = {}
    for name, letter in [('left', 'l'), ('right', 'r')]:
        if side not in ('auto', name):
            continue
        joints = [f'{prefix}finger_{letter}_joint{i}'
                  for i in range(1, NUM_HAND_JOINTS + 1)]
        if all(joint in positions for joint in joints):
            if any(message.name.count(joint) != 1 for joint in joints):
                raise ValueError(f'Duplicate joint names in the {name} hand sample')
            values = [float(positions[joint]) for joint in joints]
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f'Non-finite joint position in the {name} hand sample')
            candidates[name] = values
    if not candidates:
        raise ValueError('No complete matching hand: expected finger_l_joint1..20 '
                         'or finger_r_joint1..20 (with the configured prefix)')
    if len(candidates) > 1:
        raise ValueError('Both hands are present; choose --side left or --side right')
    return next(iter(candidates.items()))


def update_document(document, name, preset_id, pose, side, positions, base=None):
    """Update a canonical shared pose, undoing mirroring for left-hand captures."""
    result = copy.deepcopy(document)
    presets = result['presets']
    for existing_name, preset in presets.items():
        if existing_name != name and preset['id'] == preset_id:
            raise ValueError(f'Preset ID {preset_id} is already used by `{existing_name}`')
    created = name not in presets
    if created:
        base = base or result['default_preset']
        if base not in presets:
            raise ValueError(f'Template preset `{base}` does not exist')
        presets[name] = copy.deepcopy(presets[base])
        presets[name]['id'] = preset_id
    elif presets[name]['id'] != preset_id:
        raise ValueError(f'Preset `{name}` already has ID {presets[name]["id"]}; '
                         'use that ID to update its pose')
    elif base is not None:
        raise ValueError('--base is only used when creating a new preset')
    if len(positions) != NUM_HAND_JOINTS or not all(math.isfinite(v) for v in positions):
        raise ValueError('A pose must contain 20 finite joint positions')
    values = list(positions)
    if side == 'left':
        for joint in result.get('mirrored_joints', []):
            values[joint - 1] *= -1.0
    presets[name][pose] = {
        finger: values[index * 4:(index + 1) * 4]
        for index, finger in enumerate(FINGERS)}
    # This capture authors a shared pose; a per-left override must not hide it.
    removed_override = presets[name].pop(f'left_{pose}', None) is not None
    return result, created, base, removed_override


class PresetDumper(yaml.SafeDumper):
    """Keep each finger's four positions on one line."""


def represent_list(dumper, values):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', values, flow_style=True)


PresetDumper.add_representer(list, represent_list)


def save_pose(path, name, preset_id, pose, side, positions, base=None):
    """Validate the complete result and replace the file atomically."""
    path = Path(path).expanduser().resolve(strict=True)
    original = path.read_text(encoding='utf-8')
    load_presets(str(path))
    document = yaml.safe_load(original)
    # The loader also supports an omitted default_preset.
    document.setdefault('default_preset', next(iter(document['presets'])))
    result, created, template, removed = update_document(
        document, name, preset_id, pose, side, positions, base)
    header = []
    for line in original.splitlines(keepends=True):
        if line.strip() and not line.lstrip().startswith('#'):
            break
        header.append(line)
    text = ''.join(header) + yaml.dump(
        result, Dumper=PresetDumper, sort_keys=False, allow_unicode=True, width=100)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix=f'.{path.name}.', delete=False) as output:
            temporary = Path(output.name)
            output.write(text)
            output.flush()
            os.fsync(output.fileno())
        load_presets(str(temporary))
        if path.read_text(encoding='utf-8') != original:
            raise ValueError('Preset file changed while recording; run the capture again')
        temporary.chmod(stat.S_IMODE(path.stat().st_mode))
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return created, template, removed


def preset_id(value):
    number = int(value)
    if not 1 <= number <= 65535:
        raise argparse.ArgumentTypeError('ID must be in 1..65535')
    return number


def positive_timeout(value):
    seconds = float(value)
    if not math.isfinite(seconds) or seconds <= 0:
        raise argparse.ArgumentTypeError('timeout must be a positive finite number')
    return seconds


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', required=True, type=Path,
                        help='Source hand_presets.yaml to edit')
    parser.add_argument('--name', required=True, help='Preset name to create or update')
    parser.add_argument('--id', required=True, type=preset_id, help='Stable preset ID (1..65535)')
    parser.add_argument('--pose', required=True, choices=['grasp', 'release'])
    parser.add_argument('--side', default='auto', choices=['auto', 'left', 'right'],
                        help='Detect from joint names, or choose when both hands are present')
    parser.add_argument('--prefix', default='', help='Joint name prefix passed to the hand launch')
    parser.add_argument('--topic', default='/joint_states')
    parser.add_argument('--timeout', default=10.0, type=positive_timeout)
    parser.add_argument('--base', help='New preset template; defaults to default_preset')
    options = parser.parse_args(args)
    if not options.name.strip():
        parser.error('--name must not be empty')

    node = None
    initialized = False
    try:
        # Reject missing/invalid files before subscribing.
        load_presets(str(options.file.expanduser()))
        rclpy.init(args=[])
        initialized = True
        node = Node('record_hand_preset')
        sample = []
        last_error = 'No JointState messages received'

        def receive(message):
            nonlocal last_error
            try:
                sample[:] = [extract_pose(message, options.side, options.prefix)]
            except ValueError as error:
                last_error = str(error)

        node.create_subscription(JointState, options.topic, receive, qos_profile_sensor_data)
        node.get_logger().info(f'Waiting for a complete hand pose on {options.topic}...')
        deadline = time.monotonic() + options.timeout
        while not sample and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.1, max(0, deadline - time.monotonic())))
        if not sample:
            raise ValueError(f'Capture timed out: {last_error}')
        side, positions = sample[0]
        created, template, removed = save_pose(
            options.file, options.name, options.id, options.pose, side, positions, options.base)
        if created:
            print(f'Created `{options.name}` (ID {options.id}) from `{template}`; '
                  'the other pose retains the template values.')
        print(f'Saved {side} {options.pose} as a shared right-convention pose '
              f'in {options.file.resolve()}.')
        if removed:
            print(f'Removed left_{options.pose} override so the recorded pose is mirrored.')
        return 0
    except (OSError, ValueError, PresetFileError, yaml.YAMLError) as error:
        print(f'Recording failed: {error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
