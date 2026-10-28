# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Verify captured joint ordering, mirroring, IDs and valid atomic preset updates."""

import importlib
from pathlib import Path
import sys

import pytest
from sensor_msgs.msg import JointState
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
recorder = importlib.import_module('record_hand_preset')


@pytest.fixture
def preset_file(tmp_path):
    """Use the actual bringup preset format, including its header comments."""
    source = Path(__file__).resolve().parents[2] / 'ffw_bringup/config/common/hand_presets.yaml'
    target = tmp_path / 'hand_presets.yaml'
    target.write_bytes(source.read_bytes())
    return target


@pytest.mark.parametrize('side,letter', [('left', 'l'), ('right', 'r')])
def test_detect_hand_and_reorder_joints(side, letter):
    """Reorder incoming states into finger/joint order in the YAML."""
    names = [f'finger_{letter}_joint{i}' for i in range(20, 0, -1)]
    message = JointState(name=['other_joint'] + names,
                         position=[9.0] + list(map(float, range(20, 0, -1))))
    assert recorder.extract_pose(message) == (side, list(map(float, range(1, 21))))


def test_prefixed_joint_names():
    """Support the prefix argument accepted by robotis_hand_description."""
    message = JointState(name=[f'hand_finger_l_joint{i}' for i in range(1, 21)],
                         position=[.2] * 20)
    assert recorder.extract_pose(message, prefix='hand_') == ('left', [.2] * 20)


def test_both_hands_require_selection():
    """Do not silently pick one side when a follower publishes both hands."""
    message = JointState(name=[f'finger_{side}_joint{i}'
                               for side in ('l', 'r') for i in range(1, 21)],
                         position=[.1] * 20 + [.2] * 20)
    with pytest.raises(ValueError, match='Both hands'):
        recorder.extract_pose(message)
    assert recorder.extract_pose(message, side='right') == ('right', [.2] * 20)


@pytest.mark.parametrize('message', [
    JointState(name=['finger_r_joint1'], position=[0.0]),
    JointState(name=['finger_r_joint1'], position=[]),
    JointState(name=[f'finger_r_joint{i}' for i in range(1, 21)],
               position=[float('nan')] + [0.0] * 19),
    JointState(name=[f'finger_r_joint{i}' for i in range(1, 21)] + ['finger_r_joint1'],
               position=[0.0] * 21)])
def test_reject_incomplete_nonfinite_or_duplicate_samples(message):
    """Only a complete unambiguous finite sample may be recorded."""
    with pytest.raises(ValueError):
        recorder.extract_pose(message)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('pose', ['release', 'grasp'])
def test_capture_uses_shared_mirroring_and_preserves_other_poses(preset_file, side, pose):
    """Either hand can author a pose; the loader must reproduce the captured side."""
    original = preset_file.read_bytes()
    before = yaml.safe_load(original)
    values = [i * .01 for i in range(20)]
    created, _, removed = recorder.save_pose(
        preset_file, 'custom', 4, pose, side, values)
    assert not preset_file.with_name(preset_file.name + '.bak').exists()
    assert created and not removed
    after = yaml.safe_load(preset_file.read_text())
    assert all(after['presets'][name] == data for name, data in before['presets'].items())
    other = 'release' if pose == 'grasp' else 'grasp'
    assert after['presets']['custom'][other] == before['presets']['envelope'][other]
    _, presets, ids = recorder.load_presets(str(preset_file))
    assert ids[4] == 'custom'
    assert getattr(presets['custom'][side], pose) == pytest.approx(values)
    assert preset_file.read_text().startswith('# Hand presets')


def test_update_existing_preset_and_keep_id(preset_file):
    """Recording the second half updates the same preset and keeps the first half."""
    recorder.save_pose(preset_file, 'custom', 4, 'release', 'right', [.1] * 20, base='pinch')
    created, _, _ = recorder.save_pose(preset_file, 'custom', 4, 'grasp', 'left', [.2] * 20)
    assert not created
    _, presets, _ = recorder.load_presets(str(preset_file))
    assert presets['custom']['right'].release == (.1,) * 20
    assert presets['custom']['left'].grasp == (.2,) * 20


@pytest.mark.parametrize('name,number,base', [
    ('custom', 1, None), ('envelope', 4, None),
    ('custom', 4, 'missing'), ('envelope', 1, 'pinch')])
def test_invalid_selection_keeps_file_unchanged(preset_file, name, number, base):
    """Reject conflicting IDs or bad templates without changing the source."""
    original = preset_file.read_bytes()
    with pytest.raises(ValueError):
        recorder.save_pose(preset_file, name, number, 'grasp', 'right', [.2] * 20, base)
    assert preset_file.read_bytes() == original
    assert not preset_file.with_name(preset_file.name + '.bak').exists()


def test_shared_capture_replaces_corresponding_left_override(preset_file):
    """A pre-existing left override must not hide the newly recorded shared pose."""
    data = yaml.safe_load(preset_file.read_text())
    data['presets']['pinch']['left_grasp'] = data['presets']['pinch']['grasp']
    preset_file.write_text(yaml.safe_dump(data))
    _, _, removed = recorder.save_pose(preset_file, 'pinch', 2, 'grasp', 'left', [.3] * 20)
    assert removed
    assert 'left_grasp' not in yaml.safe_load(preset_file.read_text())['presets']['pinch']
    _, presets, _ = recorder.load_presets(str(preset_file))
    assert presets['pinch']['left'].grasp == (.3,) * 20
