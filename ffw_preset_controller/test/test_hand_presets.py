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


"""Exercise YAML loading and hand interpolation without a ROS graph."""

import importlib.util
from pathlib import Path
import sys

import pytest
import yaml


MODULE_PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'hand_preset_controller.py'
SPEC = importlib.util.spec_from_file_location('hand_preset_controller', MODULE_PATH)
controller = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = controller
SPEC.loader.exec_module(controller)


def preset_data():
    """Return a minimal configurable hand preset."""
    return {
        'default_preset': 'test',
        'mirrored_joints': [1, 5],
        'presets': {'test': {
            'release': {finger: [0.0] * 4 for finger in controller.FINGERS},
            'grasp': {finger: [1.0] * 4 for finger in controller.FINGERS},
        }},
    }


def load(tmp_path, data):
    """Load a user-authored YAML file."""
    path = tmp_path / 'presets.yaml'
    path.write_text(yaml.safe_dump(data))
    return controller.load_presets(str(path))


def test_interpolation_and_mirroring(tmp_path):
    """Preserve finger ordering and mirror only configured left-hand joints."""
    default, presets = load(tmp_path, preset_data())
    assert default == 'test'
    assert presets['test']['right'].at(0.5) == [0.5] * 20
    left = presets['test']['left'].at(0.5)
    assert left == [-0.5 if i in (0, 4) else 0.5 for i in range(20)]


def test_explicit_left_pose_overrides_mirroring(tmp_path):
    """Allow independent left-hand values."""
    data = preset_data()
    data['presets']['test']['left_grasp'] = {
        finger: [2.0] * 4 for finger in controller.FINGERS}
    _, presets = load(tmp_path, data)
    assert presets['test']['left'].grasp == (2.0,) * 20


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), True, 'angle'])
def test_invalid_joint_value(tmp_path, bad):
    """Reject values that cannot produce a finite joint command."""
    data = preset_data()
    data['presets']['test']['grasp']['thumb'][0] = bad
    with pytest.raises(controller.PresetFileError):
        load(tmp_path, data)


def test_invalid_default_and_finger_length(tmp_path):
    """Reject missing preset selections and incomplete fingers."""
    data = preset_data()
    data['default_preset'] = 'missing'
    with pytest.raises(controller.PresetFileError):
        load(tmp_path, data)
    data = preset_data()
    data['presets']['test']['grasp']['thumb'] = [0.0]
    with pytest.raises(controller.PresetFileError):
        load(tmp_path, data)


@pytest.mark.parametrize('release,grasp', [(-0.3, 0.8), (-0.4, 0.7)])
def test_leader_calibration_and_clamping(release, grasp):
    """LG2 and A2 grippers span the full preset and clamp outside that range."""
    side = controller.HandSide('left', [], 'gripper_l_joint1', release, grasp, None)
    for position, expected in [(release - 1, 0), (release, 0),
                               ((release + grasp) / 2, 0.5), (grasp, 1), (grasp + 1, 1)]:
        side.gripper_position = position
        assert side.grasp_ratio() == pytest.approx(expected)


def test_preset_switch_continuity():
    """A second switch starts from the currently blended pose."""
    side = controller.HandSide('left', [], 'gripper_l_joint1', 0, 1, None)
    first = controller.HandPose((0.0,) * 20, (1.0,) * 20)
    second = controller.HandPose((2.0,) * 20, (3.0,) * 20)
    side.set_pose('first', first, controller.Time(seconds=0), 1.0)
    side.set_pose('second', second, controller.Time(seconds=1), 1.0)
    halfway = side.effective_pose(controller.Time(seconds=1.5), 1.0)
    side.set_pose('first', first, controller.Time(seconds=1.5), 1.0)
    assert side.effective_pose(controller.Time(seconds=1.5), 1.0) == halfway
