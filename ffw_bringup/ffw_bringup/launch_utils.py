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

"""Small helpers for follower command routing and successful startup sequencing."""

from launch.actions import LogInfo


def command_topic(group, legacy_topic):
    """Keep follower input topics compatible with existing publishers and datasets."""
    return legacy_topic


def controller_remaps():
    result = []
    for controller, group, legacy in (
            ('arm_l', 'left',
             '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory'),
            ('arm_r', 'right',
             '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory'),
            ('head', 'head', '/leader/joystick_controller_left/joint_trajectory'),
            ('lift', 'lift', '/leader/joystick_controller_right/joint_trajectory')):
        result.extend(['--controller-ros-args', [
            f'-r /{controller}_controller/joint_trajectory:=', command_topic(group, legacy)]])
    return result


def start_after_success(required_processes, action):
    """Return an OnProcessExit callback that starts action once all prerequisites succeed."""
    pending = set(required_processes)
    if not pending:
        raise ValueError('At least one prerequisite process is required')
    failed = False
    started = False

    def on_exit(event, _context):
        nonlocal failed, started
        if started or failed or event.action not in pending:
            return []
        pending.remove(event.action)
        if event.returncode != 0:
            failed = True
            return [LogInfo(msg='Initialization failed; dependent action will not be started.')]
        if pending:
            return []
        started = True
        return [action]

    return on_exit
