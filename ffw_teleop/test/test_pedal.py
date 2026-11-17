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

from ffw_teleop.pedal import PedalHold
import pytest


def test_hold_toggles_once_until_released():
    pedal = PedalHold(0.0, 0.1, 1.0, 0.5)
    assert not pedal.update(0.5, 0.0)
    assert not pedal.update(0.0, 0.1)
    for stamp in (0.4, 0.7, 1.0):
        assert not pedal.update(0.0, stamp)
    assert pedal.update(0.0, 1.11)
    assert not pedal.update(0.0, 1.4)
    assert not pedal.update(0.5, 1.5)
    assert not pedal.update(0.0, 1.6)
    for stamp in (1.9, 2.2, 2.5):
        assert not pedal.update(0.0, stamp)
    assert pedal.update(0.0, 2.61)


def test_short_press_does_not_accumulate():
    pedal = PedalHold(0.0, 0.1, 1.0, 0.5)
    for stamp in (0.0, 0.4, 0.8, 1.2, 1.6):
        assert not pedal.update(0.5, stamp)
        assert not pedal.update(0.0, stamp + 0.1)
        assert not pedal.update(0.0, stamp + 0.3)


def test_startup_requires_release():
    pedal = PedalHold(0.0, 0.1, 1.0, 0.5)
    for stamp in (0.0, 0.4, 0.8, 1.2, 1.6):
        assert not pedal.update(0.0, stamp)


@pytest.mark.parametrize('bad_position', [math.nan, math.inf, -math.inf])
def test_invalid_input_cancels_hold(bad_position):
    pedal = PedalHold(0.0, 0.1, 1.0, 0.5)
    pedal.update(0.5, 0.0)
    pedal.update(0.0, 0.1)
    assert not pedal.update(bad_position, 0.4)
    for stamp in (0.6, 1.0, 1.4, 1.8):
        assert not pedal.update(0.0, stamp)


def test_feedback_gap_requires_release():
    pedal = PedalHold(0.0, 0.1, 1.0, 0.5)
    pedal.update(0.5, 0.0)
    pedal.update(0.0, 0.1)
    assert pedal.check_timeout(0.7)
    assert not pedal.check_timeout(0.8)
    for stamp in (0.9, 1.3, 1.7, 2.1):
        assert not pedal.update(0.0, stamp)
    assert not pedal.update(0.5, 2.2)
    assert not pedal.update(0.0, 2.3)
    for stamp in (2.6, 2.9, 3.2):
        assert not pedal.update(0.0, stamp)
    assert pedal.update(0.0, 3.31)


def test_gap_is_detected_even_before_watchdog_runs():
    pedal = PedalHold(0.0, 0.1, 1.0, 0.5)
    pedal.update(0.5, 0.0)
    pedal.update(0.0, 0.1)
    assert not pedal.update(0.0, 2.0)


@pytest.mark.parametrize('arguments', [
    (0.0, 0.0, 1.0, 0.5),
    (0.0, 0.1, -1.0, 0.5),
    (0.0, 0.1, 1.0, 0.0),
    (math.nan, 0.1, 1.0, 0.5),
])
def test_invalid_settings_are_rejected(arguments):
    with pytest.raises(ValueError):
        PedalHold(*arguments)
