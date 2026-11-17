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


class PedalHold:
    """Detect one long press from fresh pedal samples, after observing a release."""

    def __init__(self, center, window, duration, timeout):
        if not all(math.isfinite(value) for value in (center, window, duration, timeout)):
            raise ValueError('Pedal settings must be finite')
        if min(window, duration, timeout) <= 0.0:
            raise ValueError('Pedal window, duration and timeout must be positive')
        self.center = center
        self.window = window
        self.duration = duration
        self.timeout = timeout
        self.last_sample = None
        self.reset()

    def reset(self):
        """Require a release before accepting another press."""
        self.armed = False
        self.pressed_since = None

    def check_timeout(self, now):
        """Cancel a held press if feedback stops, reporting the loss only once."""
        if self.last_sample is None or now - self.last_sample <= self.timeout:
            return False
        self.last_sample = None
        self.reset()
        return True

    def update(self, position, now):
        """Return true once when a continuous, valid press reaches its duration."""
        self.check_timeout(now)
        if not math.isfinite(position):
            self.reset()
            return False
        self.last_sample = now
        if abs(position - self.center) >= self.window:
            self.armed = True
            self.pressed_since = None
        elif self.armed:
            if self.pressed_since is None:
                self.pressed_since = now
            elif now - self.pressed_since >= self.duration:
                self.armed = False
                return True
        return False
