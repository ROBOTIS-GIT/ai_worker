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
#
# Author: Jiho Park

from collections import deque


class StallDetector:
    """Detect rest from fresh measurements over a time window"""

    def __init__(self, window_s, min_elapsed_s, started, moved):
        self.window_s = float(window_s)
        self.min_elapsed_s = float(min_elapsed_s)
        self.started = started
        self.moved = moved
        self.samples = deque()

    def update(self, now, stamp, value):
        self.samples.append((now, stamp, value))

        while len(self.samples) > 1 and now - self.samples[1][0] >= self.window_s:
            self.samples.popleft()
        first_time, first_stamp, _ = self.samples[0]

        if now - self.started < self.min_elapsed_s or now - first_time < self.window_s:
            return False

        if stamp <= first_stamp:
            return False

        return not any(self.moved(old, value) for _, _, old in self.samples)
