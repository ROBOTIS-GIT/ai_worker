"""Rest detection shared by robot_io's arrival waits.

A command ends when the measured value comes to rest: inside the arrival
tolerance it arrived, outside it stopped (blocked by contact, such as fingers
closing on an object or a hand pressing on the table). Waiting for rest also
keeps a short move from counting as arrived before it has moved.
"""


from collections import deque


class StallDetector:
    """Tells when a measured value has come to rest.

    A stall needs samples covering the last window_s seconds, all within
    moved() of the newest one, from a measurement stamp that advanced during
    the window. Nothing counts as a stall before min_elapsed_s, so a command
    whose trajectory has not got going yet is not mistaken for a blocked one.
    """

    def __init__(self, window_s, min_elapsed_s, started, moved):
        self.window_s = float(window_s)
        self.min_elapsed_s = float(min_elapsed_s)
        self.started = started
        self.moved = moved
        self.samples = deque()

    def update(self, now, stamp, value):
        """Add one sample taken at monotonic time now; True when stalled"""
        self.samples.append((now, stamp, value))
        # Keep the newest sample that is at least window_s old as the window start.
        while len(self.samples) > 1 and now - self.samples[1][0] >= self.window_s:
            self.samples.popleft()
        first_time, first_stamp, _ = self.samples[0]
        if now - self.started < self.min_elapsed_s or now - first_time < self.window_s:
            return False
        if stamp <= first_stamp:
            return False  # no new measurement arrived during the window
        return not any(self.moved(old, value) for _, _, old in self.samples)
