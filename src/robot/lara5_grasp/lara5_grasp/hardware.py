"""Named physical force channels; values stay in the driver's raw units."""

import math

from .configuration import HAND_JOINTS
from .policy import DIGITS


def physical_contacts(message, baseline, thresholds, limits):
    if (len(message.name) != 6 or set(message.name) != set(HAND_JOINTS)
            or len(message.effort) != 6 or len(message.position) != 6):
        raise ValueError('Hardware feedback requires six unique named force/angle channels')
    if any(not math.isfinite(v) for v in (*message.effort, *message.position)):
        raise ValueError('Nonfinite physical hand measurement')
    measured = dict(zip(message.name, message.effort))
    forces = [abs(measured[name]-zero) for name, zero in zip(HAND_JOINTS, baseline)]
    if any(force >= limit for force, limit in zip(forces, limits)):
        raise ValueError('Physical actuator-force limit exceeded (raw driver units)')
    return forces, {digit for digit, force, threshold in zip(DIGITS, forces, thresholds)
                    if force >= threshold}


def validate_force_limits(baseline, thresholds, limits):
    if any(len(values) != 6 for values in (baseline, thresholds, limits)):
        raise ValueError('Physical force settings require six channels in HAND_JOINTS order')
    if any(not math.isfinite(v) for values in (baseline, thresholds, limits) for v in values):
        raise ValueError('Physical force settings must be finite')
    if any(not 0 < low < high for low, high in zip(thresholds, limits)):
        raise ValueError('Supply validated raw force thresholds: 0 < contact < maximum')


# Source times within this interval are one physical read. The driver's cached
# republishes must not count as new evidence, even with a recomputed stamp.
REPEAT_TOLERANCE = 1e-3


def source_time(message):
    return message.header.stamp.sec+message.header.stamp.nanosec/1e9


class ForceEvidence:
    """Sustained bilateral force from NEW physical measurements only.

    Re-checking one still-fresh cached sample never extends the interval.
    """

    def __init__(self, samples, duration, max_gap):
        if int(samples) != samples or samples < 3 or not duration > 0 or not max_gap > 0:
            raise ValueError('Stable force needs at least three measurements over a positive interval')
        self.samples, self.duration, self.max_gap = int(samples), duration, max_gap
        self.last = self.first = None
        self.count = 0

    def add(self, stamp, bilateral):
        """Return False for a repeat of the latest measurement."""
        if self.last is not None:
            if stamp < self.last-REPEAT_TOLERANCE:
                raise ValueError('Physical hand measurement source time moved backwards')
            if stamp <= self.last+REPEAT_TOLERANCE:
                return False
            if stamp-self.last > self.max_gap:
                self.count = 0
        self.last = stamp
        if not bilateral:
            self.count = 0
        elif self.count == 0:
            self.first, self.count = stamp, 1
        else:
            self.count += 1
        return True

    def stable(self):
        return self.count >= self.samples and self.last-self.first >= self.duration

    def report(self):
        return dict(bilateral_new_measurements=self.count,
                    source_span_seconds=self.last-self.first if self.count else 0.,
                    required_measurements=self.samples, required_span_seconds=self.duration)
