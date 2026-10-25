"""Bounded position closure policy; no ROS, arm commands or grasp confirmation."""

import math

from .configuration import UPPER_LIMITS

DIGITS = ('thumb', 'index', 'middle', 'ring', 'pinky', 'thumb')


def digit_for_link(link):
    if not link.startswith('right_'):
        return None
    return next((digit for digit in set(DIGITS) if digit in link), None)


def contact_digits(links, forces, threshold):
    if len(links) != len(forces) or len(set(links)) != len(links):
        raise ValueError('Contact links and forces must match and have unique links')
    if any(not math.isfinite(f) or f < 0 for f in forces):
        raise ValueError('Contact forces must be finite and nonnegative')
    return {digit_for_link(link) for link, force in zip(links, forces)
            if force >= threshold and digit_for_link(link)}


def next_positions(measured, target, frozen, step):
    """Each free joint advances at most step from its measured position."""
    if len(measured) != 6 or len(target) != 6 or step <= 0:
        raise ValueError('Six measured/target joints and a positive step are required')
    result = []
    for i, (actual, desired) in enumerate(zip(measured, target)):
        if (not math.isfinite(actual) or not math.isfinite(desired)
                or not -.01 <= actual <= UPPER_LIMITS[i]+.01):
            raise ValueError('Measured closure state is outside model tolerance')
        if i in frozen:
            command = frozen[i]
        else:
            command = actual + min(step, max(0., desired-actual))
        if not math.isfinite(command) or not -.01 <= command <= UPPER_LIMITS[i]+.01:
            raise ValueError('Closure command exceeds hand limits')
        # Physics permits tiny joint-limit penetration. Never propagate it into
        # a command when an otherwise stationary/open digit is held.
        result.append(max(0., min(UPPER_LIMITS[i], command)))
    return result


def freeze_contacts(frozen, digits, measured, target, preload):
    """A one-time bounded preload; repeated feedback cannot ratchet the hold."""
    for i, digit in enumerate(DIGITS):
        if digit in digits and i not in frozen:
            frozen[i] = min(target[i], UPPER_LIMITS[i], measured[i]+preload)


def reached(measured, target, frozen, tolerance):
    return all(i in frozen or abs(a-b) <= tolerance for i, (a, b) in enumerate(zip(measured, target)))
