"""Physical force mapping and fail-closed hardware configuration."""

from types import SimpleNamespace as NS

import pytest

from lara5_grasp.configuration import HAND_JOINTS
from lara5_grasp.hardware import ForceEvidence, physical_contacts, validate_force_limits


def test_force_channels_follow_joint_names_not_modbus_register_order():
    msg = NS(name=list(reversed(HAND_JOINTS)), position=[0.]*6,
             effort=list(reversed([0., 12., 0., 0., 0., 15.])))
    forces, digits = physical_contacts(msg, [0.]*6, [10.]*6, [30.]*6)
    assert forces == [0., 12., 0., 0., 0., 15.]
    assert digits == {'index', 'thumb'}


@pytest.mark.parametrize('fault', ['limit', 'nan', 'duplicate'])
def test_invalid_or_excessive_physical_force_is_rejected(fault):
    msg = NS(name=list(HAND_JOINTS), position=[0.]*6, effort=[0.]*6)
    if fault == 'limit': msg.effort[2] = 30.
    if fault == 'nan': msg.effort[2] = float('nan')
    if fault == 'duplicate': msg.name[1] = msg.name[0]
    with pytest.raises(ValueError):
        physical_contacts(msg, [0.]*6, [10.]*6, [30.]*6)


def test_unknown_force_thresholds_are_not_accepted_as_zero_limits():
    with pytest.raises(ValueError):
        validate_force_limits([0.]*6, [0.]*6, [0.]*6)
    validate_force_limits([0.]*6, [10.]*6, [30.]*6)


def test_force_stability_counts_only_new_physical_measurements():
    evidence = ForceEvidence(3, .02, .5)
    assert evidence.add(1.000, True)
    # Re-checking one cached sample, or a republish with a jittered stamp, adds nothing.
    for stamp in (1.000, 1.0000003, .9999997):
        assert not evidence.add(stamp, True)
    assert not evidence.stable() and evidence.count == 1
    assert evidence.add(1.010, True) and evidence.add(1.020, True)
    assert evidence.stable() and evidence.report()['bilateral_new_measurements'] == 3


def test_force_stability_needs_span_and_uninterrupted_new_bilateral_force():
    evidence = ForceEvidence(3, .05, .1)
    for stamp in (1.00, 1.01, 1.02, 1.03):
        evidence.add(stamp, True)
    assert evidence.count == 4 and not evidence.stable()
    evidence.add(1.04, False)
    assert evidence.count == 0
    for stamp in (1.05, 1.08, 1.11):
        evidence.add(stamp, True)
    assert evidence.stable()
    # A measurement gap restarts the interval.
    evidence.add(1.30, True)
    assert evidence.count == 1 and not evidence.stable()


def test_backwards_source_time_and_weak_stability_rules_are_rejected():
    evidence = ForceEvidence(3, .05, .1)
    evidence.add(1., True)
    with pytest.raises(ValueError):
        evidence.add(.99, True)
    for rule in ((2, .1, .5), (3.5, .1, .5), (3, 0., .5)):
        with pytest.raises(ValueError):
            ForceEvidence(*rule)
