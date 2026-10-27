"""Force characterization rules on synthetic recordings; no ROS or hardware."""

from types import SimpleNamespace as NS

import numpy as np
import pytest

from lara5_grasp.configuration import HAND_JOINTS
from lara5_grasp.force_check import assess, closure_peak_speed, measurement, peak_speeds, summarize

CONFIG = dict(close_positions=[1.1, 1.3, 1.3, 1.3, 1.3, .5], max_joint_step=.02, max_joint_speed=.10,
    max_joint_acceleration=.25, min_segment_duration=.4, open_start_tolerance=.08,
    open_capture_seconds=3., min_capture_seconds=1., min_samples=30, stationary_tolerance=.02,
    loaded_motion_tolerance=.05, coverage_fraction=.9, speed_window_seconds=.2,
    threshold_margin_factor=1.5, threshold_margin_raw=2.)
CANDIDATE = dict(hardware_force_baseline=[50.]*6, hardware_force_contact_thresholds=[25.]*6,
                 hardware_force_limits=[200.]*6)


def open_samples(noise=3., seconds=3.2, positions=None):
    return [(1.+i/100, list(positions or [0.]*6), [50.+(noise if i % 2 else -noise)]*6)
            for i in range(int(seconds*100)+1)]


def motion_samples(motion_force=10., seconds=12., scale=1.):
    samples = []
    for i in range(int(seconds*100)+1):
        t = i/100
        phase = min(t, seconds-t)/(seconds/2)  # close, then reopen
        samples.append((10.+t, [scale*c*phase for c in CONFIG['close_positions']],
                        [50.+motion_force*phase]*6))
    return samples


def loaded_samples(channel, load):
    forces = [50.]*6
    forces[HAND_JOINTS.index(channel)] += load
    return [(20.+i/100, [.1]*6, list(forces)) for i in range(151)]


def recording(kind, samples, **values):
    return dict(summary=summarize(kind, samples, CONFIG, **values), samples=samples)


def complete(skip=(), load=60., reference=0.):
    result = dict(unloaded_open=recording('unloaded_open', open_samples()),
                  unloaded_motion=recording('unloaded_motion', motion_samples()))
    for channel in HAND_JOINTS[1:]:  # thumb pitch represents the thumb
        if channel not in skip:
            result['loaded/'+channel] = recording('loaded', loaded_samples(channel, load),
                                                  channel=channel, reference_n=reference)
    return result


def test_measurement_reorders_named_channels_and_rejects_bad_messages():
    message = NS(name=list(reversed(HAND_JOINTS)), position=list(range(6)), effort=list(range(10, 16)))
    positions, forces = measurement(message)
    assert positions == [5, 4, 3, 2, 1, 0] and forces == [15, 14, 13, 12, 11, 10]
    for fault in ('duplicate', 'nan', 'short'):
        bad = NS(name=list(HAND_JOINTS), position=[0.]*6, effort=[0.]*6)
        if fault == 'duplicate': bad.name[1] = bad.name[0]
        if fault == 'nan': bad.effort[2] = float('nan')
        if fault == 'short': bad.effort = [0.]*5
        with pytest.raises(ValueError):
            measurement(bad)


@pytest.mark.parametrize('fault', [None, 'closed', 'moved', 'short'])
def test_open_recording_requires_new_open_stationary_measurements(fault):
    samples = open_samples(seconds=1. if fault == 'short' else 3.2,
                           positions=[.1]*6 if fault == 'closed' else None)
    if fault == 'moved':
        samples[50][1][2] = .03
    result = summarize('unloaded_open', samples, CONFIG)
    expected = dict(closed='hand_not_open', moved='hand_moved', short='insufficient_new_measurements')
    assert result['quality_criteria_met'] == (fault is None)
    if fault:
        assert expected[fault] in result['reasons']
    assert result['channels'][HAND_JOINTS[0]]['force_mean_raw'] == pytest.approx(50., abs=.1)


def test_free_air_motion_must_cover_the_closure_range_at_closure_speed():
    nominal = closure_peak_speed(CONFIG)
    assert nominal == pytest.approx(1.5*.02/np.sqrt(6*.02/.25))
    good = summarize('unloaded_motion', motion_samples(), CONFIG)
    assert good['quality_criteria_met'] and min(good['closure_range_coverage'].values()) >= .99
    partial = summarize('unloaded_motion', motion_samples(scale=.5), CONFIG)
    assert 'incomplete_closure_range' in partial['reasons']
    slow = summarize('unloaded_motion', motion_samples(seconds=120.), CONFIG)
    assert slow['reasons'] == ['slower_than_closure']


def test_windowed_speed_ignores_a_single_angle_quantization_step():
    times = np.arange(100)/100
    positions = np.zeros((100, 6)); positions[50:, 0] = .0014
    assert peak_speeds(times, positions, .2)[0] <= .0014/.2+1e-9


def test_consistent_settings_still_leave_maximum_forces_to_the_operator():
    result = assess(complete(), CANDIDATE, CONFIG)
    assert result['configuration_consistent_with_measurements'], result['reasons']
    assert result['suggested_baseline_raw'] == pytest.approx([50.]*6, abs=.1)
    channel = result['channels'][HAND_JOINTS[1]]
    # Worst unloaded deviation is the 10-unit motion effect, not open noise.
    assert channel['unloaded_peak_deviation_raw'] == pytest.approx(10.)
    assert channel['required_minimum_threshold_raw'] == pytest.approx(17.)
    assert channel['contact_detectable'] and not channel['loaded_reached_limit']
    assert not result['force_limits_validated'] and not result['closure_ready']
    assert not result['retention_verified'] and 'unloaded' in result['limits_basis']


@pytest.mark.parametrize('fault', ['threshold', 'baseline', 'weak_load', 'missing_load',
                                   'no_motion', 'unconfigured'])
def test_inconsistent_settings_or_missing_evidence_are_reported(fault):
    candidate = {key: list(value) for key, value in CANDIDATE.items()}
    recordings = complete(skip=(HAND_JOINTS[4],) if fault == 'missing_load' else (),
                          load=20. if fault == 'weak_load' else 60.)
    if fault == 'threshold': candidate['hardware_force_contact_thresholds'] = [15.]*6
    # A stale baseline consumes the threshold margin even with unchanged noise.
    if fault == 'baseline': candidate['hardware_force_baseline'] = [40.]*6
    if fault == 'no_motion': del recordings['unloaded_motion']
    if fault == 'unconfigured': candidate = dict(hardware_force_baseline=[0.]*6,
        hardware_force_contact_thresholds=[0.]*6, hardware_force_limits=[0.]*6)
    result = assess(recordings, candidate, CONFIG)
    assert not result['configuration_consistent_with_measurements']
    expected = dict(threshold='threshold_within_unloaded_margin', baseline='threshold_within_unloaded_margin',
                    weak_load='contact_not_detectable:index', missing_load='no_loaded_evidence:pinky',
                    no_motion='unloaded_motion_missing_or_failed', unconfigured='candidate_force_settings_invalid')
    assert any(reason.startswith(expected[fault]) for reason in result['reasons'])


def test_failed_recordings_are_not_used_as_evidence():
    recordings = complete()
    recordings['unloaded_open'] = recording('unloaded_open', open_samples(positions=[.2]*6))
    result = assess(recordings, CANDIDATE, CONFIG)
    assert 'unloaded_open_missing_or_failed' in result['reasons']
    assert result['suggested_baseline_raw'] is None


def test_reference_load_gives_one_point_approximate_scale():
    result = assess(complete(reference=2.), CANDIDATE, CONFIG)
    channel = result['channels'][HAND_JOINTS[1]]
    assert channel['raw_per_newton_approx'] == pytest.approx(30.)
    assert channel['threshold_n_approx'] == pytest.approx(25/30)
    assert channel['limit_n_approx'] == pytest.approx(200/30)
