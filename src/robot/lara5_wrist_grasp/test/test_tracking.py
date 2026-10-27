"""Fixed surface identity gates and distinct-frame evidence; no ROS transport."""

import numpy as np
import pytest

from lara5_object_approach.grasp_geometry import transform
from lara5_wrist_grasp.selection import Selection
from lara5_wrist_grasp.tracking import (DEFAULTS, Candidate, ConsistentFrames,
                                       TargetTracker, validate)


def config(**changes):
    return dict(DEFAULTS, target_motion_tolerance=.02, position_tolerance=.003,
                feedback_max_age=.5, **changes)


def object_candidate(offset=0., scale=1.):
    points = np.array([[x, y, .4] for x in np.linspace(-.03, .03, 16)
                       for y in np.linspace(-.03, .03, 16)])
    points[:, :2] *= scale
    points[:, 0] += offset
    return Candidate(np.array([offset, 0., .43]), points)


def test_robot_motion_compensation_preserves_the_original_world_surface():
    seed = object_candidate()
    track = TargetTracker(config())
    track.observe([seed], 1., ('camera',))
    for x in (.01, .04, .08):
        base_optical = transform([x, -.02, .03], [0., 0., np.sin(.05), np.cos(.05)])
        optical_base = np.linalg.inv(base_optical)
        points = seed.points @ optical_base[:3, :3].T + optical_base[:3, 3]
        reference = (optical_base @ np.r_[seed.reference, 1.])[:3]
        selected = Selection(points.mean(0), reference, np.ones((16, 16), bool), False, 0., points)
        candidate = Candidate.from_selection(selected, base_optical)
        assert np.allclose(track.observe([candidate], 1.+x, ('camera',)), seed.reference)
    assert np.allclose(track.seed.reference, seed.reference)


def test_distractor_does_not_replace_the_locked_target_or_force_nearest_selection():
    track = TargetTracker(config())
    track.observe([object_candidate()], 1., ('camera',))
    reference = track.observe([object_candidate(.12, 2.), object_candidate(.001)], 1.1, ('camera',))
    assert reference[0] == pytest.approx(.001)
    with pytest.raises(ValueError, match='lost'):
        track.observe([object_candidate(.12, 2.)], 1.2, ('camera',))
    with pytest.raises(ValueError, match='invalid'):
        track.observe([object_candidate()], 1.3, ('camera',))


@pytest.mark.parametrize('replacement', ['different_extent', 'occluded', 'different_reference', 'ambiguous'])
def test_changed_surface_reference_or_ambiguous_association_invalidates_track(replacement):
    track = TargetTracker(config())
    track.observe([object_candidate()], 1., ('camera',))
    candidate = object_candidate(scale=2. if replacement == 'different_extent' else 1.)
    if replacement == 'occluded': candidate.points = candidate.points[:32]
    if replacement == 'different_reference': candidate.reference += [.03, 0., 0.]
    candidates = [candidate, object_candidate()] if replacement == 'ambiguous' else [candidate]
    with pytest.raises(ValueError, match='lost|ambiguous'):
        track.observe(candidates, 1.1, ('camera',))
    assert track.failure and not track.report()['active']


def test_slow_drift_cannot_walk_the_seed_onto_another_surface():
    track = TargetTracker(config())
    track.observe([object_candidate()], 1., ('camera',))
    with pytest.raises(ValueError, match='lost'):
        for i in range(1, 31):
            track.observe([object_candidate(i*.001)], 1.+i*.01, ('camera',))
    assert track.seed.reference[0] == 0.


def test_duplicate_checks_do_not_count_as_measurements_and_profile_changes_fail():
    track = TargetTracker(config())
    track.observe([object_candidate()], 1., ('same_intrinsics_and_mount',))
    for _ in range(10):
        track.observe([], 1., ('same_intrinsics_and_mount',))
    assert track.frames == 1
    with pytest.raises(ValueError, match='intrinsics'):
        track.observe([object_candidate()], 1.1, ('changed_mount_or_intrinsics',))


def test_initial_ambiguity_and_backward_time_require_a_new_request():
    with pytest.raises(ValueError, match='unambiguous'):
        TargetTracker(config()).observe([object_candidate(), object_candidate(.1)], 1., ())
    track = TargetTracker(config())
    track.observe([object_candidate()], 1., ())
    with pytest.raises(ValueError, match='Backwards'):
        track.observe([object_candidate()], .9, ())


def test_evidence_requires_new_frames_spanning_the_interval_after_start():
    evidence = ConsistentFrames(config(), after=10.)
    point = [0., 0., .4]
    assert not evidence.add(9.9, point)
    assert not evidence.add(10.01, point)
    for _ in range(20): assert not evidence.add(10.01, point)
    assert not evidence.add(10.02, point)
    assert evidence.add(10.17, point)
    assert evidence.report()['distinct_frames'] == 3


def test_unstable_reference_and_long_measurement_gaps_restart_evidence():
    evidence = ConsistentFrames(config(), after=1.)
    assert not evidence.add(1.01, [0., 0., .4])
    assert not evidence.add(1.10, [.004, 0., .4])
    assert not evidence.add(1.20, [.004, 0., .4])
    assert evidence.add(1.27, [.004, 0., .4])
    assert not evidence.add(2., [.004, 0., .4])
    assert evidence.report()['distinct_frames'] == 1


@pytest.mark.parametrize('changes', [dict(target_confirm_frames=1), dict(tracking_min_overlap=1.2),
    dict(target_consistency_tolerance=.004), dict(tracking_surface_tolerance=0.)])
def test_invalid_tracking_configuration_is_rejected(changes):
    with pytest.raises(ValueError): validate(config(**changes))
