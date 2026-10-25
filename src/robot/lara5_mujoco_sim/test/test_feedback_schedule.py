"""Contact cadence must follow wall time even when physics runs below 500 Hz."""

import pytest

from lara5_mujoco_sim.feedback_schedule import FeedbackSchedule


@pytest.mark.parametrize('physics_period', [.002, .02, .04, .08])
def test_slow_fresh_physics_does_not_create_a_half_second_feedback_gap(physics_period):
    schedule = FeedbackSchedule(20.)
    samples = [i*physics_period for i in range(round(2/physics_period))
               if schedule.due(i*physics_period)]
    assert len(samples) >= 20
    gaps = [b-a for a,b in zip(samples,samples[1:])]
    assert max(gaps) < .15  # Above 12.5 fresh steps/s, all far below the .5s cutoff.
    assert min(gaps) >= .05-1e-8  # No catch-up bursts on a fast machine.


def test_no_new_step_produces_no_fake_measurement_and_no_catchup_burst():
    schedule = FeedbackSchedule(20.)
    assert schedule.due(0.)
    assert not schedule.due(0.)
    # The caller has been blocked for a second. Only its new completed step
    # authorizes one new sample; repeated checks cannot restamp the old one.
    assert schedule.due(1.)
    assert not schedule.due(1.)
    assert not schedule.due(1.01)
