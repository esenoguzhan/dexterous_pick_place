"""Depth feasibility, including valid close depth rejected by motion config."""

from types import SimpleNamespace as NS

import numpy as np
import pytest

from lara5_wrist_grasp.depth_check import measure, roi_bounds, summarize


CONFIG = dict(min_frames=3, capture_seconds=2., min_valid_fraction=.8,
              max_spatial_spread_m=.02, max_temporal_spread_m=.005, max_reference_error_m=.01)


def inputs(depth, encoding='16UC1'):
    depth = np.asarray(depth, dtype=np.uint16 if encoding == '16UC1' else np.float32)
    h, w = depth.shape
    header = NS(frame_id='optical')
    rgb = NS(header=header, height=h, width=w, step=w*3, encoding='rgb8',
             is_bigendian=False, data=np.full((h, w, 3), 100, np.uint8).tobytes())
    msg = NS(header=header, height=h, width=w, step=w*depth.dtype.itemsize,
             encoding=encoding, is_bigendian=False, data=depth.tobytes())
    info = NS(header=header, height=h, width=w, k=[100., 0., w/2, 0., 100., h/2, 0., 0., 1.])
    return rgb, msg, info


def stats(depth, encoding='16UC1'):
    return measure(*inputs(depth, encoding), [0, 0, 1, 1], .001, .2, .8)[2]


def window(frames, expected=0.):
    return summarize([(float(i), s) for i, s in enumerate(frames)], CONFIG, expected)


def test_close_depth_is_measured_without_applying_approach_cutoff():
    s = stats(np.full((10, 10), 130))
    result = window([s]*3)
    assert s['depth_median_m'] == pytest.approx(.13)
    assert s['below_approach_min_fraction'] == 1.
    assert result['quality_criteria_met'] and not result['approach_range_compatible']
    assert not result['accuracy_verified'] and not result['closure_ready']


def test_white_object_and_float_depth_work_without_color_or_dimensions():
    s = stats(np.full((10, 10), .25), '32FC1')
    result = window([s]*3)
    assert result['quality_criteria_met'] and result['approach_range_compatible']


def test_zero_nan_inf_and_negative_depth_are_holes_not_near_objects():
    s = stats([[0., np.nan], [np.inf, -.1]], '32FC1')
    result = window([s]*3)
    assert s['valid_fraction'] == 0 and s['depth_median_m'] is None
    assert not result['quality_criteria_met'] and not result['approach_range_compatible']
    assert result['temporal_median_spread_m'] is None


def test_intermittent_missing_depth_does_not_average_into_a_pass():
    good, missing = stats(np.full((10, 10), 250)), stats(np.zeros((10, 10)))
    result = window([good, missing, good])
    assert 'missing_depth_in_roi' in result['reasons']


def test_background_in_roi_and_moving_scene_fail_distinct_checks():
    mixed = stats(np.array([[130, 130], [700, 700]]))
    assert 'mixed_or_sloped_surfaces_in_roi' in window([mixed]*3)['reasons']
    frames = [stats(np.full((10, 10), z)) for z in (130, 150, 170)]
    assert 'depth_or_scene_not_stable' in window(frames)['reasons']


def test_stable_depth_is_not_accuracy_and_bad_reference_is_explicit():
    s = stats(np.full((10, 10), 250))
    result = window([s]*3, expected=.13)
    assert 'independent_distance_mismatch' in result['reasons']
    assert result['reference_error_m'] == pytest.approx(.12)


def test_short_recording_cannot_pass():
    result = window([stats(np.full((10, 10), 250))])
    assert 'insufficient_new_frames' in result['reasons']


@pytest.mark.parametrize('roi', [[0., 0., 0., 1.], [-.1, 0., 1., 1.], [0., np.nan, 1., 1.]])
def test_invalid_roi_rejected(roi):
    with pytest.raises(ValueError):
        roi_bounds(roi, 640, 480)


def test_alignment_and_intrinsics_are_required():
    rgb, depth, info = inputs(np.full((10, 10), 250))
    depth.header = NS(frame_id='depth_optical')
    with pytest.raises(ValueError, match='share the optical frame'):
        measure(rgb, depth, info, [0., 0., 1., 1.], .001, .2, .8)
    depth.header = rgb.header
    info.k[0] = 0.
    with pytest.raises(ValueError, match='intrinsics'):
        measure(rgb, depth, info, [0., 0., 1., 1.], .001, .2, .8)
