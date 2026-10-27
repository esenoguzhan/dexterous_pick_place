"""Depth membership, hand exclusion and explicit reference geometry; no robot."""

import numpy as np
import pytest

from lara5_wrist_grasp.core import detect_target
from lara5_wrist_grasp.selection import DEFAULTS, overlay, select_target, validate
from test_core import image, target_inputs


def selected(inputs=None, **changes):
    return select_target(*(inputs or target_inputs()), dict(DEFAULTS, **changes))[1]


@pytest.mark.parametrize('rgb', [[0, 0, 0], [100, 100, 100], [255, 0, 0], [0, 0, 255], [0, 255, 0]])
def test_color_never_changes_depth_selection(rgb):
    _, depth, info = target_inputs()
    color = image(np.full((40, 40, 3), rgb, np.uint8), 'rgb8')
    result = selected((color, depth, info), surface_to_grasp_xyz=[.01, -.02, .03])
    assert result.mask.sum() == 400
    assert np.allclose(result.surface_optical, [-.00175, -.00175, .35])
    assert np.allclose(result.reference_optical-result.surface_optical, [.01, -.02, .03])
    assert not result.report()['object_center_inferred']


def test_offset_is_applied_in_optical_axes_before_transforming_to_robot_base():
    mount = np.eye(4)
    mount[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    mount[:3, 3] = [.4, -.2, .1]
    config = dict(DEFAULTS, surface_to_grasp_xyz=[.03, .01, -.02])
    reference = detect_target(*target_inputs(), mount, config)
    assert np.allclose(reference, mount[:3, :3] @ np.array([.02825, .00825, .33])+mount[:3, 3])


def test_hand_is_removed_by_explicit_exclusion_and_ambiguity_is_not_silently_ranked():
    rgb, _, info = target_inputs()
    depths = np.full((40, 40), 700, np.uint16)
    depths[10:25, 7:16] = 250  # near hand, not necessarily the target
    depths[10:30, 22:33] = 350
    inputs = (rgb, image(depths, '16UC1'), info)
    with pytest.raises(ValueError, match='found 2'):
        selected(inputs)
    result = selected(inputs, exclusion_rois=[0., 0., .45, 1.])
    assert result.mask.sum() == 220
    assert result.surface_optical[2] == pytest.approx(.35)


@pytest.mark.parametrize('changes', [dict(roi=[.3, .1, .9, .9]),
                                     dict(exclusion_rois=[.3, .1, .4, .9])])
def test_cropped_object_anchor_is_rejected(changes):
    with pytest.raises(ValueError, match='found 0'):
        selected(**changes)


def test_featureless_background_filling_roi_is_not_an_object():
    rgb, _, info = target_inputs()
    with pytest.raises(ValueError, match='found 0'):
        selected((rgb, image(np.full((40, 40), 350, np.uint16), '16UC1'), info))


def test_optional_table_patch_separates_a_depth_connected_support_surface():
    rgb, _, info = target_inputs()
    depths = np.full((40, 40), .600, np.float32)
    depths[10:30, 10:30] = .595
    inputs = (rgb, image(depths, '32FC1'), info)
    with pytest.raises(ValueError, match='found 0'):
        selected(inputs)
    result = selected(inputs, remove_support_plane=True, support_plane_roi=[.1, .8, .9, 1.])
    assert result.plane_removed and result.mask.sum() == 400


def test_absent_table_is_not_required_for_selection():
    rgb, _, info = target_inputs()
    depths = np.zeros((40, 40), np.uint16)
    depths[10:30, 10:30] = 350
    result = selected((rgb, image(depths, '16UC1'), info), remove_support_plane=True)
    assert result.mask.sum() == 400 and not result.plane_removed


def test_broad_sloped_surface_is_rejected_even_with_local_depth_connectivity():
    rgb, _, info = target_inputs()
    depths = np.full((40, 40), .7, np.float32)
    depths[10:30, 10:30] = .3+.006*np.arange(20)[None, :]
    with pytest.raises(ValueError, match='spread'):
        selected((rgb, image(depths, '32FC1'), info), max_depth_spread=.02)


@pytest.mark.parametrize('bad', [0., np.nan, np.inf, .1, 1.])
def test_missing_nonfinite_or_out_of_range_depth_cannot_select(bad):
    rgb, _, info = target_inputs()
    with pytest.raises(ValueError, match='valid depth'):
        selected((rgb, image(np.full((40, 40), bad, np.float32), '32FC1'), info))


@pytest.mark.parametrize('changes', [dict(roi=[0., 0., 0., 1.]),
    dict(exclusion_rois=[0., .1, .2]), dict(surface_to_grasp_xyz=[0., np.nan, 0.]),
    dict(depth_connectivity_m=0.), dict(plane_min_fraction=1.1), dict(min_pixels=2)])
def test_invalid_selection_configuration_is_rejected(changes):
    with pytest.raises(ValueError):
        validate(dict(DEFAULTS, **changes))


def test_overlay_contains_mask_and_reference_with_distinct_markers():
    rgb, depth, info = target_inputs()
    config = dict(DEFAULTS, surface_to_grasp_xyz=[.02, 0., .03])
    color, result = select_target(rgb, depth, info, config)
    output = overlay(color, info, config, result)
    assert output.shape == color.shape and output.dtype == np.uint8
    assert not np.array_equal(output, color)
    assert np.any(np.all(output == [255, 0, 255], axis=2))
