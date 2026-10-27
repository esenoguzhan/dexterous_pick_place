"""Read-only depth feasibility measurements; no detection, TF or motion APIs."""

import numpy as np

from lara5_sim_approach.vision import image_array


def roi_bounds(roi, width, height):
    value = np.asarray(roi, dtype=float)
    if (value.shape != (4,) or not np.isfinite(value).all()
            or not (0 <= value[0] < value[2] <= 1 and 0 <= value[1] < value[3] <= 1)):
        raise ValueError('roi requires normalized [left, top, right, bottom]')
    x0, y0, x1, y1 = (int(value[0]*width), int(value[1]*height),
                       int(value[2]*width), int(value[3]*height))
    if x1 <= x0 or y1 <= y0:
        raise ValueError('ROI contains no pixels at this resolution')
    return x0, y0, x1, y1


def measure(rgb, depth, info, roi, depth_scale, minimum, maximum):
    if rgb.encoding not in ('rgb8', 'bgr8') or depth.encoding not in ('16UC1', '32FC1'):
        raise ValueError('Require RGB/BGR and aligned 16UC1/32FC1 depth')
    color, distances = image_array(rgb), image_array(depth, depth_scale)
    if color.shape[:2] != distances.shape or distances.shape != (info.height, info.width):
        raise ValueError('Color, aligned depth and CameraInfo dimensions must match')
    if (not rgb.header.frame_id or rgb.header.frame_id != depth.header.frame_id
            or rgb.header.frame_id != info.header.frame_id):
        raise ValueError('Color, aligned depth and CameraInfo must share the optical frame')
    k = np.asarray(info.k, dtype=float)
    if k.shape != (9,) or not np.isfinite(k).all() or k[0] <= 0 or k[4] <= 0:
        raise ValueError('Invalid color CameraInfo intrinsics')
    x0, y0, x1, y1 = roi_bounds(roi, info.width, info.height)
    patch = distances[y0:y1, x0:x1]
    valid = np.isfinite(patch) & (patch > 0)
    within = valid & (patch >= minimum) & (patch <= maximum)
    values = patch[valid]
    # Deliberately DO NOT clip depth at the approach cutoff: this diagnostic
    # distinguishes a software rejection from missing sensor measurements.
    percentiles = np.percentile(values, [5, 50, 95]).tolist() if values.size else [None]*3
    return color, patch, dict(
        roi_pixels=[x0, y0, x1, y1], pixels=int(patch.size),
        valid_pixels=int(valid.sum()), valid_fraction=float(valid.mean()),
        within_approach_range_fraction=float(within.mean()),
        below_approach_min_fraction=float((valid & (patch < minimum)).mean()),
        depth_p05_m=percentiles[0], depth_median_m=percentiles[1], depth_p95_m=percentiles[2],
        spatial_spread_m=percentiles[2]-percentiles[0] if values.size else None)


def summarize(samples, config, expected_depth=0.):
    """Samples are (unique depth source time, frame statistics), in time order."""
    reasons = []
    count = len(samples)
    span = samples[-1][0]-samples[0][0] if count > 1 else 0.
    if count < config['min_frames'] or span < config['capture_seconds']:
        reasons.append('insufficient_new_frames')
    stats = [sample[1] for sample in samples]
    valid = min((s['valid_fraction'] for s in stats), default=0.)
    within = min((s['within_approach_range_fraction'] for s in stats), default=0.)
    if valid < config['min_valid_fraction']:
        reasons.append('missing_depth_in_roi')
    spreads = [s['spatial_spread_m'] for s in stats if s['spatial_spread_m'] is not None]
    spatial = max(spreads) if spreads else None
    if spatial is not None and spatial > config['max_spatial_spread_m']:
        reasons.append('mixed_or_sloped_surfaces_in_roi')
    medians = [s['depth_median_m'] for s in stats if s['depth_median_m'] is not None]
    median = float(np.median(medians)) if medians else None
    temporal = float(np.ptp(np.percentile(medians, [5, 95]))) if medians else None
    if temporal is not None and temporal > config['max_temporal_spread_m']:
        reasons.append('depth_or_scene_not_stable')
    reference_error = abs(median-expected_depth) if median is not None and expected_depth > 0 else None
    if reference_error is not None and reference_error > config['max_reference_error_m']:
        reasons.append('independent_distance_mismatch')
    return dict(
        frames=count, source_span_seconds=span,
        minimum_valid_fraction=valid, minimum_within_approach_range_fraction=within,
        depth_median_m=median, maximum_spatial_spread_m=spatial,
        temporal_median_spread_m=temporal,
        expected_surface_depth_m=expected_depth if expected_depth > 0 else None,
        reference_error_m=reference_error,
        quality_criteria_met=not reasons, reasons=reasons,
        approach_range_compatible=bool(stats and within >= config['min_valid_fraction']),
        accuracy_verified=False, object_identity_verified=False, robot_stationary_verified=False,
        closure_ready=False)
