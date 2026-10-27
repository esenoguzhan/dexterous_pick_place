"""Read-only hand force characterization; no hand, arm or controller commands."""

import math

import numpy as np

from .configuration import HAND_JOINTS
from .hardware import validate_force_limits
from .policy import DIGITS

KINDS = ('unloaded_open', 'unloaded_motion', 'loaded')


def measurement(message):
    """Measured radians and raw forces reordered by joint NAME."""
    if (len(message.name) != 6 or set(message.name) != set(HAND_JOINTS)
            or len(message.position) != 6 or len(message.effort) != 6):
        raise ValueError('Require six unique named hand position/force channels')
    positions, forces = dict(zip(message.name, message.position)), dict(zip(message.name, message.effort))
    values = [positions[name] for name in HAND_JOINTS], [forces[name] for name in HAND_JOINTS]
    if not np.isfinite(values).all():
        raise ValueError('Nonfinite physical hand measurement')
    return values


def closure_peak_speed(config):
    """Nominal peak joint speed of one full closure segment (cubic, zero end velocity)."""
    step = config['max_joint_step']
    duration = max(config['min_segment_duration'], 1.5*step/config['max_joint_speed'],
                   math.sqrt(6*step/config['max_joint_acceleration']))
    return 1.5*step/duration


def peak_speeds(times, positions, window):
    """Per-joint peak speed over at least `window` seconds; ignores quantization steps."""
    best = np.zeros(positions.shape[1])
    j = 0
    for i in range(len(times)):
        while j < len(times) and times[j]-times[i] < window:
            j += 1
        if j == len(times):
            break
        best = np.maximum(best, np.abs(positions[j]-positions[i])/(times[j]-times[i]))
    return best


def summarize(kind, samples, config, channel='', reference_n=0.):
    """Samples are (new source time, positions, forces), in time order."""
    reasons = []
    times = np.array([s[0] for s in samples])
    positions = np.array([s[1] for s in samples]).reshape(-1, 6)
    forces = np.array([s[2] for s in samples]).reshape(-1, 6)
    count = len(samples)
    span = float(times[-1]-times[0]) if count > 1 else 0.
    minimum = config['open_capture_seconds'] if kind == 'unloaded_open' else config['min_capture_seconds']
    if count < config['min_samples'] or span < minimum:
        reasons.append('insufficient_new_measurements')
    result = dict(kind=kind, measurements=count, source_span_seconds=span,
                  max_gap_seconds=float(np.max(np.diff(times))) if count > 1 else None)
    if count:
        travel = np.ptp(positions, axis=0)
        if kind == 'unloaded_open':
            if np.max(np.abs(positions)) > config['open_start_tolerance']:
                reasons.append('hand_not_open')
            if np.max(travel) > config['stationary_tolerance']:
                reasons.append('hand_moved')
        elif kind == 'unloaded_motion':
            coverage = np.max(positions, axis=0)/np.asarray(config['close_positions'])
            speeds = peak_speeds(times, positions, config['speed_window_seconds'])
            nominal = closure_peak_speed(config)
            if np.any(coverage < config['coverage_fraction']):
                reasons.append('incomplete_closure_range')
            if np.any(speeds < nominal):
                reasons.append('slower_than_closure')
            result.update(closure_range_coverage=dict(zip(HAND_JOINTS, coverage.tolist())),
                          peak_joint_speed_rad_s=dict(zip(HAND_JOINTS, speeds.tolist())),
                          closure_nominal_peak_speed_rad_s=nominal)
        elif np.max(travel) > config['loaded_motion_tolerance']:
            reasons.append('hand_moved')
        result['channels'] = {name: dict(
            force_mean_raw=float(np.mean(forces[:, i])), force_std_raw=float(np.std(forces[:, i])),
            force_min_raw=float(np.min(forces[:, i])), force_max_raw=float(np.max(forces[:, i])),
            position_min_rad=float(np.min(positions[:, i])), position_max_rad=float(np.max(positions[:, i])))
            for i, name in enumerate(HAND_JOINTS)}
    if kind == 'loaded':
        result.update(loaded_channel=channel, reference_load_n=reference_n if reference_n > 0 else None)
    result.update(quality_criteria_met=not reasons, reasons=reasons)
    return result


def _forces(recording):
    return np.array([s[2] for s in recording['samples']]).reshape(-1, 6)


def assess(recordings, candidate, config):
    """Compare candidate raw force settings with recorded physical evidence.

    Unloaded data bounds false contact; only loaded data shows detectability.
    Neither chooses maximum forces, which depend on object and hand tolerance.
    """
    reasons = []
    baseline, thresholds, limits = (np.asarray(candidate[k], dtype=float) for k in (
        'hardware_force_baseline', 'hardware_force_contact_thresholds', 'hardware_force_limits'))
    try:
        validate_force_limits(baseline, thresholds, limits)
        configured = True
    except ValueError:
        configured = False
        reasons.append('candidate_force_settings_invalid')
    usable = {key: rec for key, rec in recordings.items() if rec['summary']['quality_criteria_met']}
    for kind in KINDS[:2]:
        if kind not in usable:
            reasons.append(f'{kind}_missing_or_failed')
    unloaded = [_forces(usable[k]) for k in KINDS[:2] if k in usable]
    unloaded = np.vstack(unloaded) if unloaded else None
    measured = np.mean(_forces(usable['unloaded_open']), axis=0) if 'unloaded_open' in usable else None
    factor, margin = config['threshold_margin_factor'], config['threshold_margin_raw']
    channels = {}
    for i, name in enumerate(HAND_JOINTS):
        # Deviation from the CONFIGURED baseline is what the closure evaluates.
        drift = float(np.max(np.abs(unloaded[:, i]-baseline[i]))) if unloaded is not None else None
        required = factor*drift+margin if drift is not None else None
        entry = dict(configured_baseline_raw=float(baseline[i]), configured_threshold_raw=float(thresholds[i]),
            configured_limit_raw=float(limits[i]),
            measured_open_baseline_raw=float(measured[i]) if measured is not None else None,
            unloaded_peak_deviation_raw=drift, required_minimum_threshold_raw=required,
            threshold_clears_unloaded=bool(configured and required is not None and thresholds[i] >= required))
        if measured is not None and unloaded is not None:
            entry['suggested_minimum_threshold_raw'] = float(
                factor*np.max(np.abs(unloaded[:, i]-measured[i]))+margin)
        loaded = usable.get('loaded/'+name)
        if loaded:
            peak = float(np.max(np.abs(_forces(loaded)[:, i]-baseline[i])))
            entry.update(loaded_peak_deviation_raw=peak, contact_detectable=bool(configured and peak >= thresholds[i]),
                         loaded_reached_limit=bool(configured and peak >= limits[i]))
            reference = loaded['summary']['reference_load_n']
            if reference and peak > 0:
                # One-point, approximate scale for THIS channel and press location.
                entry.update(raw_per_newton_approx=peak/reference, threshold_n_approx=thresholds[i]*reference/peak,
                             limit_n_approx=limits[i]*reference/peak)
        if required is not None and not entry['threshold_clears_unloaded']:
            reasons.append(f'threshold_within_unloaded_margin:{name}')
        channels[name] = entry
    digits = {}
    for digit in dict.fromkeys(DIGITS):
        entries = [channels[name] for name, owner in zip(HAND_JOINTS, DIGITS) if owner == digit]
        loaded = [e for e in entries if 'contact_detectable' in e]
        digits[digit] = dict(loaded_evidence=bool(loaded), contact_detectable=any(e['contact_detectable'] for e in loaded))
        if not loaded:
            reasons.append(f'no_loaded_evidence:{digit}')
        elif not digits[digit]['contact_detectable']:
            reasons.append(f'contact_not_detectable:{digit}')
    return dict(channels=channels, digits=digits, reasons=reasons,
        configuration_consistent_with_measurements=not reasons,
        suggested_baseline_raw=measured.tolist() if measured is not None else None,
        limits_basis='Operator decision from loaded evidence and object/hand tolerance; '
                     'unloaded measurements cannot set maximum forces',
        force_limits_validated=False, retention_verified=False, closure_ready=False)
