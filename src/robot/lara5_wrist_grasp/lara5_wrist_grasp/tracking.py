"""Fixed geometric association and evidence from distinct source measurements.

Association is deliberately conservative: it cannot prove semantic identity or
distinguish an identical replacement occupying the same observed surface.
"""

from dataclasses import dataclass

import numpy as np


DEFAULTS = dict(tracking_surface_tolerance=.006, tracking_min_overlap=.7,
    tracking_extent_fraction=.3, target_confirm_frames=3, target_confirm_duration=.15,
    target_consistency_tolerance=.002)


def validate(config):
    for name in DEFAULTS:
        if not np.isfinite(config[name]) or config[name] <= 0:
            raise ValueError(f'{name} must be finite and positive')
    if (config['tracking_min_overlap'] > 1 or config['tracking_extent_fraction'] > 1
            or config['target_confirm_frames'] < 3
            or int(config['target_confirm_frames']) != config['target_confirm_frames']
            or config['target_consistency_tolerance'] > config['position_tolerance']):
        raise ValueError('Tracking requires valid fractions, at least three frames, and consistency <= position tolerance')


@dataclass
class Candidate:
    reference: np.ndarray
    points: np.ndarray

    @classmethod
    def from_selection(cls, selected, base_optical):
        return cls((base_optical @ np.r_[selected.reference_optical, 1.])[:3],
                   selected.points_optical @ base_optical[:3, :3].T + base_optical[:3, 3])


class TargetTracker:
    def __init__(self, config):
        self.cfg = config
        self.seed = self.last = None
        self.stamp = None
        self.profile = None
        self.failure = None
        self.frames = 0

    def fail(self, message):
        self.failure = self.failure or str(message)

    def observe(self, candidates, stamp, profile):
        if self.failure:
            raise ValueError('Target track invalid: '+self.failure)
        try:
            if not np.isfinite(stamp) or stamp <= 0:
                raise ValueError('Target observation requires a positive source timestamp')
            if self.stamp is not None and stamp < self.stamp:
                raise ValueError('Backwards target source timestamp')
            if self.profile is not None and profile != self.profile:
                raise ValueError('Camera intrinsics, optical frame or mounting transform changed during tracking')
            if self.stamp == stamp:
                return self.last.reference.copy()  # Rechecks do not add evidence.
            if self.seed is None:
                if len(candidates) != 1:
                    raise ValueError('Initial target acquisition requires one unambiguous depth object')
                chosen = candidates[0]
                self.seed = Candidate(chosen.reference.copy(), chosen.points.copy())
                self.profile = profile
            else:
                matches = [item for item in candidates if self.matches(item)]
                if len(matches) != 1:
                    raise ValueError('Target lost or association ambiguous; explicit new request required')
                chosen = matches[0]
            self.last, self.stamp = chosen, float(stamp)
            self.frames += 1
            return chosen.reference.copy()
        except ValueError as error:
            self.fail(error)
            raise

    def matches(self, candidate):
        c, seed = self.cfg, self.seed
        if np.linalg.norm(candidate.reference-seed.reference) > c['target_motion_tolerance']:
            return False
        original_extent = np.ptp(np.percentile(seed.points, [5, 95], axis=0), axis=0)
        current_extent = np.ptp(np.percentile(candidate.points, [5, 95], axis=0), axis=0)
        if np.any(np.abs(current_extent-original_extent) >
                  original_extent*c['tracking_extent_fraction']+c['tracking_surface_tolerance']):
            return False
        distance2 = np.sum((seed.points[:, None, :]-candidate.points[None, :, :])**2, axis=2)
        limit2 = c['tracking_surface_tolerance']**2
        return (np.mean(distance2.min(axis=0) <= limit2) >= c['tracking_min_overlap']
                and np.mean(distance2.min(axis=1) <= limit2) >= c['tracking_min_overlap'])

    def report(self):
        return dict(active=self.seed is not None and self.failure is None,
            distinct_frames=self.frames, last_source_stamp=self.stamp, failure=self.failure,
            locked_reference_base_m=self.seed.reference.tolist() if self.seed else None,
            latest_reference_base_m=self.last.reference.tolist() if self.last else None,
            identity_rule='fixed_base_surface_overlap_and_reference', semantic_identity_verified=False)


class ConsistentFrames:
    def __init__(self, config, after):
        self.cfg, self.after = config, after
        self.samples = []
        self.last_stamp = after

    def add(self, stamp, reference):
        if stamp <= self.last_stamp:
            return False
        self.last_stamp = stamp
        reference = np.asarray(reference).copy()
        if (self.samples and stamp-self.samples[-1][0] > self.cfg['feedback_max_age']):
            self.samples = []
        if any(np.linalg.norm(reference-previous) > self.cfg['target_consistency_tolerance']
               for _, previous in self.samples):
            self.samples = []
        self.samples.append((stamp, reference))
        # Bound evidence memory while preserving a full minimum-duration window.
        while (len(self.samples) > self.cfg['target_confirm_frames']
               and stamp-self.samples[1][0] >= self.cfg['target_confirm_duration']):
            self.samples.pop(0)
        return (len(self.samples) >= self.cfg['target_confirm_frames']
                and stamp-self.samples[0][0] >= self.cfg['target_confirm_duration'])

    def report(self):
        return dict(distinct_frames=len(self.samples), source_span_seconds=(
            self.samples[-1][0]-self.samples[0][0] if self.samples else 0.))
