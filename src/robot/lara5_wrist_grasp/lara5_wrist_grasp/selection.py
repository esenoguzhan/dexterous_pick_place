"""Conservative, color-independent selection of one isolated depth component.

The visible-surface median is an anchor, not an inferred object centre. The
operator supplies the measured optical-frame offset from that anchor to the
desired finger-placement reference for the first test object and view.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from lara5_sim_approach.vision import image_array
from .depth_check import measure, roi_bounds


DEFAULTS = dict(
    roi=[.1, .1, .9, .9], exclusion_rois=[0., 0., .1, 1.], min_pixels=80,
    depth_scale=.001, min_depth=.20, max_depth=.80, max_depth_spread=.12,
    depth_connectivity_m=.008, surface_to_grasp_xyz=[0., 0., 0.],
    grasp_reference_confirmed=False, remove_support_plane=False,
    support_plane_roi=[0., .8, 1., 1.], plane_distance_m=.004,
    plane_min_fraction=.8)


def validate(config):
    roi_bounds(config['roi'], 640, 480)
    exclusions = np.asarray(config['exclusion_rois'], dtype=float)
    if exclusions.ndim != 1 or len(exclusions) % 4:
        raise ValueError('exclusion_rois requires flattened normalized rectangles')
    for rectangle in exclusions.reshape(-1, 4):
        roi_bounds(rectangle, 640, 480)
    roi_bounds(config['support_plane_roi'], 640, 480)
    offset = np.asarray(config['surface_to_grasp_xyz'], dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError('surface_to_grasp_xyz requires three finite optical-frame metres')
    for key in ('depth_scale', 'min_depth', 'max_depth', 'max_depth_spread',
                'depth_connectivity_m', 'min_pixels', 'plane_distance_m', 'plane_min_fraction'):
        if not np.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if (int(config['min_pixels']) != config['min_pixels'] or config['min_pixels'] < 3
            or config['min_depth'] >= config['max_depth'] or config['plane_min_fraction'] > 1):
        raise ValueError('Invalid pixel count, depth range or plane fraction')


def rectangle_mask(rectangle, shape):
    result = np.zeros(shape, dtype=bool)
    x0, y0, x1, y1 = roi_bounds(rectangle, shape[1], shape[0])
    result[y0:y1, x0:x1] = True
    return result


def unproject(rows, cols, depths, k):
    z = depths[rows, cols]
    return np.column_stack(((cols-k[2])*z/k[0], (rows-k[5])*z/k[4], z))


def support_plane(points, tolerance, fraction):
    """Optional RANSAC fit to a user-selected table/background patch only.

    Missing or nonplanar context is harmless: depth isolation still applies.
    No plane or table height is required to define the grasp reference.
    """
    if len(points) < 30:
        return None
    points = points[np.linspace(0, len(points)-1, min(len(points), 1200), dtype=int)]
    rng = np.random.default_rng(0)
    best = np.zeros(len(points), dtype=bool)
    for _ in range(64):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        inliers = np.abs((points-a) @ normal) <= tolerance
        if inliers.sum() > best.sum():
            best = inliers
    if best.mean() < fraction:
        return None
    center = points[best].mean(axis=0)
    _, singular, axes = np.linalg.svd(points[best]-center, full_matrices=False)
    if singular[1] < 1e-5:
        return None
    normal = axes[-1]
    if np.mean(np.abs((points-center) @ normal) <= tolerance) < fraction:
        return None
    return normal, float(center @ normal)


@dataclass
class Selection:
    surface_optical: np.ndarray
    reference_optical: np.ndarray
    mask: np.ndarray
    plane_removed: bool
    depth_spread_m: float
    points_optical: np.ndarray

    def report(self):
        rows, cols = np.nonzero(self.mask)
        return dict(surface_optical_m=self.surface_optical.tolist(),
            grasp_reference_optical_m=self.reference_optical.tolist(),
            selected_pixels=int(self.mask.sum()),
            selected_bounds_pixels=[int(cols.min()), int(rows.min()),
                                    int(cols.max()+1), int(rows.max()+1)],
            support_plane_removed=self.plane_removed, depth_spread_m=self.depth_spread_m,
            reference_rule='visible_surface_median_plus_optical_offset',
            object_center_inferred=False, object_identity_verified=False)


def select_candidates(rgb, depth, info, config, max_candidates=16):
    validate(config)
    color, _, _ = measure(rgb, depth, info, config['roi'], config['depth_scale'],
                           config['min_depth'], config['max_depth'])
    # measure validates aligned image dimensions, optical frames and intrinsics.
    depths = image_array(depth, config['depth_scale'])
    valid = (np.isfinite(depths) & (depths >= config['min_depth'])
             & (depths <= config['max_depth']))
    excluded = np.zeros(depths.shape, dtype=bool)
    for rectangle in np.asarray(config['exclusion_rois']).reshape(-1, 4):
        excluded |= rectangle_mask(rectangle, depths.shape)
    allowed = rectangle_mask(config['roi'], depths.shape) & ~excluded
    plane = None
    if config['remove_support_plane']:
        context = rectangle_mask(config['support_plane_roi'], depths.shape) & valid & ~excluded
        rows, cols = np.nonzero(context)
        plane = support_plane(unproject(rows, cols, depths, info.k),
                              config['plane_distance_m'], config['plane_min_fraction'])
    foreground = valid & allowed
    if plane is not None:
        rows, cols = np.nonzero(foreground)
        on_plane = np.abs(unproject(rows, cols, depths, info.k) @ plane[0]-plane[1]) <= config['plane_distance_m']
        foreground[rows[on_plane], cols[on_plane]] = False
    if not foreground.any():
        raise ValueError('No valid depth foreground in ROI after hand/plane exclusion')

    # OpenCV floating-range flood fill joins only neighbouring pixels whose
    # depth difference is small. RGB never participates in membership.
    work = np.ascontiguousarray(depths, dtype=np.float32)
    visited = np.ones((depths.shape[0]+2, depths.shape[1]+2), np.uint8)
    visited[1:-1, 1:-1] = (~foreground).astype(np.uint8)
    # Reject components cut by ROI, image or hand-exclusion boundaries: their
    # visible anchor would depend on a crop rather than the isolated surface.
    boundary = cv2.dilate(np.pad((~allowed).astype(np.uint8), 1, constant_values=1),
                          np.ones((3, 3), np.uint8))[1:-1, 1:-1].astype(bool)
    candidates = []
    for index in np.flatnonzero(foreground):
        row, col = divmod(int(index), depths.shape[1])
        if visited[row+1, col+1]:
            continue
        count, _, _, rect = cv2.floodFill(work, visited, (col, row), 0.,
            loDiff=config['depth_connectivity_m'], upDiff=config['depth_connectivity_m'],
            flags=4 | cv2.FLOODFILL_MASK_ONLY | (2 << 8))
        x, y, w, h = rect
        patch = visited[y+1:y+h+1, x+1:x+w+1]
        component = patch == 2
        if count >= config['min_pixels'] and not np.any(boundary[y:y+h, x:x+w] & component):
            mask = np.zeros(depths.shape, dtype=bool)
            mask[y:y+h, x:x+w] = component
            candidates.append(mask)
            if len(candidates) > max_candidates:
                raise ValueError('Too many depth components for reliable target association')
        patch[component] = 1
    result = []
    for mask in candidates:
        rows, cols = np.nonzero(mask)
        spread = float(np.ptp(np.percentile(depths[mask], [5, 95])))
        points = unproject(rows, cols, depths, info.k)
        surface = np.median(points, axis=0)
        reference = surface + np.asarray(config['surface_to_grasp_xyz'])
        if not np.isfinite(reference).all() or reference[2] <= 0:
            raise ValueError('Grasp reference must be finite and in front of the optical camera')
        points = points[np.linspace(0, len(points)-1, min(len(points), 512), dtype=int)]
        result.append(Selection(surface, reference, mask, plane is not None, spread, points))
    return color, result


def select_target(rgb, depth, info, config):
    color, candidates = select_candidates(rgb, depth, info, config)
    if len(candidates) != 1:
        raise ValueError(f'One unambiguous isolated depth object required; found {len(candidates)} '
                         '(check ROI, hand exclusions, depth gaps and clipped boundaries)')
    selected = candidates[0]
    if selected.depth_spread_m > config['max_depth_spread']:
        raise ValueError('Selected surface depth spread exceeds max_depth_spread')
    return color, selected


def overlay(color, info, config, selection=None, error=None):
    output = color.copy()
    h, w = output.shape[:2]
    x0, y0, x1, y1 = roi_bounds(config['roi'], w, h)
    cv2.rectangle(output, (x0, y0), (x1-1, y1-1), (255, 255, 255), 1)
    for rectangle in np.asarray(config['exclusion_rois']).reshape(-1, 4):
        a, b, c, d = roi_bounds(rectangle, w, h)
        cv2.rectangle(output, (a, b), (c-1, d-1), (255, 60, 60), 2)
    if selection is not None:
        output[selection.mask] = (.55*output[selection.mask]+.45*np.array([0, 220, 220])).astype(np.uint8)
        for point, tint in ((selection.surface_optical, (255, 180, 0)),
                            (selection.reference_optical, (255, 0, 255))):
            u = info.k[0]*(point[0]/point[2])+info.k[2]
            v = info.k[4]*(point[1]/point[2])+info.k[5]
            # Very large offsets can project off-screen; the 3-D value is in status.
            if np.isfinite([u, v]).all() and 0 <= u < w and 0 <= v < h:
                cv2.drawMarker(output, (int(round(u)), int(round(v))), tint, cv2.MARKER_CROSS, 12, 2)
    lines = ['READ ONLY | cyan: selected | amber: surface | magenta: grasp reference',
             error or 'Surface median + measured optical offset; hidden geometry is not inferred']
    for i, line in enumerate(lines):
        cv2.putText(output, line, (5, 18+i*20), cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 0, 0), 3)
        cv2.putText(output, line, (5, 18+i*20), cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1)
    return output
