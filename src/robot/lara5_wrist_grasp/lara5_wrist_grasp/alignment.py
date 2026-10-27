"""Read-only alignment evidence and image overlay shared by both wrist previews."""

import cv2
import numpy as np

from .selection import overlay


def alignment_observation(color, info, config, selected, base_flange, flange_color,
                          grasp_offset, mount_report):
    base_color = base_flange @ flange_color
    reference = (base_color @ np.r_[selected.reference_optical, 1.])[:3]
    current_grasp = base_flange[:3, 3]+base_flange[:3, :3] @ grasp_offset
    correction = reference-current_grasp
    flange_target = base_flange[:3, 3]+correction
    grasp_optical = (np.linalg.inv(flange_color) @ np.r_[grasp_offset, 1.])[:3]
    report = dict(selected.report(), camera_mount=mount_report,
        source_stamp=info.header.stamp.sec+info.header.stamp.nanosec/1e9,
        base_frame=config['base_frame'], grasp_reference_base_m=reference.tolist(),
        current_hand_grasp_base_m=current_grasp.tolist(), correction_base_m=correction.tolist(),
        correction_distance_m=float(np.linalg.norm(correction)),
        proposed_flange_xyz_m=flange_target.tolist(), current_hand_grasp_optical_m=grasp_optical.tolist(),
        read_only=True, motion_authorized=False, closure_ready=False,
        robot_base_alignment_verified=bool(config['base_alignment_confirmed']),
        grasp_reference_verified=bool(config['grasp_reference_confirmed']),
        path_checked=False)
    output = overlay(color, info, config, selected)
    h, w = output.shape[:2]

    def pixel(point):
        if point[2] <= 0:
            return None
        u, v = info.k[0]*(point[0]/point[2])+info.k[2], info.k[4]*(point[1]/point[2])+info.k[5]
        return (int(round(u)), int(round(v))) if np.isfinite([u, v]).all() and 0 <= u < w and 0 <= v < h else None

    start, end = pixel(grasp_optical), pixel(selected.reference_optical)
    if start is not None:
        cv2.drawMarker(output, start, (80, 255, 80), cv2.MARKER_DIAMOND, 14, 2)
        if end is not None:
            cv2.arrowedLine(output, start, end, (80, 255, 80), 2, tipLength=.15)
    label = ('NOMINAL CAD' if mount_report['mode'] == 'nominal_cad' else 'MEASURED COLOR MOUNT')
    label += ' | alignment '+('CONFIRMED' if mount_report['alignment_verified'] else 'UNVERIFIED')
    lines = [label, 'green: hand grasp -> reference | correction base XYZ (mm): '+
             ', '.join(f'{x*1000:+.1f}' for x in correction)]
    for i, line in enumerate(lines):
        y = h-28+i*20
        cv2.putText(output, line, (5, y), cv2.FONT_HERSHEY_SIMPLEX, .43, (0, 0, 0), 3)
        cv2.putText(output, line, (5, y), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 180, 0), 1)
    return output, report
