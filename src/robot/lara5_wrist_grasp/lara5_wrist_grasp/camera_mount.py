"""Shared CAD/URDF/driver camera geometry for alignment preview and approach.

T_parent_child maps child coordinates into parent coordinates. The CAD pose is
the DEPTH optical origin. Aligned depth is expressed in COLOR optical coordinates.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from tf2_ros import TransformException

from lara5_object_approach.grasp_geometry import origin, pose_components, transform


DEFAULTS = dict(camera_mount_mode='nominal_cad', nominal_mount_config_file='',
    camera_calibration_confirmed=False, depth_optical_frame='wrist_depth_optical_frame',
    camera_to_flange_xyz=[0., 0., 0.], camera_to_flange_xyzw=[0., 0., 0., 1.])


def fixed_chain(description, parent, child):
    try:
        root = ET.fromstring(description)
    except ET.ParseError as error:
        raise ValueError('Invalid robot description for camera mounting chain') from error
    links = {link.get('name') for link in root.findall('link')}
    if parent not in links or child not in links:
        raise ValueError(f'Camera mounting chain requires URDF links {parent} and {child}')
    joints = {}
    for joint in root.findall('joint'):
        name = joint.find('child').get('link')
        if name in joints:
            raise ValueError('Ambiguous camera mounting chain in robot description')
        joints[name] = joint
    result, seen = np.eye(4), set()
    while child != parent:
        joint = joints.get(child)
        if child in seen or joint is None or joint.get('type') != 'fixed':
            raise ValueError('Camera holder must have a fixed URDF chain to lara5_flange')
        seen.add(child)
        result = origin(joint.find('origin')) @ result
        child = joint.find('parent').get('link')
    return result


def stamped_matrix(stamped):
    t = stamped.transform
    return transform([t.translation.x, t.translation.y, t.translation.z],
                     [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])


def pose_report(matrix):
    xyz, xyzw = pose_components(matrix)
    return dict(xyz_m=list(map(float, xyz)), xyzw=list(map(float, xyzw)))


class CameraMount:
    def __init__(self, config):
        self.config = config
        mode = config['camera_mount_mode']
        if mode not in ('nominal_cad', 'measured_color'):
            raise ValueError('camera_mount_mode must be nominal_cad or measured_color')
        if not config['optical_frame']:
            raise ValueError('optical_frame must name the aligned COLOR optical frame')
        self.source = None
        if mode == 'measured_color':
            self.measured = transform(config['camera_to_flange_xyz'], config['camera_to_flange_xyzw'])
        else:
            if (not np.array_equal(config['camera_to_flange_xyz'], DEFAULTS['camera_to_flange_xyz'])
                    or not np.array_equal(config['camera_to_flange_xyzw'], DEFAULTS['camera_to_flange_xyzw'])):
                raise ValueError('Measured camera_to_flange values require camera_mount_mode: measured_color; '
                                 'do not silently mix measured and nominal sources')
            if not config['depth_optical_frame'] or config['depth_optical_frame'] == config['optical_frame']:
                raise ValueError('Distinct depth and color optical frame names are required for the CAD mount')
            self.source = str(Path(config['nominal_mount_config_file'] or
                Path(get_package_share_directory('lara5_mujoco_sim'))/'config/mujoco_sim.yaml'))
            with open(self.source, encoding='utf-8') as source:
                camera = yaml.safe_load(source)['wrist_camera']
            if camera.get('model') != 'realsense_d435' or camera.get('parent_frame') != 'right_wrist_camera_holder':
                raise ValueError('Nominal source must describe the CAD holder-mounted D435 depth optical pose')
            self.parent = camera['parent_frame']
            self.holder_depth = transform(camera['position'], camera['orientation_xyzw'])

    def compose(self, description, depth_color=None):
        c = self.config
        report = dict(mode=c['camera_mount_mode'],
            alignment_verified=bool(c['camera_calibration_confirmed']),
            color_optical_frame=c['optical_frame'], nominal_source_file=self.source)
        if c['camera_mount_mode'] == 'measured_color':
            mount = self.measured.copy()
            report['source'] = 'measured_T_flange_color_optical'
        else:
            if depth_color is None:
                raise ValueError('RealSense depth-to-color optical TF is required; no identity fallback')
            depth_color = np.asarray(depth_color, dtype=float)
            if (depth_color.shape != (4, 4) or not np.isfinite(depth_color).all()
                    or not np.allclose(depth_color[3], [0, 0, 0, 1])
                    or not np.allclose(depth_color[:3, :3].T @ depth_color[:3, :3], np.eye(3), atol=1e-6)
                    or not np.isclose(np.linalg.det(depth_color[:3, :3]), 1., atol=1e-6)):
                raise ValueError('Driver optical TF must be a finite rigid transform T_depth_color')
            if np.linalg.norm(depth_color[:3, 3]) < 1e-6:
                # The vendored RealSense driver can publish unity when sensor
                # extrinsics are unavailable. D435 color/depth origins differ.
                raise ValueError('D435 driver optical TF has no color/depth baseline; unity fallback is unusable')
            mount = fixed_chain(description, 'lara5_flange', self.parent) @ self.holder_depth @ depth_color
            report.update(source='nominal_CAD_depth_pose_plus_driver_optical_TF',
                nominal_optical_origin='depth', depth_optical_frame=c['depth_optical_frame'],
                driver_T_depth_color=pose_report(depth_color))
        report['T_flange_color'] = pose_report(mount)
        return mount, report

    def resolve(self, description, buffer, stamp):
        if self.config['camera_mount_mode'] == 'measured_color':
            return self.compose(description)
        c = self.config
        try:
            # lookup(target=depth, source=color) maps COLOR points into DEPTH.
            # This is the inverse of the SDK's depth_to_color point mapping.
            stamped = buffer.lookup_transform(c['depth_optical_frame'], c['optical_frame'], stamp)
        except TransformException as error:
            raise ValueError('Missing driver depth-to-color optical TF: '
                             f"{c['depth_optical_frame']} <- {c['optical_frame']}") from error
        if stamped.header.frame_id != c['depth_optical_frame'] or stamped.child_frame_id != c['optical_frame']:
            raise ValueError('Driver optical TF returned unexpected frame direction')
        return self.compose(description, stamped_matrix(stamped))
