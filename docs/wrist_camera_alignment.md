# Wrist camera alignment from the CAD mounting estimate

This is the third step, after the [depth range check](wrist_depth_check.md) and
[object/reference selection](wrist_object_selection.md). It reuses the supplied
holder geometry and previews the resulting correction without commanding motion.
The existing approach and grasp commands remain separate.

## Shared mounting geometry

The nominal depth-optical pose lives in one existing source:
`lara5_mujoco_sim/config/mujoco_sim.yaml`, section `wrist_camera`. Both simulation
and the hardware resolver read it. The resolver loads only YAML; it does not
start or import the MuJoCo runtime. `nominal_mount_config_file: ''` selects that
installed file. An explicit file path can select a reviewed replacement of that
same configuration. The mounting assumptions remain documented in
[wrist_camera_holder.md](wrist_camera_holder.md).

The active robot description supplies the fixed flange-to-holder chain, including
the existing flange-to-hand rotation and holder datum. No flange/hand rotation
or spacer translation is copied into the hardware settings. The composition is:

```text
T_flange_color = T_flange_holder(URDF)
                 * T_holder_depth(shared CAD estimate)
                 * T_depth_color(RealSense driver TF)

p_base = T_base_flange(at image capture) * T_flange_color * p_color
```

`T_parent_child` maps child coordinates into parent coordinates. The CAD optical
origin is the depth imager. Depth aligned to color and color CameraInfo use the
color imager's optical frame. The driver's SDK `depth_to_color` point mapping
must therefore be inverted for the final factor above. The resolver obtains
that inverse directly with TF target=depth optical, source=color optical. It
does not guess the baseline or reuse simulation's co-located camera assumption.

For example, inspect the driver frames in a sourced camera-domain terminal:

```bash
ros2 run tf2_ros tf2_echo wrist_depth_optical_frame wrist_color_optical_frame
```

Match both configured frame names to the actual driver. Aligned RGB/depth and
CameraInfo must all identify the configured color optical frame. Missing driver
TF fails the preview; there is no identity fallback. The vendored driver can
publish unity when sensor extrinsics are unavailable, so a zero D435 color/depth
baseline is also rejected. No nominal transform is broadcast into driver TF.

Default hardware settings now select `camera_mount_mode: nominal_cad`, with
`camera_calibration_confirmed: false`. The identity placeholders were removed
from the template. The confirmation flag records an operator's independent
verification of the complete mounting alignment; composing CAD with driver
extrinsics does not itself constitute that verification.

## Read-only alignment preview

Rebuild the package/image as in the [hardware guide](wrist_grasp_hardware.md).
Use the same real camera streams, robot description and measured robot TF as
that workflow. An existing backend may supply these; keep task execution disabled
while checking alignment. The standalone launch itself starts no backend, arm
task, hand task or planner, and has no robot service/action clients.

In a sourced container terminal on that ROS domain:

```bash
ros2 launch lara5_wrist_grasp alignment_preview.launch.py approach_config_file:=/config/hardware.yaml
```

In another sourced terminal:

```bash
ros2 run rqt_image_view rqt_image_view /arm/wrist/alignment/image
```

The overlay shows the selected object pixels, visible-surface anchor, intended
grasp reference and projected hand grasp location. Cyan marks selection, amber
the surface anchor, magenta the reference, and a green diamond/arrow shows the
hand grasp location and correction toward the reference. Optical projection
cannot show a marker behind the camera or outside the image; its XYZ and the
full 3-D correction remain available in status. In particular, an optical-axis
correction can have little visible arrow length.

The bottom label explicitly distinguishes `NOMINAL CAD` from `MEASURED COLOR
MOUNT`, and `UNVERIFIED` from operator `CONFIRMED` alignment. The correction is
reported in robot-base XYZ millimetres. The green hand marker is a geometry
prediction to compare with the assembly, not a camera detection of the fingers.

```bash
ros2 topic echo /arm/wrist/alignment/status --qos-reliability reliable --qos-durability transient_local --full-length
```

Require `state: alignment_preview_ready` with a non-null `latest`. It reports:

- The selected object region and grasp-reference rule from step two.
- `camera_mount`, including source file, driver transform direction, composed
  `T_flange_color`, and `alignment_verified`.
- `grasp_reference_base_m`, `current_hand_grasp_base_m`, `correction_base_m`,
  `proposed_flange_xyz_m`, source timestamp, and reference/base verification flags.
- `read_only: true`, `motion_authorized: false`, `closure_ready: false`, and
  `path_checked: false`, including when all configuration flags are confirmed.

Fresh images may wait briefly for their exact capture-time TF. Latest robot TF
is never substituted. Missing geometry, stale camera data, repeated source frames
or a missing driver baseline clear `latest`; invalid/stale data replace a healthy
image with a warning. This preview does not check the correction envelope,
collisions, native IK, hand forces or object retention.

The selection settings remain adjustable live on `/wrist_alignment_preview`:

```bash
ros2 param set /wrist_alignment_preview roi '[0.25, 0.20, 0.75, 0.80]'
ros2 param set /wrist_alignment_preview exclusion_rois '[0.0, 0.0, 0.20, 1.0, 0.80, 0.0, 1.0, 1.0]'
```

These rectangles are examples. Save verified settings to your host hardware
configuration, then restart tasks to load them. Mount source and confirmation
settings cannot change live on this preview.

## Check the physical assembly

Compare the actual holder datum, spacer stack, camera mounting side and image
clocking with the documented CAD assumptions. Check depth independently at the
intended range, then use a dimensioned target with known reference locations to
compare reported optical/base XYZ against independent measurements. Check more
than one visible location so translation, rotation and axis-sign errors are not
hidden by a single apparent match. Keep the manually chosen grasp orientation
and verify the separate hand-to-grasp and surface-to-grasp offsets as well.

Only mark `camera_calibration_confirmed: true` after the complete alignment is
accurate enough for the configured correction tolerance on the actual assembly.
The software cannot establish this from CAD or a visually plausible overlay.
The mount source still reports `nominal_cad` after confirmation, preserving its
provenance. Base and grasp-reference confirmation remain separate requirements.

If the nominal assembly does not match, use an independently measured complete
color-optical mounting transform instead: set `camera_mount_mode: measured_color`
and supply `camera_to_flange_xyz` and `camera_to_flange_xyzw` as
`T_flange_color_optical`, in metres and ROS xyzw order. This replaces the entire
CAD/driver composition; the driver optical transform is not applied twice.
Existing measured configurations must explicitly select `measured_color`.
Non-default measured transform values with `nominal_cad` are rejected to avoid
silently using a different mounting source.

The existing `/arm/wrist/preview` command now also allows unconfirmed alignment.
It uses the same camera resolver and retains its stationary/open-hand, freshness,
capture-time TF/native FK, envelope and native path checks. `preview_ready` does
not establish calibration or closure readiness; status includes the alignment
confirmation flags. `/arm/wrist/approach` still rejects unconfirmed camera/base
alignment before sending any goal. No settings are automatically confirmed by
either preview.

## Offline verification

Tests cover the real URDF's documented flange-to-depth origin, non-identity
optical rotation and transform inversion, missing/unity driver TF, measured
replacement, read-only ROS interfaces, capture-time TF arrival, invalidation and
the nominal path through the existing approach preview. Tests use synthetic camera
data and fake services; physical assembly alignment remains to be verified.

On 2026-10-27 the updated package built in an isolated Humble container with no
network or USB access. All **95 wrist tests passed**, with no errors, failures or
skips. The installed alignment launch started and exited cleanly on SIGINT. A
synthetic 640×480 overlay was also rendered and visually inspected. No physical
camera measurements or real robot commands were performed.
