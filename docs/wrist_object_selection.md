# Wrist object selection and grasp reference

After the [close-range depth check](wrist_depth_check.md), use this camera-only
preview to select the first test object and define where the hand should grasp.
It starts no robot backend, uses no robot TF/calibration and exposes no robot
action/service clients or motion command publishers. The approach and grasp
commands remain separate in the [hardware workflow](wrist_grasp_hardware.md).

## Start the preview

Rebuild the wrist package/image to install the new executable. Use the same
camera driver, aligned color/depth streams and ROS domain as the depth check.
In a sourced container terminal, with your copied hardware settings at `/config`:

```bash
ros2 launch lara5_wrist_grasp object_preview.launch.py approach_config_file:=/config/hardware.yaml
```

In another sourced terminal:

```bash
ros2 run rqt_image_view rqt_image_view /arm/wrist/selection/image
```

The preview reads the `wrist_approach.ros__parameters` section of that file. It
ignores arm/hand execution settings. White outlines the ROI, red outlines hand
exclusions, cyan marks selected depth pixels, amber marks the visible-surface
anchor, and magenta marks the proposed grasp reference. An off-screen reference
still appears as optical XYZ in the status. Selection failures keep the ROI
visible; invalid or stale camera input replaces the image with a warning.

```bash
ros2 topic echo /arm/wrist/selection/status --qos-reliability reliable --qos-durability transient_local --full-length
```

Require `state: selected`. `latest` includes selected pixel bounds/count, visible
surface XYZ, grasp-reference XYZ, depth spread and whether a support plane was
removed. Invalid/stale observations clear `latest`. This preview always reports
`read_only: true`, `motion_authorized: false` and `closure_ready: false`.

## Select one object without color

Place one stationary object in front of the open hand. Adjust normalized
`[left, top, right, bottom]` image coordinates live, for example:

```bash
ros2 param set /wrist_object_preview roi '[0.25, 0.20, 0.75, 0.80]'
ros2 param set /wrist_object_preview exclusion_rois '[0.0, 0.0, 0.20, 1.0, 0.80, 0.0, 1.0, 1.0]'
```

Those rectangles are examples, not a modeled hand silhouette. Cover every visible
finger/holder region with exclusions, or keep it outside the ROI. Fully contain
the visible object inside the remaining ROI, with a margin. Components touching
an ROI, image or exclusion boundary are discarded because cropping biases the
surface anchor. This is spatial exclusion, not semantic hand recognition.

The selector connects adjacent valid depth pixels when their depth differs by
at most `depth_connectivity_m` (default 8 mm). Background components reaching the
ROI boundary are discarded. Exactly one remaining component of at least
`min_pixels` must survive; multiple candidates fail instead of choosing the
nearest/largest. Depth must remain within the hardware configuration's range,
and the selected component's p95−p05 spread must meet `max_depth_spread`.
RGB is used only for display. Dark, gray and differently colored objects use
the same depth rule. No object height, upright assumption or size prior is used.

If a visible table connects to the object in depth, optionally mark a patch
containing only that table and enable plane removal:

```bash
ros2 param set /wrist_object_preview support_plane_roi '[0.20, 0.85, 0.80, 1.0]'
ros2 param set /wrist_object_preview remove_support_plane true
```

RANSAC fits that patch, requiring `plane_min_fraction` inliers within
`plane_distance_m`, and removes points close to the fitted plane before selection.
The patch must exclude hand and object surfaces. This is a supporting-plane fit,
not semantic table recognition. If no usable plane is visible, fitting falls back
to depth isolation; table detection is never mandatory. Leave removal disabled
when there is no suitable patch. Touching objects, missing depth and surfaces
without depth separation can still be unselectable; adjust the setup instead of
assuming a successful segmentation.

## Define the first object's grasp reference

The rule is explicit:

```text
surface_anchor = component-wise median of selected visible optical XYZ
grasp_reference = surface_anchor + surface_to_grasp_xyz
```

Color optical axes are X right, Y down, Z forward, with distances in metres.
This anchor is a visible-surface statistic. It does not reveal the object's
centre, back surface, orientation, suitable finger placement or hidden geometry.
Choose the first test object and intended hand orientation so the reference can
be checked independently against its dimensions and visible surface.

For example, a measured box viewed squarely at the full, symmetric front face,
with 60 mm thickness along optical Z, has a mid-thickness reference 30 mm behind
that face: `[0.0, 0.0, 0.03]`. This example assumes that mid-thickness placement is
the desired grasp and that the entire face is selected. It does not apply
automatically to cylinders, oblique faces, partial views or irregular objects.

```bash
ros2 param set /wrist_object_preview surface_to_grasp_xyz '[0.0, 0.0, 0.03]'
```

Inspect the marker and reported 3-D reference. Recheck the rule when the object,
view, orientation or visible surface changes. A zero offset explicitly targets
the visible median; it is not a default centre estimate. The template leaves
`grasp_reference_confirmed: false`. The approach can preview with that placeholder,
but refuses motion until the reference is verified and confirmed in its settings.

Live adjustments affect this read-only preview only. Copy the chosen selection
settings into your host copy of `hardware.yaml`; the preview never writes it.
Restart the hardware tasks to load them. Their detector uses the same selection
code and optical offset, then transforms the reference into robot-base coordinates.
The legacy `/arm/wrist/object_pose` topic now contains this grasp reference,
with identity orientation. `hand_to_grasp_xyz` separately describes the desired
grasp location relative to the hand; these two offsets serve different purposes.

Next use the [read-only CAD/color alignment preview](wrist_camera_alignment.md)
to check mounting geometry and the proposed correction against the actual assembly.

The [tracked approach](wrist_approach_tracking.md) now associates the original
surface/reference across bounded corrections and requires distinct-frame
acquisition/completion evidence. Exclusive approach-to-grasp coordination remains
a later plan step. This camera-only tool itself does not establish motion
readiness or physical retention.

## Offline validation

The wrist suite uses synthetic depth/color, isolated ROS camera topics and fake
native services/actions. It covers color independence, explicit offsets and frame
transforms, ambiguity, cropping, hand exclusion, optional/absent table planes,
depth faults, live settings, stale/repeated frames and the motion reference gate.
No robot commands or physical camera measurements are required by these tests.

On 2026-10-27, the updated package built in an isolated Humble container with
networking disabled and no USB access. All **70 wrist tests passed**, including
the unchanged depth-check suite and the new selection/preview cases. The installed
camera-only launch started and exited cleanly on SIGINT. These synthetic results
do not validate the physical mount, depth accuracy or the chosen object offset.
