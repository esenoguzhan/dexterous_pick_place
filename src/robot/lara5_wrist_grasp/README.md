# Wrist-only hardware approach and grasp

See the [hardware guide and ordered terminal commands](../../../docs/wrist_grasp_hardware.md).

First run the [camera-only depth feasibility check](../../../docs/wrist_depth_check.md)
at the manually chosen start and final grasp positions:

```bash
ros2 launch lara5_wrist_grasp depth_check.launch.py
```

This independent launch needs only RGB-D/CameraInfo, with no robot controller,
TF, color/size prior or calibration confirmation. It publishes an ROI overlay and
separate start/grasp depth reports without enabling either motion task.

Next inspect the [color-independent object selection and explicit grasp reference](../../../docs/wrist_object_selection.md):

```bash
ros2 launch lara5_wrist_grasp object_preview.launch.py approach_config_file:=/config/hardware.yaml
ros2 run rqt_image_view rqt_image_view /arm/wrist/selection/image
```

This camera-only preview shares the approach's depth selector, supports live ROI
and hand-exclusion rectangles, and displays the surface anchor and proposed grasp
reference. A measured optical-frame offset defines finger placement; an object's
hidden centre is never inferred. Save verified settings to the hardware config.

Then inspect the [CAD-to-color camera alignment](../../../docs/wrist_camera_alignment.md)
with the camera, robot description and measured robot TF already available:

```bash
ros2 launch lara5_wrist_grasp alignment_preview.launch.py approach_config_file:=/config/hardware.yaml
ros2 run rqt_image_view rqt_image_view /arm/wrist/alignment/image
```

This read-only launch has no robot command clients. It composes the shared CAD
depth-optical mount, fixed URDF chain and driver's depth/color optical TF, and
shows the selected region, grasp reference and proposed correction. Unverified
alignment is labeled explicitly and does not authorize motion.

The motion-task launch is separate:

```bash
ros2 launch lara5_wrist_grasp hardware.launch.py config_file:=/config/hardware.yaml
```

Tasks are idle at startup and execution defaults to disabled. The default launch
uses an existing real backend; `start_backend:=true` explicitly starts it.
A separate RealSense driver supplies aligned RGB-D. Verify the camera alignment,
grasp reference and raw physical force thresholds in a copy of `config/hardware.yaml`.

Independent Empty commands: `/arm/wrist/preview`, `/arm/wrist/approach`,
`/arm/wrist/cancel`, `/arm/grasp/target`, `/arm/grasp/cancel`. Preview sends no
arm/hand command. Approach holds the manually chosen orientation and makes bounded
wrist corrections. Grasp uses stamped physical hand force/angle feedback. No lift,
transfer or automatic opening is performed.

The [tracked approach](../../../docs/wrist_approach_tracking.md) locks one visible
surface/reference in base coordinates and checks it across corrections. Initial
acquisition and final completion require at least three distinct source frames
spanning 0.15 seconds. Loss, ambiguity or changed camera geometry invalidates the
request. Choose the hand orientation manually; the executor corrects translation.

The [exclusive handoff](../../../docs/wrist_closure_handoff.md) passes each verified
approach to one hand closure through the internal `/arm/wrist/closure_handoff`
service. The claim consumes the result. Arm commands are refused until that
closure releases it, and a new approach is required before another closure.
Neither task can stop motion from the robot UI; follow the guide's robot-UI-to-ROS
handoff before each ROS phase.

Choose the hand's raw force settings with the read-only
[force check](../../../docs/hand_force_check.md) before confirming them in
`config/hardware.yaml`. Closure success needs several new physical measurements,
and the camera is not used once closure starts.
