# Measured fixed camera

The confirmed measurement convention is relative to `lara5_root_link`, the arm's
mounting plate, along that frame's signed axes. It is not relative to the
pedestal `base_link` or to world axes.

- Translation: X = -0.555 m, Y = -0.450 m, Z = 0.900 m.
- Camera body roll = 0, pitch = +45 degrees downward, yaw = -15 degrees, so that the
  body is at roll 0, pitch 45, yaw 90 degrees in the world frame (the base is yawed 105).
- The measured camera point is treated as the color optical origin. The housing
  is an approximate visual; this measurement is not a calibrated optical pose.

`fixed_camera.xacro` defines the mount once in the shared robot description:

```text
world -> base_link -> mount_frame_neg_y -> lara5_root_link
                                             -> camera_1_link
                                                -> camera_1_color_optical_frame
                                                   -> sim_external_camera_optical_frame
```

The optical-frame conversion makes +Z point along the camera's viewing direction,
+X image-right and +Y image-down. In arm-base axes, the viewing direction is
`[sqrt(0.5), 0, -sqrt(0.5)]`. The arm's mounting plate is rotated about 105 degrees
around world Z, so the measurement cannot be added directly to world XYZ.

The simulator's external camera is coincident with `camera_1_color_optical_frame`;
it inherits the shared URDF mount rather than maintaining a second pose in YAML.
`robot_state_publisher` publishes the measured camera frames. Simulation publishes
only the separate `sim_external_camera_optical_frame` alias.

The saved `extrinsics.yaml` remains the previous calibration result for comparison.
Do not also publish its old `world -> camera_1_color_optical_frame` transform when
using this measured mount. For a launch that deliberately uses the old calibrated
camera broadcaster, disable this URDF mount with `fixed_camera:=false` at xacro
expansion.
