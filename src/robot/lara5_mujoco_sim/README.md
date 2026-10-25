# Simulated D435 cameras (ROS 2 Humble / Docker)

The hardware setup uses two RealSense D435 cameras. The simulation provides two
**ideal RGB-D cameras**: an external camera for object localization and a camera
at the hand/wrist interface for grasp refinement. Both start with the
existing simulation launch. No RealSense USB device or driver is needed in MuJoCo.

The wrist camera now uses the supplied **tcp_cam_holder_v2 CAD holder** in the
shared robot URDF. Its parent is `right_wrist_camera_holder`. The photo-matched
left/right camera screws are used; the middle cable-tie screw is excluded. The
platform's 18.435-degree tilt determines the nominal camera direction. The
previous arbitrary setback/look-at mount is no longer the default.

The camera model uses the selected outer screw pair and nominal dimensions from
the vendored D435 description. CAD-to-wrist datum, mounting-side/image rotation
and optical offsets still need measurement; this is not calibrated extrinsics.
See [the mechanical audit and pose derivation](../../../docs/wrist_camera_holder.md).
The existing flange-to-hand joint and grasp kinematics are unchanged.

The representative D435 housing has four front windows and is approximately
90 × 25 × 25 mm. The holder mesh is in the shared URDF, while the housing is drawn
in MuJoCo. Both remain visual-only; no measured mass/inertia or collision
decomposition was supplied. Rendering and optical TF use one camera pose.

Static previews: [mount](../../../docs/images/wrist_cad_holder_mount.png) and
[wrist view](../../../docs/images/wrist_cad_holder_view.png). The green sphere is a QA
marker at the nominal grasp centre, not a successful object-grasp demonstration.
These views were rendered without stepping physics or commanding the robot.

## Configuration and coordinate frames

Edit [config/mujoco_sim.yaml](config/mujoco_sim.yaml). The two independent sections
are `external_camera` and `wrist_camera`:

| Setting | Meaning |
| --- | --- |
| `enabled` | Enable/disable the camera, its images and optical TF |
| `parent_frame` | `world` for the external camera; a retained robot link for a mounted camera |
| `position` | Optical centre XYZ in the **parent frame**, metres |
| `target` | Look-at point XYZ in the **same parent frame**, metres |
| `up` | Optional image-up direction in the parent frame; sets roll and must not be parallel to the viewing direction |
| `orientation_xyzw` | ROS optical-to-parent quaternion; when non-null it replaces `target`/`up`; current default is nominal CAD-derived |
| `width`, `height`, `fovy` | Image dimensions and vertical field of view in degrees |
| `rate` | Requested image rate in Hz; rendering speed may limit the actual rate |
| `frame_id` | Unique ROS optical frame name |
| `show_body` | Draw an approximate D435 housing |
| `housing_center_optical` | Housing centre in optical XYZ; separates the physical rear mounting face from the optical zero |
| `depth_min_m`, `depth_max_m` | Optional optical-axis working range in metres; invalid/out-of-range depth is zero |

The active Inspire model extends along hand-local `+Z`, with its palm facing local
`+Y`. The CAD holder uses those same axis directions, with its origin 11.5 mm
behind the hand base as a nominal assembly datum. Camera XYZ and quaternion are
expressed in that holder frame, even when the arm is rotated in world.

When you supply measured optical extrinsics, edit the wrist camera section of
the complete config file:

```yaml
wrist_camera:
  parent_frame: right_wrist_camera_holder
  position: [tx, ty, tz]                    # optical centre, metres in parent
  orientation_xyzw: [qx, qy, qz, qw]       # optical-to-parent unit quaternion
```

The values above are symbolic measurement placeholders, not a runnable config.
If the supplied pose is in `world`, also provide the holder pose at the same instant.
The fixed mount is `T_holder_camera = inverse(T_world_holder) * T_world_camera`.
Flange-relative measurements can instead use `parent_frame: lara5_flange`.

Alternatively, for translation `t` and rotation `R` (optical coordinates to parent
coordinates), keep `orientation_xyzw: null` and set:

```text
position = t
target   = t + 0.20 * R[:, 2]   # Any positive look distance is equivalent
up       = -R[:, 1]             # Optical +Y points down in an image
```

Convert bottom-screw or housing measurements into **optical-centre** extrinsics
first. ROS optical axes are `+X` right, `+Y` down, `+Z` forward. These differ from
MuJoCo's camera convention (`+X` right, `+Y` up, `-Z` forward); the geometry helpers
handle that conversion for both rendering and TF.

The TF chain is:

```text
world -> ... -> lara5_flange -> right_hand_base_link
                                   -> right_wrist_camera_holder
                                        -> sim_wrist_camera_optical_frame
world -> sim_external_camera_optical_frame
```

`robot_state_publisher` supplies moving robot transforms. The camera publisher
supplies only the fixed mount-to-optical transforms on `/tf_static`. The rendered
wrist camera is attached to that same MuJoCo body with the same local transform.
For future perception, transform wrist points using TF **at the image timestamp**.

## Image interfaces

| Camera | RGB | Depth | Calibration |
| --- | --- | --- | --- |
| External | `/sim/external_camera/color/image_raw` | `/sim/external_camera/depth/image_raw` | `/sim/external_camera/camera_info` |
| Wrist | `/sim/wrist_camera/color/image_raw` | `/sim/wrist_camera/depth/image_raw` | `/sim/wrist_camera/camera_info` |

Images use `rgb8` and `32FC1` respectively. Depth is optical-axis depth in **metres**.
Each camera's RGB, depth and CameraInfo share a frame and capture timestamp, and
depth is already registered to that camera's RGB pixels. The two cameras capture
independently; matching timestamps across cameras are not guaranteed. A rendering
worker per camera copies physics state under the simulator lock before rendering.

Defaults are 640 × 480 and 10 Hz. The wrist's 62° vertical FOV approximates the
D435 VGA depth view; the external camera retains 60°. These are simulation
settings, **not calibrated RGB/depth intrinsics**. The simulator does
not reproduce separate color/depth optical centres, stereo disparity, infrared
images, sensor noise or material-dependent depth holes. The wrist masks depth
outside the provisional 0.20–3.0 m working range to zero. This is a conservative
profile setting, not a complete simulation of close-range stereo behavior.
Do not infer real grasp-range depth reliability from these images.
Real D435 depth streams may also use a different encoding and unit scale; the
hardware perception adapter must normalize them.

With the CAD mount, the nominal grasp reference is about **0.129 m along the
optical axis**. It is inside the current 0.20 m depth cutoff, so depth there is
zero even when RGB sees the region. Do not move the camera away from the CAD
platform or silently lower this cutoff to imply reliable hardware depth. Validate
the actual sensor profile and working distance separately.

The approximately 90 × 25 × 25 mm housing is a visual placeholder based on the
D435 description already in `ws/src/realsense-ros/realsense2_description`.
It has no mass or collision geometry. The bracket visual is now in the shared
URDF; collision decomposition and measured mass/inertia remain separate work.

## Build and view the wrist feed live

From the repository root on the host:

```bash
docker build -f src/docker/Dockerfile -t lara5_hackathon:cad-camera-holder src
```

Use a new container from this image with the usual X11/display permissions and
mount. Rebuilding does not update an already running container. Do not disable
the camera YAML entries when testing the new image. The simulator launch now
selects **EGL for offscreen camera renderers** while the viewer retains its native
GLFW window, avoiding the previous concurrent GLFW/X11 camera initialization crash.
This selection is local to the simulator process and overrides a container-level
`MUJOCO_GL=glfw` setting:

```bash
# Inside a new container with the existing X11/display settings:
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_controller_manager controller_manager.launch.py \
  deployment_mode:=simulation sim_headless:=false rviz:=true sim_camera_backend:=egl
```

In another host terminal, open a live wrist image window alongside MuJoCo:

```bash
docker exec -it -e QT_X11_NO_MITSHM=1 lara5_cad_holder_gui bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /sim/wrist_camera/color/image_raw
```

The viewer's topic selector can switch between wrist/external RGB and depth
images. A stationary arm gives a stationary wrist view; when you execute a checked
arm trajectory, the wrist view follows the hand while the external view stays
fixed. MuJoCo also contains a named `wrist_camera` for its camera selection UI.

From another sourced container shell, inspect the streams and mount:

```bash
ros2 topic hz /sim/wrist_camera/color/image_raw
ros2 topic echo /sim/wrist_camera/camera_info --once
ros2 run tf2_ros tf2_echo right_wrist_camera_holder sim_wrist_camera_optical_frame
ros2 run tf2_ros tf2_echo world sim_wrist_camera_optical_frame
```

The first TF stays fixed relative to the holder. The world TF changes as the arm
moves. Use the existing plan-only workflow before executing an arm trajectory.

For repeated mount adjustments, copy a custom YAML into the host's mounted data
directory, edit it there, and restart the launch with:

```bash
ros2 launch lara5_controller_manager controller_manager.launch.py \
  deployment_mode:=simulation sim_headless:=false sim_config:=/data/my_sim.yaml
```

For CPU-only headless camera rendering, use `sim_headless:=true sim_camera_backend:=osmesa`.
Set `wrist_camera.enabled: false` to disable
the wrist stream, or `show_body: false` to hide just its visual housing.

## Grasping integration boundary

The independent [approach task](../lara5_object_approach/README.md) still consumes
a supplied object pose. The independent [grabbing task](../lara5_grasp/README.md)
still closes only the fingers. Neither consumes wrist images or has been changed
to perform visual alignment, final insertion, object retention verification or
automatic lifting by this camera work.

The [next-step grasping plan](../../../docs/grasping_next_steps.md) is documentation
for your review. Its steps will be applied only when explicitly requested.

## Verification

The following tests compile the model and evaluate kinematics/messages without
starting ROS nodes, rendering, or advancing physics:

```bash
python3 /ws/src/robot/lara5_mujoco_sim/test/test_wrist_camera.py -v
```

They check optical axes, moving-parent attachment, fixed external-camera pose,
missing/disabled mounts, publisher topics/TF, actual robot compilation and
projection of the provisional grasp region. Additional registered package tests
check calibrated quaternion input, depth-range masking and headless EGL RGB-D
rendering:

```bash
colcon test --packages-select lara5_mujoco_sim --event-handlers console_direct+
colcon test-result --test-result-base build/lara5_mujoco_sim/test_results --verbose
```

The viewer concurrency regression is opt-in and should use a private test display,
not the desktop session. Set `DISPLAY` to that display, `MUJOCO_GL=egl` and
`LARA5_CAMERA_VIEWER_TEST=1`, then run `test/test_camera_rendering.py` with pytest.
The test opens the viewer in a fresh process and confirms it stays open while both
camera threads render 40 frames, then shuts down cleanly. No ROS nodes, physics
steps or robot commands are involved.
