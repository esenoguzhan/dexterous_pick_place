# Wrist camera holder: CAD and mounting-photo integration

The supplied directory was `/home/ercankacmaz/Downloads/tcp_cam_holder_v2 2`.
The complete file audit is [camera_holder_cad_audit.json](camera_holder_cad_audit.json).
No files in Downloads were changed.

## What was inspected

- The FreeCAD document, its 25 design objects, placements, attachments, sketches,
  hole settings, reference part `NR-25001064` and final body `Cam_Extension`.
- All 114 supplied files. The 111 extracted document files match the FCStd archive
  byte-for-byte. Colour/GUI records were inventoried; geometric records and
  mechanical feature metadata were inspected for placement.
- All 35 BREP records: 21 contain readable shapes and 14 are empty internal or
  suppressed-feature records. The audit records these empty records explicitly.
- The STL and 3MF exports. Both use millimetres and agree on bounds; the 3MF build
  transform is identity. The STL has 34,492 triangles and is imported unchanged
  with a `0.001` scale into the metre-based URDF.
- The supplied mounting photo. Only the outer left/right screws constrain the
  camera. The middle screw/zip tie is cable retention and is excluded.

The active CAD holes are **3.0 mm diameter**, despite 3.5 mm sketch circles; the
final Hole feature overrides the sketch diameter. Hole rows are 5 mm apart on
the tilted surface, with repeated X positions spaced by 4.75 mm. The selected
camera pair spans 45 mm.

## Mechanical placement

The photo is interpreted as the outer left/right pair on the row nearest the
plate's free edge. Its CAD coordinates, in millimetres, are:

```text
left  = [-22.5, 95.5131670195, 6.8377223398]
right = [ 22.5, 95.5131670195, 6.8377223398]
```

Their midpoint locates the nominal camera rear-face centre. It is not an optical
origin. The CAD platform normal is:

```text
n = [0, -0.3162277660, 0.9486832981]
tilt = atan(1/3) = 18.4349488 degrees
```

The new fixed URDF link is `right_wrist_camera_holder`, a child of
`right_hand_base_link`. Its nominal translation is `[0, 0, -0.0115]` m and its
rotation is identity. This places the CAD tool-side annular face at z=11.5 mm
on the existing hand-base plane, with CAD +Y on the palm side. The original
flange-to-hand rotation remains -90 degrees around Z; hand FK and the grasp TCP
calculation are unchanged.

**Datum assumption:** the supplied CAD and one mounting photo do not give a
measured transform to the installed robot's flange frame or identify any actual
spacer stack. This nominal tool-side-face alignment preserves the existing hand
mount. A measured flange/holder transform can replace it later. It must not be
treated as calibrated mechanical alignment.

The holder is the supplied mesh, rather than an approximate box. It is visual
geometry only. No density, measured inertia or collision decomposition was
provided; taking its convex hull would fill the wrist opening incorrectly.

## Nominal D435 optical pose

The housing dimensions/nominal depth offset come from the already vendored
`ws/src/realsense-ros/realsense2_description/urdf/_d435.urdf.xacro`:

- Housing width/height/depth: 90 / 25 / 25.05 mm.
- Depth optical origin: 17.5 mm laterally from the housing centre.
- Depth zero: 4.3 mm behind the front plate, hence 20.75 mm from the rear face.

For this nominal assembly, the front points along the positive platform normal
toward the finger region. Image right is negative CAD X and image down is the
negative platform tangent. These mounting-side/image-clock assumptions are not
determined by an optical calibration in the supplied files.

```text
p_holder_optical = midpoint(left, right)/1000
                   + 0.02075*n + [0.0175, 0, 0]

p_holder_optical = [0.0175, 0.0889514409, 0.0265229008] m
q_holder_optical = [0, -0.1601822430, 0.9870874576, 0]  # ROS xyzw
```

Under the stated nominal URDF datums, this gives optical XYZ relative to:

| Parent | XYZ, metres |
| --- | --- |
| `right_wrist_camera_holder` | `[0.017500, 0.088951, 0.026523]` |
| `right_hand_base_link` | `[0.017500, 0.088951, 0.015023]` |
| `lara5_flange` | `[0.088951, -0.017500, 0.015023]` |

The optical quaternion replaces the former look-at orientation. Rendering and
the optical TF read the same configured pose. The representative housing's rear
face is positioned on the selected mounting pair; its physical centre is offset
from the optical origin instead of pretending the two are identical.

## What changed in ROS

```text
lara5_flange -> right_hand_base_link -> right_wrist_camera_holder
                                             -> sim_wrist_camera_optical_frame
```

`robot_state_publisher` publishes the holder's fixed joint on `/tf_static`.
The simulator publishes the holder-to-optical transform on `/tf_static`.
The existing image topics are unchanged:

```text
/sim/wrist_camera/color/image_raw
/sim/wrist_camera/depth/image_raw
/sim/wrist_camera/camera_info
```

No approach/grasp command topics, arm trajectories, finger closure settings,
perception selection or camera-driven control were added or changed.

## Static rendering and near-range depth

The [mount preview](images/wrist_cad_holder_mount.png) and
[wrist-camera view](images/wrist_cad_holder_view.png) use the real robot model's
forward kinematics with no physics integration or motion commands. The green
sphere marks the planned grasp region; it is not a grasped object.

That reference projects at approximately pixel `[450.8, 119.2]` in the 640×480
image, with optical-axis distance **0.1293 m**. It is off-centre because the CAD
orientation is preserved. The render shows the open hand and the reference marker;
actual object visibility will depend on the executed posture and finger occlusion.

The current depth working-range cutoff is still 0.20 m. The close grasp reference
therefore has zero published depth even when it appears in RGB. The nominal
mount was not moved and the cutoff was not relaxed to hide this limitation.
Hardware depth behavior at this distance must be measured for the chosen profile.

## Run the updated simulation UI

Older running containers keep their previous installed models. Start a new one
from the new image. On the host:

```bash
cd ~/Documents/hackathon/neurahackathon
docker build -f src/docker/Dockerfile -t lara5_hackathon:cad-camera-holder src
xhost +SI:localuser:root
docker run --rm -it --name lara5_cad_holder_gui --network host --ipc host \
  -e ROS_DOMAIN_ID=42 -e DISPLAY="$DISPLAY" -e QT_QPA_PLATFORM=xcb \
  -e QT_X11_NO_MITSHM=1 -e MUJOCO_GL=glfw -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  lara5_hackathon:cad-camera-holder bash
```

Inside that container:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_controller_manager controller_manager.launch.py deployment_mode:=simulation sim_headless:=false rviz:=true hands:=on safety:=on sim_camera_backend:=egl
```

Stop previous simulation launches on ROS domain 42 first. Orbit/zoom MuJoCo around
the wrist to inspect the ring, extension and camera. The holder visual is also
available to RViz through the shared robot description.

For the wrist RGB feed, open a second host terminal:

```bash
docker exec -it lara5_cad_holder_gui bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /sim/wrist_camera/color/image_raw
```

To inspect the pose in another sourced container shell:

```bash
ros2 run tf2_ros tf2_echo lara5_flange sim_wrist_camera_optical_frame
```

This update does not issue any arm/grasp command on startup. Stop the launches
with Ctrl+C, exit the container shells and then revoke the temporary X11 grant:

```bash
xhost -SI:localuser:root
```

## Validation completed

The `lara5_hackathon:cad-camera-holder` Humble image built all 14 robot packages.
Its approach/grasp/camera checks passed **84 tests**, with one opt-in viewer test
skipped. Tests cover the supplied mesh dimensions and scale, nominal screw-pair
alignment, housing rear-face placement, the actual URDF, moving wrist attachment,
optical TF/rendering agreement and the existing grasp-frame behavior.

An isolated headless simulation also published external RGB plus wrist RGB,
depth and CameraInfo. A read-only subscriber received three samples of each and
confirmed both `right_hand_base_link -> right_wrist_camera_holder` and
`right_wrist_camera_holder -> sim_wrist_camera_optical_frame` on `/tf_static`.
No arm-approach or grasp commands were sent. The temporary test container was
removed afterward.
