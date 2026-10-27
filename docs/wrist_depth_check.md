# Wrist depth feasibility check — camera only

This is the first check before wrist-guided approach and grasping: determine
whether the D435 supplies usable depth at the manually chosen starting and final
hand positions. It does not start the robot backend, send arm/hand commands,
change camera settings, or enable approach/grasp execution. No object color,
object dimensions, robot TF, or confirmed flange calibration is required.

The CAD reference grasp region is about **0.1293 m along the depth optical axis**.
The approach currently accepts depth from **0.20 to 0.80 m**. These are different
things: the diagnostic measures positive raw depth below 0.20 m too, then reports
whether the approach configuration would reject it. The CAD reference is context,
not a measured distance to the object's visible surface or a projected image point.

## What to measure

Use your test object and keep the hand open. Position the arm from the robot UI.
Select a small ROI completely inside a visible object surface, excluding fingers,
table, silhouette edges and background. Check the starting position first, then
the intended final grasp position using your normal manual positioning procedure.
Keep the camera and object stationary throughout each three-second recording.
The diagnostic cannot verify robot stationarity or which surface you selected.

Observe the view along the intervening manual movement as well: two successful
endpoint measurements do not establish continuous visibility between them. If the
object becomes occluded or has no valid depth at the intended final position,
record that result; do not move closer simply to make the test pass.

The overlay uses **red for missing depth**, **amber for valid depth outside the
approach range**, and **cyan for valid depth inside it**. Color is only an overlay;
the object's own color is not used to select or measure it.

## Terminal 1 — container, diagnostic build and camera

These commands use the existing `lara5_hackathon:wrist-hardware-camera` image and
compile only the updated wrist package inside a new disposable container. This
avoids rebuilding librealsense. The repository source is mounted read-only.
Connect the D435 to the PC first; do not run another camera driver for that device.

On the host:

```bash
cd ~/Documents/hackathon/neurahackathon
xhost +SI:localuser:root
docker run --rm -it \
  --name lara5_wrist_depth_check \
  --network host --ipc host \
  -e ROS_DOMAIN_ID=74 -e ROS_LOCALHOST_ONLY=0 \
  -e DISPLAY="$DISPLAY" -e QT_QPA_PLATFORM=xcb \
  -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev/bus/usb:/dev/bus/usb \
  --device-cgroup-rule='c 189:* rwm' \
  --mount "type=bind,source=$PWD/src/robot/lara5_wrist_grasp,target=/diagnostic_src,readonly" \
  lara5_hackathon:wrist-hardware-camera bash
```

Inside the container:

```bash
cd /ws
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
cp -a /diagnostic_src/. /ws/src/robot/lara5_wrist_grasp/
colcon build --packages-select lara5_wrist_grasp
source /ws/install/setup.bash
source /camera_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=cameras camera_name:=wrist serial_no:=_262722075125 enable_color:=true enable_depth:=true enable_infra1:=false enable_infra2:=false rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=848x480x30 align_depth.enable:=true enable_sync:=true pointcloud.enable:=false wait_for_device_timeout:=10.0
```

Leave this running. If the existing image is unavailable, first build both images
using the [hardware guide](wrist_grasp_hardware.md#build-on-the-ubuntu-host).

## Terminal 2 — read-only diagnostic

```bash
docker exec -it lara5_wrist_depth_check bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_wrist_grasp depth_check.launch.py
```

This launch starts only `wrist_depth_check`. It reads the installed
`config/hardware.yaml` for the approach's minimum/maximum depth; it does not launch
`hardware.launch.py`. To compare a custom approach YAML, copy it into the container
and pass `approach_config_file:=/path/to/hardware.yaml`. The report records the file
and effective range. Neither that file nor the approach's live parameters change.

## Terminal 3 — visual ROI and depth display

```bash
docker exec -it lara5_wrist_depth_check bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
source /camera_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /arm/wrist/depth_check/image
```

The outlined box is the measured ROI. By default it covers the central 10% of
image width and height. It is not automatically aligned with the CAD grasp point.
Use Terminal 4 to move it onto your object; the grasp point may be off-center.

## Terminal 4 — configure ROI and record both positions

```bash
docker exec -it lara5_wrist_depth_check bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 topic echo /arm/wrist/depth_check/status --qos-reliability reliable --qos-durability transient_local --once --full-length
```

Set `[left, top, right, bottom]` as normalized image coordinates from 0 to 1.
This example selects x=60–70%, y=30–40%; replace it to match the object in your view:

```bash
ros2 param set /wrist_depth_check roi '[0.60, 0.30, 0.70, 0.40]'
```

If you independently measured the optical-axis distance to this surface patch,
you can enter it in metres. A slanted tape distance or housing-to-object distance
is not directly the optical-axis distance. Leave it at `0.0` if unknown:

```bash
ros2 param set /wrist_depth_check expected_surface_depth_m 0.0
```

At the stationary **starting position**, record:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/wrist/depth_check/capture_start std_msgs/msg/Empty '{}'
sleep 4
ros2 topic echo /arm/wrist/depth_check/report --qos-reliability reliable --qos-durability transient_local --once --full-length
```

Check `active_capture: null` and `recordings.start`. If recording is still active,
wait for the terminal result. Now manually position at the intended **final grasp
position**. Reposition the ROI if necessary and update/reset the optional reference
distance; the earlier distance will not apply after moving the camera. Then:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/wrist/depth_check/capture_grasp std_msgs/msg/Empty '{}'
sleep 4
ros2 topic echo /arm/wrist/depth_check/report --qos-reliability reliable --qos-durability transient_local --once --full-length
```

The retained report contains both named recordings, each with its ROI and capture
time. A repeated trigger replaces only that position's recording. ROI/reference
changes are rejected while recording. No target identity is inferred across the
two positions.

After both captures finish, save the JSON directly on the host in another terminal:

```bash
docker exec lara5_wrist_depth_check bash -c 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 topic echo /arm/wrist/depth_check/report --field data --qos-reliability reliable --qos-durability transient_local --once --full-length' | sed '/^---$/d' > wrist_depth_report.json
```

## Interpret the results

| Result | Meaning and next decision |
| --- | --- |
| `quality_criteria_met: true`, `approach_range_compatible: true` | The selected surface patch meets the configured depth stability/coverage checks and current range. Confirm actual geometry/accuracy before later motion testing. |
| Quality true, range false, median near 0.13 m | The camera reports stable close depth, but the current approach cutoff excludes it. This is evidence for reviewing the cutoff later; the diagnostic does not lower it. |
| `missing_depth_in_roi` | Insufficient positive finite depth. RGB visibility alone does not resolve it. |
| `mixed_or_sloped_surfaces_in_roi` | Patch contains a broad depth distribution. Check for background, hand, edges or a steeply sloped surface. |
| `depth_or_scene_not_stable` | Frame-median depth varies too much; inspect scene motion and depth behavior. |
| `independent_distance_mismatch` | Median differs from the optional independently measured distance beyond the configured tolerance. |
| Incomplete capture, stale/repeated timestamps or timeout | No valid recording; correct the stream and capture again. |

Default criteria: at least 30 distinct source frames spanning three seconds,
at least 80% valid ROI pixels in every frame, spatial p95−p05 spread at most
20 mm, and p95−p05 variation of frame medians at most 5 mm. These are configurable
diagnostic criteria, not D435 specifications or a calibration certificate. Stable
wrong depth can pass stability tests. A background patch can also pass; verify the
ROI visually. Each report keeps `accuracy_verified`, `object_identity_verified`,
`robot_stationary_verified`, `closure_ready`, and `motion_authorized` false.

The geometry check at this step is the comparison between the documented nominal
grasp distance and configured depth range. Full CAD-to-color optical integration,
object selection, fine-approach changes, and physical grasp changes remain later
steps. This launch has no robot action or service clients and no motion publishers.

## Validation

On 2026-10-27, the wrist package built in an isolated Humble container with no
network or USB access. All **37 tests passed**, including 16 diagnostic tests for
close depth, missing/unstable depth, stream alignment, repeated timestamps,
disconnection, both capture commands, ROI changes and custom comparison ranges.
The installed standalone launch started and exited cleanly on SIGINT.
No D435 was enumerated on the host during this implementation, so real start/grasp
depth measurements remain pending. Synthetic test results are not sensor validation.

Stop the diagnostic, viewer and camera with Ctrl+C in their respective terminals,
exit the container shells, then revoke the display grant on the host:

```bash
xhost -SI:localuser:root
```
