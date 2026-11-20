# Wrist-only hardware approach and grasp (ROS 2 Humble)

Manually position the open hand near one stationary depth-isolated object, preview the
wrist-derived final approach, execute it, then explicitly trigger hand closure.
The port uses depth selection, explicit grasp-reference geometry, native IK and staged finger
closure. It commands no global pregrasp, lift, transfer, release or automatic
opening. The current D435 is connected to the Ubuntu PC, serial **262722075125**.

Before motion testing, use the [camera-only close-range depth check](wrist_depth_check.md).
It starts no backend and measures raw depth at manually positioned start/grasp
locations, including depths below the approach's current cutoff. It does not
require the calibration/force configuration below or change any execution gates.

## Interfaces and behavior

| Interface | Type | Behavior |
| --- | --- | --- |
| `/arm/wrist/preview` | `std_msgs/Empty` | Detect and plan; no arm/hand commands |
| `/arm/wrist/approach` | `std_msgs/Empty` | Re-observe and execute bounded final corrections |
| `/arm/wrist/cancel` | `std_msgs/Empty` | Cancel arm correction, confirm terminal result |
| `/arm/wrist/status` | `std_msgs/String` | Retained JSON state, closure readiness and handoff owner |
| `/arm/wrist/closure_handoff` | `lara5_safety_msgs/ClosureHandoff` | Internal service: hand task claims one verified approach; arm commands blocked until release |
| `/arm/wrist/object_pose` | `geometry_msgs/PoseStamped` | Explicit grasp reference in `lara5_root_link`; identity orientation, yaw not estimated |
| `/arm/wrist/flange_target` | `geometry_msgs/PoseStamped` | Proposed final flange pose in robot-base coordinates |
| `/arm/wrist/alignment/image` | `sensor_msgs/Image` | Standalone read-only CAD/color alignment overlay |
| `/arm/wrist/alignment/status` | `std_msgs/String` | Standalone geometry provenance, reference and proposed correction |
| `/arm/grasp/target` | `std_msgs/Empty` | Explicit physical hand closure after verified wrist approach |
| `/arm/grasp/cancel` | `std_msgs/Empty` | Cancel closure and request measured hold |
| `/arm/grasp/status` | `std_msgs/String` | Motion result and physical force evidence; retention unverified |
| `/right_hand/measured_state` | `sensor_msgs/JointState` | Source-stamped physical hand radians and raw actuator forces |
| `/arm/grasp/force_check/*` | `std_msgs/Empty`, `std_msgs/String` | Standalone read-only force recordings and settings check; sends no commands |

Approach preserves the manually chosen flange orientation. Default corrections
are at most **1 cm**, with requested Cartesian speed **5 mm/s** and joint cap
0.15 rad/s. Initial correction is limited to 8 cm, displacement to 10 cm,
cumulative travel to 12 cm and iterations to 20. Each correction is planned
through native NEURA IK and the existing safety filter. Camera freshness,
visibility and target stability are monitored; fresh measured endpoints are
verified before the next observation. Unknown goal acceptance or unconfirmed
stop retains ownership; a new command cannot bypass it.

The [exclusive handoff](wrist_closure_handoff.md) passes each verified approach
to exactly one closure. The hand claims it from the wrist task before any finger
command. The wrist then refuses arm commands until that closure ends and releases
it. These two ROS tasks cannot stop motion from the robot UI; transfer control
explicitly as described below.

The [tracked approach](wrist_approach_tracking.md) locks the initial visible
surface/reference in robot-base coordinates, associates it across corrections,
and rejects target loss, ambiguity and changed camera geometry. Initial
acquisition and final completion each require at least three new source frames
spanning 0.15 seconds with consistent references. Repeated checks of one frame
cannot establish readiness. Target tracking continues during planning, movement
and endpoint settling. Manual positioning must establish a suitable orientation;
corrections remain translation only.

The hand driver publishes measured joint **names**, positions, and raw force
channels with the timestamp of the older successful angle/force Modbus read.
Cached republishes repeat this exact timestamp. `effort` uses the driver's raw
units, matching `actuator_force`; it is **not N or Nm**. The hardware closure
uses baseline-subtracted magnitudes, freezes loaded digits at their measured
postures, and applies neither simulation contact preload nor holding preload.
It checks every received measurement against the force limits. Success requires
thumb-plus-finger force in at least five **new** measurements spanning 0.25 s;
re-checking a cached measurement adds nothing. This evidence is not
object-specific contact, opposing normals or proof of retention. No lift follows.
See [physical closure validation](hand_force_check.md).

The color-independent detector requires one isolated depth component fully inside
the ROI, excluding configured hand/holder rectangles. It rejects ambiguity,
clipped components and missing depth. An optional table-patch plane fit removes
supporting surfaces; a visible table is not required. The grasp reference is the
median visible XYZ plus a measured offset in color optical axes. It does not infer
hidden object geometry, object centre or finger placement. See the
[camera-only object-selection guide](wrist_object_selection.md) to inspect and
configure this rule before motion. Capture-time flange TF remains required by
the motion task. No external camera or learning model is required.

Native robot/table/self-collision checking retains its existing clearance.
The target and visual-only holder/camera are **not covered** by these native
checks, reported in status. The simulation-only grasp-scene service stays
disabled. This task performs no obstacle routing or automatic orientation search.

## Supply physical measurements

On the host, copy the template outside Git:

```bash
cd ~/Documents/hackathon/neurahackathon
mkdir -p "$HOME/lara5_wrist_config"
cp src/robot/lara5_wrist_grasp/config/hardware.yaml "$HOME/lara5_wrist_config/hardware.yaml"
nano "$HOME/lara5_wrist_config/hardware.yaml"
```

Enter these settings before execution:

1. The default `camera_mount_mode: nominal_cad` reuses the shared CAD depth-optical
   pose and fixed URDF mounting chain, then composes the driver's depth/color
   optical TF. Match `optical_frame` and `depth_optical_frame` to the driver.
   Run the [read-only alignment preview](wrist_camera_alignment.md) before marking
   `camera_calibration_confirmed: true`; nominal placement is not calibration.
   A measured replacement uses `camera_mount_mode: measured_color` and
   `camera_to_flange_xyz` / `camera_to_flange_xyzw` as complete
   **T_flange_color_optical**, metres and quaternion **x,y,z,w**:
   `p_flange = R_flange_color * p_color + t_flange_color`.
2. `base_alignment_confirmed` is true in the template because the user confirmed
   the previous 50-degree discrepancy is resolved. No guessed URDF offset was
   introduced. Capture-time flange TF is compared with measured native FK.
3. Match `roi`, `exclusion_rois`, `surface_to_grasp_xyz`, `hand_to_grasp_xyz` and
   workspace bounds to the bench. Verify the reference on the first test object
   and intended view, then set `grasp_reference_confirmed: true` for execution.
   Zero offset is a preview placeholder. Color and enclosing object dimensions
   are no longer detector inputs. Manually orient the hand for the chosen grasp;
   the approach holds this orientation. The hand grasp-region offset still uses
   the existing simulation geometry and must be checked against the assembly.
4. Fill all six `hardware_force_baseline`, `hardware_force_contact_thresholds`
   and `hardware_force_limits` in the template's named joint order. Use measured
   **raw actuator-force units**, not simulation Newton thresholds. Derive them
   with the read-only [force check](hand_force_check.md). It takes the baseline
   and thresholds from unloaded open and free-air recordings and checks
   detectability with loaded fingertip recordings. Choose maximum forces yourself
   from the loaded evidence; unloaded data cannot set them. Each contact threshold
   must be positive and below its maximum. Set
   `hardware_force_limits_confirmed: true` only after a consistent report. Zero
   placeholders reject closure. Firmware force thresholds are not changed.

**Near-range depth:** the default accepted range starts at 20 cm. The nominal
CAD assembly places the reference grasp region about 13 cm from the camera, so
that geometry cannot finish this depth-guided approach with the default cutoff.
Check the actual mount/profile. Lower the software cutoff only if measured depth
is reliable there, or adjust the physical mount/grasp geometry. RGB visibility
alone never permits a blind final advance.

Settings load at startup. Restart tasks after edits. Execution defaults to
disabled; calibration and force configuration remain separately required.

## Build on the Ubuntu host

Stop previous camera and robot-controller launches before replacing their
container. Run one camera driver and one arm/hand client at a time.

```bash
cd ~/Documents/hackathon/neurahackathon
docker build -f src/docker/Dockerfile -t lara5_hackathon:wrist-hardware src
docker build -f src/docker/Dockerfile.realsense --build-arg ROBOT_IMAGE=lara5_hackathon:wrist-hardware -t lara5_hackathon:wrist-hardware-camera .
```

These rebuild the modified hand driver and add the Humble camera workspace.
Existing old images do not contain the port. The closure handoff adds a
`lara5_safety_msgs` service type, so rebuild both images. Keep MuJoCo off the
hardware domain.

## Terminal 1: container and camera

On the host:

```bash
xhost +SI:localuser:root
docker run --rm -it \
  --name lara5_wrist_hardware \
  --network host --ipc host \
  --cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
  -e ROS_DOMAIN_ID=74 -e ROS_LOCALHOST_ONLY=0 \
  -e DISPLAY="$DISPLAY" -e QT_QPA_PLATFORM=xcb \
  -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev/bus/usb:/dev/bus/usb \
  --device-cgroup-rule='c 189:* rwm' \
  --mount "type=bind,source=$HOME/lara5_wrist_config,target=/config,readonly" \
  lara5_hackathon:wrist-hardware-camera bash
```

Inside:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
source /camera_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=cameras camera_name:=wrist serial_no:=_262722075125 enable_color:=true enable_depth:=true enable_infra1:=false enable_infra2:=false rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=848x480x30 align_depth.enable:=true enable_sync:=true pointcloud.enable:=false wait_for_device_timeout:=10.0
```

Leave this running; it starts no arm/hand motion.

## Terminal 2: backend and preview-only hardware tasks

On the host, then inside the container:

```bash
docker exec -it lara5_wrist_hardware bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
source /camera_ws/install/setup.bash
ros2 launch lara5_wrist_grasp hardware.launch.py config_file:=/config/hardware.yaml start_backend:=true execution_enabled:=false rviz:=true robot_ip:=192.168.2.13 hand_ip:=192.168.2.75
```

This starts the real backend, safety filter and two idle tasks. If the same
updated backend already runs, use `start_backend:=false`; never start a duplicate.
Do not run the old standalone grasp node on the same command topic.

## Terminal 3: camera UI

```bash
docker exec -it lara5_wrist_hardware bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
source /camera_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /cameras/wrist/color/image_raw
```

The topic selector also offers
`/cameras/wrist/aligned_depth_to_color/image_raw`. Manually position the open
hand with its grasp region within 8 cm of the object. Verify usable depth and
clearance.

**Hand control from the robot UI to ROS** before any wrist preview or approach:

1. Stop jogging and hand guiding, release any enabling or guiding button, and
   leave the manual motion mode.
2. Make sure no UI program is running or paused.
3. Confirm the section 5.2 GUI settings in `src/README.md`
   still apply. Re-apply them after any e-stop release.
4. Announce the handoff. Nobody touches the robot UI until ROS reports a terminal
   state. One operator stays at the e-stop.

To adjust manually later, first cancel the active ROS task and wait for a
terminal status with `active_request_id: null`. Only then use the robot UI.
Afterwards a new approach is required before closure. See
[robot UI and ROS](wrist_closure_handoff.md#robot-ui-and-ros).

## Terminal 4: preview, approach, then grasp

```bash
docker exec -it lara5_wrist_hardware bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
source /camera_ws/install/setup.bash
ros2 control list_controllers
ros2 topic echo /cameras/wrist/color/camera_info --once --qos-reliability best_effort
ros2 topic echo /right_hand/measured_state --once --qos-reliability best_effort
ros2 service call /safety/arm/get_frame_pose lara5_safety_msgs/srv/GetFramePose '{frame: lara5_flange, reference_frame: lara5_root_link}'
```

Require active arm/hand controllers and joint-state broadcaster. Verify the
physical measurements and camera frame, then request a preview:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/wrist/preview std_msgs/msg/Empty '{}'
ros2 topic echo /arm/wrist/status --qos-reliability reliable --qos-durability transient_local --full-length
```

This read-only path preview allows unconfirmed camera/base alignment, which is
reported explicitly in status. First inspect the separate
[alignment overlay](wrist_camera_alignment.md) and verify the actual assembly.
Wait for `preview_ready`; inspect `flange_xyz` in robot-base coordinates. RViz
can show `/arm/wrist/flange_target` with a Pose display. Stop echo with Ctrl+C.
A failed preview sends no movement. Resolve its reported cause before continuing.

After camera/base alignment and the object grasp reference are verified and
confirmed in the configuration, keep the robot stationary, stop Terminal 2's launch with Ctrl+C and restart it
with dispatch enabled:

```bash
ros2 launch lara5_wrist_grasp hardware.launch.py config_file:=/config/hardware.yaml start_backend:=true execution_enabled:=true rviz:=true robot_ip:=192.168.2.13 hand_ip:=192.168.2.75
```

Keep the same `start_backend` choice as before. If tasks alone were started with
`start_backend:=false`, restart tasks alone and leave the backend running.

Back in Terminal 4:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/wrist/approach std_msgs/msg/Empty '{}'
ros2 topic echo /arm/wrist/status --qos-reliability reliable --qos-durability transient_local --full-length
```

Wait for `succeeded` and `ready_for_closure: true`. Stop echo with Ctrl+C. Readiness
invalidates on arm drift, closed hand or camera failure. One closure consumes it.
Then:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/grasp/target std_msgs/msg/Empty '{}'
ros2 topic echo /arm/grasp/status --qos-reliability reliable --qos-durability transient_local --full-length
```

The hand reports `claiming`, then `requested` once the wrist grants the result.
Meanwhile `/arm/wrist/status` shows `handed_off` and refuses every arm command.
A refused claim reports `rejected_state` with the reason and sends no hand command.
When closure ends, the wrist reports `released`. Another closure requires a new
approach. Closure success requires bilateral physical force in at least five new
measurements spanning 0.25 s; `force_evidence` reports them.
Retention remains unverified and the object is not lifted. Nothing else starts
automatically.

## Cancellation and shutdown

In a sourced shell, cancel the respective task:

```bash
ros2 topic pub --once /arm/wrist/cancel std_msgs/msg/Empty '{}'
ros2 topic pub --once /arm/grasp/cancel std_msgs/msg/Empty '{}'
```

Wait for terminal status and actual stationary hardware. An unconfirmed-stop
`error` keeps commands blocked; do not replace the active client to bypass it.
Topic cancellation and terminal Ctrl+C are not physical e-stop. If the hand task
dies without releasing the handoff, the wrist stays `handed_off`. Restart both
tasks; never call `/arm/wrist/closure_handoff` manually.

Opening is a separate decision with the arm stationary and object supported:

```bash
ros2 run lara5_controller_manager hand_command.py open --duration 5
```

Close the viewer, stop Terminal 2 and the camera launch with Ctrl+C, exit all
container shells, then on the host:

```bash
xhost -SI:localuser:root
```

## Offline verification

In an isolated container/domain without camera or robot network access:

```bash
colcon test --packages-select lara5_wrist_grasp lara5_grasp --event-handlers console_direct+
colcon test-result --test-result-base build/lara5_wrist_grasp/test_results --verbose
colcon test-result --test-result-base build/lara5_grasp/test_results --verbose
```

Tests use synthetic RGB-D and physical feedback plus fake native services/actions.
They do not establish physical grip or camera accuracy. Actual calibration,
near-range depth, physical force characterization and robot trials remain needed.

Initial hardware-port validation on 2026-10-27 used an isolated Humble container with no network or USB
access. The modified C++ hand driver and three Python packages built successfully.
The six package suites completed **230 tests: 222 passed, 8 opt-in tests skipped,
0 errors/failures**. This includes 21 wrist tests and 59 grasp cases (one skipped).
The older camera prototype regression additionally passed 19 tests with five
opt-in skips using EGL. A five-second idle hardware-task launch started both
nodes with `start_backend:=false`, then both exited cleanly on SIGINT. No real
robot command or physical hand operation was issued during implementation.

The subsequent object-selection update built successfully and passed all
**70 current wrist tests**, including the depth-check regression suite. Its
installed camera-only preview launch started and exited cleanly on SIGINT.
See the [selection guide](wrist_object_selection.md) for the new reference rule
and its physical validation limits. Other package suites were not rerun for this
detector-only update.

The subsequent shared CAD/color alignment update built and passed all **95
current wrist tests**, and its installed standalone launch started and exited
cleanly on SIGINT. Its 640×480 overlay was inspected with synthetic input. See
the [alignment guide](wrist_camera_alignment.md); the actual mounting alignment
still requires independent physical verification.

The subsequent tracked-approach update built and passed all **117 current wrist
tests**, including moving-camera tracking, distractors, target loss, replayed
camera timestamps and multi-frame completion through fake native services/actions.
Physical camera/robot trials remain pending. See the
[tracking guide](wrist_approach_tracking.md) for association limits and evidence.

The subsequent exclusive-handoff update rebuilt `lara5_safety_msgs`, `lara5_grasp`
and `lara5_wrist_grasp`. It passed all **121 current wrist tests** and **63
grasp tests with one opt-in skip**. The tests include a real wrist task and a
real hardware-mode hand task sharing one handoff. An idle hardware launch
advertised the service and exited cleanly on SIGINT. See the
[handoff guide](wrist_closure_handoff.md) for the protocol, robot-UI procedure
and limits.

The subsequent physical-closure-validation update rebuilt the hand driver,
`lara5_safety_msgs`, `lara5_grasp` and `lara5_wrist_grasp`. Three consecutive
full runs passed: **121 wrist tests** and **90 grasp tests with one opt-in skip**.
Stability now requires new physical measurements, and every measurement is
checked against the force limits. The read-only force check records the open
hand, free-air motion and loaded fingertips. See the
[force-check guide](hand_force_check.md); physical force characterization remains
pending.
