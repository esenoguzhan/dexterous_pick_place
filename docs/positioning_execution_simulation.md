# Step 3: execute open-hand positioning in MuJoCo (ROS 2 Humble)

This test moves the actual simulated arm through aligned pregrasp and final
approach, then verifies its measured pose with the hand open. It does not close
fingers or lift the object. No perception or camera feedback is used.

Use a fresh simulator for the known green-cylinder target. The cylinder settles
to world centre `[0.25, -0.20, 0.95]` m. Supply -90 degree yaw explicitly; this
orientation was accepted from reset. Identity yaw can fail native clearance.

## Terminal 1 — build, container and visible MuJoCo

Stop earlier simulation/task launches on the domain you will use. These commands
use domain **43** for every process. Run on the Linux desktop host:

```bash
cd ~/Documents/hackathon/neurahackathon
docker build -f src/docker/Dockerfile -t lara5_hackathon:positioning src
xhost +SI:localuser:root
docker run --rm -it \
  --name lara5_positioning_gui \
  --network host --ipc host \
  -e ROS_DOMAIN_ID=43 \
  -e DISPLAY="$DISPLAY" \
  -e QT_QPA_PLATFORM=xcb \
  -e QT_X11_NO_MITSHM=1 \
  -e MUJOCO_GL=glfw \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  lara5_hackathon:positioning bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_controller_manager controller_manager.launch.py deployment_mode:=simulation sim_headless:=false rviz:=false hands:=on safety:=on sim_camera_backend:=egl
```

Leave this terminal running. Wait for MuJoCo, all three controllers to activate,
and `Safety filter ready`. Let the objects settle before publishing a target.
The EGL camera backend avoids the previous shared GLFW/X11 renderer crash.
An older image/container does not include this execution implementation.

## Terminal 2 — approach execution and RViz

On the host:

```bash
docker exec -it lara5_positioning_gui bash
```

Inside:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_object_approach approach.launch.py rviz:=true
```

Leave running. Start only one approach instance. No separate preview or grasp
node is needed for this test. RViz initially shows a grid; paths and the intended
static final-hand overlay appear after a valid target has been planned.

## Terminal 3 — open hand and command the green-cylinder approach

On the host:

```bash
docker exec -it lara5_positioning_gui bash
```

Inside, source the workspace and explicitly open the hand:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 run lara5_controller_manager hand_command.py open --duration 3
```

After that command finishes, publish **once**:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/approach/target geometry_msgs/msg/PoseStamped '{header: {frame_id: world}, pose: {position: {x: 0.25, y: -0.20, z: 0.95}, orientation: {z: -0.7071067811865476, w: 0.7071067811865476}}}'
```

Monitor the full status:

```bash
ros2 topic echo /arm/approach/status --qos-reliability reliable --qos-durability transient_local --full-length
```

Typical states are `planning`, `planned`, `moving_to_pregrasp`,
`verifying_pregrasp`, `moving_to_grasp`, `verifying_grasp`, then `succeeded` with
`ready_for_closure: true`. Short-lived states can pass before the monitor starts;
Terminal 2 logs every transition. Ctrl+C in Terminal 3 stops only this monitor.

Keep MuJoCo running throughout. Conservative waypoint timing can take around
two minutes for this example; do not republish because movement is slow.
No closure command is sent by this workflow.

## What to inspect

- MuJoCo shows the actual arm moving with open fingers.
- RViz orange line is measured start to pregrasp; cyan is the final approach.
- RViz's translucent hand/holder and green cylinder are the intended static final
  geometry, not an animation or measured object tracker.
- The arm pauses at pregrasp while post-result measured state and native FK are
  checked. Phase B cannot start on controller success alone.
- At the final position, the cylinder should remain on the table in the open
  hand's intended grasp region. The arm holds there and the fingers stay open.

| Endpoint | Expected flange XYZ in world (m) |
| --- | --- |
| Pregrasp | `[0.100, -0.225, 1.250]` |
| Final | `[0.100, -0.225, 1.035]` |

The flange starts pregrasp 30 cm above the object centre; descent is 21.5 cm.
Final status reports measured flange/grasp-frame position and orientation errors,
plus maximum joint error. `succeeded` means positioning is verified, not that
an object was grasped or retained.

## Failures and stopping

- `unavailable`: wait for action/services and all controllers, then explicitly
  republish. Commands are not queued.
- `rejected_state`: inspect the message for missing/stale feedback or a hand
  outside the open posture. Open it explicitly before retrying.
- `failed`: inspect native planning, controller or measured verification reason.
  No alternate route, automatic yaw change or automatic retry is attempted.
- `rejected_busy`: an additional target was discarded; the active command keeps
  its original target.
- `canceling`: cancellation was requested; wait for a terminal result.
- `error` with unconfirmed stop: no new command is accepted while motion ownership
  is unresolved. Check the controller/backend; restarting a task does not itself
  stop an independently running controller.

Ctrl+C in **Terminal 2** requests arm cancellation and waits briefly for a terminal
controller result before exiting. Keep Terminal 1 alive until this completes.
After the task is stopped, Ctrl+C Terminal 1, exit the container shells, then on
host revoke the temporary display grant:

```bash
xhost -SI:localuser:root
```

To repeat the initial test, restart a fresh simulation rather than assuming a
previously displaced object still matches the supplied manual pose.

## Current limits

The hand preset is model based; camera extrinsics are nominal. Existing native
robot/table/self-collision checks remain active. The manual target object and
visual-only holder/camera are not included in those collision checks. The task
does not observe or verify the object's actual position during motion, so inspect
unintended contacts or displacement visually. Improving closure/contact evidence
is a later step, still separately requested through the grasp task.

## Validation record

An isolated ROS 2 Humble/MuJoCo run executed the known cylinder positioning in
about 113 seconds, including backend startup, with the hand open and no grasp command. It verified both
measured phase endpoints. Final flange XYZ was approximately
`[0.100000, -0.225000, 1.035004]` m; grasp-frame position error was 0.005 mm and
orientation error 0.000008 rad. These are simulation measurements, not physical
calibration accuracy or evidence of grasp retention. The [recorded status events](positioning_execution_review.json)
contain both the initial isolated run and the final built-image result.
The image build passed 140 ordinary checks (four opt-in checks skipped); the
native cancellation and actual MuJoCo positioning regressions were then run
explicitly and passed.

Final simulated camera observations: [external view](images/positioning_external_camera.png)
and [wrist view](images/positioning_wrist_camera.png). They were captured after
execution for inspection; images were never inputs to the positioning controller.
The wrist image shows the nearby cylinder and open fingers. Depth inside the
D435 minimum range remains unreliable.

The opt-in regression starts a fresh headless backend and executes simulated arm
motion. Run only in a private container/domain without hardware connections:

```bash
ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1 LARA5_SIM_POSITIONING_TEST=1 python3 -m pytest -q -s /ws/src/robot/lara5_object_approach/test/test_sim_positioning.py
```

The native cancellation regression starts only a safety server and a fake
stationary controller, checking cancellation before inner goal acceptance:

```bash
ROS_DOMAIN_ID=195 ROS_LOCALHOST_ONLY=1 LARA5_NATIVE_FORWARDING_TEST=1 python3 -m pytest -q /ws/src/robot/lara5_object_approach/test/test_native_forwarding.py
```
