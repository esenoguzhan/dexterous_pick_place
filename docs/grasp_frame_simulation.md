> Historical step-1 guide: its old 20 cm approach-motion instructions are superseded.
> Use [the step-3 execution guide](positioning_execution_simulation.md) for current motion tests.
> Static preview commands remain available.

# Grasp frame and 20 cm approach: Docker / simulation UI test

For the newer step-2 planning-only mode with a 30 cm aligned pregrasp, use
[the trajectory planning guide](trajectory_planning_simulation.md). The commands
below retain the original step-1 review and independent provisional motion test.

This update implements the static geometry review in step 1. It also reduces the
existing motion topic's flange offset from 0.40 m to **0.20 m above object centre**.
That motion uses a palm-down orientation with open fingers horizontal. The final
grasp pose is calculated and displayed independently; no transition from the
20 cm stop to that pose is executed yet. Object capture needs the later trajectory
and closure/contact steps.

No perception process is required. Supply the correct object pose manually.
The commands below use the default green cylinder, radius 0.03 m, height 0.10 m,
on the table at z=0.90 m. Its **settled centre** is `[0.25, -0.20, 0.95]` m. The
scene initially spawns it at z=0.97 m; wait for it to settle before motion tests.
If you move it in the viewer, supply its new correct pose manually.

## Terminal 1: build, start Docker, open the MuJoCo UI

Run on the host:

```bash
cd ~/Documents/hackathon/neurahackathon
docker build -f src/docker/Dockerfile -t lara5_hackathon:grasp-frame src
xhost +SI:localuser:root
docker run --rm -it --name lara5_grasp_frame_gui --network host --ipc host \
  -e ROS_DOMAIN_ID=42 \
  -e DISPLAY="$DISPLAY" \
  -e QT_QPA_PLATFORM=xcb \
  -e QT_X11_NO_MITSHM=1 \
  -e MUJOCO_GL=glfw \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  lara5_hackathon:grasp-frame bash
```

Now the prompt is inside the container. Run:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_controller_manager controller_manager.launch.py deployment_mode:=simulation sim_headless:=false rviz:=false hands:=on safety:=on sim_camera_backend:=egl
```

Leave this terminal running. A MuJoCo window should stay open. Camera rendering
uses EGL while the interactive viewer uses GLFW, avoiding the previous concurrent
GLFW/X11 crash. Keep both cameras enabled. `rviz:=false` here avoids opening an
extra default robot window; Terminal 2 opens the dedicated grasp-preview RViz UI.

Use only one simulation/backend on ROS domain 42. Stop your previous test's
launch processes before starting this one. The new container name avoids reusing
an older container whose installed files would still contain the old settings.

## Terminal 2: open the static grasp preview in RViz

On the host:

```bash
docker exec -it lara5_grasp_frame_gui bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_object_approach grasp_preview.launch.py
```

Leave this terminal running. RViz opens with fixed frame `world` and the preview
MarkerArray display configured. An empty view before the first target is normal.
This node cannot send arm or hand commands and can also run without Terminal 1.

## Terminal 3: provide the manual pose and inspect the preview

On the host:

```bash
docker exec -it lara5_grasp_frame_gui bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/approach/preview/target geometry_msgs/msg/PoseStamped '{header: {frame_id: world}, pose: {position: {x: 0.25, y: -0.20, z: 0.95}, orientation: {w: 1.0}}}'
ros2 topic echo /arm/approach/preview/status --qos-reliability reliable --qos-durability transient_local --once
ros2 topic echo /arm/approach/preview/flange_pose --qos-reliability reliable --qos-durability transient_local --once
```

Expected status: `preview_ready`. Expected final flange XYZ: approximately
`[0.275, -0.350, 1.035]` m. This pose is data only and is not dispatched.

In RViz you should see:

- A translucent blue open hand at the calculated final pose.
- A green cylinder at the manually supplied centre.
- A translucent grey table reference at z=0.90 m.
- A yellow grasp origin, orange flange origin and red/green/cyan grasp axes.
- A label stating that this is a static preview with no robot motion.

Drag with the left mouse button to orbit; use the wheel to zoom. Inspect from
the side and underneath. The cylinder should lie in the intended thumb/finger
capture region, with open-hand clearance. This is not a contact or retention
demonstration. The actual robot in MuJoCo remains at its starting pose.

An [inspected underside view of the static model](images/grasp_frame_static_under.png)
is included for comparison. It shows geometry only, with no closure or physics
integration; the grey table marker is supplied by the RViz preview.

To check yaw handling, publish a second preview-only command in this same shell:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/approach/preview/target geometry_msgs/msg/PoseStamped '{header: {frame_id: world}, pose: {position: {x: 0.25, y: -0.20, z: 0.95}, orientation: {z: 0.7071067811865476, w: 0.7071067811865476}}}'
```

The displayed hand rotates 90 degrees around the cylinder. The physical simulator
does not move. Republishing the first command resets the preview.

## Terminal 4: start the existing approach motion task

This section is a separate **motion test** of the requested 20 cm offset.

On the host:

```bash
docker exec -it lara5_grasp_frame_gui bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_object_approach approach.launch.py
```

Leave it running. In Terminal 3, verify the new values, open the hand explicitly,
then send one motion command:

```bash
ros2 param get /object_approach object_to_flange_xyz
ros2 param get /object_approach object_to_flange_rpy
ros2 run lara5_controller_manager hand_command.py open --duration 3
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/approach/target geometry_msgs/msg/PoseStamped '{header: {frame_id: world}, pose: {position: {x: 0.25, y: -0.20, z: 0.95}, orientation: {w: 1.0}}}'
ros2 topic echo /arm/approach/status --qos-reliability reliable --qos-durability transient_local
```

Expected parameter values are `[0.0, 0.0, 0.2]` and approximately
`[-1.5707963, 1.5707963, 0.0]`. The requested flange target is
`[0.25, -0.20, 1.15]` m. The palm should face down and the open fingers should
extend horizontally. Watch the MuJoCo robot, and wait for status `succeeded`.
Press Ctrl+C in Terminal 3 to stop the status monitor.

The native backend still checks reachability, the full direct path and joint/
collision limits. A target overlay is not proof that this path is feasible. If
the backend reports `failed`, inspect its reason in Terminal 1; no alternate route
or final grasp descent is generated by this update.

This motion stop and the static final-grasp overlay are intentionally different
poses. Closing fingers at the 20 cm stop is not an object-capture test.

## Terminal 5: optional independent finger-closure test

Use this only to exercise the existing grabbing topic while the arm is stationary.
It does not move the arm from its 20 cm stop into the previewed grasp pose.

On the host:

```bash
docker exec -it lara5_grasp_frame_gui bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_grasp grasp.launch.py
```

Leave it running. After the approach has finished, run in Terminal 3:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/grasp/target std_msgs/msg/Empty '{}'
ros2 topic echo /arm/grasp/status --qos-reliability reliable --qos-durability transient_local
```

The fingers close while the arm stays still. Controller completion is not object
retention. Press Ctrl+C to stop the monitor, then explicitly reopen to reset:

```bash
ros2 run lara5_controller_manager hand_command.py open --duration 3
```

## Stop the test

Press Ctrl+C in Terminals 5, 4 and 2, then in Terminal 1 to stop the backend.
Exit the container shells; exiting Terminal 1's main shell removes the container
because it was started with `--rm`. Finally run on the host:

```bash
xhost -SI:localuser:root
```

The image tag is `lara5_hackathon:grasp-frame`;
older image tags/containers keep their previous code and parameters.

## Validation performed

- Built all 14 robot packages in the Humble image.
- Passed 38 approach/preview tests, including transform composition, the actual
  URDF mount, four yaw angles, mesh clearances for both final and 20 cm poses,
  retained preview outputs and absence of motion clients in the preview node.
- Passed the existing 25 grasp tests and 12 camera tests; the opt-in camera-viewer
  unit test was skipped during the image build.
- Opened the installed RViz preview and the idle MuJoCo viewer on a private Xvfb
  display. Confirmed the hand meshes render and both applications remain running.
- Confirmed `preview_ready` and final flange XYZ `[0.275, -0.350, 1.035]` using the
  reliable retained-output CLI commands above.

No arm-approach or finger-closure commands were sent to the simulator during
validation. Full-path feasibility and object retention have not been demonstrated
by this geometry step. The temporary test containers were removed afterward.
