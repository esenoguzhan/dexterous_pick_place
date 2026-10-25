# Approach, grasp and lift in one Humble simulation bringup

This replaces the separate backend, approach, grasp and lift launch commands with:

```bash
ros2 launch lara5_grasp_bringup simulation.launch.py
```

It starts MuJoCo and its viewer, both simulated cameras and optical TF, robot TF,
all controllers, the native safety/planning backend, approach, grasp and lift nodes,
and one RViz window. All three tasks start idle. Their explicit inputs are `/arm/approach/target`,
`/arm/grasp/target` and `/arm/lift/target`.

## Build the image after source changes

From the repository root on host:

```bash
cd ~/Documents/hackathon/neurahackathon
docker build -f src/docker/Dockerfile -t lara5_hackathon:grasp-lift src
```

Existing containers retain their installed code. Start a new container from the
new tag. The workflow below uses ROS domain **46**, keeping it separate from the
previous domain-44/45 tests. Use one bringup instance per domain.

## Terminal 1 — container and complete stack

On host:

```bash
xhost +SI:localuser:root
docker run --rm -it \
  --name lara5_grasp_lift_gui \
  --network host --ipc host \
  -e ROS_DOMAIN_ID=46 \
  -e DISPLAY="$DISPLAY" \
  -e QT_QPA_PLATFORM=xcb \
  -e QT_X11_NO_MITSHM=1 \
  -e MUJOCO_GL=glfw \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  lara5_hackathon:grasp-lift bash
```

Inside:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_grasp_bringup simulation.launch.py
```

Leave this terminal running. MuJoCo and RViz open. Wait for active controllers
and `Safety filter ready` before publishing a target. The same launch owns all three
task nodes, so no additional task launches are necessary. No motion is triggered
by startup, and the simulated cameras remain observation only.

For an unattended headless backend, use the same launch inside a container without
display requirements:

```bash
ros2 launch lara5_grasp_bringup simulation.launch.py sim_headless:=true rviz:=false
```

## Terminal 2 — explicit positioning, grasping, then lifting

On host:

```bash
docker exec -it lara5_grasp_lift_gui bash
```

Inside, explicitly open the hand and wait for completion:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 run lara5_controller_manager hand_command.py open --duration 3
```

Publish the settled green-cylinder centre and -90 degree yaw:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/approach/target geometry_msgs/msg/PoseStamped '{header: {frame_id: world}, pose: {position: {x: 0.25, y: -0.20, z: 0.95}, orientation: {z: -0.7071067811865476, w: 0.7071067811865476}}}'
ros2 topic echo /arm/approach/status --qos-reliability reliable --qos-durability transient_local --full-length
```

Wait roughly two minutes for `succeeded` and `ready_for_closure: true`. Press
Ctrl+C in Terminal 2 to stop only that monitor, then request closure separately:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/grasp/target std_msgs/msg/Empty '{}'
ros2 topic echo /arm/grasp/status --qos-reliability reliable --qos-durability transient_local --full-length
```

Expect staged finger closure with the arm holding position. Allow up to 60 seconds.
Wait for `succeeded`, `closure_outcome: contact_limited` and `contact_evidence: true`.
The closure result alone still has `retention_verified: false`.

Press Ctrl+C in Terminal 2 to stop the grasp status monitor. Now request the lift:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/lift/target std_msgs/msg/Empty '{}'
ros2 topic echo /arm/lift/status --qos-reliability reliable --qos-durability transient_local --full-length
```

The arm first raises 4 cm. Only after the object follows the hand, loses table
support and passes a settled grip check does it finish a total **10 cm lift**.
Fingers retain their hold command; there is no automatic opening or lowering.
RViz adds a magenta lift path and MuJoCo shows the real free object moving.

Wait for `state: succeeded`, `object_rise_m` close to `0.10`,
`supported: false` and `retention_verified: true`. Failure or stale contact feedback
stops the lift and holds the hand. Inspect the message; an arm motion result
without physical object following is not lift success. Each grasp permits one
lift attempt. See [lift behavior and limits](../src/robot/lara5_object_lift/README.md).

For force inspection, stop the status monitor and run:

```bash
ros2 topic echo /sim/grasp_feedback --filter "m.object_name == 'green_cylinder'" --once
```

The viewer/process and wall-time feedback fixes from the
[contact closure UI guide](contact_closure_simulation.md) are included. Its
failure/hold/reset behavior and collision-coverage limits remain unchanged.

## Configuration and shutdown

Custom task YAML files can be passed without colliding with one another:

```bash
ros2 launch lara5_grasp_bringup simulation.launch.py approach_config:=/path/approach.yaml grasp_config:=/path/grasp.yaml
```

See [bringup options](../src/robot/lara5_grasp_bringup/README.md) for simulation,
initial-position and RViz configuration. The launch is simulation-only with the
hand and safety filter enabled; it does not select or start physical hardware. Lift enables simulation-only
closed-hand/carried-box collision updates. That carried box never attaches the
physical MuJoCo object; finger contact/friction must actually support it.

Ctrl+C Terminal 2 stops the status monitor. Ctrl+C Terminal 1 shuts down the
complete simulation stack, including all three tasks and the viewer. Exit both
container shells, then revoke display access on host:

```bash
xhost -SI:localuser:root
```

## Validation

The Humble Docker build compiled all 16 workspace packages and passed 184
ordinary checks, with eight opt-in integration checks skipped. The physical
lift and private-display checks below were run separately in isolated containers.

The complete real MuJoCo sequence raised the free green cylinder **98.86 mm**.
The final grip retained opposing contacts without table support; measured
relative slip was **7.22 mm**. The 4 cm test passed before the remaining 6 cm
was sent. The object mass, friction and actuator gains were unchanged.
This is one nominal trial; repeatability across poses remains a future test.
See [recorded measurements](lift_review.json),
[external camera](images/lift_external_camera.png) and
[wrist camera](images/lift_wrist_camera.png).

Readiness tests check all three tasks idle, all controllers active, RGB/depth from
both cameras, green-cylinder contact feedback and one command subscriber per
task. Separate task YAML overrides are checked through Humble parameter services.
Startup tests send no task commands.
The private-display startup test also passed with the MuJoCo viewer enabled and
exactly one RViz node. The physical lift result above was measured headlessly
with both simulated camera streams rendering.
