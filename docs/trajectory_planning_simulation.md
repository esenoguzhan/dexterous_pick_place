# Step 2: pregrasp and final-approach planning in the simulation UI

This step generates and validates two arm trajectories. It **does not execute
either trajectory**, open/close fingers, or lift an object. The simulated robot
stays at its starting pose while RViz displays the planned paths and final hand.
Step 3 will add positioning execution after this review.

Input remains a manually supplied correct object-centre pose in `world`.
No perception node, camera feedback, object selection or learning is required.

## What was implemented

Enable `plan_trajectories:=true` on the existing grasp-preview launch. The same
`/arm/approach/preview/target` input now requests both static geometry and planning:

```text
manual object pose -> final grasp-frame calculation
fresh measured arm + open-hand state -> phase A: aligned pregrasp
phase A endpoint -> phase B: final grasp position
native IK + trajectory checks + endpoint FK -> preview outputs only
```

The pregrasp is backed off along the configured grasp axis until the **flange** is
0.30 m above the supplied object centre. The orientation is the final grasp
orientation. This is not a 30 cm descent or a fingertip clearance.

The native comparison supported the 30 cm default: from the reset arm state,
the tested -90 degree cylinder yaw produced a valid two-phase plan at 30 cm and
failed direct-line IK at 20 cm. This is a result for the tested state/target,
not a guarantee for every object pose. There is no automatic yaw search or route
selection; rejected inputs report the native reason.

The old `/arm/approach/target` execution node keeps its independent 20 cm
provisional stop. Do not launch or command it for this planning-only test.

## Terminal 1: new Docker container and MuJoCo viewer

Stop previous simulation launches on domain 42 with Ctrl+C before starting a new
backend. Existing containers retain their previous installed code.

On the host:

```bash
cd ~/Documents/hackathon/neurahackathon
docker build -f src/docker/Dockerfile -t lara5_hackathon:trajectory-plan src
xhost +SI:localuser:root
docker run --rm -it --name lara5_plan_gui --network host --ipc host \
  -e ROS_DOMAIN_ID=42 -e DISPLAY="$DISPLAY" -e QT_QPA_PLATFORM=xcb \
  -e QT_X11_NO_MITSHM=1 -e MUJOCO_GL=glfw -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  lara5_hackathon:trajectory-plan bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_controller_manager controller_manager.launch.py deployment_mode:=simulation sim_headless:=false rviz:=false hands:=on safety:=on sim_camera_backend:=egl
```

Leave this terminal running. MuJoCo should stay open. Wait for the robot
controllers/safety backend to be ready and the objects to settle. Use the default
fresh simulation, whose hand starts open. The planner rejects a closed hand;
it never changes the hand posture itself.

## Terminal 2: planning preview and RViz

On the host:

```bash
docker exec -it lara5_plan_gui bash
```

Inside:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_object_approach grasp_preview.launch.py plan_trajectories:=true
```

Leave it running. RViz opens with a wider view that fits the approach paths.
There should be only one preview instance for these topic names. No separate
approach or grabbing node is needed.

## Terminal 3: supply the tested green-cylinder target

On the host:

```bash
docker exec -it lara5_plan_gui bash
```

Inside:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/approach/preview/target geometry_msgs/msg/PoseStamped '{header: {frame_id: world}, pose: {position: {x: 0.25, y: -0.20, z: 0.95}, orientation: {z: -0.7071067811865476, w: 0.7071067811865476}}}'
ros2 topic echo /arm/approach/preview/plan_status --qos-reliability reliable --qos-durability transient_local --full-length
```

This manually supplies **-90 degree yaw**. The green cylinder is rotationally
symmetric, so this is an explicit operator choice for the test. The planner does
not alter the supplied yaw. The cylinder's settled centre is z=0.95 m, while its
initial spawn is z=0.97 m; wait for it to settle before the test.

Watch for `planning`, then `plan_ready`. If the plan already completed before the
monitor started, its retained `plan_ready` event is shown immediately. Use Ctrl+C
in Terminal 3 to stop the monitor after reading the result.

For this -90 degree input, expected flange targets are:

| Phase endpoint | XYZ in world, metres |
| --- | --- |
| Aligned pregrasp | `[0.100, -0.225, 1.250]` |
| Final grasp position | `[0.100, -0.225, 1.035]` |

The final descent is 0.215 m. With identity yaw, the same geometry gives
`[0.275, -0.350, 1.250]` and `[0.275, -0.350, 1.035]`, but that direct path was
rejected from reset for thumb-to-arm clearance. Geometry previews remain valid
even when a trajectory cannot be planned from the current state.

## What to inspect in the UI

- MuJoCo: the actual robot remains stationary, with its hand open.
- RViz orange line: measured starting flange position to the aligned pregrasp.
- RViz cyan line: pregrasp to final grasp position.
- RViz translucent blue hand and CAD holder: the intended final static placement.
- Green cylinder and grasp axes: the manually supplied target and preset.

Orbit with the left mouse button and zoom with the wheel. Check the two paths
meet at one endpoint and the final path is straight along the grasp approach
axis. This is path/geometry review, not animated execution or object retention.

The [verified RViz planning view](images/trajectory_plan_ui.png) was captured
from the packaged image on a private virtual screen with live idle simulation
feedback. It shows both connected paths and the final static hand placement.

After `plan_ready`, Terminal 3 can inspect the pregrasp pose:

```bash
ros2 topic echo /arm/approach/preview/pregrasp_pose --qos-reliability reliable --qos-durability transient_local --once
```

The existing `/arm/approach/preview/status` reports `preview_ready` for static
geometry. **Trajectory validity is reported on `/arm/approach/preview/plan_status`.**

## Output interfaces and validity

| Topic | Message type |
| --- | --- |
| `/arm/approach/preview/target` | `geometry_msgs/msg/PoseStamped` input, unchanged |
| `/arm/approach/preview/plan_status` | `std_msgs/msg/String` JSON |
| `/arm/approach/preview/pregrasp_pose` | `geometry_msgs/msg/PoseStamped` |
| `/arm/approach/preview/pregrasp_trajectory` | `trajectory_msgs/msg/JointTrajectory` |
| `/arm/approach/preview/final_trajectory` | `trajectory_msgs/msg/JointTrajectory` |
| `/arm/approach/preview/path_markers` | `visualization_msgs/msg/MarkerArray` |

Outputs are reliable/transient-local, depth 1. Both trajectory messages share a
header timestamp, also recorded in the ready status with their plan ID, measured
start configuration, durations, waypoint counts, FK errors and native clearance.
Status always includes `executable: false`: these are review outputs.

No complete trajectories are published until both phases pass. A new request or
failure clears trajectory messages to empty, resets the pregrasp pose to a default
empty message and deletes path markers. Stale feedback, an arm-position change
over 0.01 rad or a hand posture outside its 0.03 rad open tolerance invalidates a
ready plan. An active planning request discards additional targets. Timeouts
abandon the request and ignore late responses.

Do not connect preview trajectory topics to controllers. Step 3 must own fresh
state checks, execution and phase-to-phase feedback. To change any planning
parameter, edit a copy of the full `grasp_preview.yaml` and restart with
`config_file:=/path/to/your/config.yaml`.

## Checks and limits

Only these read-only native services are used:

```text
/safety/arm/plan_cartesian_path
/safety/arm/get_frame_pose
/safety/arm/check_trajectory
```

The planner validates each waypoint's structure, time order, finite values, URDF
joint limits and a maximum 0.20 rad joint step. It verifies the local URDF model
against native FK at the start, checks both endpoints within 1 mm / 0.01 rad, and
checks interpolated FK stays within 3 mm of each Cartesian segment. Phase B starts
from the exact final joint state of phase A.

Conservative cubic segments have zero velocity at every waypoint. They stay on
the checked joint segments and bound peak joint speed/acceleration to
0.25 rad/s and 0.50 rad/s². They stop at individual waypoints, so the original
native test produced approximately **90.25 s + 18.57 s** of trajectory duration.
Planning those trajectories took under a second after backend startup. This is
not optimized or jerk-limited execution timing.

Existing native collision/joint checks remain active, with no alternate routes or
margin reductions. The manual object pose is not automatically inserted into the
shared collision scene, and the camera/holder currently have visual-only geometry.
Status reports these coverage limits explicitly. A `null` minimum-clearance value
means the valid native check did not evaluate a finite distance in its broad phase;
it does not mean zero clearance or skipped validation.

Possible outcomes include `unavailable`, `rejected_state`, `rejected_invalid`,
`rejected_busy`, `plan_failed` and `plan_invalidated`. Read the full message; do not
interpret a static hand overlay or `preview_ready` as a feasible motion plan.

## Native review record and tests

The [native review record](trajectory_planning_native_review.json) contains results
and accepted path coordinates for three manual cylinder poses, all from the same
synthetic reset joint state with an open hand. The real NEURA backend accepted
all three at -90 degree yaw and 30 cm clearance. It rejected identity yaw for
thumb-to-arm clearance and a far-away target for unreachable IK. No controllers
were started for those native integration checks.

The package tests also exercise malformed/discontinuous/partial paths, wrong FK
endpoints, stale or missing feedback, a closed hand, busy commands, late service
responses, timeouts and invalidation after arm movement. Local FK is compared
against the actual MuJoCo robot at multiple joint configurations.

The image build completed all 14 robot packages and passed **121 tests**, with two
optional tests skipped (native integration and viewer concurrency). The native
integration was then explicitly run in isolation and passed. The packaged launch
also reached `plan_ready` with actual idle simulation feedback, and RViz/MuJoCo
were inspected together on a private virtual display. The node graph showed only
the three read-only service clients, no action clients and no controller-topic
publishers. No approach/grasp motion commands were sent during validation.
The temporary validation containers were removed afterward.

To repeat the ordinary tests in a sourced Humble workspace:

```bash
colcon test --packages-select lara5_object_approach --event-handlers console_direct+
colcon test-result --test-result-base build/lara5_object_approach/test_results --verbose
```

The native integration test is opt-in and must use an isolated container/domain.
It rejects domain 0/42 and requires `ROS_LOCALHOST_ONLY=1`. It starts only the native
safety server with motion endpoints remapped to unused names; there are no
controller connections:

```bash
ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 LARA5_NATIVE_PLANNING_TEST=1 python3 -m pytest -q -s /ws/src/robot/lara5_object_approach/test/test_native_planning.py
```

Stop the preview launch and backend with Ctrl+C, exit the container shells, then
revoke the temporary X11 grant on the host:

```bash
xhost -SI:localuser:root
```
