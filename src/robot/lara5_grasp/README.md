# Independent grabbing motion (ROS 2 Humble)

Publish **`std_msgs/msg/Empty` to `/arm/grasp/target`** to close the Inspire right
hand once. The `hand_grasp` node sends a six-joint closure trajectory to
`/hand_controller/follow_joint_trajectory`. This task never commands the arm,
approaches an object, lifts, transfers, releases, or reopens the hand. No cameras,
perception, collision scene updates, contact feedback or approach node are needed.

| Interface | Type | Purpose |
| --- | --- | --- |
| `/arm/grasp/target` | `std_msgs/msg/Empty` | Explicit trigger for one closure motion |
| `/arm/grasp/status` | `std_msgs/msg/String` | JSON request/result status, retained for late subscribers |
| `/hand_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Existing hand controller backend |

The input is an empty trigger because finger closure does not need the object's
pose. Position the hand before triggering if the goal is to hold an object. In
particular, the approach task's default 0.40 m flange offset is a noncontact
stopping pose; running closure there does not guarantee fingers surround an object.
Hand positioning and closure configuration require calibration for a physical grasp.

The trigger is reliable, volatile, depth 1. Startup does not replay retained
commands. During a closure, additional triggers produce `rejected_busy` and are
discarded. If the hand controller is unavailable, the command is discarded with
`unavailable`; send it again explicitly after the controller is ready. No commands
are queued or retried automatically.

States are `idle`, `requested`, `executing`, then `succeeded` or `failed`.
Status includes `request_id` and `active_request_id`, as in the independent
approach task. `succeeded` means the hand controller completed its trajectory;
it does **not** verify object contact, force, retention or grasp success. A
transport error reports `error` and keeps the node busy because motion completion
is unknown. Ctrl+C requests controller cancellation and reports `canceling`, then
`canceled` when confirmed. There is no automatic reopen on completion or failure.

Use one hand motion client at a time, and wait for arm motion to finish before
triggering closure. The independent tasks do not arbitrate other ROS clients.

## Build in Docker

From the repository root on the host:

```bash
docker build -f src/docker/Dockerfile -t lara5_hackathon:humble-grasp src
```

The image includes both independent tasks and runs their isolated tests during
the build. Existing running containers keep their old image; a newly built
package is available in a new container created from the new tag. To start a
Humble shell for headless simulation:

```bash
docker run --rm -it --name lara5_grasp --network host --ipc host \
  -e ROS_DOMAIN_ID=42 -e MUJOCO_GL=osmesa lara5_hackathon:humble-grasp bash
```

For MuJoCo and RViz on a Linux X11/XWayland desktop, instead start the container
with display access:

```bash
xhost +SI:localuser:root
docker run --rm -it --name lara5_grasp --network host --ipc host \
  -e ROS_DOMAIN_ID=42 -e DISPLAY="$DISPLAY" \
  -e QT_QPA_PLATFORM=xcb -e QT_X11_NO_MITSHM=1 \
  -e MUJOCO_GL=glfw -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  lara5_hackathon:humble-grasp bash
```

Then start the simulation backend inside the container:

```bash
ros2 launch lara5_controller_manager controller_manager.launch.py \
  deployment_mode:=simulation sim_headless:=false rviz:=true hands:=on safety:=on
```

For the headless container use `sim_headless:=true rviz:=false`. The default
simulator cameras can be disabled using the config-copy procedure in the
[approach task README](../lara5_object_approach/README.md); neither task uses them.
Keep this backend running. Only one robot backend should run on a given ROS domain.

If your existing `lara5_approach_gui` simulation is already running on domain 42,
keep it running and use the headless container command above just to open a
shell in the **new image**. Skip the backend launch in that container and start
only `ros2 launch lara5_grasp grasp.launch.py`. The two containers communicate
with the existing hand controller through their shared network/IPC and ROS domain.

## Start and trigger the grabbing task

In a second host terminal, open a shell in the new container and start the node:

```bash
docker exec -it lara5_grasp bash
ros2 launch lara5_grasp grasp.launch.py
```

The alternative executable form allows parameter overrides; run one instance:

```bash
ros2 run lara5_grasp grasp_node.py --ros-args -p duration:=3.0
```

In a third host terminal:

```bash
docker exec -it lara5_grasp bash
ros2 control list_controllers
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/grasp/target std_msgs/msg/Empty '{}'
ros2 topic echo /arm/grasp/status --qos-durability transient_local
```

The hand should close while the arm retains its current target. If the fingers
are already at the configured closure positions, another trigger produces no
additional visible finger motion. To reopen between independent tests, stop the
status echo, wait for closure to finish, and use the existing hand utility:

```bash
ros2 run lara5_controller_manager hand_command.py open --duration 3
```

The reopen command is a separate manual test reset, not part of the grabbing task.
When finished, stop the grabbing node with Ctrl+C before stopping the backend.
After closing a GUI container, revoke display access on the host:

```bash
xhost -SI:localuser:root
```

## Configuration and testing

[`config/grasp.yaml`](config/grasp.yaml) is loaded by the task launch. Parameters
are read at startup; restart the node after edits.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `close_positions` | `[1.1, 1.3, 1.3, 1.3, 1.3, 0.5]` | Radians: thumb yaw, index, middle, ring, pinky, thumb pitch |
| `duration` | `3.0` | Positive closure motion duration in seconds |

The default preset is the existing `hand_command.py close` preset reordered by
joint name. Configuration rejects nonfinite angles, wrong vector lengths and
angles outside the repository's hand model limits (yaw 1.1641, fingers 1.4381,
pitch 0.5864 rad; lower limits zero). The controller supplies the starting hand
state and its normal tracking checks. Choose duration to suit the hardware and
desired closure speed. This task does not implement a finger collision check or
force-controlled gripping.

Build/test just this package in a sourced Humble source workspace:

```bash
colcon build --base-paths src/robot --packages-select lara5_grasp
source install/setup.bash
colcon test --packages-select lara5_grasp --event-handlers console_direct+
colcon test-result --test-result-base build/lara5_grasp/test_results --verbose
```

The tests validate configuration and use real ROS transport with a uniquely
remapped fake hand action server. They exercise hand-only goal content, custom
settings, success/error reporting, busy rejection, cancellation, retained status
and missing-controller handling. They do not move a robot or simulator.
