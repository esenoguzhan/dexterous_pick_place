# Object-pose approach task (ROS 2 Humble)

This is the independent approach task for the perception-driven architecture.
Publish one selected object's centre pose to **`/arm/approach/target`**. The node
moves the arm to a configured pose relative to that object and stops. The hand
posture stays under the separate grasp task's control.

```text
restore_perception (Jetson, planned)
  /perception/capture_scene or /perception/snapshot
        -> object selection / task coordinator
        -> /arm/approach/target (PoseStamped, explicit command)
        -> object_approach
        -> /safety/arm/move_to_pose (native NEURA IK, direct Cartesian line)
        -> arm controller

object_approach -> /arm/approach/status -> task coordinator
after succeeded -> /arm/grasp/target (reserved for the future separate grasp task)
```

Approach has no image, depth, point-cloud, camera TF, snapshot, scene-update or
grasp-feedback subscriptions. It needs the supplied pose and the existing native
motion backend, which reads robot joint feedback. It makes one direct Cartesian
move; there are no route candidates, obstacle detours, retries or visual servoing.
The backend's existing joint limits and collision rejection remain active. A
rejected direct line fails the task; this node does not search for another route.

This package runs independently of combined camera approach and pick/place
workflows.

## Input and perception boundary

| Interface | Type | Meaning |
| --- | --- | --- |
| `/arm/approach/target` | `geometry_msgs/msg/PoseStamped` | One selected object's centre pose; publishing commands motion |
| `/arm/approach/status` | `std_msgs/msg/String` | JSON task state and result; latest event retained for late subscribers |
| `/safety/arm/move_to_pose` | `lara5_safety_msgs/action/MoveToPose` | Existing absolute flange motion backend |

The command requires `header.frame_id: world` (or the configured `world_frame`),
positions in metres, and a unit yaw-only quaternion in ROS `(x,y,z,w)` order. The
timestamp is carried through to the action; no camera freshness check is made.
The caller owns object selection and the decision that a snapshot is suitable
for motion. The node freezes this pose for the complete move.

The planned `ObjectState.pose` is a `geometry_msgs/Pose`, so the perception/task
adapter only needs to wrap the chosen pose with its snapshot's timestamp and
world frame. If the final snapshot definition has a `std_msgs/Header header`,
the forwarding operation is:

```python
import copy
from geometry_msgs.msg import PoseStamped

# Run only when the coordinator explicitly requests approach, not on every snapshot.
command = PoseStamped()
command.header = copy.deepcopy(snapshot.header)
command.pose = copy.deepcopy(selected_object.pose)
approach_publisher.publish(command)
```

`restore_interfaces` and `restore_perception` are not present yet, and this
package does not invent their message definitions or launch a fake perception
implementation. Once available, the adapter should subscribe to the retained
snapshot with reliable, transient-local QoS, or obtain it through
`/perception/capture_scene`. Adapt the header extraction to the agreed schema.
Never remap the continuously published/retained scene directly to a motion
command. Approach input is reliable, **volatile**, depth 1 so startup does not
replay retained commands.

Object `size`, `footprint`, `symmetry`, `up_side`, `support_id`, confidence,
camera counts, cloud and embedding do not drive this task. The offset is measured
from the supplied **object centre**, including its provided z. The caller can
choose an offset appropriate for the object through a task-specific configuration.
Snapshot yaw is used as supplied: 180°/90° ambiguity, cap/handle direction and
upside-down state are not resolved here. Continuous-symmetry objects can use a
consistent neutral yaw chosen by the caller. No precision grasp orientation is
inferred from a snapshot.

## Object pose to flange pose

The commanded link is the backend's `lara5_flange`, not a fingertip or object TCP:

```text
T_world_flange = T_world_object * T_object_flange
```

`object_to_flange_xyz` gives the offset along the object's axes, so an XY offset
rotates with object yaw. `object_to_flange_rpy` gives roll, pitch and yaw in radians
relative to the object. The defaults are `[0, 0, 0.40]` metres and `[pi, 0, 0]`:
the flange stops 0.40 m above the object centre with a downward orientation that
follows object yaw. These are provisional hand geometry settings, not a calibrated
grasp pose. Set the stopping offset for the installed hand and object dimensions.
Approach does not open or close fingers.

Parameters are read at startup; restart after changing
[`config/approach.yaml`](config/approach.yaml). `duration: 0.0` uses the native
backend's configured speed limits; a positive value requests a minimum duration.
Topic/action names can be changed with ordinary ROS remapping when running the
executable directly.

## Build and test independently

Use the repository's supported **ROS 2 Humble** robot environment; see the
[robot build instructions](../../README.md). The existing robot Dockerfile
automatically includes this package on rebuild:

```bash
# From the repository root on the host:
docker build -f src/docker/Dockerfile -t lara5_hackathon src
```

The image uses Ubuntu 22.04, ROS 2 Humble and Python 3.10. Its build runs all
approach tests and fails if they fail. Open the Humble workspace container:

```bash
docker run --rm -it --name lara5_approach --network host --ipc host \
  -e ROS_DOMAIN_ID=42 lara5_hackathon bash
# Inside the container, /opt/ros/humble and /ws/install are already sourced.
```

Run the backend and task commands below inside this container. For another
terminal, use `docker exec -it lara5_approach bash`. All participating containers
and nodes must use the same `ROS_DOMAIN_ID`.

For a source checkout in an existing **Humble** workspace, build the task and
its interfaces (the Docker image already does this):

```bash
colcon build --base-paths src/robot --packages-up-to lara5_object_approach
source install/setup.bash
```

Start the ordinary robot/simulation backend with `safety:=on`. The independent
task launch starts only this task, so it can also connect to an existing backend:

```bash
ros2 launch lara5_object_approach approach.launch.py
# Alternative, with an explicit offset:
ros2 run lara5_object_approach approach_node.py --ros-args \
  -p object_to_flange_xyz:='[0.0, 0.0, 0.40]'
```

Run just one instance of the approach node. For a camera-free simulation, create
a copy of the simulation config with both renderers disabled, then launch the
backend with that config (inside the sourced robot container/workspace):

```bash
python3 - <<'PY'
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import yaml
source = Path(get_package_share_directory('lara5_mujoco_sim')) / 'config/mujoco_sim.yaml'
config = yaml.safe_load(source.read_text())
config.setdefault('external_camera', {})['enabled'] = False
config.setdefault('wrist_camera', {})['enabled'] = False
Path('/tmp/pose_approach_sim.yaml').write_text(yaml.safe_dump(config))
PY
ros2 launch lara5_controller_manager controller_manager.launch.py \
  deployment_mode:=simulation sim_headless:=true safety:=on \
  sim_config:=/tmp/pose_approach_sim.yaml
```

In separate sourced terminals, start the approach node and monitor status:

```bash
ros2 topic echo /arm/approach/status --qos-durability transient_local
```

Publish once to test approach without perception. This example is near the
simulated red cube; replace it with the intended object's actual centre pose.
It requests motion, and reachability depends on the current robot state:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 \
  /arm/approach/target geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: -0.35, y: 0.30, z: 0.925}, orientation: {w: 1.0}}}'
```

Normal states are `idle`, `planning`, `executing`, then `succeeded` or `failed`.
`executing` means the native action accepted the request; its internal planning
may still be running. Each command has a generated `request_id`; events also
carry `active_request_id`. `rejected_busy` identifies a discarded new command,
while `active_request_id` still identifies the ongoing motion. Busy commands are
not queued or used to retarget the arm. `rejected_invalid` and `unavailable` also
discard the command without motion; publish again explicitly after fixing it.
Transport errors report `error` and keep the task busy because stopping has not
been confirmed. Ctrl+C requests cancellation of an accepted native action, with
`canceling` and then `canceled` when confirmed.

The coordinator must wait for `succeeded` for its request before requesting grasp.
`/arm/grasp/target` is a suggested future interface name only; this change does
not implement or publish grasp commands. No perception process or grasp process
is needed to test approach. Use one arm motion client at a time, since the
approach node's busy guard cannot arbitrate other clients of the shared backend.

## Automated checks

Geometry tests need only Python:

```bash
PYTHONPATH=src/robot/lara5_object_approach python3 -m unittest discover \
  -s src/robot/lara5_object_approach/test -p test_geometry.py -v
```

After building/sourcing the package, ROS tests use unique remapped topics and a
fake `MoveToPose` action server. They never address the real arm action and need
no camera, robot, MuJoCo, or perception packages:

```bash
colcon test --packages-select lara5_object_approach --event-handlers console_direct+
colcon test-result --verbose
```

These verify transform composition, frame/quaternion rejection, command dispatch,
completion/failure, busy rejection, cancellation and missing-backend handling.
They do not establish physical reachability or calibrate the flange offset.

Validated in Docker with Ubuntu 22.04, ROS 2 Humble and Python 3.10: the full
robot workspace builds, and all 13 approach tests pass. The node
explicitly resolves the action name to support Humble action remapping.
