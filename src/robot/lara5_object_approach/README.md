# Object-pose approach and planning previews (ROS 2 Humble)

This is the independent approach task using a manually supplied object pose.
Publish the object's centre pose to **`/arm/approach/target`**. The node
moves the arm to a configured pose relative to that object and stops. The hand
posture stays under the [separate grasp task's control](../lara5_grasp/README.md).

```text
restore_perception (Jetson)
  /perception/capture_scene or /perception/snapshot
        -> object selection / task coordinator
        -> /arm/approach/target (PoseStamped, explicit command)
        -> object_approach
        -> /safety/arm/move_to_pose (native NEURA IK, direct Cartesian line)
        -> arm controller

object_approach -> /arm/approach/status -> task coordinator
after succeeded -> /arm/grasp/target (Empty, separate hand-closure task)
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

The `ObjectState.pose` field is a `geometry_msgs/Pose`, so the perception/task
adapter only needs to wrap the chosen pose with its snapshot's timestamp and
world frame. `SceneSnapshot.header` is a `std_msgs/Header`; the forwarding
operation, after selection and validation, is:

```python
import copy
from geometry_msgs.msg import PoseStamped

# Run only when the coordinator explicitly requests approach, not on every snapshot.
command = PoseStamped()
command.header = copy.deepcopy(snapshot.header)
command.pose = copy.deepcopy(selected_object.pose)
approach_publisher.publish(command)
```

`restore_interfaces` and `restore_perception` now live under `src/restore`.
Their snapshot includes the header and `objects[]` used above. A selector/adapter
should subscribe to the retained snapshot with reliable, transient-local QoS, or
obtain it through `/perception/capture_scene`. No adapter is installed by this
package. See the [perception/trajectory comparison](../../../docs/perception_trajectory_compatibility.md)
for the confirmed table-tilt mismatch, field mapping and Docker deployment gaps.
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
relative to the object. The defaults are `[0, 0, 0.20]` metres and
`[-pi/2, pi/2, 0]` radians. The flange stops **0.20 m above the object centre**.
The orientation accounts for the hand's -90 degree flange mount: palm down,
extended fingers horizontal, following object yaw. The previous `[pi, 0, 0]`
pointed the extended fingers down and is unsuitable for the reduced clearance.
This remains a provisional noncontact stop, separate from the final grasp target.
Approach does not open or close fingers.

## Static grasp-frame preview (step 1)

The new `grasp_preview_node.py` is independently launchable and has no motion
clients. It loads the installed hand URDF, derives the fixed flange-to-hand mount,
resolves the configured open posture including mimic joints, and computes:

```text
T_flange_grasp = T_flange_hand * T_hand_grasp
T_world_flange = T_world_object * T_object_grasp * inverse(T_flange_grasp)
```

The manually configured green-cylinder preset uses a 0.03 m radius and 0.10 m
height. Its grasp origin in `right_hand_base_link` is `[-0.025, 0.085, 0.150]` m.
Grasp +Z is the approach direction, pointing out of the palm and downward in the
desired world pose; +X spans the hand width, and +Y completes the right-handed
frame. Fingers and thumb converge toward this region through their own joint
motions; their individual flexion axes are not replaced by one closing axis.
This is a model-based preset requiring later contact validation, not measured TCP
calibration or evidence that the object will be retained.

| Interface | Type | Meaning |
| --- | --- | --- |
| `/arm/approach/preview/target` | `geometry_msgs/msg/PoseStamped` | Manual object centre; **does not command motion** |
| `/arm/approach/preview/markers` | `visualization_msgs/msg/MarkerArray` | Open hand, cylinder, table reference, flange and grasp axes |
| `/arm/approach/preview/flange_pose` | `geometry_msgs/msg/PoseStamped` | Calculated final flange pose; visualization/data only |
| `/arm/approach/preview/grasp_pose` | `geometry_msgs/msg/PoseStamped` | Desired grasp-frame pose |
| `/arm/approach/preview/status` | `std_msgs/msg/String` | JSON: `idle`, `preview_ready` or `rejected_invalid` |

Preview input is reliable/volatile; all outputs are reliable/transient-local,
depth 1. Invalid input clears the marker overlay and reports rejection; the last
valid pose messages remain historical results, so consult status before using
them. No preview output is connected to an arm/hand controller. The node uses
`config/grasp_preview.yaml`; restart it after configuration changes.

For object centre `[0.25, -0.20, 0.95]` and identity yaw, the preset gives final
flange XYZ `[0.275, -0.350, 1.035]` m. The separate 20 cm approach command gives
flange XYZ `[0.25, -0.20, 1.15]` m. Planning/executing the transition between these
poses is handled by the planning preview below, with execution reserved for step 3;
do not expect finger closure at the existing approach stop
to capture the object.

Launch preview plus its configured RViz window:

```bash
ros2 launch lara5_object_approach grasp_preview.launch.py
```

See [the complete ordered Docker and simulation UI commands](../../../docs/grasp_frame_simulation.md).
The preview also works without a simulator; its table and cylinder are reference
geometry from configuration and the supplied pose, not live scene feedback.

## Two-phase trajectory planning preview (step 2)

Enable planning on the same manual-pose input:

```bash
ros2 launch lara5_object_approach grasp_preview.launch.py plan_trajectories:=true
```

The native safety backend and fresh `/joint_states` must be available. The hand
must already match the configured open posture. The planner does not open it.

1. Capture one fresh measured arm configuration and verify the hand posture.
2. Derive the final flange target using the same grasp frame as the static preview.
3. Back off along the grasp approach axis until flange Z is **0.30 m above the
   supplied object centre**, retaining the final grasp orientation.
4. Plan measured start -> aligned pregrasp with native IK. Plan pregrasp endpoint
   -> final flange target using the exact endpoint joint configuration from phase A.
5. Validate native path structure, joint limits, IK continuity, interpolated local
   FK, endpoint FK and each complete phase through the native trajectory checker.
6. Publish both trajectories and RViz paths only after both phases pass. Partial,
   stale or failed plans are cleared. No action or controller commands are sent.

For identity object yaw at `[0.25, -0.20, 0.95]`, the new aligned pregrasp flange
is `[0.275, -0.350, 1.250]` m and final flange is `[0.275, -0.350, 1.035]` m.
The descent is **0.215 m**, not 0.30 m. The old `/arm/approach/target` execution
node retains its separately configured 0.20 m provisional stop; step 3 will connect
the reviewed geometry and trajectories to execution.

| Additional output | Type | Meaning |
| --- | --- | --- |
| `/arm/approach/preview/plan_status` | `std_msgs/msg/String` | JSON planning state, plan ID, result metrics and coverage limitations |
| `/arm/approach/preview/pregrasp_pose` | `geometry_msgs/msg/PoseStamped` | Aligned pregrasp flange target, published with a complete valid plan |
| `/arm/approach/preview/pregrasp_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Measured start -> pregrasp, preview data only |
| `/arm/approach/preview/final_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Pregrasp -> final grasp position, preview data only |
| `/arm/approach/preview/path_markers` | `visualization_msgs/msg/MarkerArray` | Orange pregrasp path and cyan final approach path |

All outputs use reliable/transient-local QoS, depth 1. Both trajectory headers
share the timestamp reported by `plan_status.trajectory_stamp`; the status includes
their plan ID and `executable: false`. The static `preview_ready` status is separate
from trajectory `plan_ready`: geometry alone does not establish a valid path.
Do not remap preview trajectories to controllers.

On a new input, failure or invalidation, trajectory outputs become empty,
pregrasp pose becomes an empty/default message and path markers are deleted.
Consumers must require a current `plan_ready` result with matching timestamps.
Feedback loss, hand closure or arm drift beyond tolerance invalidates a ready plan.
An active request discards additional inputs; service timeouts abandon pending
requests and ignore late responses.

Planning uses only `/safety/arm/plan_cartesian_path`,
`/safety/arm/check_trajectory` and `/safety/arm/get_frame_pose`. It never updates
the shared collision scene. The manually supplied object is not automatically
added as collision geometry, and holder/camera visual meshes have no collision
model. Status reports those coverage limits explicitly. Existing robot/table
collision checks, including the 20 mm native margin, remain active. A `null`
clearance measurement means the valid backend check reported no finite distance,
not zero clearance or a disabled collision check.

The retimed trajectory uses zero-velocity cubic segments between native IK
waypoints. Peak joint speed/acceleration are bounded analytically; each segment
stays on the checked joint line. This conservative timing stops at each waypoint,
so preview durations can be long. It is not jerk-limited timing.

Key startup parameters in `config/grasp_preview.yaml`:

| Parameter | Default |
| --- | --- |
| `pregrasp_flange_clearance` | 0.30 m |
| `planning_state_max_age` | 1.0 s; receipt and measurement timestamp checked |
| `planning_state_tolerance` | 0.01 rad arm drift |
| `planning_open_tolerance` | 0.03 rad from configured open hand posture |
| `planning_max_joint_step` | 0.20 rad |
| `planning_max_joint_speed` | 0.25 rad/s |
| `planning_max_joint_acceleration` | 0.50 rad/s² |
| `planning_position_tolerance` / `planning_angle_tolerance` | 0.001 m / 0.01 rad |
| `planning_path_tolerance` | 0.003 m |
| `planning_service_timeout` / `planning_timeout` | 15 s / 60 s |

Native validation from the reset pose accepted the green cylinder at three nearby
positions with manually supplied **-90 degree yaw** and 30 cm clearance. Identity
yaw was rejected for thumb/arm clearance; 20 cm did not yield an accepted path
for the same -90 degree test from reset. No automatic yaw selection or route
search is performed. The cylinder's continuous yaw symmetry allows the operator
to choose this orientation in the manual test input.

See [the ordered planning-only simulation UI guide](../../../docs/trajectory_planning_simulation.md)
for the tested quaternion, expected targets, failure cases and validation record.

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
  -p object_to_flange_xyz:='[0.0, 0.0, 0.20]'
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
The independent [`lara5_grasp`](../lara5_grasp/README.md) node accepts an empty
trigger on `/arm/grasp/target` to close the hand. Approach does not publish that
trigger automatically. No perception process or grasp process is needed to test
approach. Use one arm motion client at a time, since the
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
