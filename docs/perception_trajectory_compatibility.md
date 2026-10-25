# Perception output and arm approach input

Comparison after merging `origin/main` at `063dad5` into
`feat/humble-object-approach` (merge commit `28347c7`). The local hand-closure
and other simulation changes remain uncommitted. This document describes the
current interfaces; it does not install a bridge or dispatch motion.

The object-centre position, metres and default `world` frame agree. The messages
cannot be connected directly: perception publishes a list of objects in
`SceneSnapshot`, while approach consumes one `PoseStamped`. A selected object's
pose can be wrapped with the snapshot header, but there is also a confirmed
orientation mismatch for tilted table planes.

## Interfaces and field mapping

| Aspect | Perception output | Approach input or behavior | Result |
| --- | --- | --- | --- |
| Endpoint | `/perception/snapshot`, `restore_interfaces/msg/SceneSnapshot` | `/arm/approach/target`, `geometry_msgs/msg/PoseStamped` | Requires an explicit selector/adapter; remapping the topic cannot change message types |
| On-demand capture | `/perception/capture_scene`, `restore_interfaces/srv/CaptureScene` returns `success`, `message`, `snapshot` | No capture request is made by approach | Coordinator can request capture, check `success` and warnings in `message`, then select an object |
| Cardinality | `snapshot.objects[]`, zero or more objects | Exactly one object pose per command | Select one object; never send every snapshot object as a motion command |
| Frame | `snapshot.header.frame_id`, default `world` | Exact match with `world_frame`, default `world` | Compatible when both refer to the same calibrated robot frame; approach does not perform TF conversion |
| Position | `objects[i].pose.position`, object centre, metres | Object centre, metres | Copy unchanged; z already includes table/support height and object half-height |
| Orientation representation | Unit quaternion, ROS `(x,y,z,w)` order | Same representation | Compatible representation; the table-normal/world-Z distinction below remains |
| Orientation meaning | Yaw in the detected table frame, rotated into world | Almost pure yaw around world Z | Compatible for a sufficiently level table; tilted-plane poses can be rejected |
| Stamp | Snapshot header is created after pipeline processing | Passed through to the native action; no freshness check | Preserve it, but it is not the original image acquisition timestamp |
| Identity | `snapshot_id` plus `ObjectState.id`, `label`, `label_score` | Not carried in `PoseStamped` | Coordinator must retain command-to-object association |
| Quality | `confidence`, `depth_reliable`, `cameras_seen` | Not checked by approach | Define admission policy in the selector before sending a command |
| Geometry | `size`, `footprint`, `table_plane`, `support_id` | No corresponding fields | Current approach ignores object dimensions and support relationships |
| Yaw ambiguity | `symmetry`, `up_side_matters`, `up_side` | Supplied quaternion is used directly | No disambiguation or grasp-specific orientation selection is performed |
| Rich features | `cloud`, `embedding` | Not consumed | Not needed for the current approach transform |
| QoS | Reliable, transient local, depth 1 | Reliable, volatile, depth 1 | Adapter reads the retained snapshot but emits a command only on an explicit task request |

Relevant source definitions:

- [ObjectState.msg](../src/restore/restore_interfaces/msg/ObjectState.msg)
- [SceneSnapshot.msg](../src/restore/restore_interfaces/msg/SceneSnapshot.msg)
- [CaptureScene.srv](../src/restore/restore_interfaces/srv/CaptureScene.srv)
- [Perception ROS conversions and capture](../src/restore/restore_perception/restore_perception/node.py)
- [Approach command validation](../src/robot/lara5_object_approach/lara5_object_approach/node.py)

## Confirmed orientation mismatch

Perception's `TableFrame.orientation(yaw)` composes the table frame rotation with
a yaw rotation and converts the result to a world-frame quaternion. A tilted
table therefore produces nonzero quaternion x/y components, even though the
object has no independently estimated roll or pitch relative to the table.
The perception configuration accepts table tilt up to 10 degrees.

Approach currently rejects a normalized quaternion when
`hypot(q.x, q.y) > 0.001`, approximately 0.115 degrees of table tilt. Direct
checks using the actual perception orientation function produced:

| Table tilt | Quaternion x/y norm | Current approach |
| --- | --- | --- |
| 0 degrees | 0.000000 | Accepted |
| 0.1 degrees | 0.000873 | Accepted |
| 1 degree | 0.008727 | Rejected: expected a yaw-only object orientation in world |
| 5 degrees | 0.043619 | Rejected |
| 10 degrees | 0.087156 | Rejected |

This also matters for a nominally level table when plane-estimation noise exceeds
the approach tolerance. The contract needs an explicit decision: support the full
table-aligned quaternion in approach, or enforce a world-horizontal-table policy
before dispatch. Silently removing quaternion x/y components would change the
meaning of the pose and offset; it is not a transparent message conversion.
No orientation policy has been changed by this merge/comparison.

See [TableFrame](../src/restore/restore_perception/restore_perception/core/geometry.py)
and [object_to_flange](../src/robot/lara5_object_approach/lara5_object_approach/geometry.py).

## Minimal conversion, after selection and validation

```python
import copy
from geometry_msgs.msg import PoseStamped

# snapshot is from CaptureScene.response.snapshot or the retained snapshot topic.
# Select within this snapshot; ids are not persistent across captures.
matches = [obj for obj in snapshot.objects if obj.id == requested_object_id]
if len(matches) != 1:
    raise ValueError("Selected object is missing or ambiguous")
selected = matches[0]

# Before dispatch, check frame, quality, snapshot suitability and orientation policy.
command = PoseStamped()
command.header = copy.deepcopy(snapshot.header)
command.pose = copy.deepcopy(selected.pose)

# Publish once only when approach is explicitly requested.
# approach_publisher.publish(command)
```

This is illustrative conversion code, not an installed adapter. Selecting by
label also needs a uniqueness check; multiple objects can share a label or be
`unknown`. IDs such as `obj_00` are assigned by spatial sorting in each snapshot,
so retain `(snapshot_id, object_id)` together. The eight real-object gallery
labels differ from the simulation names `red_cube`, `green_cylinder`, `blue_ball`.
The new approach task itself has no dependency on either naming scheme.

`cameras_seen` is a bitmask, not a count: `1` means camera 1, `2` camera 2, `3`
both. `confidence` is a heuristic combination of scores, not a pose covariance.
`depth_reliable: false` can accompany a known-size/mask fallback pose; it does
not mean the object has no pose. Choose a quality policy appropriate to the task.

## Object pose versus trajectory target and grasp pose

The approach topic takes an **object pose**, then calculates:

```text
T_world_flange = T_world_object * T_object_flange
```

The lower-level `/safety/arm/move_to_pose` action instead takes the **flange
target**. Do not send the raw object centre to that action, and do not pre-apply
the flange offset before sending `/arm/approach/target`, which would apply it a
second time. NEURA IK converts the flange target to joint trajectories.

The current default offset remains `[0, 0, 0.40]` metres with a relative
`[pi, 0, 0]` orientation. Object size, support, symmetry and label do not select
different offsets or finger postures. Perception output therefore supplies enough
geometry for the coarse approach, but does not itself specify a calibrated hand
grasp frame or a finger trajectory. The separate `/arm/grasp/target` input is an
empty closure trigger; it consumes no `ObjectState` and uses a fixed configured
finger posture. Neither merging perception nor wrapping its pose closes the gap
between a noncontact approach and a verified physical grasp.

## Docker and simulation boundaries

The existing robot Dockerfile copies `src/robot` and `src/neura_deps`; it does
not copy `src/restore`. Merging sources does not add perception packages to the
existing robot images or running containers.

- A selector on the Jetson can use its installed `restore_interfaces` and publish
  standard `PoseStamped` commands to the existing robot container.
- A selector inside the robot container needs `restore_interfaces` built there.
  Running the full perception pipeline there additionally needs
  `restore_perception`, dependencies, and the chosen models/source. The merge has
  not changed the Docker deployment layout.
- The root `extrinsics.yaml` is not automatically loaded. The node defaults to
  `<data_dir>/calibration/extrinsics.yaml`, or accepts `extrinsics_file:=...`.
  The calibration must be available to the perception process in that location.
- The merge brings the model integration code, not the external SAM 2.1/DINOv2
  weights or a populated object gallery. Their defaults are under `/models` and
  `/data/gallery`. The offline tests use fake models; they do not establish that
  those real models are installed or identify the physical objects correctly.

For `source:=ros`, perception subscribes to different camera topics than MuJoCo
publishes. These are the required topic correspondences for the existing simulator:

| Perception subscription | Simulator publisher |
| --- | --- |
| `/cameras/camera_1/color/image_raw` | `/sim/external_camera/color/image_raw` |
| `/cameras/camera_1/aligned_depth_to_color/image_raw` | `/sim/external_camera/depth/image_raw` |
| `/cameras/camera_1/color/camera_info` | `/sim/external_camera/camera_info` |
| `/cameras/camera_2/color/image_raw` | `/sim/wrist_camera/color/image_raw` |
| `/cameras/camera_2/aligned_depth_to_color/image_raw` | `/sim/wrist_camera/depth/image_raw` |
| `/cameras/camera_2/color/camera_info` | `/sim/wrist_camera/camera_info` |

The simulated depth and colour share their pixel grid/intrinsics. TF must contain
the optical frames carried by the messages. The camera-disabled GUI workaround
publishes none of these image streams, so live perception from that configuration
cannot work. File replay can test perception without cameras. The earlier
concurrent GLFW camera/viewer crash was separate from this merge. Subsequent
provisional wrist-camera work selects EGL for offscreen camera rendering in
simulation bringup; use its updated image to run cameras with the viewer. A
camera-disabled configuration still needs its streams re-enabled for live perception.

## Validation scope

Validated with Ubuntu 22.04, ROS 2 Humble and Python 3.10 in an isolated container,
using synthetic scenes and fake models, with no ROS motion commands:

- Both `restore_interfaces` and `restore_perception` build successfully, and the
  installed perception launch file loads its arguments.
- All 17 incoming perception geometry/pipeline tests pass independently of hardware.
- An actual synthetic pipeline object pose is passed through the current approach
  geometry function, preserving the object centre and applying the offset once.
- Table-normal quaternions are checked against the current approach validator,
  including the confirmed 1-degree rejection above.
- The merged messages and perception package are built separately under Humble;
  ROS serialization and `ObjectState -> SceneSnapshot -> PoseStamped` conversion
  are checked without creating a ROS node or publishing anything.

Three standalone ROS contract checks passed: message serialization/conversion,
horizontal-table acceptance with the offset applied once, and a check deliberately
confirming the current tilted-table rejection. Passing the last check records a
compatibility gap; it does not mean tilted poses are supported.

These checks do not validate the real SAM/DINO models, calibration accuracy,
physical reachability, finger contact, object retention or live camera feedback.
