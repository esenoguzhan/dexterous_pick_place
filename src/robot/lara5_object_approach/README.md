# Object-pose positioning and planning previews (ROS 2 Humble)

Publish a manually supplied object-centre pose to **`/arm/approach/target`** to
position the open hand around it. The task now plans and executes two connected
arm phases: aligned pregrasp, then final approach. It replaces the former
single move to a provisional 20 cm flange offset.

The hand must already be open. Approach never commands finger motion.
`/arm/grasp/target` remains the [separate finger-closure task](../lara5_grasp/README.md).
Positioning success is not evidence of object retention.

See [the complete Docker/MuJoCo/RViz execution test](../../../docs/positioning_execution_simulation.md).
The existing [planning-only UI test](../../../docs/trajectory_planning_simulation.md)
still works without commanding movement.

## Input and task boundary

| Interface | Type | Meaning |
| --- | --- | --- |
| `/arm/approach/target` | `geometry_msgs/msg/PoseStamped` | Explicit object-centre command; executes arm positioning |
| `/arm/approach/status` | `std_msgs/msg/String` | Retained JSON progress and verified result |
| `/arm/grasp/target` | `std_msgs/msg/Empty` | Separate, explicit closure command; never published by approach |

Input requires `frame_id: world`, metres, finite coordinates and a unit yaw-only
quaternion in ROS `(x,y,z,w)` order. The object pose is frozen for the command.
The caller owns object selection and pose validity. No perception package,
labels, snapshots, camera feedback or learning is required. A future intermediate
layer may wrap the selected perception `Pose` in `PoseStamped`; do not connect a
continuously published/retained perception snapshot directly to a motion topic.
The command subscription is reliable/volatile, depth 1; startup cannot replay a
retained target. Busy inputs are discarded, with their own `request_id` and the
ongoing command's `active_request_id`.

No yaw search, obstacle routing, lift, transfer or placement is performed. A
rejected direct path reports its reason. See the [perception boundary comparison](../../../docs/perception_trajectory_compatibility.md)
for future integration considerations.

## Shared grasp geometry and two-phase planning

Preview and execution use the same installed URDF and manual cylinder preset:

```text
T_flange_grasp = T_flange_hand * T_hand_grasp
T_world_flange = T_world_object * T_object_grasp * inverse(T_flange_grasp)
```

The grasp origin in `right_hand_base_link` is `[-0.025, 0.085, 0.150]` m, with
RPY `[-pi/2, 0, 0]`. Object-to-grasp RPY is `[pi, 0, 0]`. The preset describes
an open-hand grasp region, not a calibrated fingertip TCP or proven grasp.
The cylinder reference radius is 0.03 m and height 0.10 m.

Pregrasp backs off along the grasp approach axis until flange Z is **0.30 m above
the supplied object centre**. This is neither fingertip clearance nor descent
distance. For the tested cylinder at `[0.25, -0.20, 0.95]` with -90 degree yaw:

| Endpoint | Flange XYZ in world (m) |
| --- | --- |
| Aligned pregrasp | `[0.100, -0.225, 1.250]` |
| Final grasp position | `[0.100, -0.225, 1.035]` |

Both phases must validate before any motion. Planning captures fresh arm and
open-hand joint feedback, uses native Cartesian IK, starts phase B at the exact
phase A endpoint, and checks joint limits/steps, interpolated FK, endpoint native
FK and both complete trajectories. Zero-velocity cubic segments bound speed to
0.25 rad/s and acceleration to 0.50 rad/s². Conservative timing can take roughly
two minutes for the example; it stops at every waypoint and is not jerk-limited.

## Execution and verification (step 3)

`approach.launch.py` loads `config/approach.yaml` and starts only the approach
node (plus optional RViz). Run one arm motion client at a time with the ordinary
simulation backend and `safety:=on`:

```bash
ros2 launch lara5_object_approach approach.launch.py rviz:=true
```

1. Validate both trajectories while measured arm state stays unchanged.
2. Recheck measured phase-start joints and send the exact validated phase A to
   `/arm_trajectory_controller/follow_joint_trajectory`, through the safety filter.
   The preview's historical timestamp is cleared for immediate controller start.
3. Monitor fresh arm/open-hand state throughout. Wait for a successful terminal
   controller result and fresh **post-result** feedback.
4. Require measured joints, flange pose and grasp-frame pose to remain within
   tolerance for the settling interval. Verify native FK of the measured state.
5. Only then dispatch the checked phase B, again checking its measured start.
6. Repeat measured verification at the final pose. Report `succeeded` with
   `ready_for_closure: true`; leave the hand open.

Normal status sequence:

```text
idle -> planning -> planned -> moving_to_pregrasp -> verifying_pregrasp
     -> moving_to_grasp -> verifying_grasp -> succeeded
```

`unavailable`, `rejected_invalid`, `rejected_state` and `rejected_busy` discard
commands. `failed` means planning/positioning did not complete. Inspect the full
JSON message. Controller success alone cannot produce approach success.

Feedback loss, a hand leaving the open posture, or a motion timeout requests
cancellation and blocks the next phase. Ctrl+C also requests cancellation.
A cancellation acknowledgment alone does not release ownership: a terminal
controller result must confirm completion/cancellation. A delayed accepted goal
is canceled. If stop cannot be confirmed, status becomes `error` and new targets
remain blocked; shutdown logs the unconfirmed stop. The safety-filter forwarding
path also handles cancellation received before its inner controller accepts.
No automatic retry or finger opening is performed.

Final success includes measured joint/flange/grasp-frame errors. Treat retained
status as the result of the identified command, not a continuously refreshed
permission to grasp after moving the arm or changing the scene.

## Preview and visualization interfaces

`grasp_preview.launch.py` remains a separate process with no controller clients:

```bash
# Static geometry only:
ros2 launch lara5_object_approach grasp_preview.launch.py
# Static geometry plus read-only native planning:
ros2 launch lara5_object_approach grasp_preview.launch.py plan_trajectories:=true
```

| Preview interface | Meaning |
| --- | --- |
| `/arm/approach/preview/target` | Manual pose, never a motion command |
| `/arm/approach/preview/status` | Static `preview_ready` or invalid input |
| `/arm/approach/preview/flange_pose`, `/grasp_pose` | Calculated final poses |
| `/arm/approach/preview/markers` | Static hand/holder/cylinder overlay |
| `/arm/approach/preview/plan_status` | `plan_ready`, failures or invalidation; `executable: false` |
| `/arm/approach/preview/pregrasp_pose` | Calculated pregrasp |
| `/arm/approach/preview/pregrasp_trajectory`, `/final_trajectory` | Complete checked review data |
| `/arm/approach/preview/path_markers` | Orange/cyan flange paths |

All names after commas use the same `/arm/approach/preview` prefix. Outputs are
reliable/transient-local, depth 1. Both complete trajectories share a timestamp;
partial/failed plans clear trajectory/path outputs. Static `preview_ready` alone
never establishes trajectory validity. Preview plans invalidate on stale state,
hand closure or arm drift. Do not remap review trajectories to controllers.

The execution node also publishes the static overlay and checked trajectory/path
review outputs for RViz. Its authoritative lifecycle is `/arm/approach/status`;
it does not publish a preview `plan_ready` event. Do not run a separate planning
preview concurrently on these same output names. The executor owns an in-memory
plan and never consumes retained preview outputs. MuJoCo shows actual movement;
RViz's translucent hand is the static intended final placement.

## Parameters and coverage limits

Parameters load at startup; restart after editing a full configuration copy:
`ros2 launch lara5_object_approach approach.launch.py config_file:=/path/approach.yaml`.
The old `object_to_flange_xyz`, `object_to_flange_rpy` and `duration` settings no
longer control this task. Use the shared grasp-frame preset and planning limits.

| Parameter | Default |
| --- | --- |
| `pregrasp_flange_clearance` | 0.30 m flange above object centre |
| `planning_state_max_age` | 1.0 s, receipt and measurement timestamps |
| `planning_state_tolerance` | 0.01 rad phase-start drift |
| `planning_open_tolerance` | 0.03 rad from configured open posture |
| `planning_max_joint_step` | 0.20 rad |
| `planning_max_joint_speed` / `planning_max_joint_acceleration` | 0.25 rad/s / 0.50 rad/s² |
| `planning_position_tolerance` / `planning_angle_tolerance` | 0.001 m / 0.01 rad |
| `planning_path_tolerance` | 0.003 m |
| `planning_service_timeout` / `planning_timeout` | 15 s / 60 s |
| `execution_joint_tolerance` | 0.01 rad measured endpoint |
| `execution_position_tolerance` / `execution_angle_tolerance` | 0.003 m / 0.02 rad, flange and grasp frame |
| `execution_settle_time` / `execution_verification_timeout` | 0.2 s / 3 s |
| `execution_goal_response_timeout` | 10 s |
| `execution_timeout_margin` | 10 s beyond each phase's trajectory duration |
| `execution_cancel_timeout` | 5 s before unconfirmed-stop error |
| `execution_tracking_tolerance` | 0.10 rad controller path tolerance |

Native robot/table/self-collision checks retain their 20 mm margin. There is no
scene update: the supplied object and visual-only camera/holder are **not covered
by those collision checks**, reported explicitly in status. Positioning accuracy
and camera extrinsics remain model/preset dependent. Wrist depth may be invalid
inside its configured minimum range. Camera images never control this task.

## Build and checks

Use Ubuntu 22.04 / ROS 2 Humble Docker, as described in the execution UI guide:

```bash
docker build -f src/docker/Dockerfile -t lara5_hackathon:positioning src
```

In the sourced container/workspace:

```bash
colcon test --packages-select lara5_object_approach --event-handlers console_direct+
colcon test-result --test-result-base build/lara5_object_approach/test_results --verbose
```

Ordinary tests use unique remapped services and fake controllers. They cover
exact phase dispatch, measured phase barriers, endpoint mismatch, closed/stale
feedback, native rejection, controller failure, busy commands, cancellation,
late goal acceptance and unconfirmed-stop locking. Preview remains independently
tested with only read-only service clients. Opt-in native tests require an
isolated container/domain; see their headers and the planning guide.
