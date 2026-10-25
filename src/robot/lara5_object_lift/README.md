# Verified vertical object lift (ROS 2 Humble simulation)

Publish **`std_msgs/msg/Empty` to `/arm/lift/target`** after a successful
contact-limited grasp. The arm makes a 4 cm test lift, verifies that the object
left the table and followed the hand, then completes a total 10 cm vertical lift.
The hand keeps its existing hold command. No finger opening, transfer or lowering
is included. Startup is idle; a retained grasp result never triggers motion.

The single bringup starts this node:

```bash
ros2 launch lara5_grasp_bringup simulation.launch.py
```

[Full Docker and terminal sequence](../../../docs/grasp_bringup_simulation.md).
A standalone `ros2 launch lara5_object_lift lift.launch.py` needs an existing
simulation backend with `allow_scene_updates:=true allow_grasp_updates:=true`.

## Interfaces and prerequisites

| Interface | Type | Meaning |
| --- | --- | --- |
| `/arm/lift/target` | `std_msgs/msg/Empty` | Explicit lift trigger, reliable/volatile depth 1 |
| `/arm/lift/status` | `std_msgs/msg/String` | Retained JSON phase, result and measured object rise |
| `/arm/lift/markers` | `visualization_msgs/msg/MarkerArray` | Magenta vertical flange path for RViz |
| `/arm/grasp/status` | `std_msgs/msg/String` | Required completed simulation closure and measured hand snapshot |
| `/joint_states` | `sensor_msgs/msg/JointState` | Fresh arm and hand feedback |
| `/sim/grasp_feedback` | `lara5_safety_msgs/msg/SimGraspFeedback` | Object pose, opposing contacts, forces and support state |

The input pose used for approach remains manually supplied. Lift uses simulated
physics feedback as verification data; it does not select objects from perception
or use camera images. `target_object_name: green_cylinder` identifies the known
simulation fixture. The preset is for its upright cylinder; yaw symmetry is
ignored when checking relative tilt.

The lift-capable grasp preset adds one finite 0.25 rad flexion preload over at
least 3 s after bilateral contact, with a 16 N summed simulated contact ceiling.
These settings are for this simulation, not calibrated physical-hand forces.

Require a completed `contact_limited` simulation grasp with contact evidence,
measured hold completion and the same current finger posture. Wait for opposing
thumb/finger contacts to remain stable for 0.35 s before dispatch. A missing or
failed grasp, moving positioning task, stale feedback or changed hand rejects or
stops the request. Each successful-grasp request ID permits only one lift attempt;
additional lift triggers do not stack 10 cm moves.

## Checked motion and physical verification

1. Capture measured arm/hand state and object pose relative to the flange.
2. Verify native FK and update the native collision model to the measured closed
   hand plus a conservative carried-object box, padded by 10 mm per side.
3. Execute the straight vertical 4 cm move through `/safety/arm/move_to_pose`.
   Only the carried collision box may contact the named tabletop support during
   this initial test; robot/table and other native checks remain active.
4. Require measured flange accuracy, object rise, loss of table support, opposing
   contact, limited slip/tilt and a settled hold. A successful arm action alone
   is insufficient. If this fails, the remaining lift is never requested.
5. Remove the payload/table exception, recheck the native collision context and
   execute the remaining 6 cm. Verify the final physical object rise and hold.

The native carried-object box is **collision geometry only**. It does not add a
MuJoCo weld, attachment, equality constraint or object actuator. The real simulated
object remains a free physics body and must be carried by finger contact/friction.
No object is teleported. Native IK and collision rejection remain active; no
obstacle route search is added.

Typical states: `verifying_grasp`, `preparing_lift`, `test_lifting`,
`verifying_test_lift`, `lifting`, `verifying_lift`, `succeeded`.
Success reports `object_rise_m`, flange error, relative slip, forces,
`supported: false`, `opposing_contacts: true` and `retention_verified: true` for
the measured endpoint verification interval. It is not an indefinite guarantee
of retention. Holder/camera visual meshes and other manually unmodelled scene
objects remain outside collision coverage, reported in status.

Feedback loss, grip loss or excessive relative motion cancels the arm action.
The task waits for a terminal action result before releasing ownership. It leaves
the fingers holding, never opens automatically and never moves back down blindly.
After a confirmed failure it removes the fictitious carried box and retains a
hand-only collision snapshot. An unconfirmed controller stop or failed collision
cleanup reports `error` and blocks new lift requests. Restart the stack if that
state cannot be resolved.

A later explicit hand-open reset restores the original open-hand collision model.
For repeatable tests, restart a fresh simulation after a lift rather than assuming
the object still matches the original manual pose.

## Parameters

Parameters load at startup from `config/lift.yaml`; use `lift_config` on the
combined launch or `config_file` on the independent launch.

| Parameter | Default |
| --- | --- |
| `lift_height` / `test_lift_height` | 0.10 / 0.04 m, measured upward in world |
| `test_lift_duration` / `final_lift_duration` | 4 / 6 s requested minimum durations |
| `object_size` / `collision_padding` | `[0.06, 0.06, 0.10]` m / 0.01 m per side |
| `feedback_max_age` | 0.5 s, measured and received timestamps |
| `grip_settle_time` / `grip_timeout` | 0.35 / 3 s |
| `contact_threshold_n` / `max_contact_force_n` | 0.15 / 16 N summed target contacts |
| `contact_loss_grace` | 0.15 s; excessive slip still stops immediately |
| `slip_tolerance` / `tilt_tolerance` | 0.012 m / 0.15 rad, cylinder relative to flange |
| `hand_tolerance` / `arm_start_tolerance` | 0.08 / 0.01 rad |
| `position_tolerance` / `angle_tolerance` | 0.003 m / 0.02 rad flange endpoint |
| `hold_settle_time` / `endpoint_timeout` | 0.75 / 4 s |
| `max_final_object_speed` | 0.03 m/s |
| `service_timeout` / `goal_response_timeout` | 10 / 10 s |
| `motion_timeout` / `cancel_timeout` | 30 / 5 s |

## Validation

The complete real MuJoCo approach → grasp → lift regression passed for one
nominal green-cylinder pose. Measured rise was **98.86 mm**, relative slip
**7.22 mm**, and maximum summed contact force during lift **14.79 N**. The
object was unsupported by the table with opposing finger contacts. Flange
endpoint error was 0.13 mm / 0.00045 rad, with a 0.75 s settled verification
interval. These results do not establish repeatability at other poses.

[Recorded measurements](../../../docs/lift_review.json),
[external view](../../../docs/images/lift_external_camera.png),
[wrist view](../../../docs/images/lift_wrist_camera.png).

Ordinary tests use isolated fake native services/actions and synthetic feedback.
They cover grasp prerequisites, two stages, support-exception removal, object
following, slip/support failures, stale data, cancellation, delayed acceptance,
model cleanup and explicit open reset. A real physics regression is opt-in in an
isolated container/domain without hardware:

```bash
ROS_DOMAIN_ID=211 ROS_LOCALHOST_ONLY=1 LARA5_SIM_LIFT_TEST=1 python3 -m pytest -q -s /ws/src/robot/lara5_object_lift/test/test_sim_lift.py
```

Set `LARA5_SIM_LIFT_VIEWER=1` with a private display for visible MuJoCo/RViz.
