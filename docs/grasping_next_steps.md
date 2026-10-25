# Trajectory and grasping: implementation and review plan

The input is a manually supplied, correct object-centre pose in the robot world
frame. Treat it exactly as the pose that the future intermediate layer will
provide. That layer owns perception communication, object selection and target
validation. These packages own arm positioning, hand closure and an explicit lift.

Steps 1–5 now have implementations: static grasp geometry, optional read-only
planning, two-phase positioning with the hand open, independent contact-limited
finger closure and a physically verified lift. The approach
motion topic now uses the shared grasp transform and a 0.30 m flange clearance
at aligned pregrasp, replacing the former 0.20 m provisional stop. Step 6 remains
a proposal; implement it only when explicitly requested and reviewed.
See [planning-only UI instructions](trajectory_planning_simulation.md) and
[execution UI instructions](positioning_execution_simulation.md), and
[contact-limited closure UI instructions](contact_closure_simulation.md).
The current complete sequence is in the
[approach → grasp → lift UI guide](grasp_bringup_simulation.md).

## Scope and current interfaces

- `/arm/approach/target`: `geometry_msgs/msg/PoseStamped`, manually supplied object
  centre in `world`, in metres, with the agreed orientation convention.
- `/arm/approach/status`: arm positioning result.
- `/arm/grasp/target`: `std_msgs/msg/Empty`, an explicit finger-closure trigger.
- `/arm/grasp/status`: hand closure result and contact evidence; not proof of object retention.
- `/arm/lift/target`: `std_msgs/msg/Empty`, an explicit vertical lift after grasp.
- `/arm/lift/status`: measured lift result and bounded retention verification.
- Keep approach and grabbing independently testable. An operator or the external
  coordination layer explicitly requests them in order.
- Start with the known green cylinder and configure its dimensions/grasp preset
  manually. Do not require object labels, IDs, snapshots or perception messages.
- No perception selection/adapter, perception-model integration, camera-driven
  alignment, learning, obstacle-avoidance routing, transfer or placement
  is part of this plan.

The CAD-mounted wrist D435 is already available for observation. Its calibrated pose
and camera-based control can be addressed later; they do not block the current
pose-driven trajectory and grabbing work. Static previews: [mount](images/wrist_cad_holder_mount.png)
and [camera view](images/wrist_cad_holder_view.png). The green sphere is a visibility
marker, not a grasp demonstration.

## 1. Define the hand grasp frame — implemented for static review

Define a frame between the thumb and fingers, its approach axis and closing
direction. Derive the fixed transform from the flange to this grasp frame using
the hand model. Choose a manually configured object-relative grasp transform for
the green cylinder, accounting for the open fingers and object dimensions.

Calculate the flange target from the supplied object pose:

```text
T_world_flange = T_world_object * T_object_grasp * inverse(T_flange_grasp)
```

The preview replaces the arbitrary flange offset with a model-based final grasp
transform. Do not treat the object centre as the flange target. Step 2 plans the
pregrasp/final trajectory; step 3 executes it through the motion topic. The
separate preview input still never executes motion.

Review result: static model overlays show the cylinder inside the intended open
hand grasp region. No motion is needed to validate this calculation.

## 2. Plan the pregrasp and final approach trajectories — implemented for review

Generate two arm phases from the manual pose: move to a noncontact pregrasp, then
advance along the configured grasp approach axis to the final flange target.
Use native IK, conservative timing, measured robot state and existing robot
checks. Handle intended finger/object proximity explicitly without adding
obstacle-routing behavior. Reject incomplete or discontinuous plans.

Review result: both trajectories pass preview checks and end at the expected
hand poses. Start with planning only; no finger closure is included.

Implementation: `grasp_preview.launch.py plan_trajectories:=true` plans from fresh
measured arm/open-hand state, validates both connected phases with native IK,
trajectory checks and FK, and displays the resulting paths in RViz. It publishes
no controller/action commands. The native 30 cm example is validated at three
manual cylinder positions with -90 degree yaw. Failures for other direct paths
are reported without changing the supplied yaw, adding routes or relaxing checks.

## 3. Execute positioning with the hand open — implemented

Execute the validated phases through the arm controller. Check measured state
between phases and verify the final grasp-frame pose. The approach task should
report completion only when positioning is complete and the hand is ready for
closure. It must not automatically trigger the grabbing topic.

Implementation: `/arm/approach/target` plans both phases before dispatch through
the filtered `FollowJointTrajectory` action. It requires fresh measured open-hand
state throughout, successful controller results, post-result joint feedback,
settled flange/grasp-frame error checks and native FK verification at both
endpoints. Only a verified pregrasp permits final approach. Cancellation and
unconfirmed-stop ownership prevent progression or retargeting. The hand stays
open; final success reports positioning readiness, not object retention.

Review criterion: the green cylinder is physically inside the open hand's grasp
region while resting on the table. No wrist-camera correction or perception
update is required. See the execution UI guide for the validation result and
remaining collision-coverage limits.

## 4. Improve the independent grabbing motion — implemented for review

Configure finger posture, sequence and closure speed for the cylinder grasp.
Use measured finger state and available simulated contact/force information to
handle contact, absence of an object and incomplete closure. Define hold/failure
behavior explicitly, keeping arm movement outside the grabbing task.

Implementation: the same Empty trigger now starts bounded measured steps, first
thumb opposition and then free-digit closing. Fresh simulation contact/normal-force
feedback freezes contacted digits with one-time bounded preload. Fresh post-result
finger state, progress/tracking checks and stationary-arm monitoring gate each
segment. Force/feedback/controller failures cancel, then hold measured positions
when available. No-object closure is a failure; unknown stop keeps the task busy.
Status separates controller completion, current contact evidence and unverified
retention. The arm is never commanded, and reopening remains an explicit reset.

Review criterion: `/arm/grasp/target` closes the fingers around the positioned
cylinder while the arm stays stationary. Step 5 will assess sustained opposing
contacts and object stability; this step does not confirm retention. See the
closure UI guide for commands and validation results.

## 5. Verify the grasp and lift the object — implemented

Use opposing thumb/finger contacts and sustained load/contact evidence to assess
whether the hand is gripping the object. In simulation, use relative object/hand
pose as verification data and check for unintended object displacement or loss of
contact. Do not use artificial attachments, welding or teleportation.

The later explicit lift request extends this step: `/arm/lift/target` first checks
a settled opposing grip, updates the native closed-hand/carried-object collision
model, then requests a 4 cm vertical test lift. It checks actual object rise,
loss of table support, relative slip/tilt and measured flange accuracy. Only a
verified test permits the remaining 6 cm, for 10 cm total. Both phases use the
native safety-filtered arm action; the lift node never commands fingers.

Simulation closure adds one finite, slow flexion preload after bilateral contact
so the existing hand actuators can support the cylinder. Force, timestamp,
stationary-arm and controller checks remain active. There is no repeated squeeze,
artificial attachment or object teleportation. A weak grasp fails the lift rather
than reporting success from the arm result alone.

Failure cancels arm motion, retains the existing hand hold and prevents the next
phase. Unknown controller stop or collision cleanup locks the task. Success
reports retention only for the checked endpoint interval; it does not promise
indefinite holding. No transfer, release or automatic lowering is included.
See the [lift package contract](../src/robot/lara5_object_lift/README.md).

## 6. Repeat the manual-pose test sequence

Test the three topics independently, then in order: manually publish object pose,
wait for approach completion, explicitly trigger grabbing, inspect grasp result,
then request and verify lifting. Restart the simulation between trials.
Repeat for the green cylinder at several known poses and then other manually
configured objects. Record final positioning error, contact/closure outcome,
repeatability and failure reasons. Agree tolerances and trial counts before the
review tests.

Review result: a repeatable pose-to-trajectory-to-grasp baseline, with no dependency
on perception selection, wrist-camera feedback or a learning method.
