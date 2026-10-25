# Trajectory and grasping: implementation and review plan

The input is a manually supplied, correct object-centre pose in the robot world
frame. Treat it exactly as the pose that the future intermediate layer will
provide. That layer owns perception communication, object selection and target
validation. This package owns arm trajectory generation/execution and hand closure.

Steps 1 and 2 now have implementations for review: a static grasp preview and
optional read-only two-phase trajectory planning on the same preview input.
The existing motion topic's flange offset has also been reduced to 0.20 m and its
default orientation changed to keep the palm down and open fingers horizontal.
The planning mode uses a 0.30 m flange clearance at aligned pregrasp; the older
execution node retains its separate 0.20 m provisional stop. Steps 3–6 remain
proposals. Implement one step when explicitly requested, then
review its result before implementing the next. No approach/grasp motion was
executed during validation. See [the planning UI instructions](trajectory_planning_simulation.md).

## Scope and current interfaces

- `/arm/approach/target`: `geometry_msgs/msg/PoseStamped`, manually supplied object
  centre in `world`, in metres, with the agreed orientation convention.
- `/arm/approach/status`: arm positioning result.
- `/arm/grasp/target`: `std_msgs/msg/Empty`, an explicit finger-closure trigger.
- `/arm/grasp/status`: hand closure result; currently not proof of object retention.
- Keep approach and grabbing independently testable. An operator or the external
  coordination layer explicitly requests them in order.
- Start with the known green cylinder and configure its dimensions/grasp preset
  manually. Do not require object labels, IDs, snapshots or perception messages.
- No perception selection/adapter, perception-model integration, camera-driven
  alignment, learning, obstacle-avoidance routing, lifting, transfer or placement
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
transform. The existing motion topic separately stops the flange 0.20 m above
the object centre, as requested; it does not execute the preview target. Do not
treat the object centre as the flange target. Connecting the geometry to the
pregrasp/final trajectory is step 2, with execution in step 3.

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

## 3. Execute positioning with the hand open

Execute the validated phases through the arm controller. Check measured state
between phases and verify the final grasp-frame pose. The approach task should
report completion only when positioning is complete and the hand is ready for
closure. It must not automatically trigger the grabbing topic.

Review result: the green cylinder is physically between the open fingers while
resting on the table. No wrist-camera correction or perception update is required.

## 4. Improve the independent grabbing motion

Configure finger posture, sequence and closure speed for the cylinder grasp.
Use measured finger state and available simulated contact/force information to
handle contact, absence of an object and incomplete closure. Define hold/failure
behavior explicitly, keeping arm movement outside the grabbing task.

Review result: `/arm/grasp/target` closes the fingers around the positioned object
while the arm remains stationary. Controller completion and contact evidence are
reported distinctly. Reopening remains a separate explicit test/reset command.

## 5. Verify a stationary grasp

Use opposing thumb/finger contacts and sustained load/contact evidence to assess
whether the hand is gripping the object. In simulation, use relative object/hand
pose as verification data and check for unintended object displacement or loss of
contact. Do not use artificial attachments, welding or teleportation.

Review result: distinguish closure completed, grasp confirmed, grasp failed and
uncertain grasp using agreed criteria. This step includes no lift or transfer;
a test lift would require a separate later request.

## 6. Repeat the manual-pose test sequence

Test the two topics independently, then in order: manually publish object pose,
wait for approach completion, explicitly trigger grabbing, inspect grasp result.
Repeat for the green cylinder at several known poses and then other manually
configured objects. Record final positioning error, contact/closure outcome,
repeatability and failure reasons. Agree tolerances and trial counts before the
review tests.

Review result: a repeatable pose-to-trajectory-to-grasp baseline, with no dependency
on perception selection, wrist-camera feedback or a learning method.
