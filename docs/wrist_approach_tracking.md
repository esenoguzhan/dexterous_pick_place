# Tracked, bounded wrist approach

This implements step four after the depth, object-reference and camera-alignment
checks. The operator still positions and orients the open hand using the robot
UI, hands control to ROS, previews the final approach, and explicitly requests
approach and grasp with the existing commands in the
[hardware guide](wrist_grasp_hardware.md). The executor corrects translation only;
choose a suitable finger orientation before starting.

## Acquire and keep one geometric target

Each `/arm/wrist/preview` or `/arm/wrist/approach` request starts a new acquisition.
Exactly one isolated depth object must be available initially. The node locks a
sample of that visible surface and its explicit surface-to-grasp reference in
robot-base coordinates. The original sample remains fixed for that request.

Later observations compensate for the measured capture-time wrist pose. A
candidate must match the original reference, metric surface extent and surface
geometry. The default rules require at least 70% overlap in **both** directions
within 6 mm and similar metric extent. The existing 20 mm reference-displacement
limit remains an additional bound. These are conservative association settings
for the first stationary test object, not calibrated camera accuracy claims.

A separate foreground object can appear without taking over the track. Exactly
one candidate must match the locked surface; zero matches or multiple matches
invalidate the request. The node never picks the nearest/largest replacement or
walks the original anchor onto a different surface through successive updates.
Object loss, substantial occlusion, changed shape/reference, changed camera
intrinsics/mount, and invalid camera data fail the track. A new explicit command
is needed to acquire again after failure. Segmentation is bounded to 16 candidate
components; more clutter fails rather than allocating unbounded matching work.

Association is geometric, with no color classification or semantic identity
recognition. An indistinguishable replacement at the same observed location
cannot be ruled out. Changes in visible geometry may cause a conservative stop
even when the physical object is unchanged. Keep the same object, grasp-reference
rule, view and open-hand exclusions established by the earlier checks.

## Require new measurements

Both initial acquisition and final completion require at least **three distinct
depth source timestamps spanning 0.15 seconds**, with reference observations
consistent within **2 mm**. Checks of a still-fresh cached frame do not count as
new evidence. New camera messages with repeated/backwards source timestamps
invalidate the request. Gaps beyond the feedback freshness limit restart the
consistency window; stale measurements cannot establish readiness.

Before initial planning, acquisition requires a stationary measured arm and a
consistent orientation. During native service waits, correction execution and
endpoint settling, the node continues checking the track and orientation. The
existing native IK, speed, displacement, workspace, cumulative travel, cancellation
and measured endpoint checks remain in use.

At the end, `verifying` collects a new window after verification starts, with the
hand open and the measured arm stationary. Every counted observation must align
the grasp reference with the hand grasp region within `position_tolerance`
(default 3 mm). Capture-time TF is compared with measured native FK, and the
original orientation must remain within `angle_tolerance`. If alignment needs
another small correction, execution returns to the bounded correction loop.
Timeout, cancellation or invalid feedback cannot produce success. Readiness
continues to invalidate on arm drift, track loss or camera faults. One hand closure
consumes it through the [exclusive handoff](wrist_closure_handoff.md).

The 3 mm criterion applies to the **observed** reference. Actual accuracy also
depends on depth quality, pixel resolution, calibration and the object-specific
reference rule; synthetic tests do not establish physical positioning accuracy.

## Configuration and status

These settings live in `wrist_approach.ros__parameters` in the hardware config:

| Setting | Default | Purpose |
| --- | --- | --- |
| `tracking_surface_tolerance` | `0.006` m | Surface-point matching distance |
| `tracking_min_overlap` | `0.7` | Minimum coverage in each direction |
| `tracking_extent_fraction` | `0.3` | Allowed relative extent change, plus surface tolerance |
| `target_motion_tolerance` | `0.02` m | Reference displacement from the original anchor |
| `target_confirm_frames` | `3` | Minimum distinct measurements; cannot be less than three |
| `target_confirm_duration` | `0.15` s | Minimum source-time span |
| `target_consistency_tolerance` | `0.002` m | Pairwise reference consistency; cannot exceed position tolerance |

Restart the tasks after config edits. Source camera geometry and the chosen
reference rule remain fixed during a request. The camera-only selection and
alignment tools remain read-only per-frame diagnostics; their previews do not
create or transfer an approach track.

The retained `/arm/wrist/status` JSON now includes:

- `acquiring` and `verifying` states before planning/completion.
- `target_tracking`: original/latest reference, accepted distinct-frame count,
  source timestamp, association rule and latched failure reason.
- `acquisition_evidence` and `completion_evidence`: distinct-frame counts and
  source-time spans. Only successful execution sets `ready_for_closure`.

`/arm/wrist/object_pose` keeps the observed reference and now carries its camera
capture timestamp. It is still an identity-orientation reference pose, not an
estimate of object yaw.

## Validation scope and next step

Offline tests exercise camera-motion compensation, distractors, replacement and
ambiguous geometry, gradual drift, occlusion, timestamp faults, changed camera
geometry, and new-frame evidence. Real ROS camera/TF topics drive the existing
executor against fake native services/actions, including cancellation during
motion and loss of feedback during final verification.

On 2026-10-27 the updated package built in an isolated Humble container with no
network or USB access. All **117 wrist tests passed**, with no errors, failures or
skips, including the earlier depth and camera-alignment suites. No real robot
motion or physical camera validation was performed.

The [exclusive approach-to-grasp handoff](wrist_closure_handoff.md) followed as
step five. Stronger physical closure checks remain; neither step proves physical
object retention.
