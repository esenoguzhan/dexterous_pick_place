# Exclusive approach-to-grasp handoff

This implements step five after the depth, object-reference, camera-alignment and
tracked-approach checks. The operator still positions the open hand with the
robot UI, hands control to ROS, requests the wrist approach, then explicitly
requests closure. The commands `/arm/wrist/approach` and `/arm/grasp/target` are
unchanged. See the [hardware guide](wrist_grasp_hardware.md) for terminal commands.

Previously the two tasks coordinated only through retained status messages. The
hand read `/arm/wrist/status` readiness and the wrist watched `/arm/grasp/status`.
Those messages travel independently, so nearly simultaneous commands could both
pass their checks before either status arrived. Ownership now transfers through
one service hosted by the wrist task, which decides every transfer in one place.

## Ownership sequence

1. **Robot UI.** The operator positions and orients the open hand, then performs
   the [robot-UI-to-ROS handoff](#robot-ui-and-ros) below.
2. **Wrist owns the arm.** `/arm/wrist/approach` runs bounded corrections. A claim
   during this run is refused with `Wrist approach is still running`.
3. **One verified result.** `succeeded` creates one unconsumed result. Every 0.1 s
   the wrist rechecks arm drift, the open hand and the tracked reference. Any
   failure invalidates the result.
4. **Hand claims.** `/arm/grasp/target` checks the hand's local preconditions,
   reports `claiming`, then calls `/arm/wrist/closure_handoff` with operation
   `claim` and its own `request_id`. No finger command is sent before the grant.
5. **Atomic grant.** The wrist grants the claim only if an unconsumed result was
   rechecked within `feedback_max_age` and measured joints still match it. The
   grant consumes the result, returns its `approach_request_id` and arm joints,
   and reports `handed_off`. From then on the wrist refuses preview, approach and
   any other claim. It also refuses a second claim by the same closure.
6. **Hand closes.** The hand rechecks its own preconditions and compares its
   measured arm with the returned joints (0.01 rad). It then runs the existing
   physical closure. Camera occlusion by the fingers no longer matters; closure
   uses hand feedback and the accepted approach result.
7. **Release.** Every terminal hand outcome sends `release` with the same closure
   id: `succeeded`, `failed`, `canceled`, or a rejection after a grant. The wrist
   then reports `released`. Readiness is **not** restored. Another closure needs
   a new explicit approach, which first requires the hand to be reopened.

Service calls run in the wrist task's mutually exclusive callback group, like
its command subscriptions and readiness timer. The approach worker creates the
result only after its last arm command. A preview or approach request first
clears the result. A claim therefore never overlaps an arm correction.

## Failure behavior

| Situation | Result |
| --- | --- |
| No result, consumed result, approach running, stale recheck or arm drift | Claim refused; hand reports `rejected_state`; no hand command |
| Handoff service unavailable | Hand reports `unavailable`; trigger discarded |
| No claim response within `goal_response_timeout` (5 s default) | Hand reports `failed` and still sends `release` |
| `/arm/grasp/cancel` during the claim | Hand reports `canceled` and still sends `release` |
| Returned joints differ from the measured arm | Hand reports `failed`, sends no finger command and releases |
| Hand stop unconfirmed (`error`) | The closure stays active, so the wrist stays blocked |
| Ctrl+C on the hand task | Cancels closure, then waits up to `cancel_timeout` to deliver the release |
| Hand task killed, or release unconfirmed (logged) | Wrist stays `handed_off`; restart both tasks |

A claim and its release come from the same client, so the wrist processes them
in order. A grant that arrives after the hand gave up is therefore released at
once. If the release is lost, the wrist remains blocked. It never becomes
permissive. Restarting the hardware launch restarts both tasks.

The status-based checks remain as secondary guards. While `/arm/grasp/status`
reports an active closure, the wrist refuses commands and stops any correction.
That retained status also blocks a restarted wrist task while a closure continues.
Grasp status no longer invalidates readiness, because the hand's `claiming` status
can arrive before its claim. Finger motion still invalidates readiness through
the open-hand check. The hand no longer reads `/arm/wrist/status`.

## Robot UI and ROS

Coordination between these two ROS tasks cannot stop motion commanded from the
robot UI, a UI program, or another ROS client. The tasks detect arm motion only
after the fact. They cancel corrections, invalidate readiness on 0.01 rad drift,
and abort closure on drift. Neither is a safety function. Ownership must be handed
over explicitly, with one operator at the e-stop throughout.

**Robot UI to ROS**, before any `/arm/wrist/preview` or `/arm/wrist/approach`:

1. Finish positioning and orienting the open hand in the robot UI.
2. Stop jogging and hand guiding. Release any enabling or guiding button, and
   leave the manual motion mode.
3. Make sure no UI program is running or paused.
4. Confirm the robot GUI settings from section 5.2 of [`src/README.md`](../src/README.md)
   still apply, including the enabled C++ interface. Re-apply them after any
   e-stop release.
5. Announce the handoff. Nobody touches the robot UI until ROS reports a
   terminal state. The e-stop operator stays at the e-stop.

**During ROS control**, from approach request through the hand's terminal result,
the robot UI must not command motion. The arm should be stationary after
`succeeded`, `handed_off` and closure.

**ROS to robot UI**, to adjust manually:

1. Cancel the active task with `/arm/wrist/cancel` or `/arm/grasp/cancel`.
2. Wait for a terminal state with `active_request_id: null` in the respective
   status.
3. Only then take over in the robot UI.
4. Treat the earlier result as consumed. A new approach is required before
   another closure. Reopening the hand remains a separate command.

An `error` state means a stop is unconfirmed. Do not use the robot UI to override
an active controller. Follow the site's e-stop procedure.

## Status

`/arm/wrist/status` adds:

- `closure_handoff`: the closure `request_id` holding the result, or `null`.
- States `handed_off` and `released`, each with `closure_request_id`.

`/arm/grasp/status` in hardware mode adds:

- State `claiming` before `requested`. Its `active_request_id` is already set.
- `approach_request_id` in `requested` and in terminal events after a grant.

`/arm/wrist/closure_handoff` (`lara5_safety_msgs/srv/ClosureHandoff`) is internal
to the two tasks. Do not call it manually: a manual claim blocks the wrist, and a
manual release could unblock it during closure.

Simulation and `joint_only` hand modes do not use the handoff. The new service
type changes `lara5_safety_msgs`. Rebuild both hardware images before use.

## Validation scope

On 2026-10-27 the updated `lara5_safety_msgs`, `lara5_grasp` and
`lara5_wrist_grasp` built in an isolated Humble container with no network or USB
access. **lara5_wrist_grasp: 121 tests passed. lara5_grasp: 63 passed, 1 opt-in
skip.** There were no errors or failures. The handoff tests also passed five
repeated runs.

The tests cover:

- one-time consumption and refusal of second claims;
- refusal during correction and after drift;
- arm commands blocked until the matching release;
- a grasp status that arrives before its claim;
- refused, delayed, canceled and mismatched claims, with release ordering;
- a real wrist task and a real hardware-mode hand task together. The hand task
  consumes the approach once, blocks a concurrent approach, releases, then
  cannot close again without a new approach.

An idle `hardware.launch.py` advertised the service and refused a claim without
an approach. It exited cleanly on SIGINT.

No robot command, physical hand closure or robot-UI interaction was performed.
[Physical closure validation](hand_force_check.md) followed as step six.
