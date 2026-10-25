# Independent contact-limited closure (ROS 2 Humble, step 4)

Publish **`std_msgs/msg/Empty` to `/arm/grasp/target`** after positioning the open
hand. The task opposes the thumb, then closes free digits in small measured steps.
It limits further motion when simulated target contact is detected and holds the
hand at its contact posture. It never commands the arm, reopens automatically,
lifts, transfers or attaches an object.

See [the ordered Docker/MuJoCo/RViz test](../../../docs/contact_closure_simulation.md).
The approach task remains separately triggered through `/arm/approach/target`.
No perception, object-pose selection, wrist-camera correction or learning is used.

## Interfaces

| Interface | Type | Purpose |
| --- | --- | --- |
| `/arm/grasp/target` | `std_msgs/msg/Empty` | Explicit closure trigger; reliable/volatile, depth 1 |
| `/arm/grasp/status` | `std_msgs/msg/String` | Retained JSON lifecycle, measured motion result and contact evidence |
| `/hand_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Hand-only position trajectory action |
| `/joint_states` | `sensor_msgs/msg/JointState` | Fresh measured finger and stationary-arm state |
| `/sim/grasp_feedback` | `lara5_safety_msgs/msg/SimGraspFeedback` | Simulation-only contacts and per-link normal forces |

The configured `target_object_name: green_cylinder` names the simulation fixture
whose contacts are observed. It is not a perception label, object selector or
pose input. The task ignores the message's object poses, opposing-contact verdict,
support flag and speed. Those observations remain available for later step-5
retention assessment, which is not implemented here.

`SimGraspFeedback.contact_normal_forces` contains positive normal force in newtons,
one value per `contact_links` entry, summed over contacts on that link. The
simulator computes it with MuJoCo's contact-force API. Rebuild interfaces and
restart all participating containers after this schema change; old feedback
without matching forces is rejected. These are simulated forces, not a calibrated
physical hand force interface.

## Closure sequence

1. Require fresh hand/arm joint samples, a hand near the open posture, a measured
   stationary arm, the hand action server and fresh configured-fixture feedback.
   An unavailable input discards the trigger; nothing is queued.
2. Oppose thumb yaw toward 0.8 rad. Then close still-free fingers and thumb toward
   their configured ceiling angles, at most 0.05 rad per measured step.
3. Use zero-velocity cubic segment endpoints. Segment duration bounds peak speed
   to 0.20 rad/s and acceleration to 0.50 rad/s². Each segment starts at measured
   finger positions. Controller completion alone is insufficient: require fresh
   post-result measurements, detect lack of physical progress and check tracking.
4. When a target contact exceeds 0.15 N, freeze that digit's command with a
   **one-time**, at-most-0.04 rad preload. Subsequent feedback does not ratchet the
   contact position. Thumb contact freezes both thumb actuators. The other digits
   can continue closing. A thumb plus another current finger contact ends free
   closing; this is contact evidence, not proof of retention.
5. In simulation, apply a **single additional flexion preload of at most 0.25 rad**
   over at least 3 s after bilateral contact. Keep thumb yaw at its contact-limited
   value. This finite ramp supplies holding torque with the existing simulated
   hand gains; it does not repeat or increase in response to slipping. Joint
   ceilings, peak speed/acceleration and the 16 N summed normal-force ceiling
   remain active. Contacted joints may lag the preload command by its bounded
   amount plus the normal tracking tolerance. Fresh measured posture and current
   contact are required after completion. Joint-only mode skips this preload.
6. Verify the holding trajectory, then report the outcome. No
   automatic reopening occurs. The hand controller retains its last hold target.

Normal states: `requested`, `opposing_thumb`, `closing`, `preloading_hold` (or
`holding` without bilateral simulation contact), then a terminal
`succeeded` or `failed`. Short intermediate events can pass before a retained
status subscriber connects. Status always carries `request_id`,
`active_request_id`, `feedback_mode` and **`retention_verified: false`**.

A successful step-4 result has `closure_outcome: contact_limited`, measured hold
completion and separate `contact_evidence`, link names and normal-force values.
It does not assert opposing surface normals, sustained load, object stability or
retention. The separate [lift task](../lara5_object_lift/README.md) checks these
before and during a physical test lift.

An explicit `feedback_mode: joint_only` configuration supports measured closure
without simulation feedback. Its success is `closure_outcome: motion_only`, with
contact/retention unverified. It cannot detect an absent object or limit motion
using contact forces, and is not validated for physical hardware gripping.
Default simulation mode never silently falls back to this mode.

## Failure and hold behavior

- A closed initial hand, missing/stale/malformed state, moving arm, or missing
  contact-force feedback reports `rejected_state` before motion.
- Full closure without current finger contact reports `failed` with
  `closure_outcome: no_object_contact`, even if the controller completed motion.
- Stalled measured joints, tracking errors, controller errors, arm movement,
  excessive target normal force or stale feedback terminate the closure sequence.
- During motion, request cancellation and require a terminal controller result.
  Then command a measured-position hold if fresh measurements are available.
  If feedback is lost, use the controller's cancellation hold and explicitly
  report that a measured hold could not be verified. No blind hold pose is sent.
- A cancellation acknowledgment alone does not release ownership. Unknown goal
  delivery, delayed acceptance and unconfirmed stop remain busy; late accepted
  goals are canceled. An unconfirmed stop reports `error` and blocks new triggers.
- Ctrl+C/SIGTERM requests cancellation and waits briefly for confirmation. It
  does not open the hand or command the arm.

The arm is only observed: drift over 0.01 rad aborts hand closure. This node cannot
stop another arm client. Run one hand client at a time and finish positioning
before triggering closure. After any attempt, opening is a separate explicit
reset:

```bash
ros2 run lara5_controller_manager hand_command.py open --duration 3
```

## Configuration

`grasp.launch.py` loads `config/grasp.yaml`. Restart after edits:

```bash
ros2 launch lara5_grasp grasp.launch.py
# Full custom YAML:
ros2 launch lara5_grasp grasp.launch.py config_file:=/path/grasp.yaml
```

| Parameter | Default |
| --- | --- |
| `close_positions` | `[1.1, 1.3, 1.3, 1.3, 1.3, 0.5]` rad ceilings: thumb yaw, index, middle, ring, pinky, thumb pitch |
| `thumb_opposition` | 0.8 rad |
| `max_joint_step` / `contact_preload` | 0.05 / 0.04 rad |
| `hold_preload` / `hold_preload_duration` | 0.25 rad / 3 s, one simulation-only flexion ramp after bilateral contact |
| `max_joint_speed` / `max_joint_acceleration` | 0.20 rad/s / 0.50 rad/s² |
| `min_segment_duration` | 0.25 s |
| `contact_threshold_n` / `max_contact_force_n` | 0.15 / 16.0 N; maximum applies to summed target/hand normal forces |
| `feedback_mode` / `target_object_name` | `simulation` / `green_cylinder` |
| `feedback_max_age` | 0.5 s; receipt and measurement timestamp |
| `open_start_tolerance` | 0.08 rad |
| `joint_tolerance` / `tracking_tolerance` | 0.04 / 0.08 rad |
| `arm_stationary_tolerance` / `arm_stationary_window` | 0.01 rad / 0.20 s |
| `verification_timeout` | 1 s after each controller result |
| `goal_response_timeout` / `segment_timeout_margin` | 5 s / 2 s |
| `closure_timeout` / `cancel_timeout` | 60 s / 5 s |

The former single-shot `duration` parameter is replaced by bounded segment speed,
acceleration and step settings. Force checking is sampled at 20 Hz; this is
contact-limited position control, not a continuous force controller or a guarantee
against brief force peaks. The holder remains visual-only; contact feedback here
covers the configured object against the simulated hand, not all possible
hand/environment collisions. Stalls can therefore indicate self/table contact.

## Build and validation

From the repository root:

```bash
docker build -f src/docker/Dockerfile -t lara5_hackathon:contact-grasp src
```

In a sourced Humble workspace:

```bash
colcon build --packages-up-to lara5_grasp lara5_mujoco_sim
source install/setup.bash
colcon test --packages-select lara5_grasp lara5_mujoco_sim --event-handlers console_direct+
colcon test-result --test-result-base build/lara5_grasp/test_results --verbose
```

Ordinary tests use uniquely remapped synthetic feedback and a fake hand action.
They cover step bounds, non-ratcheting contact holds, no-object behavior,
stationary/open-state requirements, force limits, stale feedback, stalled fingers,
controller failure, busy rejection, late acceptance and unconfirmed-stop locking.
A MuJoCo contact test verifies actual normal-force extraction. Motion integration
is opt-in and must run in a private container/domain without hardware access:

```bash
ROS_DOMAIN_ID=203 ROS_LOCALHOST_ONLY=1 LARA5_SIM_CLOSURE_TEST=1 python3 -m pytest -q -s /ws/src/robot/lara5_grasp/test/test_sim_closure.py
```

For the empty-hand negative case at reset, add `LARA5_SIM_CLOSURE_EMPTY=1`.
The standalone closure test does not lift the object. The separate lift regression
does; neither welds nor teleports it.

The lift-capable preset uses 0.04 rad first-contact preload plus the finite
0.25 rad flexion ramp for the 0.15 kg cylinder, with a 16 N summed contact limit.
These are simulation holding settings, not calibrated physical-hand force
commands. No simulated mass, friction or actuator gain is changed to obtain a lift.
