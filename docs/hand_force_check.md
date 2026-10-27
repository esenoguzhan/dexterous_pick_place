# Physical hand closure validation

This implements step six after the depth, object-reference, camera-alignment,
tracked-approach and handoff steps. The closure itself is unchanged: thumb
opposition, small measured segments, per-digit contact stops, cancellation and a
measured hold. This step adds two things.

- A read-only **force check** that records the hand's raw force channels. It
  covers the open hand, free-air motion and known fingertip loads. It then checks a
  candidate baseline, threshold and limit configuration against those recordings.
- Stricter closure evidence. Success needs several **new** physical measurements,
  and every measurement is checked against the force limits.

Success still means a force-supported closure. `retention_verified` stays false
and nothing lifts the object.

## What changed in the closure

| Before | Now |
| --- | --- |
| Stability counted elapsed time since the first bilateral observation. Re-checking one still-fresh cached sample at 50 Hz could fill the 0.25 s interval. | Stability needs at least `hardware_contact_stable_samples` (5) bilateral measurements with distinct source times. Together they must span `hardware_contact_stable_time` (0.25 s), with no intervening non-bilateral reading or measurement gap. |
| Limits were checked only on the latest measurement when the 50 Hz timer ran. A reading between ticks was never checked. | Every received measurement is checked against the limits. A single over-limit reading latches a fault and stops the closure. |
| The driver recomputed each cached republish's stamp from two clock reads. Repeats could differ by fractions of a microsecond. | The driver converts the stamp once per physical read, so republishes repeat it exactly. Readers treat stamps within 1 ms as one read, which also covers older drivers. |

Every hardware terminal status now includes `force_evidence`: the count of new
bilateral measurements, their source-time span, and the required values.

Fingers occlude the wrist camera once closure starts. This is expected. After the
[handoff](wrist_closure_handoff.md) the wrist task no longer evaluates the camera.
Closure uses only physical hand feedback and the accepted approach result.

## What the force check can and cannot decide

| Evidence | Decides | Does not decide |
| --- | --- | --- |
| Unloaded open recording | Baseline per channel and its noise | Whether any contact is detectable |
| Free-air motion recording | Force readings caused by motion alone. Thresholds must clear these, or free closing looks like contact. | Maximum forces |
| Loaded recordings (one per pressed channel) | Whether a known fingertip load crosses the threshold; an approximate raw-per-newton scale | Safe maximum forces for the object or hand |

**Maximum forces remain your decision.** Base them on the loaded evidence and on
what the object and hand tolerate. Unloaded data cannot set them. The report
always keeps `force_limits_validated`, `closure_ready` and `retention_verified`
false. `hardware_force_limits_confirmed: true` remains the operator's explicit
confirmation in the hardware config.

The tool subscribes only to `/right_hand/measured_state`. It publishes only its
own status and report. It has no action or service clients. You move the hand
yourself with `hand_command.py`, with the arm stationary and the hand clear of
people, objects and the camera holder. Keep the e-stop in reach.

## Procedure

Use the container and backend from the [hardware guide](wrist_grasp_hardware.md)
(Terminals 1 and 2). The camera and wrist tasks are not needed. Do not start a
closure (`/arm/grasp/target`) during these recordings. Rebuild the image first:
this step changes the hand driver and the `lara5_grasp` package.

Optionally zero the force sensors first, with the hand fully open and touching
nothing. This takes up to 6 s. Recalibration changes the baseline, so make every
recording afterwards, and repeat them after any later calibration.

```bash
ros2 run lara5_controller_manager hand_command.py open --duration 5
ros2 service call /right_hand/calibrate_force_sensors std_srvs/srv/Trigger {}
```

**Terminal A: force check.** It compares against the same YAML the closure loads:

```bash
docker exec -it lara5_wrist_hardware bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_grasp force_check.launch.py grasp_config_file:=/config/hardware.yaml
```

**Terminal B: commands and live view.**

```bash
docker exec -it lara5_wrist_hardware bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 topic echo /arm/grasp/force_check/status --qos-reliability reliable --qos-durability transient_local --field data
```

The status shows each channel's position, raw force, deviation from the candidate
baseline, and whether that deviation crosses the candidate threshold. It reports
`stale_hand` or `invalid_hand_data` for missing, stale, backwards or malformed
measurements. Stop echo with Ctrl+C before running the next commands.

### 1. Unloaded, open and stationary

```bash
ros2 run lara5_controller_manager hand_command.py open --duration 5
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/grasp/force_check/record_unloaded_open std_msgs/msg/Empty '{}'
```

This ends automatically after 3 s of new measurements. It requires the hand open
within 0.08 rad and stationary within 0.02 rad.

### 2. Unloaded free-air motion

Start recording, then reproduce the closure order in free air: thumb opposition,
close, reopen. `hand_command.py` takes **thumb_yaw thumb_pitch index middle ring
pinky**, which differs from the config order. The `close` preset equals the
default `close_positions`. If you changed `close_positions`, pass your six values
in this order instead.

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/grasp/force_check/record_unloaded_motion std_msgs/msg/Empty '{}'
ros2 run lara5_controller_manager hand_command.py 0.8 0 0 0 0 0 --duration 4
ros2 run lara5_controller_manager hand_command.py close --duration 8
ros2 run lara5_controller_manager hand_command.py open --duration 8
ros2 topic pub --once /arm/grasp/force_check/stop std_msgs/msg/Empty '{}'
```

Each joint must reach 90% of its `close_positions` ceiling. Its peak speed must
also reach the closure's nominal peak segment speed (0.043 rad/s with the
template), measured over 0.2 s windows. The suggested durations exceed both.
The closure's stop-and-go segments have a different acceleration profile from
this continuous motion. Treat the recording as an estimate, then confirm with the
first real closure below.

### 3. Loaded fingertips, one channel at a time

Hold the hand stationary and partly flexed, so a press loads the actuator rather
than the open end stop:

```bash
ros2 run lara5_controller_manager hand_command.py 0.8 0.25 0.6 0.6 0.6 0.6 --duration 5
```

For each channel, select it and optionally enter the force-gauge reading at the
peak, in newtons (a float such as `2.0`; zero means unknown). Then record. Press
the fingertip pad in the direction an object would push it, toward opening. Use
a load you would accept on the object, hold for about two seconds, release, then
stop:

```bash
ros2 param set /hand_force_check loaded_channel right_index_finger_proximal
ros2 param set /hand_force_check loaded_reference_n 2.0
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/grasp/force_check/record_loaded std_msgs/msg/Empty '{}'
# press, hold, release
ros2 topic pub --once /arm/grasp/force_check/stop std_msgs/msg/Empty '{}'
```

Repeat for `right_middle_finger_proximal`, `right_ring_finger_proximal`,
`right_pinky_finger_proximal` and `right_thumb_finger_proximal_pitch`. Optionally
also record `right_thumb_finger_proximal_yaw`. Each digit needs loaded evidence
on at least one of its channels. Both thumb channels count for the thumb. The
pressed digit may yield by at most 0.05 rad. Settings cannot change during a
recording. Repeating a channel replaces only that channel's recording.

### 4. Read the report, choose settings, re-check

```bash
ros2 topic echo /arm/grasp/force_check/report --qos-reliability reliable --qos-durability transient_local --once --full-length --field data
```

In `assessment.channels`, for each channel:

- `suggested_minimum_threshold_raw` is 1.5 times the worst unloaded deviation
  from the measured open baseline, plus 2 raw units.
  `suggested_baseline_raw` lists the open means.
- `threshold_clears_unloaded` checks the *configured* threshold against unloaded
  deviation from the *configured* baseline. A stale baseline therefore uses up
  margin.
- `contact_detectable` reports whether the recorded load crossed the threshold.
- `loaded_reached_limit` reports whether that load would already abort a closure.
- With a reference load, `raw_per_newton_approx`, `threshold_n_approx` and
  `limit_n_approx` give a one-point approximation. It applies to that channel and
  press location only.

Edit `/config/hardware.yaml` on the host and set all six values in each list,
in config joint order. Then re-evaluate the stored recordings without
re-recording:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/grasp/force_check/reload std_msgs/msg/Empty '{}'
```

A malformed file sets `reload_error` and keeps the previous settings. Continue
until `configuration_consistent_with_measurements: true` with empty `reasons`.
Then save the report on the host:

```bash
docker exec lara5_wrist_hardware bash -c 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 topic echo /arm/grasp/force_check/report --field data --qos-reliability reliable --qos-durability transient_local --once --full-length' | sed '/^---$/d' > hand_force_report.json
```

| Reason | Meaning |
| --- | --- |
| `candidate_force_settings_invalid` | Zeros, a missing channel, or a threshold not below its limit |
| `unloaded_open_missing_or_failed` / `unloaded_motion_missing_or_failed` | Record again; see that recording's own `reasons` |
| `threshold_within_unloaded_margin:<joint>` | Free motion or noise could look like contact. Raise the threshold or correct the baseline. |
| `no_loaded_evidence:<digit>` | No usable loaded recording for that digit |
| `contact_not_detectable:<digit>` | The recorded load stayed below the threshold. Lower the threshold only if it still clears unloaded motion. |
| `slower_than_closure`, `incomplete_closure_range` | The free-air motion did not represent the closure |
| `hand_not_open`, `hand_moved`, `insufficient_new_measurements` | The recording conditions were not met |

### 5. Confirm and validate with the real closure

Set `hardware_force_limits_confirmed: true` only after a consistent report and a
deliberate choice of limits. Restart the hardware tasks. Then run the normal
sequence from the hardware guide: robot-UI handoff, approach, and closure on the
test object. Keep the force check running to watch live forces.

Check the closure's terminal `/arm/grasp/status`:

- `force_evidence`, `actuator_forces_raw` and `current_contact_digits`;
- the margin between holding forces and your limits.

A `failed` result citing new physical measurements or a limit is a valid outcome
to investigate. Do not loosen the rules to pass. Opening is a separate command
with the arm stationary and the object supported.

## Validation scope

On 2026-10-27, `inspire_dftp_e2`, `lara5_safety_msgs`, `lara5_grasp` and
`lara5_wrist_grasp` built in an isolated Humble container with no network or USB
access. Three consecutive full runs passed:

- **lara5_grasp:** 90 tests, plus 1 existing opt-in skip.
- **lara5_wrist_grasp:** 121 tests.

They had no errors or failures. The driver's lint suite still shows the same 43
pre-existing flake8 findings in its bundled GUI utilities, and no new ones.

Two mutation checks confirmed the new regression tests catch the old weaknesses:

- Without the new-measurement rule, the cached-measurement test fails.
- With limits checked only on the timer-sampled measurement, a single over-limit
  reading went undetected in 3 of 3 runs, and the test failed.

The real wrist and hand tasks completed a closure with the camera lost after the
handoff. The force-check launch loaded the hardware template, exposed only status
and report publishers, and exited cleanly on SIGINT.

No physical hand, force gauge or robot was used. Synthetic recordings do not
establish real channel behavior, thresholds, limits or object retention. The
recordings above are the next physical step.
