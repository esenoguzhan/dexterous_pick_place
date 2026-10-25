> For the current single-command startup and two-terminal workflow, see
> [grasp simulation bringup](grasp_bringup_simulation.md). The separate-node
> instructions below remain useful for individual task testing.

# Step 4: independent contact-limited closure in visible simulation

Run the terminals below in order on your Linux desktop. Input remains a manually
supplied correct object-centre pose; no perception or camera-based control runs.
The hand closure node is independent and never triggers approach or arm movement.

Use a fresh simulation for the known green cylinder. The new feedback schema and
closure logic require the **contact-grasp image and a new container**.

## Terminal 1: build, new Docker container and MuJoCo

On host:

```bash
cd ~/Documents/hackathon/neurahackathon
docker build -f src/docker/Dockerfile -t lara5_hackathon:contact-grasp src
xhost +SI:localuser:root
docker run --rm -it \
  --name lara5_contact_grasp_gui \
  --network host --ipc host \
  -e ROS_DOMAIN_ID=44 \
  -e DISPLAY="$DISPLAY" \
  -e QT_QPA_PLATFORM=xcb \
  -e QT_X11_NO_MITSHM=1 \
  -e MUJOCO_GL=glfw \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  lara5_hackathon:contact-grasp bash
```

Inside:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_controller_manager controller_manager.launch.py deployment_mode:=simulation sim_headless:=false rviz:=false hands:=on safety:=on sim_camera_backend:=egl
```

Leave running. Wait for the viewer, active controllers and `Safety filter ready`.
Use only one backend on domain 44; other domains should not share these nodes.

## Terminal 2: approach task and RViz

On host:

```bash
docker exec -it lara5_contact_grasp_gui bash
```

Inside:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_object_approach approach.launch.py rviz:=true
```

Leave running. RViz paths/static hand appear after the object pose is supplied.
MuJoCo shows the actual robot and object motion.

## Terminal 3: independent closure task

On host:

```bash
docker exec -it lara5_contact_grasp_gui bash
```

Inside:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch lara5_grasp grasp.launch.py
```

Leave running. Starting this node does not move fingers or the arm.

## Terminal 4: explicit positioning and closure commands

On host:

```bash
docker exec -it lara5_contact_grasp_gui bash
```

Inside, open the hand explicitly and wait for completion:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 run lara5_controller_manager hand_command.py open --duration 3
```

Supply the known green-cylinder centre and -90 degree yaw once:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/approach/target geometry_msgs/msg/PoseStamped '{header: {frame_id: world}, pose: {position: {x: 0.25, y: -0.20, z: 0.95}, orientation: {z: -0.7071067811865476, w: 0.7071067811865476}}}'
ros2 topic echo /arm/approach/status --qos-reliability reliable --qos-durability transient_local --full-length
```

Wait roughly two minutes for `succeeded` and `ready_for_closure: true`. Press
Ctrl+C in **Terminal 4** to stop only the status echo. Inspect the hand/object in
MuJoCo; leave the arm in this final positioning pose. Then explicitly request
closure:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /arm/grasp/target std_msgs/msg/Empty '{}'
ros2 topic echo /arm/grasp/status --qos-reliability reliable --qos-durability transient_local --full-length
```

Watch `opposing_thumb`, `closing` and `preloading_hold` (or `holding`). Terminal 3
logs the sequence. The current lift-capable preset adds one finite 0.25 rad
simulation flexion ramp after bilateral contact; the historical force measurements
below describe the earlier closure-only preset.
The arm stays in place while the thumb/fingers close. Closure is slower than the
previous blind three-second command because it uses bounded measured steps.
Allow up to 60 seconds; additional triggers are discarded while busy.

A completed contact-limited motion reports `state: succeeded`,
`closure_outcome: contact_limited`, `controller_motion_completed: true` and
separate current contact link/force evidence. **`retention_verified` stays false.**
Step 5 will assess sustained opposing contacts and object stability.

If it reports `failed`, inspect the full message. Missing contact, stalled joints,
force limit or feedback loss must not be interpreted as a successful grasp.
A fully closed hand alone is insufficient. The node never reopens automatically.

## Inspect contact data separately

After Ctrl+C stops the status echo in Terminal 4:

```bash
ros2 topic echo /sim/grasp_feedback --filter "m.object_name == 'green_cylinder'" --once
```

Feedback is published separately for each scene object. The filter above selects
only the green cylinder; an unfiltered `--once` can show the red cube first. Its
`contact_normal_forces` entries align with `contact_links` and use newtons.
These are simulation observations, not perception-selected input poses.

To reset the hand after the task reaches a terminal result:

```bash
ros2 run lara5_controller_manager hand_command.py open --duration 3
```

Opening may release or displace the object. Restart a fresh simulation before
reusing the fixed manual pose if the object moved.

## Stop

Ctrl+C Terminal 3 first to cancel/stop any hand motion; wait for its result.
Ctrl+C Terminal 2 next to stop/cancel the approach task. Finally Ctrl+C Terminal 1
and exit the container shells. On host:

```bash
xhost -SI:localuser:root
```

An unconfirmed controller stop reports `error` and blocks new commands. Do not
assume restarting a client stops a separately running controller.

## Validation and captured result

The Humble image build passed **160 ordinary checks**, with five opt-in checks
skipped. The actual MuJoCo cylinder sequence and empty-hand negative case were
then run explicitly in isolated containers/domains without hardware access.

The cylinder trial executed positioning, then an explicit independent closure.
Closure took approximately 26.8 seconds and stopped with thumb/index
contact evidence and a verified measured hold. Final normal forces were
approximately 0.60 / 0.54 N. Measured arm drift during closure was 0.000081 rad.
The cylinder remained a free physics body on the table: no attachment, weld,
teleportation or lift was used. These observations do not establish retention.

The empty-hand trial completed its motion but correctly reported
`failed`, `closure_outcome: no_object_contact`, `controller_motion_completed: true`
and `contact_evidence: false`. It held position rather than reopening.

[Recorded results](contact_closure_review.json) include the final image ID,
request events and force observations. Final simulated RGB views:
[external view](images/contact_closure_external_camera.png) and
[wrist view](images/contact_closure_wrist_camera.png). They were captured for
inspection only and never used to control positioning or closure.

## Viewer feedback timing fix

A reported visible-UI test stopped during thumb opposition with
`Stale simulation contact/force feedback`. Positioning had succeeded; closure
canceled before contact. The previous simulator stepped physics, published
contacts and waited for `viewer.sync()` in the same loop. Slow rendering could
therefore stop new contact measurements.

The viewer now runs in a separate spawned process with its own model/data. It
receives compact latest-state snapshots through a nonblocking buffer; rendering
and viewer locks cannot hold up physics or contact publication. Contacts are now
scheduled by wall time after completed physics steps, instead of every 25 steps.
This preserves fresh feedback when physics runs below its nominal 500 Hz. The window still
supports orbit/zoom. It is an observation view; simulation commands remain ROS
commands. Closing its window leaves physics running.

The GUI regression reproduced the reported six-segment failure before the
wall-time scheduling change. It then completed thumb/index contact closure with
MuJoCo, RViz and both cameras active.

The 0.5-second freshness requirement remains unchanged. Measurements are never
restamped to hide a stalled producer, and genuine feedback loss still cancels
closure. Stale errors now include measurement age, receive gap and configured
limit. Use a new container from the rebuilt `contact-grasp` image for this fix.

### Passing visible-UI regression

The regression was run with native MuJoCo and RViz windows on a private Xvfb
screen, two EGL cameras active and software graphics enabled. The GUI workflow
completed thumb/index contact closure. Its maximum green-cylinder message gap
was 0.169 s and maximum received measurement age was 0.108 s. The reproduced
failure before wall-time scheduling had a 0.508 s gap and canceled after six
thumb segments. The 0.5 s limit was unchanged in both runs.

The rebuilt production image passed **167 ordinary checks** (six opt-in checks
skipped). Additional viewer regressions prove a blocked/unresponsive display
cannot block physics and that its process shuts down cleanly. Genuine missing
contact feedback still fails the independent closure tests.

[GUI timing/result record](contact_closure_ui_review.json),
[final wrist RGB view](images/contact_closure_gui_wrist.png) and
[final external RGB view](images/contact_closure_gui_external.png) were saved from
this run. These images are observations of the physical simulation with the UI
active; they are not used for control and do not verify retention.

For maintainers, the visible regression is opt-in and needs a private display,
container/domain and no hardware connections:

```bash
ROS_DOMAIN_ID=206 ROS_LOCALHOST_ONLY=1 LARA5_SIM_CLOSURE_TEST=1 LARA5_SIM_CLOSURE_VIEWER=1 python3 -m pytest -q -s /ws/src/robot/lara5_grasp/test/test_sim_closure.py
```
