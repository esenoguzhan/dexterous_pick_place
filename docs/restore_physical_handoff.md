# Restore on the real robot: where we stopped (27 October 2026, evening)

The pipeline runs end to end on the hardware up to the grasp. Perception, planning, collision-checked
motion, the side approach and the wrist-camera correction all work. The grasp itself is not reliable yet,
and `restore.py run` (the full reference, diff, pick and place loop) has not been run on the robot.

## How it is set up

| Where | What |
|---|---|
| aras PC (`aras@192.168.253.69`, hotspot "S", key auth) | Git worktree of `origin/main` at `~/restore_main` (branch `restore-main` there; this repo's branch `restore-physical` holds the same changes). Container `restore_main` (image `lara5_hackathon:main-fa79aed`): `~/restore_main/src` mounted at `/ws/src` and built in `/ws`, the repo at `/repo`, `~/restore_run` at `/run_data` (logs, captures, the `robot_state` tool). |
| Jetson (`169.254.211.202` on the USB-C link) | App `perception-test` running (HTTP 8090), `restore-perception` stopped. Both D435 are read over HTTP, so no DDS crosses the USB-C link. Fixed camera `263522073705` (USB 3.2); wrist camera `262722075125`, now on **USB 2.1** and dropping out when it is reopened. |
| Old setup | `lara5_workspace` (the old code with the Codex helpers) still exists, with its processes stopped. `lara5_sim_main` is stopped. Neither is used. |

Everything for the physical task is in [`tools/restore_physical/`](../tools/restore_physical/README.md).
`~/restore_main` on aras PC has the same files as this branch, copied over as uncommitted changes
(`config/seeds.json` grows there while the arm plans). To put it on the pushed branch:
`git -C ~/restore_main stash && git -C ~/restore_main fetch && git -C ~/restore_main checkout -B restore-physical origin/restore-physical`.
After a change to `src/`, rebuild in the container (`colcon build --packages-select ...`, without
`--symlink-install`) and restart the stack.

```bash
~/restore_main/tools/restore_physical/stack.sh start          # controllers + safety filter (robot mode, scene boxes on)
~/restore_main/tools/restore_physical/rrun.sh status.py       # joints, flange, hand
~/restore_main/tools/restore_physical/rrun.sh restore.py observe
~/restore_main/tools/restore_physical/rrun.sh restore.py move X Y TX TY [--dry]   # one pick and place
~/restore_main/tools/restore_physical/rrun.sh restore.py reference                # record the tidy scene
~/restore_main/tools/restore_physical/rrun.sh restore.py run [--dry]              # restore the moved objects
```

After a protective stop (the status shows `collision/b6=1`), check what was hit, then clear it with the
stack stopped, and restart the stack:

```bash
~/restore_main/tools/restore_physical/stack.sh stop
docker exec restore_main bash -c "LD_LIBRARY_PATH=/ws/build/neura_driver/neura_bundle/usr/lib /run_data/robot_state reset"
~/restore_main/tools/restore_physical/stack.sh start
```

If the arm powers off (`power/b0=0`), power it on at the pendant. The `reset` above then brings it back
to external mode.

## What was wrong, and what was fixed today

1. **The arm did not follow, and later froze or stopped with "collisions" that were not real.** The
   arm trails its servo command by about 0.4 s. The driver dropped every command more than 10° from the
   measured joints, so the arm froze short of its goal while the controller reported success. When
   streaming resumed, the arm jumped about 10°, which tripped the robot's collision detection.
   Fixed in `neura_driver/src/NeuraROSControllers.cpp`: only a jump between successive commands (over
   2°) or a 30° lag is refused. `robot.py` also finishes every move that stops short.
2. **The calibration was made with the old joint-1 zero.** Main has `joint1_zero_yaw = -50°`, so
   `convert_extrinsics.py` moved the 26 October hand-eye calibration into main's frame. Check: the model
   arm projected into the fixed camera image lies on the real arm (`project_model.py`).
3. **The hand mount was wrong in main.** The replacement hand is rolled 180° (`+90°` instead of `-90°`)
   and stands 2.5 cm off the flange: you measured 28 cm from the flange to the middle fingertip, against
   25.5 cm in the model. Now xacro arguments (`hand_mount_yaw`, `hand_mount_z`):
   `controller_manager.launch.py` passes the measured mount in robot mode, and the defaults (and the
   checked-in URDF) keep main's mount for the simulation and its tests.
4. **Perception.** It runs on aras PC on the fixed camera's RGB-D (no SAM or DINO): the workspace is
   the white sheet only, the table plane is refitted by least squares, and each object's axis comes from
   a RANSAC circle fit on its side wall. At the board centre the can was found 3 to 5 mm from the
   ChArUco ground truth; the shiny lid had made the old estimate 1 to 2.5 cm off. Identity: a
   hue-saturation histogram plus height and radius, matched with the Hungarian algorithm (`match.py`).
5. **Collisions with objects.** Every perceived object except the one in hand is sent to the safety
   filter as a box (`allow_scene_updates:=true`), and the filter now refuses paths into them. Every
   planned segment is checked before anything moves.
6. **Side grasp with the wrist camera** (your request). The palm faces the object, the fingers point
   along the approach and the thumb is up. From 15 cm back, the wrist D435 sees the object and its depth
   corrects the axis: on the can it moved it by 27 mm and measured the radius correctly (38 mm, where the
   fixed camera said 40 to 48 mm).

## Status on 28 October: the restore loop works on the robot

`restore.py reference`, then the red can moved 16 cm by hand, then `restore.py run`. The run
recognised the moved can (the mug and the bottle 1 to 4 mm off, so left alone) and approached it from
the side. The wrist depth corrected its axis by 11 mm, and the fingers and thumb closed on it (0.62
to 0.68 rad). The arm lifted it 20 cm, carried it over the others and put it down at home. The final
check found the can 22 mm from home (tolerance 25 mm) and the others 1 to 2 mm off.

What made the grasp work, after 27 October:

- The thumb is turned into opposition (yaw 1.16) before the approach (your suggestion). Swinging it
  in at the grasp jammed it on the lid.
- The palm gap is 12 mm (22 mm missed); fingers and thumb pitch close together.
- After the wrist correction the slide-in is checked again, and the approach is re-planned for the
  corrected position when it is refused (the forearm passed 19 mm from the mug's box).
- After placing, the hand goes straight up. The old low slide back along the fingers headed for the
  table and was e-stopped.

If the robot and its table are moved together under the camera, re-pose the camera from the board:
`board_shift.py before` was saved on 28 October with the board on the table, so move them, then run
`board_shift.py after`. If only the robot base moves, the table model and the sheet area must shift
too.

### Two objects (28 October, later)

The can and the bottle were displaced together, and one `restore.py run` put back both: the can 5 mm
and the bottle 8 mm from home, with the mug, which was not moved, 13 mm off. Four changes:

- IK on the URDF chain (`kin.py`, 0.03 ms per FK) instead of the safety filter's FK service. That
  service refuses near-collision states, so the IK gave up near the table. Planning dropped from
  40-60 s to 5-10 s per object, and far placements solve.
- Tall objects are held 3.5 cm below their top (`grip_below_top_m`), so the arm stays higher on far
  reaches; the bottle's home is 75 cm from the base.
- The place pose corrects the in-hand shift (`held_shift_m`: 15 mm along the palm normal and along
  the fingers), which was measured from the two can placements that ended 22 and 27 mm off.
- Object boxes for the safety filter have a 1 cm margin (it was 2 cm), on top of the filter's 20 mm.
  Parking now happens only when a home is taken.

The arm stops following after a long idle (about 12 minutes without commands, with no fault shown).
Restart the stack before a run (`stack.sh restart`).

## Next steps

1. Place accuracy (22 mm): the object shifts in the hand while gripping. Measure it with the wrist
   camera after the lift, or with the fixed camera before the release, and correct the place pose.
2. Several moved objects at once, homes that are taken (the parking spot logic is untested), and the
   bottle (tall, so it should work like the can).
3. The mug is too short for the side grasp (85 mm; the hand origin is 8.5 cm above the table because
   of the wrist camera). Swap it for a taller matte object, or add a top grasp for short objects.
4. Planning time: 40 to 60 s per object (finite-difference IK over the FK service).

## Tests

Before the mount became an argument, the description-dependent suites ran on aras PC (373 tests):
the 4 hand-geometry failures came from the changed mount, which the defaults now undo; 10 ROS
workflow tests timed out because the live robot stack shared ROS domain 42 in the container. Not
rerun after the last change. Rerun with the stack stopped, or on another `ROS_DOMAIN_ID`.

## Facts worth keeping

- World frame (main's URDF): the arm base is at (-0.32, 0, 0.915). White sheet corners: (-0.04, 0.36),
  (0.53, 0.33), (0.51, -0.36), (-0.07, -0.33). Table plane about z 0.903 to 0.905.
- Observe pose (arm folded up, out of the fixed camera's view of the sheet):
  `[-1.0654, -0.4808, 1.4492, 3.3252, 0.1777, -1.6418]`.
- Hand frame: +z along the fingers (palm centre 0.12 m, fingertips 0.25 m), +y out of the palm, thumb
  at -x. The open thumb's base reaches 5.4 cm out of the palm on the thumb side; the wrist camera hangs
  about 9 to 11 cm out of the palm near the wrist (`config/grasp.yaml`, `camera_box`).
- Planning is slow: IK by finite differences over the safety filter's FK takes 20 to 60 s per pick.
  `config/seeds.json` caches the solutions found.
