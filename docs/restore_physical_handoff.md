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

## Where it stopped

Side grasp attempts on the red can (about 0.26, 0.15):

- With the thumb closing too (yaw 1.16, pitch 0.6), all four fingers wrapped the can and stopped at
  about 0.9 rad. The thumb jammed on the lid, and the arm stopped with a collision flag as soon as the
  lift started.
- With the thumb left open, the hand went to the same corrected position and the fingers closed fully
  without touching the can.

So the can probably sits a little farther from the palm, or further back along the fingers, than
planned. The thumb sweep had pushed it into the fingers in the first attempt.

## Next steps (in this order)

1. Side grasp: reduce `palm_gap_m` (0.022 to about 0.008) in `config/side.yaml` and check that the
   fingers stop on the can (below 1.2 rad) with the thumb open. If that still misses, close the thumb by
   pitch only, or by a small yaw (0.4), after the fingers, as a stop rather than a full sweep.
2. Lift test with the can held: does the robot's collision detection trip from the payload? If it does,
   lift more slowly.
3. `restore.py move` end to end (pick, carry above the others, place, slide out), then
   `restore.py reference`, move one object, then `restore.py run --dry` and `restore.py run`.
4. Wrist camera: re-seat or flip the USB-C plug, or use another cable or port, until the app shows
   `usb 3.2`. On USB 2.1 its colour and depth stream fails to reopen ("Couldn't resolve requests").
5. Objects: the side grasp needs objects at least 10.5 cm tall (the hand origin sits 8.5 cm above the
   table because the wrist camera hangs below it) and 55 to 80 mm wide. The red can and the yellow
   bottle (standing) are fine. The steel mug is too short and too shiny.

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
