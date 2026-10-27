# Restore on the real LARA5 + Inspire hand

Scripts that run in the `restore_main` container on aras PC (`rrun.sh SCRIPT.py`). They use the
controllers from `controller_manager.launch.py` (started by `stack.sh`) and the Jetson's
`perception-test` app over HTTP for both D435s. Status and next steps are in
[`docs/restore_physical_handoff.md`](../../docs/restore_physical_handoff.md).

| File | Role |
|---|---|
| `stack.sh` | Start, stop or restart the controllers (robot mode, hand on, safety filter with scene boxes) |
| `rrun.sh` | Run a script here inside the container |
| `robot.py` | Arm, hand and safety-filter client: checked joint and Cartesian moves that finish when the arm stops short, scene boxes, FK, TF, the arm's machine state |
| `jetson.py` | Fixed and wrist camera captures over HTTP (colour, median depth, intrinsics) |
| `scene.py` | Objects on the white sheet from the fixed camera: restore_perception geometry, table refit, cylinder axis fit |
| `match.py` | Reference to current object assignment (colour histogram, height, radius) |
| `restore.py` | `reference`, `run`, `move`, `observe`: the task |
| `side.py` | Side grasp: plan, wrist-depth correction, approach, close, lift, place |
| `grasp.py`, `hand_shape.py` | Top-grasp geometry against the hand's underside (superseded by the side grasp) |
| `ik.py`, `motion.py` | Numeric IK over the safety filter's FK, IK seeds, hand headings |
| `convert_extrinsics.py`, `check_board.py`, `project_model.py`, `measure_hand.py` | Calibration conversion and checks |
| `neura/robot_state.cpp` | Read-only arm status; `reset` clears errors and powers on (stack stopped) |
| `config/` | `extrinsics.yaml` (main's frame), `scene.yaml`, `grasp.yaml`, `side.yaml`, IK seeds, the hand's underside |

The other scripts (`jog.py`, `hover.py`, `grasp_debug.py`, `close_lift.py`, `pose_error.py`,
`thumb_probe.py`, and so on) are single hardware checks used while bringing this up.
