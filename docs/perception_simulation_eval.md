# Perception in simulation: how accurate is the pose estimation?

Runs `restore_perception` on the MuJoCo cameras and scores every estimated object against the
simulator's true poses. Everything below ran in the robot Docker image on Ubuntu 22.04 / ROS 2
Humble; the perception packages are built into that image since this document was added.

Three pieces make it work without touching the perception pipeline itself:

| Piece | What it does |
|---|---|
| `/sim/objects` (`std_msgs/String`, JSON, 10 Hz) | The simulator publishes the true pose, dimensions and yaw symmetry of every free object. `sim_objects_file:=...` swaps the object layout. |
| [`perception_sim.launch.py`](../src/restore/restore_perception/launch/perception_sim.launch.py) | Starts `scene_perception_node` with `source:=ros` and remaps `/cameras/camera_1/...` to `/sim/external_camera/...` (and `camera_2` to the wrist camera with `wrist:=true`). Camera poses come from TF. Models are off: every label is `unknown`, masks come from depth. [`perception_sim.yaml`](../src/restore/restore_perception/config/perception_sim.yaml) shortens the capture to 3 frames and keeps the arm's mounting plate out of the workspace box. |
| `ros2 run restore_perception sim_eval` | Optionally parks the arm, calls `/perception/capture_scene` N times, matches estimates to true objects by horizontal distance and writes `report.md`, `results.csv`, `truth.json` and the overlay image of every capture. |

The layout [`perception_eval_objects.yaml`](../src/robot/lara5_mujoco_sim/config/perception_eval_objects.yaml)
puts five objects inside the fixed camera's view: the three MuJoCo props, an elongated box at
-35 degrees to test the yaw of a long side, and a tuna-can sized cylinder. The default layout of
`mujoco_sim.yaml` leaves only the red cube in view, the other two sit just below the image.

## Terminal 1: simulation with the evaluation objects

On the host, from the repository root, with the usual GPU and display flags (see the README):

```bash
docker run --rm -it --name lara5_perception_sim --network host --ipc host \
  --device nvidia.com/gpu=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ROS_DOMAIN_ID=42 -e DISPLAY="$DISPLAY" \
  -e QT_QPA_PLATFORM=xcb -e QT_X11_NO_MITSHM=1 -e MUJOCO_GL=glfw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  lara5_hackathon:latest bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
ros2 launch lara5_controller_manager controller_manager.launch.py \
  deployment_mode:=simulation sim_headless:=false rviz:=true sim_camera_backend:=egl \
  sim_objects_file:=$(ros2 pkg prefix lara5_mujoco_sim)/share/lara5_mujoco_sim/config/perception_eval_objects.yaml
```

Leave it running. `sim_headless:=true` works as well; the evaluation does not need the window.

## Terminal 2: perception on the simulated camera

```bash
docker exec -it lara5_perception_sim bash
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
ros2 launch restore_perception perception_sim.launch.py
```

Expected log line: `ready: source ros (ok), 0 calibrated camera(s), gallery empty, models off`.
Add `wrist:=true` to also read the wrist camera; the arm then has to look at the table.

## Terminal 3: capture and score

```bash
docker exec -it lara5_perception_sim bash
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
ros2 run restore_perception sim_eval --captures 5 --observe-joints 0 0 0 0 0 0 --out /tmp/perception_sim/eval
```

`--observe-joints 0 0 0 0 0 0` first sends the arm straight up through the safety filter. At its
start pose the hand hangs in the camera view and is reported as an object; parked, it is outside
the view and above the workspace box. Leave the option out to test with the arm where it is.

Each capture prints one line per true object with its horizontal error, height error, yaw error
(where the shape defines a yaw) and size error, plus a warning for every unmatched estimate with
its position and size. The report and the overlay images land in `--out`. Copy them to the host
with `docker cp lara5_perception_sim:/tmp/perception_sim/eval .` before the container exits.

Useful options: `--radius` (match radius in the table plane, default 80 mm), `--captures`,
`--pause` between captures, `--timeout`.

## Result of the reference run

Five captures, fixed camera only, arm parked, 27 October 2026:

| Object | Detected | XY error mean / max [mm] | Z error mean [mm] | Yaw error mean / max [deg] | Size error L / W / H [mm] |
|---|---|---|---|---|---|
| red_cube | 5/5 | 1.0 / 1.1 | +3.0 | 1.6 / 2.2 | -0.2 / -1.7 / -4.6 |
| green_cylinder | 5/5 | 1.4 / 1.4 | -6.4 | - | +13.9 / -4.1 / -5.7 |
| blue_ball | 5/5 | 13.4 / 13.4 | +0.4 | - | -1.5 / -17.9 / -27.3 |
| orange_box | 5/5 | 1.2 / 2.3 | +7.6 | 2.5 / 6.1 | -0.7 / -1.2 / -13.8 |
| grey_can | 5/5 | 1.1 / 1.5 | +4.2 | - | -1.7 / -3.9 / -6.4 |

No false positives. Overall XY mean 3.6 mm, yaw mean 2.1 degrees, which meets the targets of
[`perception_architecture.md`](perception_architecture.md) section 11.2 (1 cm, 5 degrees) for
everything except the ball.

How to read it:

- **Flat-topped objects are placed within 1 to 2 mm** horizontally from one camera 1.1 m away.
  The yaw of a box's long side is within a few degrees; the square cube is compared modulo 90.
- **Heights are underestimated by 5 to 14 mm**, which shifts the centre up by half of that.
  One camera sees the top face and one or two sides; the bottom edge of the mask is cut by the
  table-bleed threshold, and the sides' lowest points are hidden by the object itself.
- **The ball is 13 mm off and 27 mm too short.** Perception models a footprint plus a height from
  the point cloud. A sphere shows only its upper cap to the camera, so the visible centroid is
  pulled toward the camera and the measured height is the cap. Round objects that matter on the
  real table (the egg, the lying bottle) will show the same bias.
- **Z errors are consistent** across captures (the simulated depth has no noise), so repeated
  captures do not average them out. Camera noise on the real D435 adds on top.

What this run does not tell you: the real D435's depth noise, holes on dark or shiny objects,
calibration error of the camera pose, or label quality, since SAM 2.1 and DINOv2 are off.
`load_models:=true` on the launch turns them on when the weights are available under `/models`.

## Files

- [`lara5_mujoco_sim/ground_truth.py`](../src/robot/lara5_mujoco_sim/lara5_mujoco_sim/ground_truth.py): the JSON on `/sim/objects`; `yaw_deg` is an optional key of a free object.
- [`restore_perception/core/sim_truth.py`](../src/restore/restore_perception/restore_perception/core/sim_truth.py): matching, errors, summary and report (no ROS, unit-tested).
- [`restore_perception/sim_eval.py`](../src/restore/restore_perception/restore_perception/sim_eval.py): the ROS command.
