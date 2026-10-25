# Scene perception packages

The perception layer of [`docs/perception_architecture.md`](../../docs/perception_architecture.md),
meant to run on the Jetson ([`docs/deployment.md`](../../docs/deployment.md)).

| Package | Contents |
|---|---|
| [`restore_interfaces`](restore_interfaces) | `ObjectState.msg`, `SceneSnapshot.msg`, `CaptureScene.srv`, `RegisterObject.srv` (section 8.1) |
| [`restore_perception`](restore_perception) | The pipeline without ROS (`restore_perception/core/`), `scene_perception_node`, `offline_eval`, tests |

## What a snapshot gives you

Every object on the table, in the robot's `world` frame:

- `pose`: the centre of the object, and its yaw about the table normal as a quaternion. Objects are
  assumed to rest on the table or on another object, so there is no roll or pitch. Yaw is the
  direction of the footprint's long side, so it is only defined modulo 180 degrees (90 for a
  square); `symmetry` says which yaw differences matter.
- `size` (length, width, height), `footprint` polygon, `support_id` (`table` or the object below).
- `label` and `label_score` from the gallery (`unknown` without a match), `up_side`, the DINOv2
  `embedding`, a 3 mm point cloud, `cameras_seen`, `depth_reliable` and `confidence`.

## Build

```bash
colcon build --packages-select restore_interfaces restore_perception
```

Python needs NumPy, SciPy, OpenCV and PyYAML; SAM 2.1, DINOv2 (`torch`, `transformers`) and
`pyrealsense2` only when the models or the cameras are used. There is no Open3D: the geometry is NumPy
and SciPy.

## Run

```bash
# Jetson, both D435s, models from /models, gallery and calibration under /data
ros2 launch restore_perception perception.launch.py

# simulation: cameras on /cameras/camera_N/..., camera poses from TF
ros2 launch restore_perception perception.launch.py source:=ros

# replay saved captures without cameras or models
ros2 launch restore_perception perception.launch.py source:=file file_source_path:=/data/snapshots/reference load_models:=false
```

| Launch argument | Default | Meaning |
|---|---|---|
| `source` | `realsense` | `realsense`, `ros` (simulated camera topics) or `file` |
| `config_file` | `config/perception.yaml` | Every parameter of the document; a file may list only what it changes |
| `objects_file` | `config/objects.yaml` | Sizes, symmetry and flags of the eight objects |
| `data_dir` | `/data` | `gallery/`, `snapshots/` and `calibration/extrinsics.yaml` |
| `extrinsics_file` | `<data_dir>/calibration/extrinsics.yaml` | Camera poses from the calibration page |
| `file_source_path` | | For `source:=file`: a snapshot folder, a bench download, or a folder of them |
| `load_models` | `true` | SAM 2.1 and DINOv2; without them every label is `unknown` and masks come from depth |

```bash
ros2 service call /perception/capture_scene restore_interfaces/srv/CaptureScene "{name: reference, save_to_disk: true}"
ros2 service call /perception/register_object restore_interfaces/srv/RegisterObject "{name: sponge, up_side: yellow}"
```

Camera poses: a calibrated fixed camera from the calibration file; a calibrated hand camera from
the file and the flange pose in TF (`world` to `lara5_flange`, from the Ubuntu PC); any other camera,
such as a simulated one, from TF of its image's `frame_id`. The node publishes the calibrated
cameras on `/tf_static`.

Captures named `reference` and saved to disk become the reference of the change mask (section 6) for
later captures.

## Offline evaluation

Runs the pipeline on saved captures and writes overlays and `report.md` (section 11.1):

```bash
cd src/restore/restore_perception
python3 -m restore_perception.offline_eval CAPTURE... --out /tmp/eval [--extrinsics extrinsics.yaml] [--models --gallery /data/gallery]
```

A test bench download (`capture.zip`) of an uncalibrated camera can be tried with `--estimate-pose`:
the table plane in its depth then defines `world`.

## Tests

```bash
cd src/restore/restore_perception
python3 -m pytest test
```

The tests ray-cast scenes of boxes and cylinders seen by a fixed and a hand camera and check the
pipeline against the ground truth: positions within 1 cm, yaw within 3 degrees, sizes within 12 mm,
two touching objects split by SAM prompts, a stacked object, the known-size fallback for an object
without depth, gallery registration and identification, save and replay, and the change mask. SAM
and DINOv2 are replaced by fakes that answer from the ground truth.
