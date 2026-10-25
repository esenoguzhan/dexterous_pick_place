# Perception architecture

> **Status: implemented, not yet run on the robot.** Sections 5 to 8 are implemented in
> [`src/restore`](../src/restore/README.md) (`restore_interfaces`, `restore_perception`). The
> pipeline is tested on ray-cast two-camera scenes and on one real D435 capture. The node is built and
> tested end to end in a ROS 2 Humble container (x86), with saved captures from a fixed and a hand
> camera and with simulated camera topics. Still missing are the Jetson image (`wendy/restore`), the
> simulated MuJoCo cameras, and a run with both real cameras calibrated. The calibration (3.4) and the SAM 2.1 / DINOv2
> test bench live in [`wendy/perception_test`](../wendy/perception_test/README.md). All parameter
> values are starting points to tune in simulation and on the real cameras.
>
> Perception runs on the Jetson Orin Nano, with both cameras plugged into it. See
> [`deployment.md`](deployment.md) for which machine runs what.

## 1. Purpose

Perception turns the two RealSense D435 cameras into a **scene snapshot**: a list of the objects on
the table, each with its identity, pose in the `world` frame, size, footprint, point cloud and
appearance fingerprint.

The rest of the system only consumes snapshots:

| Consumer | Uses from the snapshot |
|---|---|
| Scene diff | Identity, DINOv2 embedding, pose, point cloud, symmetry |
| Task planner | Footprint polygon, support relation (what an object rests on) |
| Grasp planner | Size, shape and grasp primitive (from `objects.yaml`) |
| Supervisor | The capture service, called for the reference, the current scene and after every placement |

The diff, the planners and the supervisor form the orchestrator, described in
[`task_architecture.md`](task_architecture.md).

## 2. Design principles

1. **Snapshots, not tracking.** The scene is captured at a few moments only: the tidy reference,
   the disturbed scene, and after each placement. A capture may take 1 to 3 seconds, so accuracy
   wins over speed.
2. **Geometry first, models second.** A table-plane fit plus clustering finds any object that
   sticks up from the table, with no training. SAM 2.1 then refines outlines and DINOv2 identifies
   objects.
3. **Known objects, generic fallback.** The eight hackathon objects are registered in a gallery
   with their sizes, symmetries and grasps. An object that matches nothing still gets a pose and a
   fingerprint and goes through the generic path.
4. **Relative measurement.** The same cameras measure the reference and the current scene, so a
   constant calibration offset largely cancels out in the displacement the robot executes.
5. **Same code in simulation and on the robot.** The pipeline reads frames through one
   frame-source interface (section 3.2): the real D435s on the robot, the simulated MuJoCo cameras in
   simulation, saved captures in offline tests.
6. **ROS-free core.** The algorithms live in plain Python modules with a thin ROS 2 node on top, so
   the same code runs offline on saved captures and in tests.

## 3. Inputs

### 3.1 Cameras

One D435 is fixed and sees the whole workspace; the other rides on the hand. Both plug into the
Jetson's USB 3 ports; the hand camera's cable runs along the arm.

- **Fixed camera:** angle it roughly 30 to 60 degrees from vertical. Looking straight down at a can
  lid reflects the camera's own infrared pattern back and destroys the depth on metal. About 0.6 to
  1.0 m from the table centre works well.
- **Hand camera:** it captures from the observe pose (5.0), so it sees the scene from the same
  viewpoint every time. The hand covers part of its view; keep the objects in the free part.
- Keep every object at least 0.3 m from each camera (D435 minimum depth range at 848x480).
- Use rigid mounts. A camera that shifts invalidates its calibration (the change mask in section 6
  detects it).
- Add the hand camera to the hand's collision geometry: the safety filter does not know about it.

### 3.2 Frame sources

The pipeline reads frames through one interface with three implementations:

| Source | Used | Reads |
|---|---|---|
| `RealSenseSource` | On the robot | Both D435s directly with `pyrealsense2`, on the Jetson |
| `RosTopicSource` | In simulation | The MuJoCo cameras, published by the Ubuntu PC on the topics below |
| `FileSource` | Offline tests | Captures saved by the test bench or by the pipeline (section 8.3) |

Simulated camera topics, per camera `N` in {1, 2}. They use the RealSense driver's names, so the
vendored ROS driver could also feed `RosTopicSource` if a camera ever moved to a ROS machine.

| Topic | Type | Notes |
|---|---|---|
| `/cameras/camera_N/color/image_raw` | `sensor_msgs/Image` | `rgb8` |
| `/cameras/camera_N/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | `16UC1`, millimetres, on the colour pixel grid |
| `/cameras/camera_N/color/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics `fx, fy, cx, cy` |

A simulated camera has no entry in the calibration file, so its pose comes from TF: `world` to the
`frame_id` of its colour image, published by the simulation.

The real cameras are not published as ROS image streams. Two colour and depth streams at 1280x720
would load the network with no consumer; the perception node publishes debug overlays only when it
captures (section 8.2).

### 3.3 Camera settings

Set in `RealSenseSource`, as [`cameras.py`](../wendy/perception_test/cameras.py) in the test bench
already does:

| Setting | Value | Why |
|---|---|---|
| Colour | 1280x720, RGB8, 15 fps | More pixels on small objects; 15 fps spares the Jetson's CPU |
| Depth | 848x480, Z16, 15 fps | The D435's most accurate depth resolution |
| Laser power | 250 (D435 maximum: 360) | Raise it if the black mouse has holes in its depth |
| Infrared streams | Off | Not used; saves USB bandwidth |
| Depth-to-colour alignment | Only for the frames a capture uses | `rs.align` on every frame costs CPU. On request, align the next 10 framesets. |
| Spatial filter | Optional; try it in the test bench | Edge-preserving smoothing |

No temporal filter: the 10-frame median in 5.1 does that job. The
[`streams.yaml`](../ws/src/neurahackathon_cameras/config/streams.yaml) of the ROS camera workspace
is not used by perception.

### 3.4 Calibration

The camera poses are stored on the Jetson in `/data/calibration/extrinsics.yaml`, keyed by serial
number so that swapping USB ports cannot swap calibrations: the fixed camera as `world` to
`camera_N_color_optical_frame`, the hand camera as `lara5_flange` to `camera_N_color_optical_frame`.
The perception node loads the file at start-up and publishes both as static TF, so TF gives the hand
camera's world pose at any moment and Foxglove shows both cameras.

**Method.** A ChArUco board lies flat on the table where the fixed camera sees it, and the robot moves
the hand camera over it. Each pose gives the flange pose from TF and the board's pose in each camera:

- The hand camera's poses give, in closed form (OpenCV `calibrateRobotWorldHandEye`, Shah's method),
  both the camera's pose on the flange and the board's pose in `world`.
- The fixed camera's pose follows from its view of the same board.
- All unknowns are then refined together by minimising the corner reprojection error in every camera.
  Samples above max(2 px, 3 times the median) are dropped once.
- A partly covered board is fine: every ChArUco corner is identified by its neighbouring markers, and a
  view needs 8 of the 24 corners.

The board can instead be fixed to the hand, which calibrates fixed cameras only.

**Two halves** (how to run it: [the test bench README](../wendy/perception_test/README.md#camera-calibration)):

- **Jetson:** the test bench's calibration page (`/calibration`,
  [`calibration.py`](../wendy/perception_test/calibration.py)) sets the board location and each
  camera's mount, shows live board detection, keeps the samples in `/data/calibration/session.json`,
  solves, and saves the file.
- **Ubuntu PC:** `ros2 run lara5_controller_manager calibrate_cameras.py` reads the flange pose from TF
  at every pose and sends it to the Jetson over HTTP (port 8090). It records while you move the arm,
  or replays saved `robot.joints` from an exported session through the safety filter's public
  `FollowJointTrajectory` action, verifying measured joints before every capture. The explicit
  `auto --random` mode visits poses around a start pose through the safety filter's `MoveToPose`,
  tilting about a point in front of the hand so the hand camera keeps the board in view.
  Until the Jetson runs ROS, this HTTP
  call is the only link the calibration needs.

**Accuracy on synthetic renders** (D435 intrinsics, the hand covering part of the hand camera's view,
22 poses, one of them corrupted and dropped):

| Estimated | Position error | Rotation error |
|---|---|---|
| Hand camera on the flange | 0.6 mm | 0.13° |
| Board in `world` | 0.2 mm | 0.06° |
| Fixed camera in `world` | 2.3 mm | 0.1° |

The fixed camera inherits the board's orientation error, so spread the hand camera's poses widely,
with tilts up to 25 to 30 degrees. With half that spread its rotation error was 0.24 degrees, about
6 mm at 1.4 m.

## 4. Pipeline overview

```mermaid
flowchart TD
  A["Observe pose<br/>arm out of view"] --> B["Capture<br/>10 frames, median depth"]
  B --> C["Back-project<br/>pixels to world, crop"]
  C --> D["Remove table<br/>RANSAC plane"]
  D --> E["Cluster<br/>DBSCAN blobs"]
  E --> F["Refine masks<br/>SAM 2.1 box prompts"]
  F --> G["Fuse views<br/>camera 1 + camera 2"]
  G --> H["Identify<br/>DINOv2 + gallery"]
  H --> I["Describe<br/>pose, size, footprint"]
  I --> S[("Scene snapshot")]
  B -.-> X["Change mask<br/>reference vs current"]
  X -.-> S
```

| Stage | Technique | Main package | Needs training |
|---|---|---|---|
| Observe pose | Fixed joint configuration | Supervisor (Jetson), controllers (Ubuntu PC) | No |
| Capture | Aligned framesets, per-pixel median | `pyrealsense2`, NumPy | No |
| Back-project | Pinhole model + calibration file | NumPy | No |
| Remove table | RANSAC plane fit | NumPy | No |
| Cluster | DBSCAN | SciPy (KD-tree, connected components) | No |
| Refine masks | Box- and point-prompted segmentation | SAM 2.1 (`facebook/sam2.1-hiera-small`) | No |
| Fuse views | Footprint association + outlier removal | SciPy | No |
| Identify | Embedding nearest neighbour against a gallery | DINOv2 (`facebook/dinov2-small`) via `transformers` | No (gallery registration only) |
| Describe | Footprint rectangle on the table plane | OpenCV | No |

## 5. Stages

### 5.0 Observe pose

- **Method:** the supervisor moves the arm to a fixed `observe` joint configuration, chosen once so
  the arm is outside both camera views, and waits until joint speeds are near zero plus 0.5 s.
- **Guard:** the capture service checks `/joint_states`, published by the Ubuntu PC, and refuses to
  capture when the arm is not at the observe pose (parameter `require_observe_pose`). It judges
  freshness by when a message arrives on the Jetson, not by its header stamp, so a clock offset
  between the two machines cannot block captures.
- **Why:** the arm would otherwise be clustered as an object and would hide objects behind it. The
  observe pose also places the hand camera; the arm returns to it precisely, so the hand camera sees
  the scene from the same viewpoint at every capture.
- **Later option:** a robot self-filter that renders the URDF into each depth image and masks the
  arm, so captures work with the arm in view.

### 5.1 Capture

- **Method:** on request, `RealSenseSource` aligns the next 10 framesets of each camera to colour,
  both cameras in parallel (about 0.7 s at 15 fps). Per pixel, take the median depth, ignoring
  zeros. A pixel is valid when at least half of its samples are valid. Colour is the newest frame,
  and the intrinsics come from the colour stream profile. In simulation, `RosTopicSource` collects 10
  messages per camera instead.
- **Output per camera:** colour image, depth in metres, intrinsics, camera serial, timestamp.
- **Why:** D435 depth flickers at object edges and drops pixels. The median removes most of that,
  which keeps object centres stable between captures.

### 5.2 Back-project to the world frame

- **Method:** for each valid pixel `(u, v)` with depth `Z`:
  `X = (u - cx) * Z / fx`, `Y = (v - cy) * Z / fy`. Transform the points to `world` with the
  camera's pose from the calibration file (3.4), then crop to the workspace box. For the hand camera
  that pose is the flange pose from TF at capture time times its calibrated pose on the flange.
- **Keep:** the source `(camera, v, u)` of every point, so 2D masks can select 3D points and 3D
  blobs can be projected back into the images.
- **Pixel stride:** every 2nd pixel in each direction is back-projected (`workspace.pixel_stride`).
  At 1 m that is still a point every 2 mm, finer than the 4 mm clustering voxels, and four times
  fewer points for the Jetson's CPU. Masks stay at full resolution.
- **Workspace box (simulation):** x from -0.6 to 0.6, y from -0.4 to 0.4, z from 0.88 to 1.30 m
  (see [`mujoco_sim.yaml`](../src/robot/lara5_mujoco_sim/config/mujoco_sim.yaml)). Measure the real
  table and set the same parameters for the robot.

### 5.3 Remove the table

- **Method:** voxel-downsample to 5 mm, then RANSAC with a 0.008 m inlier distance: 1000
  three-point hypotheses scored on 5000 random voxels, the best refined by least squares on all its
  inliers. Keep points more than 0.01 m above the plane.
- **Checks:** the plane normal must be within 10 degrees of world up, otherwise the capture fails
  with "table not found or camera moved". A table height more than 0.01 m from the previous capture
  is reported as a warning in the reply.
- **Output:** plane coefficients, table height, the above-table point cloud.
- **Why:** anything resting on a table sticks out of its plane, whatever the object is. A new table
  only needs a new plane fit. The plane also provides the true "up" direction and the surface used
  for placing objects and for objects with bad depth.

### 5.4 Cluster into object candidates

- **Method:** DBSCAN on the above-table points downsampled to 4 mm voxels, with `eps = 0.015 m`; a
  voxel is a core voxel when at least 20 voxels lie within `eps`, itself included. A surface seen at
  4 mm has about 44 voxels within 15 mm, so object surfaces are core and stray edge pixels are noise.
  Drop clusters with fewer than 50 points, an extent above 0.30 m, or a height under 0.015 m.
- **Output:** candidates, each with its 3D points and a padded 2D box (10 px) per camera, obtained
  by projecting the points into that image.
- **Known weaknesses, handled in 5.5:** touching objects merge into one cluster; flat, shiny or
  transparent objects may have too few depth points to form a cluster.

### 5.5 Refine masks with SAM 2.1

- **Method:** per camera, call `SAM2ImagePredictor.set_image` once, then `predict(box=...,
  multimask_output=True)` for each candidate visible in that camera. Choose the output mask with the
  highest IoU with the candidate's projected pixels. A pixel belongs to one object per camera: a
  later mask loses the pixels an earlier one already took. This also joins an object that depth
  split into two clusters (the steel mug), because the second cluster's mask is then empty.
- **Mask to points:** an object's points are the points whose source pixel is inside the mask and
  that lie at least 0.005 m above the table. This removes table bleed and shadows at the edges.
- **Splitting touching objects:** if the chosen mask covers less than 70 % of the candidate's
  projected pixels, the cluster probably holds more than one object. Prompt SAM again with a point
  at the centre of the uncovered pixels, and split the cluster by mask membership. Repeat until
  coverage is reached or no new mask appears.
- **Objects with bad depth:** run `SAM2AutomaticMaskGenerator` on the image. A mask of plausible
  size (0.05 % to 5 % of the image) that overlaps no candidate by more than 10 % becomes a
  depth-poor candidate. For associating cameras, its footprint is its outline projected onto the
  plane 3 cm above the table. Once identified, the known-size fallback (5.8) places it; an
  unidentified one is dropped, since nothing gives it a size. This step is off by default
  (`segment.depth_poor.enabled`): it took 7.6 s per camera on the Orin Nano. Turn it on, perhaps
  for the fixed camera only, if the steel mug or the can lids go missing.
- **Why SAM:** it outlines objects it has never seen, it follows the prompt, so it never has to
  guess which object is meant, and it works on colour, where depth fails (the steel mug, can lids).

### 5.6 Fuse the two camera views

- **Method:** associate per-camera objects with the Hungarian algorithm on their footprints on the
  table: cost `1 - IoU` of the occupied 5 mm cells plus the centre distance in metres. Accept pairs
  with an IoU of at least 0.3 or centres within 0.02 m. Merge the points of each pair, then drop
  statistical outliers (20 neighbours, 2 standard deviations).
- **Why footprints, not 3D centroids:** two cameras see opposite halves of an object, so their 3D
  centroids differ by up to the object's radius (4.5 cm for a lying can seen from both sides). Both
  halves stand on the same footprint.
- **Objects seen by one camera** are kept and marked in `cameras_seen`.
- **Why:** one camera sees the front of an object and the other its back. The fuller shape gives a
  better centre, a better footprint and a better ICP alignment in the scene diff.

### 5.7 Identify with DINOv2 and the gallery

- **Embedding:** per camera, crop the mask's bounding box with 10 % padding, set the background
  outside the mask to grey (127), pad the crop to a grey square so the object keeps its aspect ratio
  at every rotation, resize to 224x224 and run DINOv2. Take the CLS token (384 values
  for the small model) and L2-normalise. Average over the cameras that saw the object and normalise
  again.
- **Gallery score:** for each gallery object, the highest cosine similarity between the detection
  and any of that object's stored views.
- **Assignment:** all eight objects are different, so each gallery object can be used at most once.
  Solve detections against gallery objects with the Hungarian algorithm on `1 - score`.
- **Acceptance:** accept a match when the score is at least 0.6, the margin over the second-best
  object is at least 0.05, and the measured size fits the gallery size within 30 % per dimension
  (any axis order, because an object may lie on a different side). Otherwise the label is `unknown`.
  The size check is skipped for objects marked `depth_reliable: false`, whose measured size is not
  trusted. The reference size is `size_m` from `objects.yaml`, else the size measured at
  registration.
- **Tie-break:** a hue-saturation histogram of the masked pixels, compared with the gallery views
  (Bhattacharyya coefficient). When the score passes but the margin does not, the match is still
  accepted if its histogram is at least 0.1 closer than the runner-up's.
- **Why DINOv2:** its features recognise the same object across viewpoint and lighting changes
  without training, and they separate objects of similar colour.

### 5.8 Describe each object

- **Footprint:** project the object's points onto the table plane and take `cv2.minAreaRect`. This
  gives the centre `(x, y)`, length, width and yaw about the table normal. The footprint polygon is
  stored for the planner.
- **Height:** the 95th percentile of the point heights above the table.
- **Pose:** position at the footprint centre with `z = table height + height / 2`, orientation as
  the yaw about the table normal. The yaw is that of the footprint rectangle's long side.
- **Yaw ambiguity:** a rectangle's yaw is only defined modulo 180 degrees (90 for a square). The
  full orientation of asymmetric objects (mouse, mug handle, bottle cap) is resolved in the scene
  diff, by ICP of the current cloud against the reference cloud.
- **Known-size fallback:** when `depth_reliable` is false for the object, or it has fewer than 200
  points, place a box of the gallery size from the masks instead. Each camera's mask outline is
  projected onto the plane at the object's mid-height, and the projections are intersected like a
  visual hull. The centre and yaw of the intersection give the pose. With one camera the projection
  is stretched away from the camera, but roughly about the centre.
- **Which side is up:** gallery views are tagged with the side facing up (for example the sponge's
  yellow or dark side). The best-matching view's tag gives the side.
- **Support:** an object rests on another when three things hold:
  - its lowest points (1st percentile) are more than 0.015 m above the table,
  - at least 20 % of its footprint lies over the other's,
  - the other's top is within 2 cm of those lowest points.

  Its height then starts there. Otherwise it rests on `table`. The lowest visible point alone is not
  enough, because the lower half of a lying can is hidden from above.
- **Confidence:** combines the SAM score, the gallery score, the point count and the number of
  cameras that saw the object.

## 6. Change mask

The fixed camera does not move and the hand camera always captures from the same observe pose, so
the reference and current images can be compared pixel by pixel.

- **Method:** a pixel has changed when its depth differs by more than 0.01 m, or its colour differs
  by more than a threshold in Lab space. Clean up with a morphological opening and drop regions
  under 200 px.
- **Uses:**
  - An object whose pixels did not change has not moved. The scene diff skips it, which prevents
    false "moved" detections caused by noise.
  - A changed region that contains no detected object can be a person's hand still in the scene:
    wait and capture again.
  - If most of the image changed, the camera was probably bumped: warn and request recalibration.

## 7. Object gallery and `objects.yaml`

### 7.1 Registration

1. Clear the table and place one object on it.
2. Call `/perception/register_object` with the object's name.
3. Rotate the object by hand in steps of about 30 degrees, calling the service for each view, for
   10 to 20 views. Register every resting side that can occur (for the sponge, both sides up).
4. The pipeline runs stages 5.1 to 5.6, expects exactly one object, and stores its crops,
   embeddings, colour histogram and measured size under `gallery/<name>/`.

### 7.2 Per-object configuration

Sizes below are estimates from a photo. Measure the real objects and replace them.

```yaml
# config/objects.yaml
# size_m: [length, width, height] as the object rests in the reference scene
# symmetry: yaw symmetry about the table normal
#   none          every yaw is distinct
#   continuous    yaw does not matter (round, upright)
#   n: <k>        k-fold (2 = 180 degrees, 4 = 90 degrees)
objects:
  tuna_can:
    size_m: [0.085, 0.085, 0.040]
    shape: cylinder_upright
    symmetry: continuous
    grasp: wrap_top
    rolls: false
    depth_reliable: false     # shiny lid
  yellow_bottle:
    size_m: [0.140, 0.090, 0.090]
    shape: cylinder_lying
    symmetry: none            # cap end is distinct
    grasp: wrap_wide
    rolls: true
    depth_reliable: true
    mass_kg: 0.5
  mouse:
    size_m: [0.115, 0.075, 0.045]
    shape: irregular
    symmetry: none
    grasp: wrap_top
    rolls: false
    depth_reliable: true      # black plastic: watch for holes
  steel_mug:
    size_m: [0.110, 0.080, 0.080]   # length includes the handle
    shape: cylinder_upright
    symmetry: none            # handle direction
    grasp: wrap_body
    rolls: false
    depth_reliable: false     # mirror-like metal
  soup_can:
    size_m: [0.110, 0.073, 0.073]
    shape: cylinder_lying
    symmetry: {n: 2}
    grasp: wrap_top
    rolls: true
    depth_reliable: true      # lids are shiny, the label is not
  sponge:
    size_m: [0.090, 0.060, 0.045]
    shape: box
    symmetry: {n: 2}
    up_side_matters: true     # yellow or dark side up
    grasp: pinch_wide
    rolls: false
    depth_reliable: true
  wooden_cube:
    size_m: [0.055, 0.055, 0.055]
    shape: box
    symmetry: {n: 4}
    grasp: pinch_wide
    rolls: false
    depth_reliable: true
  wooden_egg:
    size_m: [0.060, 0.045, 0.045]
    shape: ellipsoid_lying
    symmetry: {n: 2}
    grasp: tripod_top
    rolls: true
    depth_reliable: true
```

### 7.3 Object-specific risks

| Object | Perception risk | Handling |
|---|---|---|
| Steel mug | Mirror-like surface: bad depth, appearance changes with the room | Angled cameras, SAM mask on colour, known-size fallback, several gallery views under venue lighting |
| Tuna can, soup can lids | Specular lids | Angled cameras; the can bodies give good depth |
| Mouse | Black plastic can return weak depth | Higher laser power, median over 10 frames |
| Wooden cube, wooden egg | Same colour as the wooden table | Found by height above the table, not by colour; SAM prompted with the 3D box |
| Sponge | Two colours, deformable | Side-up tags in the gallery |
| Touching pairs (cube and egg, sponge and soup can, mug and soup can) | Merged into one cluster | SAM split (5.5) and the gallery size check (5.7) |
| Bottle, soup can, egg | Roll when placed | Not a perception issue; flagged with `rolls: true` for the motion module |

## 8. Interfaces

### 8.1 Messages (package `restore_interfaces`)

`ObjectState.msg`

```text
uint8 SYM_CONTINUOUS=0
uint8 SYM_NONE=1

string id                          # unique within a snapshot, e.g. "obj_03"
string label                       # gallery name, or "unknown"
float32 label_score                # gallery similarity
geometry_msgs/Pose pose            # world frame; yaw about the table normal
geometry_msgs/Vector3 size         # length, width, height [m]
geometry_msgs/Point[] footprint    # polygon on the table plane, world frame
uint8 symmetry                     # SYM_CONTINUOUS, SYM_NONE, or k for k-fold
bool up_side_matters
string up_side                     # gallery view tag, empty if unknown
string support_id                  # "table" or another object's id
float32[] embedding                # DINOv2, L2-normalised
sensor_msgs/PointCloud2 cloud      # world frame, downsampled to 3 mm
uint8 cameras_seen                 # bitmask: 1 = camera_1, 2 = camera_2
bool depth_reliable
float32 confidence
```

`SceneSnapshot.msg`

```text
std_msgs/Header header             # frame_id: world
string snapshot_id
float64[4] table_plane             # a, b, c, d with a*x + b*y + c*z + d = 0
ObjectState[] objects
```

`CaptureScene.srv`

```text
string name                        # e.g. "reference"; empty for an automatic id
bool save_to_disk
---
bool success
string message
SceneSnapshot snapshot
```

`RegisterObject.srv`

```text
string name
string up_side                     # optional view tag
---
bool success
string message
uint32 views_stored
```

### 8.2 Node `scene_perception_node`

Runs on the Jetson, in the `restore` container ([`deployment.md`](deployment.md), section 5.2).

| Kind | Name | Type |
|---|---|---|
| Reads | Both D435s through `RealSenseSource`; in simulation, the camera topics in 3.2 | Frames |
| Subscribes | `/joint_states` (from the Ubuntu PC) | Observe-pose guard |
| Service | `/perception/capture_scene` | `CaptureScene` |
| Service | `/perception/register_object` | `RegisterObject` |
| Publishes | `/tf_static` | From the calibration file: `world` to a fixed camera's optical frame, `lara5_flange` to the hand camera's |
| Publishes | `/perception/snapshot` | `SceneSnapshot`, transient local |
| Publishes | `/perception/markers` | `visualization_msgs/MarkerArray`: boxes, labels, footprints |
| Publishes | `/perception/debug/camera_N/overlay` | `sensor_msgs/Image`: masks and labels drawn on colour, once per capture |

Models are loaded once at start-up and warmed up with one dummy inference, so the first capture is
not slow. A model that fails to load leaves the node running without it (no SAM: masks from depth;
no DINOv2: every label `unknown`), and every reply says so.

- **Camera poses at capture time:**
  - a calibrated fixed camera: from the calibration file;
  - a calibrated hand camera: the file times the flange pose from TF;
  - any other camera: TF of its image's `frame_id`.
- **Reference:** a capture named `reference` and saved to disk becomes the change-mask reference (6)
  for later captures.
- **Concurrency:** the services run one at a time, in their own callback group, while
  `/joint_states`, TF and the simulated camera topics keep being received.

Launch arguments, parameters and example calls are in the
[package README](../src/restore/README.md).

Only one program can open a camera: the test bench (`wendy/perception_test`) and this node cannot
run at the same time.

### 8.3 On-disk format

```text
data/
  calibration/extrinsics.yaml      # camera poses (3.4)
  snapshots/<snapshot_id>/
    snapshot.json                  # everything in SceneSnapshot except clouds, plus each camera's
                                   # intrinsics and pose, so a capture can be run again offline
    objects/<id>.ply               # per-object point cloud
    camera_N/color.png
    camera_N/depth.png             # 16-bit, millimetres
    camera_N/masks.png             # one label value per object
    camera_N/overlay.png
  gallery/<object_name>/
    view_XX.png
    embeddings.npy                 # one row per view
    histograms.npy
    meta.json                      # measured size, view tags
```

## 9. Code layout

```text
src/restore/
  restore_interfaces/                     # ament_cmake: the messages and services of 8.1
  restore_perception/                     # ament_python package
    package.xml, setup.py
    config/
      perception.yaml                     # all parameters in this document
      objects.yaml
    launch/perception.launch.py
    restore_perception/
      core/                               # no ROS imports
        config.py                         # parameter defaults, perception.yaml and objects.yaml
        types.py                          # dataclasses: CameraFrame, ObjectState, Snapshot
        sources.py                        # RealSenseSource, FileSource (RosTopicSource lives in node.py)
        extrinsics.py                     # calibration file: camera poses by serial number
        capture.py                        # median depth
        geometry.py                       # back-projection, plane, DBSCAN, table frame, rays
        segment.py                        # SAM 2.1 wrapper, splitting, depth-poor objects
        fuse.py                           # cross-camera association
        embed.py                          # masked crops, DINOv2 wrapper, colour histograms
        gallery.py                        # registration, matching
        describe.py                       # pose, height, known-size fallback, support, confidence
        change.py                         # change mask
        pipeline.py                       # the stages in order: frames in, Snapshot out
        overlay.py                        # debug overlays
        snapshot_io.py                    # read and write data/, read test bench downloads
      node.py                             # ROS 2 wrapper
      offline_eval.py                     # run core on saved captures, write overlays and a report
    test/                                 # ray-cast scenes with ground truth

wendy/restore/                           # (to do) Wendy app that runs all our code on the Jetson
  build.stagefile.yaml                    # ubuntu:22.04 + ROS 2 Humble + Jetson PyTorch; builds src/restore
  wendy.json                              # entitlements: host network, GPU, USB, camera, /data, ports
```

`core/` imports neither ROS nor `pyrealsense2`, `torch` or SAM 2 at module level, so `offline_eval.py`
and the tests also run on a PC with saved captures. Captures for offline tests come from the test
bench downloads, or from `capture_scene` with `save_to_disk`, which keeps everything needed to run a
capture again.

## 10. Software stack

Everything below goes into the `restore` image on the Jetson. The versions are the ones the test
bench's [`build.stagefile.yaml`](../wendy/perception_test/build.stagefile.yaml) already builds and
runs, except where marked.

| Package | Version and source | Used for |
|---|---|---|
| Base image | `ubuntu:22.04`, arm64 | Matches ROS 2 Humble and the Jetson wheels |
| ROS 2 Humble | apt `ros-humble-ros-base` (new) | `rclpy`, `tf2_ros`, `launch_ros`, the message packages |
| `foxglove_bridge` | apt `ros-humble-foxglove-bridge` (new) | Visualisation from the Windows PC |
| PyTorch | `torch==2.8.0`, `torchvision==0.23.0` from the Jetson wheel index (CUDA 12.6) | Runs SAM 2.1 and DINOv2 on the GPU |
| `pyrealsense2` | `2.59.1.11053`, aarch64 wheel | Reads the cameras |
| SAM 2.1 | Source at a pinned commit, checkpoint `sam2.1_hiera_small.pt` | Masks |
| DINOv2 | `transformers==4.56.2`, `facebook/dinov2-small` baked into the image | Embeddings |
| OpenCV | `opencv-python-headless==4.10.0.84` | `minAreaRect`, morphology, drawing |
| NumPy | `1.26.4` | Everything |
| SciPy | pip | KD-tree (DBSCAN, outliers), connected components, Hungarian assignment |
| PyYAML | pip | `perception.yaml`, `objects.yaml`, the calibration file |

No Open3D and no Shapely: plane fit, voxel downsampling, DBSCAN and outlier removal are a few dozen
lines of NumPy and SciPy, which avoids a large aarch64 wheel; footprints use OpenCV.

Pitfalls:

- **Pin NumPy 1.26.4 in every pip group.** The groups install into the same tree, so an unpinned
  group pulls NumPy 2 underneath and breaks OpenCV and SciPy (found while building the test bench).
- **Skip `cv_bridge`.** Convert `sensor_msgs/Image` to and from NumPy directly (`rgb8` and `16UC1`
  are simple), so pip's OpenCV is never mixed with the OpenCV that ROS installs from apt.
- **Mind the shared memory.** The Orin Nano shares 8 GB between CPU and GPU. Keep SAM's automatic
  mask generator at 32 points per batch, as the test bench does.
- **Expect slow builds.** Images are built on the Windows PC under ARM emulation; see the test
  bench README for two workarounds.

## 11. Validation

### 11.1 Offline test on real captures (first)

Record about 20 captures from both cameras: with the test bench (the `capture.zip` downloads), or
with `capture_scene` and `save_to_disk`. Include each object alone, and groups arranged like the
real task, including the touching pairs. Run `offline_eval.py` and inspect the overlays and
`report.md`. The test bench answers the three questions below directly on the Jetson, and
`offline_eval.py` answers them with the geometry stages added:

1. Does SAM give a clean mask for the steel mug?
2. Does SAM split the touching pairs?
3. Does the gallery name all eight objects correctly?

### 11.2 Simulation against ground truth

The simulated cameras on the Ubuntu PC publish the topics in 3.2, which `RosTopicSource` reads on
the Jetson, and MuJoCo knows every object's true pose. Run the pipeline over randomised scenes and
measure position and yaw errors per object.

### 11.3 Real repeatability

Capture the same untouched scene ten times. The spread of each object's centre must be well below
the 1.5 cm "moved" threshold used by the scene diff.

### 11.4 Targets

| Metric | Target |
|---|---|
| Objects detected per capture | All 8 in at least 95 % of captures |
| Gallery naming accuracy | At least 95 % |
| Position error (simulation) | At most 1 cm |
| Yaw error, asymmetric objects (simulation, after the diff's ICP) | At most 5 degrees |
| Centre repeatability (real, standard deviation) | At most 3 mm |
| Capture time, both cameras | At most 3 s |

## 12. Fallback: RF-DETR

If the offline test shows that SAM cannot reliably segment the steel mug or split the touching
objects, add a fine-tuned RF-DETR segmentation model for the eight objects. Its training labels can
come from this pipeline: geometry and SAM give the masks on easy captures, and the gallery gives the
names. DINOv2 and the gallery stay in place for identity and for unknown objects.

## 13. Open questions

- Where can the cameras be mounted at the venue? Both cables must reach the Jetson's USB 3 ports.
- Will the judges stand lying objects upright (or the reverse)? That needs reorientation, which is
  outside this design.
- Will the judges add objects that are not in the gallery? The generic path handles them, but
  without names, sizes or symmetry.
- What is the lighting at the venue? Register gallery views under that lighting if possible.
