# Neurahackathon

This repository contains the robot code from `main` and the two-camera workspace:

- [`src/`](src/README.md): LARA5 arm, Inspire hand, robot descriptions, and MuJoCo
  simulation. Follow its README for the supported **Ubuntu 22.04 / ROS 2 Humble**
  environment and robot build instructions.
- [`ws/`](ws/src/neurahackathon_cameras): standalone camera workspace, built here
  with **Ubuntu 24.04 / ROS 2 Jazzy**. Its setup is documented below.

The merge preserves the existing robot source layout. The camera build below
does not build or validate the robot stack on Jazzy.
Robot meshes and the NEURA dependency archive use Git LFS. Install Git LFS on a
new machine, run `git lfs install`, and run `git lfs pull` after cloning this branch
to download those assets. The vendored RealSense models remain ordinary Git files.

## Multi-camera workspace

ROS 2 Jazzy workspace for two USB RealSense D435 cameras. Each camera publishes
color, depth, left infrared (`infra1`), and right infrared (`infra2`) images.

## Source layout

```text
ws/src/
  realsense-ros/            # Upstream driver 4.58.4, copied into this repository
  neurahackathon_cameras/   # Shared stream parameters and dual-camera launch
    config/streams.yaml
    launch/dual_realsense.launch.py
```

The driver is pinned to commit `9a11121700cb4780e273e34141f6402fe184321d`, matching
the ROS driver version installed on the original machine. See
[`UPSTREAM.md`](ws/src/realsense-ros/UPSTREAM.md) for provenance and SDK requirements.

## Set up and build

Use Ubuntu 24.04 with ROS 2 Jazzy and source its environment. Install the Jazzy
RealSense SDK package and development tools if needed:

```bash
sudo apt update
sudo apt install ros-jazzy-librealsense2 python3-colcon-common-extensions python3-rosdep ros-jazzy-rqt-image-view
```

Clone this branch and install the workspace dependencies. If rosdep has never
been initialized on your machine, run `sudo rosdep init` once before `rosdep update`.

```bash
git clone --branch multi-camera-config git@github.com:arasf/neurahackathon.git
cd neurahackathon/ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --packages-up-to neurahackathon_cameras \
  --parallel-workers 2 --allow-overriding realsense2_camera realsense2_camera_msgs \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The SDK must provide librealsense **2.58.0 or newer**; this machine uses 2.58.4.
The build command builds the copied driver, its messages, and our launch package.
Build/install/log directories and recordings are excluded from Git.
For zsh, use the corresponding `setup.zsh` files instead of `setup.bash`.

## Run both cameras

Close other RealSense driver launches or applications using these cameras first.
Connect each camera to a suitable USB 3 port and check serial numbers:

```bash
rs-enumerate-devices -s
ros2 launch neurahackathon_cameras dual_realsense.launch.py
```

Default assignments:

| ROS node | Camera serial |
| --- | --- |
| `/cameras/camera_1` | `263522073705` |
| `/cameras/camera_2` | `262722075125` |

Friends using different hardware can override the serials without editing code:

```bash
ros2 launch neurahackathon_cameras dual_realsense.launch.py \
  camera_1_serial:=FIRST_SERIAL camera_2_serial:=SECOND_SERIAL
```

Use the numeric serials as shown by enumeration; the launch file explicitly
passes them as strings. Each serial must identify a different camera.

Edit `ws/src/neurahackathon_cameras/config/streams.yaml` (relative to the repository
root) to change stream settings. Alternatively, pass `config_file:=/absolute/path/to/streams.yaml`
with the same ROS parameter YAML structure. Rebuild after editing files if your
installation does not reflect source changes.

## View images

```bash
ros2 run rqt_image_view rqt_image_view
```

Choose a topic in the dropdown. Open additional viewer instances to compare images.

| Stream | Camera 1 topic | Camera 2 topic |
| --- | --- | --- |
| Color | `/cameras/camera_1/color/image_raw` | `/cameras/camera_2/color/image_raw` |
| Depth | `/cameras/camera_1/depth/image_rect_raw` | `/cameras/camera_2/depth/image_rect_raw` |
| Left infrared | `/cameras/camera_1/infra1/image_rect_raw` | `/cameras/camera_2/infra1/image_rect_raw` |
| Right infrared | `/cameras/camera_1/infra2/image_rect_raw` | `/cameras/camera_2/infra2/image_rect_raw` |

Defaults are RGB 640x480 at 30 FPS and depth/infrared 848x480 at 30 FPS.
Unique camera names keep topics and camera TF frame names separate.
The configuration does not calibrate the two cameras relative to one another
or synchronize their exposures. `infra1` and `infra2` refer to the stereo
sensors inside each D435, rather than the physical placement of the two D435s.

If a camera disappears, check `rs-enumerate-devices -s`, its USB cable/port, and
that another process is not using it. The driver waits and retries when the
selected serial is unavailable.
