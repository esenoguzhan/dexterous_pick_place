# Neurahackathon

This repository contains the LARA5 robot stack and a two-camera RealSense workspace:

- [`src/`](src/README.md): LARA5 arm, Inspire hand, robot descriptions, and MuJoCo
  simulation. Follow its README for the supported **Ubuntu 22.04 / ROS 2 Humble**
  environment and robot build instructions.
- [`ws/`](ws/src/neurahackathon_cameras): two D435 cameras with color, depth, and
  left/right infrared streams, built for **ROS 2 Humble** in the Docker setup below.
- [Object-pose approach task](src/robot/lara5_object_approach/README.md):
  independent `/arm/approach/target` command, supplied perception pose, direct
  NEURA IK motion and completion status. Grasping is a separate future task.

Use the Docker setup below to run both workspaces in **Ubuntu 22.04 / ROS 2
Humble**. The host needs Linux Docker Engine and Docker Compose; **neither ROS 2
nor Jazzy needs to be installed on the host**. Optional native Jazzy instructions
are provided later for running only the cameras outside Docker.

Robot meshes and the NEURA dependency archive use Git LFS. Install Git LFS on a
new machine, run `git lfs install`, and run `git lfs pull` after cloning this branch
to download those assets. The vendored RealSense models remain ordinary Git files.

## Robot and cameras in the Humble Docker container

Use this option to run the camera workspace alongside the robot stack, even on
an Ubuntu 24.04 host. Both workspaces are built from source **inside Ubuntu 22.04 /
ROS 2 Humble**; any host ROS installation and `ws/build`, `ws/install`, and
`ws/log` are not used. The existing native Jazzy instructions below still apply
when running cameras outside Docker.

The extension in [`src/docker/Dockerfile.realsense`](src/docker/Dockerfile.realsense)
adds librealsense **2.58.4**, the vendored ROS driver **4.58.4**, our dual-camera
launch package, `rs-enumerate-devices`, and `rqt_image_view` to the existing robot
image. It builds the SDK with its userspace RSUSB backend, so there is no SDK
DKMS/kernel installation inside the container. Linux Docker Engine and Docker
Compose are required for the USB passthrough configuration below.

### Build both images

Run from the repository root, after downloading any Git LFS assets:

```bash
docker build -f src/docker/Dockerfile -t lara5_hackathon src
docker compose -f compose.realsense.yaml build
```

The first build uses `src/` as context; the camera extension uses the repository
root so it can copy `ws/src/`. The first SDK build can take several minutes.
Build parallelism defaults to two jobs. If necessary, set `BUILD_JOBS=1` before
the Compose build to reduce memory usage.

### Open the workspace container

If you previously started the camera auto-launch service, stop that old service
once before switching to the interactive workflow:

```bash
docker compose -f compose.realsense.yaml down
```

Set your team's ROS domain, then open an interactive shell, just like the
original robot container:

```bash
export ROS_DOMAIN_ID=42
docker compose -f compose.realsense.yaml run --rm --name lara5_workspace cameras
```

This starts **Bash only**. Cameras and robot nodes start when you launch them.
Both workspaces are already sourced inside the shell.

The supplied serials (`263522073705` and `262722075125`) and stream settings are
preserved. To select different cameras, set these variables before opening the
container:

```bash
export CAMERA_1_SERIAL=FIRST_SERIAL
export CAMERA_2_SERIAL=SECOND_SERIAL
```

Compose shares the host network and IPC namespace and mounts `/dev/bus/usb` with
a USB device cgroup rule; it does not require `--privileged`. This grants access
to the host USB bus, including cameras that disconnect and reconnect. Run this
on the Linux machine physically connected to the cameras. The USB bus must exist
on that host; Docker Desktop/VM USB forwarding requires separate setup.

### Launch cameras manually

When ready, connect both D435 cameras to USB 3 ports and stop any host camera
launches or RealSense applications already using them. Inside the container:

```bash
rs-enumerate-devices -s
ros2 launch neurahackathon_cameras camera.launch.py
```

`camera.launch.py` includes the existing dual-camera launch. It reads the mounted
stream settings and camera serial defaults from the container environment. You
can still override them explicitly:

```bash
ros2 launch neurahackathon_cameras camera.launch.py \
  camera_1_serial:=FIRST_SERIAL camera_2_serial:=SECOND_SERIAL
```

The original `dual_realsense.launch.py` command also remains available. Press
Ctrl+C to stop the camera launch and return to the workspace shell.

### Run robot simulation in the same container

Open a second terminal in the repository root and launch the robot simulation:

```bash
docker exec -it lara5_workspace bash
# Inside the container, both workspaces are already sourced:
ros2 launch lara5_controller_manager controller_manager.launch.py \
  deployment_mode:=simulation sim_headless:=true
```

Robot sources/builds live under `/ws`; camera sources/builds live under
`/camera_ws`. Both use Humble and the same ROS domain. Real hardware setup and
the additional container runtime permissions are documented in
[`src/README.md`, section 3.6](src/README.md#36-alternative-docker).

The workspace shell runs without a graphical display. For image viewing on a
Linux X11/XWayland desktop, open a Bash shell in a separate container with display
access, then start the viewer manually:

```bash
xhost +local:root
docker run --rm -it --net=host --ipc=host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  lara5_hackathon:realsense bash
# Inside that container:
ros2 run rqt_image_view rqt_image_view
# After closing the viewer, exit the container, then on the host:
xhost -local:root
```

### Change settings and stop

`ws/src/neurahackathon_cameras/config/streams.yaml` is mounted read-only into
the container. Edit it on the host, stop the camera launch with Ctrl+C, and run
`ros2 launch neurahackathon_cameras camera.launch.py` again to apply it.

For camera source/launch changes, exit the container, rebuild the extension, and
open a new workspace shell:

```bash
docker compose -f compose.realsense.yaml build
docker compose -f compose.realsense.yaml run --rm --name lara5_workspace cameras
```

For robot source changes, rebuild the base image first. Container filesystem
changes are discarded when the `--rm` container exits; save code and recordings
to the host before doing so. Stop launches with Ctrl+C, then use `exit` in the
original workspace shell to close/remove the container.

Alternatively, keep the workspace shell running in the background:

```bash
docker compose -f compose.realsense.yaml up -d
docker compose -f compose.realsense.yaml exec cameras bash
# Stop/remove the background workspace when finished:
docker compose -f compose.realsense.yaml down
```

If the SDK reports `RS2_USB_STATUS_BUSY`, `failed to claim usb interface`, or
`uvc_get_ctrl` errors, check for a native Jazzy/Humble camera driver or RealSense
viewer still using the device and stop that launch before retrying. A different
`ROS_DOMAIN_ID` does not allow two drivers to share the same physical camera.
If a serial is not detected, check cables/USB ports and run
`rs-enumerate-devices -s` inside the container. The missing camera's node keeps
waiting/retrying; it does not select the other camera instead.

## Multi-camera workspace

The camera workspace runs on **ROS 2 Humble inside Docker**. Each USB RealSense
D435 publishes color, depth, left infrared (`infra1`), and right infrared
(`infra2`) images. The optional native setup below uses ROS 2 Jazzy.

## Source layout

```text
ws/src/
  realsense-ros/            # Upstream driver 4.58.4, copied into this repository
  neurahackathon_cameras/   # Shared stream parameters and dual-camera launch
    config/streams.yaml
    launch/camera.launch.py
    launch/dual_realsense.launch.py
```

The driver is pinned to commit `9a11121700cb4780e273e34141f6402fe184321d`, matching
the ROS driver version installed on the original machine. See
[`UPSTREAM.md`](ws/src/realsense-ros/UPSTREAM.md) for provenance and SDK requirements.

## Optional native camera setup (Ubuntu 24.04 / ROS 2 Jazzy)

Skip this section when using Docker: the Dockerfiles install dependencies and
build both workspaces with Humble. This native alternative is for the camera
workspace only; it does not build or validate the robot stack on Jazzy.

For native camera use, install Ubuntu 24.04 with ROS 2 Jazzy and source its
environment. Install the Jazzy RealSense SDK package and development tools if needed:

```bash
sudo apt update
sudo apt install ros-jazzy-librealsense2 python3-colcon-common-extensions python3-rosdep ros-jazzy-rqt-image-view
```

Clone this branch and install the workspace dependencies. If rosdep has never
been initialized on your machine, run `sudo rosdep init` once before `rosdep update`.

```bash
git clone --branch main git@github.com:arasf/neurahackathon.git
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
