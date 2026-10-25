# Perception test bench (Wendy, Jetson Orin Nano)

A browser tool for trying the SAM 2.1 and DINOv2 stages of
[`docs/perception_architecture.md`](../../docs/perception_architecture.md) on the real D435
cameras before the ROS 2 perception package is written. It is not the pipeline itself: there is no
table-plane fit, clustering, view fusion or 3D size check.

| Doc section | What the page does |
|---|---|
| 3.3 | Colour 1280x720, depth 848x480 aligned to colour, laser power 250 (15 fps to spare the CPU) |
| 5.1 | Capture depth is the per-pixel median of the next 10 frames (selectable), zeros ignored, a pixel needs half its samples valid |
| 5.5 | `SAM2ImagePredictor` with point and box prompts, three candidates, best score first |
| 5.5 | `SAM2AutomaticMaskGenerator` over the whole image, masks above a chosen area dropped |
| 5.7 | Masked crop (10 % padding, grey 127 background, padded to a grey square, 224x224), DINOv2-small CLS, L2-normalised |
| 3.4 | Camera calibration page: ChArUco board on the hand, samples from the robot side, hand-eye solve, `extrinsics.yaml` |
| 5.7 | Gallery score = best cosine over an object's views; accepted at >= 0.6 with margin >= 0.05; auto masks use Hungarian assignment |
| 7.1, 8.3 | Gallery stored as `gallery/<name>/view_XX.png`, `embeddings.npy`, `meta.json` |

Models: `sam2.1_hiera_small` and `facebook/dinov2-small`, both baked into the image.

## Run

From the repository root, with Docker Desktop running:

```bash
wendy run --prefix wendy/perception_test
```

The first build runs under emulation on an x86 PC and uploads several GB, so it takes a while.
Later builds reuse the cached layers. The page opens at `http://<device>.local:8090`.

The gallery lives in the `perception-test-data` volume, so it survives redeploys.

### Building on a Windows PC

Two problems we hit with the Wendy CLI 2026.10.22 and Docker Desktop 28.0.4:

- **`qemu: uncaught target signal 11` while installing `libc-bin`.** Docker Desktop's ARM
  emulator crashes on Ubuntu 22.04's `ldconfig`. Install the newer emulator once; repeat it after
  every Docker Desktop restart:

  ```bash
  docker run --privileged --rm tonistiigi/binfmt --install arm64
  ```

- **`local cache importer requires either explicit digest, "latest" tag ...`.** Wendy saves its
  local image cache without a tag and then cannot read it back on the next build. Tag it before
  rebuilding (PowerShell):

  ```powershell
  $p = "$env:LOCALAPPDATA\wendy\ocilayout\perception-test-linux_arm64\index.json"; $j = Get-Content $p -Raw | ConvertFrom-Json; $j.manifests[0] | Add-Member annotations ([pscustomobject]@{ 'org.opencontainers.image.ref.name' = 'latest' }) -Force; [IO.File]::WriteAllText($p, ($j | ConvertTo-Json -Depth 10 -Compress))
  ```

Measured on the Orin Nano (25 W mode, 1280x720): SAM image embedding 0.33 s, SAM decoder 40 to
75 ms, DINOv2 about 20 ms per mask, automatic masks at 24 points per side 7.6 s (36 masks).

## Use

1. Pick a camera tab, check the colour and depth streams, press **Capture**.
2. Left-click an object (right-click marks background), or switch to **Box** and drag.
   The table lists the SAM candidates with their score, area, median depth and the share of
   pixels with valid depth.
3. Type a name and press **Add view**. Rotate the object about 30 degrees and repeat for 10 to
   20 views (section 7.1).
4. Put several objects down, capture, and press **Auto masks** to see which ones the gallery names.

Each capture can be downloaded as `colour.png`, a 16-bit millimetre `depth.png`, `intrinsics.json`
(camera serial, `fx`, `fy`, `cx`, `cy`, distortion, depth units and median settings), or all three in
one zip for offline tests. File names start with the capture time and the camera serial.

## Camera calibration

Gives the fixed camera's pose in the robot's `world` frame and the hand camera's pose on the flange
(section 3.4). Calibrate with the cameras in their final positions, and again whenever one moves.

1. **Board:** open `http://<device>.local:8090/calibration` and download `board.pdf` (5 x 7 ChArUco,
   36 mm squares, `DICT_4X4_50`, fits A4). Print it at 100 % (actual size) and glue it flat on a rigid
   plate. Measure one square with a ruler, enter the measured size (and the marker size, three quarters
   of it) and press **Apply**.
2. **Setup:** on the page, choose **board on the table** and mark the camera on the hand as
   **on the hand**. Lay the board flat where the fixed camera sees it well, and do not touch it until
   the recording is done. The page shows live detection for each camera; a partly covered board is fine.
3. **Record** on the Ubuntu PC, inside the robot container with the controller launch running
   (`safety:=on`). Rebuild `lara5_controller_manager` once so the script is installed. Move the arm so
   the hand camera sees the board with room around the hand. To repeat a previous manually recorded
   session, copy its exported JSON to the robot container (run this from the repository root on the host):

   ```bash
   docker cp calibration_session.json lara5_workspace:/ws/calibration_session.json
   ```

   Inside the robot container, preview the saved targets, then replay them:

   ```bash
   ros2 run lara5_controller_manager calibrate_cameras.py --jetson http://<jetson>:8090 auto --session /ws/calibration_session.json --dry-run
   ros2 run lara5_controller_manager calibrate_cameras.py --jetson http://<jetson>:8090 auto --session /ws/calibration_session.json
   ```

   `auto` now defaults to replaying the six `robot.joints` values from every sample in
   `./calibration_session.json`, in file order. Missing/nonfinite joints reject the whole input before
   motion; `--count N` limits the run to the first N targets. The input file is not modified. The
   robot goes through the safety filter's public `FollowJointTrajectory` action and returns to its
   starting joints after a completed run. Intervening paths are joint-space lines, not recordings of
   the original manual paths, and each is collision-checked. A rejected path is skipped. Peak joint
   speed defaults to 0.2 rad/s (`--joint-speed`, maximum 0.5), with smooth starts and stops and at least
   3 seconds per move. `--dry-run` only lists targets; it makes no HTTP or ROS connections.

   For newly generated Cartesian poses, explicitly use `auto --random --pivot 0.4`. That mode visits
   20 poses within 8 cm and 8 to 25 degrees of tilt; `--pivot` is the approximate board distance along
   the flange's z axis, adjustable with `--pivot-axis`. `manual` records whenever you press Enter,
   while you move the arm yourself. **The safety filter knows neither the board nor the hand camera**,
   so start with plenty of room and keep the e-stop in reach.

   Replay verifies the measured joints within 0.01 rad before recording; a controller error or
   measured mismatch stops the run without another move or a return motion. Ctrl+C requests
   cancellation of an active joint goal and does not request a return motion. Captures and flange
   poses are fresh measurements, not copies of the old observations. To start a fresh calibration,
   export the Jetson's current session and clear its samples on the page before replaying the local
   pose file; keep the board stationary throughout the new run.
   In Cartesian mode, after each successful controller result, the script requires fresh joint feedback
   and checks the measured flange against the requested pose (within 5 mm and 2 degrees). If the controller reports
   success while the arm stays still, auto stops before recording or requesting another move. Check
   the robot's drive/control status and controller logs; do not use samples from an unmoving arm.
4. **Solve** on the page (or `calibrate_cameras.py ... solve`). Good results: reprojection under
   0.5 px and board error under 2 mm (green). Delete bad samples with **×** and solve again. Widely
   spread poses matter most for the fixed camera, which inherits the board's orientation error.
   Sessions with less than one degree of measured robot rotation are rejected before solving.
5. **Save** writes `/data/calibration/extrinsics.yaml`, the file perception loads, plus a timestamped
   copy. **Export session** downloads all samples, so a solve can be repeated later.

## Questions it answers (section 11.1)

1. Does SAM give a clean mask for the steel mug?
2. Does SAM split the touching pairs (box prompt on the pair, then a point on each object)?
3. Does the gallery name all eight objects correctly, and with what margin?
