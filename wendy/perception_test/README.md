# Perception test bench (Wendy, Jetson Orin Nano)

A browser tool for trying the SAM 2.1 and DINOv2 stages of
[`docs/perception_architecture.md`](../../docs/perception_architecture.md) on the real D435
cameras before the ROS 2 perception package is written. It is not the pipeline itself: there is no
table-plane fit, clustering, view fusion or 3D size check.

| Doc section | What the page does |
|---|---|
| 3.3 | Colour 1280x720, depth 848x480 aligned to colour, laser power 250 (15 fps to spare the CPU) |
| 5.5 | `SAM2ImagePredictor` with point and box prompts, three candidates, best score first |
| 5.5 | `SAM2AutomaticMaskGenerator` over the whole image, masks above a chosen area dropped |
| 5.7 | Masked crop (10 % padding, grey 127 background, 224x224), DINOv2-small CLS, L2-normalised |
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

Each capture can be downloaded as `colour.png` and a 16-bit millimetre `depth.png` for offline tests.

## Questions it answers (section 11.1)

1. Does SAM give a clean mask for the steel mug?
2. Does SAM split the touching pairs (box prompt on the pair, then a point on each object)?
3. Does the gallery name all eight objects correctly, and with what margin?
