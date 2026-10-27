#!/usr/bin/env bash
# Run the perception test bench natively on an x86_64 laptop with an NVIDIA GPU
# (no Wendy, no Docker). One-time setup: see requirements-laptop.txt.
#
#   ./run_laptop.sh            # serves http://localhost:8090
#   PORT=9000 ./run_laptop.sh
#
# Plug the D435 into a USB 3 port. Without a RealSense the app falls back to the
# first V4L2 webcam (colour only), which is enough to exercise SAM 2 and DINOv2.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
export SAM2_CKPT="${SAM2_CKPT:-$HERE/models/sam2.1_hiera_small.pt}"
export DINO_DIR="${DINO_DIR:-$HERE/models/dinov2-small}"
export DATA_DIR="${DATA_DIR:-$HERE/data}"
export HF_HUB_OFFLINE=1
export PORT="${PORT:-8090}"
export PYTHONUNBUFFERED=1
mkdir -p "$DATA_DIR"
exec "$HERE/.venv/bin/python" "$HERE/app.py"
