#!/usr/bin/env bash
# deploy.sh - push this project to the Pi and free the camera.
# Run from the autocar/ directory in Git Bash on Windows:
#     bash tools/deploy.sh [pi_host]
#
# Default host below; override with an argument.
set -euo pipefail

PI="${1:-pi@192.168.0.202}"
DEST="~/autocar"

echo "==> Copying project to $PI:$DEST"
# --- copy everything except logs and local model weights ---
rsync -av --exclude 'logs/' --exclude 'models/*.pt' --exclude '__pycache__' \
      ./ "$PI:$DEST/" 2>/dev/null || \
  scp -r . "$PI:$DEST"

echo "==> Freeing the camera on the Pi (disables the old CCTV service)"
ssh "$PI" bash -s <<'REMOTE'
set -e
if systemctl is-enabled cctv.service >/dev/null 2>&1; then
  echo "Stopping and disabling cctv.service ..."
  sudo systemctl stop cctv.service || true
  sudo systemctl disable cctv.service || true
fi
pkill -f cctv.py 2>/dev/null || true
pkill -f 'rclone move' 2>/dev/null || true
find ~ -maxdepth 1 -name 'CHUNK_*.avi' -size 0 -delete 2>/dev/null || true
echo "Camera status:"
fuser /dev/video0 2>/dev/null && echo "  (still busy!)" || echo "  camera is free"
REMOTE

echo "==> Done. Next on the Pi:"
echo "    ssh $PI"
echo "    cd autocar && python tools/test_crc.py && python tools/serial_check.py"
