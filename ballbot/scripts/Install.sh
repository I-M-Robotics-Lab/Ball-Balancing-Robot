#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

chmod +x scripts/install.sh scripts/run_hmi.sh scripts/run_runtime.sh 2>/dev/null || true

echo "[INFO] Installing CLI entry points..."

sudo ln -sf "$PWD/scripts/run_hmi.sh" /usr/local/bin/hmi
sudo ln -sf "$PWD/scripts/run_runtime.sh" /usr/local/bin/ballbot-runtime

mkdir -p logs

echo "=== BallBot Install ==="

# Normalize line endings (prevents apt 'package\r' issues)
sed -i 's/\r$//' requirements-apt.txt 2>/dev/null || true
sed -i 's/\r$//' requirements.txt 2>/dev/null || true

# 1) Remove old venv
rm -rf .venv

# 2) Apt deps (single source of truth)
echo "[1/4] Installing apt dependencies..."
sudo apt-get update

APT_PKGS=$(grep -vE '^\s*#|^\s*$' requirements-apt.txt | tr '\n' ' ')
echo "[apt] $APT_PKGS"
sudo apt-get install -y $APT_PKGS

# 3) Create venv
echo "[2/4] Creating venv..."
python3 -m venv .venv --system-site-packages
source .venv/bin/activate

# 4) Pip deps
echo "[3/4] Installing pip deps..."
python -m pip install --upgrade pip wheel setuptools
python -m pip install -r requirements.txt

# 5) Sanity check
echo "[4/4] Sanity check..."
python - <<'PY'
import numpy
import cv2
from picamera2 import Picamera2
print("✔ Imports OK")
PY

echo "Install complete."
