#!/usr/bin/env bash
set -euo pipefail

ENV_DIR="${ADAPTIVE_GRASP_ENV:-/home/ubuntu/adaptive_grasp_venv}"
PYTHON_BIN="${PYTHON_BIN:-python3}"

"$PYTHON_BIN" -m venv --system-site-packages "$ENV_DIR"
"$ENV_DIR/bin/python" -m pip install --upgrade pip setuptools wheel
"$ENV_DIR/bin/python" -m pip install \
  torch torchvision --index-url https://download.pytorch.org/whl/cu128
"$ENV_DIR/bin/python" -m pip install ultralytics open3d graspnetAPI

cat <<'EOF'
AI environment created.

GraspNet remains a separate, noncommercial-research dependency:
  1. Review and accept https://github.com/graspnet/graspnet-baseline/blob/master/LICENSE
  2. Clone the official repository to /home/ubuntu/open_source/graspnet-baseline
  3. Build its pointnet2 extension with this environment's Python
  4. Put the official RealSense checkpoint at
     /home/ubuntu/models/graspnet/checkpoint-rs.tar

Do not install these packages into robot_env; that environment runs the vendor services.
EOF
