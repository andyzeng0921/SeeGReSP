#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
PACKAGE_ROOT="${ADAPTIVE_GRASP_PACKAGE_ROOT:-}"
if [ -z "$PACKAGE_ROOT" ]; then
  SOURCE_CANDIDATE="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
  if [ -f "$SOURCE_CANDIDATE/CMakeLists.txt" ] && [ -f "$SOURCE_CANDIDATE/package.xml" ]; then
    PACKAGE_ROOT="$SOURCE_CANDIDATE"
  else
    PREFIX="$(ros2 pkg prefix adaptive_object_grasping)"
    WORKSPACE_ROOT="$(CDPATH= cd -- "$PREFIX/../.." && pwd)"
    PACKAGE_ROOT="$WORKSPACE_ROOT/src/adaptive_object_grasping"
  fi
fi
if [ ! -f "$PACKAGE_ROOT/CMakeLists.txt" ] || [ ! -f "$PACKAGE_ROOT/package.xml" ]; then
  echo "Cannot locate adaptive_object_grasping source package: $PACKAGE_ROOT" >&2
  exit 1
fi
ENV_DIR="${ADAPTIVE_GRASP_ENV:-$PACKAGE_ROOT/third_party/venv}"
PYTHON_BIN="${PYTHON_BIN:-python3}"

mkdir -p "$PACKAGE_ROOT/third_party" "$PACKAGE_ROOT/models/yolo" \
  "$PACKAGE_ROOT/models/graspnet"
if ! "$PYTHON_BIN" -m venv --system-site-packages "$ENV_DIR"; then
  VIRTUALENV_PYZ="$PACKAGE_ROOT/third_party/virtualenv.pyz"
  echo "python venv support is unavailable; using a package-local virtualenv bootstrap."
  curl --fail --location --retry 3 \
    https://bootstrap.pypa.io/virtualenv.pyz --output "$VIRTUALENV_PYZ"
  "$PYTHON_BIN" "$VIRTUALENV_PYZ" --clear --system-site-packages "$ENV_DIR"
fi
"$ENV_DIR/bin/python" -m pip install --upgrade pip setuptools wheel
"$ENV_DIR/bin/python" -m pip install \
  torch torchvision --index-url https://download.pytorch.org/whl/cu128
"$ENV_DIR/bin/python" -m pip install ultralytics open3d graspnetAPI

(
  cd "$PACKAGE_ROOT/models/yolo"
  "$ENV_DIR/bin/python" -c "from ultralytics import YOLO; YOLO('yolo11n-seg.pt')"
)

cat <<'EOF'
AI environment created.

Everything is stored below this ROS package:
  third_party/venv
  models/yolo/yolo11n-seg.pt

GraspNet remains a separate, noncommercial-research dependency:
  1. Review and accept https://github.com/graspnet/graspnet-baseline/blob/master/LICENSE
  2. Clone the official repository to third_party/graspnet-baseline
  3. Build its pointnet2 extension with this environment's Python
  4. Put the official RealSense checkpoint at
     models/graspnet/checkpoint-rs.tar

Do not install these packages into robot_env; that environment runs the vendor services.
EOF
