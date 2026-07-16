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
PYTORCH_INDEX_URL="https://download.pytorch.org/whl/cu128"
TORCH_VERSION="2.11.0+cu128"
TORCHVISION_VERSION="0.26.0+cu128"
TORCH_WHEEL_NAME="torch-2.11.0+cu128-cp312-cp312-manylinux_2_28_x86_64.whl"
TORCH_WHEEL="$PACKAGE_ROOT/third_party/wheels/$TORCH_WHEEL_NAME"
TORCH_WHEEL_SHA256="d252cf975fb18c94a85336323ad425f473df56dab35a44b00399bd70c7a3b997"

mkdir -p "$PACKAGE_ROOT/third_party" "$PACKAGE_ROOT/models/yolo" \
  "$PACKAGE_ROOT/models/graspnet" "$PACKAGE_ROOT/third_party/wheels" \
  "$PACKAGE_ROOT/third_party/pip-cache" "$PACKAGE_ROOT/third_party/tmp" \
  "$PACKAGE_ROOT/third_party/ultralytics-config" \
  "$PACKAGE_ROOT/third_party/matplotlib-config"
export PIP_CACHE_DIR="$PACKAGE_ROOT/third_party/pip-cache"
export TMPDIR="$PACKAGE_ROOT/third_party/tmp"
export YOLO_CONFIG_DIR="$PACKAGE_ROOT/third_party/ultralytics-config"
export MPLCONFIGDIR="$PACKAGE_ROOT/third_party/matplotlib-config"

if [ ! -x "$ENV_DIR/bin/python" ]; then
  if ! "$PYTHON_BIN" -m venv --system-site-packages "$ENV_DIR"; then
    VIRTUALENV_PYZ="$PACKAGE_ROOT/third_party/virtualenv.pyz"
    echo "python venv support is unavailable; using a package-local virtualenv bootstrap."
    curl --fail --location --retry 3 \
      https://bootstrap.pypa.io/virtualenv.pyz --output "$VIRTUALENV_PYZ"
    "$PYTHON_BIN" "$VIRTUALENV_PYZ" --clear --system-site-packages "$ENV_DIR"
  fi
fi
"$ENV_DIR/bin/python" -m pip install --upgrade pip setuptools wheel

if [ ! -f "$TORCH_WHEEL" ] && [ -n "${PYTORCH_WHEEL_MIRROR_URL:-}" ]; then
  curl --fail --location --retry 3 "$PYTORCH_WHEEL_MIRROR_URL" \
    --output "$TORCH_WHEEL.part"
  echo "$TORCH_WHEEL_SHA256  $TORCH_WHEEL.part" | sha256sum --check
  mv "$TORCH_WHEEL.part" "$TORCH_WHEEL"
fi

if [ -f "$TORCH_WHEEL" ]; then
  echo "$TORCH_WHEEL_SHA256  $TORCH_WHEEL" | sha256sum --check
  "$ENV_DIR/bin/python" -m pip install "$TORCH_WHEEL" \
    "torchvision==$TORCHVISION_VERSION" --index-url "$PYTORCH_INDEX_URL"
else
  "$ENV_DIR/bin/python" -m pip install "torch==$TORCH_VERSION" \
    "torchvision==$TORCHVISION_VERSION" --index-url "$PYTORCH_INDEX_URL"
fi
"$ENV_DIR/bin/python" -m pip install \
  ultralytics open3d 'transforms3d>=0.4.2' trimesh tqdm scipy cvxopt dill \
  h5py scikit-learn scikit-image pywavefront ninja gdown grasp_nms
# graspnetAPI 1.2.11 pins NumPy 1.20.3 and transforms3d 0.3.1, neither of
# which builds on ROS Jazzy's Python 3.12. Its runtime works with the compatible
# versions installed above; avoid forcing those obsolete package metadata pins.
"$ENV_DIR/bin/python" -m pip install --no-deps 'graspnetAPI==1.2.11'

(
  cd "$PACKAGE_ROOT/models/yolo"
  "$ENV_DIR/bin/python" -c "from ultralytics import YOLO; YOLO('yolo11n-seg.pt')"
)

if [ "${KEEP_PIP_CACHE:-0}" != "1" ]; then
  "$ENV_DIR/bin/python" -m pip cache purge >/dev/null || true
fi

cat <<'EOF'
AI environment created.

Everything is stored below this ROS package:
  third_party/venv
  third_party/wheels
  third_party/pip-cache
  models/yolo/yolo11n-seg.pt

GraspNet is a separate noncommercial-research dependency. Review its license,
then install and verify it with:
  GRASPNET_LICENSE_ACCEPTED=1 tools/install_graspnet.sh

Do not install these packages into robot_env; that environment runs the vendor services.
EOF
