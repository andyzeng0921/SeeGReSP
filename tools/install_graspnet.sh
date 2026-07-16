#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
PACKAGE_ROOT="${ADAPTIVE_GRASP_PACKAGE_ROOT:-$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)}"
ENV_DIR="${ADAPTIVE_GRASP_ENV:-$PACKAGE_ROOT/third_party/venv}"
BASELINE_DIR="$PACKAGE_ROOT/third_party/graspnet-baseline"
CHECKPOINT="$PACKAGE_ROOT/models/graspnet/checkpoint-rs.tar"
BASELINE_URL="https://github.com/graspnet/graspnet-baseline.git"
BASELINE_COMMIT="280c215129f759ed8649cb4e89fc5dfee55f4f80"
CHECKPOINT_URL="${GRASPNET_CHECKPOINT_URL:-https://hf-mirror.com/dgrachev/a2_pretrained/resolve/main/checkpoint-rs.tar}"
CHECKPOINT_SHA256="60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868"
CUDA_HOME="${CUDA_HOME:-/usr/local/cuda}"
WORKSPACE_ROOT="$(CDPATH= cd -- "$PACKAGE_ROOT/../.." && pwd)"

if [ "${GRASPNET_LICENSE_ACCEPTED:-0}" != "1" ]; then
  cat >&2 <<'EOF'
GraspNet baseline is restricted to noncommercial research use.
Read https://github.com/graspnet/graspnet-baseline/blob/master/LICENSE,
then rerun with GRASPNET_LICENSE_ACCEPTED=1 if you accept those terms.
EOF
  exit 2
fi
if [ ! -x "$ENV_DIR/bin/python" ]; then
  echo "Package-local AI environment is missing; run tools/install_ai_environment.sh first." >&2
  exit 1
fi
if [ ! -x "$CUDA_HOME/bin/nvcc" ]; then
  echo "CUDA compiler not found below CUDA_HOME=$CUDA_HOME" >&2
  exit 1
fi

mkdir -p "$PACKAGE_ROOT/third_party" "$PACKAGE_ROOT/models/graspnet"
if [ ! -d "$BASELINE_DIR/.git" ]; then
  git clone "$BASELINE_URL" "$BASELINE_DIR"
fi
git -C "$BASELINE_DIR" fetch --depth 1 origin "$BASELINE_COMMIT"
git -C "$BASELINE_DIR" checkout --detach "$BASELINE_COMMIT"

"$ENV_DIR/bin/python" -m pip install ninja gdown grasp_nms

if [ ! -f "$CHECKPOINT" ]; then
  curl --fail --location --retry 3 "$CHECKPOINT_URL" --output "$CHECKPOINT.part"
  mv "$CHECKPOINT.part" "$CHECKPOINT"
fi
echo "$CHECKPOINT_SHA256  $CHECKPOINT" | sha256sum --check

export CUDA_HOME
export LD_LIBRARY_PATH="$CUDA_HOME/lib64:${LD_LIBRARY_PATH:-}"
export MAX_JOBS="${MAX_JOBS:-4}"
if [ -z "${TORCH_CUDA_ARCH_LIST:-}" ]; then
  TORCH_CUDA_ARCH_LIST="$($ENV_DIR/bin/python -c \
    'import torch; major, minor = torch.cuda.get_device_capability(); print(f"{major}.{minor}")')"
  export TORCH_CUDA_ARCH_LIST
fi

build_extension() {
  local source_dir="$1"
  (
    cd "$source_dir"
    "$ENV_DIR/bin/python" - <<'PY'
import runpy
import sys
import torch.utils.cpp_extension as cpp_extension

# The robot provides a newer nvcc that is binary-compatible with Torch's CUDA runtime.
# Torch rejects the version string before compiling, so defer compatibility to the
# compile and runtime checks performed below.
cpp_extension._check_cuda_version = lambda *args, **kwargs: None
sys.argv = ['setup.py', 'build_ext', '--inplace']
runpy.run_path('setup.py', run_name='__main__')
PY
  )
}

mkdir -p "$BASELINE_DIR/pointnet2/pointnet2"
build_extension "$BASELINE_DIR/pointnet2"
POINTNET_SO="$(find "$BASELINE_DIR/pointnet2/build" -path '*/pointnet2/_ext*.so' | head -n 1)"
test -n "$POINTNET_SO"
cp "$POINTNET_SO" "$BASELINE_DIR/pointnet2/"

mkdir -p "$BASELINE_DIR/knn/knn_pytorch"
touch "$BASELINE_DIR/knn/knn_pytorch/__init__.py"
build_extension "$BASELINE_DIR/knn"

if ! command -v ros2 >/dev/null 2>&1; then
  ROS_SETUP="$(find /opt/ros -mindepth 2 -maxdepth 2 -name setup.bash | sort | tail -n 1)"
  if [ -z "$ROS_SETUP" ]; then
    echo "ROS 2 setup.bash was not found below /opt/ros." >&2
    exit 1
  fi
  # shellcheck disable=SC1090
  set +u
  source "$ROS_SETUP"
  set -u
fi
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  set +u
  source "$WORKSPACE_ROOT/install/setup.bash"
  set -u
fi

cd "$PACKAGE_ROOT"
"$ENV_DIR/bin/python" - <<'PY'
import numpy as np
import torch
from adaptive_object_grasping_nodes.graspnet_server import OfficialGraspNetBackend

root = 'third_party/graspnet-baseline'
checkpoint = 'models/graspnet/checkpoint-rs.tar'
backend = OfficialGraspNetBackend(root, checkpoint, 'cuda:0', 20000, 300)
rng = np.random.default_rng(7)
points = np.column_stack((
    rng.uniform(-0.08, 0.08, 20000),
    rng.uniform(-0.08, 0.08, 20000),
    rng.uniform(0.45, 0.60, 20000),
)).astype(np.float32)
rows = backend.infer(points, points, 0.0, 0.01, 5)
if rows.shape != (5, 17):
    raise RuntimeError(f'unexpected GraspNet output shape: {rows.shape}')
print(f'GraspNet ready on {torch.cuda.get_device_name(0)}; output={rows.shape}')
PY

echo "GraspNet baseline, CUDA extensions, and RealSense checkpoint are ready."
