#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="/home/ubuntu/miniconda3/envs/robot_env/lib/python3.12/site-packages:${PYTHONPATH:-}"
# robot_env's NumPy depends on libcblas.so.3, supplied by the compatible
# cuda_env BLAS runtime on robot 306.
export LD_LIBRARY_PATH="/home/ubuntu/miniconda3/envs/cuda_env/lib:/home/ubuntu/miniconda3/envs/robot_env/lib:${LD_LIBRARY_PATH:-}"
PYTHON_BIN="${ADAPTIVE_GRASP_VENDOR_PYTHON:-/usr/bin/python3}"
exec "$PYTHON_BIN" "$SCRIPT_DIR/motion_executor_node.py" "$@"
