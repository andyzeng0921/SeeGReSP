#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
PYTHON_BIN="${ADAPTIVE_GRASP_VENDOR_PYTHON:-/home/ubuntu/miniconda3/envs/robot_env/bin/python}"
exec "$PYTHON_BIN" "$SCRIPT_DIR/motion_executor_node.py" "$@"
