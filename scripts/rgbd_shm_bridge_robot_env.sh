#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
exec /home/ubuntu/miniconda3/envs/robot_env/bin/python \
  "$SCRIPT_DIR/rgbd_shm_bridge_node.py" "$@"
