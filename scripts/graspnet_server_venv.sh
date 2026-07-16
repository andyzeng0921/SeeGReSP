#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
PYTHON_BIN="${ADAPTIVE_GRASP_PYTHON:-/home/ubuntu/adaptive_grasp_venv/bin/python}"
exec "$PYTHON_BIN" "$SCRIPT_DIR/graspnet_server_node.py" "$@"
