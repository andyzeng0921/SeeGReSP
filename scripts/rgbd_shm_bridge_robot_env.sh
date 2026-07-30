#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
RESOLVED_SCRIPT="$(readlink -f "$0")"
SOURCE_ROOT="$(CDPATH= cd -- "$(dirname -- "$RESOLVED_SCRIPT")/.." && pwd)"
PACKAGE_ROOT="${ADAPTIVE_GRASP_PACKAGE_ROOT:-$SOURCE_ROOT}"
PYTHON_BIN="${ADAPTIVE_GRASP_PYTHON:-$PACKAGE_ROOT/third_party/venv/bin/python}"
exec "$PYTHON_BIN" \
  "$SCRIPT_DIR/rgbd_shm_bridge_node.py" "$@"
