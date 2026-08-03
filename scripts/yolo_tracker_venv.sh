#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
RESOLVED_SCRIPT="$(readlink -f "$0")"
SOURCE_ROOT="$(CDPATH= cd -- "$(dirname -- "$RESOLVED_SCRIPT")/.." && pwd)"
if [ ! -f "$SOURCE_ROOT/package.xml" ]; then
  PREFIX="$(ros2 pkg prefix adaptive_object_grasping)"
  WORKSPACE_ROOT="$(CDPATH= cd -- "$PREFIX/../.." && pwd)"
  SOURCE_ROOT="$WORKSPACE_ROOT/src/adaptive_object_grasping"
fi
PACKAGE_ROOT="${ADAPTIVE_GRASP_PACKAGE_ROOT:-$SOURCE_ROOT}"
PYTHON_BIN="${ADAPTIVE_GRASP_PYTHON:-$PACKAGE_ROOT/third_party/venv/bin/python}"
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-$PACKAGE_ROOT/third_party/ultralytics-config}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$PACKAGE_ROOT/third_party/matplotlib-config}"
mkdir -p "$YOLO_CONFIG_DIR" "$MPLCONFIGDIR"
cd "$PACKAGE_ROOT"
exec "$PYTHON_BIN" "$SCRIPT_DIR/yolo_tracker_node.py" "$@"
