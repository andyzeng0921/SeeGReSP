#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
PROJECT_ROOT="${ADAPTIVE_GRASP_PACKAGE_ROOT:-$(CDPATH= cd -- "$(dirname -- "$0")/../../../../.." && pwd)}"
MOVEIT_OVERLAY="$PROJECT_ROOT/third_party/ros-overlay/root/opt/ros/jazzy"
if [[ -d "$MOVEIT_OVERLAY/share/ament_index" ]]; then
  export AMENT_PREFIX_PATH="$MOVEIT_OVERLAY:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="$MOVEIT_OVERLAY:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$MOVEIT_OVERLAY/lib:${LD_LIBRARY_PATH:-}"
fi
PYTHON_BIN="${ADAPTIVE_GRASP_VENDOR_PYTHON:-/usr/bin/python3}"
exec "$PYTHON_BIN" "$SCRIPT_DIR/motion_executor_node.py" "$@"
