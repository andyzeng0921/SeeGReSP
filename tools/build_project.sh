#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$ROOT"
colcon --log-base "$ROOT/log" build \
  --base-paths "$ROOT" \
  --build-base "$ROOT/build" \
  --install-base "$ROOT/install" \
  --packages-select adaptive_object_grasping \
  --symlink-install
set +u
source "$ROOT/install/setup.bash"
set -u
colcon --log-base "$ROOT/test-log" test \
  --base-paths "$ROOT" \
  --build-base "$ROOT/build" \
  --install-base "$ROOT/install" \
  --packages-select adaptive_object_grasping
colcon test-result --test-result-base "$ROOT/build/adaptive_object_grasping" --verbose
