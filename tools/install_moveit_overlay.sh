#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
PACKAGES="$ROOT/third_party/ros-overlay/packages"
OVERLAY="$ROOT/third_party/ros-overlay/root"
mkdir -p "$PACKAGES" "$OVERLAY"
cd "$PACKAGES"
shopt -s nullglob
for package in ros-jazzy-control-msgs ros-jazzy-moveit-simple-controller-manager; do
  apt-get download "$package"
done
debs=(ros-jazzy-control-msgs_*.deb ros-jazzy-moveit-simple-controller-manager_*.deb)
[[ ${#debs[@]} -ge 2 ]] || {
  echo "Official MoveIt overlay dependencies were not downloaded" >&2
  exit 2
}
for deb in "${debs[@]}"; do
  dpkg-deb -x "$deb" "$OVERLAY"
done
plugin="$OVERLAY/opt/ros/jazzy/share/moveit_simple_controller_manager/moveit_simple_controller_manager_plugin_description.xml"
library="$OVERLAY/opt/ros/jazzy/lib/libmoveit_simple_controller_manager.so"
control_typesupport="$OVERLAY/opt/ros/jazzy/lib/libcontrol_msgs__rosidl_typesupport_cpp.so"
[[ -f "$plugin" && -e "$library" && -e "$control_typesupport" ]] || {
  echo "Project-local MoveIt overlay is incomplete" >&2
  exit 3
}
echo "Installed official MoveIt planning plugin under $OVERLAY"
